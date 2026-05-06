#define NOMINMAX
#include <windows.h>
#include <tchar.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <stdint.h>
#include <algorithm>

// -------- WMX3 / CoreMotion --------
#include "IOApi.h"
#include "WMX3Api.h"
#include "CoreMotionApi.h"

// 외부 WMX3 심볼
using namespace wmx3Api;
extern wmx3Api::WMX3Api g_wmx;
extern wmx3Api::CoreMotion g_cm;
extern Home g_home;

extern bool g_deviceOpened;
extern bool g_commStarted;

extern bool EnsureServoOn(int axis);
extern bool EnsurePosModeNoStop(int axis);
extern int  TimeMsToAcc(double vel_cnt_per_s, double t_ms);
extern bool StartAbsMoveWithProfile(int axis, long long target, double vpps, double tAcc, double tDec);
extern void StopAxis(int axis);
extern bool WriteOutputBit(int addr, int bit, bool onLogical, bool activeHigh);

// ========== EtherCAT 0x6063 읽기 ==========
extern bool ReadAxis_TxPDO_6063(int slaveId, int& outVal);
extern const int kAxisSlaveId[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

// 현재 위치(주행) 코드: Load=0x01, Unload=0x02, 그 외=0x00
extern inline unsigned char CalcPosTravelCode();
extern unsigned char CalcPosGripCode();

extern inline bool IsAxis2LimitOn();


// 외부 atEAPI
#include "atEAPI.h"

#pragma comment(lib, "User32.lib")
#pragma comment(lib, "Gdi32.lib")
#pragma comment(lib, "Comctl32.lib")

// =======================================
// 상태 관리 (기존 Demo 유지)
// =======================================
enum class TaskId : int {
    GoLeft = 0,
    GoWorkstation,
    GoRight,

    Forward,
    Backward,

    Forking,
    Unforking,

    Forking2,
    Unforking2,

    Open,
    Close,

    Up,
    Down,

    HoistUp,
    HoistDown,

    All_Demo,
    DemoLoad,
    DemoUnload,

    GoOne,
	GoTwo,
	GoThree,
	GoFour,
	GoFive,

    COUNT
};
enum class TaskState : int { Idle = 0, Running, Done, Failed, Stopped };

const TCHAR* TaskName(TaskId id) {
    switch (id) {
    case TaskId::GoLeft:        return TEXT("GoLeft");
    case TaskId::GoWorkstation: return TEXT("GoWorkstation");
    case TaskId::GoRight:       return TEXT("GoRight");
    case TaskId::Forward:       return TEXT("Forward");
    case TaskId::Backward:      return TEXT("Backward");
    case TaskId::Forking:       return TEXT("Forking");
    case TaskId::Unforking:     return TEXT("Unforking");
    case TaskId::Open:          return TEXT("Open");
    case TaskId::Close:         return TEXT("Close");
    case TaskId::Up:            return TEXT("Up");
    case TaskId::Down:          return TEXT("Down");
    case TaskId::HoistUp:       return TEXT("HoistUp");
    case TaskId::HoistDown:     return TEXT("HoistDown");
    case TaskId::All_Demo:      return TEXT("All_Demo");
    case TaskId::DemoLoad:      return TEXT("DemoLoad");
    case TaskId::DemoUnload:    return TEXT("DemoUnload");
    case TaskId::GoOne:       return TEXT("GoOne");
    case TaskId::GoTwo:       return TEXT("GoTwo");
    case TaskId::GoThree:       return TEXT("GoThree");
    case TaskId::GoFour:       return TEXT("GoFour");
    case TaskId::GoFive:       return TEXT("GoFive");
    default: return TEXT("Unknown");
    }
}

const TCHAR* TaskStateStr(TaskState s) {
    switch (s) {
    case TaskState::Idle:    return TEXT("대기");
    case TaskState::Running: return TEXT("진행중");
    case TaskState::Done:    return TEXT("완료");
    case TaskState::Failed:  return TEXT("실패");
    case TaskState::Stopped: return TEXT("중단");
    default: return TEXT("-");
    }
}
struct TaskStatus {
    std::atomic<TaskState> state{ TaskState::Idle };
    std::atomic<DWORD>     lastChangeTick{ 0 };
};
TaskStatus g_taskStatus[(int)TaskId::COUNT];

TaskState GetTaskState(TaskId id) {
    return g_taskStatus[(int)id].state.load();
}

// LED manager hook (implemented later)
void LedOnTaskStateChanged(HWND hWnd, TaskId tid, TaskState st);

// =======================================
// 공통 UI 유틸
// =======================================
static HFONT MakeUIFont(int pt, int weight = FW_NORMAL) {
    LOGFONT lf = { 0 };
    HDC sdc = GetDC(NULL);
    int logPix = GetDeviceCaps(sdc, LOGPIXELSY);
    ReleaseDC(NULL, sdc);
    lf.lfHeight = -MulDiv(pt, logPix, 72);
    lf.lfWeight = weight;
#ifdef UNICODE
    wcscpy_s(lf.lfFaceName, L"Segoe UI");
#else
    strcpy_s(lf.lfFaceName, "Segoe UI");
#endif
    return CreateFontIndirect(&lf);
}

// =======================================
// Demo 상태 표시
// =======================================
HWND g_hDemoWnd = nullptr;
HWND g_hStatusStatics[(int)TaskId::COUNT] = { 0 };
void SetTaskState(TaskId id, TaskState st) {
    int idx = (int)id;
    g_taskStatus[idx].state.store(st, std::memory_order_relaxed);
    g_taskStatus[idx].lastChangeTick.store(GetTickCount(), std::memory_order_relaxed);
    if (g_hDemoWnd && g_hStatusStatics[idx]) {
        std::wstring text = std::wstring(TaskName(id)) + L": " + TaskStateStr(st);
        SetWindowTextW(g_hStatusStatics[idx], text.c_str());
    }

    // If this task owns the common action LED, finalize/cancel blink based on state.
    LedOnTaskStateChanged(g_hDemoWnd, id, st);

    if ((id == TaskId::Forking || id == TaskId::Unforking) && st == TaskState::Done) {
        WriteOutputBit(38, 2, false, true);
        WriteOutputBit(38, 1, false, true);
    }

}
void ResetAllTaskStates() {
    for (int i = 0; i < (int)TaskId::COUNT; ++i)
        SetTaskState((TaskId)i, TaskState::Idle);
}

// =======================================
// Barcode follower (NO ProfileStop, smooth decel + aDecEma scale + jerk-limited S-curve)
// =======================================
struct BarcodeParams {
    // 주행부 축 (요구사항: 7번)
    int axis = 7;

    // 목표 바코드(0x6063, abs)
    long long targetBarcodeAbs = 0;

    // Main profile
    double mainVel = 10000.0; // pps
    double mainAcc = 1000.0;  // ms
    double mainDec = 1000.0;  // ms

    // Fine(Correction) profile
    double corrVel = 1000.0;  // pps
    double corrAcc = 300.0;   // ms
    double corrDec = 300.0;   // ms

    // (표시용/호환용) 기존 deadband 필드 유지 (main 알고리즘은 arriveCnt=±2cnt 사용)
    int deadband = 1;

    // Conversion
    double gear = 4.4248;
    double wheelDia = 115.0;    // mm
    double motorCpr = 10000.0;  // pulses per motor rev
    double bcMmPerCnt = 1.07;   // mm per barcode count
};

class BarcodeFollower {
public:
    void Start(const BarcodeParams& p, TaskId taskToReport) {
        Stop();
        params_ = p;
        reportTask_ = taskToReport;
        running_ = true;
        worker_ = std::thread(&BarcodeFollower::ThreadProc, this);
        worker_.detach();
    }
    void Stop() {
        running_ = false;
        if (params_.axis >= 0 && params_.axis < 9) {
            StopAxis(params_.axis);
        }
    }
    bool IsRunning() const { return running_.load(); }

private:
    // ===== main의 Hybrid Barcode state를 Demo-thread 형태로 이식 =====
    static constexpr int  kMaxAxes_ = 12;
    static constexpr int  kBarcodeAxis_ = 9;     // ✅ 바코드 0x6063 읽는 축
    static constexpr DWORD POLL_MS_ = 30;
    static constexpr double dtSec_ = 0.03;

    std::atomic<bool> running_{ false };
    BarcodeParams params_{};
    TaskId reportTask_ = TaskId::GoWorkstation;

    struct HbcSnapshot { double gear, wheelDia, motorCpr, bcMmPerCnt; };

    static inline int sgnll(long long v) { return (v > 0) - (v < 0); }

    // ✅ main 코드의 (주의: gear 반영 방식은 원본 그대로)
    static inline double pulsesPerMm(const HbcSnapshot& s) {
        // NOTE: main 파일의 구현을 그대로 가져옴(요청사항: 알고리즘 그대로)
        // pulses per wheel rev = motorCpr * gear 라는 주석은 있으나,
        // 실제 식은 motorCpr / (pi * wheelDia) 로 되어 있음.
        return s.motorCpr / (3.14159265358979323846 * s.wheelDia);
    }
    static inline double bcToMm(const HbcSnapshot& s, long long bc) { return (double)bc * s.bcMmPerCnt; }
    static inline long long mmToPulses(const HbcSnapshot& s, double mm) {
        double pulses = mm * pulsesPerMm(s);
        return (long long)std::llround(pulses);
    }
    static inline long long bcToPulses(const HbcSnapshot& s, long long bc) {
        return mmToPulses(s, bcToMm(s, bc));
    }

    // motor rpm -> pps (main과 동일)
    static inline double RpmToPps(double motorRpm, double motorCpr) {
        return std::fabs(motorRpm) * motorCpr / 60.0;
    }

    static inline double DecelPps2_Model(double profileVelPps, double decMs, double safetyFactor) {
        double t = std::max(1.0, decMs) / 1000.0;
        double a = profileVelPps / t;
        safetyFactor = std::clamp(safetyFactor, 0.05, 1.0);
        a *= safetyFactor;
        return std::max(1.0, a);
    }
    static inline double StopDistPulses(double vPps, double aDecPps2) {
        return (vPps * vPps) / (2.0 * std::max(1.0, aDecPps2));
    }
    static inline double VelLimitFromDist(double distPulses, double aDecPps2) {
        if (distPulses <= 0) return 0.0;
        return std::sqrt(2.0 * std::max(1.0, aDecPps2) * distPulses);
    }

    bool Read6063Now(int& out) {
        if (kBarcodeAxis_ < 0 || kBarcodeAxis_ >= kMaxAxes_) return false;
        return ReadAxis_TxPDO_6063(kAxisSlaveId[kBarcodeAxis_], out);
    }

    void SendMoveThrottled(
        int ax,
        long long absTarget,
        double vel_pps,
        double acc_ms,
        double dec_ms,
        double velChangeRatio,
        long long minTargetDeltaPulses,
        DWORD  minPeriodMs
    ) {
        if (!running_) return;

        ULONGLONG now = GetTickCount64();
        if (now < stopCooldownUntil_) return;
        if (now < fineStartCooldownUntil_) return;

        bool periodOk = (now - lastCmdTick_) >= (ULONGLONG)minPeriodMs;

        double lastV = lastCmdVel_;
        bool velChanged = (lastV <= 1.0) ? true : ((std::fabs(vel_pps - lastV) / lastV) >= velChangeRatio);

        long long lastT = lastCmdTarget_;
        bool targetChanged = (std::llabs(absTarget - lastT) >= std::max(1LL, minTargetDeltaPulses));

        if (periodOk && (velChanged || targetChanged)) {
            StartAbsMoveWithProfile(ax, absTarget, vel_pps, acc_ms, dec_ms);
            lastCmdVel_ = vel_pps;
            lastCmdTarget_ = absTarget;
            lastCmdTick_ = now;
        }
    }

    // ---- runtime state (ProfileStop/COARSE_BRAKE 제거) ----
    enum Phase { IDLE = 0, COARSE = 1, FINE = 2, DONE = 3 };
    Phase phase_ = IDLE;

    long long targetBarcodeAbs_ = 0;
    long long coarseTargetBarcodeAbs_ = 0;

    // coarse → 목표-preStopCnt_ (preStop 구간에서 "속도만" 0에 수렴시킴)
    int preStopCnt_ = 10;                // 원래 10 유지 (필요시 20 추천)
    int coarseArriveCnt_ = 1;
    int coarseStableTicksNeed_ = 5;
    int coarseStableTicks_ = 0;

    // fine → ±2cnt 안정화
    int arriveCnt_ = 2;
    int arriveStableTicksNeed_ = 10;
    int arriveStableTicks_ = 0;

    // overshoot
    int lastErrSign_ = 0;
    int signFlipTicks_ = 0;
    bool forbidReverse_ = true;

    // throttle tracking
    double lastCmdVel_ = 0.0;
    long long lastCmdTarget_ = 0;
    ULONGLONG lastCmdTick_ = 0;

    // creep/fine limit
    int creepCnt_ = 50;
    double creepVel_ = 200.0;
    int fineStepCntMax_ = 1;

    // decel estimator
    double vPrevPps_ = 0.0;
    double aDecEma_ = 0.0;
    bool havePrevV_ = false;

    // cooldown
    ULONGLONG stopCooldownUntil_ = 0;
    ULONGLONG fineStartCooldownUntil_ = 0;

    // ===== Added: jerk-limit parameter =====
    double maxJerkPps2_ = 25000.0;   // pps/s (tick당 dv 제한 = maxJerkPps2_ * dtSec_)

    void ThreadProc() {
        const int ax = params_.axis;

        if (!g_commStarted || ax < 0 || ax >= kMaxAxes_) {
            SetTaskState(reportTask_, TaskState::Failed);
            running_ = false;
            return;
        }
        if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) {
            SetTaskState(reportTask_, TaskState::Failed);
            running_ = false;
            return;
        }

        // reset
        phase_ = IDLE;
        arriveStableTicks_ = 0;
        coarseStableTicks_ = 0;
        lastErrSign_ = 0;
        signFlipTicks_ = 0;
        stopCooldownUntil_ = 0;
        fineStartCooldownUntil_ = 0;
        havePrevV_ = false;
        vPrevPps_ = 0.0;
        aDecEma_ = 0.0;

        // command history reset
        lastCmdVel_ = 0.0;
        lastCmdTarget_ = 0;
        lastCmdTick_ = GetTickCount64();

        SetTaskState(reportTask_, TaskState::Running);

        int now6063 = 0;
        if (!Read6063Now(now6063)) {
            SetTaskState(reportTask_, TaskState::Failed);
            running_ = false;
            return;
        }

        targetBarcodeAbs_ = params_.targetBarcodeAbs;

        long long finalErr = targetBarcodeAbs_ - (long long)now6063;
        int dir = sgnll(finalErr);
        if (dir == 0) {
            phase_ = FINE;
            lastErrSign_ = 0;
        }
        else {
            coarseTargetBarcodeAbs_ = targetBarcodeAbs_ - (long long)dir * (long long)preStopCnt_;
            lastErrSign_ = dir;

            // initial move to coarse target
            CoreMotionStatus st{};
            g_cm.GetStatus(&st);
            long long curPos = (long long)st.axesStatus[ax].actualPos;

            HbcSnapshot snap{ params_.gear, params_.wheelDia, params_.motorCpr, params_.bcMmPerCnt };
            long long coarseRel = (long long)coarseTargetBarcodeAbs_ - (long long)now6063;
            long long coarsePulses = bcToPulses(snap, coarseRel);
            long long absTarget = curPos + coarsePulses;

            StartAbsMoveWithProfile(ax, absTarget, params_.mainVel, params_.mainAcc, params_.mainDec);
            lastCmdVel_ = params_.mainVel;
            lastCmdTarget_ = absTarget;
            lastCmdTick_ = GetTickCount64();

            phase_ = COARSE;
        }

        const int overshootConfirmTicks = 3;
        HbcSnapshot snap{ params_.gear, params_.wheelDia, params_.motorCpr, params_.bcMmPerCnt };
        long long pulsesPerCnt = std::llabs(bcToPulses(snap, 1));
        if (pulsesPerCnt < 1) pulsesPerCnt = 1;

        bool completed = false;

        while (running_.load()) {
            if (!Read6063Now(now6063)) { completed = false; break; }

            // phase에 따라 활성 목표(6063 abs)
            long long activeTargetAbs =
                (phase_ == COARSE) ? coarseTargetBarcodeAbs_ : targetBarcodeAbs_;

            long long bcErr = activeTargetAbs - (long long)now6063;
            long long bcErrAbs = llabs(bcErr);

            // status (속도/위치)
            CoreMotionStatus st{};
            g_cm.GetStatus(&st);

            double motorRpm = (double)st.axesStatus[ax].actualVelocity;
            double vCurPps = RpmToPps(motorRpm, params_.motorCpr);

            // measured decel EMA
            if (!havePrevV_) {
                havePrevV_ = true;
                vPrevPps_ = vCurPps;
            }
            else {
                double dv = vPrevPps_ - vCurPps; // + when decelerating
                double aInst = dv / dtSec_;
                vPrevPps_ = vCurPps;

                if (aInst > 50.0) {
                    const double alpha = 0.15;
                    if (aDecEma_ <= 0.0) aDecEma_ = aInst;
                    else aDecEma_ = (1.0 - alpha) * aDecEma_ + alpha * aInst;
                }
            }

            // overshoot detect
            int sgn = sgnll(bcErr);
            if (forbidReverse_) {
                if (lastErrSign_ != 0 && sgn != 0 && sgn != lastErrSign_) signFlipTicks_++;
                else signFlipTicks_ = 0;

                if (signFlipTicks_ >= overshootConfirmTicks) {
                    StopAxis(ax);
                    completed = false;
                    break;
                }
            }
            else {
                signFlipTicks_ = 0;
            }
            if (sgn != 0) lastErrSign_ = sgn;

            // ==========================
            // PHASE: COARSE (NO ProfileStop, smooth decel + aDecEma scale + jerk limit)
            // ==========================
            if (phase_ == COARSE) {
                // coarse target 도달 안정화 -> 바로 FINE 전환
                if (bcErrAbs <= coarseArriveCnt_) {
                    if (++coarseStableTicks_ >= coarseStableTicksNeed_) {
                        phase_ = FINE;
                        arriveStableTicks_ = 0;

                        long long fe = targetBarcodeAbs_ - (long long)now6063;
                        lastErrSign_ = sgnll(fe);
                        signFlipTicks_ = 0;

                        // 전환 직후 명령 충돌 방지 (필요시 유지)
                        fineStartCooldownUntil_ = GetTickCount64() + 120;

                        // command 갱신 유도
                        lastCmdVel_ = 0;
                        lastCmdTarget_ = 0;

                        coarseStableTicks_ = 0;
                    }
                    ::Sleep(POLL_MS_);
                    continue;
                }
                coarseStableTicks_ = 0;

                long long remainingPulses = bcToPulses(snap, bcErr);
                long long distAbs = llabs(remainingPulses);

                auto safe_sqrt_scale = [](double base, double refVel, double vel, double lo, double hi) {
                    double v = std::max(1.0, vel);
                    double scale = std::sqrt(std::max(0.2, refVel / v));
                    return std::clamp(base * scale, lo, hi);
                    };

                double mainDecSafety = safe_sqrt_scale(0.28, 5000.0, params_.mainVel, 0.15, 0.35);
                double aMainModel = DecelPps2_Model(params_.mainVel, params_.mainDec, mainDecSafety);

                double aMeas = (aDecEma_ > 0.0) ? aDecEma_ : 0.0;
                double aMainDec = (aMeas > 0.0) ? std::min(aMeas, aMainModel * 1.2) : aMainModel;

                double stopDistNow = StopDistPulses(vCurPps, aMainDec);

                long long marginPulses =
                    (long long)(pulsesPerCnt * 6) +        // coarse는 6cnt
                    (long long)(vCurPps * 0.15) +          // 150ms 선행
                    (long long)(stopDistNow * 0.10) +      // 10% 버퍼
                    300;

                if (marginPulses < pulsesPerCnt * 6) marginPulses = pulsesPerCnt * 6;

                double distForPlan = (double)distAbs - (double)marginPulses;
                if (distForPlan < 0) distForPlan = 0;

                // 기본 stop-distance 기반 속도 상한
                double vEnvMain = std::min(params_.mainVel, VelLimitFromDist(distForPlan, aMainDec));

                // ---------- Smooth preStop decel (ratio^1.5) ----------
                // COARSE 목표(coarseTarget) 근처에서 속도만 부드럽게 0쪽으로 수렴
                if (bcErrAbs <= preStopCnt_) {
                    // ratio: 멀면 1, 가까울수록 0
                    double ratio = std::clamp((double)bcErrAbs / (double)std::max(1, preStopCnt_), 0.0, 1.0);

                    // 더 부드러운 곡선: ratio^1.5
                    double smooth = std::pow(ratio, 1.5);

                    // ramp (저속 바닥값)
                    const double vMin = 40.0;
                    const double vMax = std::max(200.0, creepVel_);

                    double vRamp = vMin + (vMax - vMin) * smooth;

                    // aDecEma 기반 자동 스케일 (감속 잘 안되면 더 보수적으로)
                    if (aDecEma_ > 0.0) {
                        const double aRef = 18000.0; // 튜닝 포인트
                        double decScale = std::clamp(aDecEma_ / aRef, 0.5, 1.0);
                        vRamp *= decScale;
                    }

                    vEnvMain = std::min(vEnvMain, vRamp);
                }
                else if (bcErrAbs <= (preStopCnt_ + 10)) {
                    // 기존 close-in 제한 유지(완만)
                    vEnvMain = std::min(vEnvMain, std::max(200.0, creepVel_));
                }

                // ---------- jerk 제한 (S-curve 효과) ----------
                // tick당 속도 변화량 제한으로 "툭" 줄어드는 것을 방지
                {
                    double maxDv = std::max(1.0, maxJerkPps2_) * dtSec_;
                    double dvCmd = vEnvMain - lastCmdVel_;
                    if (std::fabs(dvCmd) > maxDv) {
                        vEnvMain = lastCmdVel_ + std::copysign(maxDv, dvCmd);
                    }
                }

                if (vEnvMain < 30.0 && distAbs > 0) vEnvMain = 30.0;

                long long curPos = (long long)st.axesStatus[ax].actualPos;
                long long absTarget = curPos + remainingPulses;

                DWORD period = (bcErrAbs <= 80) ? 40 : 90;
                double ratio = (bcErrAbs <= 80) ? 0.05 : 0.12;

                SendMoveThrottled(ax, absTarget, vEnvMain, params_.mainAcc, params_.mainDec,
                    ratio, pulsesPerCnt, period);

                ::Sleep(POLL_MS_);
                continue;
            }

            // ==========================
            // PHASE: FINE
            // ==========================
            if (phase_ == FINE) {
                long long finalErr2 = targetBarcodeAbs_ - (long long)now6063;
                long long finalErrAbs = llabs(finalErr2);

                if (finalErrAbs <= arriveCnt_) {
                    if (++arriveStableTicks_ >= arriveStableTicksNeed_) {
                        StopAxis(ax);
                        completed = true;
                        break;
                    }
                    ::Sleep(POLL_MS_);
                    continue;
                }
                arriveStableTicks_ = 0;

                long long remainingPulses = bcToPulses(snap, finalErr2);

                long long stepMaxPulses = llabs(bcToPulses(snap, (long long)fineStepCntMax_));
                if (stepMaxPulses < 1) stepMaxPulses = 1;

                long long stepPulses = remainingPulses;
                if (stepPulses > stepMaxPulses) stepPulses = stepMaxPulses;
                if (stepPulses < -stepMaxPulses) stepPulses = -stepMaxPulses;

                if (finalErrAbs <= 10) {
                    long long oneCntPulse = llabs(bcToPulses(snap, 1));
                    stepPulses = std::clamp(stepPulses, -std::max(1LL, oneCntPulse), std::max(1LL, oneCntPulse));
                }

                double vPlan = std::min(params_.corrVel, creepVel_);
                if (finalErrAbs <= creepCnt_)
                    vPlan = std::min(vPlan, creepVel_);

                if (finalErrAbs <= 6) vPlan = std::min(vPlan, 200.0);
                if (vPlan < 20.0) vPlan = 20.0;

                long long curPos = (long long)st.axesStatus[ax].actualPos;
                long long absTarget = curPos + stepPulses;

                SendMoveThrottled(ax, absTarget, vPlan, params_.corrAcc, params_.corrDec,
                    0.01, 1, 80);

                ::Sleep(POLL_MS_);
                continue;
            }

            // IDLE/DONE
            ::Sleep(POLL_MS_);
        }

        if (!running_.load())
            SetTaskState(reportTask_, TaskState::Stopped);
        else
            SetTaskState(reportTask_, completed ? TaskState::Done : TaskState::Failed);

        running_ = false;
        phase_ = IDLE;
        arriveStableTicks_ = 0;
        coarseStableTicks_ = 0;
        signFlipTicks_ = 0;
        stopCooldownUntil_ = GetTickCount64() + 300;
        fineStartCooldownUntil_ = stopCooldownUntil_;
    }

    std::thread worker_;
};

static BarcodeFollower g_bcRunner;


// =======================================
// Move monitors (기존 유지)
// =======================================
struct MoveMonitorArgs {
    int axis = -1;
    long long target = 0;
    TaskId task;
    double posEps = 10.0;
    double velEps = 2.0;
    DWORD timeoutMs = 15000;
    bool treatStoppedAsDone = false;
};
static void StartMoveAndMonitor(const MoveMonitorArgs& m, double vpps, double accMs, double decMs) {
    if (!g_commStarted) { SetTaskState(m.task, TaskState::Failed); return; }
    if (m.axis < 0 || m.axis >= 4) { SetTaskState(m.task, TaskState::Failed); return; }
    if (!EnsureServoOn(m.axis) || !EnsurePosModeNoStop(m.axis)) {
        SetTaskState(m.task, TaskState::Failed);
        return;
    }

    SetTaskState(m.task, TaskState::Running);

    if (!StartAbsMoveWithProfile(m.axis, m.target, vpps, accMs, decMs)) {
        SetTaskState(m.task, TaskState::Failed);
        return;
    }

    std::thread([m]() {
        DWORD start = GetTickCount();
        CoreMotionStatus st{};
        TaskState result = TaskState::Failed;

        while (GetTickCount() - start < m.timeoutMs) {
            g_cm.GetStatus(&st);
            long long posErr = (long long)st.axesStatus[m.axis].actualPos - m.target;
            double v = std::fabs(st.axesStatus[m.axis].actualVelocity);
            if (std::llabs(posErr) <= (long long)m.posEps && v <= m.velEps) {
                result = TaskState::Done;
                break;
            }
            if (m.treatStoppedAsDone && v <= m.velEps) {
                result = TaskState::Done;
                break;
            }
            ::Sleep(30);
        }
        SetTaskState(m.task, result);
        }).detach();
}

struct ApproachProfile {
    double vpps = 500.0;
    double accMs = 100.0;
    double decMs = 100.0;
};
// Axis2HomeSoftDecelTo500() 스타일을 일반화한 Approach 버전
static void StartMoveWithApproach(int axis, long long target, TaskId task,
    double mainVpps, double mainAccMs, double mainDecMs,
    double posEps, double velEps, DWORD timeoutMs,
    double approachEps, const ApproachProfile& ap)
{
    if (!g_commStarted) {
        SetTaskState(task, TaskState::Failed);
        return;
    }
    if (axis < 0 || axis >= 9) {
        SetTaskState(task, TaskState::Failed);
        return;
    }
    if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) {
        SetTaskState(task, TaskState::Failed);
        return;
    }

    // 1) 먼저 target까지 "빠른" 프로파일로 이동 시작 (5000 등)
    Motion::PosCommand fast{};
    fast.axis = axis;
    fast.profile.type = ProfileType::SCurve;
    fast.profile.velocity = (int)std::lround(mainVpps);
    fast.profile.acc = TimeMsToAcc(fast.profile.velocity, mainAccMs);
    fast.profile.dec = TimeMsToAcc(fast.profile.velocity, mainDecMs);
    fast.target = target;

    long err = g_cm.motion->StartPos(&fast);
    if (err != ErrorCode::None) {
        SetTaskState(task, TaskState::Failed);
        return;
    }

    SetTaskState(task, TaskState::Running);

    // 2) 모니터링 스레드: 3단계 상태
    //   - phase 0 : 빠른 구간 (그냥 감시만)
    //   - phase 1 : Axis2HomeSoftDecelTo500 패턴으로 "급 감속용 소타겟" 이동
    //   - phase 2 : 저속(1000)으로 최종 target 접근
    std::thread([=]() {
        DWORD startTick = GetTickCount();
        CoreMotionStatus st{};
        TaskState result = TaskState::Failed;

        enum Phase { FastRun = 0, SoftDecel, FinalApproach };
        Phase phase = FastRun;

        long long softTarget = 0;   // 급감속용 중간 타겟
        bool softTargetSet = false;

        while (true) {
            // ★★★ 1) 전역 Stop(DoStopAll 등) 감지: task가 Running이 아니면 즉시 종료
            TaskState cur = g_taskStatus[(int)task].state.load(std::memory_order_relaxed);
            if ((cur == TaskState::Stopped || cur == TaskState::Failed) && phase != FinalApproach) {
                result = cur;   // 보통 Stopped
                break;
            }

            if (GetTickCount() - startTick > timeoutMs) {
                result = TaskState::Failed;
                break;
            }

            g_cm.GetStatus(&st);
            const auto& axst = st.axesStatus[axis];

            double actPos = (double)axst.actualPos;
            double actVel = std::fabs(axst.actualVelocity);
            double remErr = std::fabs((double)target - actPos);

            if (phase == FastRun) {
                // 아직 빠른 구간: 남은 거리 remErr 를 체크
                // remErr <= approachEps (예: 3500) 이 되면
                // Axis2HomeSoftDecelTo500 스타일로 "현재 위치 기준 소타겟"으로 급감속
                if (remErr <= approachEps) {
                    // 현재 위치, 진행 방향 계산
                    long long cur = (long long)axst.actualPos;
                    long long dist = target - cur;
                    if (dist == 0) {
                        // 이미 타겟 가까우면 바로 종료 처리
                        if (actVel < velEps && remErr <= posEps) {
                            result = TaskState::Done;
                        }
                        else {
                            result = TaskState::Stopped;
                        }
                        break;
                    }

                    int dir = (dist > 0) ? +1 : -1;

                    // Axis2HomeSoftDecelTo500 처럼 "현재 위치에서 작은 step" 만큼
                    // 앞쪽으로 소타겟 잡기. (여기서는 remErr의 절반 정도, 최대 4000 제한)
                    long long maxStep = 4000;  // 필요하면 조정
                    long long step = (long long)remErr / 2;
                    if (step > maxStep) step = maxStep;
                    if (step < 1000)   step = 1000;  // 너무 짧으면 감속 효과가 약하니 최소값

                    step *= dir;
                    softTarget = cur + step;

                    // 혹시 소타겟이 target 을 넘어가지 않도록 보정
                    if (dir > 0 && softTarget > target) softTarget = target;
                    if (dir < 0 && softTarget < target) softTarget = target;

                    // === 1단계 급감속: Axis2HomeSoftDecelTo500 과 같은 방식 ===
                    Motion::PosCommand pc{};
                    pc.axis = axis;
                    pc.target = softTarget;
                    pc.profile.type = ProfileType::SCurve;
                    pc.profile.velocity = (int)std::lround(ap.vpps); // 최종에서 쓸 저속과 비슷하게
                    pc.profile.acc = TimeMsToAcc(pc.profile.velocity, ap.accMs);
                    pc.profile.dec = TimeMsToAcc(pc.profile.velocity, ap.decMs); // decMs 작은 값으로 "확" 감속

                    g_cm.motion->StartPos(&pc);

                    softTargetSet = true;
                    phase = SoftDecel;
                }
            }
            else if (phase == SoftDecel && softTargetSet) {
                // 급감속 소타겟(softTarget)으로 가는 중
                // 속도가 충분히 줄고, softTarget 근처에 오면 → 최종 타겟으로 저속 접근 시작
                double eSoft = 0.0;
                if (softTargetSet)
                    eSoft = std::fabs((double)softTarget - actPos);

                // "충분히 감속되었다" 판단 기준:
                //  - 속도 actVel 이 ap.vpps 근처 or 매우 낮아졌을 때
                //  - 소타겟 근처(eSoft <= posEps * 2 정도)
                if (softTargetSet &&
                    actVel <= (ap.vpps + 50.0) &&     // 속도 기준 (적당히 여유)
                    eSoft <= (posEps * 3.0))
                {
                    // === 2단계: 이제 저속(1000)으로 최종 target까지 이동 ===
                    Motion::PosCommand pc2{};
                    pc2.axis = axis;
                    pc2.target = target;
                    pc2.profile.type = ProfileType::SCurve;
                    pc2.profile.velocity = (int)std::lround(ap.vpps);
                    pc2.profile.acc = TimeMsToAcc(pc2.profile.velocity, ap.accMs);
                    pc2.profile.dec = TimeMsToAcc(pc2.profile.velocity, ap.decMs);

                    g_cm.motion->StartPos(&pc2);
                    phase = FinalApproach;
                }
            }
            else if (phase == FinalApproach) {
                // 최종 저속 접근 단계: 평소처럼 Done/Stopped 판정
                if (actVel < velEps) {
                    result = TaskState::Done;
                    break;
                }                    
            }

            ::Sleep(10);
        }

        // If another part of the system (e.g., limit-sensor stop) already marked this task as Done,
        // don't overwrite it with Stopped/Failed.
        if (GetTaskState(task) != TaskState::Done) {
            SetTaskState(task, result);
        }
        }).detach();
}

// =======================================
// Axis2 Limit/Home 센서 (기존 유지 + LIMIT 2개로 확장)
// =======================================
static HWND g_hAx2LimitStatic = nullptr;
static HWND g_hAx2HomeStatic = nullptr;

static std::wstring g_ax5UiExtra = L""; // AX5 extra UI text (side/check)
// Added: AX0~AX5 limit sensor display (left panel)
static HWND g_hAxLimitStatics[6] = { nullptr,nullptr,nullptr,nullptr,nullptr,nullptr }; // AX0..AX5 (AX4 reserved)

// --- LIMIT 센서 2개 (요청 반영) ---
static const int AX4_LIMIT1_ADDR = 67;
static const int AX4_LIMIT1_BIT = 0;
static const int AX4_LIMIT2_ADDR = 67;
static const int AX4_LIMIT2_BIT = 1;
static const bool AX4_LIMIT1_ACTIVE_HIGH = true;
static const bool AX4_LIMIT2_ACTIVE_HIGH = true;

//// --- HOME 센서 (기존 유지) ---
static const int  AX2_HOME_ADDR = 8;
static const int  AX2_HOME_BIT = 2;
static const bool AX2_HOME_ACTIVE_HIGH = true;

// 타이밍 (기존 유지)
static const DWORD AX4_SENSOR_POLL_MS = 5;
static const DWORD AX4_SENSOR_DEBOUNCE_MS = 5;
static const DWORD AX4_SENSOR_ENABLE_DELAY_MS = 1500;

// ======================================================
// Added limit sensors (AX0/AX1/AX2/AX3/AX5) - 2026-01-15
// ======================================================
static const int  AX0_LIMIT_ADDR = 65;
static const int  AX0_LIMIT_BIT = 7;
static const bool AX0_LIMIT_ACTIVE_HIGH = true;

// AX0 limit location: set true if AX0 limit is at +end (positive direction),
// set false if AX0 limit is at -end. This allows jogging away from an active limit.
static const bool AX0_LIMIT_AT_POSITIVE_END = true;

static const int  AX1_LIMIT1_ADDR = 65;
static const int  AX1_LIMIT1_BIT = 5;
static const bool AX1_LIMIT1_ACTIVE_HIGH = true;
static const int  AX1_LIMIT2_ADDR = 65;
static const int  AX1_LIMIT2_BIT = 6;
static const bool AX1_LIMIT2_ACTIVE_HIGH = true;

static const int  AX2_UPLIMIT_ADDR = 66;
static const int  AX2_UPLIMIT_BIT = 7;
static const bool AX2_UPLIMIT_ACTIVE_HIGH = true;

static const int  AX3_UPLIMIT_ADDR = 66;
static const int  AX3_UPLIMIT_BIT = 5;
static const bool AX3_UPLIMIT_ACTIVE_HIGH = true;

static const int  AX2_DOWNLIMIT_ADDR = 66;
static const int  AX2_DOWNLIMIT_BIT = 6;
static const bool AX2_DOWNLIMIT_ACTIVE_HIGH = true;

static const int  AX3_DOWNLIMIT_ADDR = 66;
static const int  AX3_DOWNLIMIT_BIT = 4;
static const bool AX3_DOWNLIMIT_ACTIVE_HIGH = true;

static const int  AX5_LIMIT1_ADDR = 66;
static const int  AX5_LIMIT1_BIT = 0;
static const bool AX5_LIMIT1_ACTIVE_HIGH = true;

static const int  AX5_LIMIT2_ADDR = 66;
static const int  AX5_LIMIT2_BIT = 1;
static const bool AX5_LIMIT2_ACTIVE_HIGH = true;

static const int  AX5_LIMIT3_ADDR = 66;
static const int  AX5_LIMIT3_BIT = 2;
static const bool AX5_LIMIT3_ACTIVE_HIGH = true;

static const int  AX5_LIMIT4_ADDR = 66;
static const int  AX5_LIMIT4_BIT = 3;
static const bool AX5_LIMIT4_ACTIVE_HIGH = true;

// Poll period for added limit logic
static const DWORD AX_LIMIT_POLL_MS = 5;

// AX0: if limit stays ON for >= 1s -> home (actpos->0)
static const DWORD AX0_LIMIT_HOME_HOLD_MS = 2000;

// AX5: decel profile (tune as needed)
static const double AX5_DECEL_VEL_PPS = 500.0;
static const double AX5_DECEL_ACC_MS = 120.0;
static const double AX5_DECEL_DEC_MS = 120.0;
static const long long AX5_DECEL_LOOKAHEAD_PULSE = 250000; // "far enough" target for slow approach

// 상태 플래그들 (기존 유지)
static std::atomic<bool> g_ax2LimitOn{ false };
static std::atomic<bool> g_ax2HomeOn{ false };
static std::atomic<bool> g_ax2LimitLatched{ false };
static std::atomic<bool> g_ax2LimitBlocking{ false };
static std::atomic<bool> g_ax2StopIssuedOnLimit{ false };
static std::atomic<bool> g_ax2HomingStarted{ false };
static std::atomic<bool> g_ax2HomeDebounceOn{ false };
static std::atomic<DWORD> g_ax2HomeLastTick{ 0 };
static std::atomic<bool> g_ax2HomeRampIssued{ false };
static std::atomic<DWORD> g_ax2LimitIdleTime{ 0 };
static std::atomic<bool> g_ax2ServoReady{ false };
static std::atomic<DWORD> g_ax2ServoOnTime{ 0 };

// 판정 파라미터 (기존 유지)
static const double vel_idle_threshold = 1.0;
static const long long inpos_tol_counts = 10;

static bool IsAxis2ServoOn()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    return st.axesStatus[4].servoOn;
}

static bool ReadInputBitRaw(int addr, int bit, bool activeHigh) {
    if (!g_commStarted) return false;
    Io io(&g_wmx);
    unsigned char v = 0;
    long e = io.GetInBitEx(addr, bit, &v);
    if (e != ErrorCode::None) return false;
    bool onRaw = (v != 0);
    return activeHigh ? onRaw : !onRaw;
}
static bool Axis2IsIdle() {
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    int av = (int)std::lround(st.axesStatus[4].actualVelocity);
    long long perr = (long long)st.axesStatus[4].posCmd - (long long)st.axesStatus[4].actualPos;
    return (std::abs(av) <= vel_idle_threshold) && (std::llabs(perr) <= inpos_tol_counts);
}
static int Axis2CurrentDir() {
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double vcmd = st.axesStatus[4].velocityCmd;
    if (vcmd > vel_idle_threshold) return +1;
    if (vcmd < -vel_idle_threshold) return -1;
    return 0;
}
static void Axis2HomeSoftDecelTo500() {
    if (!g_commStarted) return;
    if (Axis2IsIdle()) return;
    if (!EnsureServoOn(4) || !EnsurePosModeNoStop(4)) return;
    CoreMotionStatus st{}; g_cm.GetStatus(&st);
    long long cur = (long long)st.axesStatus[4].actualPos;
    int dir = Axis2CurrentDir(); if (dir == 0) dir = +1;
    long long smallStep = 5000 * dir;
    long long softTarget = cur + smallStep;
    double newVel = 500.0;
    double accMs = 80.0;
    double decMs = 10.0;
    Motion::PosCommand pc{};
    pc.axis = 4; pc.target = softTarget;
    pc.profile.type = ProfileType::SCurve;
    pc.profile.velocity = (int)std::lround(newVel);
    pc.profile.acc = TimeMsToAcc(pc.profile.velocity, accMs);
    pc.profile.dec = TimeMsToAcc(pc.profile.velocity, decMs);
    g_cm.motion->StartPos(&pc);
    g_ax2HomeRampIssued = true;
}
void Axis2HandleLimitOnceAndHome() {
    if (!g_commStarted) return;
    if (!g_ax2LimitLatched.load()) return;
    if (!g_ax2StopIssuedOnLimit.exchange(true)) {
        StopAxis(4);
    }
    if (!Axis2IsIdle()) { g_ax2LimitIdleTime = 0; return; }
    if (g_ax2LimitIdleTime.load() == 0) { g_ax2LimitIdleTime = GetTickCount(); return; }
    DWORD elapsed = GetTickCount() - g_ax2LimitIdleTime.load();
    if (elapsed < 2000) return;
    if (!g_ax2HomingStarted.load()) {
        if (EnsureServoOn(4) && EnsurePosModeNoStop(4)) {
            g_home.StartHome(4);
            g_ax2HomingStarted = true;
        }
    }
}
static void UpdateAx2SensorLabels(bool ready, bool limitOn, bool homeOn) {
    if (!g_hDemoWnd) return;
    if (g_hAx2LimitStatic) {
        if (!ready) SetWindowText(g_hAx2LimitStatic, TEXT("Axis2 LIMIT: WAIT"));
        else        SetWindowText(g_hAx2LimitStatic, limitOn ? TEXT("Axis2 LIMIT: ON") : TEXT("Axis2 LIMIT: OFF"));
    }
    if (g_hAx2HomeStatic) {
        if (!ready) SetWindowText(g_hAx2HomeStatic, TEXT("Axis2 HOME: WAIT"));
        else        SetWindowText(g_hAx2HomeStatic, homeOn ? TEXT("Axis2 HOME: ON") : TEXT("Axis2 HOME: OFF"));
    }
}
void Axis2SensorInit()
{
    g_ax2LimitOn = false; g_ax2HomeOn = false;
    g_ax2LimitLatched = false; g_ax2LimitBlocking = false; g_ax2StopIssuedOnLimit = false; g_ax2HomingStarted = false;
    g_ax2HomeDebounceOn = false; g_ax2HomeLastTick = GetTickCount(); g_ax2HomeRampIssued = false;
    g_ax2LimitIdleTime = 0;
    g_ax2ServoReady = false; g_ax2ServoOnTime = 0;
}
void Axis2SensorTimerProc(HWND)
{
    if (!g_commStarted) { UpdateAx2SensorLabels(false, false, false); return; }
    if (!g_ax2ServoReady.load()) {
        if (IsAxis2ServoOn()) {
            if (g_ax2ServoOnTime.load() == 0) { g_ax2ServoOnTime = GetTickCount(); }
            else {
                DWORD diff = GetTickCount() - g_ax2ServoOnTime.load();
                if (diff >= AX4_SENSOR_ENABLE_DELAY_MS) g_ax2ServoReady = true;
            }
        }
    }
    // -------------------------------------------------------
    // LIMIT: 2개 중 하나라도 ON이면 LIMIT ON 처리 (요청 반영)
    // -------------------------------------------------------
    bool limit1Raw = ReadInputBitRaw(AX4_LIMIT1_ADDR, AX4_LIMIT1_BIT, AX4_LIMIT1_ACTIVE_HIGH);
    bool limit2Raw = ReadInputBitRaw(AX4_LIMIT2_ADDR, AX4_LIMIT2_BIT, AX4_LIMIT2_ACTIVE_HIGH);
    bool limitRaw = (limit1Raw || limit2Raw);

    // HOME (기존 유지)
    bool homeRaw = ReadInputBitRaw(AX2_HOME_ADDR, AX2_HOME_BIT, AX2_HOME_ACTIVE_HIGH);

    g_ax2LimitOn = limitRaw;
    g_ax2HomeOn = homeRaw;

    UpdateAx2SensorLabels(g_ax2ServoReady.load(), limitRaw, homeRaw);

    if (g_ax2ServoReady.load()) {
        if (limitRaw) {
            if (!g_ax2LimitLatched.load()) {
                g_ax2LimitLatched = true;
                g_ax2LimitBlocking = true;
                g_ax2StopIssuedOnLimit = false;
                g_ax2HomingStarted = false;
            }
            Axis2HandleLimitOnceAndHome();

            // 요구사항: HoistUp은 AX4 리밋센서에 닿아 멈추면 완료로 처리
            TaskState hu = GetTaskState(TaskId::HoistUp);
            if (Axis2IsIdle() && (hu == TaskState::Running || hu == TaskState::Stopped)) {
                SetTaskState(TaskId::HoistUp, TaskState::Done);
            }
        }
        else {
            g_ax2LimitBlocking = false;
            g_ax2LimitLatched = false;
            g_ax2StopIssuedOnLimit = false;
            g_ax2HomingStarted = false;
            g_ax2LimitIdleTime = 0;
        }
        DWORD now = GetTickCount();
        if (homeRaw) {
            if (!g_ax2HomeDebounceOn.load()) {
                if (now - g_ax2HomeLastTick.load() >= AX4_SENSOR_DEBOUNCE_MS) {
                    g_ax2HomeDebounceOn = true;
                    if (!g_ax2HomeRampIssued.load()) {
                        //Axis2HomeSoftDecelTo500();
                        //g_ax2HomeRampIssued = true;
                    }
                }
            }
        }
        else {
            if (g_ax2HomeDebounceOn.load()) g_ax2HomeDebounceOn = false;
            g_ax2HomeLastTick = now;
            g_ax2HomeRampIssued = false;
        }
    }
}

// =======================================
// GPIO 창 로직을 그대로 통합
// - EAPI 래퍼, 은행 선택, 디바운스, 펄스, 토글 컨트롤, 오버레이 포함
// =======================================
#define BANK_MAX 4
typedef struct {
    uint8_t supPinNum;
    uint32_t supInput;
    uint32_t supOutput;
} GPIOInfo, * PGPIOInfo;

static HINSTANCE g_hEAPIDLL = NULL;
static GPIOInfo g_gpioInfo[BANK_MAX];
static const int kFixedBank = -1;
static int g_gpioBank = -1;

// 마스크
static const uint32_t kMaskDI0_7 = 0x000000FFu;
static const uint32_t kMaskDO8_15 = 0x0000FF00u;

// 출력 펄스 예약
bool  g_outputPendingOff[16] = { false };
DWORD g_outputOnTick[16] = { 0 };
const DWORD kOutputPulseMs = 50;

// 출력 후 입력 안정화 지연
static const UINT kPostOutputInputDelayMs = 8;

// 입력 디바운스
static const int  kDI_SampleCount = 5;
const UINT kPollIntervalMs = 20; // 빠른 폴링
static const int  kDI_BufferDepth = kDI_SampleCount;

// DI 오버레이: Motioning(DI0) 표시 강제 OFF를 위해 사용
static bool g_uiOverlayDIValid[8] = { false };
static bool g_uiOverlayDILevel[8] = { false };

static HWND g_lblDI[8] = { 0 };  // 좌측 DI 라벨
static bool g_cachedLevels[16] = { 0 };

// 토글 컨트롤
static const TCHAR* TOGGLE_CLS = _T("GPIO_CTRL_TOGGLE");
struct ToggleState { bool on = false; bool enabled = true; int pin = -1; };
static HFONT g_fontToggle = NULL;
static HWND  g_swDO[16] = { 0 }; // DO8..15 사용

// 외부 함수 포인터 래퍼
static HINSTANCE GetEAPIInstance() {
    if (g_hEAPIDLL == NULL) g_hEAPIDLL = OpenEAPI();
    return g_hEAPIDLL;
}
static bool InitializeEAPI(HINSTANCE hDLL) {
    uint32_t status;
    EAPIFunction(hDLL, EApiLibInitialize);
    if (EApiLibInitialize) {
        status = EApiLibInitialize();
        if (status != EAPI_STATUS_SUCCESS && status != EAPI_STATUS_INITIALIZED) return false;
    }
    return true;
}
static bool DeInitializeEAPI(void) {
    if (g_hEAPIDLL) {
        uint32_t status;
        EAPIFunction(g_hEAPIDLL, EApiLibUnInitialize);
        if (EApiLibUnInitialize) { status = EApiLibUnInitialize(); (void)status; }
        CloseEAPI(g_hEAPIDLL);
        g_hEAPIDLL = NULL;
    }
    return true;
}

static uint32_t BankValidMask(int bank) {
    uint8_t n = g_gpioInfo[bank].supPinNum;
    if (n == 0) return 0;
    if (n >= 32) return 0xFFFFFFFFu;
    return (1u << n) - 1u;
}
bool EnumerateGPIO() {
    uint32_t status, supportPin, id;
    uint8_t found = 0;
    HINSTANCE hDLL = GetEAPIInstance();
    if (!hDLL) return false;
    if (!InitializeEAPI(hDLL)) return false;
    EAPIFunction(hDLL, EApiGPIOGetDirectionCaps);
    if (!EApiGPIOGetDirectionCaps) return false;
    for (uint8_t i = 0; i < BANK_MAX; i++) {
        id = EAPI_ID_GPIO_BANK(i);
        status = EApiGPIOGetDirectionCaps(id, &g_gpioInfo[i].supInput, &g_gpioInfo[i].supOutput);
        if (status != EAPI_STATUS_SUCCESS) { g_gpioInfo[i].supPinNum = 0; continue; }
        supportPin = g_gpioInfo[i].supInput | g_gpioInfo[i].supOutput;
        if (supportPin > 0) {
            uint8_t j;
            for (j = 32; j > 0; j--) {
                if (supportPin & (1u << (j - 1))) { g_gpioInfo[i].supPinNum = j; break; }
            }
            if (j == 0) g_gpioInfo[i].supPinNum = 0;
        }
        else {
            g_gpioInfo[i].supPinNum = 0;
        }
        found++;
    }
    return found > 0;
}
bool PickBank_DI0_7_DO8_15() {
    if (kFixedBank >= 0 && kFixedBank < BANK_MAX) { g_gpioBank = kFixedBank; return true; }
    for (int b = 0; b < BANK_MAX; ++b) {
        uint32_t valid = BankValidMask(b);
        if ((valid & 0x0000FFFFu) != 0x0000FFFFu) continue;
        uint32_t diOk = g_gpioInfo[b].supInput & kMaskDI0_7;
        uint32_t doOk = g_gpioInfo[b].supOutput & kMaskDO8_15;
        if (diOk == kMaskDI0_7 && doOk == kMaskDO8_15) { g_gpioBank = b; return true; }
    }
    int best = -1, scoreBest = -1;
    for (int b = 0; b < BANK_MAX; ++b) {
        uint32_t valid = BankValidMask(b) & 0x0000FFFFu;
        int diCnt = 0, doCnt = 0;
        for (int i = 0; i < 8; ++i) if ((valid & (1u << i)) && (g_gpioInfo[b].supInput & (1u << i))) diCnt++;
        for (int i = 8; i < 16; ++i) if ((valid & (1u << i)) && (g_gpioInfo[b].supOutput & (1u << i))) doCnt++;
        int score = diCnt + doCnt;
        if (score > scoreBest) { scoreBest = score; best = b; }
    }
    if (best >= 0) { g_gpioBank = best; return true; }
    return false;
}

// EAPI helpers
static bool GPIO_Single_GetDirection(uint32_t globalPin, uint32_t* pBit, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetDirection); if (!EApiGPIOGetDirection) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t dir = 0; uint32_t st = EApiGPIOGetDirection(id, 1u, &dir);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pBit) *pBit = (dir & 1u); return true; }
    return false;
}
static bool GPIO_Single_SetDirection(uint32_t globalPin, bool makeOutput, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetDirection); if (!EApiGPIOSetDirection) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t st = EApiGPIOSetDirection(id, 1u, makeOutput ? 1u : 0u);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
static bool GPIO_Single_GetLevel(uint32_t globalPin, uint32_t* pBit, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetLevel); if (!EApiGPIOGetLevel) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t lvl = 0; uint32_t st = EApiGPIOGetLevel(id, 1u, &lvl);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pBit) *pBit = (lvl & 1u); return true; }
    return false;
}
static bool GPIO_Single_SetLevel(uint32_t globalPin, bool hi, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetLevel); if (!EApiGPIOSetLevel) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t st = EApiGPIOSetLevel(id, 1u, hi ? 1u : 0u);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
static bool GPIO_GetDirection_Bank(int bank, uint32_t mask, uint32_t* pValue, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetDirection); if (!EApiGPIOGetDirection) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t dir = 0; uint32_t st = EApiGPIOGetDirection(id, mask, &dir);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pValue) *pValue = dir; return true; }
    return false;
}
static bool GPIO_SetDirection_Bank(int bank, uint32_t mask, uint32_t setVal, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetDirection); if (!EApiGPIOSetDirection) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t st = EApiGPIOSetDirection(id, mask, setVal);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
static bool GPIO_GetLevel_Bank(int bank, uint32_t mask, uint32_t* pValue, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetLevel); if (!EApiGPIOGetLevel) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t lvl = 0; uint32_t st = EApiGPIOGetLevel(id, mask, &lvl);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pValue) *pValue = lvl; return true; }
    return false;
}
static bool GPIO_SetLevel_Bank(int bank, uint32_t mask, uint32_t setVal, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetLevel); if (!EApiGPIOSetLevel) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t st = EApiGPIOSetLevel(id, mask, setVal);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
bool EnsureDO8to15AsOutput_BankFirst() {
    if (g_gpioBank < 0) return false;
    uint32_t valid = BankValidMask(g_gpioBank);
    uint32_t targetMask = (kMaskDO8_15 & valid) & g_gpioInfo[g_gpioBank].supOutput;
    if (!targetMask) return false;
    uint32_t st = 0;
    return GPIO_SetDirection_Bank(g_gpioBank, targetMask, targetMask, &st);
}

// 디바운스 버퍼
static uint8_t g_diSamples[8][kDI_BufferDepth] = { 0 };
static int     g_diSamplePos = 0;
bool    g_diStable[8] = { 0 };

static bool MajorityOfSamples(int pin) {
    int ones = 0, total = 0;
    for (int k = 0; k < kDI_BufferDepth; ++k) {
        ones += (g_diSamples[pin][k] ? 1 : 0);
        total++;
    }
    return ones >= (total + 1) / 2;
}
static bool SampleDI_Once(uint32_t& diBitsOut) {
    diBitsOut = 0;
    if (g_gpioBank < 0) return false;
    uint32_t valid = BankValidMask(g_gpioBank);
    uint32_t maskDI = (kMaskDI0_7 & valid) & g_gpioInfo[g_gpioBank].supInput;
    uint32_t lvl = 0; uint32_t st = 0;
    bool ok = GPIO_GetLevel_Bank(g_gpioBank, maskDI, &lvl, &st);
    if (!ok) {
        lvl = 0;
        for (int i = 0; i < 8; ++i) {
            if (!(maskDI & (1u << i))) continue;
            uint32_t bit = 0, stp = 0;
            if (GPIO_Single_GetLevel((uint32_t)(g_gpioBank * 32 + i), &bit, &stp)) {
                if (bit) lvl |= (1u << i);
            }
        }
        ok = true;
    }
    if (ok) diBitsOut = lvl & 0xFFu;
    return ok;
}
static void ApplyDIOverlayRules() {
    // STO(DO10) 눌렀을 때 Motioning(DI0)을 강제로 OFF로 표시
    if (g_uiOverlayDIValid[0]) {
        if (!g_diStable[0]) {
            g_uiOverlayDIValid[0] = false;
        }
    }
}
static void UpdateDIText(int i, bool supported, bool level, HWND hWnd) {
    if (!g_lblDI[i]) return;
    TCHAR buf[64];
    if (!supported) {
        _stprintf_s(buf, _T("Pin %d : N/A"), i);
        SetWindowText(g_lblDI[i], buf);
        return;
    }
    bool showLevel = level;
    if (g_uiOverlayDIValid[i]) showLevel = g_uiOverlayDILevel[i];
    if (i == 0) _stprintf_s(buf, _T("Motioning : %s"), showLevel ? _T("ON") : _T("OFF"));
    else if (i == 1) _stprintf_s(buf, _T("Catched   : %s"), showLevel ? _T("OFF") : _T("ON"));
    else _stprintf_s(buf, _T("Pin %d : %s"), i, showLevel ? _T("OFF") : _T("ON"));
    SetWindowText(g_lblDI[i], buf);
}

static bool s_servoOffHandledOnce = false;

//static void RefreshInputsWithDebounce(HWND hWnd) {
//    uint32_t diNow = 0;
//    if (!SampleDI_Once(diNow)) return;
//    for (int i = 0; i < 8; ++i) g_diSamples[i][g_diSamplePos] = (diNow >> i) & 1u;
//    g_diSamplePos = (g_diSamplePos + 1) % kDI_BufferDepth;
//    for (int i = 0; i < 8; ++i) {
//        bool newStable = MajorityOfSamples(i);
//        g_diStable[i] = newStable;
//        g_cachedLevels[i] = newStable;
//    }
//    ApplyDIOverlayRules();
//    for (int i = 0; i < 8; ++i) {
//        bool sup = (g_gpioInfo[g_gpioBank].supInput & (1u << i)) != 0;
//        UpdateDIText(i, sup, g_diStable[i], hWnd);
//    }
//    // ============================================
//    // ★ Motioning(DI0) ON → OFF 전이 감지
//    //    → GripOpen / GripClose 를 Done 으로 전환
//    // ============================================
//    {
//        static bool s_prevMotioning = false;
//        static bool s_init = false;
//
//        // DI0 = Motioning (input 1번 신호)
//        bool curMotioning = g_diStable[0];
//
//        if (!s_init) {
//            // 첫 호출 시에는 기준값만 세팅하고 끝
//            s_prevMotioning = curMotioning;
//            s_init = true;
//        }
//        else {
//            // 이전에 ON이었다가 지금 OFF로 떨어진 순간만 감지
//            if (s_prevMotioning && !curMotioning) {
//                // GripOpen 이 진행중이면 완료로
//                TaskState openState = g_taskStatus[(int)TaskId::GripOpen].state.load();
//                if (openState == TaskState::Running) {
//                    SetTaskState(TaskId::GripOpen, TaskState::Done);
//                }
//
//                // GripClose 도 진행중이면 완료로
//                TaskState closeState = g_taskStatus[(int)TaskId::GripClose].state.load();
//                if (closeState == TaskState::Running) {
//                    SetTaskState(TaskId::GripClose, TaskState::Done);
//                }
//            }
//            // ★ 추가: 프로그램 전체에서 딱 1번만 GripServoOff 상태를 보고
//            //         GripOpen / GripClose 를 Done 으로 맞춰준다.
//            if (!s_servoOffHandledOnce) {
//                TaskState servoOffState =
//                    g_taskStatus[(int)TaskId::GripServoOff].state.load();
//                if (servoOffState == TaskState::Done) {
//
//                    TaskState openState =
//                        g_taskStatus[(int)TaskId::GripOpen].state.load();
//                    if (openState == TaskState::Running) {
//                        SetTaskState(TaskId::GripOpen, TaskState::Done);
//                    }
//
//                    TaskState closeState =
//                        g_taskStatus[(int)TaskId::GripClose].state.load();
//                    if (closeState == TaskState::Running) {
//                        SetTaskState(TaskId::GripClose, TaskState::Done);
//                    }
//
//                    // ★ 한 번 처리했으니 다시는 안 하도록 플래그 ON
//                    s_servoOffHandledOnce = true;
//                }
//            }
//
//            // 이전 Motioning 상태 업데이트
//            s_prevMotioning = curMotioning;
//        }
//    }
//}
static bool ReadDO16Bits(uint32_t& dirOut, uint32_t& lvlOut) {
    dirOut = 0; lvlOut = 0;
    if (g_gpioBank < 0) return false;
    uint32_t valid = BankValidMask(g_gpioBank);
    uint32_t mask16 = (kMaskDI0_7 | kMaskDO8_15) & valid;
    uint32_t st1 = 0, st2 = 0;
    bool okDir = GPIO_GetDirection_Bank(g_gpioBank, mask16, &dirOut, &st1);
    bool okLvl = GPIO_GetLevel_Bank(g_gpioBank, mask16, &lvlOut, &st2);
    if (!okDir || !okLvl) {
        dirOut = 0; lvlOut = 0;
        for (int i = 0; i < 16; ++i) {
            uint32_t gp = g_gpioBank * 32 + i;
            uint32_t bit = 0, st = 0;
            if (GPIO_Single_GetDirection(gp, &bit, &st)) { if (bit) dirOut |= (1u << i); }
            if (GPIO_Single_GetLevel(gp, &bit, &st)) { if (bit) lvlOut |= (1u << i); }
        }
    }
    return true;
}
static void RefreshOutputs(HWND hWnd) {
    if (g_gpioBank < 0) return;
    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl)) return;
    for (int i = 8; i < 16; ++i) {
        bool sup = (g_gpioInfo[g_gpioBank].supOutput & (1u << i)) != 0;
        bool on = (lvl & (1u << i)) != 0;
        g_cachedLevels[i] = on;
        if (g_swDO[i]) {
            EnableWindow(g_swDO[i], sup ? TRUE : FALSE);
            ToggleState* st = (ToggleState*)GetWindowLongPtr(g_swDO[i], GWLP_USERDATA);
            if (st) { st->on = sup ? on : false; InvalidateRect(g_swDO[i], NULL, TRUE); }
        }
    }
}
void RefreshLevels(HWND hWnd) {
    if (g_gpioBank < 0) return;
    //RefreshInputsWithDebounce(hWnd);
    RefreshOutputs(hWnd);
}

// Toggle control
static void DrawRoundedRect(HDC hdc, const RECT& rc, int r, COLORREF fill, COLORREF border) {
    HBRUSH hb = CreateSolidBrush(fill);
    HPEN   hp = CreatePen(PS_SOLID, 1, border);
    HGDIOBJ ob = SelectObject(hdc, hb);
    HGDIOBJ op = SelectObject(hdc, hp);
    RoundRect(hdc, rc.left, rc.top, rc.right, rc.bottom, r, r);
    SelectObject(hdc, op);
    SelectObject(hdc, ob);
    DeleteObject(hp);
    DeleteObject(hb);
}
static LRESULT CALLBACK ToggleProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    ToggleState* st = (ToggleState*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
    switch (msg) {
    case WM_NCCREATE: {
        CREATESTRUCT* cs = (CREATESTRUCT*)lParam;
        ToggleState* ns = new ToggleState();
        ns->on = false; ns->enabled = true; ns->pin = (int)(INT_PTR)cs->lpCreateParams;
        SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)ns);
        return TRUE;
    }
    case WM_NCDESTROY:
        if (st) { delete st; SetWindowLongPtr(hWnd, GWLP_USERDATA, 0); }
        return 0;
    case WM_ENABLE:
        if (st) st->enabled = (wParam != FALSE);
        InvalidateRect(hWnd, NULL, TRUE);
        return 0;
    case WM_SETFONT:
        g_fontToggle = (HFONT)wParam;
        return 0;
    case WM_GETFONT:
        return (LRESULT)g_fontToggle;
    case WM_LBUTTONDOWN:
    case WM_KEYDOWN:
        if (msg == WM_KEYDOWN && (wParam != VK_SPACE && wParam != VK_RETURN)) break;
        if (st && st->enabled) {
            st->on = !st->on;
            InvalidateRect(hWnd, NULL, TRUE);
            HWND parent = GetParent(hWnd);
            if (parent) SendMessage(parent, WM_COMMAND, MAKELONG(GetDlgCtrlID(hWnd), BN_CLICKED), (LPARAM)hWnd);
        }
        return 0;
    case WM_PAINT: {
        PAINTSTRUCT ps; HDC hdc = BeginPaint(hWnd, &ps);
        RECT rc; GetClientRect(hWnd, &rc);
        HDC memdc = CreateCompatibleDC(hdc);
        HBITMAP membmp = CreateCompatibleBitmap(hdc, rc.right - rc.left, rc.bottom - rc.top);
        HGDIOBJ oldbmp = SelectObject(memdc, membmp);
        HBRUSH bg = CreateSolidBrush(GetSysColor(COLOR_BTNFACE));
        FillRect(memdc, &rc, bg);
        DeleteObject(bg);
        RECT rToggle = rc;
        rToggle.top += 4; rToggle.bottom -= 4; rToggle.left += 4; rToggle.right -= 4;
        COLORREF ColOn = RGB(40, 167, 69);
        COLORREF ColOff = RGB(217, 83, 79);
        COLORREF bgcol = (st && st->on) ? ColOn : ColOff;
        DrawRoundedRect(memdc, rToggle, 18, bgcol, RGB(80, 80, 80));
        int w = rToggle.right - rToggle.left;
        int h = rToggle.bottom - rToggle.top;
        int diameter = h - 6;
        int cx_off = (st && st->on) ? (w - diameter - 6) : 6;
        RECT rKnob = { rToggle.left + cx_off, rToggle.top + 3, rToggle.left + cx_off + diameter, rToggle.top + 3 + diameter };
        HBRUSH knobBrush = CreateSolidBrush(RGB(240, 240, 240));
        HPEN   knobPen = CreatePen(PS_SOLID, 1, RGB(100, 100, 100));
        HGDIOBJ okb = SelectObject(memdc, knobBrush);
        HGDIOBJ okp = SelectObject(memdc, knobPen);
        Ellipse(memdc, rKnob.left, rKnob.top, rKnob.right, rKnob.bottom);
        SelectObject(memdc, okp); DeleteObject(knobPen);
        SelectObject(memdc, okb); DeleteObject(knobBrush);
        const TCHAR* txt = (st && st->on) ? _T("ON") : _T("OFF");
        SetBkMode(memdc, TRANSPARENT);
        SetTextColor(memdc, RGB(255, 255, 255));
        HFONT f = (HFONT)SendMessage(hWnd, WM_GETFONT, 0, 0);
        HGDIOBJ of = NULL; if (f) of = SelectObject(memdc, f);
        RECT rText = rToggle; rText.left += 10; rText.right -= 10;
        DrawText(memdc, txt, -1, &rText, DT_SINGLELINE | DT_CENTER | DT_VCENTER);
        if (of) SelectObject(memdc, of);
        BitBlt(hdc, 0, 0, rc.right - rc.left, rc.bottom - rc.top, memdc, 0, 0, SRCCOPY);
        SelectObject(memdc, oldbmp);
        DeleteObject(membmp);
        DeleteDC(memdc);
        EndPaint(hWnd, &ps);
        return 0;
    }
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}
static void RegisterToggleClass(HINSTANCE hInst) {
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = ToggleProc;
    wc.hInstance = hInst;
    wc.lpszClassName = TOGGLE_CLS;
    wc.hCursor = LoadCursor(NULL, IDC_HAND);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    RegisterClass(&wc);
}

// 핀 이름
static const TCHAR* g_DOFuncNames[16] = {
    nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
    _T("Open"),     // 8
    _T("Close"),    // 9
    _T("STO"),      // 10
    _T("None"),     // 11
    _T("None"),     // 12
    _T("None"),     // 13
    nullptr,        // 14
    nullptr         // 15
};

// 출력 동작(펄스 예약/오버레이/입력샘플 반영)
void ToggleDO_HW(int pin, bool turnOn, HWND hWnd)
{
    if (g_gpioBank < 0) return;
    uint32_t bit = (1u << pin);
    if (!(g_gpioInfo[g_gpioBank].supOutput & bit)) return;

    (void)EnsureDO8to15AsOutput_BankFirst();

    uint32_t gp = (uint32_t)(g_gpioBank * 32 + pin);
    uint32_t st = 0;
    GPIO_Single_SetDirection(gp, true, &st);
    GPIO_Single_SetLevel(gp, turnOn, &st);

    // UI 즉시 반영
    if (g_swDO[pin]) {
        ToggleState* ts = (ToggleState*)GetWindowLongPtr(g_swDO[pin], GWLP_USERDATA);
        if (ts) { ts->on = turnOn; InvalidateRect(g_swDO[pin], NULL, TRUE); }
    }

    // 출력 직후 입력 안정화 지연 후 1회 추가 샘플
    if (kPostOutputInputDelayMs > 0) {
        Sleep(kPostOutputInputDelayMs);
        uint32_t diNow = 0;
        if (SampleDI_Once(diNow)) {
            g_diSamples[0][g_diSamplePos] = (diNow >> 0) & 1u;
            g_diSamples[1][g_diSamplePos] = (diNow >> 1) & 1u;
            g_diSamples[2][g_diSamplePos] = (diNow >> 2) & 1u;
            g_diSamples[3][g_diSamplePos] = (diNow >> 3) & 1u;
            g_diSamples[4][g_diSamplePos] = (diNow >> 4) & 1u;
            g_diSamples[5][g_diSamplePos] = (diNow >> 5) & 1u;
            g_diSamples[6][g_diSamplePos] = (diNow >> 6) & 1u;
            g_diSamples[7][g_diSamplePos] = (diNow >> 7) & 1u;
            g_diSamplePos = (g_diSamplePos + 1) % kDI_BufferDepth;
        }
    }

    // STO(DO10) 눌렀을 때 Motioning(DI0) 표시 강제 OFF
    if (pin == 10 && turnOn) {
        g_uiOverlayDIValid[0] = true;
        g_uiOverlayDILevel[0] = false;
    }

    // 펄스 자동 OFF
    if (pin == 10 && turnOn) { g_outputPendingOff[10] = true; g_outputOnTick[10] = GetTickCount(); }
    else if (!turnOn) { g_outputPendingOff[pin] = false; }

    // 출력 후 곧바로 입력 갱신
    //RefreshInputsWithDebounce(hWnd);
}

// 축0 바코드가 Conveyor 위치에 있는지 확인 (0x6063 값 기준)
bool IsAxisLeft()
{
    // GO_Conveyor()에서 사용한 타겟 바코드 값과 동일하게 맞춰줌
    const long long targetBc = 134;   // GO_Conveyor 의 targetBarcodeAbs
    const int bcEps = 5;                 // 허용 오차 (필요시 조정)

    int nowBc = 0;
    if (!ReadAxis_TxPDO_6063(kAxisSlaveId[9], nowBc))
        return false;

    long long diff = (long long)nowBc - targetBc;
    return std::llabs(diff) <= bcEps;
}

// 축0 바코드가 Workstation 위치에 있는지 확인 (0x6063 값 기준)
bool IsAxisRight()
{
    // GO_Conveyor()에서 사용한 타겟 바코드 값과 동일하게 맞춰줌
    const long long targetBc = 2776;   // GO_Conveyor 의 targetBarcodeAbs
    const int bcEps = 5;                 // 허용 오차 (필요시 조정)

    int nowBc = 0;
    if (!ReadAxis_TxPDO_6063(kAxisSlaveId[9], nowBc))
        return false;

    long long diff = (long long)nowBc - targetBc;
    return std::llabs(diff) <= bcEps;
}

// 축0 바코드가 Workstation 위치에 있는지 확인 (0x6063 값 기준)
bool IsAxisWorkstation()
{
    // GO_Conveyor()에서 사용한 타겟 바코드 값과 동일하게 맞춰줌
    const long long targetBc = 1457;   // GO_Conveyor 의 targetBarcodeAbs
    const int bcEps = 5;                 // 허용 오차 (필요시 조정)

    int nowBc = 0;
    if (!ReadAxis_TxPDO_6063(kAxisSlaveId[9], nowBc))
        return false;

    long long diff = (long long)nowBc - targetBc;
    return std::llabs(diff) <= bcEps;
}

bool IsAxisLeftStopped()
{
    if (!IsAxisLeft())
        return false;

    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double v = std::fabs(st.axesStatus[7].actualVelocity);
    return v <= 1.0;    // 적당한 정지 기준
}

bool IsAxisRightStopped()
{
    if (!IsAxisRight())
        return false;

    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double v = std::fabs(st.axesStatus[7].actualVelocity);
    return v <= 1.0;
}

bool IsAxisWorkstationStopped()
{
    if (!IsAxisWorkstation())
        return false;

    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double v = std::fabs(st.axesStatus[7].actualVelocity);
    return v <= 1.0;
}


// 축2가 Up 위치(0 근처)인지 확인
bool IsAxis4Up()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 0;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[4];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

bool IsAxis5ok()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 400000;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[5];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

bool IsAxis7ok()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 400000;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[7];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

// 축2가 Up 위치(0 근처)인지 확인
bool IsAxis4Workdown()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 45000;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[4];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

// 축2가 Up 위치(0 근처)인지 확인
bool IsAxis4Conveyordown()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 68000;       // HoistDown() target (요구사항)
    //const long long targetPos = 40500;       // HoistDown() target (요구사항)
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[4];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}


// Motioning 출력(DO11)의 현재 상태 읽기
static bool IsMotioningOutputOn()
{
    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl))
        return false;  // 읽기 실패하면 일단 OFF 취급(원하면 true로 바꿔도 됨)

    const uint32_t bitMotion = (1u << 11); // Pin 11 = Motioning 출력
    bool on = (lvl & bitMotion) != 0;
    return on;
}

// 예: DI1을 "박스 감지" 센서로 쓴다고 가정 (input2가 DI1인 경우)
bool HasBox()
{
    // g_diStable[1] == false  → input2 ON → 박스 있음
    // g_diStable[1] == true → input2 OFF → 박스 없음
    return !g_diStable[1];
}

// 박스 "없음" 상태를 기다릴 때 쓰기 위한 헬퍼
bool NoBox()
{
    return !HasBox();
}

// 모든 축이 정지 + Motioning(DI0) OFF 될 때까지 대기
// 모든 축이 정지 + GripServoOff 태스크 Done 될 때까지 대기
bool WaitAllAxesStopped(double velEps, DWORD timeoutMs)
{
    DWORD start = GetTickCount();
    CoreMotionStatus st{};

    while (true) {
        g_cm.GetStatus(&st);

        bool allStopped = true;
        for (int ax = 0; ax < 9; ++ax) {
            double v = std::fabs(st.axesStatus[ax].actualVelocity);
            if (v > velEps) {
                allStopped = false;
                break;
            }
        }

        // 둘 다 만족해야 "정지 완료"로 인정
        if (allStopped) {
            return true;    // 모든 축 멈췄고, GripServoOff도 Done
        }

        if (GetTickCount() - start > timeoutMs) {
            return false;   // 타임아웃 (축 또는 GripServoOff 둘 중 뭔가 아직)
        }

        ::Sleep(20);
    }
}


// 주어진 조건 함수(pred)가 true가 될 때까지 기다리는 유틸
//  - pred() 가 true 이면 즉시 true 리턴
//  - timeoutMs 안에 만족 못하면 false 리턴
bool WaitUntil(bool (*pred)(), DWORD timeoutMs, DWORD pollMs = 20)
{
    DWORD start = GetTickCount();
    while (true) {
        if (pred())
            return true;

        if (GetTickCount() - start > timeoutMs)
            return false;

        ::Sleep(pollMs);
    }
}

// TaskId 기준으로 시퀀스가 끝날 때까지 대기
//  - Done  이면 true
//  - Failed / Stopped / timeout 이면 false
bool WaitTaskFinished(TaskId id, DWORD timeoutMs, DWORD pollMs = 20)
{
    DWORD start = GetTickCount();

    while (true) {
        TaskState st = g_taskStatus[(int)id].state.load();

        // 정상 완료
        if (st == TaskState::Done) {
            return true;
        }

        // 실패 / 중단
        if (st == TaskState::Failed || st == TaskState::Stopped) {
            return false;
        }

        // 타임아웃
        if (GetTickCount() - start > timeoutMs) {
            return false;
        }

        ::Sleep(pollMs);
    }
}
// Unload 시퀀스 시작 전 조건:
//  - 축0 : Conveyor 위치
//  - 축2 : Up 상태
//  - Gripper : Close
//  - 박스 있음 (Catched ON)

void LedStartBlinkForTask(HWND hWnd, TaskId tid);
void LedCancel(HWND hWnd);
void DoGripServoOff_Compat(HWND hWnd) {};


// =======================================
// Demo 동작: Lift/Down/Go, Stop
// =======================================
//void DoOpen_Compat(HWND hWnd)
//{
//    // 그리퍼 Open 명령 시작
//    SetTaskState(TaskId::Open, TaskState::Running);
//
//
//    //   // 1) STO 펄스 (10번: ON → 타이머로 자동 OFF)
//       //DoGripServoOff_Compat(hWnd);
//
//       // 2) Close OFF
//    ToggleDO_HW(9, false, hWnd);
//    SetTaskState(TaskId::Close, TaskState::Idle);
//
//    // 3) Open 신호: 8번을 한번 OFF 했다가 ON (에지 만들기)
//    ToggleDO_HW(8, false, hWnd);
//    ToggleDO_HW(8, true, hWnd);   // 이 상태가 계속 유지 → Open 상태
//
//    //DoGripServoOff_Compat(hWnd);
//}

// =======================================
// Open/Close: Axis0 move + 완료 판정(타겟 위치에 정지)
//  - 동작 시작: TaskState=Running + LED Blink
//  - 완료 조건: Axis0 actualPos가 target 근처 + actualVelocity ~ 0
// =======================================
static void StartAxis0MoveDoneMonitor(TaskId tid, long long targetPos,
    long long posEps = 10, double velEps = 1.0,
    DWORD timeoutMs = 15000, DWORD pollMs = 20)
{
    std::thread([=]() {
        DWORD start = GetTickCount();
        while (true) {
            TaskState st = GetTaskState(tid);
            if (st == TaskState::Done || st == TaskState::Failed || st == TaskState::Stopped || st == TaskState::Idle)
                return;

            if (!g_commStarted) {
                SetTaskState(tid, TaskState::Failed);
                return;
            }

            CoreMotionStatus ms{};
            g_cm.GetStatus(&ms);
            const auto& ax = ms.axesStatus[0];

            long long perr = (long long)ax.actualPos - targetPos;
            double v = std::fabs(ax.actualVelocity);

            if (std::llabs(perr) <= posEps && v <= velEps) {
                SetTaskState(tid, TaskState::Done);
                return;
            }

            if (GetTickCount() - start > timeoutMs) {
                // 타임아웃 시, 아직 Running이면 Failed로
                if (GetTaskState(tid) == TaskState::Running)
                    SetTaskState(tid, TaskState::Failed);
                return;
            }

            ::Sleep(pollMs);
        }
        }).detach();
}

void Open() {
    if (!g_commStarted) { SetTaskState(TaskId::Open, TaskState::Failed); return; }

    // 상태/LED
    SetTaskState(TaskId::Open, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Open);

    const long long tgt = -20;
    // Move (OPEN은 타겟 도달이 아니라 AX0 리밋센서로 Stop될 때 완료)
    StartMoveWithApproach(0, tgt, TaskId::Open,
        40000.0, 100.0, 1000.0,
        10.0, 2.0, 30000,
        10000, { 10000.0, 500.0, 500.0 });

    // 완료 판정은 AxLimitSensorTimerProc()의 AX0 limit stop 로직에서 수행
}

void Close() {
    if (!g_commStarted) { SetTaskState(TaskId::Close, TaskState::Failed); return; }

    // 상태/LED
    SetTaskState(TaskId::Close, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Close);

    // Move
    const long long tgt = 48500;
    StartMoveWithApproach(0, tgt, TaskId::Close,
        40000.0, 100.0, 1000.0,
        10.0, 2.0, 30000,
        10000, { 10000.0, 500.0, 500.0 });

    // 완료 감시
    StartAxis0MoveDoneMonitor(TaskId::Close, tgt);
}

//void DoGripServoOff_Compat(HWND hWnd) {
//    ToggleDO_HW(10, true, hWnd); // STO ON (펄스)
//    ToggleDO_HW(10, false, hWnd); // STO ON (펄스)
//    std::this_thread::sleep_for(std::chrono::seconds(1));
//    ToggleDO_HW(11, false, hWnd); // LED OFF
//    SetTaskState(TaskId::GripServoOff, TaskState::Done);
//}

void HoistDown() {
    if (!g_commStarted) { SetTaskState(TaskId::HoistDown, TaskState::Failed); return; }

    SetTaskState(TaskId::HoistDown, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::HoistDown); // ✅ 동일 LED

    int ax = 4;
    long long tgt = 68000;
    //long long tgt = 40500;
    StartMoveWithApproach(ax, tgt, TaskId::HoistDown,
        40000.0, 1000.0, 1500.0,
        10.0, 2.0, 30000,
        3500, { 1000.0, 80.0, 10.0 });
}
void HoistUp() {
    if (!g_commStarted) { SetTaskState(TaskId::HoistUp, TaskState::Failed); return; }

    SetTaskState(TaskId::HoistUp, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::HoistUp); // ✅ 동일 LED

    int ax = 4;
    long long tgt = 0;
    StartMoveWithApproach(ax, tgt, TaskId::HoistUp,
        40000.0, 1000.0, 1500.0,
        10.0, 2.0, 30000,
        3500, { 1000.0, 80.0, 10.0 });
}
void DoStopAll(HWND hWnd) {
    LedCancel(hWnd);  // ✅ 추가
    g_bcRunner.Stop();
    for (int a = 0; a < 9; ++a) StopAxis(a);
    for (int i = 0; i < (int)TaskId::COUNT; ++i) {
        if (g_taskStatus[i].state.load() == TaskState::Running) SetTaskState((TaskId)i, TaskState::Stopped);
    }
}
void GoWorkstation() {
    if (!g_commStarted) { SetTaskState(TaskId::GoWorkstation, TaskState::Failed); return; }

    SetTaskState(TaskId::GoWorkstation, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::GoWorkstation); // ✅ 동일 LED

    BarcodeParams p{};
    p.axis = 7;
    p.targetBarcodeAbs = 1457;
    p.mainVel = 10000.0; p.mainAcc = 1000.0; p.mainDec = 1000.0;
    p.corrVel = 1000.0; p.corrAcc = 300.0; p.corrDec = 300.0;
    p.deadband = 1;
    p.gear = 4.4248; p.wheelDia = 115.0; p.motorCpr = 10000.0; p.bcMmPerCnt = 1.07;
    g_bcRunner.Start(p, TaskId::GoWorkstation);
}
void GoLeft() {
    if (!g_commStarted) { SetTaskState(TaskId::GoLeft, TaskState::Failed); return; }

    SetTaskState(TaskId::GoLeft, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::GoLeft); // ✅ 동일 LED

    BarcodeParams p{};
    p.axis = 7;
    p.targetBarcodeAbs = 134;
    p.mainVel = 10000.0; p.mainAcc = 1000.0; p.mainDec = 1000.0;
    p.corrVel = 1000.0; p.corrAcc = 300.0; p.corrDec = 300.0;
    p.deadband = 1;
    p.gear = 4.4248; p.wheelDia = 115.0; p.motorCpr = 10000.0; p.bcMmPerCnt = 1.07;
    g_bcRunner.Start(p, TaskId::GoLeft);
}

void GoRight() {
    if (!g_commStarted) { SetTaskState(TaskId::GoRight, TaskState::Failed); return; }

    SetTaskState(TaskId::GoRight, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::GoRight); // ✅ 동일 LED

    BarcodeParams p{};
    p.axis = 7;
    p.targetBarcodeAbs = 2776;
    p.mainVel = 10000.0; p.mainAcc = 1000.0; p.mainDec = 1000.0;
    p.corrVel = 1000.0; p.corrAcc = 300.0; p.corrDec = 300.0;
    p.deadband = 1;
    p.gear = 4.4248; p.wheelDia = 115.0; p.motorCpr = 10000.0; p.bcMmPerCnt = 1.07;
    g_bcRunner.Start(p, TaskId::GoRight);
}
void Forward();
void Backward();
void Forking() {
    if (!g_commStarted) { SetTaskState(TaskId::Forking, TaskState::Failed); return; }

    SetTaskState(TaskId::Forking, TaskState::Running);
    WriteOutputBit(38, 2, true, true);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Forking); // ✅ 동일 LED

    int ax = 1;
    long long tgt = 65000;
    StartMoveWithApproach(ax, tgt, TaskId::Forking,
        50000.0, 1000.0, 1000.0,
        10.0, 2.0, 30000,
        2000, { 1000.0, 80.0, 10.0 });
}
void Unforking() {
    if (!g_commStarted) { SetTaskState(TaskId::Unforking, TaskState::Failed); return; }

    SetTaskState(TaskId::Unforking, TaskState::Running);
    WriteOutputBit(38, 1, true, true);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Unforking); // ✅ 동일 LED

    int ax = 1;
    long long tgt = -1000; // NOTE: Unforking은 타겟 도달이 아니라 AX1 리밋으로 Stop될 때 Done
    StartMoveWithApproach(ax, tgt, TaskId::Unforking,
        50000.0, 1000.0, 1000.0,
        10.0, 2.0, 30000,
        4000, { 1000.0, 80.0, 10.0 });
}
void Forking2() {
    if (!g_commStarted) { SetTaskState(TaskId::Forking2, TaskState::Failed); return; }

    SetTaskState(TaskId::Forking2, TaskState::Running);
    WriteOutputBit(38, 2, true, true);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Forking2); // ✅ 동일 LED

    int ax = 1;
    long long tgt = -65000;
    StartMoveWithApproach(ax, tgt, TaskId::Forking2,
        50000.0, 1000.0, 1000.0,
        10.0, 2.0, 30000,
        2000, { 1000.0, 80.0, 10.0 });
}
void Unforking2() {
    if (!g_commStarted) { SetTaskState(TaskId::Unforking2, TaskState::Failed); return; }

    SetTaskState(TaskId::Unforking2, TaskState::Running);
    WriteOutputBit(38, 1, true, true);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Unforking2); // ✅ 동일 LED

    int ax = 1;
    long long tgt = +1000; // NOTE: Unforking은 타겟 도달이 아니라 AX1 리밋으로 Stop될 때 Done
    StartMoveWithApproach(ax, tgt, TaskId::Unforking2,
        50000.0, 1000.0, 1000.0,
        10.0, 2.0, 30000,
        4000, { 1000.0, 80.0, 10.0 });
}

// Down 버튼(임시): 기존 WorkDown(45000)으로 내려감
void Down() {
    if (!g_commStarted) { SetTaskState(TaskId::Down, TaskState::Failed); return; }

    SetTaskState(TaskId::Down, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Down); // ✅ 동일 LED

    int ax = 2;
    long long tgt = -80000;
    StartMoveWithApproach(ax, tgt, TaskId::Down,
        20000.0, 500.0, 500.0,
        10.0, 2.0, 30000,
        1000, { 1000.0, 80.0, 10.0 });
}
void Up() {
    if (!g_commStarted) { SetTaskState(TaskId::Up, TaskState::Failed); return; }

    SetTaskState(TaskId::Up, TaskState::Running);
    LedStartBlinkForTask(g_hDemoWnd, TaskId::Up); // ✅ 동일 LED

    int ax = 2;
    long long tgt = 80000;
    StartMoveWithApproach(ax, tgt, TaskId::Up,
        20000.0, 500.0, 500.0,
        10.0, 2.0, 30000,
        1000, { 1000.0, 80.0, 10.0 });
}

struct PosFlags {
    bool IsUp = false;
    bool IsDown = false;

    bool isCenter = false;
    bool isLeft = false;
    bool isRight = false;
    bool isLeftForward = false;
    bool isRightForward = false;
};

static inline PosFlags ReadPosFlags()
{
    PosFlags f{};

    bool ax2_up = ReadInputBitRaw(AX2_UPLIMIT_ADDR, AX2_UPLIMIT_BIT, AX2_UPLIMIT_ACTIVE_HIGH);
    bool ax2_dn = ReadInputBitRaw(AX2_DOWNLIMIT_ADDR, AX2_DOWNLIMIT_BIT, AX2_DOWNLIMIT_ACTIVE_HIGH);
    bool ax3_up = ReadInputBitRaw(AX3_UPLIMIT_ADDR, AX3_UPLIMIT_BIT, AX3_UPLIMIT_ACTIVE_HIGH);
    bool ax3_dn = ReadInputBitRaw(AX3_DOWNLIMIT_ADDR, AX3_DOWNLIMIT_BIT, AX3_DOWNLIMIT_ACTIVE_HIGH);

    bool l1 = ReadInputBitRaw(AX5_LIMIT1_ADDR, AX5_LIMIT1_BIT, AX5_LIMIT1_ACTIVE_HIGH);
    bool l2 = ReadInputBitRaw(AX5_LIMIT2_ADDR, AX5_LIMIT2_BIT, AX5_LIMIT2_ACTIVE_HIGH);
    bool l3 = ReadInputBitRaw(AX5_LIMIT3_ADDR, AX5_LIMIT3_BIT, AX5_LIMIT3_ACTIVE_HIGH);
    bool l4 = ReadInputBitRaw(AX5_LIMIT4_ADDR, AX5_LIMIT4_BIT, AX5_LIMIT4_ACTIVE_HIGH);

    f.IsUp = (ax2_up && ax3_up);
    f.IsDown = (ax2_dn && ax3_dn);

    // ✅ 너가 확정한 조건(바코드 판정 함수는 () 붙임)
    f.isCenter = (!l1 && !l2 && !l3 && !l4) && IsAxisWorkstation();
    f.isLeft = (!l1 && l2 && l3 && l4) && IsAxisLeft();
    f.isRight = (l1 && !l2 && l3 && l4) && IsAxisRight();
    f.isLeftForward = (l1 && l2 && !l3 && l4);
    f.isRightForward = (l1 && l2 && l3 && !l4);

    return f;
}

void GoOne()
{
    if (!g_commStarted) { SetTaskState(TaskId::GoOne, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::GoOne].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::GoOne, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        PosFlags f = ReadPosFlags();

        if (f.isCenter) {
            if (ok) { GoLeft();   ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
            if (ok) { Down();     ok = WaitTaskFinished(TaskId::Down, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { Forward();  (void)WaitTaskFinished(TaskId::Forward, 60000); }
        }
        else if (f.isLeft) {
            if (f.IsDown) {
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else if (f.IsUp) {
                if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else ok = false;
        }
        else if (f.isRight) {
            if (f.IsUp) {
                if (ok) { GoLeft();  ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
                if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else if (f.IsDown) {
                if (ok) { Up();      ok = WaitTaskFinished(TaskId::Up, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { GoLeft();  ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
                if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else ok = false;
        }
        else if (f.isLeftForward) {
            // 이미 목표위치: do nothing
        }
        else if (f.isRightForward) {
            if (ok) { Backward(); ok = WaitTaskFinished(TaskId::Backward, 60000); }
            if (ok) { Up();       ok = WaitTaskFinished(TaskId::Up, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { GoLeft();   ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
            if (ok) { Down();     ok = WaitTaskFinished(TaskId::Down, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { Forward();  (void)WaitTaskFinished(TaskId::Forward, 60000); }
        }
        else ok = false;

        SetTaskState(TaskId::GoOne, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}
void GoTwo()
{
    if (!g_commStarted) { SetTaskState(TaskId::GoTwo, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::GoTwo].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::GoTwo, TaskState::Running);

    std::thread([]() {
        bool ok = true;
        PosFlags f = ReadPosFlags();

        if (f.isCenter) {
            if (ok) { GoLeft(); ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
        }
        else if (f.isLeft) {
            // do nothing
        }
        else if (f.isRight) {
            if (f.IsUp) {
                if (ok) { GoLeft(); ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
            }
            else if (f.IsDown) {
                if (ok) { Up();     ok = WaitTaskFinished(TaskId::Up, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { GoLeft(); ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
            }
            else ok = false;
        }
        else if (f.isLeftForward) {
            if (ok) { Backward(); ok = WaitTaskFinished(TaskId::Backward, 60000); }
        }
        else if (f.isRightForward) {
            if (ok) { Backward(); ok = WaitTaskFinished(TaskId::Backward, 60000); }
            if (ok) { Up();       ok = WaitTaskFinished(TaskId::Up, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { GoLeft();   ok = WaitTaskFinished(TaskId::GoLeft, 30000); }
        }
        else ok = false;

        SetTaskState(TaskId::GoTwo, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}
void GoThree()
{
    if (!g_commStarted) { SetTaskState(TaskId::GoThree, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::GoThree].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::GoThree, TaskState::Running);

    std::thread([]() {
        bool ok = true;
        PosFlags f = ReadPosFlags();

        if (f.isCenter) {
            // do nothing
        }
        else if (f.isLeft || f.isRight) {
            if (f.IsUp) {
                if (ok) { GoWorkstation(); ok = WaitTaskFinished(TaskId::GoWorkstation, 30000); }
            }
            else if (f.IsDown) {
                if (ok) { Up();            ok = WaitTaskFinished(TaskId::Up, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { GoWorkstation(); ok = WaitTaskFinished(TaskId::GoWorkstation, 30000); }
            }
            else ok = false;
        }
        else if (f.isLeftForward || f.isRightForward) {
            if (ok) { Backward();      ok = WaitTaskFinished(TaskId::Backward, 60000); }
            if (ok) { Up();            ok = WaitTaskFinished(TaskId::Up, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { GoWorkstation(); ok = WaitTaskFinished(TaskId::GoWorkstation, 30000); }
        }
        else ok = false;

        SetTaskState(TaskId::GoThree, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}
void GoFour()
{
    if (!g_commStarted) { SetTaskState(TaskId::GoFour, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::GoFour].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::GoFour, TaskState::Running);

    std::thread([]() {
        bool ok = true;
        PosFlags f = ReadPosFlags();

        if (f.isCenter) {
            if (ok) { GoRight(); ok = WaitTaskFinished(TaskId::GoRight, 30000); }
        }
        else if (f.isLeft) {
            if (f.IsUp) {
                if (ok) { GoRight(); ok = WaitTaskFinished(TaskId::GoRight, 30000); }
            }
            else if (f.IsDown) {
                if (ok) { Up();      ok = WaitTaskFinished(TaskId::Up, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { GoRight(); ok = WaitTaskFinished(TaskId::GoRight, 30000); }
            }
            else ok = false;
        }
        else if (f.isRight) {
            // do nothing
        }
        else if (f.isLeftForward) {
            if (ok) { Backward(); ok = WaitTaskFinished(TaskId::Backward, 60000); }
            if (ok) { Up();       ok = WaitTaskFinished(TaskId::Up, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { GoRight();  ok = WaitTaskFinished(TaskId::GoRight, 30000); }
        }
        else if (f.isRightForward) {
            if (ok) { Backward(); ok = WaitTaskFinished(TaskId::Backward, 60000); }
        }
        else ok = false;

        SetTaskState(TaskId::GoFour, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}
void GoFive()
{
    if (!g_commStarted) { SetTaskState(TaskId::GoFive, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::GoFive].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::GoFive, TaskState::Running);

    std::thread([]() {
        bool ok = true;
        PosFlags f = ReadPosFlags();

        if (f.isCenter) {
            if (ok) { GoRight(); ok = WaitTaskFinished(TaskId::GoRight, 30000); }
            if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
        }
        else if (f.isLeft) {
            if (f.IsUp) {
                if (ok) { GoRight(); ok = WaitTaskFinished(TaskId::GoRight, 30000); }
                if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else if (f.IsDown) {
                if (ok) { Up();      ok = WaitTaskFinished(TaskId::Up, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { GoRight(); ok = WaitTaskFinished(TaskId::GoRight, 30000); }
                if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else ok = false;
        }
        else if (f.isRight) {
            if (f.IsUp) {
                if (ok) { Down();    ok = WaitTaskFinished(TaskId::Down, 30000); }
                if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else if (f.IsDown) {
                if (ok) { Forward(); (void)WaitTaskFinished(TaskId::Forward, 60000); }
            }
            else ok = false;
        }
        else if (f.isLeftForward) {
            if (ok) { Backward(); ok = WaitTaskFinished(TaskId::Backward, 60000); }
            if (ok) { Up();       ok = WaitTaskFinished(TaskId::Up, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { GoRight();  ok = WaitTaskFinished(TaskId::GoRight, 30000); }
            if (ok) { Down();     ok = WaitTaskFinished(TaskId::Down, 30000); }
            if (ok) { Sleep(500); }   // ✅ Down 끝난 뒤 0.5초 딜레이
            if (ok) { Forward();  (void)WaitTaskFinished(TaskId::Forward, 60000); }
        }
        else if (f.isRightForward) {
            // do nothing
        }
        else ok = false;

        SetTaskState(TaskId::GoFive, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}


// Load 시퀀스: (요청 변경) GoThree → Forking → Close → Unforking → GoFive
void StartDemoLoad()
{
    if (!g_commStarted) { SetTaskState(TaskId::DemoLoad, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::DemoLoad].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::DemoLoad, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        // 0) GoThree (Workstation 위치로 이동)
        if (ok) {
            GoThree();
            ok = WaitTaskFinished(TaskId::GoThree, 60000);   // 시간은 필요 시 조정
        }

        Sleep(500);

        // 1) Forking
        if (ok) {
            Forking();                                      // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::Forking, 30000);   // ✅ TaskId 존재/시간 조정
        }

        Sleep(300);

        // 2) Close (박스 잡기)
        if (ok) {
            Close();                                        // ✅ DoClose_Compat(...)를 쓰는 구조면 그걸로 교체
            ok = WaitTaskFinished(TaskId::Close, 60000);

            // TaskId::Close를 별도로 관리한다면 아래를 사용
            // ok = WaitTaskFinished(TaskId::Close, 10000);
        }

        Sleep(300);

        // 3) Unforking
        if (ok) {
            Unforking();                                    // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::Unforking, 30000);
        }

        Sleep(500);

        // 3) Unforking
        if (ok) {
            HoistDown();                                    // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::HoistDown, 30000);
        }

        Sleep(500);

        // 3) Unforking
        if (ok) {
            HoistUp();                                    // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::HoistUp, 30000);
        }

        Sleep(500);

        // 4) GoFive
        if (ok) {
            GoOne();
            ok = WaitTaskFinished(TaskId::GoOne, 90000);    // GoRight+Down+Forward 포함이라 넉넉히
        }

        SetTaskState(TaskId::DemoLoad, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}


// Unload 시퀀스: Workstation → Conveyor
void StartDemoUnload()
{
    if (!g_commStarted) { SetTaskState(TaskId::DemoUnload, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::DemoUnload].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::DemoUnload, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        // 0) GoThree (Workstation 위치로 이동)
        if (ok) {
            GoThree();
            ok = WaitTaskFinished(TaskId::GoThree, 60000);   // 시간은 필요 시 조정
        }

        Sleep(500);

        // 1) Forking
        if (ok) {
            Forking();                                      // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::Forking, 30000);   // ✅ TaskId 존재/시간 조정
        }

        Sleep(300);

        // 2) Close (박스 잡기)
        if (ok) {
            Open();                                        // ✅ DoClose_Compat(...)를 쓰는 구조면 그걸로 교체
            ok = WaitTaskFinished(TaskId::Open, 60000);

            // TaskId::Close를 별도로 관리한다면 아래를 사용
            // ok = WaitTaskFinished(TaskId::Close, 10000);
        }

        Sleep(300);

        // 3) Unforking
        if (ok) {
            Unforking();                                    // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::Unforking, 30000);
        }

        Sleep(500);

        // 3) Unforking
        if (ok) {
            HoistDown();                                    // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::HoistDown, 30000);
        }

        Sleep(500);

        // 3) Unforking
        if (ok) {
            HoistUp();                                    // ✅ 프로젝트 함수명에 맞게
            ok = WaitTaskFinished(TaskId::HoistUp, 30000);
        }

        Sleep(500);

        // 4) GoFive
        if (ok) {
            GoFive();
            ok = WaitTaskFinished(TaskId::GoFive, 90000);    // GoRight+Down+Forward 포함이라 넉넉히
        }

        SetTaskState(TaskId::DemoUnload, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}

// 전체 Demo: (임시 구현) DemoLoad → DemoUnload 순서로 실행
// 전체 Demo: DemoLoad → DemoUnload 순서로 반복 실행
void StartAllDemo()
{
    if (!g_commStarted) { SetTaskState(TaskId::All_Demo, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::All_Demo].state.load() == TaskState::Running) return;

    SetTaskState(TaskId::All_Demo, TaskState::Running);

    std::thread([]() {
        bool ok = true;
        int repeatCount = 3;  // 반복 횟수 설정 (원하는 횟수로 조정)

        for (int i = 0; i < repeatCount && ok; ++i) {
            // 0) GoThree (Workstation 위치로 이동)
            if (ok) {
                StartDemoLoad();
                // DemoLoad가 완료될 때까지 대기 (TaskState::Done 확인)
                while (g_taskStatus[(int)TaskId::DemoLoad].state.load() != TaskState::Done) {
                    Sleep(100);  // 잠시 대기
                }
            }

            Sleep(1000);  // 대기 시간

            // 1) Forking (DemoLoad가 완료된 후 실행)
            if (ok) {
                StartDemoUnload();  // 프로젝트 함수명에 맞게
                // DemoUnload가 완료될 때까지 대기 (TaskState::Done 확인)
                while (g_taskStatus[(int)TaskId::DemoUnload].state.load() != TaskState::Done) {
                    Sleep(100);  // 잠시 대기
                }
            }

            Sleep(1000);  // 대기 시간
        }

        SetTaskState(TaskId::All_Demo, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}

// ======================================================
// Added limit handling logic (AX0/AX1/AX2/AX3/AX5)
//  - Preserves existing Axis2 sensor/home logic on axis index 4
// ======================================================
static inline void SetStaticText(HWND h, const std::wstring& s) {
    if (h) SetWindowTextW(h, s.c_str());
}
static void UpdateAxLimitPanel(
    bool ax0, bool ax1_1, bool ax1_2,
    bool ax2_up, bool ax2_dn,
    bool ax3_up, bool ax3_dn,
    bool ax5_1, bool ax5_2, bool ax5_3, bool ax5_4)
{
    if (!g_hDemoWnd) return;

    SetStaticText(g_hAxLimitStatics[0], std::wstring(L"AX0 LIMIT : ") + (ax0 ? L"ON" : L"OFF"));
    SetStaticText(g_hAxLimitStatics[1], std::wstring(L"AX1 L1/L2 : ") + (ax1_1 ? L"1" : L"0") + L"/" + (ax1_2 ? L"1" : L"0"));
    SetStaticText(g_hAxLimitStatics[2], std::wstring(L"AX2 UP/DN : ") + (ax2_up ? L"1" : L"0") + L"/" + (ax2_dn ? L"1" : L"0"));
    SetStaticText(g_hAxLimitStatics[3], std::wstring(L"AX3 UP/DN : ") + (ax3_up ? L"1" : L"0") + L"/" + (ax3_dn ? L"1" : L"0"));
    SetStaticText(g_hAxLimitStatics[4], L"AX4 : (kept - existing logic)"); // axis index 4 is used by existing Axis2 limit/home logic
    SetStaticText(g_hAxLimitStatics[5], std::wstring(L"AX5 L1..L4 : ") + (ax5_1 ? L"1" : L"0") + (ax5_2 ? L"1" : L"0") + (ax5_3 ? L"1" : L"0") + (ax5_4 ? L"1" : L"0") + std::wstring(L"  ") + g_ax5UiExtra);
}

// ---------- AX0 ----------
static bool  g_ax0LimitPrev = false;
static DWORD g_ax0LimitOnTick = 0;
static bool  g_ax0HomeIssued = false;

// ---------- AX1 ----------
static bool g_ax1StopHomeIssued = false;
static bool g_ax1HomePending = false;
static DWORD g_ax1HomeRequestTick = 0;
static const DWORD AX1_HOME_DELAY_MS = 1500; // Stop 후 Home까지 딜레이


// ---------- AX2/AX3 (just stop) ----------
static bool g_ax2StopIssued = false;
static bool g_ax3StopIssued = false;

// ---------- AX5 state machine ----------
enum class Ax5Side { Unknown, LeftForward, LeftBackward, RightForward, RightBackward, Center };
enum class Ax5Dir { None, Plus, Minus };

static Ax5Side g_ax5Side = Ax5Side::Unknown;
static Ax5Side g_ax5ExpectedSide = Ax5Side::Unknown; // Expected destination side for current move
static Ax5Side g_ax5SelectedSide = Ax5Side::Unknown; // Side chosen by last forward() for backward()
static Ax5Side g_ax5LastNonCenterSide = Ax5Side::Unknown; // Remember last detected Left/Right when idle
static Ax5Dir  g_ax5Dir = Ax5Dir::None;

static bool g_ax5PrevL1 = false, g_ax5PrevL2 = false, g_ax5PrevL3 = false, g_ax5PrevL4 = false;
static int  g_ax5PlusRiseCount = 0; // L1 (left-side) or L2 (right-side) rising count
static bool g_ax5DecelIssued = false;
static bool g_ax5StopIssued = false;

// AX5: 어떤 Task가 현재 AX5를 구동 중인지 표시 (Forward 완료 처리용)
static TaskId g_ax5ActiveTask = TaskId::COUNT;


// For "- direction from center" side inference
static bool g_ax5FromCenter = false;
// AX5 Forward(+) 전용 전이 플래그
static bool g_ax5Fwd_L4SawOff = false;   // Left에서: L4 ON→OFF를 한번 봤는지
static bool g_ax5Fwd_L1SawOff = false;   // Left에서: L1이 OFF로 떨어진 적이 있는지(정지 조건용)

static bool g_ax5Fwd_L3SawOff = false;   // Right에서: L3 ON→OFF를 한번 봤는지
static bool g_ax5Fwd_L2SawOff = false;   // Right에서: L2가 OFF로 떨어진 적이 있는지(정지 조건용)

// AX5 Backward(-) 전용 전이 플래그
static bool g_ax5Bwd_L3SawOff = false;   // Left(시작 123)에서: L3 ON→OFF를 봤는지 (감속용)
static bool g_ax5Bwd_L4SawOff = false;   // Right(시작 124)에서: L4 ON→OFF를 봤는지 (감속용)

static bool g_ax5Bwd_L4SawOff2 = false;  // Left에서: L4가 OFF로 떨어진 적이 있는지 (정지 조건용)
static bool g_ax5Bwd_L3SawOff2 = false;  // Right에서: L3가 OFF로 떨어진 적이 있는지 (정지 조건용)

// =====================
// Common LED Blink Manager (addr=38, bit=3/4)
// =====================
static const int  LED_ADDR = 38;
static const int  LED_BIT_A = 3;
static const int  LED_BIT_B = 4;
static const DWORD LED_BLINK_MS = 500;
static const UINT  IDT_LED_BLINK = 41001; // 기존 타이머랑 안 겹치게

static bool   g_ledBlinkActive = false;
static bool   g_ledBlinkStateOn = false;
static DWORD  g_ledLastToggleTick = 0;
static TaskId g_ledOwnerTask = TaskId::COUNT;

static void LedSet(bool on) {
    WriteOutputBit(LED_ADDR, LED_BIT_A, on, true);
    WriteOutputBit(LED_ADDR, LED_BIT_B, on, true);
}

static void LedCancel(HWND hWnd) {
    g_ledBlinkActive = false;
    g_ledOwnerTask = TaskId::COUNT;
    KillTimer(hWnd, IDT_LED_BLINK);
    LedSet(false); // 취소/정지 시 OFF (원하면 true로 바꿔도 됨)
}

static void LedHoldOn(HWND hWnd) {
    g_ledBlinkActive = false;
    g_ledOwnerTask = TaskId::COUNT;
    KillTimer(hWnd, IDT_LED_BLINK);
    LedSet(true);  // 완료 시 ON 고정
}

static void LedStartBlinkForTask(HWND hWnd, TaskId tid) {
    // 다른 작업 LED가 돌고 있으면 끊고 새로 시작
    KillTimer(hWnd, IDT_LED_BLINK);

    g_ledBlinkActive = true;
    g_ledOwnerTask = tid;
    g_ledBlinkStateOn = false;
    g_ledLastToggleTick = GetTickCount();

    LedSet(false);
    SetTimer(hWnd, IDT_LED_BLINK, 50, nullptr); // 50ms 폴링, 500ms 토글
}

// Called by SetTaskState() (hook) to finish the blink cycle.
void LedOnTaskStateChanged(HWND hWnd, TaskId tid, TaskState st)
{
    if (!hWnd) return;
    if (g_ledOwnerTask != tid) return;

    if (st == TaskState::Done) {
        // 요구사항: 동작 완료 후 계속 ON 유지
        LedHoldOn(hWnd);
    }
    else if (st == TaskState::Failed || st == TaskState::Stopped) {
        // 실패/중단은 깜빡임 종료 (OFF)
        LedCancel(hWnd);
    }
}

// AX5 UI/status
static bool g_ax5LastCheckOk = false;
static std::wstring g_ax5LastCheckName = L"";
static Ax5Dir GetAxisDirFromStatus(int axis) {
    CoreMotionStatus st{}; g_cm.GetStatus(&st);
    // Use actualVelocity so direction detection works for StartPos-based jog/motion.
    // velocityCmd can be near 0 in position profiles on some WMX3 setups.
    const double v = st.axesStatus[axis].actualVelocity; // rpm
    const double th = 1.0; // rpm threshold
    if (v > th) return Ax5Dir::Plus;
    if (v < -th) return Ax5Dir::Minus;
    return Ax5Dir::None;
}

static void Ax5IssueDecel(int dirSign) {
    if (g_ax5DecelIssued) return;
    if (!g_commStarted) return;
    if (!EnsureServoOn(5) || !EnsurePosModeNoStop(5)) return;

    CoreMotionStatus st{}; g_cm.GetStatus(&st);
    long long cur = (long long)st.axesStatus[5].actualPos;

    Motion::PosCommand pc{};
    pc.axis = 5;
    pc.target = cur + (long long)dirSign * AX5_DECEL_LOOKAHEAD_PULSE;
    pc.profile.type = ProfileType::SCurve;
    pc.profile.velocity = (int)std::lround(AX5_DECEL_VEL_PPS);
    pc.profile.acc = TimeMsToAcc(pc.profile.velocity, AX5_DECEL_ACC_MS);
    pc.profile.dec = TimeMsToAcc(pc.profile.velocity, AX5_DECEL_DEC_MS);
    g_cm.motion->StartPos(&pc);

    g_ax5DecelIssued = true;
}

static void Ax5ResetState() {
    g_ax5Side = Ax5Side::Unknown;
    g_ax5Dir = Ax5Dir::None;
    g_ax5PlusRiseCount = 0;
    g_ax5DecelIssued = false;
    g_ax5StopIssued = false;
    g_ax5FromCenter = false;
    // AX5 Forward(+) 전용 전이 플래그
    g_ax5Fwd_L4SawOff = false;   // Left에서: L4 ON→OFF를 한번 봤는지
    g_ax5Fwd_L1SawOff = false;   // Left에서: L1이 OFF로 떨어진 적이 있는지(정지 조건용)

    g_ax5Fwd_L3SawOff = false;   // Right에서: L3 ON→OFF를 한번 봤는지
    g_ax5Fwd_L2SawOff = false;   // Right에서: L2가 OFF로 떨어진 적이 있는지(정지 조건용)

    // AX5 Backward(-) 전용 전이 플래그
    g_ax5Bwd_L3SawOff = false;   // Left(시작 123)에서: L3 ON→OFF를 봤는지 (감속용)
    g_ax5Bwd_L4SawOff = false;   // Right(시작 124)에서: L4 ON→OFF를 봤는지 (감속용)

    g_ax5Bwd_L4SawOff2 = false;  // Left에서: L4가 OFF로 떨어진 적이 있는지 (정지 조건용)
    g_ax5Bwd_L3SawOff2 = false;  // Right에서: L3가 OFF로 떨어진 적이 있는지 (정지 조건용)
}

static HWND g_hAx5StopFlagsStatic = nullptr;
static std::wstring g_ax5UiStopFlags;   // Demo 왼쪽 UI에 표시할 Stop 조건 상태 4줄


static const wchar_t* OnOff(bool v) { return v ? L"ON" : L"OFF"; }

static void BuildAx5StopFlagsUi(bool l1, bool l2, bool l3, bool l4)
{
    bool uiFwdLeftPat = (l1 && l2 && !l3 && l4); // 1o 2o 3x 4o
    bool uiFwdRightPat = (l1 && l2 && l3 && !l4); // 1o 2o 3o 4x
    bool uiBwdLeftPat = (!l1 && l2 && l3 && l4); // 1x 2o 3o 4o
    bool uiBwdRightPat = (l1 && !l2 && l3 && l4); // 1o 2x 3o 4o

    // 참고로 StopIssued 상태도 같이 보고 싶으면 한 줄 추가로 넣어도 됨.
    // 여기선 사용자가 원하는 "계속 실시간 표시"에 집중해서 4줄만 유지.
    g_ax5UiStopFlags =
        std::wstring(L"Forward left  : ") + OnOff(uiFwdLeftPat) + L"\r\n" +
        std::wstring(L"Forward right : ") + OnOff(uiFwdRightPat) + L"\r\n" +
        std::wstring(L"Backward left : ") + OnOff(uiBwdLeftPat) + L"\r\n" +
        std::wstring(L"Backward right: ") + OnOff(uiBwdRightPat) + L"\r\n" + L"\r\n" +
        std::wstring(L"AX0 : 폭조절") + L"\r\n" +
        std::wstring(L"AX1 : 포킹암") + L"\r\n" +
        std::wstring(L"AX2 : 사이드 업다운") + L"\r\n" +
        std::wstring(L"AX3 : 사이드 업다운") + L"\r\n" +
        std::wstring(L"AX4 : 호이스트") + L"\r\n" +
        std::wstring(L"AX5 : 사이드 주행") + L"\r\n" +
        std::wstring(L"AX6 : 사이드 주행") + L"\r\n" +
        std::wstring(L"AX7 : 메인 주행") + L"\r\n" +
        std::wstring(L"AX8 : 메인 주행");


    // ✅ 실제 Static 컨트롤 텍스트를 즉시 갱신 (핵심)
    if (g_hAx5StopFlagsStatic) {
        SetWindowTextW(g_hAx5StopFlagsStatic, g_ax5UiStopFlags.c_str());
    }
}



// ---------- AX5 forward/backward helpers ----------
static Ax5Side DetectAx5SideFromLimits(bool l1, bool l2, bool l3, bool l4)
{
    if (l1 && l2 && l3 && l4) return Ax5Side::Center;
    if (!l1 && l2 && l3 && l4) return Ax5Side::LeftBackward;
    if (l1 && !l2 && l3 && l4) return Ax5Side::RightBackward;
    if (l1 && l2 && !l3 && l4) return Ax5Side::LeftForward;  // Forward 중 Left 도착 예상
    if (l1 && l2 && l3 && !l4) return Ax5Side::RightForward;  // Forward 중 Left 도착 예상
    return Ax5Side::Unknown;
}

static void ReadAx5Limits(bool& l1, bool& l2, bool& l3, bool& l4)
{
    l1 = ReadInputBitRaw(AX5_LIMIT1_ADDR, AX5_LIMIT1_BIT, AX5_LIMIT1_ACTIVE_HIGH);
    l2 = ReadInputBitRaw(AX5_LIMIT2_ADDR, AX5_LIMIT2_BIT, AX5_LIMIT2_ACTIVE_HIGH);
    l3 = ReadInputBitRaw(AX5_LIMIT3_ADDR, AX5_LIMIT3_BIT, AX5_LIMIT3_ACTIVE_HIGH);
    l4 = ReadInputBitRaw(AX5_LIMIT4_ADDR, AX5_LIMIT4_BIT, AX5_LIMIT4_ACTIVE_HIGH);
}

static void SyncAx5PrevLimitsToCurrent()
{
    bool l1, l2, l3, l4;
    ReadAx5Limits(l1, l2, l3, l4);
    g_ax5PrevL1 = l1; g_ax5PrevL2 = l2; g_ax5PrevL3 = l3; g_ax5PrevL4 = l4;
}

// Travel settings for AX5 "search move" (state machine will decel/stop by limits)
static const long long AX5_FORWARD_TRAVEL_PULSE = 8000000;  // + direction long move
static const long long AX5_BACKWARD_TRAVEL_PULSE = 8000000; // - direction long move
static const double    AX5_CRUISE_VEL_PPS = 4600.0;
static const double    AX5_CRUISE_ACC_MS = 1000.0;
static const double    AX5_CRUISE_DEC_MS = 200.0;

static void StartAx5LongMove(int dirSign)
{
    if (!g_commStarted) return;
    if (!EnsureServoOn(5) || !EnsurePosModeNoStop(5)) return;

    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    long long cur = (long long)st.axesStatus[5].actualPos;
    long long tgt = cur + (dirSign > 0 ? AX5_FORWARD_TRAVEL_PULSE : -AX5_BACKWARD_TRAVEL_PULSE);

    StartAbsMoveWithProfile(5, tgt, AX5_CRUISE_VEL_PPS, AX5_CRUISE_ACC_MS, AX5_CRUISE_DEC_MS);
}

// ===== Public-style motion entry points =====
// forward(): AX5 +방향 동작. 시작 시 Left/Right 패턴을 보고 g_ax5ExpectedSide 설정 후 긴 +이동을 시작.
// backward(): AX5 -방향 동작. 시작은 Center(1,2,3,4 ON)에서, 직전(또는 기대) Left/Right를 기준으로 -이동을 시작.
void Forward()
{
    WriteOutputBit(38, 2, true, true); //전진 led on

    // Task 시작 표시
    SetTaskState(TaskId::Forward, TaskState::Running);
    g_ax5ActiveTask = TaskId::Forward;

    bool l1, l2, l3, l4;
    ReadAx5Limits(l1, l2, l3, l4);

    // Left = 2,3,4 ON / Right = 1,3,4 ON
    bool isLeft = (!l1 && l2 && l3 && l4);
    bool isRight = (l1 && !l2 && l3 && l4);

    if (!isLeft && !isRight) {
        g_ax5UiExtra = L"Forward NG: invalid start sensors";
        SetTaskState(TaskId::Forward, TaskState::Failed);
        g_ax5ActiveTask = TaskId::COUNT;
        return;
    }

    // ===== Forward 시작 초기화(중요) =====
    g_ax5Side = isLeft ? Ax5Side::LeftBackward : Ax5Side::RightBackward;

    g_ax5DecelIssued = false;
    g_ax5StopIssued = false;

    // ===== 전용 플래그 리셋 (핵심) =====
    g_ax5Fwd_L4SawOff = false;
    g_ax5Fwd_L1SawOff = false;
    g_ax5Fwd_L3SawOff = false;
    g_ax5Fwd_L2SawOff = false;


    // 체크 표시 초기화
    g_ax5LastCheckName.clear();
    g_ax5LastCheckOk = false;

    // UI
    g_ax5UiExtra = isLeft ? L"Forward START (Left)" : L"Forward START (Right)";

    // 이전 센서 상태를 현재로 동기화 (필수: 첫 tick에서 Rise/Fall 튀는 것 방지)
    SyncAx5PrevLimitsToCurrent();

    Sleep(1000);

    // + 방향 장거리 이동 시작 (기존 함수 그대로 사용)
    StartAx5LongMove(+1);
}

void Backward()
{
    WriteOutputBit(38, 1, true, true); //후진 led on

    // Task 시작 표시
    SetTaskState(TaskId::Backward, TaskState::Running);
    g_ax5ActiveTask = TaskId::Backward;

    bool l1, l2, l3, l4;
    ReadAx5Limits(l1, l2, l3, l4);

    // 시작 위치 판별(요구사항)
    //  Left  : 1,2,3 ON
    //  Right : 1,2,4 ON
    bool isLeftStart = (l1 && l2 && !l3 && l4);   // (l4는 상관없음)
    bool isRightStart = (l1 && l2 && l3 && !l4);   // (l3는 상관없음)

    if (!isLeftStart && !isRightStart) {
        g_ax5UiExtra = L"Backward NG: start pattern not (123 or 124)";
        SetTaskState(TaskId::Backward, TaskState::Failed);
        g_ax5ActiveTask = TaskId::COUNT;
        return;
    }

    // 시작 Side 설정
    g_ax5Side = isLeftStart ? Ax5Side::LeftForward : Ax5Side::RightForward;

    // 공통 초기화
    g_ax5DecelIssued = false;
    g_ax5StopIssued = false;

    g_ax5LastCheckName.clear();
    g_ax5LastCheckOk = false;

    // Backward 전용 플래그 리셋(핵심)
    g_ax5Bwd_L3SawOff = false;
    g_ax5Bwd_L4SawOff = false;
    g_ax5Bwd_L4SawOff2 = false;
    g_ax5Bwd_L3SawOff2 = false;

    g_ax5UiExtra = isLeftStart ? L"Backward START (Left=123)" : L"Backward START (Right=124)";

    // Prev 동기화 (첫 tick 에지 튐 방지)
    SyncAx5PrevLimitsToCurrent();

    Sleep(1000);

    // - 방향 이동 시작
    StartAx5LongMove(-1);
}


// Call this periodically (timer)
static void AxLimitSensorTimerProc(HWND)
{
    if (!g_commStarted) {
        return;
    }

    // Read all requested inputs (active-high mapping)
    bool ax0 = ReadInputBitRaw(AX0_LIMIT_ADDR, AX0_LIMIT_BIT, AX0_LIMIT_ACTIVE_HIGH);

    bool ax1_1 = ReadInputBitRaw(AX1_LIMIT1_ADDR, AX1_LIMIT1_BIT, AX1_LIMIT1_ACTIVE_HIGH);
    bool ax1_2 = ReadInputBitRaw(AX1_LIMIT2_ADDR, AX1_LIMIT2_BIT, AX1_LIMIT2_ACTIVE_HIGH);

    bool ax2_up = ReadInputBitRaw(AX2_UPLIMIT_ADDR, AX2_UPLIMIT_BIT, AX2_UPLIMIT_ACTIVE_HIGH);
    bool ax2_dn = ReadInputBitRaw(AX2_DOWNLIMIT_ADDR, AX2_DOWNLIMIT_BIT, AX2_DOWNLIMIT_ACTIVE_HIGH);

    bool ax3_up = ReadInputBitRaw(AX3_UPLIMIT_ADDR, AX3_UPLIMIT_BIT, AX3_UPLIMIT_ACTIVE_HIGH);
    bool ax3_dn = ReadInputBitRaw(AX3_DOWNLIMIT_ADDR, AX3_DOWNLIMIT_BIT, AX3_DOWNLIMIT_ACTIVE_HIGH);

    bool l1 = ReadInputBitRaw(AX5_LIMIT1_ADDR, AX5_LIMIT1_BIT, AX5_LIMIT1_ACTIVE_HIGH);
    bool l2 = ReadInputBitRaw(AX5_LIMIT2_ADDR, AX5_LIMIT2_BIT, AX5_LIMIT2_ACTIVE_HIGH);
    bool l3 = ReadInputBitRaw(AX5_LIMIT3_ADDR, AX5_LIMIT3_BIT, AX5_LIMIT3_ACTIVE_HIGH);
    bool l4 = ReadInputBitRaw(AX5_LIMIT4_ADDR, AX5_LIMIT4_BIT, AX5_LIMIT4_ACTIVE_HIGH);

    UpdateAxLimitPanel(ax0, ax1_1, ax1_2, ax2_up, ax2_dn, ax3_up, ax3_dn, l1, l2, l3, l4);

    // ✅ StopFlags UI 갱신 (실시간)
    BuildAx5StopFlagsUi(l1, l2, l3, l4);

    DWORD now = GetTickCount();

    // ---------------- AX0: stop only on the FIRST ON (rising edge), allow motion while held ON; home if held ON & idle >= 1s ----------------
    {
        CoreMotionStatus st0{};
        g_cm.GetStatus(&st0);
        const double vcmd0 = st0.axesStatus[0].velocityCmd;
        const double vth = 2.0; // pps threshold to treat as moving
        const bool isMoving = (vcmd0 > vth) || (vcmd0 < -vth);
        const double vact0 = std::fabs(st0.axesStatus[0].actualVelocity);
        const bool isIdle = (vact0 <= 1.0);

        if (ax0) {
            // Stop ONLY when the limit first turns ON (rising edge).
            if (!g_ax0LimitPrev) {
                StopAxis(0);
                g_ax0LimitOnTick = now;   // start hold timer for home
                g_ax0HomeIssued = false;
            }

            // While the limit stays ON, DO NOT keep stopping.
            // Home trigger: only if the limit stays ON for >= hold time while the axis is idle.
            if (isMoving) {
                // operator is jogging / axis is moving -> don't count toward auto-home
                g_ax0LimitOnTick = now;
            }
            else {
                if (!g_ax0HomeIssued && g_ax0LimitOnTick != 0 && (now - g_ax0LimitOnTick) >= AX0_LIMIT_HOME_HOLD_MS) {
                    if (EnsureServoOn(0) && EnsurePosModeNoStop(0)) {
                        g_home.StartHome(0);
                        g_ax0HomeIssued = true;
                    }
                }
            }

            // Open: AX0 리밋센서로 Stop되어 축이 정지하면 동작 완료
            if (isIdle) {
                TaskState openSt = GetTaskState(TaskId::Open);
                if (openSt == TaskState::Running || openSt == TaskState::Stopped) {
                    SetTaskState(TaskId::Open, TaskState::Done);
                }
            }

        }
        else {
            g_ax0LimitOnTick = 0;
            g_ax0HomeIssued = false;
        }

        g_ax0LimitPrev = ax0;
    }

    // ---------------- AX1: if BOTH OFF -> stop + (delayed) home ----------------
    if (!ax1_1 && !ax1_2) {

        // 1) 조건 처음 진입 시 Stop + Home Pending
        if (!g_ax1StopHomeIssued) {
            StopAxis(1);
            g_ax1HomePending = true;
            g_ax1HomeRequestTick = GetTickCount();
            g_ax1StopHomeIssued = true;
        }

        // 2) Stop 후 일정 시간 지나면 Home 시도 (매틱 반복 호출 방지)
        if (g_ax1HomePending) {
            DWORD now = GetTickCount();
            if (now - g_ax1HomeRequestTick >= AX1_HOME_DELAY_MS) {
                if (EnsureServoOn(1) && EnsurePosModeNoStop(1)) {
                    g_home.StartHome(1);
                }
                // Home 시도는 일단 1번만 하고 pending 해제
                // (만약 실패 재시도 원하면, 홈 상태 체크해서 실패면 다시 pending=true로 올리면 됨)
                g_ax1HomePending = false;
            }
        }
        // Unforking: AX1 리밋 조건(!L1 && !L2)로 Stop되고 축이 정지하면 동작 완료
        CoreMotionStatus st1{};
        g_cm.GetStatus(&st1);
        const double v1 = std::fabs(st1.axesStatus[1].actualVelocity);
        if (v1 <= 1.0) {
            TaskState ufSt = GetTaskState(TaskId::Unforking);
            if (ufSt == TaskState::Running || ufSt == TaskState::Stopped) {
                SetTaskState(TaskId::Unforking, TaskState::Done);
            }
        }


    }
    else {
        // 조건 해제 시 리셋
        g_ax1StopHomeIssued = false;
        g_ax1HomePending = false;
    }


    // ---------------- AX2: if UP or DOWN ON -> stop ----------------
    // 요구사항: Up/Down은 리밋센서에 닿아 멈추면 완료 처리
    const bool ax2UpStop = (ax2_up && ax3_up);
    const bool ax2DnStop = (ax2_dn && ax3_dn);

    if (ax2UpStop || ax2DnStop) {
        if (!g_ax2StopIssued) {
            StopAxis(2);
            g_ax2StopIssued = true;
        }

        CoreMotionStatus st2{};
        g_cm.GetStatus(&st2);
        const double v2 = std::fabs(st2.axesStatus[2].actualVelocity);
        if (v2 <= 1.0) {
            TaskState upSt = GetTaskState(TaskId::Up);
            if (ax2UpStop && (upSt == TaskState::Running || upSt == TaskState::Stopped)) {
                SetTaskState(TaskId::Up, TaskState::Done);
            }
            TaskState dnSt = GetTaskState(TaskId::Down);
            if (ax2DnStop && (dnSt == TaskState::Running || dnSt == TaskState::Stopped)) {
                SetTaskState(TaskId::Down, TaskState::Done);
            }
        }
    }
    else {
        g_ax2StopIssued = false;
    }


    // ---------------- AX5: special behavior ----------------
    bool isCenter = (!l1 && !l2 && !l3 && !l4);
    bool isLeft = (!l1 && l2 && l3 && l4);
    bool isRight = (l1 && !l2 && l3 && l4);
    bool isLeftForward = (l1 && l2 && !l3 && l4);
    bool isRightForward = (l1 && l2 && l3 && !l4);

    bool l1Rise = (!g_ax5PrevL1 && l1);
    bool l2Rise = (!g_ax5PrevL2 && l2);
    bool l3Rise = (!g_ax5PrevL3 && l3);
    bool l4Rise = (!g_ax5PrevL4 && l4);
    bool l1Fall = (g_ax5PrevL1 && !l1);
    bool l2Fall = (g_ax5PrevL2 && !l2);

    Ax5Dir dir = GetAxisDirFromStatus(5);

    // Reset one-shot flags when direction changes
    // ✅ 중요: StopIssued는 Done 확정 전까지 리셋하면 안 됨 (Running이 계속 남는 원인)
    if (dir != g_ax5Dir) {
        g_ax5Dir = dir;
        g_ax5DecelIssued = false;

        // StopIssued는 유지 (Done 확정용)
        // g_ax5StopIssued = false;   // ❌ 삭제

        g_ax5FromCenter = isCenter;

        // 아래 플래그는 "새로운 이동 시작" 때만 리셋되는 게 이상적이지만,
        // 기존 동작 유지 위해 dir 바뀔 때도 초기화하되 StopIssued 중이면 최소만 건드린다.
        if (!g_ax5StopIssued) {
            // ===== 전용 플래그 리셋 (핵심) =====
            g_ax5Fwd_L4SawOff = false;
            g_ax5Fwd_L1SawOff = false;
            g_ax5Fwd_L3SawOff = false;
            g_ax5Fwd_L2SawOff = false;

            g_ax5Bwd_L3SawOff = false;
            g_ax5Bwd_L4SawOff = false;
            g_ax5Bwd_L4SawOff2 = false;
            g_ax5Bwd_L3SawOff2 = false;
        }
    }

    // Update side when idle (start/settled)
    if (dir == Ax5Dir::None) {
        if (isCenter) {
            g_ax5Side = Ax5Side::Center;
            g_ax5ExpectedSide = Ax5Side::Unknown;
        }
        else if (isLeft) {
            g_ax5Side = Ax5Side::LeftBackward;
            g_ax5LastNonCenterSide = Ax5Side::LeftBackward;
        }
        else if (isRight) {
            g_ax5Side = Ax5Side::RightBackward;
            g_ax5LastNonCenterSide = Ax5Side::RightBackward;
        }
        else if (isLeftForward) {
            g_ax5Side = Ax5Side::LeftForward;
            g_ax5LastNonCenterSide = Ax5Side::LeftForward;
        }
        else if (isRightForward) {
            g_ax5Side = Ax5Side::RightForward;
            g_ax5LastNonCenterSide = Ax5Side::RightForward;
        }
    }
    else {
        // If side is unknown, try to infer from current pattern during motion
        if (g_ax5Side == Ax5Side::Unknown) {
            if (isLeft)  g_ax5Side = Ax5Side::LeftBackward;
            else if (isRight) g_ax5Side = Ax5Side::RightBackward;
            else if (isCenter) g_ax5Side = Ax5Side::Center;
            else if (isLeftForward) g_ax5Side = Ax5Side::LeftForward;
            else if (isRightForward) g_ax5Side = Ax5Side::RightForward;
        }

        // ---------------- PLUS (+) ----------------
        if (dir == Ax5Dir::Plus) {

            // PLUS Left: from LeftBackward
            if (g_ax5Side == Ax5Side::LeftBackward) {

                bool decelPat = (!l1 && l2 && !l3 && l4);
                bool stopPat = (l1 && l2 && !l3 && l4);

                if (!g_ax5StopIssued) {

                    if (!g_ax5DecelIssued && decelPat) {
                        Ax5IssueDecel(+1);
                        g_ax5DecelIssued = true;
                    }

                    if (stopPat) {
                        StopAxis(5);
                        WriteOutputBit(38, 2, false, true); // 전진 led off
                        g_ax5StopIssued = true;

                        g_ax5LastCheckName = L"PLUS_L_STOP";
                        g_ax5LastCheckOk = true;
                        g_ax5Side = Ax5Side::LeftForward;
                        g_ax5UiExtra = L"Plus STOP (Left): 1o2o3x4o";
                    }
                }
            }

            // PLUS Right: from RightBackward
            else if (g_ax5Side == Ax5Side::RightBackward) {

                bool decelPat = (l1 && !l2 && l3 && !l4);
                bool stopPat = (l1 && l2 && l3 && !l4);

                if (!g_ax5StopIssued) {

                    if (!g_ax5DecelIssued && decelPat) {
                        Ax5IssueDecel(+1);
                        g_ax5DecelIssued = true;
                    }

                    if (stopPat) {
                        StopAxis(5);
                        WriteOutputBit(38, 2, false, true); // 전진 led off
                        g_ax5StopIssued = true;

                        g_ax5LastCheckName = L"PLUS_R_STOP";
                        g_ax5LastCheckOk = true;
                        g_ax5Side = Ax5Side::RightForward;
                        g_ax5UiExtra = L"Plus STOP (Right): 1o2o3o4x";
                    }
                }
            }
        }

        // ---------------- MINUS (-) ----------------
        if (dir == Ax5Dir::Minus) {

            // MINUS Left: from LeftForward
            if (g_ax5Side == Ax5Side::LeftForward) {

                bool decelPat = (!l1 && l2 && l3 && !l4);
                bool stopPat = (!l1 && l2 && l3 && l4);

                if (!g_ax5StopIssued) {

                    if (!g_ax5DecelIssued && decelPat) {
                        Ax5IssueDecel(-1);
                        g_ax5DecelIssued = true;
                    }

                    if (stopPat) {
                        StopAxis(5);
                        WriteOutputBit(38, 1, false, true); // 후진 led off
                        g_ax5StopIssued = true;

                        g_ax5LastCheckName = L"MINUS_L_STOP";
                        g_ax5LastCheckOk = true;
                        g_ax5Side = Ax5Side::LeftBackward;
                        g_ax5UiExtra = L"Minus STOP (Left): 1x2o3o4o";
                    }
                }
            }

            // MINUS Right: from RightForward
            else if (g_ax5Side == Ax5Side::RightForward) {

                bool decelPat = (l1 && !l2 && !l3 && l4);
                bool stopPat = (l1 && !l2 && l3 && l4);

                if (!g_ax5StopIssued) {

                    if (!g_ax5DecelIssued && decelPat) {
                        Ax5IssueDecel(-1);
                        g_ax5DecelIssued = true;
                    }

                    if (stopPat) {
                        StopAxis(5);
                        WriteOutputBit(38, 1, false, true); // 후진 led off
                        g_ax5StopIssued = true;

                        g_ax5LastCheckName = L"MINUS_R_STOP";
                        g_ax5LastCheckOk = true;
                        g_ax5Side = Ax5Side::RightBackward;
                        g_ax5UiExtra = L"Minus STOP (Right): 1o2x3o4o";
                    }
                }
            }
        }
    }

    // ✅ 핵심: StopAxis 이후 Done 확정은 dir/패턴과 무관하게 여기서 처리해야 한다.
    if (g_ax5StopIssued) {
        CoreMotionStatus st{};
        g_cm.GetStatus(&st);

        if (std::fabs(st.axesStatus[5].actualVelocity) <= 1.0) {

            // Forward()/Backward()가 이미 세팅한 g_ax5ActiveTask를 사용
            TaskId t = g_ax5ActiveTask;

            if (t == TaskId::Forward || t == TaskId::Backward) {
                TaskState ts = GetTaskState(t);
                if (ts == TaskState::Running || ts == TaskState::Stopped) {
                    SetTaskState(t, TaskState::Done);
                }
            }

            // 다음 동작 대비 정리
            g_ax5StopIssued = false;
            g_ax5ActiveTask = TaskId::COUNT;
        }
    }

    // Build AX5 UI status string
    const wchar_t* sideStr = L"Unknown";
    switch (g_ax5Side) {
    case Ax5Side::LeftBackward:   sideStr = L"LeftBackward"; break;
    case Ax5Side::RightBackward:  sideStr = L"RightBackward"; break;
    case Ax5Side::LeftForward:   sideStr = L"LeftForward"; break;
    case Ax5Side::RightForward:  sideStr = L"RightForward"; break;
    case Ax5Side::Center: sideStr = L"Center"; break;
    default: break;
    }

    g_ax5UiExtra = std::wstring(L"Side=") + sideStr;
    if (!g_ax5LastCheckName.empty()) {
        g_ax5UiExtra += std::wstring(L"  Check=") + g_ax5LastCheckName + (g_ax5LastCheckOk ? L" OK" : L" NG");
    }

    // Update demo panel after logic
    UpdateAxLimitPanel(ax0, ax1_1, ax1_2, ax2_up, ax2_dn, ax3_up, ax3_dn, l1, l2, l3, l4);
    g_ax5PrevL1 = l1; g_ax5PrevL2 = l2; g_ax5PrevL3 = l3; g_ax5PrevL4 = l4;
}

// =======================================
// UI 배치
// 좌측: GPIO 제어/표시(토글/라벨)
// 우측: Demo 버튼/상태/센서
// =======================================

enum : UINT_PTR {
    IDT_AX4_SENSOR_POLL = 0x2001,
    IDT_GPIO_REFRESH = 0x2002,
    IDT_AX_LIMIT_POLL = 0x2003
};
enum : int {
    // Right-panel buttons
    ID_BTN_GOLEFT = 11001,
    ID_BTN_GOWORKSTATION,
    ID_BTN_GORIGHT,

    ID_BTN_FORWARD,
    ID_BTN_BACKWARD,

    ID_BTN_UP,
    ID_BTN_DOWN,

    ID_BTN_FORKING,
    ID_BTN_UNFORKING,
    ID_BTN_OPEN,
    ID_BTN_CLOSE,

    ID_BTN_HOIST_UP,
    ID_BTN_HOIST_DOWN,

    ID_BTN_ALL_DEMO,
    ID_BTN_DEMO_LOAD,
    ID_BTN_DEMO_UNLOAD,

    // Demo extra buttons
    ID_BTN_GO1,
    ID_BTN_GO2,
    ID_BTN_GO3,
    ID_BTN_GO4,
    ID_BTN_GO5,

    ID_BTN_STOP_ALL
};

static void CreateStatusArea(HWND h, int x, int y, int w, int hgt)
{
    HWND grp = CreateWindow(TEXT("BUTTON"), TEXT("Status"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, y, w, hgt, h, 0, 0, 0);

    const int padL = 6, padR = 6;
    const int topPad = 22;
    const int bottomPad = 8;

    const int lineH = 22;
    const int gapY = 4;

    const int cols = 3;       // ✅ 무조건 3열
    const int gapX = 10;      // ✅ 열 간격(원하는 만큼 조절)

    const int innerX = x + padL;
    const int innerY = y + topPad;
    const int innerW = w - padL - padR;

    int availableH = hgt - topPad - bottomPad;
    int maxRowsPerCol = std::max(1, availableH / (lineH + gapY));

    // 3열 너비 계산 (간격 포함)
    int totalGapW = gapX * (cols - 1);
    int colW = (innerW - totalGapW) / cols;
    if (colW < 40) colW = 40; // 너무 좁아지는 경우 최소폭

    const int total = (int)TaskId::COUNT;

    // 문자열 준비
    std::vector<std::wstring> lines(total);
    for (int i = 0; i < total; ++i) {
        lines[i] = std::wstring(TaskName((TaskId)i)) + L": " +
            TaskStateStr(g_taskStatus[i].state.load());
    }

    for (int i = 0; i < total; ++i) {
        int col = i / maxRowsPerCol;   // 0,1,2...
        int row = i % maxRowsPerCol;

        // 3열 공간을 넘으면 표시 안 함(원하면 여기서 "...more" 처리 가능)
        if (col >= cols) continue;

        int xx = innerX + col * (colW + gapX);
        int yy = innerY + row * (lineH + gapY);

        HWND s = CreateWindow(TEXT("STATIC"),
            TEXT(""),
            WS_CHILD | WS_VISIBLE | SS_LEFT | SS_NOPREFIX,
            xx, yy, colW, lineH,
            h, 0, 0, 0);

        g_hStatusStatics[i] = s;
        SetWindowTextW(s, lines[i].c_str());
    }

    (void)grp;
}


static void CreateLeftGPIOUI(HWND h, HINSTANCE hInst, int x, int y, int w, int hgt)
{
    HFONT hTitle = MakeUIFont(16, FW_SEMIBOLD);
    HFONT hText = MakeUIFont(12);

    CreateWindow(TEXT("BUTTON"), TEXT("GPIO"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, y, w, hgt, h, 0, 0, 0);

    HWND hTitleLbl = CreateWindowEx(0, _T("STATIC"), _T("Found 16 GPIO(s)"),
        WS_CHILD | WS_VISIBLE, x + 10, y + 8, 240, 24, h, 0, hInst, 0);
    SendMessage(hTitleLbl, WM_SETFONT, (WPARAM)hTitle, TRUE);

    // DI 0..7 라벨 (2열) - 간격 확대
    int xDI = x + 10, yDI = y + 40;
    int diColW = 200;
    int diRowH = 24;
    for (int i = 0; i < 8; ++i) {
        TCHAR buf[64]; _stprintf_s(buf, _T("Pin %d : --"), i);
        int col = (i % 2);
        int row = (i / 2);
        g_lblDI[i] = CreateWindowEx(0, _T("STATIC"), buf,
            WS_CHILD | WS_VISIBLE, xDI + col * diColW, yDI + row * diRowH, diColW - 10, diRowH, h, 0, hInst, 0);
        SendMessage(g_lblDI[i], WM_SETFONT, (WPARAM)hText, TRUE);
    }

    // 토글 스위치 등록
    RegisterToggleClass(hInst);

    // DO 8..15 토글 스위치 + 라벨 (4열 x 2행)로 변경하여 높이 절약
    int xBase = x + 10, yBase = y + 40 + diRowH * 4 + 20; // DI 섹션 아래 충분한 간격
    int cols = 4;
    int colW = (w - 20) / cols; // 그룹박스 내 가용 폭 분배
    int rowH = 80;              // 각 토글의 세로 공간
    for (int idx = 0; idx < 8; ++idx) {
        int i = 8 + idx;
        int col = idx % cols;
        int row = idx / cols;
        int bx = xBase + col * colW + 5;
        int by = yBase + row * rowH;

        TCHAR lbl[128];
        if (g_DOFuncNames[i])
            _stprintf_s(lbl, _T("Pin %d : %s"), i, g_DOFuncNames[i]);
        else
            _stprintf_s(lbl, _T("Pin %d"), i);

        HWND hLbl = CreateWindowEx(WS_EX_CLIENTEDGE, _T("STATIC"), lbl,
            WS_CHILD | WS_VISIBLE, bx, by, colW - 12, 20, h, 0, hInst, 0);
        SendMessage(hLbl, WM_SETFONT, (WPARAM)hText, TRUE);

        HWND hSw = CreateWindowEx(0, TOGGLE_CLS, _T(""),
            WS_CHILD | WS_VISIBLE | WS_TABSTOP, bx, by + 24, colW - 40, 32, h, (HMENU)(100 + i), hInst, (LPVOID)(INT_PTR)i);
        SendMessage(hSw, WM_SETFONT, (WPARAM)hText, TRUE);
        g_swDO[i] = hSw;
    }
    // -------------------------------------------------------
    // Added: AX0~AX5 limit sensor readout (left side)
    // -------------------------------------------------------
    {
        int ySensors = yBase + rowH * 2 + 12;
        int hSensors = std::max(120, (y + hgt) - ySensors - 12);
        if (hSensors > 210) hSensors = 210;

        CreateWindow(TEXT("BUTTON"), TEXT("AX Limit Sensors"),
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            x + 10, ySensors, w - 20, hSensors, h, 0, 0, 0);

        int sx = x + 24;
        int sy = ySensors + 28;
        int lineH = 20;

        // 6 compact lines
        for (int i = 0; i < 6; ++i) {
            g_hAxLimitStatics[i] = CreateWindowEx(0, _T("STATIC"), _T(""),
                WS_CHILD | WS_VISIBLE, sx, sy + i * (lineH + 6), w - 48, lineH, h, 0, hInst, 0);
            SendMessage(g_hAxLimitStatics[i], WM_SETFONT, (WPARAM)hText, TRUE);
        }
        // Initialize text
        SetWindowText(g_hAxLimitStatics[0], _T("AX0 LIMIT : --"));
        SetWindowText(g_hAxLimitStatics[1], _T("AX1 L1/L2 : --/--"));
        SetWindowText(g_hAxLimitStatics[2], _T("AX2 UP/DN : --/--"));
        SetWindowText(g_hAxLimitStatics[3], _T("AX3 UP/DN : --/--"));
        SetWindowText(g_hAxLimitStatics[4], _T("AX4 : (kept - existing logic)"));
        SetWindowText(g_hAxLimitStatics[5], _T("AX5 L1..L4 : ----"));

        // ✅ StopFlags 4줄 표시 Static (왼쪽 패널에 "실시간" 표시)
        //   - SS_LEFT + 줄바꿈(\r\n) 표시를 위해 넉넉한 height
        int stopY = sy + 6 * (lineH + 6) + 8;
        g_hAx5StopFlagsStatic = CreateWindowW(
            L"STATIC",
            L"Forward left  : OFF\r\nForward right : OFF\r\nBackward left : OFF\r\nBackward right: OFF",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            sx, stopY, w - 48, 280,
            h, nullptr, hInst, nullptr
        );
        if (g_hAx5StopFlagsStatic) {
            SendMessageW(g_hAx5StopFlagsStatic, WM_SETFONT, (WPARAM)hText, TRUE);
        }
    }

    DeleteObject(hTitle);
    DeleteObject(hText);
}

static void CreateRightDemoUI(HWND h, int x, int y, int w, int hgt)
{
    HFONT hBtn = MakeUIFont(12);
    int yCursor = y;

    auto addGroup = [&](const TCHAR* title, int height) {
        CreateWindow(TEXT("BUTTON"), title, WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            x, yCursor, w, height, h, 0, 0, 0);
        int top = yCursor + 30;
        int left = x + 16;
        return std::pair<int, int>(left, top);
        };

    const int gapY = 10;
    const int btnH = 32;
    const int padX = 16;

    // 1) Main주행
    {
        int grpH = 78;
        auto [bx, by] = addGroup(TEXT("Main주행"), grpH);

        int btnW = (w - padX * 2 - 20) / 3;
        CreateWindow(TEXT("BUTTON"), TEXT("GoLeft"), WS_CHILD | WS_VISIBLE,
            bx, by, btnW, btnH, h, (HMENU)ID_BTN_GOLEFT, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("GoWorkstation"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by, btnW, btnH, h, (HMENU)ID_BTN_GOWORKSTATION, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("GoRight"), WS_CHILD | WS_VISIBLE,
            bx + (btnW + 10) * 2, by, btnW, btnH, h, (HMENU)ID_BTN_GORIGHT, 0, 0);

        yCursor += grpH + gapY;
    }

    // 2) Side주행
    {
        int grpH = 78;
        auto [bx, by] = addGroup(TEXT("Side주행"), grpH);

        int btnW = (w - padX * 2 - 10) / 2;
        CreateWindow(TEXT("BUTTON"), TEXT("Forward"), WS_CHILD | WS_VISIBLE,
            bx, by, btnW, btnH, h, (HMENU)ID_BTN_FORWARD, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Backward"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by, btnW, btnH, h, (HMENU)ID_BTN_BACKWARD, 0, 0);

        yCursor += grpH + gapY;
    }

    // 3) 본체 업다운
    {
        int grpH = 78;
        auto [bx, by] = addGroup(TEXT("본체 업다운"), grpH);

        int btnW = (w - padX * 2 - 10) / 2;
        CreateWindow(TEXT("BUTTON"), TEXT("Up(Main)"), WS_CHILD | WS_VISIBLE,
            bx, by, btnW, btnH, h, (HMENU)ID_BTN_UP, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Down(Side)"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by, btnW, btnH, h, (HMENU)ID_BTN_DOWN, 0, 0);

        yCursor += grpH + gapY;
    }

    // 4) 포킹부
    {
        int grpH = 120;
        auto [bx, by] = addGroup(TEXT("포킹부"), grpH);

        int btnW = (w - padX * 2 - 10) / 2;
        CreateWindow(TEXT("BUTTON"), TEXT("Forking"), WS_CHILD | WS_VISIBLE,
            bx, by, btnW, btnH, h, (HMENU)ID_BTN_FORKING, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Unforking"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by, btnW, btnH, h, (HMENU)ID_BTN_UNFORKING, 0, 0);

        CreateWindow(TEXT("BUTTON"), TEXT("Open"), WS_CHILD | WS_VISIBLE,
            bx, by + btnH + 10, btnW, btnH, h, (HMENU)ID_BTN_OPEN, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Close"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by + btnH + 10, btnW, btnH, h, (HMENU)ID_BTN_CLOSE, 0, 0);

        yCursor += grpH + gapY;
    }

    // 5) 승하강부
    {
        int grpH = 78;
        auto [bx, by] = addGroup(TEXT("승하강부"), grpH);

        int btnW = (w - padX * 2 - 10) / 2;
        CreateWindow(TEXT("BUTTON"), TEXT("HoistUp"), WS_CHILD | WS_VISIBLE,
            bx, by, btnW, btnH, h, (HMENU)ID_BTN_HOIST_UP, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("HoistDown"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by, btnW, btnH, h, (HMENU)ID_BTN_HOIST_DOWN, 0, 0);

        yCursor += grpH + gapY;
    }

    // 6) Demo
    {
        int grpH = 220;
        auto [bx, by] = addGroup(TEXT("Demo"), grpH);

        int btnW = (w - padX * 2 - 20) / 3;
        CreateWindow(TEXT("BUTTON"), TEXT("All_Demo"), WS_CHILD | WS_VISIBLE,
            bx, by, btnW, btnH, h, (HMENU)ID_BTN_ALL_DEMO, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("DemoLoad"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, by, btnW, btnH, h, (HMENU)ID_BTN_DEMO_LOAD, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("DemoUnload"), WS_CHILD | WS_VISIBLE,
            bx + (btnW + 10) * 2, by, btnW, btnH, h, (HMENU)ID_BTN_DEMO_UNLOAD, 0, 0);

        // 추가: Go1~Go5
        int row2Y = by + btnH + 10;
        int row3Y = by + (btnH + 10) * 2;
        CreateWindow(TEXT("BUTTON"), TEXT("Go1"), WS_CHILD | WS_VISIBLE,
            bx, row2Y, btnW, btnH, h, (HMENU)ID_BTN_GO1, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Go2"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, row2Y, btnW, btnH, h, (HMENU)ID_BTN_GO2, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Go3"), WS_CHILD | WS_VISIBLE,
            bx + (btnW + 10) * 2, row2Y, btnW, btnH, h, (HMENU)ID_BTN_GO3, 0, 0);

        CreateWindow(TEXT("BUTTON"), TEXT("Go4"), WS_CHILD | WS_VISIBLE,
            bx, row3Y, btnW, btnH, h, (HMENU)ID_BTN_GO4, 0, 0);
        CreateWindow(TEXT("BUTTON"), TEXT("Go5"), WS_CHILD | WS_VISIBLE,
            bx + btnW + 10, row3Y, btnW, btnH, h, (HMENU)ID_BTN_GO5, 0, 0);

        // STOP ALL 버튼을 Demo 그룹 아래에 크게 하나 배치
        CreateWindow(TEXT("BUTTON"), TEXT("STOP ALL"), WS_CHILD | WS_VISIBLE,
            bx, by + (btnH + 10) * 3 + 8, w - padX * 2, 36, h, (HMENU)ID_BTN_STOP_ALL, 0, 0);

        yCursor += grpH + gapY;
    }

    // Status 박스
    {
        int statusLines = (int)TaskId::COUNT;
        int lineH = 22;
        int statusH = 25 + statusLines * (lineH + 6) + 10;
        if (yCursor + statusH > y + hgt) {
            statusH = std::max(120, (y + hgt) - yCursor - 10);
        }
        CreateStatusArea(h, x, yCursor, w, statusH);
        yCursor += statusH + gapY;
    }

    // 버튼 폰트 적용
    for (int id : {
        ID_BTN_GOLEFT, ID_BTN_GOWORKSTATION, ID_BTN_GORIGHT,
            ID_BTN_FORWARD, ID_BTN_BACKWARD,
            ID_BTN_UP, ID_BTN_DOWN,
            ID_BTN_FORKING, ID_BTN_UNFORKING, ID_BTN_OPEN, ID_BTN_CLOSE,
            ID_BTN_HOIST_UP, ID_BTN_HOIST_DOWN,
            ID_BTN_ALL_DEMO, ID_BTN_DEMO_LOAD, ID_BTN_DEMO_UNLOAD,
            ID_BTN_GO1, ID_BTN_GO2, ID_BTN_GO3, ID_BTN_GO4, ID_BTN_GO5,
            ID_BTN_STOP_ALL
    })
    {
        if (HWND b = GetDlgItem(h, id)) SendMessage(b, WM_SETFONT, (WPARAM)hBtn, TRUE);
    }

    DeleteObject(hBtn);
}

// 레이아웃 헬퍼: 리사이즈 시 재배치
static void LayoutChildren(HWND hWnd) {
    RECT rc; GetClientRect(hWnd, &rc);
    int totalW = rc.right - rc.left;
    int totalH = rc.bottom - rc.top;

    int margin = 10;
    int gap = 10;

    // 좌측/우측 폭 비율
    int leftW = 560;               // 좌측 GPIO 패널 기본 폭 확대
    if (totalW < 900) leftW = totalW / 2 - gap; // 극단적으로 작아질 때 비율 보정
    int rightX = margin + leftW + gap;
    int rightW = totalW - rightX - margin;

    // 좌측 높이는 전체 높이 - 여백
    int leftH = totalH - margin * 2;
    int rightH = leftH;

    (void)rightX; (void)rightW; (void)leftH; (void)rightH;

    // 좌측 그룹을 전체 높이 사용
    // 만들어진 컨트롤을 재생성하지 않고, 그룹박스 기준으로는 별도 핸들을 보관하지 않았으므로
    // 자식들을 상대적 좌표로 만들었고, 여기서는 레이아웃 재구성이 필요하면 재생성하는 구조이나
    // 간단히 윈도우 전체를 다시 만들어지는 패턴이 아니므로, 리사이즈 대비는 비율만 유지.
    // 이미 생성된 컨트롤의 위치를 옮기려면 핸들을 저장해야 하므로, 여기서는 최초 생성 시 충분한 여백이 있어 겹치지 않게 보장.
    // 따라서 리사이즈는 영향을 덜 주도록 기본 사이즈를 크게 설정함.
    // 필요 시 전체 재구성 로직으로 확장 가능.

    // 좌측 GPIO 영역은 CreateLeftGPIOUI 생성 당시 좌표 고정이므로 리사이즈 영향 최소화
    // 우측은 CreateRightDemoUI 생성 당시 좌표 고정이므로 역시 기본 창을 크게 유지하여 겹침 방지
}
static const TCHAR* kDemoClass = TEXT("WMX_DEMO_SHUTTLE");

static LRESULT CALLBACK DemoWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE: {
        g_hDemoWnd = hWnd;

        RECT rc; GetClientRect(hWnd, &rc);
        int totalW = rc.right - rc.left;
        int totalH = rc.bottom - rc.top;

        int margin = 10;
        int gap = 10;
        int leftW = 560;
        if (totalW < 900) leftW = totalW / 2 - gap;
        int rightX = margin + leftW + gap;
        int rightW = totalW - rightX - margin;

        CreateLeftGPIOUI(hWnd, ((LPCREATESTRUCT)lParam)->hInstance, margin, margin, leftW, totalH - margin * 2);
        CreateRightDemoUI(hWnd, rightX, margin, rightW, totalH - margin * 2);

        // 타이머 시작
        SetTimer(hWnd, IDT_AX4_SENSOR_POLL, AX4_SENSOR_POLL_MS, nullptr);
        SetTimer(hWnd, IDT_GPIO_REFRESH, kPollIntervalMs, nullptr);

        SetTimer(hWnd, IDT_AX_LIMIT_POLL, AX_LIMIT_POLL_MS, nullptr);

        // 창이 열릴 때 STO 펄스 1회 (오류 클리어)
        DoGripServoOff_Compat(hWnd);

        // 요구사항: 기본은 ON 유지, 동작 중에는 깜빡임
        LedSet(true);

        return 0;
    }
    case WM_SIZE:
        LayoutChildren(hWnd);
        return 0;

    case WM_COMMAND: {
        int id = LOWORD(wParam);

        // 좌측 GPIO 토글 클릭(BN_CLICKED): 100+8..100+15
        if (id >= 108 && id <= 115 && HIWORD(wParam) == BN_CLICKED) {
            HWND hSw = (HWND)lParam;
            int pin = id - 100;
            ToggleState* ts = (ToggleState*)GetWindowLongPtr(hSw, GWLP_USERDATA);
            bool on = ts ? ts->on : false;

            // Open(8) / Close(9) 상호 배타 처리
            if (pin == 8 && on) {
                ToggleDO_HW(9, false, g_hDemoWnd);
                SetTaskState(TaskId::Close, TaskState::Idle);
            }
            else if (pin == 9 && on) {
                ToggleDO_HW(8, false, g_hDemoWnd);
                SetTaskState(TaskId::Open, TaskState::Idle);
            }

            // Open/Close는 STO 먼저
            if (on && (pin == 8 || pin == 9)) {
                DoGripServoOff_Compat(g_hDemoWnd);
            }
            ToggleDO_HW(pin, on, g_hDemoWnd);

            if (pin == 8) SetTaskState(TaskId::Open, on ? TaskState::Running : TaskState::Stopped);
            if (pin == 9) SetTaskState(TaskId::Close, on ? TaskState::Running : TaskState::Stopped);

            return 0;
        }

        switch (id) {
        case ID_BTN_GOLEFT:          GoLeft();           return 0;
        case ID_BTN_GOWORKSTATION:   GoWorkstation();    return 0;
        case ID_BTN_GORIGHT:         GoRight();          return 0;

        case ID_BTN_FORWARD:         Forward();          return 0;
        case ID_BTN_BACKWARD:        Backward();         return 0;

        case ID_BTN_UP:              Up();             return 0;
        case ID_BTN_DOWN:            Down();             return 0;

        case ID_BTN_FORKING:         Forking();          return 0;
        case ID_BTN_UNFORKING:       Unforking();        return 0;
        case ID_BTN_OPEN:            Open();  return 0;
        case ID_BTN_CLOSE:           Close(); return 0;

        case ID_BTN_HOIST_UP:        HoistUp();          return 0;
        case ID_BTN_HOIST_DOWN:      HoistDown();        return 0;

        case ID_BTN_ALL_DEMO:        StartAllDemo();     return 0;
        case ID_BTN_DEMO_LOAD:       StartDemoLoad();    return 0;
        case ID_BTN_DEMO_UNLOAD:     StartDemoUnload();  return 0;

        case ID_BTN_GO1:             GoOne();            return 0;
        case ID_BTN_GO2:             GoTwo();            return 0;
        case ID_BTN_GO3:             GoThree();          return 0;
        case ID_BTN_GO4:             GoFour();           return 0;
        case ID_BTN_GO5:             GoFive();           return 0;

        case ID_BTN_STOP_ALL:        DoStopAll(hWnd);    return 0;
        default: break;
        }
        break;
    }
    case WM_TIMER:
        if (wParam == IDT_AX4_SENSOR_POLL) {
            Axis2SensorTimerProc(hWnd);
            return 0;
        }
        else if (wParam == IDT_LED_BLINK) {
            if (g_ledBlinkActive) {
                DWORD now = GetTickCount();
                if (now - g_ledLastToggleTick >= LED_BLINK_MS) {
                    g_ledLastToggleTick = now;
                    g_ledBlinkStateOn = !g_ledBlinkStateOn;
                    LedSet(g_ledBlinkStateOn);
                }
            }
            return 0;
        }
        else if (wParam == IDT_AX_LIMIT_POLL) {
            AxLimitSensorTimerProc(hWnd);
            return 0;
        }
        else if (wParam == IDT_GPIO_REFRESH) {
            // 펄스 자동 OFF 처리
            DWORD now = GetTickCount();
            for (int i = 10; i < 11; ++i) {
                if (g_outputPendingOff[i]) {
                    if (now - g_outputOnTick[i] >= kOutputPulseMs) {
                        g_outputPendingOff[i] = false;
                        ToggleDO_HW(i, false, g_hDemoWnd);
                    }
                }
            }

            // 주기적으로 DI/DO 상태 갱신
            RefreshLevels(hWnd);

            // Motioning(DI0) ↔ DO11 1:1 동기화 (DemoLoad/Unload 동작 중에는 제외)
            {
                TaskState loadState = GetTaskState(TaskId::DemoLoad);
                TaskState unloadState = GetTaskState(TaskId::DemoUnload);

                bool demoBusy =
                    (loadState == TaskState::Running) ||
                    (unloadState == TaskState::Running);

                if (!demoBusy) {
                    static bool s_prevDo11 = false;
                    bool motioning = g_diStable[0];

                    if (motioning != s_prevDo11) {
                        ToggleDO_HW(11, motioning, g_hDemoWnd);
                        s_prevDo11 = motioning;
                    }
                }
            }
            return 0;
        }
        break;

    case WM_DESTROY:
        KillTimer(hWnd, IDT_AX4_SENSOR_POLL);
        KillTimer(hWnd, IDT_GPIO_REFRESH);
        KillTimer(hWnd, IDT_AX_LIMIT_POLL);
        //PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// 외부에서 호출하는 진입 함수 (기존 ShowDemoControlWindow 대체)
void ShowDemoControlWindow(HWND hParent, bool minimized)
{
    if (g_hDemoWnd && IsWindow(g_hDemoWnd)) {
        ShowWindow(g_hDemoWnd, minimized ? SW_SHOWMINNOACTIVE : SW_SHOWNORMAL);
        if (!minimized) {
            SetForegroundWindow(g_hDemoWnd);
        }
        return;
    }

    WNDCLASS wc{};
    wc.lpszClassName = TEXT("WMX3DemoGPIOUnifiedWnd");
    wc.lpfnWndProc = DemoWndProc;
    wc.hInstance = (HINSTANCE)GetWindowLongPtr(hParent, GWLP_HINSTANCE);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);

    ATOM atom = RegisterClass(&wc);
    if (!atom) {
        DWORD err = GetLastError();
        if (err != ERROR_CLASS_ALREADY_EXISTS) {
            MessageBox(hParent, TEXT("Failed to register Demo window class!"), TEXT("Error"), MB_ICONERROR);
            return;
        }
    }

    int winW = 1280;
    int winH = 1000;
    g_hDemoWnd = CreateWindow(
        TEXT("WMX3DemoGPIOUnifiedWnd"), TEXT("Demo + GPIO Control"),
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_THICKFRAME,
        CW_USEDEFAULT, CW_USEDEFAULT, winW, winH, hParent, nullptr, wc.hInstance, nullptr);

    ShowWindow(g_hDemoWnd, minimized ? SW_SHOWMINNOACTIVE : SW_SHOWNORMAL);
    UpdateWindow(g_hDemoWnd);
}