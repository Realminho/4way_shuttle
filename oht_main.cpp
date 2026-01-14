// ===============================
// WMX3 Practice + Sync Group UI (Restored + Fixed) + ECAT 0x6063/0x603F Monitor
// + Alt Target helper + Error Manual JSON Viewer (Fastech/Welcon)
// - Added: Auto column width by content + multi-keyword search
// - Added: E-Stop toggle buttons and status on Main and Sync windows
// - Modified: TCP Serial Monitor moved to a separate window with "Serial Monitor..." button
// - Modified by request: Replace Demo0/1/2 buttons with Demo + GPIO buttons
// Demo: opens external DemoControl window (ShowDemoControlWindow)
// GPIO: opens external GPIO window (RunGPIOWindowExternal)
// - Fixed: desyncDec member removed from Sync::SyncGroup (use Config::SyncParam.masterDesyncDec/slaveDesyncDec)
// - Added: Motion Log/Scope windows and background logger thread
// ===============================
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <commctrl.h>
#include <tchar.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <functional>
#include <fstream>
#include <iterator>
#include <cwctype>
#include <locale>
#include <direct.h>
#include <cwchar>
#include <shlwapi.h>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"
#include "EcApi.h" // EtherCAT API
#include "LogApi.h"
#include "DemoShared.h"

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "Shlwapi.lib")

using namespace wmx3Api;
using namespace ecApi;

// 축 갯수(확장): 0~8 총 9축
static const int kNumAxes = 12;

// ===== External launchers declared by user-provided modules =====
void ShowDemoControlWindow(HWND hParent, bool minimized = false);
extern "C" int RunGPIOWindowExternal(HINSTANCE hInst);

// ==== DemoControl에서 제공하는 GPIO/그리퍼/상태 함수 extern ====
// CHANGED: 아래 extern들은 democontrol 모듈의 함수를 창 없이도 호출하기 위해 필요
static int g_gpioBank = -1;
extern void ToggleDO_HW(int pin, bool turnOn, HWND hWnd);
//extern void DoGripServoOff_Compat(HWND hWnd);
extern bool EnumerateGPIO();
extern bool PickBank_DI0_7_DO8_15();
extern bool EnsureDO8to15AsOutput_BankFirst();
extern void RefreshLevels(HWND hWnd);
extern bool IsGripperOpen();
extern bool IsGripperOpenAndIdle();
extern bool IsGripperClosed();
extern bool IsGripperClosedAndIdle();
extern bool g_diStable[8]; // DI0(Motioning), DI1(Catched) 등 디바운스 결과 사용
extern void GoLeft();     // Conveyor 버튼이 눌렸을 때 실행되는 함수
extern void GoWorkstation();  // Workstation 버튼이 눌렸을 때 실행되는 함수
extern void HoistDown();    // Conveyor Down 버튼
extern void Down();        // Work Down 버튼
extern void Up();            // Up 버튼
extern void DoClose_Compat(HWND hWnd); // Close 버튼
extern void DoOpen_Compat(HWND hWnd); // Open 버튼
extern void DoStopAll(HWND hWnd); // Stop All 버튼
extern bool IsGripperOpen(); // 그립퍼 열림 상태 외부 참조
extern bool IsGripperOpenAndIdle(); // 그립퍼 열림 상태 외부 참조
extern bool IsGripperClosed(); // 그립퍼 닫힘 상태 외부 참조
extern bool IsGripperClosedAndIdle(); // 그립퍼 닫힘 상태 외부 참조
extern bool g_distable[8]; // 그립퍼 축 비활성화 플래그 외부 참조

// 보조 대기 함수들 (질문 본문과 동일) ? WaitUntil, WaitTaskFinished, WaitAllAxesStopped 등
extern bool WaitUntil(bool (*pred)(), DWORD timeoutMs, DWORD pollMs);
extern bool WaitAllAxesStopped(double velEps, DWORD timeoutMs); // 외부 참조
extern bool WaitTaskFinished(TaskId id, DWORD timeoutMs, DWORD pollMs);

// Axis2 Limit 감지 → Stop → Idle되면 home 시작
extern void Axis2HandleLimitOnceAndHome();

// ApproachProfile 구조체 및 StartMoveWithApproach 함수 선언
extern struct ApproachProfile {
	double vpps = 1000.0;
	double accMs = 80.0;
	double decMs = 10.0;
};

extern void StartMoveWithApproach(int axis, long long target, TaskId task, double mainVpps, double mainAccMs, double mainDecMs, double posEps, double velEps, DWORD timeoutMs, double approachEps, const ApproachProfile& ap);

// ============ 상태 조회 헬퍼(예시) ============
static bool IsGripperAlreadyOpen()
{
	// IsGripperOpenAndIdle가 충분하면 그것으로 대체 가능
	return IsGripperOpenAndIdle();
}

static bool IsGripperAlreadyClosed()
{
	return IsGripperClosedAndIdle();
}

// ===================== 사용자 설정 =====================
static TCHAR g_installPath[] = TEXT("C:\\Program Files\\SoftServo\\WMX3");
static UINT POLL_MS = 100;
static const int vel_idle_threshold = 5;
static const long long inpos_tol_counts = 5;
static const DWORD idle_wait_poll_ms = 10;

// ======================================================

// 로그/스코프 설정
const DWORD LOG_POLL_MS = 50;

// 축별 명령 수신/완료 카운터
std::atomic<unsigned long> g_cmdDoneCount[kNumAxes];


// 명령 추적
struct AxisCommandInfo {
	std::atomic<long long> target{ 0 };
	std::atomic<int> axis{ -1 };
	std::atomic<int> vel{ 0 };
	std::atomic<int> acc{ 0 };
	std::atomic<int> dec{ 0 };
	std::atomic<bool> active{ false };
	std::atomic<ULONGLONG> startTick{ 0 };
	std::atomic<ULONGLONG> endTick{ 0 };
};
AxisCommandInfo g_axisCmdInfo[kNumAxes];

// CHANGED: Per-axis sampling enable flag and per-command row index for possible future logic
std::atomic<bool> g_axisLogEnabled[kNumAxes];
std::atomic<unsigned long> g_axisLogRowIdx[kNumAxes];

// WMX3 전역
WMX3Api g_wmx;
CoreMotion g_cm(&g_wmx);
Home g_home(&g_cm);
bool g_deviceOpened = false;
bool g_commStarted = false;

// Log 클래스 전역
Log g_log(&g_wmx);

// 재사용 상태 버퍼
static CoreMotionStatus g_status{};

static void StopMultiJog();
static void StopJogIfActive();

// ===== CHANGED: 그립 동작 중인지 여부 플래그 =====
// democontrol 창 없이 TCP로 Open/Close를 수행할 때, 움직이는 동안은 0x00 상태를 내기 위함
static std::atomic<bool> g_gripBusy{ false };

// ===== STO 펄스 자동 OFF를 위한 예약 =====
// oht main에서도 AutoInitIoAndRefresh 시점에 ToggleDO_HW(10,true) 후 자동 OFF를 처리
static std::atomic<bool> g_ohtStoPulsePendingOff{ false };
static std::atomic<DWORD> g_ohtStoPulseOnTick{ 0 };
static const DWORD kOhtStoPulseMs = 100;

// ------------------ Serial Monitor Window ------------------
static HWND g_hSerialWnd = nullptr;

// WndProc 먼저 선언
LRESULT CALLBACK SerialWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);


// ===================== NEW: Manual/Auto Mode =====================
static std::atomic<bool> g_autoMode{ false }; // false=Manual, true=Auto

// Helper to check/guard manual-only commands
static bool IsManualAllowed(HWND hWnd) {
	if (g_autoMode.load()) {
		MessageBox(hWnd, TEXT("현재 자동 모드입니다. 수동 동작은 무시됩니다."), TEXT("모드"), MB_ICONWARNING);
		return false;
	}
	return true;
}
// Helper to check/guard auto-only commands (TCP etc.)
static bool IsAutoAllowed() {
	return g_autoMode.load();
}

// Ensure Serial Monitor window exists and is a top-level independent window
static void EnsureSerialWindowTopLevel(HWND hMain)
{
	if (g_hSerialWnd && IsWindow(g_hSerialWnd))
		return;

	WNDCLASS wc{};
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = SerialWndProc; // 람다 대신 일반 함수 포인터
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = (HINSTANCE)GetWindowLongPtr(hMain, GWLP_HINSTANCE);
	wc.hIcon = nullptr;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	wc.lpszMenuName = nullptr;
	wc.lpszClassName = TEXT("WMX3SerialWnd");

	RegisterClass(&wc);

	// Create as top-level independent window (no parent), ensure it shows on taskbar
	g_hSerialWnd = CreateWindowEx(
		WS_EX_APPWINDOW,
		TEXT("WMX3SerialWnd"),
		TEXT("Serial Monitor (TCP)"),
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, 800, 400,
		nullptr,           // NO parent
		nullptr,           // menu
		wc.hInstance,
		nullptr);

	if (g_hSerialWnd) {
		ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
		UpdateWindow(g_hSerialWnd);
		BringWindowToTop(g_hSerialWnd);
		SetForegroundWindow(g_hSerialWnd);
		SetWindowPos(g_hSerialWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
		SetWindowPos(g_hSerialWnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
	}
}

static void ApplyModeUI(HWND hMain) {
	// 수동 모드: 메인 GUI 전면, SerialMonitor 숨기거나 최소화
	// 자동 모드: 메인 GUI 최소화, SerialMonitor(있으면) 전면
	if (!hMain) return;
	if (g_autoMode.load()) {
		ShowWindow(hMain, SW_MINIMIZE);
		EnsureSerialWindowTopLevel(hMain);
		if (IsWindow(g_hSerialWnd)) {
			ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
			BringWindowToTop(g_hSerialWnd);
			SetForegroundWindow(g_hSerialWnd);
		}
	}
	else {
		ShowWindow(hMain, SW_SHOWNORMAL);
		SetForegroundWindow(hMain);
		if (IsWindow(g_hSerialWnd)) {
			// 자동 모드 해제 시 Serial을 굳이 닫지 않고 그대로 유지. 필요하면 최소화.
			ShowWindow(g_hSerialWnd, SW_MINIMIZE);
		}
	}
}

static void SwitchToManual(HWND hMain) {
	// Stop any jog
	StopMultiJog();
	StopJogIfActive();
	// Optionally stop all motion?
	for (int a = 0; a < kNumAxes - 3; ++a) StopAxis(a); // 선택사항

	g_autoMode = false;
	ApplyModeUI(hMain);
}
static void SwitchToAuto(HWND hMain) {
	// Stop jog
	StopMultiJog();
	StopJogIfActive();
	// Optionally stop all motion?
	for (int a = 0; a < kNumAxes - 3; ++a) StopAxis(a); // 선택사항

	g_autoMode = true;
	ApplyModeUI(hMain);
}

static void ShowErrMsgBox(const TCHAR* title, long err, WMX3Api& api) {
	char bufA[256] = {};
	api.ErrorToString(err, bufA, (unsigned)sizeof(bufA));
#ifdef UNICODE
	wchar_t wbuf[256]{}; MultiByteToWideChar(CP_ACP, 0, bufA, -1, wbuf, 256);
	TCHAR msg[512] = {};
	_stprintf_s(msg, TEXT("%s\r\nerr=%ld (%s)"), title, err, wbuf);
#else
	TCHAR msg[512] = {};
	_stprintf_s(msg, TEXT("%s\r\nerr=%ld (%hs)"), title, err, bufA);
#endif
	MessageBox(nullptr, msg, TEXT("WMX3 Error"), MB_OK | MB_ICONERROR);
}

int TimeMsToAcc(double vel_cnt_per_s, double t_ms) {
	if (t_ms <= 0) t_ms = 1;
	double acc = vel_cnt_per_s / (t_ms / 1000.0);
	if (acc < 1) acc = 1;
	if (acc > 200000000) acc = 200000000;
	return (int)acc;
}
bool EnsureServoOn(int axis) {
	g_cm.GetStatus(&g_status);
	if (!g_status.axesStatus[axis].servoOn) {
		long e = g_cm.axisControl->SetServoOn(axis, 1);
		if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("Servo ON 실패"), e, g_wmx); return false; }
		Sleep(20);
	}
	return true;
}
// 급정지(QuickStop Dec 파라미터 사용)로 축 정지
void StopAxis(int axis)
{
	if (!g_commStarted) return;

	// 1) 모션 쪽은 Quick Stop 사용
	//    - WMX3APIFUNC ExecQuickStop(int axis)
	//    - QuickStop Dec 파라미터로 감속 → 급정지
	long e = g_cm.motion->ExecQuickStop(axis);

	// 필요하면 실패 시 일반 Stop 으로 폴백
	if (e != ErrorCode::None) {
		g_cm.motion->Stop(axis);
	}

	// 2) 속도/토크 명령도 모두 끊어준다 (이전 코드 유지)
	g_cm.velocity->Stop(axis);
	if (g_cm.torque) {
		g_cm.torque->StopTrq(axis);
	}
}

static bool EnterPosMode(int axis) {
	long e = g_cm.axisControl->SetAxisCommandMode(axis, AxisCommandMode::Position);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("SetAxisCommandMode(Position) 실패"), e, g_wmx); return false; }
	return true;
}
bool EnsurePosModeNoStop(int axis) {
	g_cm.GetStatus(&g_status);
	if (g_status.axesStatus[axis].axisCommandModeFeedback == AxisCommandMode::Position) return true;
	return EnterPosMode(axis);
}

// ====================== NEW: Limit/Home sensors for Axis2 ======================
// Limit sensor: IO_ADDR=8, IO_BIT=1 (Active High)
// Home sensor:  IO_ADDR=8, IO_BIT=2 (Active High)
// EtherCAT IO 보드 주소/비트
static const int AX0_LIMIT_L_ADDR = 0;
static const int AX0_LIMIT_L_BIT = 0;
static const int AX0_LIMIT_R_ADDR = 0;
static const int AX0_LIMIT_R_BIT = 1;

static const int AX2_LIMIT_ADDR = 8;
static const int AX2_LIMIT_BIT = 1;
static const int AX2_HOME_ADDR = 8;
static const int AX2_HOME_BIT = 2;

// 센서 활성 레벨 (필요시 LOW → HIGH로 수정)
static const bool AX2_LIMIT_ACTIVE_HIGH = true;
static const bool AX2_HOME_ACTIVE_HIGH = true;
static const bool AX0_LIMIT_L_ACTIVE_HIGH = true;
static const bool AX0_LIMIT_R_ACTIVE_HIGH = true;

// 폴링/디바운스 시간
static const DWORD AX2_SENSOR_POLL_MS = 5;
static const DWORD AX2_SENSOR_DEBOUNCE_MS = 5;

// Axis2 센서 상태 (GUI 표시용)
static std::atomic<bool> g_ax2LimitOn{ false };   // 현재 Limit 입력 ON 여부
static std::atomic<bool> g_ax2HomeOn{ false };    // 현재 Home 입력 ON 여부

// Axis2 Limit/Home 제어 플래그
static std::atomic<bool>  g_ax2LimitLatched{ false };      // Limit 최초 인식 래치
static std::atomic<bool>  g_ax2LimitBlocking{ false };     // Limit 중 -방향 명령 차단
static std::atomic<bool>  g_ax2StopIssuedOnLimit{ false }; // Limit에서 Stop 명령 1회 발행
static std::atomic<bool>  g_ax2HomingStarted{ false };     // Limit 후 Home 시작 여부

static std::atomic<bool>  g_ax2HomeDebounceOn{ false };    // Home ON 디바운스 완료
static std::atomic<DWORD> g_ax2HomeLastTick{ 0 };          // Home 디바운스용 시각
static std::atomic<bool>  g_ax2HomeRampIssued{ false };    // Home ON 시 속도 2000 전환 명령 1회 발행

static std::atomic<DWORD> g_ax2LimitIdleTime{ 0 };   // Idle 최초 감지 시각

static std::atomic<bool>  g_ax2ServoReady{ false };    // axis2 servo on 준비 완료 여부
static std::atomic<DWORD> g_ax2ServoOnTime{ 0 };       // axis2 servoOn 감지 시각
static const DWORD AX2_SENSOR_ENABLE_DELAY_MS = 1500;  // 1초 지연

// ================= Axis0 Left/Right Limit 상태/블록 플래그 =================
// AX0 쪽은 센서 읽기 결과를 "free = true, limit 감지 = false" 로 사용
static std::atomic<bool> g_ax0LimitLFree{ true };          // true = 정상, false = L 리밋 감지
static std::atomic<bool> g_ax0LimitRFree{ true };          // true = 정상, false = R 리밋 감지

static std::atomic<bool> g_ax0LimitLLatched{ false };      // L 리밋 최초 인식 래치
static std::atomic<bool> g_ax0LimitRLatched{ false };      // R 리밋 최초 인식 래치

// L 리밋 ON → +방향 블록, R 리밋 ON → -방향 블록
static std::atomic<bool> g_ax0BlockPlus{ false };          // Axis0 + 방향 명령 차단
static std::atomic<bool> g_ax0BlockMinus{ false };         // Axis0 - 방향 명령 차단


static bool IsAxis2ServoOn()
{
	g_cm.GetStatus(&g_status);
	return g_status.axesStatus[2].servoOn;
}


// 리밋/홈 센서 읽기
static bool ReadInputBit(int addr, int bit, bool activeHigh) {
	if (!g_commStarted) return false;
	Io io(&g_wmx);
	unsigned char v = 0;
	long e = io.GetInBitEx(addr, bit, &v);
	if (e != ErrorCode::None) return false; // 실패 시 OFF 취급
	bool onRaw = (v != 0);
	return activeHigh ? onRaw : !onRaw;
}

static bool WriteOutputBit(int addr, int bit, bool onLogical, bool activeHigh)
{
	if (!g_commStarted) return false;

	Io io(&g_wmx);

	// 논리 ON/OFF -> 장비에 쓸 raw 값(0/1)으로 변환
	unsigned char v = 0;
	if (activeHigh) v = onLogical ? 1 : 0;
	else            v = onLogical ? 0 : 1;

	// ? 여기 호출은 네 IOApi.h에 있는 "쓰기 함수"로 맞춰야 함
	long e = io.SetOutBitEx(addr, bit, v);   // (예상 시그니처)
	if (e != ErrorCode::None) return false;

	return true;
}


static bool Axis2IsIdle()
{
	g_cm.GetStatus(&g_status);
	int av = (int)std::lround(g_status.axesStatus[2].actualVelocity);
	long long perr =
		(long long)g_status.axesStatus[2].posCmd -
		(long long)g_status.axesStatus[2].actualPos;

	return (std::abs(av) <= vel_idle_threshold) &&
		(std::llabs(perr) <= inpos_tol_counts);
}

// Axis2 현재 이동 방향 추정 (+1 / -1 / 0)
static int Axis2CurrentDir()
{
	g_cm.GetStatus(&g_status);
	double vcmd = g_status.axesStatus[2].velocityCmd;
	if (vcmd > vel_idle_threshold) return +1;
	if (vcmd < -vel_idle_threshold) return -1;
	return 0;
}

// Axis2 감속/저속 전환: 홈센서 ON 때 빠르게 감속하여 1000pps 수준으로 낮춤
static void Axis2HomeSoftDecelTo500() {
	if (!g_commStarted) return;

	// 홈센서 대응은 "움직이고 있을 때만" 수행. 정지 상태면 무시
	if (Axis2IsIdle()) return;

	if (!EnsureServoOn(2) || !EnsurePosModeNoStop(2)) return;

	// 현재 위치/속도
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[2].actualPos;

	int dir = Axis2CurrentDir();
	if (dir == 0) dir = +1; // 정지에 가까우면 +쪽으로 소폭

	// 가까운 소타겟으로 감속: 2만 펄스 앞(또는 뒤) 지점
	long long smallStep = 5000 * dir;
	long long softTarget = cur + smallStep;

	// 감속을 빠르게: Dec 시간을 짧게(예: 30ms), 목표속도 5000pps로 전환
	double newVel = 500.0; // 요청사항
	double accMs = 80.0;
	double decMs = 10.0;

	// 1단계: 가까운 점까지 빠르게 감속
	{
		Motion::PosCommand pc{};
		pc.axis = 2;
		pc.target = softTarget;
		pc.profile.type = ProfileType::SCurve;
		pc.profile.velocity = (int)std::lround(newVel);
		pc.profile.acc = TimeMsToAcc(pc.profile.velocity, accMs);
		pc.profile.dec = TimeMsToAcc(pc.profile.velocity, decMs);
		g_cm.motion->StartPos(&pc);
	}
	g_ax2HomeRampIssued = true;
}

// Axis2 리밋 상태에서 -방향 명령 금지 확인
static bool Axis2IsMinusCommandBlocked(long long currentPos, long long targetPos, long long stepOrSign) {
	// 리밋 블로킹 상태가 아닐 때 허용
	if (!g_ax2LimitBlocking.load()) return false;

	// 조그: stepOrSign이 +1/-1 로 들어온다고 보고 음수면 차단
	if (stepOrSign == +1 || stepOrSign == -1) {
		return (stepOrSign < 0);
	}
	// 절대/상대: 타깃이 현재 위치보다 작은 방향이면 음(-) 방향
	long long delta = targetPos - currentPos;
	return (delta < 0);
}

// Axis2 마이너스 금지 시 경고
static void Axis2ShowMinusBlockedWarning(HWND hWnd) {
	MessageBox(hWnd, TEXT("Axis2: Limit 센서 ON 상태입니다.\r\n-방향 명령은 허용되지 않습니다."), TEXT("Axis2 보호"), MB_ICONWARNING | MB_OK);
}

// ---------------------------------------------------------------------
// Axis0 Limit 블록 검사 + 경고 (L=+ 방향 차단, R=- 방향 차단)
// ---------------------------------------------------------------------
static bool Axis0IsCommandBlocked(long long currentPos,
	long long targetPos,
	long long stepOrSign)
{
	bool blockPlus = g_ax0BlockPlus.load();
	bool blockMinus = g_ax0BlockMinus.load();

	if (!blockPlus && !blockMinus) return false;

	// 조그 명령: stepOrSign = +1 / -1
	if (stepOrSign == +1 || stepOrSign == -1) {
		if (stepOrSign > 0 && blockPlus)  return true; // + 방향 조그 차단
		if (stepOrSign < 0 && blockMinus) return true; // - 방향 조그 차단
		return false;
	}

	// 일반 이동: targetPos 기준으로 방향 판단
	long long delta = targetPos - currentPos;
	if (delta > 0 && blockPlus)  return true; // + 방향 이동 차단
	if (delta < 0 && blockMinus) return true; // - 방향 이동 차단

	return false;
}

static void Axis0ShowBlockedWarning(HWND hWnd, int dirSign)
{
	if (dirSign > 0) {
		MessageBox(hWnd,
			TEXT("Axis0: Left Limit 센서 ON 상태입니다.\r\n+ 방향 명령은 허용되지 않습니다."),
			TEXT("Axis0 보호"),
			MB_ICONWARNING | MB_OK);
	}
	else if (dirSign < 0) {
		MessageBox(hWnd,
			TEXT("Axis0: Right Limit 센서 ON 상태입니다.\r\n- 방향 명령은 허용되지 않습니다."),
			TEXT("Axis0 보호"),
			MB_ICONWARNING | MB_OK);
	}
}

// TCP Server globals
static std::atomic<bool> g_tcpRunning{ false };
static std::thread g_tcpThread;
static SOCKET g_listenSock = INVALID_SOCKET;
static SOCKET g_clientSock = INVALID_SOCKET;
static HWND g_hMainWnd = nullptr;
static HWND g_hTcpLogList = nullptr;
static wchar_t g_tcpBindIp[64] = L"0.0.0.0";
static int g_tcpBindPort = 9100;

#define WM_APP_TCP_LOG (WM_APP + 101)
#define WM_APP_TCP_STATE (WM_APP + 102)
#define WM_APP_SHOW_DEMO_MIN (WM_APP + 103)   // ★ 추가
#define WM_APP_MAP_GOTO (WM_APP + 201)

struct MapGotoParam { long long target; };

// E-Stop 상태
static std::atomic<bool> g_estopActive{ false };

// ECAT 전역
static Ecat g_ecat(&g_wmx);

// Jog 상태
static int g_jogActiveAxis = -1;
static int g_jogActiveSign = 0;
static bool g_multiJogActive = false;
static int g_multiJogSign = 0;
static bool g_multiJogAxisActive[kNumAxes - 3] = {};

static int g_lastCmdVel[kNumAxes - 3] = {};
static int g_lastCmdTrq[kNumAxes - 3] = {};

// Demo 상태
static std::atomic<bool> g_demoRunning{ false };

// ==== Sync 창 전역 핸들
static HWND g_hSyncWnd = nullptr;



// Forward decl
static bool IsAxisChecked(HWND hWnd, int axis);
static void SetAxisChecked(HWND hWnd, int axis, bool checked);
static void UpdateSelectedAxesTextOnDemand(HWND hWnd);
static void DoAbsMoveAxis(HWND hWnd, int axis);
static void DoRelMoveAxis(HWND hWnd, int axis, int dir);
static void DoMultiAbs(HWND hWnd);
static void DoMultiRel(HWND hWnd);
static void DoMultiAlarmReset(HWND hWnd);

static void DisableAllEnabledSyncGroups();

// ------------------ 유틸 ------------------
static double GetDlgDouble(HWND h, int id, double def) {
	wchar_t buf[64] = {};
	HWND he = GetDlgItem(h, id);
	if (!he) return def;
	GetWindowTextW(he, buf, (int)std::size(buf));
	if (buf[0] == 0) return def;
	return _wtof(buf);
}
static int GetDlgInt(HWND h, int id, int def) {
	wchar_t buf[64] = {};
	HWND he = GetDlgItem(h, id);
	if (!he) return def;
	GetWindowTextW(he, buf, (int)std::size(buf));
	if (buf[0] == 0) return def;
	return _wtoi(buf);
}
static void SetDlgDouble(HWND h, int id, double v) {
	wchar_t buf[64]; _snwprintf_s(buf, _TRUNCATE, L"%.6f", v);
	HWND he = GetDlgItem(h, id);
	if (he) SetWindowTextW(he, buf);
}
static void SetDlgInt(HWND h, int id, int v) {
	wchar_t buf[64]; _snwprintf_s(buf, _TRUNCATE, L"%d", v);
	HWND he = GetDlgItem(h, id);
	if (he) SetWindowTextW(he, buf);
}

// ------------------ NEW: robust edit parsing + UI update guard (Selected Axis Control) ------------------
// _wtof() returns 0.0 even on invalid strings (e.g. "-" while typing). For "live save" we must avoid
// clobbering stored parameters with accidental zeros. This helper returns `def` unless the whole text
// is a valid number (leading/trailing spaces are allowed).
static double GetDlgDoubleOrDefIfInvalid(HWND h, int id, double def) {
	wchar_t buf[128] = {};
	HWND he = GetDlgItem(h, id);
	if (!he) return def;

	GetWindowTextW(he, buf, (int)std::size(buf));

	const wchar_t* p = buf;
	while (*p && iswspace(*p)) ++p;
	if (*p == 0) return def;

	wchar_t* end = nullptr;
	double v = wcstod(p, &end);
	if (end == p) return def;

	while (*end && iswspace(*end)) ++end;
	if (*end != 0) return def;

	return v;
}

// Selected Axis Control UI는 프로그램이 값을 "로드"할 때도 EN_CHANGE/CBN_SELCHANGE가 발생합니다.
// 로드 중에는 이러한 notify를 무시하기 위해 depth-based guard를 둡니다.
static int g_selUiUpdateDepth = 0;
struct SelUiUpdateGuard {
	SelUiUpdateGuard() { ++g_selUiUpdateDepth; }
	~SelUiUpdateGuard() { --g_selUiUpdateDepth; }
};
static bool IsSelUiUpdating() { return g_selUiUpdateDepth > 0; }

// ------------------ ECAT 0x6063/0x603F 읽기 함수 ------------------
const int kAxisSlaveId[kNumAxes] = { 0,1,2,3,4,5,6,7,8,9,10,11 };

static const unsigned short kIdx6063 = 0x6002;
static const unsigned char kSubIdx6063 = 0x01;

static const unsigned short kIdx603F = 0x603F;
static const unsigned char kSubIdx603F = 0x00;

bool ReadAxis_TxPDO_6063(int slaveId, int& outVal) {
	if (!g_deviceOpened || !g_commStarted) return false;
	unsigned char buf[8] = {};
	unsigned int actual = 0;
	long e = g_ecat.PdoRead(slaveId, kIdx6063, kSubIdx6063, (unsigned int)sizeof(buf), buf, &actual);
	if (e != ErrorCode::None) return false;
	if (actual < 4) return false;
	int32_t v = (int32_t)((uint32_t)buf[0]
		| ((uint32_t)buf[1] << 8)
		| ((uint32_t)buf[2] << 16)
		| ((uint32_t)buf[3] << 24));
	outVal = (int)v;
	return true;
}

static bool ReadAxis_TxPDO_603F(int slaveId, int& outVal) {
	if (!g_deviceOpened || !g_commStarted) return false;
	unsigned char buf[8] = {};
	unsigned int actual = 0;
	long e = g_ecat.PdoRead(slaveId, kIdx603F, kSubIdx603F, (unsigned int)sizeof(buf), buf, &actual);
	if (e != ErrorCode::None) return false;
	if (actual >= 4) {
		int32_t v = (int32_t)((uint32_t)buf[0]
			| ((uint32_t)buf[1] << 8)
			| ((uint32_t)buf[2] << 16)
			| ((uint32_t)buf[3] << 24));
		outVal = (int)v;
		return true;
	}
	else if (actual >= 2) {
		uint16_t v16 = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
		outVal = (int)(uint32_t)v16;
		return true;
	}
	return false;
}

static bool ReadAxis0_TxPDO_6063(int& outVal) {
	return ReadAxis_TxPDO_6063(kAxisSlaveId[9], outVal);
}

// ======================== 인터락 보조 함수들 ========================

// 현재 위치(주행) 코드: Load=0x01, Unload=0x02, 그 외=0x00
inline unsigned char CalcPosTravelCode()
{
	int bc = 0;
	if (!ReadAxis0_TxPDO_6063(bc)) return 0x00;
	if (std::llabs((long long)bc - 476774) <= 10) return 0x01; // Load
	if (std::llabs((long long)bc - 491332) <= 10) return 0x02; // Unload
	return 0x00;
}

// Axis2 리밋 센서 현재 상태
inline bool IsAxis2LimitOn()
{
	return ReadInputBit(AX2_LIMIT_ADDR, AX2_LIMIT_BIT, AX2_LIMIT_ACTIVE_HIGH);
}

// Auto 모드일 때만 적용되는 인터락: Axis0(주행) 시작 가능 여부
static bool CanAxis0Move_Auto()
{
	// 요구: Axis2 리밋 센서 ON일 때만 Axis0 동작 가능
	return IsAxis2LimitOn();
}

// Auto 모드일 때만 적용되는 인터락: Axis2(상하) 시작 가능 여부
static bool CanAxis2Move_Auto()
{
	// 요구: 주행 위치가 Load/Unload 위치일 때만 Axis2 동작 가능
	unsigned char code = CalcPosTravelCode();
	return (code == 0x01 || code == 0x02);
}

// Auto 모드에서 Axis0/Axis2 인터락 메시지
static void ShowAxis0InterlockMsg()
{
	MessageBox(g_hMainWnd ? g_hMainWnd : nullptr,
		TEXT("인터락: Axis2 리밋 센서가 ON일 때만 주행축(Axis0) 동작 가능합니다."),
		TEXT("Interlock (Axis0)"), MB_ICONWARNING);
}
static void ShowAxis2InterlockMsg()
{
	MessageBox(g_hMainWnd ? g_hMainWnd : nullptr,
		TEXT("인터락: 주행부가 Load/Unload 위치에 있을 때만 상하축(Axis2) 동작 가능합니다."),
		TEXT("Interlock (Axis2)"), MB_ICONWARNING);
}

// Auto 모드에서 Axis0/Axis2의 이동/조그/상대/절대 명령에 대해 공통 검사
static bool CheckInterlockBeforeAxisCommand(int axis)
{
	if (!g_autoMode.load()) return true; // 인터락은 Auto 모드에서만

	if (axis == 0) {
		if (!CanAxis0Move_Auto()) { ShowAxis0InterlockMsg(); return false; }
	}
	if (axis == 2) {
		if (!CanAxis2Move_Auto()) { ShowAxis2InterlockMsg(); return false; }
	}
	return true;
}

// ===================================================================

static bool StartRelMoveWithProfile(int axis, long long delta, double vpps, double tAcc, double tDec) {

	// Axis0/Axis2 인터락
	//if (!CheckInterlockBeforeAxisCommand(axis)) return false;

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;
	long long tgt = cur + delta;

	//// Axis2 보호: 리밋 ON 시 -방향 금지
	//if (axis == 2 && g_commStarted) {
	//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)((delta >= 0) ? +1 : -1))) {
	//		if (g_hMainWnd) Axis2ShowMinusBlockedWarning(g_hMainWnd);
	//		return false;
	//	}
	//}

	//// Axis0 보호: L/R 리밋에 따른 방향 차단
	//if (axis == 0 && g_commStarted) {
	//	long long sign = (delta >= 0) ? +1 : -1;
	//	if (Axis0IsCommandBlocked(cur, tgt, sign)) {
	//		if (g_hMainWnd) Axis0ShowBlockedWarning(g_hMainWnd, (int)sign);
	//		return false;
	//	}
	//}

	Motion::PosCommand pc; pc.axis = axis; pc.target = tgt;
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);
	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos(Rel) 실패"), e, g_wmx); return false; }

	// CHANGED: Enable logging for this axis and mark new command
	g_axisCmdInfo[axis].axis = axis;
	g_axisCmdInfo[axis].target = tgt;
	g_axisCmdInfo[axis].vel = pc.profile.velocity;
	g_axisCmdInfo[axis].acc = pc.profile.acc;
	g_axisCmdInfo[axis].dec = pc.profile.dec;
	g_axisCmdInfo[axis].startTick = GetTickCount64();
	g_axisCmdInfo[axis].endTick = 0;
	g_axisCmdInfo[axis].active = true;
	g_axisLogEnabled[axis] = true;
	g_axisLogRowIdx[axis] = 0;

	return true;
}

// ------------------ 컨트롤 ID ------------------
inline int ID_EDIT_POS_A(int axis) { return 1000 + axis; }
inline int ID_EDIT_VEL_A(int axis) { return 1100 + axis; }
inline int ID_EDIT_ACCT_A(int axis) { return 1200 + axis; }
inline int ID_EDIT_DECT_A(int axis) { return 1300 + axis; }
inline int ID_BTN_SVON_A(int axis) { return 2000 + axis; }
inline int ID_BTN_SVOFF_A(int axis) { return 2100 + axis; }
inline int ID_BTN_HOME_A(int axis) { return 2200 + axis; }
inline int ID_BTN_ABS_A(int axis) { return 2300 + axis; }
inline int ID_BTN_REL_A(int axis) { return 2400 + axis; }
inline int ID_BTN_JOGP_A(int axis) { return 2500 + axis; }
inline int ID_BTN_JOGM_A(int axis) { return 2600 + axis; }
inline int ID_BTN_STOP_A(int axis) { return 2700 + axis; }

inline int ID_EDIT_ALTTGT_A(int axis) { return 3000 + axis; }
inline int ID_BTN_APPLY_ALT_A(int axis) { return 3100 + axis; }

// Selected checkboxes: 확대(0..8)
#define ID_CHECK_AXIS_BASE 5000
inline int ID_CHECK_AXIS(int axis) { return ID_CHECK_AXIS_BASE + axis; }

#define ID_BTN_MULTI_ABS 6000
#define ID_BTN_MULTI_REL 6001
#define ID_BTN_MULTI_STOP 6002
#define ID_BTN_MULTI_JOGP 6003
#define ID_BTN_MULTI_JOGM 6004
#define ID_BTN_MULTI_SVON 6005
#define ID_BTN_MULTI_SVOFF 6006
#define ID_BTN_SELECT_ALL 6007
#define ID_BTN_CLEAR_ALL 6008
#define ID_BTN_MULTI_ALARM_RST 6009
#define ID_BTN_MULTI_HOME 6010

#define ID_BTN_CREATE_DEVICE 7000
#define ID_BTN_START_COMM 7001
#define ID_BTN_DEMO2 7003

#define ID_BTN_SYNC_WINDOW 7100

#define ID_BTN_FASTECH_MANUAL 7200
#define ID_BTN_WELCON_MANUAL 7201

#define ID_BTN_ESTOP_TOGGLE 7250
#define ID_TXT_ESTOP_STATE 7251

// NEW: Manual/Auto mode buttons and label
#define ID_BTN_MODE_MANUAL 7260
#define ID_BTN_MODE_AUTO   7261
#define ID_TXT_MODE_STATE  7262

#define ID_BTN_VELCONV_MAIN 7270
inline int ID_TXT_STATUS(int axis, int col) { return 8000 + axis * 30 + col; }
#define ID_TXT_SELECTED_AXES 9000
#define ID_TXT_ZONE_ANNOUNCE 9100
#define ID_TXT_MOTION_ANNOUNCE 9101

#define ID_TIMER 1

#define ID_TXT_ECAT_6063 9150
#define ID_TXT_ECAT_603F 9151

// Axis2 Limit / Home 상태 표시용 텍스트
#define ID_TXT_AX2_LIMIT  9152
#define ID_TXT_AX2_HOME   9153
#define ID_TXT_AX0_LIMIT_L  9154
#define ID_TXT_AX0_LIMIT_R  9155

// ==== Serial Monitor Window (separate) ====
#define ID_EDIT_TCP_IP 9300
#define ID_EDIT_TCP_PORT 9301
#define ID_BTN_TCP_START 9302
#define ID_BTN_TCP_STOP 9303
#define ID_LIST_TCP_LOG 9304
#define ID_BTN_SERIAL_WINDOW 9400
#define ID_TXT_TCP_STATE 9450
#define ID_BTN_LOG_WINDOW 9600
#define ID_BTN_SCOPE_WINDOW 9601

// ==== Sync 창 컨트롤 ID ====
enum : int {
	ID_SYNC_TIMER = 10001,
	ID_SYNC_GROUP_COMBO = 10002,

	ID_SYNC_PARAM_ENABLE = 10010,
	ID_SYNC_PARAM_DISABLE = 10011,
	ID_SYNC_PARAM_REFRESH = 10012,

	ID_SYNC_PARAM_SERVO_ONOFF = 10100,
	ID_SYNC_PARAM_STARTUP = 10101,
	ID_SYNC_PARAM_DESYNC = 10102, // placeholder label
	ID_SYNC_PARAM_CYCLE_RATIO = 10103,
	ID_SYNC_PARAM_MAX_CATCH_UP = 10104,
	ID_SYNC_PARAM_CATCHUP_VEL = 10105,
	ID_SYNC_PARAM_CATCHUP_ACC = 10106,
	ID_SYNC_PARAM_TOLERANCE = 10107,

	// NOTE: no grp.desyncDec in SyncGroup. Keep an edit box to set Config::SyncParam.masterDesyncDec / slaveDesyncDec
	ID_SYNC_PARAM_MASTER_DESYNC_DEC = 10108,
	ID_SYNC_PARAM_SLAVE_DESYNC_DEC = 10109,

	ID_SYNC_PARAM_USE_MASTER_FB = 10110,
	ID_SYNC_PARAM_AMP_ERR_SVON = 10111,

	ID_SYNC_RAD_MASTER = 10200,
	ID_SYNC_RAD_SLAVE = 10201,
	ID_SYNC_AXIS_SELECT = 10202,
	ID_SYNC_POS_CMD = 10203,
	ID_SYNC_POS_ACT = 10204,
	ID_SYNC_OPSTATE = 10205,
	ID_SYNC_BTN_SVON = 10206,
	ID_SYNC_BTN_HOME = 10207,
	ID_SYNC_BTN_STOP = 10208,
	ID_SYNC_BTN_ALARM_RST = 10209,

	ID_SYNC_JOG_SPEED = 10300,
	ID_SYNC_ACC = 10301,
	ID_SYNC_DEC = 10302,
	ID_SYNC_JERK = 10303,
	ID_SYNC_CMD_VEL = 10304,
	ID_SYNC_ACT_VEL = 10305,
	ID_SYNC_BTN_JOG_CCW = 10306,
	ID_SYNC_BTN_JOG_CW = 10307,
	ID_SYNC_ABS_POS = 10308,
	ID_SYNC_BTN_ABS = 10309,
	ID_SYNC_REL_STEP = 10310,
	ID_SYNC_BTN_REL_P = 10311,
	ID_SYNC_BTN_REL_M = 10312,

	ID_SYNC_ALT_TARGET = 10350,
	ID_SYNC_BTN_APPLY_ALT = 10351,
	ID_SYNC_ECAT_6063 = 10352,
	ID_SYNC_ECAT_603F = 10353,

	ID_SYNC_BTN_FASTECH_MANUAL = 10360,
	ID_SYNC_BTN_WELCON_MANUAL = 10361,

	ID_SYNC_STATE_ENABLED = 10400,
	ID_SYNC_STATE_HOMEDONE = 10401,
	ID_SYNC_ALL_SERVO_ON = 10402,
	ID_SYNC_ALL_SERVO_OFF = 10403,
	ID_SYNC_GROUP_HOME = 10404,
	ID_SYNC_GROUP_CLEAR = 10405,
	ID_SYNC_BTN_VELCONV = 10406,

	ID_SYNC_AXIS_LIST = 10450,

	ID_SYNC_MASTER_AXIS_BTN_BASE = 12000,
	ID_SYNC_SLAVE_AXIS_CHK_BASE = 13000,

	ID_SYNC_BTN_ESTOP_TOGGLE = 14000,
	ID_SYNC_TXT_ESTOP_STATE = 14001
};

// ==== ActualVel(rpm) -> m/s conversion settings window ====
// Default: all axes disabled. User can enable per-axis conversion from UI.
// NOTE: WMX3 axis config in this program auto-applies gear ratio at startup.
//       Therefore, do NOT add gear ratio in this conversion formula.

static HWND g_hVelConvWnd = nullptr;

// Axis count shown in Sync window is (kNumAxes - 3) ...
static const int kSyncUiAxes = (kNumAxes - 3);

// Per-axis flags/params (0..8). Default off.
static bool   g_velToMS_Enable[kSyncUiAxes] = {};
static double g_velToMS_WheelDiameterMM[kSyncUiAxes] = {};
static int    g_velToMS_PulsesPerRev[kSyncUiAxes] = {};

// Defaults requested by user for axis7 (still disabled by default)
static void InitVelConvDefaultsOnce() {
	static bool inited = false;
	if (inited) return;
	inited = true;
	for (int i = 0; i < kSyncUiAxes; ++i) {
		g_velToMS_Enable[i] = false;
		g_velToMS_WheelDiameterMM[i] = 0.0;
		g_velToMS_PulsesPerRev[i] = 0;
	}
	if (7 >= 0 && 7 < kSyncUiAxes) {
		g_velToMS_WheelDiameterMM[7] = 115.0;
		g_velToMS_PulsesPerRev[7] = 10000;
	}
}

static double RpmToMps(double rpm, double wheelDiameterMM) {
	// rpm -> rps: rpm/60
	// distance per rev: pi * D
	if (!(wheelDiameterMM > 0.0)) return NAN;
	double Dm = wheelDiameterMM / 1000.0;
	constexpr double kPi = 3.1415926535897932384626433832795;
	return (rpm / 60.0) * (kPi * Dm);
}

// VelConv window control IDs
#define ID_VELCONV_AXIS_CHK_BASE   15000
#define ID_VELCONV_DIAM_EDIT_BASE  15100
#define ID_VELCONV_PPR_EDIT_BASE   15200
#define ID_VELCONV_BTN_APPLY       15300
#define ID_VELCONV_BTN_CLOSE       15301

static LRESULT CALLBACK VelConvWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
static void ShowVelConvWindow(HWND hParent);

// ======================================================

// UI 헬퍼
struct FieldPos { int endX; };
static FieldPos TightLabeledEdit2(HWND parent, int x, int y,
	const TCHAR* lab, int labelEditGap,
	int editW, int cid, const TCHAR* init,
	const TCHAR* unit = nullptr, int editUnitGap = 10) {
	HDC hdc = GetDC(parent);
	HFONT hFont = (HFONT)SendMessage(parent, WM_GETFONT, 0, 0);
	HFONT hOld = (HFONT)SelectObject(hdc, hFont);
	RECT rc = { 0,0,0,0 };
#ifdef UNICODE
	DrawTextW(hdc, lab, -1, &rc, DT_CALCRECT);
#else
	DrawText(hdc, lab, -1, &rc, DT_CALCRECT);
#endif
	int lw = rc.right - rc.left; if (lw < 30) lw = 30;
	SelectObject(hdc, hOld); ReleaseDC(parent, hdc);

	CreateWindow(TEXT("STATIC"), lab, WS_CHILD | WS_VISIBLE, x, y, lw, 20, parent, nullptr, nullptr, nullptr);
	int ex = x + lw + labelEditGap;
	CreateWindow(TEXT("EDIT"), init, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
		ex, y - 2, editW, 24, parent, (HMENU)(INT_PTR)cid, nullptr, nullptr);
	int endX = ex + editW;

	if (unit && unit[0]) {
		// unit 텍스트 폭을 실제로 계산해서(고정 70px 제거) 간격을 더 촘촘하게 만듭니다.
		HDC hdc2 = GetDC(parent);
		HFONT hFont2 = (HFONT)SendMessage(parent, WM_GETFONT, 0, 0);
		HFONT hOld2 = (HFONT)SelectObject(hdc2, hFont2);
		RECT urc = { 0,0,0,0 };
#ifdef UNICODE
		DrawTextW(hdc2, unit, -1, &urc, DT_CALCRECT);
#else
		DrawText(hdc2, unit, -1, &urc, DT_CALCRECT);
#endif
		int uw = (urc.right - urc.left);
		if (uw < 12) uw = 12;      // 너무 붙지 않게 최소폭
		if (uw > 60) uw = 60;      // 너무 길면 제한
		SelectObject(hdc2, hOld2);
		ReleaseDC(parent, hdc2);

		const int unitW = uw + 6;  // 약간의 여백
		CreateWindow(TEXT("STATIC"), unit, WS_CHILD | WS_VISIBLE, endX + editUnitGap, y, unitW, 20, parent, nullptr, nullptr, nullptr);
		endX += editUnitGap + unitW;
	}
	return { endX };
}

// ------------------ 조그 ------------------
// NOTE: oht_wongwang.cpp의 "누르는 동안만 jog / 떼면 stop" 동작을 그대로 가져오기 위해
// 버튼을 subclass 해서 WM_LBUTTONDOWN/UP로 조그를 제어합니다(= BS_NOTIFY 의존 X).

enum class JogBtnKind { SelectedGroup, MultiChecked, Sync };

struct JogBtnCtx {
	JogBtnKind kind;
	int groupIdx; // SelectedGroup: 0..3, others: -1
	int sign;     // +1 / -1
};

// Forward decl (impl is placed later)
static void AttachHoldToJogButton(HWND hBtn, JogBtnKind kind, int groupIdx, int sign);

static bool StartJog(HWND hWnd, int axis, int sign) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return false; }

	//// Auto 모드 인터락: Axis0/2 조그 시작 전 검사
	//if (g_autoMode.load()) {
	//	if (!CheckInterlockBeforeAxisCommand(axis)) return false;
	//}

	//if (axis == 2) {
	//	// Axis2는 Limit ON 시 -방향 차단
	//	g_cm.GetStatus(&g_status);
	//	long long curPos = (long long)g_status.axesStatus[2].actualPos;
	//	if (Axis2IsMinusCommandBlocked(curPos, curPos, sign)) {
	//		Axis2ShowMinusBlockedWarning(hWnd);
	//		return false;
	//	}
	//}

	//if (axis == 0) {
	//	// Axis0: L/R 리밋 방향 차단
	//	g_cm.GetStatus(&g_status);
	//	long long curPos = (long long)g_status.axesStatus[0].actualPos;
	//	if (Axis0IsCommandBlocked(curPos, curPos, (long long)sign)) {
	//		Axis0ShowBlockedWarning(hWnd, sign);
	//		return false;
	//	}
	//}

	//DisableAllEnabledSyncGroups();
	if (!EnsureServoOn(axis)) return false;
	if (!EnsurePosModeNoStop(axis)) return false;

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;

	double vpps = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 10000.0);
	double tAcc = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
	double tDec = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);

	Motion::PosCommand pc;
	pc.axis = axis;
	pc.target = cur + (long long)(sign * 1000000000LL);
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);

	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos(JOG) 실패"), e, g_wmx); return false; }

	// 로그 트래킹 및 enable
	g_axisCmdInfo[axis].axis = axis;
	g_axisCmdInfo[axis].target = pc.target;
	g_axisCmdInfo[axis].vel = pc.profile.velocity;
	g_axisCmdInfo[axis].acc = pc.profile.acc;
	g_axisCmdInfo[axis].dec = pc.profile.dec;
	g_axisCmdInfo[axis].startTick = GetTickCount64();
	g_axisCmdInfo[axis].endTick = 0;
	g_axisCmdInfo[axis].active = true;
	g_axisLogEnabled[axis] = true; // CHANGED
	g_axisLogRowIdx[axis] = 0; // CHANGED

	g_lastCmdVel[axis] = (int)std::lround(pc.profile.velocity * sign);
	g_jogActiveAxis = axis;
	g_jogActiveSign = sign;
	SetCapture(hWnd);
	return true;
}
static void StopJogIfActive() {
	if (g_jogActiveAxis >= 0) {
		g_cm.motion->Stop(g_jogActiveAxis);
		g_cm.velocity->Stop(g_jogActiveAxis);
		if (g_cm.torque) g_cm.torque->StopTrq(g_jogActiveAxis);
		g_jogActiveAxis = -1;
		g_jogActiveSign = 0;
	}
}
static bool StartMultiJog(HWND hWnd, int sign) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return false; }
	//DisableAllEnabledSyncGroups();
	bool any = false; for (int a = 0; a < kNumAxes - 3; ++a) g_multiJogAxisActive[a] = false;

	g_cm.GetStatus(&g_status);
	for (int a = 0; a < kNumAxes - 3; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;

		// Auto 모드 인터락: Axis0/2에만 적용
		if (g_autoMode.load()) {
			//if (!CheckInterlockBeforeAxisCommand(a)) continue;
		}

		//// Axis2 보호: 리밋 ON시 -방향 JOG 차단
		//if (a == 2) {
		//	long long cur = (long long)g_status.axesStatus[2].actualPos;
		//	long long tgt = cur + (long long)(sign * 1000000000LL);
		//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)sign)) {
		//		Axis2ShowMinusBlockedWarning(hWnd);
		//		continue;
		//	}
		//}

		//// Axis0 보호: L/R 리밋에 따른 방향 차단
		//if (a == 0) {
		//	long long cur = (long long)g_status.axesStatus[0].actualPos;
		//	long long tgt = cur + (long long)(sign * 1000000000LL);
		//	if (Axis0IsCommandBlocked(cur, tgt, (long long)sign)) {
		//		Axis0ShowBlockedWarning(hWnd, sign);
		//		continue;
		//	}
		//}

		if (!EnsureServoOn(a)) continue;
		if (!EnsurePosModeNoStop(a)) continue;

		long long cur = (long long)g_status.axesStatus[a].actualPos;
		double vpps = GetDlgDouble(hWnd, ID_EDIT_VEL_A(a), 10000.0);
		double tAcc = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(a), 100.0);
		double tDec = GetDlgDouble(hWnd, ID_EDIT_DECT_A(a), 100.0);

		Motion::PosCommand pc; pc.axis = a; pc.target = cur + (long long)(sign * 1000000000LL);
		pc.profile.type = ProfileType::SCurve; pc.profile.velocity = (int)std::lround(vpps);
		pc.profile.acc = TimeMsToAcc(vpps, tAcc); pc.profile.dec = TimeMsToAcc(vpps, tDec);
		long e = g_cm.motion->StartPos(&pc);
		if (e == ErrorCode::None) {
			g_lastCmdVel[a] = (int)std::lround((double)pc.profile.velocity * sign);
			g_multiJogAxisActive[a] = true; any = true;

			// enable logging per-axis
			g_axisCmdInfo[a].axis = a;
			g_axisCmdInfo[a].target = pc.target;
			g_axisCmdInfo[a].vel = pc.profile.velocity;
			g_axisCmdInfo[a].acc = pc.profile.acc;
			g_axisCmdInfo[a].dec = pc.profile.dec;
			g_axisCmdInfo[a].startTick = GetTickCount64();
			g_axisCmdInfo[a].endTick = 0;
			g_axisCmdInfo[a].active = true;
			g_axisLogEnabled[a] = true; // CHANGED
			g_axisLogRowIdx[a] = 0; // CHANGED
		}
		else ShowErrMsgBox(TEXT("StartPos(Multi Jog) 실패"), e, g_wmx);
	}
	if (any) { g_multiJogActive = true; g_multiJogSign = sign; SetCapture(hWnd); return true; }
	g_multiJogActive = false; return false;
}
static void StopMultiJog() {
	if (!g_multiJogActive) return;
	for (int a = 0; a < kNumAxes - 3; ++a) if (g_multiJogAxisActive[a]) {
		g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a); g_multiJogAxisActive[a] = false;
	}
	g_multiJogActive = false; g_multiJogSign = 0;
}

// ------------------ E-Stop 토글 공용 함수 ------------------
static void UpdateEStopUi(HWND hWnd, bool syncWindow)
{
	if (syncWindow) {
		if (HWND h = GetDlgItem(hWnd, ID_SYNC_TXT_ESTOP_STATE))
			SetWindowText(h, g_estopActive.load() ? TEXT("E-STOP ACTIVE") : TEXT("NORMAL"));
		if (HWND b = GetDlgItem(hWnd, ID_SYNC_BTN_ESTOP_TOGGLE))
			SetWindowText(b, g_estopActive.load() ? TEXT("비상정지해제") : TEXT("비상정지"));
	}
	else {
		if (HWND h = GetDlgItem(hWnd, ID_TXT_ESTOP_STATE))
			SetWindowText(h, g_estopActive.load() ? TEXT("E-STOP ACTIVE") : TEXT("NORMAL"));
		if (HWND b = GetDlgItem(hWnd, ID_BTN_ESTOP_TOGGLE))
			SetWindowText(b, g_estopActive.load() ? TEXT("비상정지해제") : TEXT("비상정지"));
	}
}

static void DoToggleEStop(HWND hWnd, bool syncWindow)
{
	if (!g_deviceOpened || !g_commStarted) {
		MessageBox(hWnd, TEXT("먼저 Device 생성 및 통신을 시작하세요."), TEXT("E-Stop"), MB_ICONWARNING);
		return;
	}
	if (!g_estopActive.load()) {
		long e = g_cm.ExecEStop(EStopLevel::Final);
		if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("ExecEStop 실패"), e, g_wmx); return; }
		g_estopActive = true;
	}
	else {
		long e = g_cm.ReleaseEStop();
		if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("ReleaseEStop 실패"), e, g_wmx); return; }
		g_estopActive = false;
	}
	UpdateEStopUi(hWnd, syncWindow);
}

bool StartAbsMoveWithProfile(int axis, long long target, double vpps, double tAcc, double tDec) {

	// Auto 모드 인터락: Axis0/2 이동 전 검사
	//if (!CheckInterlockBeforeAxisCommand(axis)) return false;

	//// Axis2 보호 체크: 리밋 ON시 -방향 금지
	//if (axis == 2 && g_commStarted) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[2].actualPos;
	//	if (Axis2IsMinusCommandBlocked(cur, target, 0)) {
	//		if (g_hMainWnd) Axis2ShowMinusBlockedWarning(g_hMainWnd);
	//		return false;
	//	}
	//}

	//// Axis0 보호 체크: L/R 리밋에 따른 방향 금지
	//if (axis == 0 && g_commStarted) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[0].actualPos;
	//	long long delta = target - cur;
	//	long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
	//	if (sign != 0 && Axis0IsCommandBlocked(cur, target, sign)) {
	//		if (g_hMainWnd) Axis0ShowBlockedWarning(g_hMainWnd, (int)sign);
	//		return false;
	//	}
	//}

	Motion::PosCommand pc; pc.axis = axis; pc.target = target;
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);
	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos 실패"), e, g_wmx); return false; }

	// 고정된 시작 시간 및 active 설정 + CHANGED: enable per-axis sampling
	g_axisCmdInfo[axis].axis = axis;
	g_axisCmdInfo[axis].target = target;
	g_axisCmdInfo[axis].vel = pc.profile.velocity;
	g_axisCmdInfo[axis].acc = pc.profile.acc;
	g_axisCmdInfo[axis].dec = pc.profile.dec;
	g_axisCmdInfo[axis].startTick = GetTickCount64();
	g_axisCmdInfo[axis].endTick = 0;
	g_axisCmdInfo[axis].active = true;

	g_axisLogEnabled[axis] = true; // CHANGED: resume logging on new command
	g_axisLogRowIdx[axis] = 0; // reset row index for this command

	return true;
}
// ------------------ WMX3 init/shutdown ------------------
// ====== Init guards & retry helpers ======
static std::atomic<bool> g_initTried{ false };
static std::atomic<bool> g_commTried{ false };
static std::atomic<bool> g_autoStartDone{ false };
static std::atomic<bool> g_ioInitDone{ false };

static long CreateDeviceWithRetry(WMX3Api& api, const TCHAR* installPath, int maxRetry = 2, DWORD backoffMs = 300)
{
	long err = ErrorCode::None;
	for (int i = 0; i <= maxRetry; ++i) {
		err = api.CreateDevice(installPath, DeviceType::DeviceTypeNormal);
		if (err == ErrorCode::None) return err;
		// 에러 268 등 제어 채널 락 실패 시에는 CloseDevice 하고 백오프 후 재시도
		api.CloseDevice();
		Sleep(backoffMs * (i + 1));
	}
	return err;
}
static bool InitDevice() {
	// 이미 열려 있으면 OK
	if (g_deviceOpened) return true;

	// 중복 시도 방지
	bool expected = false;
	if (!g_initTried.compare_exchange_strong(expected, true)) {
		// 다른 경로에서 이미 시도 중이거나 완료됨
		// 현재 상태를 그대로 보고
		return g_deviceOpened;
	}

	// 재시도 포함하여 CreateDevice
	long e = CreateDeviceWithRetry(g_wmx, g_installPath, /*maxRetry*/2, /*backoffMs*/300);
	if (e != ErrorCode::None) {
		ShowErrMsgBox(TEXT("CreateDevice 실패"), e, g_wmx);
		g_initTried = false; // 다음에 다시 눌러볼 수 있게
		return false;
	}

	g_deviceOpened = true;
	return true;
}

// Helper: Set gear ratio for one axis (numerator/denominator)
static bool SetAxisGearRatio(int axis, double numerator, double denominator) {
	if (!g_commStarted) return false;
	long err = g_cm.config->SetGearRatio(axis, numerator, denominator);
	if (err != ErrorCode::None) {
		ShowErrMsgBox(TEXT("SetGearRatio 실패"), err, g_wmx);
		return false;
	}
	return true;
}

static bool StartComm() {
	if (!g_deviceOpened) {
		// 장치가 없으면 먼저 InitDevice
		if (!InitDevice()) return false;
	}

	if (g_commStarted) return true;

	bool expected = false;
	if (!g_commTried.compare_exchange_strong(expected, true)) {
		// 다른 경로에서 이미 시도 중이거나 완료됨
		return g_commStarted;
	}

	long e = g_wmx.StartCommunication(15000);
	if (e != ErrorCode::None) {
		ShowErrMsgBox(TEXT("StartCommunication 실패"), e, g_wmx);
		g_commTried = false; // 다음 시도 허용
		return false;
	}

	g_commStarted = true;
	g_estopActive = false;

	// 기어비 설정 등 초기 파라미터
	SetAxisGearRatio(0, 43000.0, 10000.0); // 폭조절
	SetAxisGearRatio(1, 43000.0, 10000.0); //포킹암
	SetAxisGearRatio(2, 30000.0, 10000.0); // 사이드바퀴 업다운
	SetAxisGearRatio(3, 30000.0, 10000.0); //사이드바퀴 업다운
	SetAxisGearRatio(4, 100000.0, 10000.0); //벨트업다운
	SetAxisGearRatio(5, 100000.0, 10000.0); //사이드바퀴 주행
	SetAxisGearRatio(6, 100000.0, 10000.0); //사이드바퀴 주행
	SetAxisGearRatio(7, 44247.8, 10000.0); //좌우주행
	SetAxisGearRatio(8, 44247.8, 10000.0); //좌우주행

	// Axis2 sensor flags reset
	g_ax2LimitOn = false;
	g_ax2HomeOn = false;
	g_ax2LimitLatched = false;
	g_ax2LimitBlocking = false;
	g_ax2StopIssuedOnLimit = false;
	g_ax2HomingStarted = false;
	g_ax2HomeDebounceOn = false;
	g_ax2HomeLastTick = GetTickCount();
	g_ax2HomeRampIssued = false;

	WriteOutputBit(36, 0, true, true);

	return true;
}

static void ShutdownWMX() {
	StopJogIfActive();
	StopMultiJog();

	if (g_commStarted) {
		g_wmx.StopCommunication();
		g_commStarted = false;
		g_commTried = false;
	}

	if (g_deviceOpened) {
		g_wmx.CloseDevice();
		g_deviceOpened = false;
		g_initTried = false;
	}

	g_estopActive = false;
}

static bool StartJogWithProfileParams(HWND hWnd, int axis, int sign, double vel, double tAccMs, double tDecMs);

// ------------------ 주기 상태 전송(1초) 쓰레드 [NEW] ------------------
std::atomic<bool> g_periodicRun{ false };
std::thread g_periodicThread;
std::atomic<unsigned char> g_heartbeat{ 0 };

// 운전 준비 완료 플래그 (주기 프레임에서 사용)
std::atomic<bool> g_driveReady{ false };

static inline bool BetweenTol(long long v, long long center, long long tol) {
	return (std::llabs(v - center) <= tol);
}

// 현재위치(주행) 계산: Axis0의 6063 기준
// Load=01 (278000±10), Unload=02 (253381±10), 그 외 00
unsigned char CalcPosTravelCode(); // 위에서 정의됨
//static unsigned char CalcPosTravelCode()
//{
//	int bc = 0;
//	if (!ReadAxis0_TxPDO_6063(bc)) return 0x00;
//	if (BetweenTol(bc, 278000, 10)) return 0x01; // Load
//	if (BetweenTol(bc, 253381, 10)) return 0x02; // Unload
//	return 0x00;
//}

// 현재위치(상하) 계산: Axis2 actualPos
// Load=01 (60000±10), Unload=02 (57000±10), Up=03 (0±10), 그 외 00
static unsigned char CalcPosHoistCode()
{
	if (!g_commStarted) return 0x00;
	g_cm.GetStatus(&g_status);
	long long p = (long long)g_status.axesStatus[2].actualPos;
	if (BetweenTol(p, 51600, 10)) return 0x01;
	if (BetweenTol(p, 45000, 10)) return 0x02;
	if (BetweenTol(p, 0, 10))     return 0x03;
	return 0x00;
}

unsigned char CalcPosGripCode()
{
	// 진행중이면 0x00
	if (g_gripBusy.load()) return 0x00;

	// Motioning 입력(DI0)을 참조: ON이면 동작중이므로 0x00
	// g_diStable[0] == true 면 Motioning ON으로 사용하고 있으므로, true => 동작중
	if (g_diStable[0]) return 0x00;

	// DemoControl 쪽 Gripper 판단 로직 재사용
	bool isOpen = IsGripperOpenAndIdle();
	bool isClose = IsGripperClosedAndIdle();
	bool openinit = IsGripperOpen();
	bool closeinit = IsGripperClosed();
	bool hasbox = HasBox();

	// Open만 ON
	if (isOpen && !isClose || openinit)
		return 0x01;

	// Close만 ON
	if (!isOpen && isClose || closeinit || hasbox)
		return 0x02;

	// 둘 다 OFF이거나, 둘 다 ON이거나, 판단 불가 → 0x00
	return 0x00;
}


// 알람코드(주행축): axis0/1 중 0이 아닌 603F 반환(우선 axis0)
static unsigned short GetTravelAlarm603F()
{
	int e0 = 0, e1 = 0;
	bool ok0 = ReadAxis_TxPDO_603F(kAxisSlaveId[0], e0);
	bool ok1 = ReadAxis_TxPDO_603F(kAxisSlaveId[1], e1);
	unsigned short v0 = ok0 ? (unsigned short)(e0 & 0xFFFF) : 0;
	unsigned short v1 = ok1 ? (unsigned short)(e1 & 0xFFFF) : 0;
	return v0 ? v0 : v1;
}

// 알람코드(상하축): axis2의 603F
static unsigned short GetHoistAlarm603F()
{
	int e2 = 0;
	bool ok2 = ReadAxis_TxPDO_603F(kAxisSlaveId[2], e2);
	return ok2 ? (unsigned short)(e2 & 0xFFFF) : 0;
}

static std::atomic<unsigned short> g_ohtMsgId{ 1 };

// 0x0001 ~ 0xFFFF 사용, 0x0000은 건너뜀
static unsigned short NextOhtMsgId()
{
	unsigned short cur = g_ohtMsgId.load(std::memory_order_relaxed);
	while (true) {
		unsigned short next = (cur == 0xFFFF) ? 1 : (unsigned short)(cur + 1);
		if (g_ohtMsgId.compare_exchange_weak(
			cur, next,
			std::memory_order_release,
			std::memory_order_relaxed))
		{
			return cur; // cur 값을 실제로 쓸 MsgID로 사용
		}
		// 실패하면 cur가 새 값으로 갱신되니 다시 루프 돌면서 재시도
	}
}

static void SendPeriodicStateFrame(SOCKET s)
{
	if (s == INVALID_SOCKET) return;

	unsigned short msgId = NextOhtMsgId();

	unsigned char mode = g_autoMode.load() ? 0x01 : 0x00;
	unsigned char posTravel = CalcPosTravelCode();
	unsigned char posHoist = CalcPosHoistCode();
	unsigned char posGrip = CalcPosGripCode(); // 보류
	unsigned short almTravel = GetTravelAlarm603F();
	unsigned short almHoist = GetHoistAlarm603F();
	unsigned char almGripL = 0x00, almGripH = 0x00; // 보류
	unsigned char hb = g_heartbeat.load();
	//unsigned char driveReady = g_driveReady.load() ? 0x01 : 0x00;

	// [수정] 실제 페이로드 바이트 수 계산 (11바이트)
	// mode(1) + posTravel(1) + posHoist(1) + posGrip(1)
	// + almTravel(2) + almHoist(2) + almGrip(2) + hb(1)
	const unsigned char payload_len = 0x0B; // [수정] 0x09 -> 0x0B (11)

	// [수정] 프레임 총 길이: STX(1) + MsgID(2) + Op(1) + Len(1) + Payload(payload_len) + ETX(1)
	const size_t frame_capacity = 5 + payload_len + 1;

	// [수정] 고정 크기 대신 계산된 크기로 배열 확보
	unsigned char frame[5 + 0x0B + 1] = {}; // = 5 + 12 + 1 = 18 바이트

	// Header
	frame[0] = 0x02;       // STX
	frame[1] = 0x00; // MsgID High
	frame[2] = 0x00;        // MsgID Low
	frame[3] = 0xFE;       // Operation Code: State
	frame[4] = payload_len; // [수정] 0x09 -> payload_len

	// Payload
	size_t i = 5;
	frame[i++] = mode;
	frame[i++] = posTravel;
	frame[i++] = posHoist;
	frame[i++] = posGrip;
	// 알람코드(주행) 2 bytes (LSB, MSB)
	frame[i++] = (unsigned char)(almTravel & 0xFF);
	frame[i++] = (unsigned char)((almTravel >> 8) & 0xFF);
	// 알람코드(상하) 2 bytes
	frame[i++] = (unsigned char)(almHoist & 0xFF);
	frame[i++] = (unsigned char)((almHoist >> 8) & 0xFF);
	// 알람코드(그립) 2 bytes
	frame[i++] = almGripL;
	frame[i++] = almGripH;
	// 운전준비 완료 플래그
	//frame[i++] = driveReady;
	// Heartbeat (토글 대상 값)
	frame[i++] = hb;

	// ETX
	frame[i++] = 0x03;

	// [추가] 방어적 검사: i는 frame_capacity와 같아야 함
	// (개발 중 디버그 보조용, 릴리스에서는 제거 가능)
	// assert(i == frame_capacity);

	send(s, (const char*)frame, (int)i, 0);

	// 토글
	g_heartbeat = (unsigned char)(hb ? 0x00 : 0x01);
}

// msgId는 이제 필요 없음, OHT 자체 시퀀스로 보냄
static void SendSimpleAck(SOCKET s, unsigned char reqOpCode)
{
	if (s == INVALID_SOCKET) return;

	unsigned char f[7];
	unsigned short msgId = NextOhtMsgId();

	f[0] = 0x02;                                 // STX
	f[1] = 0x00; // MsgID High
	f[2] = 0x00;        // MsgID Low
	f[3] = 0xFF;                                 // ACK OpCode
	f[4] = 0x01;                                 // Payload Length = 1
	f[5] = reqOpCode;                            // Payload: 원 요청 OpCode
	f[6] = 0x03;                                 // ETX

	send(s, (const char*)f, 7, 0);
}


static void SendSimpleDone(SOCKET s, unsigned char reqOpCode)
{
	if (s == INVALID_SOCKET) return;

	unsigned char f[7];
	unsigned short msgId = NextOhtMsgId();

	f[0] = 0x02;                                 // STX
	f[1] = 0x00; // MsgID High
	f[2] = 0x00;        // MsgID Low
	f[3] = 0x81;                                 // DONE OpCode
	f[4] = 0x01;                                 // Payload Length = 1
	f[5] = reqOpCode;                            // Payload: 원 요청 OpCode
	f[6] = 0x03;                                 // ETX

	send(s, (const char*)f, 7, 0);
}



// ---- NEW: Human-readable op name ----
static const wchar_t* DecodeOpName(unsigned char op)
{
	switch (op) {
	case 0x21: return L"Load Request";
	case 0x22: return L"Unload Request";
	case 0x29: return L"Stop Motion Request";
	case 0x2A: return L"Travel Position Command";
	case 0x2B: return L"Hoist Position Command";
	case 0x2C: return L"Grip Position Command";
	case 0x2F: return L"Drive Ready Request";
	case 0xFD: return L"Reset Request";
	case 0xFE: return L"State Frame";
	case 0xFF: return L"ACK";
	case 0x81: return L"Done";
	case 0xF0: return L"Comm Open";
	default:   return L"Unknown";
	}
}

// ---- NEW: Append human readable log to Serial Monitor ----
static void AppendLog(const wchar_t* wmsg)
{
	size_t len = wcslen(wmsg);
	wchar_t* dup = (wchar_t*)malloc((len + 1) * sizeof(wchar_t));
	if (!dup) return;
	wcscpy_s(dup, len + 1, wmsg);
	PostMessage(g_hMainWnd ? g_hMainWnd : GetDesktopWindow(), WM_APP_TCP_LOG, 0, (LPARAM)dup);
}

static void LogFrameHuman(const unsigned char* f, int len, const wchar_t* prefix)
{
	if (!f || len <= 0) return;

	// Hex line
	wchar_t hexbuf[2048];
	int wi = swprintf_s(hexbuf, L"%s HEX (%d): ", prefix, len);
	for (int i = 0; i < len && wi < (int)_countof(hexbuf) - 4; ++i)
		wi += swprintf_s(hexbuf + wi, _countof(hexbuf) - wi, L"%02X ", (unsigned char)f[i]);
	AppendLog(hexbuf);

	// If basic frame 02 .... 03, decode op and length
	if (len >= 6 && f[0] == 0x02 && f[len - 1] == 0x03) {
		unsigned char op = f[3];
		unsigned char plen = f[4];
		wchar_t info[256];
		swprintf_s(info, L"%s Decoded: OP=0x%02X (%s), PayloadLen=%u", prefix, op, DecodeOpName(op), (unsigned)plen);
		AppendLog(info);
	}
}


static void DoOhtAction_EStopAll() {
	// 예시: 전체 급정지
	for (int a = 0; a < kNumAxes - 3; ++a) StopAxis(a);
}


// Periodic thread proc [NEW]
void PeriodicThreadProc()
{
	while (g_periodicRun.load()) {
		if (g_clientSock != INVALID_SOCKET) {
			SendPeriodicStateFrame(g_clientSock);
		}
		for (int i = 0; i < 10 && g_periodicRun.load(); ++i) Sleep(100); // 1초
	}
}

void PostTcpStateToMain(const wchar_t* msg)
{
	size_t len = wcslen(msg);
	wchar_t* dup = (wchar_t*)malloc((len + 1) * sizeof(wchar_t));
	if (!dup) return;
	wcscpy_s(dup, len + 1, msg);
	PostMessage(g_hMainWnd ? g_hMainWnd : GetDesktopWindow(), WM_APP_TCP_STATE, 0, (LPARAM)dup);
}

static void CloseClient()
{
	if (g_clientSock != INVALID_SOCKET) {
		closesocket(g_clientSock);
		g_clientSock = INVALID_SOCKET;
	}
}
static void CloseListen()
{
	if (g_listenSock != INVALID_SOCKET) {
		closesocket(g_listenSock);
		g_listenSock = INVALID_SOCKET;
	}
}

std::atomic<bool> g_motionBusy{ false };

// DONE(0x21, 0x22)에 대한 ACK 수신 여부
std::atomic<bool> g_ackLoadDone{ false };
std::atomic<bool> g_ackUnloadDone{ false };


void TcpServerThreadProc()
{
	WSADATA wsa{};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		AppendLog(L"[TCP] WSAStartup failed");
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}
	AppendLog(L"[TCP] WSAStartup OK");

	g_listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (g_listenSock == INVALID_SOCKET) {
		AppendLog(L"[TCP] socket() failed");
		WSACleanup();
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}

	int opt = 1;
	setsockopt(g_listenSock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons((u_short)g_tcpBindPort);

	char ipA[64];
	WideCharToMultiByte(CP_ACP, 0, g_tcpBindIp, -1, ipA, sizeof(ipA), 0, 0);

	if (inet_pton(AF_INET, ipA, &addr.sin_addr) != 1) {
		AppendLog(L"[TCP] inet_pton() failed. Using default 0.0.0.0");
		inet_pton(AF_INET, "0.0.0.0", &addr.sin_addr);
	}

	if (bind(g_listenSock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
		AppendLog(L"[TCP] bind() failed (port in use?)");
		CloseListen();
		WSACleanup();
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}

	if (listen(g_listenSock, 1) == SOCKET_ERROR) {
		AppendLog(L"[TCP] listen() failed");
		CloseListen();
		WSACleanup();
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}

	{
		wchar_t msg[128];
		swprintf_s(msg, L"TCP: LISTEN %s:%d", g_tcpBindIp, g_tcpBindPort);
		PostTcpStateToMain(msg);
		swprintf_s(msg, L"[TCP] Listening on %s:%d", g_tcpBindIp, g_tcpBindPort);
		AppendLog(msg);
	}

	while (g_tcpRunning.load()) {
		sockaddr_in cli{};
		int clen = sizeof(cli);
		AppendLog(L"[TCP] Waiting for client...");
		g_clientSock = accept(g_listenSock, (sockaddr*)&cli, &clen);
		if (g_clientSock == INVALID_SOCKET) {
			if (!g_tcpRunning.load()) break;
			AppendLog(L"[TCP] accept() failed");
			continue;
		}
		AppendLog(L"[TCP] Client connected");
		PostTcpStateToMain(L"TCP: CLIENT CONNECTED");

		// 연결되자마자 주기프레임 1번 즉시 전송
		g_heartbeat = 0;
		g_driveReady.store(false, std::memory_order_relaxed); // ★ 운전준비 플
		SendPeriodicStateFrame(g_clientSock);

		// Periodic thread start
		g_periodicRun = true;
		g_heartbeat = 0;
		std::thread th(PeriodicThreadProc);
		th.detach();

		char rbuf[512];
		std::string line;

		auto HandleCreateDevice = [&]() {
			AppendLog(L"[BIN] CreateDevice (legacy) ignored in new protocol");
			};

		// 이벤트 프레임 처리
		auto HandleEventFrame = [&](const unsigned char* f, int len) -> bool {
			if (len < 6) return false;
			if (f[0] != 0x02 || f[len - 1] != 0x03) return false;

			unsigned char op = f[3];
			unsigned char payLen = f[4];
			int expected = 5 + payLen + 1;
			if (expected != len) return false;

			// 수신 프레임 로그
			LogFrameHuman(f, len, L"[RX]");

			bool ok = false;

			switch (op) {
				// ---------------- LOAD ----------------
			case 0x21: // LOAD Request
			{
				AppendLog(L"[INFO] PLC -> PC : Load Request");

				// 이미 다른 모션/시퀀스 동작 중이면 거부
				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Load command ignored: motion already in progress");
					return false;
				}

				// ACK 먼저
				SendSimpleAck(g_clientSock, 0x21);
				AppendLog(L"[TX] Load Request ACK sent");

				SOCKET sockLoad = g_clientSock;
				std::thread([sockLoad]() {
					AppendLog(L"[ACT] Load Action Start");
					ToggleDO_HW(11, true, nullptr);
					StartDemoLoad(); // TaskId::DemoLoad 를 Running으로 세팅

					bool okLoad = WaitTaskFinished(TaskId::DemoLoad, 120000);
					if (!okLoad) {
						AppendLog(L"[WARN] Load sequence timeout or failed");
						ToggleDO_HW(11, false, nullptr);
						g_motionBusy.store(false);
						return;
						// 실패시 Done은 보내지 않음 (현재 정책)
					}
					else {
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[ACT] Load action complete (OK)");
						// ACK 플래그 초기화
						g_ackLoadDone.store(false, std::memory_order_relaxed);
						SendSimpleDone(sockLoad, 0x21);
						AppendLog(L"[TX] Load Done sent");

						DWORD lastSend = GetTickCount();

						while (g_tcpRunning.load()) {
							// PLC에서 ACK(0xFF, payload=0x21)를 받으면 g_ackLoadDone = true
							if (g_ackLoadDone.load(std::memory_order_relaxed)) {
								AppendLog(L"[INFO] Load Done ACK received from PLC");
								break;
							}

							DWORD now = GetTickCount();
							if (now - lastSend >= 2000) {
								// 2초 동안 ACK를 못 받으면 Done 다시 전송
								AppendLog(L"[WARN] Load Done ACK not received, resending...");
								SendSimpleDone(sockLoad, 0x21);
								AppendLog(L"[TX] Load Done re-sent");
								lastSend = now;
								// g_ackLoadDone 는 ACK 올 때만 true로 바뀜
							}

							::Sleep(50);
						}
					}

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- UNLOAD ----------------
			case 0x22: // UNLOAD Request
			{
				AppendLog(L"[INFO] PLC -> PC : Unload Request");

				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Unload command ignored: motion already in progress");
					return false;
				}

				SendSimpleAck(g_clientSock, 0x22);
				AppendLog(L"[TX] Unload Request ACK sent");

				SOCKET sockUnload = g_clientSock;
				std::thread([sockUnload]() {
					AppendLog(L"[ACT] Unload Action Start");
					ToggleDO_HW(11, true, nullptr);
					StartDemoUnload();

					bool okUnload = WaitTaskFinished(TaskId::DemoUnload, 60000);
					if (!okUnload) {
						AppendLog(L"[WARN] Unload sequence timeout or failed");
						ToggleDO_HW(11, false, nullptr);
						g_motionBusy.store(false);
						return;
					}
					else {
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[ACT] Unload action complete (OK)");
						g_ackUnloadDone.store(false, std::memory_order_relaxed);
						SendSimpleDone(sockUnload, 0x22);
						AppendLog(L"[TX] Unload Done sent");

						DWORD lastSend = GetTickCount();

						while (g_tcpRunning.load()) {
							if (g_ackUnloadDone.load(std::memory_order_relaxed)) {
								AppendLog(L"[INFO] Unload Done ACK received from PLC");
								break;
							}

							DWORD now = GetTickCount();
							if (now - lastSend >= 2000) {
								AppendLog(L"[WARN] Unload Done ACK not received, resending...");
								SendSimpleDone(sockUnload, 0x22);
								AppendLog(L"[TX] Unload Done re-sent");
								lastSend = now;
							}

							::Sleep(50);
						}
					}

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- STOP (즉시 처리) ----------------
			case 0x29: // 축정지
				AppendLog(L"[INFO] PLC -> PC : Stop Motion Request");
				SendSimpleAck(g_clientSock, 0x29);
				AppendLog(L"[TX] Stop Request ACK sent");

				DoStopAll(nullptr);
				ToggleDO_HW(11, false, nullptr);
				AppendLog(L"[ACT] All axes QuickStop executed");

				if (WaitAllAxesStopped(1.0, 10000)) {
					AppendLog(L"[INFO] All axes stopped (vel <= 1.0, Motioning OFF)");
					SendSimpleDone(g_clientSock, 0x29);
					AppendLog(L"[TX] Stop Done sent");
					return true;
				}
				else {
					AppendLog(L"[WARN] Stop timeout : some axes still moving or Motioning still ON");
					return false;
				}

				// ---------------- RESET ----------------
			case 0xFD: // 리셋
				AppendLog(L"[INFO] PLC -> PC : Reset Request");
				for (int a = 0; a < kNumAxes - 3; ++a) {
					g_cm.axisControl->ClearAmpAlarm(a);
				}
				SendSimpleAck(g_clientSock, 0xFD);
				AppendLog(L"[TX] Reset ACK sent");

				// 운전 준비 플래그도 리셋
				g_driveReady.store(false, std::memory_order_relaxed);

				return true;

				// ---------------- COMM OPEN ----------------
			case 0xF0: // Comm Open
				if (payLen >= 1 && f[5] == 0x01) {
					AppendLog(L"[INFO] PLC -> PC : Comm Open (통신 오픈)");
					PostTcpStateToMain(L"TCP: COMM OPEN");
				}
				else {
					AppendLog(L"[WARN] Comm Open frame payload unexpected");
				}
				return true;

				// ---------------- TRAVEL (Axis0) ----------------
			case 0x2A: // 주행 포지션 기동
			{
				// 인터락 체크
				if (!CheckInterlockBeforeAxisCommand(0)) {
					AppendLog(L"[INTERLOCK] Axis0 blocked: Axis2 limit must be ON in Auto");
					return false;
				}

				unsigned char posNo = (payLen >= 1) ? f[5] : 0;
				wchar_t info[128];
				swprintf_s(info, L"[INFO] PLC -> PC : Travel Position Command, Pos=%u", (unsigned)posNo);
				AppendLog(info);

				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Travel command ignored: motion already in progress");
					return false;
				}

				// ACK 전송
				SendSimpleAck(g_clientSock, 0x2A);
				AppendLog(L"[TX] Travel Position ACK sent");

				SOCKET sockTravel = g_clientSock;
				std::thread([sockTravel, posNo]() {
					bool okTravel = false;

					switch (posNo) {
					case 1:
						AppendLog(L"[ACT] Travel Pos1 -> Conveyor");
						ToggleDO_HW(11, true, nullptr);
						GoLeft();
						okTravel = WaitTaskFinished(TaskId::GoLeft, 30000);
						AppendLog(okTravel
							? L"[ACT] Travel Pos1 -> Conveyor DONE"
							: L"[ACT] Travel Pos1 -> Conveyor FAILED or TIMEOUT");
						if (okTravel)
							ToggleDO_HW(11, false, nullptr);
						break;
					case 2:
						AppendLog(L"[ACT] Travel Pos2 -> Workstation");
						ToggleDO_HW(11, true, nullptr);
						GoWorkstation();
						okTravel = WaitTaskFinished(TaskId::GoWorkstation, 30000);
						AppendLog(okTravel
							? L"[ACT] Travel Pos2 -> Workstation DONE"
							: L"[ACT] Travel Pos2 -> Workstation FAILED or TIMEOUT");
						if (okTravel)
							ToggleDO_HW(11, false, nullptr);
						break;
					case 3:
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[WARN] Travel Pos3 not implemented");
						okTravel = false;
						break;
					default:
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[WARN] Travel Position invalid PosNo");
						okTravel = false;
						break;
					}

					/*if (okTravel) {
						SendSimpleDone(sockTravel, 0x2A);
						AppendLog(L"[TX] Travel Pos Done sent");
					}*/

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- HOIST (Axis2) ----------------
			case 0x2B: // 상하 포지션 기동
			{
				if (!CheckInterlockBeforeAxisCommand(2)) {
					AppendLog(L"[INTERLOCK] Axis2 blocked: Travel must be at Load/Unload in Auto");
					return false;
				}

				unsigned char posNo = (payLen >= 1) ? f[5] : 0;
				wchar_t info[128];
				swprintf_s(info, L"[INFO] PLC -> PC : Hoist Position Command, Pos=%u", (unsigned)posNo);
				AppendLog(info);

				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Hoist command ignored: motion already in progress");
					return false;
				}

				SendSimpleAck(g_clientSock, 0x2B);
				AppendLog(L"[TX] Hoist Position ACK sent");

				SOCKET sockHoist = g_clientSock;
				std::thread([sockHoist, posNo]() {
					bool okHoist = false;

					switch (posNo) {
					case 1:
						AppendLog(L"[ACT] Hoist Pos1 -> Conveyor Down");
						ToggleDO_HW(11, true, nullptr);
						HoistDown();
						okHoist = WaitTaskFinished(TaskId::HoistDown, 20000);
						AppendLog(okHoist
							? L"[ACT] Hoist Pos1 -> ConveyorDown DONE"
							: L"[ACT] Hoist Pos1 -> ConveyorDown FAILED or TIMEOUT");
						if (okHoist)
							ToggleDO_HW(11, false, nullptr);
						break;

					case 2:
						AppendLog(L"[ACT] Hoist Pos2 -> Work Down");
						ToggleDO_HW(11, true, nullptr);
						HoistDown();
						okHoist = WaitTaskFinished(TaskId::HoistDown, 20000);
						AppendLog(okHoist
							? L"[ACT] Hoist Pos2 -> WorkDown DONE"
							: L"[ACT] Hoist Pos2 -> WorkDown FAILED or TIMEOUT");
						if (okHoist)
							ToggleDO_HW(11, false, nullptr);
						break;

					case 3:
						AppendLog(L"[ACT] Hoist Pos3 -> Up Position");
						ToggleDO_HW(11, true, nullptr);
						Up();
						okHoist = WaitTaskFinished(TaskId::Up, 20000);
						AppendLog(okHoist
							? L"[ACT] Hoist Pos3 -> Up DONE"
							: L"[ACT] Hoist Pos3 -> Up FAILED or TIMEOUT");
						if (okHoist)
							ToggleDO_HW(11, false, nullptr);
						break;

					default:
						AppendLog(L"[WARN] Hoist Position invalid PosNo");
						okHoist = false;
						break;
					}

					/*if (okHoist) {
						SendSimpleDone(sockHoist, 0x2B);
						AppendLog(L"[TX] Hoist Pos Done sent");
					}*/

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- GRIP ----------------
			case 0x2C: // 그립 포지션 기동
			{
				unsigned char posNo = (payLen >= 1) ? f[5] : 0;
				wchar_t info[128];
				swprintf_s(info, L"[INFO] PLC -> PC : Grip Position Command, Pos=%u", (unsigned)posNo);
				AppendLog(info);

				// 모션 Busy 체크 (그립도 모션으로 간주)
				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Grip command ignored: motion already in progress");
					return false;
				}

				// ACK 전송
				SendSimpleAck(g_clientSock, 0x2C);
				AppendLog(L"[TX] Grip Position ACK sent");

				SOCKET sockGrip = g_clientSock;
				std::thread([sockGrip, posNo]() {
					// 재진입 방지(그립 전용)
					if (g_gripBusy) {
						AppendLog(L"[BUSY] Grip command ignored: g_gripBusy == true");
						g_motionBusy.store(false);
						return;
					}

					bool okGrip = false;
					bool motioning = g_diStable[0]; // Motioning 상태 (필요시 수정)

					switch (posNo) {
					case 1: // Open
						if (HasBox()) {
							AppendLog(L"[INTERLOCK] Grip Open blocked: HasBox()==true");
							break;
						}
						if (IsGripperAlreadyOpen()) {
							AppendLog(L"[SKIP] Grip already OPEN. No action performed.");
							okGrip = true;
							break;
						}

						AppendLog(L"[ACT] Grip Pos1 -> GripOpen");
						g_gripBusy = true;


						ToggleDO_HW(11, motioning, nullptr);
						DoOpen_Compat(nullptr);
						okGrip = WaitUntil(IsGripperOpenAndIdle, 10000);
						ToggleDO_HW(11, motioning, nullptr);

						g_gripBusy = false;
						AppendLog(okGrip
							? L"[ACT] Grip Pos1 -> Open DONE"
							: L"[ACT] Grip Pos1 -> Open FAILED or TIMEOUT");
						break;

					case 2: // Close
						if (IsGripperAlreadyClosed()) {
							AppendLog(L"[SKIP] Grip already CLOSED. No action performed.");
							okGrip = false;
							break;
						}

						AppendLog(L"[ACT] Grip Pos2 -> GripClose");
						g_gripBusy = true;


						ToggleDO_HW(11, motioning, nullptr);
						DoClose_Compat(nullptr);
						okGrip = WaitUntil(IsGripperClosedAndIdle, 10000);
						ToggleDO_HW(11, motioning, nullptr);
						g_gripBusy = false;

						AppendLog(okGrip
							? L"[ACT] Grip Pos2 -> Close DONE"
							: L"[ACT] Grip Pos2 -> Close FAILED or TIMEOUT");
						break;

					case 3:
						AppendLog(L"[WARN] Grip Pos3 not implemented");
						okGrip = false;
						break;

					default:
						AppendLog(L"[WARN] Grip Position invalid PosNo");
						okGrip = false;
						break;
					}

					// 현재 프로토콜에서는 Grip에 대해 Done 프레임은 보내지 않고 ACK만 있는 상태 유지
					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- 운전 준비 요구 (Drive Ready) ----------------
			case 0x2F:
			{
				AppendLog(L"[INFO] PLC -> PC : Drive Ready Request (0x2F)");

				// 다른 모션 중이면 거부
				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Drive Ready command ignored: motion already in progress");
					return false;
				}

				// ACK 먼저 전송 (02 00 00 FF 01 2F 03)
				SendSimpleAck(g_clientSock, 0x2F);
				AppendLog(L"[TX] Drive Ready ACK sent");

				SOCKET sockReady = g_clientSock;
				std::thread([sockReady]() {
					// 2) 그리퍼 상태 확인
					unsigned char gcode = CalcPosGripCode(); // 0x00이면 중간(애매한) 상태라고 가정

					wchar_t info[128];
					swprintf_s(info, L"[ACT] DriveReady: Current GripCode=0x%02X", (unsigned)gcode);
					AppendLog(info);

					if (gcode == 0x00) {
						// 2-1) 먼저 Close 쪽으로 정리
						AppendLog(L"[ACT] DriveReady: Grip ambiguous -> Close then ServoOff");
						DoClose_Compat(g_hDemoWnd);
						(void)WaitUntil(IsGripperClosedAndIdle, 5000);

						// 2-2) Close 상태에서 Servo OFF
						//DoGripServoOff_Compat(g_hDemoWnd);

						// 2-3) 박스 보유 여부 체크
						if (HasBox()) {
							// 박스 들고 있으면 Close+ServoOff 상태 유지하고 종료
							AppendLog(L"[ACT] DriveReady: HasBox()==true -> keep Close+ServoOff");
						}
						else {
							// 박스가 없다면 Open 상태로 정리
							AppendLog(L"[ACT] DriveReady: HasBox()==false -> Open then ServoOff");
							DoOpen_Compat(g_hDemoWnd);
							(void)WaitUntil(IsGripperOpenAndIdle, 5000);
							//DoGripServoOff_Compat(g_hDemoWnd);
						}
					}
					else {
						// gcode != 0x00 이면 (이미 Open 또는 Close 쪽이라면) 추가 그리퍼 동작 없이 종료
						AppendLog(L"[ACT] DriveReady: Grip code already non-zero, no extra grip motion");
					}

					// 1) Axis2가 Limit(Up) 상태가 아니면 먼저 Up으로 정리
					unsigned char hcode = CalcPosHoistCode();
					swprintf_s(info, L"[ACT] DriveReady: Current HoistCode=0x%02X", (unsigned)hcode);
					AppendLog(info);

					if (hcode != 0x03) {
						AppendLog(L"[ACT] DriveReady: Hoist not UP(0x03) -> DoUp()");
						Up();
						(void)WaitUntil(IsAxis2Up, 20000);
					}
					else {
						AppendLog(L"[ACT] DriveReady: Hoist already UP(0x03)");
					}

					// 운전 준비 완료 플래그 ON
					g_driveReady.store(true, std::memory_order_relaxed);
					AppendLog(L"[INFO] DriveReady sequence complete -> driveReady = 1");

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- ACK ----------------
			case 0xFF: // ACK
				AppendLog(L"[INFO] PLC -> PC : ACK received");
				if (payLen >= 1) {
					unsigned char ackFor = f[5];
					wchar_t info[128];
					swprintf_s(info, L"[INFO] ACK for OP=0x%02X (%s)",
						ackFor, DecodeOpName(ackFor));
					AppendLog(info);
				}
				// ★ Load / Unload 두 쪽 다 깨워줌
				// (동시에 둘 다 돌지 않게 g_motionBusy로 막아놨으므로 안전)
				g_ackLoadDone.store(true, std::memory_order_relaxed);
				g_ackUnloadDone.store(true, std::memory_order_relaxed);
				return true;

				// ---------------- 기타 ----------------
			default:
			{
				wchar_t info[128];
				swprintf_s(info, L"[WARN] Unsupported OP=0x%02X (%s)", op, DecodeOpName(op));
				AppendLog(info);
			}
			return false;
			} // switch
			}; // HandleEventFrame

		// === RECV LOOP ===
		while (g_tcpRunning.load()) {
			int r = recv(g_clientSock, rbuf, sizeof(rbuf), 0);
			if (r <= 0) {
				AppendLog(L"[TCP] Client disconnected");
				PostTcpStateToMain(L"TCP: LISTENING");
				break;
			}

			// RAW 로그
			{
				unsigned char* p = (unsigned char*)rbuf;
				LogFrameHuman(p, r, L"[RAW]");
			}

			// 7바이트 구버전
			if (r == 7 && (unsigned char)rbuf[0] == 0x02 && (unsigned char)rbuf[6] == 0x03) {
				HandleEventFrame((unsigned char*)rbuf, r);
				continue;
			}

			// 새 이벤트 프레임
			if (r >= 6 && (unsigned char)rbuf[0] == 0x02 && (unsigned char)rbuf[r - 1] == 0x03) {
				HandleEventFrame((unsigned char*)rbuf, r);
				continue;
			}

			// 텍스트 라인 파싱
			line.append(rbuf, r);
			size_t pos;
			while ((pos = line.find_first_of("\r\n")) != std::string::npos) {
				std::string one = line.substr(0, pos);
				size_t next = pos + 1;
				while (next < line.size() && (line[next] == '\r' || line[next] == '\n')) ++next;
				line.erase(0, next);

				auto trimA = [](std::string s) {
					size_t i = 0, j = s.size();
					while (i < j && (s[i] == ' ' || s[i] == '\t')) ++i;
					while (j > i && (s[j - 1] == ' ' || s[j - 1] == '\t')) --j;
					return s.substr(i, j - i);
					};
				one = trimA(one);
				if (one.empty()) continue;

				wchar_t wline[256];
				MultiByteToWideChar(CP_ACP, 0, one.c_str(), -1, wline, 256);
				wchar_t wmsg[300];
				swprintf_s(wmsg, L"[RX ASCII] %s", wline);
				AppendLog(wmsg);

				if (one == "0") {
					HandleCreateDevice();
				}
				else if (one == "1") {
					if (!g_deviceOpened) AppendLog(L"[RX] : Device not created");
					else if (g_commStarted) AppendLog(L"[RX] : Communication already started");
					else {
						bool okComm = StartComm();
						AppendLog(okComm ? L"[RX] : Communication Success" : L"[RX] : Communication Failed");
					}
				}
			}
		}

		// 클라이언트 종료 처리
		g_periodicRun = false;
		CloseClient();
	}

	CloseListen();
	WSACleanup();
	AppendLog(L"[TCP] Server thread exit");
	PostTcpStateToMain(L"TCP: STOPPED");
	g_tcpRunning = false;
}


static void StartTcpServer()
{
	if (g_tcpRunning.load()) return;
	g_tcpRunning = true;
	g_tcpThread = std::thread(TcpServerThreadProc);
	g_tcpThread.detach();
}
static void StopTcpServer()
{
	if (!g_tcpRunning.load()) return;
	g_tcpRunning = false;
	g_periodicRun = false;
	shutdown(g_listenSock, SD_BOTH);
	shutdown(g_clientSock, SD_BOTH);
	CloseClient();
	CloseListen();
}

// ------------------ 수동 이동/체크박스 등 ------------------
static bool IsAxisChecked(HWND hWnd, int axis) {
	int id = ID_CHECK_AXIS(axis);
	HWND hb = GetDlgItem(hWnd, id);
	if (!hb) return false;
	return SendMessage(hb, BM_GETCHECK, 0, 0) == BST_CHECKED;
}
static void SetAxisChecked(HWND hWnd, int axis, bool checked) {
	int id = ID_CHECK_AXIS(axis);
	HWND hb = GetDlgItem(hWnd, id);
	if (hb) SendMessage(hb, BM_SETCHECK, checked ? BST_CHECKED : BST_UNCHECKED, 0);
}
static void UpdateSelectedAxesTextOnDemand(HWND hWnd) {
	bool sel[kNumAxes - 3] = {};
	for (int a = 0; a < kNumAxes - 3; ++a) sel[a] = IsAxisChecked(hWnd, a);
	TCHAR text[256] = TEXT("Selected: "); bool any = false;
	for (int a = 0; a < kNumAxes - 3; ++a) if (sel[a]) { TCHAR t[16]; _stprintf_s(t, TEXT("%s%d"), any ? TEXT(", ") : TEXT(""), a); _tcscat_s(text, t); any = true; }
	if (!any) _tcscat_s(text, TEXT("(none)"));
	SetWindowText(GetDlgItem(hWnd, ID_TXT_SELECTED_AXES), text);
}

static void Multi_SetAllAxisChecked(HWND hWnd, bool checked) {
	for (int a = 0; a < kNumAxes - 3; ++a) SetAxisChecked(hWnd, a, checked);
	UpdateSelectedAxesTextOnDemand(hWnd);
}

// Sync Group이 활성화된 상태에서 Slave 축은 개별 Position 명령이 무시/제한될 수 있음.
// (특히 Gantry/Sync 구성에서는 Slave는 Master만 따라가므로, Slave에 StartPos를 걸어도 "안 움직이는 것"처럼 보일 수 있음)
static bool IsAxisSlaveInEnabledSyncGroup(int axis, int* outGroupId = nullptr, int* outMasterAxis = nullptr) {
	if (!g_commStarted) return false;
	if (!g_cm.sync) return false;

	for (int gid = 0; gid < kNumAxes - 3; ++gid) {
		Sync::SyncGroupStatus st{};
		long se = g_cm.sync->GetSyncGroupStatus(gid, &st);
		if (se != ErrorCode::None || !st.enabled) continue;

		Sync::SyncGroup grp{};
		long ge = g_cm.sync->GetSyncGroup(gid, &grp);
		if (ge != ErrorCode::None) continue;

		for (int i = 0; i < (int)grp.slaveAxisCount; ++i) {
			if ((int)grp.slaveAxis[i] == axis) {
				if (outGroupId) *outGroupId = gid;
				if (outMasterAxis) *outMasterAxis = (int)grp.masterAxis;
				return true;
			}
		}
	}
	return false;
}
static void DoAbsMoveAxis(HWND hWnd, int axis) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }

	/*if (axis == 2) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[2].actualPos;
		double tgtD = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0);
		long long tgt = (long long)std::llround(tgtD);
		if (Axis2IsMinusCommandBlocked(cur, tgt, 0)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return;
		}
	}

	if (axis == 0) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[0].actualPos;
		double tgtD = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0);
		long long tgt = (long long)std::llround(tgtD);
		long long delta = tgt - cur;
		long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
		if (sign != 0 && Axis0IsCommandBlocked(cur, tgt, sign)) {
			Axis0ShowBlockedWarning(hWnd, (int)sign);
			return;
		}
	}*/

	//DisableAllEnabledSyncGroups();
	if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) return;
	double tgt = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0);
	double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 10000.0);
	double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
	double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);
	StartAbsMoveWithProfile(axis, (long long)std::llround(tgt), v, ta, td);
}
static void DoRelMoveAxis(HWND hWnd, int axis, int dir) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }

	//// Axis2 보호: 상대 이동 전 차단 검사
	//if (axis == 2) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[2].actualPos;
	//	double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0) * dir;
	//	long long tgt = cur + (long long)std::llround(step);
	//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)dir)) {
	//		Axis2ShowMinusBlockedWarning(hWnd);
	//		return;
	//	}
	//}

	//// Axis0 보호: 상대 이동 전 L/R 리밋 방향 차단
	//if (axis == 0) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[0].actualPos;
	//	double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0) * dir;
	//	long long tgt = cur + (long long)std::llround(step);
	//	if (Axis0IsCommandBlocked(cur, tgt, (long long)dir)) {
	//		Axis0ShowBlockedWarning(hWnd, dir);
	//		return;
	//	}
	//}

	//DisableAllEnabledSyncGroups();
	if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) return;
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;
	double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0) * dir;
	double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 10000.0);
	double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
	double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);
	StartAbsMoveWithProfile(axis, cur + (long long)std::llround(step), v, ta, td);
}
static void DoMultiAbs(HWND hWnd) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }
	//DisableAllEnabledSyncGroups();
	for (int a = 0; a < kNumAxes - 3; ++a) if (IsAxisChecked(hWnd, a) && EnsureServoOn(a) && EnsurePosModeNoStop(a)) {

		//// Axis2 보호
		//if (a == 2) {
		//	g_cm.GetStatus(&g_status);
		//	long long cur = (long long)g_status.axesStatus[2].actualPos;
		//	long long tgt = (long long)std::llround(GetDlgDouble(hWnd, ID_EDIT_POS_A(2), 0.0));
		//	if (Axis2IsMinusCommandBlocked(cur, tgt, 0)) {
		//		Axis2ShowMinusBlockedWarning(hWnd);
		//		continue;
		//	}
		//}

		//// Axis0 보호: L/R 리밋에 따른 방향 금지
		//if (a == 0) {
		//	g_cm.GetStatus(&g_status);
		//	long long cur = (long long)g_status.axesStatus[0].actualPos;
		//	long long tgt = (long long)std::llround(GetDlgDouble(hWnd, ID_EDIT_POS_A(0), 0.0));
		//	long long delta = tgt - cur;
		//	long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
		//	if (sign != 0 && Axis0IsCommandBlocked(cur, tgt, sign)) {
		//		Axis0ShowBlockedWarning(hWnd, (int)sign);
		//		continue;
		//	}
		//}

		double tgt = GetDlgDouble(hWnd, ID_EDIT_POS_A(a), 0.0);
		double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(a), 10000.0);
		double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(a), 100.0);
		double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(a), 100.0);
		StartAbsMoveWithProfile(a, (long long)std::llround(tgt), v, ta, td);
	}
}
static void DoMultiRel(HWND hWnd) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }
	//DisableAllEnabledSyncGroups();
	for (int a = 0; a < kNumAxes - 3; ++a) if (IsAxisChecked(hWnd, a) && EnsureServoOn(a) && EnsurePosModeNoStop(a)) {

		//// Axis2 보호
		//if (a == 2) {
		//	g_cm.GetStatus(&g_status);
		//	long long cur = (long long)g_status.axesStatus[2].actualPos;
		//	double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(2), 0.0);
		//	long long tgt = cur + (long long)std::llround(step);
		//	int dir = (step >= 0) ? +1 : -1;
		//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)dir)) {
		//		Axis2ShowMinusBlockedWarning(hWnd);
		//		continue;
		//	}
		//}

		//// Axis0 보호: 상대 이동 방향 차단
		//if (a == 0) {
		//	g_cm.GetStatus(&g_status);
		//	long long cur = (long long)g_status.axesStatus[0].actualPos;
		//	double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(0), 0.0);
		//	long long tgt = cur + (long long)std::llround(step);
		//	int dir = (step >= 0) ? +1 : -1;
		//	if (Axis0IsCommandBlocked(cur, tgt, (long long)dir)) {
		//		Axis0ShowBlockedWarning(hWnd, dir);
		//		continue;
		//	}
		//}

		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[a].actualPos;
		double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(a), 0.0);
		double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(a), 10000.0);
		double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(a), 100.0);
		double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(a), 100.0);
		StartAbsMoveWithProfile(a, cur + (long long)std::llround(step), v, ta, td);
	}
}
static void DoMultiAlarmReset(HWND hWnd) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Alarm Reset(Selected)"), MB_ICONWARNING); return; }
	for (int a = 0; a < kNumAxes - 3; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;
		g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a); Sleep(10);
		g_cm.GetStatus(&g_status);
		bool wasOn = g_status.axesStatus[a].servoOn ? true : false;
		if (wasOn) { g_cm.axisControl->SetServoOn(a, 0); Sleep(20); }
		long e = g_cm.axisControl->ClearAmpAlarm(a);
		if (e != ErrorCode::None) { Sleep(30); e = g_cm.axisControl->ClearAmpAlarm(a); }
		if (wasOn) { Sleep(20); g_cm.axisControl->SetServoOn(a, 1); }
		if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Alarm Reset 실패"), e, g_wmx);
	}
}

void UpdateTcpUiState(HWND hWnd)
{
	if (!hWnd) return;
	if (HWND h = GetDlgItem(hWnd, ID_TXT_TCP_STATE)) {
		if (g_tcpRunning.load()) {
			wchar_t msg[128];
			swprintf_s(msg, L"TCP: LISTEN %s:%d", g_tcpBindIp, g_tcpBindPort);
			SetWindowTextW(h, msg);
		}
		else {
			SetWindowTextW(h, L"TCP: STOPPED");
		}
	}
}

static bool WaitAxesIdle(const std::vector<int>& axes, DWORD timeout_ms) {
	DWORD t0 = GetTickCount();
	while (g_demoRunning.load()) {
		g_cm.GetStatus(&g_status);
		bool allIdle = true;
		for (int a : axes) {
			int actVel = (int)std::lround(g_status.axesStatus[a].actualVelocity);
			if (std::abs(actVel) > vel_idle_threshold) { allIdle = false; break; }
		}
		if (allIdle) return true;
		if (GetTickCount() - t0 > timeout_ms) return false;
		Sleep(idle_wait_poll_ms);
	}
	g_cm.GetStatus(&g_status);
	bool allIdle = true;
	for (int a : axes) {
		int actVel = (int)std::lround(g_status.axesStatus[a].actualVelocity);
		if (std::abs(actVel) > vel_idle_threshold) { allIdle = false; break; }
	}
	return allIdle;
}

// ======================================================
// =================== Sync Group Window =================
// ======================================================
struct SyncUiState {
	int groupId = 0;
	int masterAxis = -1;
	std::vector<int> slaveAxes;
	bool controlMaster = true;
	int controlAxis = 0;
	std::vector<int> detectedAxes;
};
static SyncUiState g_syncUi;

static void Sync_DetectAxes() {
	g_syncUi.detectedAxes.clear();
	if (!g_commStarted) return;
	g_cm.GetStatus(&g_status);
	for (int ax = 0; ax < kNumAxes - 3; ++ax) {
		if (ax < kNumAxes - 3) g_syncUi.detectedAxes.push_back(ax);
	}
}

static void Sync_ClearDynamicAxisWidgets(HWND h) {
	for (int ax = 0; ax < 64; ++ax) {
		if (HWND w = GetDlgItem(h, ID_SYNC_MASTER_AXIS_BTN_BASE + ax)) DestroyWindow(w);
	}
	for (int ax = 0; ax < 64; ++ax) {
		if (HWND w = GetDlgItem(h, ID_SYNC_SLAVE_AXIS_CHK_BASE + ax)) DestroyWindow(w);
	}
}

static void Sync_RebuildAxisPickers(HWND h) {
	Sync_ClearDynamicAxisWidgets(h);

	int xMaster = 20, yMaster = 100;
	CreateWindow(TEXT("STATIC"), TEXT("마스터 선택"), WS_CHILD | WS_VISIBLE, xMaster, yMaster - 24, 100, 20, h, 0, 0, 0);

	int btnW = 48, btnH = 26, gap = 6;
	int col = 0, row = 0, maxCols = 8;
	bool first = true;
	for (int a : g_syncUi.detectedAxes) {
		int bx = xMaster + (btnW + gap) * col;
		int by = yMaster + (btnH + gap) * row;
		TCHAR cap[16]; _stprintf_s(cap, TEXT("%d"), a);
		DWORD style = WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | (first ? (WS_GROUP | WS_TABSTOP) : 0);
		HWND b = CreateWindow(TEXT("BUTTON"), cap, style, bx, by, btnW, btnH, h, (HMENU)(INT_PTR)(ID_SYNC_MASTER_AXIS_BTN_BASE + a), 0, 0);
		first = false;
		if (g_syncUi.masterAxis == a) SendMessage(b, BM_SETCHECK, BST_CHECKED, 0);
		++col; if (col >= maxCols) { col = 0; ++row; }
	}

	int xSlave = 20, ySlave = yMaster + 10 + ((row + (col ? 1 : 0)) * (btnH + gap)) + 20;
	CreateWindow(TEXT("STATIC"), TEXT("슬레이브 선택"), WS_CHILD | WS_VISIBLE, xSlave, ySlave - 24, 100, 20, h, 0, 0, 0);

	col = 0; row = 0;
	for (int a : g_syncUi.detectedAxes) {
		int bx = xSlave + (btnW + gap) * col;
		int by = ySlave + (btnH + gap) * row;
		TCHAR cap[16]; _stprintf_s(cap, TEXT("%d"), a);
		HWND b = CreateWindow(TEXT("BUTTON"), cap, WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
			bx, by, btnW, btnH, h, (HMENU)(INT_PTR)(ID_SYNC_SLAVE_AXIS_CHK_BASE + a), 0, 0);
		bool sel = std::find(g_syncUi.slaveAxes.begin(), g_syncUi.slaveAxes.end(), a) != g_syncUi.slaveAxes.end();
		SendMessage(b, BM_SETCHECK, sel ? BST_CHECKED : BST_UNCHECKED, 0);
		++col; if (col >= maxCols) { col = 0; ++row; }
	}
}

static void Sync_CreateAxisListColumns(HWND hList) {
	LVCOLUMN col{};
	col.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_SUBITEM;

	col.pszText = const_cast<LPTSTR>(TEXT("Axis")); col.cx = 60; col.iSubItem = 0; ListView_InsertColumn(hList, 0, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("Role")); col.cx = 90; col.iSubItem = 1; ListView_InsertColumn(hList, 1, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("OpState")); col.cx = 80; col.iSubItem = 2; ListView_InsertColumn(hList, 2, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("PosCmd")); col.cx = 100; col.iSubItem = 3; ListView_InsertColumn(hList, 3, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("ActualPos")); col.cx = 100; col.iSubItem = 4; ListView_InsertColumn(hList, 4, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("CmdVel")); col.cx = 80; col.iSubItem = 5; ListView_InsertColumn(hList, 5, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("ActVel")); col.cx = 80; col.iSubItem = 6; ListView_InsertColumn(hList, 6, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("ActVel(m/s)")); col.cx = 90; col.iSubItem = 7; ListView_InsertColumn(hList, 7, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("Err(603F)")); col.cx = 100; col.iSubItem = 8; ListView_InsertColumn(hList, 8, &col);
}

static int StartupEnumToComboIndex(Sync::SyncGroupStartupType::T t) {
	switch (t) {
	case Sync::SyncGroupStartupType::Normal: return 0;
	case Sync::SyncGroupStartupType::CatchUp: return 1;
	default: return 0;
	}
}
static Sync::SyncGroupStartupType::T ComboIndexToStartupEnum(int idx) {
	return (idx == 1) ? Sync::SyncGroupStartupType::CatchUp : Sync::SyncGroupStartupType::Normal;
}

// ---- Helper: Read/Write Config::SyncParam master/slave desync dec ----
static bool ReadSyncParam(Config::SyncParam& sp) {
	if (!g_commStarted || g_syncUi.masterAxis < 0) return false;
	int axis = g_syncUi.masterAxis;
	long e = g_cm.config->GetSyncParam(axis, &sp);
	return (e == ErrorCode::None);
}

static bool WriteSyncParam(const Config::SyncParam& sp) {
	if (!g_commStarted || g_syncUi.masterAxis < 0) return false;
	int axis = g_syncUi.masterAxis;
	long e = g_cm.config->SetSyncParam(axis, (Config::SyncParam*)&sp, nullptr);
	return (e == ErrorCode::None);
}

static void Sync_LoadGroupParamsToUI(HWND hWnd) {
	Sync_DetectAxes();

	Sync::SyncGroup grp{};
	long e = g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp);

	if (e != ErrorCode::None) {
		g_syncUi.masterAxis = (g_syncUi.detectedAxes.empty() ? -1 : g_syncUi.detectedAxes.front());
		g_syncUi.slaveAxes.clear();

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_SERVO_ONOFF), CB_SETCURSEL, 1, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_STARTUP), CB_SETCURSEL, 0, 0);

		SetDlgInt(hWnd, ID_SYNC_PARAM_CYCLE_RATIO, 1);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_TOLERANCE, 1000.0);

		SetDlgDouble(hWnd, ID_SYNC_PARAM_MAX_CATCH_UP, 0.0);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_VEL, 0.0);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_ACC, 0.0);

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_USE_MASTER_FB), CB_SETCURSEL, 0, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_AMP_ERR_SVON), CB_SETCURSEL, 0, 0);
	}
	else {
		g_syncUi.masterAxis = (int)grp.masterAxis;

		g_syncUi.slaveAxes.clear();
		for (int k = 0; k < (int)grp.slaveAxisCount; ++k)
			g_syncUi.slaveAxes.push_back((int)grp.slaveAxis[k]);

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_SERVO_ONOFF), CB_SETCURSEL, grp.servoOnOffSynchronization ? 1 : 0, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_STARTUP), CB_SETCURSEL, StartupEnumToComboIndex(grp.startupType), 0);

		SetDlgInt(hWnd, ID_SYNC_PARAM_CYCLE_RATIO, (int)grp.gantryLoopCycleRatio);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_MAX_CATCH_UP, grp.maxCatchUpDistance);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_VEL, grp.catchUpVelocity);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_ACC, grp.catchUpAcc);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_TOLERANCE, grp.syncErrorTolerance);

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_USE_MASTER_FB), CB_SETCURSEL, grp.useMasterFeedback ? 1 : 0, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_AMP_ERR_SVON), CB_SETCURSEL, 0, 0);
	}

	// Read Config::SyncParam master/slave Desync Dec into UI
	Config::SyncParam sp{};
	if (ReadSyncParam(sp)) {
		SetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, sp.masterDesyncDec);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, sp.slaveDesyncDec);
	}
	else {
		SetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, 10000.0);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, 10000.0);
	}

	Sync_RebuildAxisPickers(hWnd);
}

static bool Sync_ApplyGroupParamsFromUI(HWND hWnd) {
	int selMaster = -1;
	for (int a : g_syncUi.detectedAxes) {
		if (SendMessage(GetDlgItem(hWnd, ID_SYNC_MASTER_AXIS_BTN_BASE + a), BM_GETCHECK, 0, 0) == BST_CHECKED) {
			selMaster = a; break;
		}
	}
	if (selMaster < 0) {
		MessageBox(hWnd, TEXT("마스터 축을 선택하세요."), TEXT("Sync Group"), MB_ICONWARNING);
		return false;
	}

	std::vector<int> slaves;
	for (int a : g_syncUi.detectedAxes) {
		if (SendMessage(GetDlgItem(hWnd, ID_SYNC_SLAVE_AXIS_CHK_BASE + a), BM_GETCHECK, 0, 0) == BST_CHECKED) {
			if (a != selMaster) slaves.push_back(a);
		}
	}

	Sync::SyncGroup grp{};
	grp.masterAxis = (unsigned char)selMaster;

	int maxSlaves = (int)(sizeof(grp.slaveAxis) / sizeof(grp.slaveAxis[0]));
	if ((int)slaves.size() > maxSlaves) {
		MessageBox(hWnd, TEXT("슬레이브 축 개수가 최대치를 초과했습니다."), TEXT("Sync Group"), MB_ICONWARNING);
		return false;
	}
	grp.slaveAxisCount = (unsigned char)slaves.size();
	for (int i = 0; i < (int)grp.slaveAxisCount; ++i)
		grp.slaveAxis[i] = (unsigned char)slaves[i];

	grp.servoOnOffSynchronization = (unsigned char)(SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_SERVO_ONOFF), CB_GETCURSEL, 0, 0) == 1 ? 1 : 0);
	{
		int idx = (int)SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_STARTUP), CB_GETCURSEL, 0, 0);
		grp.startupType = ComboIndexToStartupEnum(idx);
	}
	grp.gantryLoopCycleRatio = (unsigned int)GetDlgInt(hWnd, ID_SYNC_PARAM_CYCLE_RATIO, 1);
	grp.maxCatchUpDistance = GetDlgDouble(hWnd, ID_SYNC_PARAM_MAX_CATCH_UP, 0.0);
	grp.catchUpVelocity = GetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_VEL, 0.0);
	grp.catchUpAcc = GetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_ACC, 0.0);
	grp.syncErrorTolerance = GetDlgDouble(hWnd, ID_SYNC_PARAM_TOLERANCE, 1000.0);
	grp.useMasterFeedback = (unsigned char)(SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_USE_MASTER_FB), CB_GETCURSEL, 0, 0) == 1 ? 1 : 0);

	auto isDetected = [&](int a) { return std::find(g_syncUi.detectedAxes.begin(), g_syncUi.detectedAxes.end(), a) != g_syncUi.detectedAxes.end(); };
	if (!isDetected(selMaster)) { MessageBox(hWnd, TEXT("선택된 마스터 축이 인식되지 않았습니다."), TEXT("Sync Group"), MB_ICONWARNING); return false; }
	for (int s : slaves) {
		if (!isDetected(s)) { MessageBox(hWnd, TEXT("선택된 슬레이브 축 중 인식되지 않은 축이 있습니다."), TEXT("Sync Group"), MB_ICONWARNING); return false; }
		if (s == selMaster) { MessageBox(hWnd, TEXT("마스터 축은 슬레이브로 중복 지정할 수 없습니다."), TEXT("Sync Group"), MB_ICONWARNING); return false; }
	}

	grp.gantryLoopCycleRatio = std::max(1u, grp.gantryLoopCycleRatio);
	if (grp.syncErrorTolerance <= 0) grp.syncErrorTolerance = 1000.0;

	if (grp.startupType == Sync::SyncGroupStartupType::CatchUp) {
		if (grp.catchUpVelocity <= 0) grp.catchUpVelocity = 50000.0;
		if (grp.catchUpAcc <= 0) grp.catchUpAcc = 100000.0;
		if (grp.maxCatchUpDistance <= 0) grp.maxCatchUpDistance = 10000.0;
	}
	else {
		if (grp.catchUpVelocity < 0) grp.catchUpVelocity = 0.0;
		if (grp.catchUpAcc < 0) grp.catchUpAcc = 0.0;
		if (grp.maxCatchUpDistance < 0) grp.maxCatchUpDistance = 0.0;
	}

	// If enabled, disable first
	Sync::SyncGroupStatus gst{};
	if (g_cm.sync->GetSyncGroupStatus(g_syncUi.groupId, &gst) == ErrorCode::None && gst.enabled) {
		g_cm.sync->EnableSyncGroup(g_syncUi.groupId, 0);
		Sleep(5);
	}

	long se = g_cm.sync->SetSyncGroup(g_syncUi.groupId, grp);
	if (se != ErrorCode::None) { ShowErrMsgBox(TEXT("SetSyncGroup 실패"), se, g_wmx); return false; }

	// Apply Config::SyncParam desync dec from UI
	Config::SyncParam sp{};
	if (ReadSyncParam(sp)) {
		sp.masterDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, 10000.0);
		sp.slaveDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, 10000.0);
		if (!WriteSyncParam(sp)) {
			MessageBox(hWnd, TEXT("Config::SyncParam 적용 실패(master/slave Desync Dec)"), TEXT("Sync Group"), MB_ICONWARNING);
		}
	}

	g_syncUi.masterAxis = selMaster;
	g_syncUi.slaveAxes = slaves;

	return true;
}

static void Sync_EnableGroup(HWND hWnd, bool en) {
	long e = g_cm.sync->EnableSyncGroup(g_syncUi.groupId, en ? 1 : 0);
	if (e != ErrorCode::None) ShowErrMsgBox(en ? TEXT("EnableSyncGroup 실패") : TEXT("DisableSyncGroup 실패"), e, g_wmx);

	// Ensure Config::SyncParam desync dec stays in sync with UI even on Enable button
	if (en) {
		Config::SyncParam sp{};
		if (ReadSyncParam(sp)) {
			sp.masterDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, sp.masterDesyncDec);
			sp.slaveDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, sp.slaveDesyncDec);
			WriteSyncParam(sp);
		}
	}
}

static void Sync_ClearGroupError() {
	long e = g_cm.sync->ClearSyncGroupError(g_syncUi.groupId);
	if (e != ErrorCode::None) ShowErrMsgBox(TEXT("ClearSyncGroupError 실패"), e, g_wmx);
}

static void Sync_AllServoOnOff(bool on) {
	Sync::SyncGroup grp{}; if (g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp) != ErrorCode::None) return;
	auto turn = [&](int ax) {
		if (on) { EnsureServoOn(ax); EnsurePosModeNoStop(ax); }
		else g_cm.axisControl->SetServoOn(ax, 0);
		};
	turn(grp.masterAxis);
	for (int i = 0; i < (int)grp.slaveAxisCount; ++i) turn(grp.slaveAxis[i]);
}

static int Sync_GetControlAxis(HWND hWnd) {
	bool master = (SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_MASTER), BM_GETCHECK, 0, 0) == BST_CHECKED);
	if (master) return g_syncUi.masterAxis >= 0 ? g_syncUi.masterAxis : 0;
	return GetDlgInt(hWnd, ID_SYNC_AXIS_SELECT, 0);
}

static void Sync_Control_Jog(HWND hWnd, int sign) {
	if (!IsManualAllowed(hWnd)) return;
	int ax = Sync_GetControlAxis(hWnd);

	//// Axis2 보호: 리밋 ON시 -방향 조그 차단
	//if (ax == 2) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[2].actualPos;
	//	long long tgt = cur + (long long)sign * 1000000000LL;
	//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)sign)) {
	//		Axis2ShowMinusBlockedWarning(hWnd);
	//		return;
	//	}
	//}

	//// Axis0 보호: L/R 리밋에 따른 방향 조그 차단
	//if (ax == 0) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[0].actualPos;
	//	long long tgt = cur + (long long)sign * 1000000000LL;
	//	if (Axis0IsCommandBlocked(cur, tgt, (long long)sign)) {
	//		Axis0ShowBlockedWarning(hWnd, sign);
	//		return;
	//	}
	//}

	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;
	double vpps = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
	double tAcc = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
	double tDec = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[ax].actualPos;

	Motion::PosCommand pc{};
	pc.axis = ax;
	pc.target = cur + (long long)sign * 1000000000LL;
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);
	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Sync Jog 실패"), e, g_wmx);
}
static void Sync_Control_Stop(HWND, int ax) {
	g_cm.motion->Stop(ax); g_cm.velocity->Stop(ax); if (g_cm.torque) g_cm.torque->StopTrq(ax);
}
static void Sync_Control_Abs(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;
	int ax = Sync_GetControlAxis(hWnd);
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;
	double tgt = GetDlgDouble(hWnd, ID_SYNC_ABS_POS, 0.0);
	double v = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
	double ta = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
	double td = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);

	//// Axis2 보호: 절대 이동 차단 체크
	//if (ax == 2) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[2].actualPos;
	//	long long tgtLL = (long long)std::llround(tgt);
	//	if (Axis2IsMinusCommandBlocked(cur, tgtLL, 0)) {
	//		Axis2ShowMinusBlockedWarning(hWnd);
	//		return;
	//	}
	//}

	//// Axis0 보호: 절대 이동 방향 차단 체크
	//if (ax == 0) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[0].actualPos;
	//	long long tgtLL = (long long)std::llround(tgt);
	//	long long delta = tgtLL - cur;
	//	long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
	//	if (sign != 0 && Axis0IsCommandBlocked(cur, tgtLL, sign)) {
	//		Axis0ShowBlockedWarning(hWnd, (int)sign);
	//		return;
	//	}
	//}

	StartAbsMoveWithProfile(ax, (long long)std::llround(tgt), v, ta, td);
}
static void Sync_Control_Rel(HWND hWnd, int sign) {
	if (!IsManualAllowed(hWnd)) return;
	int ax = Sync_GetControlAxis(hWnd);
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;
	double step = GetDlgDouble(hWnd, ID_SYNC_REL_STEP, 0.0) * sign;
	double v = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
	double ta = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
	double td = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[ax].actualPos;
	long long tgt = cur + (long long)std::llround(step);

	//// Axis2 보호: 상대 이동 차단 체크
	//if (ax == 2) {
	//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)sign)) {
	//		Axis2ShowMinusBlockedWarning(hWnd);
	//		return;
	//	}
	//}

	//// Axis0 보호: 상대 이동 방향 차단 체크
	//if (ax == 0) {
	//	if (Axis0IsCommandBlocked(cur, tgt, (long long)sign)) {
	//		Axis0ShowBlockedWarning(hWnd, sign);
	//		return;
	//	}
	//}

	StartAbsMoveWithProfile(ax, tgt, v, ta, td);
}

static void Sync_UpdateMonitor(HWND hWnd) {
	if (!g_commStarted) return;

	Sync::SyncGroupStatus st{};
	long e = g_cm.sync->GetSyncGroupStatus(g_syncUi.groupId, &st);
	if (e == ErrorCode::None) {
		SetWindowText(GetDlgItem(hWnd, ID_SYNC_STATE_ENABLED), st.enabled ? (LPTSTR)TEXT("Enabled") : (LPTSTR)TEXT("Disabled"));
		SetWindowText(GetDlgItem(hWnd, ID_SYNC_STATE_HOMEDONE), st.homeDone ? (LPTSTR)TEXT("Home Done") : (LPTSTR)TEXT("Home Not Done"));
	}

	HWND hList = GetDlgItem(hWnd, ID_SYNC_AXIS_LIST);
	if (!hList) return;

	ListView_DeleteAllItems(hList);

	Sync::SyncGroup grp{}; if (g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp) != ErrorCode::None) return;

	std::vector<std::pair<int, const TCHAR*>> axes;
	axes.push_back({ (int)grp.masterAxis, TEXT("Master") });
	for (int i = 0; i < (int)grp.slaveAxisCount; ++i) axes.push_back({ (int)grp.slaveAxis[i], TEXT("Slave") });

	g_cm.GetStatus(&g_status);

	for (int i = 0; i < (int)axes.size(); ++i) {
		int ax = axes[i].first;
		const TCHAR* role = axes[i].second;

		TCHAR buf[64];
		LVITEM it{}; it.mask = LVIF_TEXT; it.iItem = i; it.iSubItem = 0;
		_stprintf_s(buf, TEXT("%d"), ax); it.pszText = buf;
		ListView_InsertItem(hList, &it);

		ListView_SetItemText(hList, i, 1, const_cast<LPTSTR>(role));

		const auto& a = g_status.axesStatus[ax];
		const TCHAR* ops = (std::abs((int)std::lround(a.actualVelocity)) > vel_idle_threshold) ? TEXT("MOTION") : TEXT("IDLE");
		ListView_SetItemText(hList, i, 2, const_cast<LPTSTR>(ops));

		_stprintf_s(buf, TEXT("%lld"), (long long)a.posCmd);
		ListView_SetItemText(hList, i, 3, buf);
		_stprintf_s(buf, TEXT("%lld"), (long long)a.actualPos);
		ListView_SetItemText(hList, i, 4, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(a.velocityCmd));
		ListView_SetItemText(hList, i, 5, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(a.actualVelocity));
		ListView_SetItemText(hList, i, 6, buf);

		// Optional conversion: actualVelocity(rpm) -> m/s
		// (Enable per axis in the "m/s 설정..." window; default OFF)
		{
			InitVelConvDefaultsOnce();
			if (ax >= 0 && ax < kSyncUiAxes && g_velToMS_Enable[ax]) {
				double ms = RpmToMps(a.actualVelocity, g_velToMS_WheelDiameterMM[ax]);
				if (std::isfinite(ms)) {
					// 0.0001m/s 단위까지 표시 (필요하면 조정)
					_stprintf_s(buf, TEXT("%.4f"), ms);
				}
				else {
					_stprintf_s(buf, TEXT("-"));
				}
			}
			else {
				_stprintf_s(buf, TEXT("-"));
			}
			ListView_SetItemText(hList, i, 7, buf);
		}
		
		int err603f = 0;
		bool ok603f = ReadAxis_TxPDO_603F(kAxisSlaveId[ax], err603f);
		if (ok603f) _stprintf_s(buf, TEXT("0x%04X"), (unsigned)(err603f & 0xFFFF));
		else _stprintf_s(buf, TEXT("-"));
		ListView_SetItemText(hList, i, 8, buf);
	}
}

// ======================================================
// ===== ActualVel(rpm) -> m/s conversion settings UI ====
// ======================================================
static void VelConv_CreateUI(HWND h) {
	CreateWindow(TEXT("STATIC"), TEXT("축별 ActualVel(rpm) → m/s 변환"), WS_CHILD | WS_VISIBLE,
		10, 10, 460, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("(체크된 축만 변환, 디폴트 OFF)"), WS_CHILD | WS_VISIBLE,
		10, 30, 460, 18, h, nullptr, nullptr, nullptr);

	// Header
	CreateWindow(TEXT("STATIC"), TEXT("Axis"), WS_CHILD | WS_VISIBLE, 10, 55, 40, 18, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Use"), WS_CHILD | WS_VISIBLE, 70, 55, 40, 18, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Wheel D (mm)"), WS_CHILD | WS_VISIBLE, 150, 55, 90, 18, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("PPR(옵션)"), WS_CHILD | WS_VISIBLE, 280, 55, 70, 18, h, nullptr, nullptr, nullptr);

	int y0 = 75;
	for (int ax = 0; ax < kSyncUiAxes; ++ax) {
		int y = y0 + ax * 26;
		TCHAR t[16]; _stprintf_s(t, TEXT("%d"), ax);
		CreateWindow(TEXT("STATIC"), t, WS_CHILD | WS_VISIBLE, 15, y + 4, 40, 18, h, nullptr, nullptr, nullptr);
		CreateWindow(TEXT("BUTTON"), TEXT(""), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
			70, y + 2, 18, 18, h, (HMENU)(INT_PTR)(ID_VELCONV_AXIS_CHK_BASE + ax), nullptr, nullptr);
		CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
			150, y, 100, 22, h, (HMENU)(INT_PTR)(ID_VELCONV_DIAM_EDIT_BASE + ax), nullptr, nullptr);
		CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
			280, y, 90, 22, h, (HMENU)(INT_PTR)(ID_VELCONV_PPR_EDIT_BASE + ax), nullptr, nullptr);
	}

	CreateWindow(TEXT("BUTTON"), TEXT("Apply"), WS_CHILD | WS_VISIBLE,
		150, y0 + kSyncUiAxes * 26 + 10, 80, 28, h, (HMENU)ID_VELCONV_BTN_APPLY, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Close"), WS_CHILD | WS_VISIBLE,
		240, y0 + kSyncUiAxes * 26 + 10, 80, 28, h, (HMENU)ID_VELCONV_BTN_CLOSE, nullptr, nullptr);

	CreateWindow(TEXT("STATIC"),
		TEXT("※ Wheel D(mm)만 있으면 rpm→m/s 변환 가능합니다.\r\n   (PPR은 참고용: 현재 계산에 사용하지 않음)"),
		WS_CHILD | WS_VISIBLE, 10, y0 + kSyncUiAxes * 26 + 45, 460, 40, h, nullptr, nullptr, nullptr);
}

static void VelConv_LoadToControls(HWND h) {
	InitVelConvDefaultsOnce();
	for (int ax = 0; ax < kSyncUiAxes; ++ax) {
		HWND chk = GetDlgItem(h, ID_VELCONV_AXIS_CHK_BASE + ax);
		if (chk) SendMessage(chk, BM_SETCHECK, g_velToMS_Enable[ax] ? BST_CHECKED : BST_UNCHECKED, 0);
		SetDlgDouble(h, ID_VELCONV_DIAM_EDIT_BASE + ax, g_velToMS_WheelDiameterMM[ax]);
		SetDlgInt(h, ID_VELCONV_PPR_EDIT_BASE + ax, g_velToMS_PulsesPerRev[ax]);
	}
}

static void VelConv_ApplyFromControls(HWND h) {
	for (int ax = 0; ax < kSyncUiAxes; ++ax) {
		HWND chk = GetDlgItem(h, ID_VELCONV_AXIS_CHK_BASE + ax);
		g_velToMS_Enable[ax] = (chk && (SendMessage(chk, BM_GETCHECK, 0, 0) == BST_CHECKED));
		g_velToMS_WheelDiameterMM[ax] = GetDlgDoubleOrDefIfInvalid(h, ID_VELCONV_DIAM_EDIT_BASE + ax, 0.0);
		g_velToMS_PulsesPerRev[ax] = GetDlgInt(h, ID_VELCONV_PPR_EDIT_BASE + ax, 0);
	}
}

static void ShowVelConvWindow(HWND hParent) {
	InitVelConvDefaultsOnce();
	if (g_hVelConvWnd && IsWindow(g_hVelConvWnd)) {
		ShowWindow(g_hVelConvWnd, SW_SHOWNORMAL);
		SetForegroundWindow(g_hVelConvWnd);
		VelConv_LoadToControls(g_hVelConvWnd);
		return;
	}

	WNDCLASS wc{};
	wc.lpszClassName = TEXT("WMX3VelConvWnd");
	wc.lpfnWndProc = VelConvWndProc;
	wc.hInstance = (HINSTANCE)GetWindowLongPtr(hParent, GWLP_HINSTANCE);
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	RegisterClass(&wc);

	g_hVelConvWnd = CreateWindow(TEXT("WMX3VelConvWnd"), TEXT("m/s 변환 설정"),
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, 500, 420, hParent, nullptr, wc.hInstance, nullptr);
	ShowWindow(g_hVelConvWnd, SW_SHOWNORMAL);
	UpdateWindow(g_hVelConvWnd);
}

static LRESULT CALLBACK VelConvWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_CREATE:
		VelConv_CreateUI(hWnd);
		VelConv_LoadToControls(hWnd);
		return 0;

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);
		if (id == ID_VELCONV_BTN_APPLY) {
			VelConv_ApplyFromControls(hWnd);
			MessageBox(hWnd, TEXT("적용되었습니다."), TEXT("m/s 변환"), MB_ICONINFORMATION);
			return 0;
		}
		if (id == ID_VELCONV_BTN_CLOSE) {
			DestroyWindow(hWnd);
			return 0;
		}
		return 0;
	}

	case WM_CLOSE:
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		g_hVelConvWnd = nullptr;
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

static void Sync_CreateUI(HWND h) {
	CreateWindow(TEXT("BUTTON"), TEXT("동기 그룹 설정"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 10, 480, 440, h, 0, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("동기 그룹"), WS_CHILD | WS_VISIBLE, 20, 40, 70, 22, h, 0, 0, 0);
	HWND hCmb = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 100, 36, 120, 200, h, (HMENU)ID_SYNC_GROUP_COMBO, 0, 0);
	for (int gid = 0; gid < kNumAxes - 3; ++gid) {
		TCHAR t[16]; _stprintf_s(t, TEXT("Group %d"), gid);
		SendMessage(hCmb, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)t);
	}
	SendMessage(hCmb, CB_SETCURSEL, 0, 0);
	g_syncUi.groupId = 0;

	CreateWindow(TEXT("BUTTON"), TEXT("파라미터"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 500, 10, 520, 250, h, 0, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Servo On/Off Sync"), WS_CHILD | WS_VISIBLE, 510, 40, 120, 22, h, 0, 0, 0);
	HWND cb1 = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 640, 36, 100, 200, h, (HMENU)ID_SYNC_PARAM_SERVO_ONOFF, 0, 0);
	SendMessage(cb1, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Disabled"));
	SendMessage(cb1, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Enabled"));
	SendMessage(cb1, CB_SETCURSEL, 1, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Startup Type"), WS_CHILD | WS_VISIBLE, 510, 70, 120, 22, h, 0, 0, 0);
	HWND cb2 = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 640, 66, 150, 200, h, (HMENU)ID_SYNC_PARAM_STARTUP, 0, 0);
	SendMessage(cb2, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Normal"));
	SendMessage(cb2, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("CatchUp"));
	SendMessage(cb2, CB_SETCURSEL, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Cycle Ratio [0..10]"), WS_CHILD | WS_VISIBLE, 510, 100, 140, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("1"), WS_CHILD | WS_VISIBLE | WS_BORDER, 660, 96, 80, 24, h, (HMENU)ID_SYNC_PARAM_CYCLE_RATIO, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Max Catch Up Dist [p]"), WS_CHILD | WS_VISIBLE, 510, 130, 140, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 660, 126, 80, 24, h, (HMENU)ID_SYNC_PARAM_MAX_CATCH_UP, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("CatchUp Vel [p/s]"), WS_CHILD | WS_VISIBLE, 760, 100, 130, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 900, 96, 100, 24, h, (HMENU)ID_SYNC_PARAM_CATCHUP_VEL, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("CatchUp Acc [p/s^2]"), WS_CHILD | WS_VISIBLE, 760, 130, 130, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 900, 126, 100, 24, h, (HMENU)ID_SYNC_PARAM_CATCHUP_ACC, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Sync Err Tol [p]"), WS_CHILD | WS_VISIBLE, 510, 160, 140, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("1000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 660, 156, 80, 24, h, (HMENU)ID_SYNC_PARAM_TOLERANCE, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Master Desync Dec [p/s^2]"), WS_CHILD | WS_VISIBLE, 760, 160, 160, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("10000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 930, 156, 80, 24, h, (HMENU)ID_SYNC_PARAM_MASTER_DESYNC_DEC, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Slave Desync Dec [p/s^2]"), WS_CHILD | WS_VISIBLE, 760, 190, 160, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("10000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 930, 186, 80, 24, h, (HMENU)ID_SYNC_PARAM_SLAVE_DESYNC_DEC, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Use Master FB"), WS_CHILD | WS_VISIBLE, 510, 190, 120, 22, h, 0, 0, 0);
	HWND cb3 = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 640, 186, 100, 200, h, (HMENU)ID_SYNC_PARAM_USE_MASTER_FB, 0, 0);
	SendMessage(cb3, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Disabled"));
	SendMessage(cb3, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Enabled"));
	SendMessage(cb3, CB_SETCURSEL, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Enable"), WS_CHILD | WS_VISIBLE, 510, 220, 80, 26, h, (HMENU)ID_SYNC_PARAM_ENABLE, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Disable"), WS_CHILD | WS_VISIBLE, 600, 220, 80, 26, h, (HMENU)ID_SYNC_PARAM_DISABLE, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Refresh"), WS_CHILD | WS_VISIBLE, 690, 220, 80, 26, h, (HMENU)ID_SYNC_PARAM_REFRESH, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Control"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 460, 1010, 260, h, 0, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Master Axis"), WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP | WS_TABSTOP, 20, 490, 110, 22, h, (HMENU)ID_SYNC_RAD_MASTER, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Sync Axis"), WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON, 140, 490, 100, 22, h, (HMENU)ID_SYNC_RAD_SLAVE, 0, 0);
	SendMessage(GetDlgItem(h, ID_SYNC_RAD_MASTER), BM_SETCHECK, BST_CHECKED, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Axis#"), WS_CHILD | WS_VISIBLE, 250, 490, 40, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 300, 486, 50, 24, h, (HMENU)ID_SYNC_AXIS_SELECT, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Cmd Pos :"), WS_CHILD | WS_VISIBLE, 20, 520, 90, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 110, 516, 100, 24, h, (HMENU)ID_SYNC_POS_CMD, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Actual Pos :"), WS_CHILD | WS_VISIBLE, 220, 520, 90, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 310, 516, 100, 24, h, (HMENU)ID_SYNC_POS_ACT, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Op Status :"), WS_CHILD | WS_VISIBLE, 420, 520, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("IDLE"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 500, 516, 80, 24, h, (HMENU)ID_SYNC_OPSTATE, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Servo On"), WS_CHILD | WS_VISIBLE, 600, 514, 80, 26, h, (HMENU)ID_SYNC_BTN_SVON, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Home Start"), WS_CHILD | WS_VISIBLE, 690, 514, 80, 26, h, (HMENU)ID_SYNC_BTN_HOME, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Stop"), WS_CHILD | WS_VISIBLE, 780, 514, 60, 26, h, (HMENU)ID_SYNC_BTN_STOP, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Alarm Reset"), WS_CHILD | WS_VISIBLE, 845, 514, 100, 26, h, (HMENU)ID_SYNC_BTN_ALARM_RST, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Jog Speed"), WS_CHILD | WS_VISIBLE, 20, 555, 70, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("100000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 95, 551, 80, 24, h, (HMENU)ID_SYNC_JOG_SPEED, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Accel/Decel"), WS_CHILD | WS_VISIBLE, 190, 555, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("100"), WS_CHILD | WS_VISIBLE | WS_BORDER, 270, 551, 80, 24, h, (HMENU)ID_SYNC_ACC, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("100"), WS_CHILD | WS_VISIBLE | WS_BORDER, 355, 551, 80, 24, h, (HMENU)ID_SYNC_DEC, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Jerk"), WS_CHILD | WS_VISIBLE, 450, 555, 40, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0.75"), WS_CHILD | WS_VISIBLE | WS_BORDER, 490, 551, 60, 24, h, (HMENU)ID_SYNC_JERK, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Cmd Vel"), WS_CHILD | WS_VISIBLE, 560, 555, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 645, 551, 80, 24, h, (HMENU)ID_SYNC_CMD_VEL, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Actual Vel"), WS_CHILD | WS_VISIBLE, 740, 555, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 825, 551, 80, 24, h, (HMENU)ID_SYNC_ACT_VEL, 0, 0);

	HWND hSyncJogCcw = CreateWindow(TEXT("BUTTON"), TEXT("JOG CCW"), WS_CHILD | WS_VISIBLE, 915, 545, 80, 30, h, (HMENU)ID_SYNC_BTN_JOG_CCW, 0, 0);
	if (hSyncJogCcw) AttachHoldToJogButton(hSyncJogCcw, JogBtnKind::Sync, -1, -1);
	HWND hSyncJogCw = CreateWindow(TEXT("BUTTON"), TEXT("JOG CW"), WS_CHILD | WS_VISIBLE, 915, 580, 80, 30, h, (HMENU)ID_SYNC_BTN_JOG_CW, 0, 0);
	if (hSyncJogCw) AttachHoldToJogButton(hSyncJogCw, JogBtnKind::Sync, -1, +1);

	CreateWindow(TEXT("STATIC"), TEXT("Abs Pos"), WS_CHILD | WS_VISIBLE, 20, 590, 60, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 80, 586, 100, 24, h, (HMENU)ID_SYNC_ABS_POS, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("AbsMove"), WS_CHILD | WS_VISIBLE, 185, 584, 80, 26, h, (HMENU)ID_SYNC_BTN_ABS, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Alt Target"), WS_CHILD | WS_VISIBLE, 280, 590, 70, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 350, 586, 100, 24, h, (HMENU)ID_SYNC_ALT_TARGET, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Apply Alt→Abs"), WS_CHILD | WS_VISIBLE, 455, 584, 110, 26, h, (HMENU)ID_SYNC_BTN_APPLY_ALT, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Axis0 0x6063:"), WS_CHILD | WS_VISIBLE, 575, 590, 190, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("-"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 675, 586, 100, 24, h, (HMENU)ID_SYNC_ECAT_6063, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("비상정지"), WS_CHILD | WS_VISIBLE, 20, 625, 90, 26, h, (HMENU)ID_SYNC_BTN_ESTOP_TOGGLE, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("NORMAL"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 115, 627, 105, 24, h, (HMENU)ID_SYNC_TXT_ESTOP_STATE, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("동기 그룹 모니터"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 730, 1010, 220, h, 0, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("그룹상태:"), WS_CHILD | WS_VISIBLE, 20, 760, 70, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("Disabled"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 90, 756, 100, 24, h, (HMENU)ID_SYNC_STATE_ENABLED, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("Home Not Done"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 200, 756, 120, 24, h, (HMENU)ID_SYNC_STATE_HOMEDONE, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("All Servo On"), WS_CHILD | WS_VISIBLE, 350, 754, 100, 26, h, (HMENU)ID_SYNC_ALL_SERVO_ON, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("All Servo Off"), WS_CHILD | WS_VISIBLE, 455, 754, 100, 26, h, (HMENU)ID_SYNC_ALL_SERVO_OFF, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Home Start"), WS_CHILD | WS_VISIBLE, 560, 754, 90, 26, h, (HMENU)ID_SYNC_GROUP_HOME, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Clear Error"), WS_CHILD | WS_VISIBLE, 655, 754, 90, 26, h, (HMENU)ID_SYNC_GROUP_CLEAR, 0, 0);

	HWND hAxisList = CreateWindow(WC_LISTVIEW, TEXT(""), WS_CHILD | WS_VISIBLE | WS_BORDER | LVS_REPORT | LVS_SINGLESEL,
		20, 790, 980, 150, h, (HMENU)ID_SYNC_AXIS_LIST, 0, 0);
	ListView_SetExtendedListViewStyle(hAxisList, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
	Sync_CreateAxisListColumns(hAxisList);

	Sync_LoadGroupParamsToUI(h);
	UpdateEStopUi(h, true);
}

static LRESULT CALLBACK SyncWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_CREATE:
		Sync_CreateUI(hWnd);
		SetTimer(hWnd, ID_SYNC_TIMER, 100, nullptr);
		return 0;

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);

		if (id == ID_SYNC_GROUP_COMBO && HIWORD(wParam) == CBN_SELCHANGE) {
			int sel = (int)SendMessage((HWND)lParam, CB_GETCURSEL, 0, 0);
			g_syncUi.groupId = sel;
			Sync_LoadGroupParamsToUI(hWnd);
			return 0;
		}
		if (id == ID_SYNC_PARAM_REFRESH) { Sync_LoadGroupParamsToUI(hWnd); return 0; }

		if (id >= ID_SYNC_MASTER_AXIS_BTN_BASE && id < ID_SYNC_MASTER_AXIS_BTN_BASE + 64) {
			for (int a : g_syncUi.detectedAxes) {
				SendMessage(GetDlgItem(hWnd, ID_SYNC_MASTER_AXIS_BTN_BASE + a), BM_SETCHECK, BST_UNCHECKED, 0);
			}
			int ax = id - ID_SYNC_MASTER_AXIS_BTN_BASE;
			SendMessage(GetDlgItem(hWnd, id), BM_SETCHECK, BST_CHECKED, 0);
			g_syncUi.masterAxis = ax;
			return 0;
		}

		if (id == ID_SYNC_PARAM_ENABLE) {
			if (Sync_ApplyGroupParamsFromUI(hWnd)) Sync_EnableGroup(hWnd, true);
			return 0;
		}
		if (id == ID_SYNC_PARAM_DISABLE) { Sync_EnableGroup(hWnd, false); return 0; }

		if (id == ID_SYNC_ALL_SERVO_ON) { Sync_AllServoOnOff(true); return 0; }
		if (id == ID_SYNC_ALL_SERVO_OFF) { Sync_AllServoOnOff(false); return 0; }
		if (id == ID_SYNC_GROUP_CLEAR) { Sync_ClearGroupError(); return 0; }
		if (id == ID_SYNC_GROUP_HOME) {
			Sync::SyncGroup grp{}; if (g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp) == ErrorCode::None) {
				int ax = grp.masterAxis;
				EnsureServoOn(ax);
				if (EnterPosMode(ax)) g_home.StartHome(ax);
			}
			return 0;
		}

		if (id == ID_SYNC_BTN_SVON) {
			int ax = Sync_GetControlAxis(hWnd);
			EnsureServoOn(ax); EnsurePosModeNoStop(ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_HOME) {
			int ax = Sync_GetControlAxis(hWnd);
			EnsureServoOn(ax); if (EnterPosMode(ax)) g_home.StartHome(ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_STOP) {
			int ax = Sync_GetControlAxis(hWnd);
			Sync_Control_Stop(hWnd, ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_ALARM_RST) {
			int ax = Sync_GetControlAxis(hWnd);
			g_cm.axisControl->ClearAmpAlarm(ax);
			return 0;
		}
		// Sync Jog: hold-to-jog (press = start, release = stop)
		if (id == ID_SYNC_BTN_JOG_CCW || id == ID_SYNC_BTN_JOG_CW) {
			int sign = (id == ID_SYNC_BTN_JOG_CCW) ? -1 : +1;
			int code = HIWORD(wParam);

			if (code == BN_PUSHED) {
				if (!IsManualAllowed(hWnd)) return 0;

				int ax = Sync_GetControlAxis(hWnd);

				double vpps = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
				double tAcc = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
				double tDec = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);

				StartJogWithProfileParams(hWnd, ax, sign, vpps, tAcc, tDec);
				return 0;
			}
			if (code == BN_UNPUSHED) {
				StopJogIfActive(); StopMultiJog(); ReleaseCapture();
				return 0;
			}

			// BN_CLICKED는 무시
			return 0;
		}
		if (id == ID_SYNC_BTN_ABS) { Sync_Control_Abs(hWnd); return 0; }
		if (id == ID_SYNC_BTN_REL_P) { Sync_Control_Rel(hWnd, +1); return 0; }
		if (id == ID_SYNC_BTN_REL_M) { Sync_Control_Rel(hWnd, -1); return 0; }

		if (id == ID_SYNC_BTN_APPLY_ALT) {
			int ecat6063 = 0;
			bool ok = ReadAxis0_TxPDO_6063(ecat6063);
			double alt = GetDlgDouble(hWnd, ID_SYNC_ALT_TARGET, 0.0);
			if (!ok) {
				MessageBox(hWnd, TEXT("0x6063 값을 읽을 수 없습니다."), TEXT("Sync Apply Alt→Abs"), MB_ICONWARNING);
				return 0;
			}
			double tgt = alt - (double)ecat6063;
			SetDlgDouble(hWnd, ID_SYNC_ABS_POS, tgt);
			return 0;
		}

		if (id == ID_SYNC_RAD_MASTER || id == ID_SYNC_RAD_SLAVE) {
			if (id == ID_SYNC_RAD_MASTER) {
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_MASTER), BM_SETCHECK, BST_CHECKED, 0);
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_SLAVE), BM_SETCHECK, BST_UNCHECKED, 0);
			}
			else {
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_MASTER), BM_SETCHECK, BST_UNCHECKED, 0);
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_SLAVE), BM_SETCHECK, BST_CHECKED, 0);
			}
			return 0;
		}



		if (id == ID_SYNC_BTN_ESTOP_TOGGLE) { DoToggleEStop(hWnd, true); return 0; }

	}
	return 0;

	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
	case WM_MBUTTONUP:
	case WM_KILLFOCUS:
	case WM_CANCELMODE:
		StopJogIfActive(); StopMultiJog(); ReleaseCapture(); return 0;


	case WM_TIMER:
		if (wParam == ID_SYNC_TIMER) {
			if (g_commStarted) {
				g_cm.GetStatus(&g_status);

				int ax = Sync_GetControlAxis(hWnd);
				const auto& a = g_status.axesStatus[ax];
				TCHAR b[64];
				_stprintf_s(b, TEXT("%lld"), (long long)a.posCmd); SetWindowText(GetDlgItem(hWnd, ID_SYNC_POS_CMD), b);
				_stprintf_s(b, TEXT("%lld"), (long long)a.actualPos); SetWindowText(GetDlgItem(hWnd, ID_SYNC_POS_ACT), b);
				const TCHAR* ops = (std::abs((int)std::lround(a.actualVelocity)) > vel_idle_threshold) ? TEXT("MOTION") : TEXT("IDLE");
				SetWindowText(GetDlgItem(hWnd, ID_SYNC_OPSTATE), (LPTSTR)ops);
				_stprintf_s(b, TEXT("%d"), (int)std::lround(a.velocityCmd)); SetWindowText(GetDlgItem(hWnd, ID_SYNC_CMD_VEL), b);
				_stprintf_s(b, TEXT("%d"), (int)std::lround(a.actualVelocity)); SetWindowText(GetDlgItem(hWnd, ID_SYNC_ACT_VEL), b);

				if (HWND h6063 = GetDlgItem(hWnd, ID_SYNC_ECAT_6063)) {
					int val = 0;
					if (ReadAxis0_TxPDO_6063(val)) {
						TCHAR t[64]; _stprintf_s(t, TEXT("%d"), val);
						SetWindowText(h6063, t);
					}
					else {
						SetWindowText(h6063, TEXT("-"));
					}
				}

				Sync_UpdateMonitor(hWnd);
			}
			UpdateEStopUi(hWnd, true);
		}
		return 0;

	case WM_CLOSE:
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		KillTimer(hWnd, ID_SYNC_TIMER);
		//DisableAllEnabledSyncGroups();
		g_hSyncWnd = nullptr;
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

static bool ExclusiveStopExcept(const std::vector<int>& allowed, DWORD timeout_ms = 5000) {
	for (int a = 0; a < kNumAxes - 3; ++a) if (!std::count(allowed.begin(), allowed.end(), a)) StopAxis(a);
	std::vector<int> waitAxes; waitAxes.reserve(kNumAxes - 3);
	for (int a = 0; a < kNumAxes - 3; ++a) if (!std::count(allowed.begin(), allowed.end(), a)) waitAxes.push_back(a);
	if (!waitAxes.empty()) return WaitAxesIdle(waitAxes, timeout_ms);
	return true;
}

static void DisableAllEnabledSyncGroups() {
	if (!g_commStarted) return;

	for (int gid = 0; gid < kNumAxes - 3; ++gid) {
		Sync::SyncGroupStatus s{};
		if (g_cm.sync->GetSyncGroupStatus(gid, &s) == ErrorCode::None && s.enabled) {
			g_cm.sync->EnableSyncGroup(gid, 0);
			Sleep(5);
		}

		Sync::SyncGroup grp{};
		if (g_cm.sync->GetSyncGroup(gid, &grp) == ErrorCode::None) {
			auto setPos = [&](int ax) {
				g_cm.axisControl->SetAxisCommandMode(ax, AxisCommandMode::Position);
				};
			setPos(grp.masterAxis);
			for (int i = 0; i < (int)grp.slaveAxisCount; ++i)
				setPos(grp.slaveAxis[i]);
		}

		g_cm.sync->ClearSyncGroupError(gid);
	}
}

// ------------------ 상태 갱신 ------------------
static void UpdateStatus(HWND hWnd) {
	for (int a = 0; a < kNumAxes - 3; ++a) {
		const auto& ax = g_status.axesStatus[a];
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 0))) SetWindowText(h, ax.servoOn ? (TCHAR*)TEXT("ON") : (TCHAR*)TEXT("OFF"));

		long long perr = (long long)ax.posCmd - (long long)ax.actualPos;
		bool inpos = std::llabs(perr) <= inpos_tol_counts;
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 1))) SetWindowText(h, inpos ? (TCHAR*)TEXT("IN POSITION") : (TCHAR*)TEXT("OUT OF POSITION"));

		int actVel = (int)std::lround(ax.actualVelocity);
		bool motioning = (std::abs(actVel) > vel_idle_threshold);
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 2))) SetWindowText(h, motioning ? (TCHAR*)TEXT("MOTION") : (TCHAR*)TEXT("IDLE"));

		TCHAR buf[64];
		_stprintf_s(buf, TEXT("%lld"), (long long)ax.posCmd); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 3))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%lld"), (long long)ax.actualPos); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 4))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(ax.velocityCmd)); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 5))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), actVel); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 6))) SetWindowText(h, buf);

		// ActVel(m/s) 출력: UI에서 축별로 Enable된 경우에만 표시 (기어비는 이미 적용된 rpm으로 가정)
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 7))) {
			if (a >= 0 && a < kSyncUiAxes && g_velToMS_Enable[a]) {
				double ms = RpmToMps(ax.actualVelocity, g_velToMS_WheelDiameterMM[a]);
				if (std::isfinite(ms)) {
					_stprintf_s(buf, TEXT("%.3f"), ms);
					SetWindowText(h, buf);
				}
				else {
					SetWindowText(h, TEXT("-"));
				}
			}
			else {
				SetWindowText(h, TEXT("-"));
			}
		}

		_stprintf_s(buf, TEXT("%d"), (int)std::lround(ax.torqueCmd)); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 8))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(ax.actualTorque)); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 9))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%lld"), perr); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 10))) SetWindowText(h, buf);

		bool ampAlm = false;
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 11))) SetWindowText(h, ampAlm ? (TCHAR*)TEXT("ALARM") : (TCHAR*)TEXT("OK"));

		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 12))) {
			int err603f = 0;
			bool ok603f = ReadAxis_TxPDO_603F(kAxisSlaveId[a], err603f);
			if (ok603f) {
				TCHAR t[64]; _stprintf_s(t, TEXT("0x%04X"), (unsigned)(err603f & 0xFFFF));
				SetWindowText(h, t);
			}
			else {
				SetWindowText(h, TEXT("-"));
			}
		}
	}

	if (HWND hTxt = GetDlgItem(hWnd, ID_TXT_ECAT_6063)) {
		int val = 0;
		if (ReadAxis0_TxPDO_6063(val)) {
			TCHAR t[64]; _stprintf_s(t, TEXT("%d"), val);
			SetWindowText(hTxt, t);
		}
		else {
			SetWindowText(hTxt, TEXT("-"));
		}
	}
	if (HWND hTxt = GetDlgItem(hWnd, ID_TXT_ECAT_603F)) {
		int val = 0;
		if (ReadAxis_TxPDO_603F(kAxisSlaveId[0], val)) {
			TCHAR t[64]; _stprintf_s(t, TEXT("0x%04X"), (unsigned)(val & 0xFFFF));
			SetWindowText(hTxt, t);
		}
		else {
			SetWindowText(hTxt, TEXT("-"));
		}
	}
	if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) {
		SetWindowText(h, g_ax2ServoReady ? (g_ax2LimitOn ? TEXT("ON") : TEXT("OFF")) : TEXT("WAIT"));
	}
	if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME)) {
		SetWindowText(h, g_ax2ServoReady ? (g_ax2HomeOn ? TEXT("ON") : TEXT("OFF")) : TEXT("WAIT"));
	}

	if (HWND h = GetDlgItem(hWnd, ID_TXT_MODE_STATE)) {
		SetWindowText(h, g_autoMode ? TEXT("MODE: AUTO") : TEXT("MODE: MANUAL"));
	}

	UpdateEStopUi(hWnd, false);
	UpdateTcpUiState(hWnd);
}

void EnsureDirW(const wchar_t* path) {
	if (!path || !path[0]) return;
	// CreateDirectoryW는 하위 한 단계만 만들 수 있음. 전체 트리를 보장하기 위해 분해 생성.
	wchar_t tmp[MAX_PATH];
	wcsncpy_s(tmp, path, _TRUNCATE);
	size_t len = wcslen(tmp);
	for (size_t i = 0; i < len; ++i) {
		if (tmp[i] == L'/' || tmp[i] == L'\\') {
			wchar_t c = tmp[i];
			tmp[i] = 0;
			if (wcslen(tmp) > 0) CreateDirectoryW(tmp, nullptr);
			tmp[i] = c;
		}
	}
	CreateDirectoryW(tmp, nullptr);
}

static void Serial_CreateUI(HWND h)
{
	CreateWindow(TEXT("BUTTON"), TEXT("PLC TCP Monitor"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 10, 765, 340, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Bind IP"), WS_CHILD | WS_VISIBLE, 20, 40, 60, 20, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), g_tcpBindIp, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, 80, 36, 120, 24, h, (HMENU)ID_EDIT_TCP_IP, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Port"), WS_CHILD | WS_VISIBLE, 210, 40, 40, 20, h, 0, 0, 0);
	TCHAR portText[16]; _stprintf_s(portText, TEXT("%d"), g_tcpBindPort);
	CreateWindow(TEXT("EDIT"), portText, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, 250, 36, 70, 24, h, (HMENU)ID_EDIT_TCP_PORT, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Start TCP"), WS_CHILD | WS_VISIBLE, 330, 35, 90, 26, h, (HMENU)ID_BTN_TCP_START, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Stop TCP"), WS_CHILD | WS_VISIBLE, 430, 35, 90, 26, h, (HMENU)ID_BTN_TCP_STOP, 0, 0);
	g_hTcpLogList = CreateWindow(TEXT("LISTBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | WS_BORDER | LBS_NOINTEGRALHEIGHT | WS_VSCROLL,
		20, 80, 700, 110, h, (HMENU)ID_LIST_TCP_LOG, 0, 0);
}

static LRESULT CALLBACK SerialWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE:
		Serial_CreateUI(hWnd);
		return 0;

	case WM_COMMAND:
		if (LOWORD(wParam) == ID_BTN_TCP_START) {
			wchar_t ip[64] = L"0.0.0.0";
			wchar_t portStr[32] = L"9100";
			GetWindowText(GetDlgItem(hWnd, ID_EDIT_TCP_IP), ip, 64);
			GetWindowText(GetDlgItem(hWnd, ID_EDIT_TCP_PORT), portStr, 32);
			wcsncpy_s(g_tcpBindIp, ip, _TRUNCATE);
			g_tcpBindPort = _wtoi(portStr);
			if (g_tcpBindPort <= 0) g_tcpBindPort = 9100;
			StartTcpServer();
			return 0;
		}
		if (LOWORD(wParam) == ID_BTN_TCP_STOP) {
			StopTcpServer();
			return 0;
		}
		return 0;

	case WM_SIZE:
	{
		RECT rc{}; GetClientRect(hWnd, &rc);
		int cx = rc.right - rc.left;
		int cy = rc.bottom - rc.top;
		if (HWND lb = GetDlgItem(hWnd, ID_LIST_TCP_LOG)) {
			SetWindowPos(lb, nullptr, 20, 80, cx - 40, cy - 100, SWP_NOZORDER);
		}
		return 0;
	}

	case WM_CLOSE:
		// 닫을 때 그냥 숨기기
		ShowWindow(hWnd, SW_HIDE);
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		if (g_hTcpLogList && !IsWindow(g_hTcpLogList)) g_hTcpLogList = nullptr;
		g_hSerialWnd = nullptr;
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}



// =====================================================
// Hybrid Barcode Demo (Stop-distance based auto decel)
// Copy-paste whole block
// =====================================================

HWND g_hBarcodeWnd = nullptr;

enum : int {
	ID_BC_TIMER = 30001,
	ID_BC_ECAT_6063_NOW = 30002,
	ID_BC_TXT_STATUS = 30030,

	ID_BC_EDIT_TARGET = 30010,
	ID_BC_EDIT_VEL = 30011,
	ID_BC_EDIT_ACC = 30012,
	ID_BC_EDIT_DEC = 30013,

	// Correction profile controls
	ID_BC_EDIT_CORR_VEL = 30101,
	ID_BC_EDIT_CORR_ACC = 30102,
	ID_BC_EDIT_CORR_DEC = 30103,

	ID_BC_EDIT_GEAR = 30040,
	ID_BC_EDIT_WHEEL_D = 30041,
	ID_BC_EDIT_MOT_CPR = 30042,
	ID_BC_EDIT_BC_MM_PER_CNT = 30043,
	ID_BC_TXT_COMPUTED_PULSES = 30050,

	// NOTE: 기존 ID 이름 유지 (표시는 Static로 사용 가능)
	ID_BC_EDIT_CORR_DEADBAND = 30106,     // (표시용: Arrive band)
	ID_BC_EDIT_CORR_START_BCERR = 30107,  // (표시용: Auto Corr EntryDist)

	ID_BC_BTN_SERVO_ON = 30020,
	ID_BC_BTN_SERVO_OFF = 30021,
	ID_BC_BTN_START = 30022,
	ID_BC_BTN_STOP = 30023,
	ID_BC_COMBO_AXIS = 30150,
};

// ======================================================
// UI helpers
static void Barcode_SetTxt(HWND h, int id, const wchar_t* s)
{
	if (HWND hh = GetDlgItem(h, id)) SetWindowTextW(hh, s);
}
static void Barcode_SetInt(HWND h, int id, int v)
{
	wchar_t b[64]; _snwprintf_s(b, _TRUNCATE, L"%d", v);
	Barcode_SetTxt(h, id, b);
}
static void Barcode_SetLL(HWND h, int id, long long v)
{
	wchar_t b[64]; _snwprintf_s(b, _TRUNCATE, L"%lld", v);
	Barcode_SetTxt(h, id, b);
}

// =====================================================
// State
struct HybridBarcodeState
{
	std::atomic<bool> running{ false };

	int axis = 7;

	long long targetBarcodeAbs = 0;     // final target barcode abs (6063)
	long long targetBarcodeRel = 0;     // final target - now (cnt)
	long long remainingMotorPulse = 0;  // remaining pulses (final target 기준)

	// ====== 2-Stage control ======
	// COARSE: 목표 10cnt 전(coarseTarget)으로 접근
	// COARSE_BRAKE: StopAxis 없이 "프로파일 감속으로" 속도 0 도달 대기
	// FINE: 1cnt(step)씩 ±2cnt 안으로 들어갈 때까지 접근
	enum Phase { IDLE = 0, COARSE = 1, COARSE_BRAKE = 2, FINE = 3, DONE = 4 };
	Phase phase = IDLE;

	int preStopCnt = 10;                  // ✅ 목표보다 10cnt 전에 정지(속도0)
	long long coarseTargetBarcodeAbs = 0; // 목표-10cnt의 임시 목표(6063 abs)

	int coarseArriveCnt = 1;              // coarse 목표에 ±1cnt
	int coarseStableTicksNeed = 5;        // 30ms*5=150ms 안정화
	int coarseStableTicks = 0;

	// ✅ "프로파일 정지" 파라미터(StopAxis 대신 사용)
	double coarseBrakeVelPps = 800.0;     // 정지 명령 시 vel 상한(너무 작을 필요 없음)
	double coarseBrakeAccMs = 500.0;
	double coarseBrakeDecMs = 500.0;      // 작을수록 더 급감속(충격↑), 크면 부드러움↑
	double coarseStopVelThreshPps = 80.0; // 이 이하이면 '거의 0속도'로 판정

	ULONGLONG fineStartCooldownUntil = 0; // coarse stop 이후 잠깐 대기 후 fine 시작

	// Main profile (UI)
	double mainVel = 10000; // pps
	double mainAcc = 1000;  // ms
	double mainDec = 1000;  // ms

	// Correction profile (UI)
	double corrVel = 1000;  // pps
	double corrAcc = 300;   // ms
	double corrDec = 300;   // ms

	// Arrival (barcode) - final
	int arriveCnt = 2;                 // final: ±2 cnt
	int arriveStableTicksNeed = 10;    // 30ms*10=300ms
	int arriveStableTicks = 0;

	// Conversion (UI)
	double gear = 4.4248;
	double wheelDia = 115;     // mm
	double motorCpr = 10000;   // pulses per motor rev
	double bcMmPerCnt = 1.07;  // mm per barcode count

	// Overshoot detect
	int lastErrSign = 0;
	int signFlipTicks = 0;
	bool forbidReverse = true;

	// command throttling
	double lastCmdVel = 0.0;
	long long lastCmdTarget = 0;
	ULONGLONG lastCmdTick = 0;

	// close-in creep (fine)
	int creepCnt = 50;          // 50cnt 이내면 더 느리게
	double creepVel = 200.0;    // pps

	// fine step limit (overshoot 방지용)
	int fineStepCntMax = 1;     // fine에서 한 번에 최대 Ncnt만 이동 (1 추천)

	// 표시용: 자동 corr 진입 거리(펄스)
	long long autoCorrEntryDistPulse = 0;

	// --- measured decel estimator (pps^2) ---
	double vPrevPps = 0.0;
	double aDecEma = 0.0;
	bool   havePrevV = false;

	// Stop cooldown (startpos/명령충돌 방지)
	ULONGLONG stopCooldownUntil = 0;
};
static HybridBarcodeState g_hbc;

static inline int HBC_signll(long long v) { return (v > 0) - (v < 0); }

struct HbcSnapshot {
	double gear, wheelDia, motorCpr, bcMmPerCnt;
};

// ✅ gear 반영 (필수)
static inline double HBC_pulsesPerMm(const HbcSnapshot& s)
{
	// pulses per wheel rev = motorCpr * gear
	// wheel circumference = pi * wheelDia
	return s.motorCpr / (3.14159265358979323846 * s.wheelDia);
}
static inline double HBC_bcToMm(const HbcSnapshot& s, long long bc)
{
	return (double)bc * s.bcMmPerCnt;
}
static inline long long HBC_mmToPulses(const HbcSnapshot& s, double mm)
{
	double pulses = mm * HBC_pulsesPerMm(s);
	return (long long)std::llround(pulses);
}
static inline long long HBC_bcToPulses(const HbcSnapshot& s, long long bc)
{
	return HBC_mmToPulses(s, HBC_bcToMm(s, bc));
}

// motor rpm -> pps
static inline double HBC_RpmToPps(double motorRpm, double motorCpr)
{
	return std::fabs(motorRpm) * motorCpr / 60.0;
}

// profile decel model (pps^2), acc/dec(ms) = time-to-reach-vel
static inline double HBC_DecelPps2_Model(double profileVelPps, double decMs, double safetyFactor /* <1 => conservative */)
{
	double t = std::max(1.0, decMs) / 1000.0; // sec
	double a = profileVelPps / t;             // pps^2
	safetyFactor = std::clamp(safetyFactor, 0.05, 1.0);
	a *= safetyFactor;
	return std::max(1.0, a);
}
static inline double HBC_StopDistPulses(double vPps, double aDecPps2)
{
	return (vPps * vPps) / (2.0 * std::max(1.0, aDecPps2));
}
static inline double HBC_VelLimitFromDist(double distPulses, double aDecPps2)
{
	if (distPulses <= 0) return 0.0;
	return std::sqrt(2.0 * std::max(1.0, aDecPps2) * distPulses);
}

// =====================================================
// ✅ Selected axis barcode read (axis=9 고정 제거)
static bool Bc_ReadSelectedAxis_6063(int& outVal)
{
	int axis = 9;
	if (axis < 0 || axis >= kNumAxes) return false;
	return ReadAxis_TxPDO_6063(kAxisSlaveId[axis], outVal);
}

// =====================================================
// ✅ StopAxis 대신 "현재 위치로 AbsMove"를 보내서 프로파일 감속으로 0속도 만들기
static void HBC_ProfileStop(int ax, double vel_pps, double acc_ms, double dec_ms)
{
	// 현재 위치를 목표로 AbsMove를 보내면, 드라이브는 감속해서 정지(0속도)하게 됨
	g_cm.GetStatus(&g_status);
	long long curPos = g_status.axesStatus[ax].actualPos;

	vel_pps = std::max(50.0, vel_pps);
	acc_ms = std::max(1.0, acc_ms);
	dec_ms = std::max(1.0, dec_ms);

	StartAbsMoveWithProfile(ax, curPos, vel_pps, acc_ms, dec_ms);

	g_hbc.lastCmdVel = vel_pps;
	g_hbc.lastCmdTarget = curPos;
	g_hbc.lastCmdTick = GetTickCount64();
}

// =====================================================
// ✅ Throttle: 속도변경뿐 아니라 "target 변경"도 반영
static void HBC_SendMoveThrottled(
	int ax,
	long long absTarget,
	double vel_pps,
	double acc_ms,
	double dec_ms,
	double velChangeRatio,
	long long minTargetDeltaPulses,
	DWORD  minPeriodMs
)
{
	if (!g_hbc.running) return;

	ULONGLONG now = GetTickCount64();
	if (now < g_hbc.stopCooldownUntil) return;        // Stop 직후 명령 금지
	if (now < g_hbc.fineStartCooldownUntil) return;   // coarse→fine 전환 직후 대기

	bool periodOk = (now - g_hbc.lastCmdTick) >= (ULONGLONG)minPeriodMs;

	double lastV = g_hbc.lastCmdVel;
	bool velChanged = (lastV <= 1.0) ? true : ((std::fabs(vel_pps - lastV) / lastV) >= velChangeRatio);

	long long lastT = g_hbc.lastCmdTarget;
	bool targetChanged = (std::llabs(absTarget - lastT) >= std::max(1LL, minTargetDeltaPulses));

	if (periodOk && (velChanged || targetChanged))
	{
		StartAbsMoveWithProfile(ax, absTarget, vel_pps, acc_ms, dec_ms);
		g_hbc.lastCmdVel = vel_pps;
		g_hbc.lastCmdTarget = absTarget;
		g_hbc.lastCmdTick = now;
	}
}

// =====================================================
void HBC_Start(HWND hWnd)
{
	if (!g_commStarted) {
		MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Barcode"), MB_ICONWARNING);
		return;
	}

	g_hbc.running = false;

	// resets
	g_hbc.phase = HybridBarcodeState::IDLE;
	g_hbc.arriveStableTicks = 0;
	g_hbc.coarseStableTicks = 0;
	g_hbc.lastErrSign = 0;
	g_hbc.signFlipTicks = 0;
	g_hbc.stopCooldownUntil = 0;
	g_hbc.fineStartCooldownUntil = 0;

	// Measured decel reset
	g_hbc.havePrevV = false;
	g_hbc.vPrevPps = 0.0;
	g_hbc.aDecEma = 0.0;

	// Read UI parameters - Main profile
	g_hbc.mainVel = GetDlgDouble(hWnd, ID_BC_EDIT_VEL, 10000);
	g_hbc.mainAcc = GetDlgDouble(hWnd, ID_BC_EDIT_ACC, 1000);
	g_hbc.mainDec = GetDlgDouble(hWnd, ID_BC_EDIT_DEC, 1000);

	// Correction profile
	g_hbc.corrVel = GetDlgDouble(hWnd, ID_BC_EDIT_CORR_VEL, 1000);
	g_hbc.corrAcc = GetDlgDouble(hWnd, ID_BC_EDIT_CORR_ACC, 300);
	g_hbc.corrDec = GetDlgDouble(hWnd, ID_BC_EDIT_CORR_DEC, 300);

	// Conversion
	g_hbc.gear = GetDlgDouble(hWnd, ID_BC_EDIT_GEAR, 4.4248);
	g_hbc.wheelDia = GetDlgDouble(hWnd, ID_BC_EDIT_WHEEL_D, 115);
	g_hbc.motorCpr = GetDlgDouble(hWnd, ID_BC_EDIT_MOT_CPR, 10000);
	g_hbc.bcMmPerCnt = GetDlgDouble(hWnd, ID_BC_EDIT_BC_MM_PER_CNT, 1.07);

	// Arrive 설정 (final)
	g_hbc.arriveCnt = 2;
	g_hbc.arriveStableTicksNeed = 10;

	// Pre-stop 설정 (10cnt)
	g_hbc.preStopCnt = 10;
	g_hbc.coarseArriveCnt = 1;
	g_hbc.coarseStableTicksNeed = 5;

	int now6063 = 0;
	if (!Bc_ReadSelectedAxis_6063(now6063)) {
		MessageBox(hWnd, TEXT("Barcode Read Fail (6063)"), TEXT("Barcode"), MB_ICONWARNING);
		return;
	}

	// Final target barcode abs
	g_hbc.targetBarcodeAbs = (long long)GetDlgDouble(hWnd, ID_BC_EDIT_TARGET, 0);

	// 방향(부호) 결정
	long long finalErr = g_hbc.targetBarcodeAbs - (long long)now6063;
	int dir = HBC_signll(finalErr);
	if (dir == 0) {
		g_hbc.phase = HybridBarcodeState::FINE;
		g_hbc.running = true;
		g_hbc.lastErrSign = 0;
		return;
	}

	// COARSE 임시 목표: 목표보다 10cnt 앞(방향 기준)
	g_hbc.coarseTargetBarcodeAbs = g_hbc.targetBarcodeAbs - (long long)dir * (long long)g_hbc.preStopCnt;

	// initial sign
	g_hbc.lastErrSign = dir;

	// 초기 move는 COARSE 목표로 한 번 보내고, 이후 Poll에서 피드백 보정
	g_cm.GetStatus(&g_status);
	int ax = g_hbc.axis;
	long long curPos = g_status.axesStatus[ax].actualPos;

	HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
	long long coarseRel = (long long)g_hbc.coarseTargetBarcodeAbs - (long long)now6063;
	long long coarsePulses = HBC_bcToPulses(s, coarseRel);
	long long absTarget = curPos + coarsePulses;

	StartAbsMoveWithProfile(ax, absTarget, g_hbc.mainVel, g_hbc.mainAcc, g_hbc.mainDec);
	g_hbc.lastCmdVel = g_hbc.mainVel;
	g_hbc.lastCmdTarget = absTarget;
	g_hbc.lastCmdTick = GetTickCount64();

	g_hbc.phase = HybridBarcodeState::COARSE;
	g_hbc.running = true;
}

// =====================================================
void HBC_UpdateUi(HWND hWnd)
{
	int now6063 = 0;
	if (Bc_ReadSelectedAxis_6063(now6063))
		Barcode_SetInt(hWnd, ID_BC_ECAT_6063_NOW, now6063);
	else
		Barcode_SetTxt(hWnd, ID_BC_ECAT_6063_NOW, L"-");

	long long userTarget = (long long)GetDlgDouble(hWnd, ID_BC_EDIT_TARGET, 0.0);
	g_hbc.targetBarcodeAbs = userTarget;

	long long bcErr = userTarget - (long long)now6063;
	g_hbc.targetBarcodeRel = bcErr;

	HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
	long long pulses = HBC_bcToPulses(s, bcErr);
	g_hbc.remainingMotorPulse = pulses;

	Barcode_SetLL(hWnd, ID_BC_TXT_COMPUTED_PULSES, pulses);

	// 표시: Auto corr entry dist
	Barcode_SetLL(hWnd, ID_BC_EDIT_CORR_START_BCERR, g_hbc.autoCorrEntryDistPulse);

	const wchar_t* phaseStr =
		(g_hbc.phase == HybridBarcodeState::IDLE) ? L"IDLE" :
		(g_hbc.phase == HybridBarcodeState::COARSE) ? L"COARSE(to -10cnt)" :
		(g_hbc.phase == HybridBarcodeState::COARSE_BRAKE) ? L"COARSE_BRAKE(profile stop)" :
		(g_hbc.phase == HybridBarcodeState::FINE) ? L"FINE(step to ±2cnt)" :
		L"DONE";

	wchar_t st[980];
	swprintf_s(st,
		L"Run=%s Phase=%s Axis=%d Now6063=%d Target=%lld Err(cnt)=%lld RemPulses=%lld  "
		L"CoarseTarget=%lld(preStop=%dcnt)  AutoCorrEntry(pulse)=%lld  "
		L"Arrive=±%dcnt(%d/%d) CoarseArr=±%dcnt(%d/%d) "
		L"Main[V/A/D]=%.0f/%.0f/%.0f  Corr[V/A/D]=%.0f/%.0f/%.0f  aDecEma=%.0fpps2",
		g_hbc.running ? L"Y" : L"N",
		phaseStr,
		g_hbc.axis,
		now6063,
		userTarget,
		bcErr,
		pulses,
		g_hbc.coarseTargetBarcodeAbs,
		g_hbc.preStopCnt,
		g_hbc.autoCorrEntryDistPulse,
		g_hbc.arriveCnt, g_hbc.arriveStableTicks, g_hbc.arriveStableTicksNeed,
		g_hbc.coarseArriveCnt, g_hbc.coarseStableTicks, g_hbc.coarseStableTicksNeed,
		g_hbc.mainVel, g_hbc.mainAcc, g_hbc.mainDec,
		g_hbc.corrVel, g_hbc.corrAcc, g_hbc.corrDec,
		g_hbc.aDecEma
	);
	Barcode_SetTxt(hWnd, ID_BC_TXT_STATUS, st);
}

// =====================================================
// Stop-distance based auto decel (with measured decel assist)
// + 2-stage: COARSE to (target-10cnt) -> ProfileStop to 0 speed -> FINE step to ±2cnt
void HBC_Poll(HWND hWnd)
{
	if (!g_hbc.running) return;

	const double dtSec = 0.03; // 30ms
	const int overshootConfirmTicks = 3;

	int ax = g_hbc.axis;

	int now6063 = 0;
	if (!Bc_ReadSelectedAxis_6063(now6063)) return;

	// phase에 따라 활성 목표(6063 abs)
	long long activeTargetAbs =
		(g_hbc.phase == HybridBarcodeState::COARSE || g_hbc.phase == HybridBarcodeState::COARSE_BRAKE)
		? g_hbc.coarseTargetBarcodeAbs
		: g_hbc.targetBarcodeAbs;

	long long bcErr = activeTargetAbs - (long long)now6063;
	long long bcErrAbs = llabs(bcErr);

	// ===== 공통: 실제 속도 읽기(감속 EMA) =====
	g_cm.GetStatus(&g_status);
	double motorRpm = (double)g_status.axesStatus[ax].actualVelocity;
	double vCurPps = HBC_RpmToPps(motorRpm, g_hbc.motorCpr);

	// measured decel update (EMA)
	if (!g_hbc.havePrevV) {
		g_hbc.havePrevV = true;
		g_hbc.vPrevPps = vCurPps;
	}
	else {
		double dv = g_hbc.vPrevPps - vCurPps; // + when decelerating
		double aInst = dv / dtSec;            // pps^2
		g_hbc.vPrevPps = vCurPps;

		if (aInst > 50.0) {
			const double alpha = 0.15;
			if (g_hbc.aDecEma <= 0.0) g_hbc.aDecEma = aInst;
			else g_hbc.aDecEma = (1.0 - alpha) * g_hbc.aDecEma + alpha * aInst;
		}
	}

	// ===== Overshoot detect =====
	int sgn = HBC_signll(bcErr);

	if (g_hbc.forbidReverse)
	{
		if (g_hbc.lastErrSign != 0 && sgn != 0 && sgn != g_hbc.lastErrSign)
			g_hbc.signFlipTicks++;
		else
			g_hbc.signFlipTicks = 0;

		if (g_hbc.signFlipTicks >= overshootConfirmTicks)
		{
			g_hbc.running = false;
			g_hbc.phase = HybridBarcodeState::IDLE;
			g_hbc.stopCooldownUntil = GetTickCount64() + 300;
			StopAxis(ax);
			MessageBox(hWnd,
				TEXT("Overshoot detected (target crossed). Motion stopped.\n")
				TEXT("변환계수(bcMmPerCnt/gear) 또는 감속(ms) 튜닝을 확인하세요."),
				TEXT("Hybrid Barcode"), MB_ICONWARNING);
			return;
		}
	}
	else
	{
		g_hbc.signFlipTicks = 0;
	}
	if (sgn != 0) g_hbc.lastErrSign = sgn;

	HbcSnapshot snap{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
	long long pulsesPerCnt = llabs(HBC_bcToPulses(snap, 1));

	// ==========================================================
	// PHASE: COARSE_BRAKE (StopAxis 없이 프로파일 감속으로 0속도 만들기)
	// ==========================================================
	if (g_hbc.phase == HybridBarcodeState::COARSE_BRAKE)
	{
		// 0속도 근처 도달하면 FINE로 전환
		if (vCurPps <= g_hbc.coarseStopVelThreshPps)
		{
			// final 목표로 FINE 접근
			g_hbc.phase = HybridBarcodeState::FINE;
			g_hbc.arriveStableTicks = 0;

			long long finalErr = g_hbc.targetBarcodeAbs - (long long)now6063;
			g_hbc.lastErrSign = HBC_signll(finalErr);
			g_hbc.signFlipTicks = 0;

			// 전환 직후 명령 충돌 방지
			g_hbc.fineStartCooldownUntil = GetTickCount64() + 120;

			// 커맨드 갱신 유도
			g_hbc.lastCmdVel = 0;
			g_hbc.lastCmdTarget = 0;
		}
		return;
	}

	// ==========================================================
	// PHASE: COARSE (목표-10cnt로 접근하다가, 근처에서 ProfileStop 발동)
	// ==========================================================
	if (g_hbc.phase == HybridBarcodeState::COARSE)
	{
		// 도착 판정(±1cnt 안정화) -> StopAxis 대신 ProfileStop -> COARSE_BRAKE
		if (bcErrAbs <= g_hbc.coarseArriveCnt)
		{
			if (++g_hbc.coarseStableTicks >= g_hbc.coarseStableTicksNeed)
			{
				// ✅ StopAxis(ax) 대신 프로파일 감속 정지
				HBC_ProfileStop(ax, g_hbc.coarseBrakeVelPps, g_hbc.coarseBrakeAccMs, g_hbc.coarseBrakeDecMs);

				g_hbc.stopCooldownUntil = GetTickCount64() + 120;
				g_hbc.fineStartCooldownUntil = g_hbc.stopCooldownUntil;

				g_hbc.coarseStableTicks = 0;
				g_hbc.phase = HybridBarcodeState::COARSE_BRAKE;
				return;
			}
			return;
		}
		g_hbc.coarseStableTicks = 0;

		// COARSE: stop-distance 기반 감속(목표는 coarseTarget)
		long long remainingPulses = HBC_bcToPulses(snap, bcErr);
		long long distAbs = llabs(remainingPulses);

		auto safe_sqrt_scale = [](double base, double refVel, double vel, double lo, double hi) {
			double v = std::max(1.0, vel);
			double scale = std::sqrt(std::max(0.2, refVel / v));
			return std::clamp(base * scale, lo, hi);
			};

		double mainDecSafety = safe_sqrt_scale(0.28, 5000.0, g_hbc.mainVel, 0.15, 0.35);
		double aMainModel = HBC_DecelPps2_Model(g_hbc.mainVel, g_hbc.mainDec, mainDecSafety);

		double aMeas = (g_hbc.aDecEma > 0.0) ? g_hbc.aDecEma : 0.0;
		double aMainDec = (aMeas > 0.0) ? std::min(aMeas, aMainModel * 1.2) : aMainModel;

		double stopDistNow = HBC_StopDistPulses(vCurPps, aMainDec);

		long long marginPulses =
			(long long)(pulsesPerCnt * 6) +        // coarse는 6cnt
			(long long)(vCurPps * 0.15) +          // 150ms 선행
			(long long)(stopDistNow * 0.10) +      // 10% 버퍼
			300;

		if (marginPulses < pulsesPerCnt * 6) marginPulses = pulsesPerCnt * 6;

		double distForPlan = (double)distAbs - (double)marginPulses;
		if (distForPlan < 0) distForPlan = 0;

		double vEnvMain = std::min(g_hbc.mainVel, HBC_VelLimitFromDist(distForPlan, aMainDec));

		// COARSE도 너무 가까우면 천천히
		if (bcErrAbs <= (g_hbc.preStopCnt + 10))
			vEnvMain = std::min(vEnvMain, std::max(200.0, g_hbc.creepVel));

		if (vEnvMain < 50.0 && distAbs > 0) vEnvMain = 50.0;

		long long curPos = g_status.axesStatus[ax].actualPos;
		long long absTarget = curPos + remainingPulses;

		DWORD period = (bcErrAbs <= 80) ? 40 : 90;
		double ratio = (bcErrAbs <= 80) ? 0.05 : 0.12;

		HBC_SendMoveThrottled(ax, absTarget, vEnvMain, g_hbc.mainAcc, g_hbc.mainDec, ratio,
			/*minTargetDeltaPulses*/ pulsesPerCnt, period);

		return;
	}

	// ==========================================================
	// PHASE: FINE (final target로 step 이동 → ±2cnt)
	// ==========================================================
	if (g_hbc.phase == HybridBarcodeState::FINE)
	{
		long long finalErr = g_hbc.targetBarcodeAbs - (long long)now6063;
		long long finalErrAbs = llabs(finalErr);

		if (finalErrAbs <= g_hbc.arriveCnt)
		{
			if (++g_hbc.arriveStableTicks >= g_hbc.arriveStableTicksNeed)
			{
				g_hbc.running = false;
				g_hbc.phase = HybridBarcodeState::DONE;
				g_hbc.stopCooldownUntil = GetTickCount64() + 300;
				StopAxis(ax); // final에서는 이미 저속이라 충격 작음(원하면 ProfileStop으로 바꿔도 됨)
				return;
			}
			return;
		}
		g_hbc.arriveStableTicks = 0;

		long long remainingPulses = HBC_bcToPulses(snap, finalErr);

		long long stepMaxPulses = llabs(HBC_bcToPulses(snap, (long long)g_hbc.fineStepCntMax));
		if (stepMaxPulses < 1) stepMaxPulses = 1;

		long long stepPulses = remainingPulses;
		if (stepPulses > stepMaxPulses) stepPulses = stepMaxPulses;
		if (stepPulses < -stepMaxPulses) stepPulses = -stepMaxPulses;

		if (finalErrAbs <= 10) {
			long long oneCntPulse = llabs(HBC_bcToPulses(snap, 1));
			stepPulses = std::clamp(stepPulses, -std::max(1LL, oneCntPulse), std::max(1LL, oneCntPulse));
		}

		double vPlan = std::min(g_hbc.corrVel, g_hbc.creepVel);
		if (finalErrAbs <= g_hbc.creepCnt)
			vPlan = std::min(vPlan, g_hbc.creepVel);

		if (finalErrAbs <= 6) vPlan = std::min(vPlan, 200.0);
		if (vPlan < 20.0) vPlan = 20.0;

		long long curPos = g_status.axesStatus[ax].actualPos;
		long long absTarget = curPos + stepPulses;

		HBC_SendMoveThrottled(ax, absTarget, vPlan, g_hbc.corrAcc, g_hbc.corrDec,
			0.01, 1, 80);

		return;
	}

	// DONE/IDLE
	return;
}

// =====================================================
// UI and Window Proc
// =====================================================
LRESULT CALLBACK BarcodeWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_CREATE:
	{
		// GROUP
		CreateWindow(TEXT("BUTTON"), TEXT("Hybrid Barcode Demo"),
			WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
			10, 10, 1100, 360, hWnd, 0, 0, 0);

		// AXIS
		CreateWindow(TEXT("STATIC"), TEXT("Axis"), WS_CHILD | WS_VISIBLE,
			20, 40, 40, 22, hWnd, 0, 0, 0);

		HWND hAxis = CreateWindow(TEXT("COMBOBOX"), TEXT(""),
			WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
			65, 36, 80, 200, hWnd, (HMENU)ID_BC_COMBO_AXIS, 0, 0);

		for (int i = 0; i <= 8; ++i) {
			wchar_t tmp[8]; swprintf_s(tmp, L"%d", i);
			SendMessage(hAxis, CB_ADDSTRING, 0, (LPARAM)tmp);
		}
		SendMessage(hAxis, CB_SETCURSEL, g_hbc.axis, 0);

		// NOW(6063)
		CreateWindow(TEXT("STATIC"), TEXT("Now(6063)"),
			WS_CHILD | WS_VISIBLE,
			20, 80, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("0"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			125, 78, 120, 24, hWnd, (HMENU)ID_BC_ECAT_6063_NOW, 0, 0);

		// TARGET BARCODE ABS
		CreateWindow(TEXT("STATIC"), TEXT("Target Barcode(abs)"),
			WS_CHILD | WS_VISIBLE,
			20, 120, 150, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("0"),
			WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
			175, 118, 120, 24, hWnd, (HMENU)ID_BC_EDIT_TARGET, 0, 0);

		// MAIN MOVE PROFILE
		CreateWindow(TEXT("STATIC"), TEXT("Main Vel[pps]"),
			WS_CHILD | WS_VISIBLE,
			310, 120, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("10000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			400, 118, 90, 24, hWnd, (HMENU)ID_BC_EDIT_VEL, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Main Acc[ms]"),
			WS_CHILD | WS_VISIBLE,
			500, 120, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			590, 118, 90, 24, hWnd, (HMENU)ID_BC_EDIT_ACC, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Main Dec[ms]"),
			WS_CHILD | WS_VISIBLE,
			690, 120, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			780, 118, 90, 24, hWnd, (HMENU)ID_BC_EDIT_DEC, 0, 0);

		// CORRECTION PROFILE
		CreateWindow(TEXT("STATIC"), TEXT("Corr Vel[pps]"),
			WS_CHILD | WS_VISIBLE,
			310, 155, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			400, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_VEL, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Corr Acc[ms]"),
			WS_CHILD | WS_VISIBLE,
			500, 155, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("300"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			590, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_ACC, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Corr Dec[ms]"),
			WS_CHILD | WS_VISIBLE,
			690, 155, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("300"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			780, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_DEC, 0, 0);

		// CONVERSION PARAMS
		CreateWindow(TEXT("STATIC"), TEXT("Gear"),
			WS_CHILD | WS_VISIBLE,
			20, 155, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("4.4248"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			125, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_GEAR, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("WheelDia[mm]"),
			WS_CHILD | WS_VISIBLE,
			20, 190, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("115"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			125, 188, 90, 24, hWnd, (HMENU)ID_BC_EDIT_WHEEL_D, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Motor CPR"),
			WS_CHILD | WS_VISIBLE,
			230, 155, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("10000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			330, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_MOT_CPR, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Barcode mm/cnt"),
			WS_CHILD | WS_VISIBLE,
			230, 190, 110, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1.07"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			330, 188, 90, 24, hWnd, (HMENU)ID_BC_EDIT_BC_MM_PER_CNT, 0, 0);

		// COMPUTED PULSES (remaining)
		CreateWindow(TEXT("STATIC"), TEXT("Remaining Pulses"),
			WS_CHILD | WS_VISIBLE,
			20, 225, 140, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("0"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			165, 223, 150, 24, hWnd, (HMENU)ID_BC_TXT_COMPUTED_PULSES, 0, 0);

		// ARRIVE RANGE (표시용)
		CreateWindow(TEXT("STATIC"), TEXT("Arrive band(±cnt)"),
			WS_CHILD | WS_VISIBLE,
			330, 225, 120, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("2"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			455, 223, 80, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_DEADBAND, 0, 0);

		// Auto corr entry dist display
		CreateWindow(TEXT("STATIC"), TEXT("Auto Corr EntryDist(pulse)"),
			WS_CHILD | WS_VISIBLE,
			550, 225, 200, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("-"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			760, 223, 160, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_START_BCERR, 0, 0);

		// BUTTONS
		CreateWindow(TEXT("BUTTON"), TEXT("Servo ON"),
			WS_CHILD | WS_VISIBLE,
			930, 223, 80, 26, hWnd, (HMENU)ID_BC_BTN_SERVO_ON, 0, 0);

		CreateWindow(TEXT("BUTTON"), TEXT("Servo OFF"),
			WS_CHILD | WS_VISIBLE,
			1015, 223, 80, 26, hWnd, (HMENU)ID_BC_BTN_SERVO_OFF, 0, 0);

		CreateWindow(TEXT("BUTTON"), TEXT("Start"),
			WS_CHILD | WS_VISIBLE,
			930, 255, 80, 26, hWnd, (HMENU)ID_BC_BTN_START, 0, 0);

		CreateWindow(TEXT("BUTTON"), TEXT("Stop"),
			WS_CHILD | WS_VISIBLE,
			1015, 255, 80, 26, hWnd, (HMENU)ID_BC_BTN_STOP, 0, 0);

		// STATUS
		CreateWindow(TEXT("STATIC"), TEXT("Status"),
			WS_CHILD | WS_VISIBLE,
			20, 290, 70, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("-"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			90, 288, 1000, 50, hWnd, (HMENU)ID_BC_TXT_STATUS, 0, 0);

		// TIMER
		SetTimer(hWnd, ID_BC_TIMER, 30, nullptr);
		return 0;
	}

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);

		if (id == ID_BC_COMBO_AXIS && HIWORD(wParam) == CBN_SELCHANGE)
		{
			g_hbc.axis = (int)SendMessage((HWND)lParam, CB_GETCURSEL, 0, 0);
			return 0;
		}

		if (id == ID_BC_BTN_SERVO_ON)
		{
			EnsureServoOn(g_hbc.axis);
			EnsurePosModeNoStop(g_hbc.axis);
			return 0;
		}

		if (id == ID_BC_BTN_SERVO_OFF)
		{
			g_cm.axisControl->SetServoOn(g_hbc.axis, 0);
			return 0;
		}

		if (id == ID_BC_BTN_START)
		{
			HBC_Start(hWnd);
			return 0;
		}

		if (id == ID_BC_BTN_STOP)
		{
			g_hbc.running = false;
			g_hbc.phase = HybridBarcodeState::IDLE;
			g_hbc.arriveStableTicks = 0;
			g_hbc.coarseStableTicks = 0;
			g_hbc.signFlipTicks = 0;
			g_hbc.stopCooldownUntil = GetTickCount64() + 300;
			g_hbc.fineStartCooldownUntil = g_hbc.stopCooldownUntil;
			StopAxis(g_hbc.axis);
			return 0;
		}
		return 0;
	}

	case WM_TIMER:
		if (wParam == ID_BC_TIMER)
		{
			HBC_UpdateUi(hWnd);
			if (g_hbc.running) HBC_Poll(hWnd);
			return 0;
		}
		break;

	case WM_CLOSE:
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		if (g_hBarcodeWnd == hWnd) g_hBarcodeWnd = nullptr;
		return 0;
	}

	return DefWindowProc(hWnd, msg, wParam, lParam);
}

// =====================================================
// Show Window
// =====================================================
void ShowBarcodeDemoWindow(HWND parent)
{
	if (!g_hBarcodeWnd || !IsWindow(g_hBarcodeWnd))
	{
		WNDCLASS wc{};
		wc.lpszClassName = TEXT("HybridBarcodeDemoWnd");
		wc.lpfnWndProc = BarcodeWndProc;
		wc.hInstance = (HINSTANCE)GetWindowLongPtr(parent, GWLP_HINSTANCE);
		wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);

		RegisterClass(&wc);

		g_hBarcodeWnd = CreateWindow(
			TEXT("HybridBarcodeDemoWnd"),
			TEXT("Hybrid Barcode Demo"),
			WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX,
			CW_USEDEFAULT, CW_USEDEFAULT,
			1160, 440,
			parent,
			nullptr,
			wc.hInstance,
			nullptr
		);

		ShowWindow(g_hBarcodeWnd, SW_SHOWNORMAL);
		UpdateWindow(g_hBarcodeWnd);
	}
	else
	{
		ShowWindow(g_hBarcodeWnd, SW_SHOWNORMAL);
		SetForegroundWindow(g_hBarcodeWnd);
	}
}






// ------------------ 메인 윈도우 ------------------
#define ID_BTN_DEMO_MAIN 9602
#define ID_BTN_GPIO_MAIN 9603

// ------------------ 메인 윈도우 ------------------
// 기존 코드의 큰 동작은 유지하되, 메인 UI를 재배치한다.

// 추가: 공용 작은 글꼴 핸들
HFONT g_hSmallFont = nullptr;
HFONT GetSmallFont()
{
	if (!g_hSmallFont) {
		LOGFONT lf{}; SystemParametersInfo(SPI_GETICONTITLELOGFONT, sizeof(lf), &lf, 0);
		lf.lfHeight = -12; // 작은 글씨
		wcscpy_s(lf.lfFaceName, L"Segoe UI");
		g_hSmallFont = CreateFontIndirect(&lf);
	}
	return g_hSmallFont;
}

// 새 컨트롤 ID
#define ID_BTN_MAP_WINDOW 9500

// Map 창 핸들
HWND g_hMapWnd = nullptr;

HWND ShowMapWindow(HWND hParent)
{
	if (!g_hMapWnd || !IsWindow(g_hMapWnd)) {
#ifdef MAPVIEW_CREATE_DECLARED
		g_hMapWnd = CreateMapWindow(hParent, &g_wmx, &g_cm);
#else
		g_hMapWnd = nullptr;
#endif
		if (g_hMapWnd) {
			ShowWindow(g_hMapWnd, SW_SHOWNORMAL);
			UpdateWindow(g_hMapWnd);
			SetForegroundWindow(g_hMapWnd);
		}
		else {
			MessageBox(hParent, TEXT("Map window creation failed."), TEXT("Map"), MB_ICONERROR);
		}
	}
	else {
		ShowWindow(g_hMapWnd, SW_SHOWNORMAL);
		SetForegroundWindow(g_hMapWnd);
	}
	return g_hMapWnd;
}

// ------------------ 메인 UI 재구성 ------------------

// 선택 축 제어용 ID (기존 단일 → 4그룹 확장)
enum : int {
	// Group A
	ID_SEL_A_AXIS_COMBO = 10001,
	ID_SEL_A_ABS_POS = 10002,
	ID_SEL_A_REL_STEP = 10003,
	ID_SEL_A_VEL = 10004,
	ID_SEL_A_ACC = 10005,
	ID_SEL_A_DEC = 10006,
	ID_SEL_A_BTN_SVON = 10007,
	ID_SEL_A_BTN_SVOFF = 10008,
	ID_SEL_A_BTN_HOME = 10009,
	ID_SEL_A_BTN_ABS = 10010,
	ID_SEL_A_BTN_REL = 10011,
	ID_SEL_A_BTN_STOP = 10012,
	ID_SEL_A_BTN_JOGP = 10013,
	ID_SEL_A_BTN_JOGM = 10014,
	ID_SEL_A_BTN_APPLY_ALT = 10015,
	ID_SEL_A_ALT_TARGET = 10016,

	// Group B
	ID_SEL_B_AXIS_COMBO = 10101,
	ID_SEL_B_ABS_POS = 10102,
	ID_SEL_B_REL_STEP = 10103,
	ID_SEL_B_VEL = 10104,
	ID_SEL_B_ACC = 10105,
	ID_SEL_B_DEC = 10106,
	ID_SEL_B_BTN_SVON = 10107,
	ID_SEL_B_BTN_SVOFF = 10108,
	ID_SEL_B_BTN_HOME = 10109,
	ID_SEL_B_BTN_ABS = 10110,
	ID_SEL_B_BTN_REL = 10111,
	ID_SEL_B_BTN_STOP = 10112,
	ID_SEL_B_BTN_JOGP = 10113,
	ID_SEL_B_BTN_JOGM = 10114,
	ID_SEL_B_BTN_APPLY_ALT = 10115,
	ID_SEL_B_ALT_TARGET = 10116,

	// Group C
	ID_SEL_C_AXIS_COMBO = 10201,
	ID_SEL_C_ABS_POS = 10202,
	ID_SEL_C_REL_STEP = 10203,
	ID_SEL_C_VEL = 10204,
	ID_SEL_C_ACC = 10205,
	ID_SEL_C_DEC = 10206,
	ID_SEL_C_BTN_SVON = 10207,
	ID_SEL_C_BTN_SVOFF = 10208,
	ID_SEL_C_BTN_HOME = 10209,
	ID_SEL_C_BTN_ABS = 10210,
	ID_SEL_C_BTN_REL = 10211,
	ID_SEL_C_BTN_STOP = 10212,
	ID_SEL_C_BTN_JOGP = 10213,
	ID_SEL_C_BTN_JOGM = 10214,
	ID_SEL_C_BTN_APPLY_ALT = 10215,
	ID_SEL_C_ALT_TARGET = 10216,

	// Group D
	ID_SEL_D_AXIS_COMBO = 10301,
	ID_SEL_D_ABS_POS = 10302,
	ID_SEL_D_REL_STEP = 10303,
	ID_SEL_D_VEL = 10304,
	ID_SEL_D_ACC = 10305,
	ID_SEL_D_DEC = 10306,
	ID_SEL_D_BTN_SVON = 10307,
	ID_SEL_D_BTN_SVOFF = 10308,
	ID_SEL_D_BTN_HOME = 10309,
	ID_SEL_D_BTN_ABS = 10310,
	ID_SEL_D_BTN_REL = 10311,
	ID_SEL_D_BTN_STOP = 10312,
	ID_SEL_D_BTN_JOGP = 10313,
	ID_SEL_D_BTN_JOGM = 10314,
	ID_SEL_D_BTN_APPLY_ALT = 10315,
	ID_SEL_D_ALT_TARGET = 10316,
};

// Checked Axis Control: 기존 파라미터창 제거하고 버튼만 유지
#define ID_MULTI_BTN_ABS     11100
#define ID_MULTI_BTN_REL     11101
#define ID_MULTI_BTN_JOGP    11102
#define ID_MULTI_BTN_JOGM    11103
#define ID_MULTI_BTN_HOME    11104
#define ID_MULTI_BTN_SVON    11105
#define ID_MULTI_BTN_SVOFF   11106
#define ID_MULTI_BTN_STOP    11107
#define ID_MULTI_BTN_ALARMRST 11108

#define ID_MULTI_BTN_ALLCHECK 11109
#define ID_MULTI_BTN_ALLCLEAR 11110
// --------- Selected x4 + Per-axis parameter storage ---------
struct AxisParam {
	double absPos = 0.0;
	double relStep = 0.0;
	double vel = 10000.0;
	double acc = 100.0;
	double dec = 100.0;
	double altTarget = 0.0;
};
// 축별 파라미터(축을 바꿔도 저장/복원)
static AxisParam g_axisParam[kNumAxes - 3];

// Selected 그룹(A/B/C/D)이 제어할 축 (중복 불가)
static int g_selGroupAxis[4] = { 0, 1, 2, 3 };

static int GetGroupAxisByIndex(int idx) {
	if (idx < 0 || idx >= 4) return 0;
	int ax = g_selGroupAxis[idx];
	if (ax < 0 || ax >= kNumAxes - 3) ax = 0;
	return ax;
}
static void SetGroupAxisByIndex(int idx, int axis) {
	if (idx < 0 || idx >= 4) return;
	if (axis < 0 || axis >= kNumAxes - 3) axis = 0;
	g_selGroupAxis[idx] = axis;
}

// 그룹 인덱스 ↔ 컨트롤 ID 묶음 액세스 헬퍼
struct SelIds {
	int cmbAxis, eAbs, eRel, eVel, eAcc, eDec, eAlt;
	int bApplyAlt;
	int bSvOn, bSvOff, bHome;
	int bAbs, bRel;
	int bJogP, bJogM;
	int bStop;
};
static SelIds GetSelIds(int groupIdx) {
	switch (groupIdx) {
	case 0: return { ID_SEL_A_AXIS_COMBO, ID_SEL_A_ABS_POS, ID_SEL_A_REL_STEP, ID_SEL_A_VEL, ID_SEL_A_ACC, ID_SEL_A_DEC, ID_SEL_A_ALT_TARGET,
		ID_SEL_A_BTN_APPLY_ALT,
		ID_SEL_A_BTN_SVON, ID_SEL_A_BTN_SVOFF, ID_SEL_A_BTN_HOME,
		ID_SEL_A_BTN_ABS, ID_SEL_A_BTN_REL,
		ID_SEL_A_BTN_JOGP, ID_SEL_A_BTN_JOGM,
		ID_SEL_A_BTN_STOP };
	case 1: return { ID_SEL_B_AXIS_COMBO, ID_SEL_B_ABS_POS, ID_SEL_B_REL_STEP, ID_SEL_B_VEL, ID_SEL_B_ACC, ID_SEL_B_DEC, ID_SEL_B_ALT_TARGET,
		ID_SEL_B_BTN_APPLY_ALT,
		ID_SEL_B_BTN_SVON, ID_SEL_B_BTN_SVOFF, ID_SEL_B_BTN_HOME,
		ID_SEL_B_BTN_ABS, ID_SEL_B_BTN_REL,
		ID_SEL_B_BTN_JOGP, ID_SEL_B_BTN_JOGM,
		ID_SEL_B_BTN_STOP };
	case 2: return { ID_SEL_C_AXIS_COMBO, ID_SEL_C_ABS_POS, ID_SEL_C_REL_STEP, ID_SEL_C_VEL, ID_SEL_C_ACC, ID_SEL_C_DEC, ID_SEL_C_ALT_TARGET,
		ID_SEL_C_BTN_APPLY_ALT,
		ID_SEL_C_BTN_SVON, ID_SEL_C_BTN_SVOFF, ID_SEL_C_BTN_HOME,
		ID_SEL_C_BTN_ABS, ID_SEL_C_BTN_REL,
		ID_SEL_C_BTN_JOGP, ID_SEL_C_BTN_JOGM,
		ID_SEL_C_BTN_STOP };
	default: return { ID_SEL_D_AXIS_COMBO, ID_SEL_D_ABS_POS, ID_SEL_D_REL_STEP, ID_SEL_D_VEL, ID_SEL_D_ACC, ID_SEL_D_DEC, ID_SEL_D_ALT_TARGET,
		ID_SEL_D_BTN_APPLY_ALT,
		ID_SEL_D_BTN_SVON, ID_SEL_D_BTN_SVOFF, ID_SEL_D_BTN_HOME,
		ID_SEL_D_BTN_ABS, ID_SEL_D_BTN_REL,
		ID_SEL_D_BTN_JOGP, ID_SEL_D_BTN_JOGM,
		ID_SEL_D_BTN_STOP };
	}
}

static AxisParam& GetAxisParam(int axis) {
	if (axis < 0 || axis >= kNumAxes - 3) axis = 0;
	return g_axisParam[axis];
}

// Selected Axis Control: 어떤 Edit 컨트롤이 어느 그룹(A/B/C/D)에 속하는지 찾는다.
// (Abs/Rel/Vel/Acc/Dec/Alt Target 변경 시 "즉시 저장"을 위해 사용)
static bool SelEditIdToGroupIdx(int id, int& outGroupIdx) {
	for (int g = 0; g < 4; ++g) {
		SelIds ids = GetSelIds(g);
		if (id == ids.eAbs || id == ids.eRel || id == ids.eVel || id == ids.eAcc || id == ids.eDec || id == ids.eAlt) {
			outGroupIdx = g;
			return true;
		}
	}
	return false;
}

// 콤보 박스는 항목이 필터링되므로(CB_GETCURSEL==축번호가 아님), itemData로 축번호를 꺼낸다.
static int Combo_GetAxis(HWND hCombo) {
	int sel = (int)SendMessage(hCombo, CB_GETCURSEL, 0, 0);
	if (sel < 0) return 0;
	LRESULT data = SendMessage(hCombo, CB_GETITEMDATA, sel, 0);
	int axis = (data == CB_ERR) ? sel : (int)data;
	if (axis < 0 || axis >= kNumAxes - 3) axis = 0;
	return axis;
}
static void Combo_SetAxis(HWND hCombo, int axis) {
	if (axis < 0 || axis >= kNumAxes - 3) axis = 0;
	int cnt = (int)SendMessage(hCombo, CB_GETCOUNT, 0, 0);
	for (int i = 0; i < cnt; ++i) {
		LRESULT data = SendMessage(hCombo, CB_GETITEMDATA, i, 0);
		if ((int)data == axis) { SendMessage(hCombo, CB_SETCURSEL, i, 0); return; }
	}
	if (cnt > 0) SendMessage(hCombo, CB_SETCURSEL, 0, 0);
}

// 그룹 UI 입력값을 특정 축(axisToSave)의 파라미터로 저장
static void SaveAxisParamFromUiForAxis(HWND hWnd, int groupIdx, int axisToSave) {
	SelIds ids = GetSelIds(groupIdx);
	AxisParam& ap = GetAxisParam(axisToSave);

	ap.absPos = GetDlgDoubleOrDefIfInvalid(hWnd, ids.eAbs, ap.absPos);
	ap.relStep = GetDlgDoubleOrDefIfInvalid(hWnd, ids.eRel, ap.relStep);
	ap.vel = GetDlgDoubleOrDefIfInvalid(hWnd, ids.eVel, ap.vel);
	ap.acc = GetDlgDoubleOrDefIfInvalid(hWnd, ids.eAcc, ap.acc);
	ap.dec = GetDlgDoubleOrDefIfInvalid(hWnd, ids.eDec, ap.dec);
	ap.altTarget = GetDlgDoubleOrDefIfInvalid(hWnd, ids.eAlt, ap.altTarget);
}

// 그룹 UI 입력값을 현재 선택된 축의 파라미터로 저장
static void SaveAxisParamFromUi(HWND hWnd, int groupIdx) {
	SelIds ids = GetSelIds(groupIdx);
	HWND hCombo = GetDlgItem(hWnd, ids.cmbAxis);
	int axis = Combo_GetAxis(hCombo);
	SetGroupAxisByIndex(groupIdx, axis);
	SaveAxisParamFromUiForAxis(hWnd, groupIdx, axis);
}

// 현재 그룹이 선택한 축 파라미터를 그룹 UI에 로드
static void LoadAxisParamToUi(HWND hWnd, int groupIdx) {
	SelUiUpdateGuard __guard;
	SelIds ids = GetSelIds(groupIdx);
	HWND hCombo = GetDlgItem(hWnd, ids.cmbAxis);
	int axis = GetGroupAxisByIndex(groupIdx);
	Combo_SetAxis(hCombo, axis);

	AxisParam& ap = GetAxisParam(axis);
	SetDlgDouble(hWnd, ids.eAbs, ap.absPos);
	SetDlgDouble(hWnd, ids.eRel, ap.relStep);
	SetDlgDouble(hWnd, ids.eVel, ap.vel);
	SetDlgDouble(hWnd, ids.eAcc, ap.acc);
	SetDlgDouble(hWnd, ids.eDec, ap.dec);
	SetDlgDouble(hWnd, ids.eAlt, ap.altTarget);
}

// 4개 그룹 콤보 항목을 "중복 불가" 규칙으로 다시 구성
static void RefreshSelAxisCombos(HWND hWnd) {
	SelUiUpdateGuard __guard;
	// 중복/범위 오류 자동 복구
	bool used[kNumAxes - 3] = { false };
	for (int g = 0; g < 4; ++g) {
		int ax = GetGroupAxisByIndex(g);
		if (ax < 0 || ax >= kNumAxes - 3 || used[ax]) {
			for (int a = 0; a < kNumAxes - 3; ++a) if (!used[a]) { ax = a; break; }
			SetGroupAxisByIndex(g, ax);
		}
		used[ax] = true;
	}

	int selAx[4] = { GetGroupAxisByIndex(0), GetGroupAxisByIndex(1), GetGroupAxisByIndex(2), GetGroupAxisByIndex(3) };

	for (int g = 0; g < 4; ++g) {
		SelIds ids = GetSelIds(g);
		HWND hCombo = GetDlgItem(hWnd, ids.cmbAxis);
		if (!hCombo) continue;

		int curAxis = selAx[g];
		SendMessage(hCombo, CB_RESETCONTENT, 0, 0);

		for (int a = 0; a < kNumAxes - 3; ++a) {
			bool taken = false;
			for (int og = 0; og < 4; ++og) {
				if (og == g) continue;
				if (selAx[og] == a) { taken = true; break; }
			}
			if (taken && a != curAxis) continue;

			TCHAR t[8]; _stprintf_s(t, TEXT("%d"), a);
			int idx = (int)SendMessage(hCombo, CB_ADDSTRING, 0, (LPARAM)t);
			SendMessage(hCombo, CB_SETITEMDATA, idx, (LPARAM)a);
		}
		Combo_SetAxis(hCombo, curAxis);
	}
}

// 그룹 콤보 변경 처리: 이전 축 값 저장 → 새 축 값 로드 → 콤보 목록 갱신
static void OnSelGroupAxisChanged(HWND hWnd, int groupIdx) {
	SelIds ids = GetSelIds(groupIdx);
	HWND hCombo = GetDlgItem(hWnd, ids.cmbAxis);
	if (!hCombo) return;

	int oldAxis = GetGroupAxisByIndex(groupIdx);
	int newAxis = Combo_GetAxis(hCombo);

	// 현재 UI 입력은 oldAxis에 대한 것이므로 먼저 oldAxis에 저장
	SaveAxisParamFromUiForAxis(hWnd, groupIdx, oldAxis);

	// 그룹 축 변경
	SetGroupAxisByIndex(groupIdx, newAxis);

	// 중복 방지 콤보 리프레시(다른 그룹에서도 새 축이 사라지도록)
	RefreshSelAxisCombos(hWnd);

	// 변경된 축의 파라미터를 UI에 반영
	LoadAxisParamToUi(hWnd, groupIdx);
}


// Selected 그룹에서 조작 수행 헬퍼
static void SelectedGroup_ServoOn(HWND hWnd, int groupIdx) {
	if (!IsManualAllowed(hWnd)) return;
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	if (axis < 0) axis = 0;
	EnsureServoOn(axis);
	EnsurePosModeNoStop(axis);
	// 저장
	SaveAxisParamFromUi(hWnd, groupIdx);
}

static void SelectedGroup_ServoOff(HWND hWnd, int groupIdx) {
	if (!IsManualAllowed(hWnd)) return;
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	if (axis < 0) axis = 0;
	g_cm.axisControl->SetServoOn(axis, 0);
	// 저장
	SaveAxisParamFromUi(hWnd, groupIdx);
}

static void SelectedGroup_Home(HWND hWnd, int groupIdx) {
	if (!IsManualAllowed(hWnd)) return;
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Home"), MB_ICONWARNING); return; }
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	if (axis < 0) axis = 0;
	if (!EnsureServoOn(axis)) return;
	g_cm.motion->Stop(axis); g_cm.velocity->Stop(axis); if (g_cm.torque) g_cm.torque->StopTrq(axis);
	if (EnterPosMode(axis)) { long e = g_home.StartHome(axis); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Home 시작 실패"), e, g_wmx); }
	SaveAxisParamFromUi(hWnd, groupIdx);
}

static void SelectedGroup_Abs(HWND hWnd, int groupIdx) {
	if (!IsManualAllowed(hWnd)) return;
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) return;

	double tgt = GetDlgDouble(hWnd, ids.eAbs, 0.0);
	double v = GetDlgDouble(hWnd, ids.eVel, 10000.0);
	double ta = GetDlgDouble(hWnd, ids.eAcc, 100.0);
	double td = GetDlgDouble(hWnd, ids.eDec, 100.0);

	//// 보호 체크
	//if (axis == 2) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[2].actualPos;
	//	long long tgtLL = (long long)std::llround(tgt);
	//	if (Axis2IsMinusCommandBlocked(cur, tgtLL, 0)) { Axis2ShowMinusBlockedWarning(hWnd); return; }
	//}
	//if (axis == 0) {
	//	g_cm.GetStatus(&g_status);
	//	long long cur = (long long)g_status.axesStatus[0].actualPos;
	//	long long tgtLL = (long long)std::llround(tgt);
	//	long long delta = tgtLL - cur;
	//	long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
	//	if (sign != 0 && Axis0IsCommandBlocked(cur, tgtLL, sign)) { Axis0ShowBlockedWarning(hWnd, (int)sign); return; }
	//}
	StartAbsMoveWithProfile(axis, (long long)std::llround(tgt), v, ta, td);
	SaveAxisParamFromUi(hWnd, groupIdx);
}

static void SelectedGroup_Rel(HWND hWnd, int groupIdx) {
	if (!IsManualAllowed(hWnd)) return;
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) return;
	double step = GetDlgDouble(hWnd, ids.eRel, 0.0);
	double v = GetDlgDouble(hWnd, ids.eVel, 10000.0);
	double ta = GetDlgDouble(hWnd, ids.eAcc, 100.0);
	double td = GetDlgDouble(hWnd, ids.eDec, 100.0);
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;
	long long tgt = cur + (long long)std::llround(step);
	//// 보호 체크
	//if (axis == 2) {
	//	int dir = (step >= 0) ? +1 : -1;
	//	if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)dir)) { Axis2ShowMinusBlockedWarning(hWnd); return; }
	//}
	//if (axis == 0) {
	//	int dir = (step >= 0) ? +1 : -1;
	//	if (Axis0IsCommandBlocked(cur, tgt, (long long)dir)) { Axis0ShowBlockedWarning(hWnd, dir); return; }
	//}
	StartAbsMoveWithProfile(axis, tgt, v, ta, td);
	SaveAxisParamFromUi(hWnd, groupIdx);
}

static bool StartJogWithProfileParams(HWND hWnd, int axis, int sign, double vel, double tAccMs, double tDecMs) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return false; }

	//// 보호 체크: Axis2 -방향 차단, Axis0 L/R 차단
	//g_cm.GetStatus(&g_status);
	//if (axis == 2) {
	//	long long curPos = (long long)g_status.axesStatus[2].actualPos;
	//	long long tgt = curPos + (long long)sign * 1000000000LL;
	//	if (Axis2IsMinusCommandBlocked(curPos, tgt, (long long)sign)) { Axis2ShowMinusBlockedWarning(hWnd); return false; }
	//}
	//if (axis == 0) {
	//	long long curPos = (long long)g_status.axesStatus[0].actualPos;
	//	long long tgt = curPos + (long long)sign * 1000000000LL;
	//	if (Axis0IsCommandBlocked(curPos, tgt, (long long)sign)) { Axis0ShowBlockedWarning(hWnd, sign); return false; }
	//}

	if (!EnsureServoOn(axis)) return false;
	if (!EnsurePosModeNoStop(axis)) return false;

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;

	Motion::PosCommand pc;
	pc.axis = axis;
	pc.target = cur + (long long)(sign * 1000000000LL);
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vel);
	pc.profile.acc = TimeMsToAcc(vel, tAccMs);
	pc.profile.dec = TimeMsToAcc(vel, tDecMs);

	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos(JOG) 실패"), e, g_wmx); return false; }

	g_lastCmdVel[axis] = (int)std::lround((double)pc.profile.velocity * sign);
	g_jogActiveAxis = axis;
	g_jogActiveSign = sign;
	SetCapture(hWnd);
	return true;
}

static void SelectedGroup_Jog(HWND hWnd, int groupIdx, int sign) {
	if (!IsManualAllowed(hWnd)) return;
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	if (axis < 0) axis = 0;

	// UI 입력값(특히 vel/acc/dec)을 즉시 반영
	double v = GetDlgDouble(hWnd, ids.eVel, GetAxisParam(axis).vel);
	double ta = GetDlgDouble(hWnd, ids.eAcc, GetAxisParam(axis).acc);
	double td = GetDlgDouble(hWnd, ids.eDec, GetAxisParam(axis).dec);

	SaveAxisParamFromUi(hWnd, groupIdx);
	StartJogWithProfileParams(hWnd, axis, sign, v, ta, td);
}


static void SelectedGroup_Stop(HWND hWnd, int groupIdx) {
	if (!IsManualAllowed(hWnd)) return;
	SelIds ids = GetSelIds(groupIdx);
	int axis = Combo_GetAxis(GetDlgItem(hWnd, ids.cmbAxis));
	StopAxis(axis);
	SaveAxisParamFromUi(hWnd, groupIdx);
}

static void SelectedGroup_ApplyAlt(HWND hWnd, int groupIdx) {
	SelIds ids = GetSelIds(groupIdx);
	int ecat6063 = 0;
	bool ok = ReadAxis0_TxPDO_6063(ecat6063);
	double alt = GetDlgDouble(hWnd, ids.eAlt, 0.0);
	if (!ok) {
		MessageBox(hWnd, TEXT("0x6063 값을 읽을 수 없습니다."), TEXT("Apply Alt→Abs"), MB_ICONWARNING);
		return;
	}
	double tgt = alt - (double)ecat6063;
	SetDlgDouble(hWnd, ids.eAbs, tgt);
	SaveAxisParamFromUi(hWnd, groupIdx);
}

// Checked Axis Control 버튼 동작: 각 축은 자기 파라미터 소스 그룹의 파라미터로 동작
static void Multi_DoAbs(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;

	// 상태 1회 읽어서 (이미 목표 위치인지) 판정에 활용
	g_cm.GetStatus(&g_status);

	struct SlaveInfo { int axis; int gid; int master; };
	std::vector<SlaveInfo> slaveSkipped;
	std::vector<int> alreadyAtTarget;

	for (int a = 0; a < kNumAxes - 3; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;

		int gid = -1, master = -1;
		if (IsAxisSlaveInEnabledSyncGroup(a, &gid, &master)) {
			slaveSkipped.push_back({ a, gid, master });
			continue;
		}

		if (!EnsureServoOn(a) || !EnsurePosModeNoStop(a)) continue;

		AxisParam& ap = GetAxisParam(a);
		long long tgt = (long long)std::llround(ap.absPos);
		long long cur = (long long)g_status.axesStatus[a].actualPos;

		// 목표가 현재와 거의 같으면 "안 움직이는 것"처럼 보일 수 있음 → 안내만 하고 스킵
		if (std::llabs(tgt - cur) <= (long long)inpos_tol_counts) {
			alreadyAtTarget.push_back(a);
			continue;
		}

		StartAbsMoveWithProfile(a, tgt, ap.vel, ap.acc, ap.dec);
	}

	// 사용자 혼동을 줄이기 위한 1회 안내(필요할 때만)
	if (!slaveSkipped.empty() || !alreadyAtTarget.empty()) {
		std::basic_string<TCHAR> msg;

		if (!slaveSkipped.empty()) {
			msg += TEXT("[Sync Group] 아래 축들은 Sync Group이 'Enabled' 상태에서 Slave로 설정되어 있어,\r\n");
			msg += TEXT("Checked Axis의 개별 Absolute/Relative/Jog 명령이 무시되거나 기대와 다르게 동작할 수 있습니다.\r\n");
			msg += TEXT("→ Sync Group을 Disable 하거나, Master 축을 목표로 이동시키세요.\r\n\r\n");
			for (const auto& s : slaveSkipped) {
				TCHAR line[128];
				_stprintf_s(line, TEXT(" - Axis %d (Group %d, Master %d)\r\n"), s.axis, s.gid, s.master);
				msg += line;
			}
			msg += TEXT("\r\n");
		}

		if (!alreadyAtTarget.empty()) {
			msg += TEXT("[이미 목표 위치] 아래 축들은 저장된 Abs 목표가 현재 위치와 거의 같아서(±inpos_tol) 이동이 눈에 안 보입니다:\r\n");
			for (size_t i = 0; i < alreadyAtTarget.size(); ++i) {
				TCHAR t[32];
				_stprintf_s(t, TEXT("%s%d"), (i ? TEXT(", ") : TEXT(" - Axis ")), alreadyAtTarget[i]);
				msg += t;
			}
			msg += TEXT("\r\n");
		}

		MessageBox(hWnd, msg.c_str(), TEXT("Checked Axis - Absolute"), MB_OK | MB_ICONINFORMATION);
	}
}

static void Multi_DoRel(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;

	g_cm.GetStatus(&g_status);

	struct SlaveInfo { int axis; int gid; int master; };
	std::vector<SlaveInfo> slaveSkipped;
	std::vector<int> zeroStep;

	for (int a = 0; a < kNumAxes - 3; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;

		int gid = -1, master = -1;
		if (IsAxisSlaveInEnabledSyncGroup(a, &gid, &master)) {
			slaveSkipped.push_back({ a, gid, master });
			continue;
		}

		if (!EnsureServoOn(a) || !EnsurePosModeNoStop(a)) continue;

		AxisParam& ap = GetAxisParam(a);
		long long step = (long long)std::llround(ap.relStep);
		if (std::llabs(step) <= (long long)inpos_tol_counts) { // step이 0에 가까우면 체감상 "안 움직임"
			zeroStep.push_back(a);
			continue;
		}

		long long cur = (long long)g_status.axesStatus[a].actualPos;
		long long tgt = cur + step;

		StartAbsMoveWithProfile(a, tgt, ap.vel, ap.acc, ap.dec);
	}

	if (!slaveSkipped.empty() || !zeroStep.empty()) {
		std::basic_string<TCHAR> msg;

		if (!slaveSkipped.empty()) {
			msg += TEXT("[Sync Group] 아래 축들은 Sync Group이 'Enabled' 상태에서 Slave로 설정되어 있어,\r\n");
			msg += TEXT("Checked Axis의 개별 Absolute/Relative/Jog 명령이 무시되거나 기대와 다르게 동작할 수 있습니다.\r\n");
			msg += TEXT("→ Sync Group을 Disable 하거나, Master 축을 목표로 이동시키세요.\r\n\r\n");
			for (const auto& s : slaveSkipped) {
				TCHAR line[128];
				_stprintf_s(line, TEXT(" - Axis %d (Group %d, Master %d)\r\n"), s.axis, s.gid, s.master);
				msg += line;
			}
			msg += TEXT("\r\n");
		}

		if (!zeroStep.empty()) {
			msg += TEXT("[Rel Step이 0] 아래 축들은 저장된 Rel Step이 0(또는 매우 작음)이라 이동이 없습니다:\r\n");
			for (size_t i = 0; i < zeroStep.size(); ++i) {
				TCHAR t[32];
				_stprintf_s(t, TEXT("%s%d"), (i ? TEXT(", ") : TEXT(" - Axis ")), zeroStep[i]);
				msg += t;
			}
			msg += TEXT("\r\n");
		}

		MessageBox(hWnd, msg.c_str(), TEXT("Checked Axis - Relative"), MB_OK | MB_ICONINFORMATION);
	}
}

static void Multi_DoJog(HWND hWnd, int sign) {
	if (!IsManualAllowed(hWnd)) return;

	// Hold-to-jog semantics:
	// - Start on BN_PUSHED (mouse/keyboard down)
	// - Stop on BN_UNPUSHED / WM_LBUTTONUP (see WndProc)
	// Reset any previous multi-jog bookkeeping
	g_multiJogActive = false;
	g_multiJogSign = 0;
	for (int a = 0; a < kNumAxes - 3; ++a) g_multiJogAxisActive[a] = false;

	g_cm.GetStatus(&g_status);

	struct SlaveInfo { int axis; int gid; int master; };
	std::vector<SlaveInfo> slaveSkipped;

	bool anyStarted = false;

	for (int a = 0; a < kNumAxes - 3; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;

		int gid = -1, master = -1;
		if (IsAxisSlaveInEnabledSyncGroup(a, &gid, &master)) {
			slaveSkipped.push_back({ a, gid, master });
			continue;
		}

		// 각 축의 파라미터 소스 그룹에서 vel/acc/dec만 사용해서 조그 시작
		AxisParam& ap = GetAxisParam(a);
		double vel = ap.vel;
		double acc = ap.acc;
		double dec = ap.dec;

		if (!EnsureServoOn(a) || !EnsurePosModeNoStop(a)) continue;

		long long cur = (long long)g_status.axesStatus[a].actualPos;

		Motion::PosCommand pc;
		pc.axis = a;
		pc.target = cur + (long long)(sign * 1000000000LL);
		pc.profile.type = ProfileType::SCurve;
		pc.profile.velocity = (int)std::lround(vel);
		pc.profile.acc = TimeMsToAcc(vel, acc);
		pc.profile.dec = TimeMsToAcc(vel, dec);

		long e = g_cm.motion->StartPos(&pc);
		if (e == ErrorCode::None) {
			g_lastCmdVel[a] = (int)std::lround((double)pc.profile.velocity * sign);

			// Multi jog stop을 위해 축별 active 표시
			g_multiJogAxisActive[a] = true;
			anyStarted = true;

			// 로그 트래킹
			g_axisCmdInfo[a].axis = a;
			g_axisCmdInfo[a].target = pc.target;
			g_axisCmdInfo[a].vel = pc.profile.velocity;
			g_axisCmdInfo[a].acc = pc.profile.acc;
			g_axisCmdInfo[a].dec = pc.profile.dec;
			g_axisCmdInfo[a].startTick = GetTickCount64();
			g_axisCmdInfo[a].endTick = 0;
			g_axisCmdInfo[a].active = true;
			g_axisLogEnabled[a] = true;
			g_axisLogRowIdx[a] = 0;
		}
		else {
			ShowErrMsgBox(TEXT("Jog start failed"), e, g_wmx);
		}
	}

	if (anyStarted) {
		g_multiJogActive = true;
		g_multiJogSign = sign;
		SetCapture(hWnd);
	}

	if (!slaveSkipped.empty()) {
		std::basic_string<TCHAR> msg;
		msg += TEXT("[Sync Group] 아래 축들은 Sync Group이 'Enabled' 상태에서 Slave로 설정되어 있어,\r\n");
		msg += TEXT("Checked Axis의 개별 Jog 명령이 무시되거나 기대와 다르게 동작할 수 있습니다.\r\n");
		msg += TEXT("→ Sync Group을 Disable 하거나, Master 축을 Jog 하세요.\r\n\r\n");
		for (const auto& s : slaveSkipped) {
			TCHAR line[128];
			_stprintf_s(line, TEXT(" - Axis %d (Group %d, Master %d)\r\n"), s.axis, s.gid, s.master);
			msg += line;
		}
		MessageBox(hWnd, msg.c_str(), TEXT("Checked Axis - Jog"), MB_OK | MB_ICONINFORMATION);
	}
}

static void Multi_DoHome(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Home(Checked)"), MB_ICONWARNING); return; }
	for (int a = 0; a < kNumAxes - 3; ++a) if (IsAxisChecked(hWnd, a)) {
		if (!EnsureServoOn(a)) continue;
		g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a);
		if (EnterPosMode(a)) { long e = g_home.StartHome(a); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Home 시작 실패"), e, g_wmx); }
		Sleep(5);
	}
}

static void Multi_DoSvOn(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;
	for (int a = 0; a < kNumAxes - 3; ++a) if (IsAxisChecked(hWnd, a)) {
		long e = g_cm.axisControl->SetServoOn(a, 1); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Servo ON(Selected) 실패"), e, g_wmx);
	}
}
static void Multi_DoSvOff(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;
	StopMultiJog();
	for (int a = 0; a < kNumAxes - 3; ++a) if (IsAxisChecked(hWnd, a)) {
		long e = g_cm.axisControl->SetServoOn(a, 0); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Servo OFF(Selected) 실패"), e, g_wmx);
	}
}
static void Multi_DoStop(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;
	StopMultiJog();
	for (int a = 0; a < kNumAxes - 3; ++a) if (IsAxisChecked(hWnd, a)) StopAxis(a);
}
static void Multi_DoAlarmReset(HWND hWnd) { if (!IsManualAllowed(hWnd)) return; DoMultiAlarmReset(hWnd); }


// ------------------ Jog Button: hold-to-run (ported from OHT_Wongwang.cpp) ------------------
static LRESULT CALLBACK JogButtonSubclassProc(HWND hBtn, UINT msg, WPARAM wParam, LPARAM lParam,
	UINT_PTR uIdSubclass, DWORD_PTR dwRefData)
{
	JogBtnCtx* ctx = reinterpret_cast<JogBtnCtx*>(dwRefData);
	HWND hParent = GetParent(hBtn);

	switch (msg) {
	case WM_LBUTTONDOWN:
		if (!ctx) break;

		// 기존 조그가 있으면 먼저 정리(동일 동작: single jog <-> multi jog 서로 배타)
		if (g_multiJogActive) StopMultiJog();
		if (g_jogActiveAxis >= 0) StopJogIfActive();

		if (ctx->kind == JogBtnKind::SelectedGroup) {
			SelectedGroup_Jog(hParent, ctx->groupIdx, ctx->sign);
			SetCapture(hBtn);
			return 0;
		}
		if (ctx->kind == JogBtnKind::MultiChecked) {
			Multi_DoJog(hParent, ctx->sign);
			SetCapture(hBtn);
			return 0;
		}
		if (ctx->kind == JogBtnKind::Sync) {
			if (!IsManualAllowed(hParent)) { return 0; }

			int ax = Sync_GetControlAxis(hParent);
			double vpps = GetDlgDouble(hParent, ID_SYNC_JOG_SPEED, 10000.0);
			double tAcc = GetDlgDouble(hParent, ID_SYNC_ACC, 100.0);
			double tDec = GetDlgDouble(hParent, ID_SYNC_DEC, 100.0);

			StartJogWithProfileParams(hParent, ax, ctx->sign, vpps, tAcc, tDec);
			SetCapture(hBtn);
			return 0;
		}
		break;

	case WM_LBUTTONUP:
	case WM_CAPTURECHANGED:
		StopJogIfActive();
		StopMultiJog();
		ReleaseCapture();
		return 0;

	case WM_NCDESTROY:
		RemoveWindowSubclass(hBtn, JogButtonSubclassProc, uIdSubclass);
		delete ctx;
		break;
	}

	return DefSubclassProc(hBtn, msg, wParam, lParam);
}

static void AttachHoldToJogButton(HWND hBtn, JogBtnKind kind, int groupIdx, int sign)
{
	if (!hBtn) return;
	JogBtnCtx* ctx = new JogBtnCtx{ kind, groupIdx, sign };
	// uIdSubclass는 임의(1)로 통일
	SetWindowSubclass(hBtn, JogButtonSubclassProc, 1, (DWORD_PTR)ctx);
}




// 상단 툴바 + 상단 센서/상태 + selected axis x4 (VERTICAL STACK) + checked axis control(버튼만) + 상태표
static void CreateUI_MainRebuild(HWND h)
{
	// 창 크기: 세로 스택이므로 세로 높이를 넉넉히 확보
	SetWindowPos(h, HWND_TOP, 0, 0,
		std::max(1420, GetSystemMetrics(SM_CXSCREEN)),
		std::max(1400, GetSystemMetrics(SM_CYSCREEN)),
		SWP_SHOWWINDOW);

	// 상단 툴바 라인 (왼쪽부터)
	int x = 10, y = 10, hbtn = 28, gap = 6;

	CreateWindow(TEXT("BUTTON"), TEXT("Create Device"), WS_CHILD | WS_VISIBLE,
		x, y, 140, hbtn, h, (HMENU)(INT_PTR)ID_BTN_CREATE_DEVICE, nullptr, nullptr); x += 140 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("Start Communication"), WS_CHILD | WS_VISIBLE,
		x, y, 170, hbtn, h, (HMENU)(INT_PTR)ID_BTN_START_COMM, nullptr, nullptr); x += 170 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("Demo"), WS_CHILD | WS_VISIBLE,
		x, y, 100, hbtn, h, (HMENU)(INT_PTR)ID_BTN_DEMO_MAIN, nullptr, nullptr); x += 100 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("GPIO"), WS_CHILD | WS_VISIBLE,
		x, y, 100, hbtn, h, (HMENU)(INT_PTR)ID_BTN_GPIO_MAIN, nullptr, nullptr); x += 100 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("비상정지"), WS_CHILD | WS_VISIBLE,
		x, y, 90, hbtn, h, (HMENU)(INT_PTR)ID_BTN_ESTOP_TOGGLE, nullptr, nullptr); x += 90 + gap;

	CreateWindow(TEXT("STATIC"), TEXT("NORMAL"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER,
		x, y + 3, 120, hbtn - 6, h, (HMENU)(INT_PTR)ID_TXT_ESTOP_STATE, nullptr, nullptr); x += 120 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("Barcode Demo"), WS_CHILD | WS_VISIBLE,
		x, y, 120, hbtn, h, (HMENU)(INT_PTR)ID_BTN_DEMO2, nullptr, nullptr); x += 120 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("Sync Group..."), WS_CHILD | WS_VISIBLE,
		x, y, 120, hbtn, h, (HMENU)(INT_PTR)ID_BTN_SYNC_WINDOW, nullptr, nullptr); x += 120 + gap;

	CreateWindow(TEXT("BUTTON"), TEXT("Serial Monitor..."), WS_CHILD | WS_VISIBLE,
		x, y, 150, hbtn, h, (HMENU)(INT_PTR)ID_BTN_SERIAL_WINDOW, nullptr, nullptr);

	// 상단 센서/모드/ECAT 표시 라인
	int y2 = y + hbtn + 10;
	int sx = 10;

	// Axis2 Limit/Home
	CreateWindow(TEXT("STATIC"), TEXT("A2 Limit:"), WS_CHILD | WS_VISIBLE | SS_LEFT, sx, y2, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx + 75, y2 - 2, 90, 22, h, (HMENU)(INT_PTR)ID_TXT_AX2_LIMIT, nullptr, nullptr);
	sx += 75 + 90 + 30;

	CreateWindow(TEXT("STATIC"), TEXT("A2 Home:"), WS_CHILD | WS_VISIBLE | SS_LEFT, sx, y2, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx + 75, y2 - 2, 90, 22, h, (HMENU)(INT_PTR)ID_TXT_AX2_HOME, nullptr, nullptr);
	sx += 75 + 90 + 30;

	// Axis0 Left/Right Limit
	CreateWindow(TEXT("STATIC"), TEXT("A0 L-Lim:"), WS_CHILD | WS_VISIBLE | SS_LEFT, sx, y2, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx + 75, y2 - 2, 90, 22, h, (HMENU)(INT_PTR)ID_TXT_AX0_LIMIT_L, nullptr, nullptr);
	sx += 75 + 90 + 30;

	CreateWindow(TEXT("STATIC"), TEXT("A0 R-Lim:"), WS_CHILD | WS_VISIBLE | SS_LEFT, sx, y2, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx + 75, y2 - 2, 90, 22, h, (HMENU)(INT_PTR)ID_TXT_AX0_LIMIT_R, nullptr, nullptr);
	sx += 75 + 90 + 30;

	// ECAT 6063/603F 간단표
	CreateWindow(TEXT("STATIC"), TEXT("Barcode(6063):"), WS_CHILD | WS_VISIBLE | SS_LEFT, sx, y2, 100, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx + 105, y2 - 2, 120, 22, h, (HMENU)(INT_PTR)ID_TXT_ECAT_6063, nullptr, nullptr);
	sx += 105 + 120 + 30;

	CreateWindow(TEXT("STATIC"), TEXT("Err603F:"), WS_CHILD | WS_VISIBLE | SS_LEFT, sx, y2, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx + 75, y2 - 2, 90, 22, h, (HMENU)(INT_PTR)ID_TXT_ECAT_603F, nullptr, nullptr);
	sx += 75 + 90 + 30;

	// 모드 표시
	CreateWindow(TEXT("BUTTON"), TEXT("Manual Mode"), WS_CHILD | WS_VISIBLE, sx, y2 - 2, 110, 24, h, (HMENU)ID_BTN_MODE_MANUAL, nullptr, nullptr);
	sx += 110 + 6;
	CreateWindow(TEXT("BUTTON"), TEXT("Auto Mode"), WS_CHILD | WS_VISIBLE, sx, y2 - 2, 110, 24, h, (HMENU)ID_BTN_MODE_AUTO, nullptr, nullptr);
	sx += 110 + 6;
	CreateWindow(TEXT("STATIC"), TEXT("MODE: MANUAL"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, sx, y2 - 2, 140, 22, h, (HMENU)ID_TXT_MODE_STATE, nullptr, nullptr);

	// ================= Selected Axis Control x4 (Vertical Stack) =================
	int groupTop = y2 + 40;

	auto CreateSelGroup = [&](int groupIdx, int leftX, int topY, const TCHAR* title) {
		// IMPORTANT:
		// 기존처럼 입력필드 뒤(오른쪽 끝)에 버튼을 붙이면, DPI/창폭에 따라 버튼이 화면 밖으로 밀려
		// "버튼이 안 보인다" 문제가 자주 발생합니다. 그래서 버튼을 왼쪽(축 선택 바로 옆)으로 배치하고,
		// 입력필드는 뒤로(오른쪽) 밀리도록 재배치했습니다.
		CreateWindow(TEXT("BUTTON"), title, WS_CHILD | WS_VISIBLE | BS_GROUPBOX, leftX, topY, 1670, 110, h, nullptr, nullptr, nullptr);

		const int row1Y = topY + 26;
		const int row2Y = row1Y + 34;
		int cx = leftX + 10;

		SelIds ids = GetSelIds(groupIdx);

		// Axis Selector (중복 방지 콤보는 RefreshSelAxisCombos에서 채움)
		CreateWindow(TEXT("STATIC"), TEXT("Axis"), WS_CHILD | WS_VISIBLE, cx, row1Y + 4, 40, 20, h, nullptr, nullptr, nullptr);
		CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
			cx + 45, row1Y, 70, 200, h, (HMENU)ids.cmbAxis, 0, 0);

		// === Row1: 핵심 제어 버튼을 '왼쪽'에 고정 ===
		int bx = cx + 130;
		CreateWindow(TEXT("BUTTON"), TEXT("Servo ON"), WS_CHILD | WS_VISIBLE, bx, row1Y, 80, 26, h, (HMENU)ids.bSvOn, 0, 0); bx += 85;
		CreateWindow(TEXT("BUTTON"), TEXT("Servo OFF"), WS_CHILD | WS_VISIBLE, bx, row1Y, 85, 26, h, (HMENU)ids.bSvOff, 0, 0); bx += 90;
		CreateWindow(TEXT("BUTTON"), TEXT("Home"), WS_CHILD | WS_VISIBLE, bx, row1Y, 65, 26, h, (HMENU)ids.bHome, 0, 0); bx += 70;
		CreateWindow(TEXT("BUTTON"), TEXT("Stop"), WS_CHILD | WS_VISIBLE, bx, row1Y, 65, 26, h, (HMENU)ids.bStop, 0, 0); bx += 75;
		CreateWindow(TEXT("BUTTON"), TEXT("Absolute"), WS_CHILD | WS_VISIBLE, bx, row1Y, 80, 26, h, (HMENU)ids.bAbs, 0, 0); bx += 85;
		CreateWindow(TEXT("BUTTON"), TEXT("Relative"), WS_CHILD | WS_VISIBLE, bx, row1Y, 80, 26, h, (HMENU)ids.bRel, 0, 0); bx += 85;
		HWND hSelJogP = CreateWindow(TEXT("BUTTON"), TEXT("Jog +"), WS_CHILD | WS_VISIBLE, bx, row1Y, 60, 26, h, (HMENU)ids.bJogP, 0, 0); bx += 65;
		if (hSelJogP) AttachHoldToJogButton(hSelJogP, JogBtnKind::SelectedGroup, groupIdx, +1);
		HWND hSelJogM = CreateWindow(TEXT("BUTTON"), TEXT("Jog -"), WS_CHILD | WS_VISIBLE, bx, row1Y, 60, 26, h, (HMENU)ids.bJogM, 0, 0); bx += 75;
		if (hSelJogM) AttachHoldToJogButton(hSelJogM, JogBtnKind::SelectedGroup, groupIdx, -1);

		// === Row1: 자주 쓰는 입력필드(오른쪽으로 밀리더라도 버튼은 보이게) ===
		int px = bx;
		FieldPos f1 = TightLabeledEdit2(h, px, row1Y + 3, TEXT("Abs Pos"), 6, 110, ids.eAbs, TEXT("0"), TEXT("p"), 4); px = f1.endX + 6;
		FieldPos f3 = TightLabeledEdit2(h, px, row1Y + 3, TEXT("Velocity"), 6, 110, ids.eVel, TEXT("10000"), TEXT("pps"), 4); px = f3.endX + 6;
		FieldPos f4 = TightLabeledEdit2(h, px, row1Y + 3, TEXT("Acc"), 6, 95, ids.eAcc, TEXT("100"), TEXT("ms"), 4); px = f4.endX + 6;
		FieldPos f5 = TightLabeledEdit2(h, px, row1Y + 3, TEXT("Dec"), 6, 95, ids.eDec, TEXT("100"), TEXT("ms"), 4);

		// === Row2: 상대이동/Alt 설정 ===
		int px2 = cx + 130;
		FieldPos f2 = TightLabeledEdit2(h, px2, row2Y + 3, TEXT("Rel Step"), 8, 110, ids.eRel, TEXT("0"), TEXT("p"), 10); px2 = f2.endX + 12;
		FieldPos f6 = TightLabeledEdit2(h, px2, row2Y + 3, TEXT("Alt Target"), 8, 110, ids.eAlt, TEXT("0"), TEXT("p"), 10); px2 = f6.endX + 10;
		CreateWindow(TEXT("BUTTON"), TEXT("Apply Alt→Abs"), WS_CHILD | WS_VISIBLE, px2, row2Y, 120, 26, h, (HMENU)ids.bApplyAlt, nullptr, nullptr);
		};


	// 수직으로 4개 배치 (왼쪽 X 고정, Y만 증가)
	const int leftX = 10;
	const int groupHeight = 128; // 그룹 박스 높이(2줄 레이아웃)
	CreateSelGroup(0, leftX, groupTop + 0 * groupHeight, TEXT("Selected Axis Control A"));
	CreateSelGroup(1, leftX, groupTop + 1 * groupHeight, TEXT("Selected Axis Control B"));
	CreateSelGroup(2, leftX, groupTop + 2 * groupHeight, TEXT("Selected Axis Control C"));
	CreateSelGroup(3, leftX, groupTop + 3 * groupHeight, TEXT("Selected Axis Control D"));


	// 그룹 콤보(축 중복 방지) 구성 + 축별 파라미터 로드
	RefreshSelAxisCombos(h);
	for (int g = 0; g < 4; ++g) LoadAxisParamToUi(h, g);

	// ================= Checked Axis Control (버튼만) =================
	int group2Top = groupTop + 4 * groupHeight + 10;
	CreateWindow(TEXT("BUTTON"), TEXT("Checked Axis Control"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, group2Top, 1670, 110, h, nullptr, nullptr, nullptr);

	int cx2 = 20, cy2 = group2Top + 28;

	// 버튼 줄
	CreateWindow(TEXT("BUTTON"), TEXT("전부 체크"), WS_CHILD | WS_VISIBLE, cx2, cy2, 100, 26, h, (HMENU)ID_MULTI_BTN_ALLCHECK, 0, 0); cx2 += 105;
	CreateWindow(TEXT("BUTTON"), TEXT("전부 체크 해제"), WS_CHILD | WS_VISIBLE, cx2, cy2, 120, 26, h, (HMENU)ID_MULTI_BTN_ALLCLEAR, 0, 0); cx2 += 125;
	CreateWindow(TEXT("BUTTON"), TEXT("Absolute Move"), WS_CHILD | WS_VISIBLE, cx2, cy2, 140, 26, h, (HMENU)ID_MULTI_BTN_ABS, 0, 0); cx2 += 145;
	CreateWindow(TEXT("BUTTON"), TEXT("Relative Move"), WS_CHILD | WS_VISIBLE, cx2, cy2, 140, 26, h, (HMENU)ID_MULTI_BTN_REL, 0, 0); cx2 += 145;
	HWND hMultiJogP = CreateWindow(TEXT("BUTTON"), TEXT("Jog +"), WS_CHILD | WS_VISIBLE, cx2, cy2, 80, 26, h, (HMENU)ID_MULTI_BTN_JOGP, 0, 0); cx2 += 85;
	if (hMultiJogP) AttachHoldToJogButton(hMultiJogP, JogBtnKind::MultiChecked, -1, +1);
	HWND hMultiJogM = CreateWindow(TEXT("BUTTON"), TEXT("Jog -"), WS_CHILD | WS_VISIBLE, cx2, cy2, 80, 26, h, (HMENU)ID_MULTI_BTN_JOGM, 0, 0); cx2 += 85;
	if (hMultiJogM) AttachHoldToJogButton(hMultiJogM, JogBtnKind::MultiChecked, -1, -1);
	CreateWindow(TEXT("BUTTON"), TEXT("Home (Checked)"), WS_CHILD | WS_VISIBLE, cx2, cy2, 140, 26, h, (HMENU)ID_MULTI_BTN_HOME, 0, 0); cx2 += 145;
	CreateWindow(TEXT("BUTTON"), TEXT("Servo ON (Checked)"), WS_CHILD | WS_VISIBLE, cx2, cy2, 160, 26, h, (HMENU)ID_MULTI_BTN_SVON, 0, 0); cx2 += 165;
	CreateWindow(TEXT("BUTTON"), TEXT("Servo OFF (Checked)"), WS_CHILD | WS_VISIBLE, cx2, cy2, 165, 26, h, (HMENU)ID_MULTI_BTN_SVOFF, 0, 0); cx2 += 170;
	CreateWindow(TEXT("BUTTON"), TEXT("Stop (Checked)"), WS_CHILD | WS_VISIBLE, cx2, cy2, 120, 26, h, (HMENU)ID_MULTI_BTN_STOP, 0, 0); cx2 += 125;
	CreateWindow(TEXT("BUTTON"), TEXT("Alarm Reset (Checked)"), WS_CHILD | WS_VISIBLE, cx2, cy2, 180, 26, h, (HMENU)ID_MULTI_BTN_ALARMRST, 0, 0);

	// 선택 체크박스 (0~8축)
	int chkTop = cy2 + 40;
	CreateWindow(TEXT("STATIC"), TEXT("Selected: (none)"), WS_CHILD | WS_VISIBLE | SS_LEFT, 850, chkTop, 250, 20, h, (HMENU)ID_TXT_SELECTED_AXES, 0, 0);
	for (int a = 0; a < kNumAxes - 3; ++a) {
		int xx = 20 + a * 90;
		TCHAR cap[16]; _stprintf_s(cap, TEXT("Axis %d"), a);
		CreateWindow(TEXT("BUTTON"), cap, WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
			xx, chkTop, 85, 26, h, (HMENU)ID_CHECK_AXIS(a), nullptr, nullptr);
	}

	// ================= 상태 테이블 (작은 글꼴) =================
	int statusTop = group2Top + 120;
	int statusH = 24 + (kNumAxes - 3 * (20 + 3)) + 30;
	CreateWindow(TEXT("BUTTON"), TEXT("Status"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, statusTop, 1670, statusH, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("m/s 설정..."), WS_CHILD | WS_VISIBLE, 1540, statusTop + 18, 110, 24, h, (HMENU)ID_BTN_VELCONV_MAIN, nullptr, nullptr);

	int ox = 20, oy = statusTop + 24, colW = 100, rowH = 20;

	HWND lblAxis = CreateWindow(TEXT("STATIC"), TEXT("Axis"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox, oy, 60, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl1 = CreateWindow(TEXT("STATIC"), TEXT("Servo"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl2 = CreateWindow(TEXT("STATIC"), TEXT("In Position"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl3 = CreateWindow(TEXT("STATIC"), TEXT("Motioning"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 2, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl4 = CreateWindow(TEXT("STATIC"), TEXT("CmdPos"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 3, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl5 = CreateWindow(TEXT("STATIC"), TEXT("ActPos"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 4, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl6 = CreateWindow(TEXT("STATIC"), TEXT("CmdVel"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 5, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl7 = CreateWindow(TEXT("STATIC"), TEXT("ActVel(rpm)"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 6, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl8 = CreateWindow(TEXT("STATIC"), TEXT("ActVel(m/s)"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 7, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl9 = CreateWindow(TEXT("STATIC"), TEXT("CmdTrq"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 8, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl10 = CreateWindow(TEXT("STATIC"), TEXT("ActTrq"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 9, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl11 = CreateWindow(TEXT("STATIC"), TEXT("PosErr"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 10, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl12 = CreateWindow(TEXT("STATIC"), TEXT("AmpAlarm"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 11, oy, colW, rowH, h, nullptr, nullptr, nullptr);
	HWND lbl13 = CreateWindow(TEXT("STATIC"), TEXT("Err(0x603F)"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 70 + colW * 12, oy, colW + 20, rowH, h, nullptr, nullptr, nullptr);

	HWND labels[] = { lblAxis,lbl1,lbl2,lbl3,lbl4,lbl5,lbl6,lbl7,lbl8,lbl9,lbl10,lbl11,lbl12,lbl13 };
	for (HWND lab : labels) if (lab) SendMessage(lab, WM_SETFONT, (WPARAM)GetSmallFont(), TRUE);

	oy += rowH + 3;
	for (int a = 0; a < kNumAxes - 3; ++a) {
		TCHAR lab[16]; _stprintf_s(lab, TEXT("Axis %d"), a);
		HWND stA = CreateWindow(TEXT("STATIC"), lab, WS_CHILD | WS_VISIBLE | SS_CENTER, ox, oy + a * (rowH + 3), 60, rowH, h, nullptr, nullptr, nullptr);

		HWND h0 = CreateWindow(TEXT("STATIC"), TEXT("OFF"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 0), nullptr, nullptr);
		HWND h1 = CreateWindow(TEXT("STATIC"), TEXT("OUT OF POSITION"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 1), nullptr, nullptr);
		HWND h2 = CreateWindow(TEXT("STATIC"), TEXT("IDLE"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 2, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 2), nullptr, nullptr);
		HWND h3 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 3, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 3), nullptr, nullptr);
		HWND h4 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 4, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 4), nullptr, nullptr);
		HWND h5 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 5, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 5), nullptr, nullptr);
		HWND h6 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 6, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 6), nullptr, nullptr);
		HWND h7 = CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 7, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 7), nullptr, nullptr);
		HWND h8 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 8, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 8), nullptr, nullptr);
		HWND h9 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 9, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 9), nullptr, nullptr);
		HWND h10 = CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 10, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 10), nullptr, nullptr);
		HWND h11 = CreateWindow(TEXT("STATIC"), TEXT("OK"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 11, oy + a * (rowH + 3), colW, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 11), nullptr, nullptr);
		HWND h12 = CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 70 + colW * 12, oy + a * (rowH + 3), colW + 20, rowH, h, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 12), nullptr, nullptr);

		HWND rowCtrls[] = { stA,h0,h1,h2,h3,h4,h5,h6,h7,h8,h9,h10,h11,h12 };
		for (HWND rc : rowCtrls) if (rc) SendMessage(rc, WM_SETFONT, (WPARAM)GetSmallFont(), TRUE);
	}
}
// 한 통신 주기 이상 기다리기 위한 헬퍼
static void WaitOneCommCycle()
{
	// 통신주기가 1ms 근처라면 10ms 정도면 충분히 여유 있음
	Sleep(10);
}

static void AutoStart(HWND hWnd)
{
	// 이미 한 번 수행했다면 스킵
	bool expected = false;
	if (!g_autoStartDone.compare_exchange_strong(expected, true)) {
		return;
	}

	// UI가 안정화될 시간을 충분히 둡니다. (서비스/드라이버 준비 포함)
	Sleep(1500);

	// 이미 열린 경우는 스킵
	if (!g_deviceOpened) {
		if (!InitDevice()) {
			MessageBox(hWnd, TEXT("AutoStart: CreateDevice 실패. (다른 인스턴스가 이미 사용 중이거나 드라이버 초기화 지연일 수 있습니다)\r\n"
				"프로그램을 다시 실행하거나 관리자 권한으로 실행해 보세요."),
				TEXT("AutoStart"), MB_ICONERROR);
			return;
		}
	}

	Sleep(200); // 한 틱 대기

	if (!g_commStarted) {
		if (!StartComm()) {
			MessageBox(hWnd, TEXT("AutoStart: StartCommunication 실패."), TEXT("AutoStart"), MB_ICONERROR);
			return;
		}
		Sleep(50);
	}

	// Servo ON
	/*for (int a = 0; a < kNumAxes-3; ++a) {
		EnsureServoOn(a);
		EnsurePosModeNoStop(a);
	}*/

	//std::this_thread::sleep_for(std::chrono::seconds(3));
	//// Sync Group 0: Master=0, Slave=[1]
	//if (g_commStarted) {
	//	Sync::SyncGroup grp{};
	//	grp.masterAxis = 0;
	//	grp.slaveAxisCount = 1;
	//	grp.slaveAxis[0] = 1;
	//	grp.servoOnOffSynchronization = 1;
	//	grp.startupType = Sync::SyncGroupStartupType::Normal;
	//	grp.gantryLoopCycleRatio = 1;
	//	grp.maxCatchUpDistance = 0.0;
	//	grp.catchUpVelocity = 0.0;
	//	grp.catchUpAcc = 0.0;
	//	grp.syncErrorTolerance = 1000.0;
	//	grp.useMasterFeedback = 0;

	//	Sync::SyncGroupStatus gst{};
	//	if (g_cm.sync->GetSyncGroupStatus(0, &gst) == ErrorCode::None && gst.enabled) {
	//		g_cm.sync->EnableSyncGroup(0, 0);
	//		Sleep(10);
	//	}

	//	long se = g_cm.sync->SetSyncGroup(0, grp);
	//	if (se == ErrorCode::None) {
	//		Sleep(10);
	//		Config::SyncParam sp{};
	//		if (g_cm.config->GetSyncParam(grp.masterAxis, &sp) == ErrorCode::None) {
	//			sp.masterDesyncDec = 10000.0;
	//			sp.slaveDesyncDec = 10000.0;
	//			g_cm.config->SetSyncParam(grp.masterAxis, &sp, nullptr);
	//			Sleep(10);
	//		}
	//		g_cm.sync->EnableSyncGroup(0, 1);
	//	}
	//}

	PostMessage(hWnd, WM_APP_SHOW_DEMO_MIN, 0, 0);

	UpdateEStopUi(hWnd, false);
	UpdateTcpUiState(hWnd);

}


static void LaunchGPIOWindow() {
	std::thread([] {
		HINSTANCE hInst = GetModuleHandle(nullptr);
		RunGPIOWindowExternal(hInst);
		}).detach();
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_CREATE:
	{
		g_hMainWnd = hWnd;
		INITCOMMONCONTROLSEX icc; icc.dwSize = sizeof(icc); icc.dwICC = ICC_WIN95_CLASSES | ICC_LISTVIEW_CLASSES;
		InitCommonControlsEx(&icc);

		// 메인 UI 재배치
		CreateUI_MainRebuild(hWnd);

		InitVelConvDefaultsOnce();

		SetTimer(hWnd, ID_TIMER, POLL_MS, nullptr);
		UpdateEStopUi(hWnd, false);
		UpdateTcpUiState(hWnd);

		// Axis2 flags reset
		g_ax2LimitOn = false;
		g_ax2HomeOn = false;
		g_ax2LimitLatched = false;
		g_ax2LimitBlocking = false;
		g_ax2StopIssuedOnLimit = false;
		g_ax2HomingStarted = false;
		g_ax2HomeDebounceOn = false;
		g_ax2HomeLastTick = GetTickCount();
		g_ax2HomeRampIssued = false;

		// Axis0 flags reset
		g_ax0LimitLFree = true;
		g_ax0LimitRFree = true;
		g_ax0LimitLLatched = false;
		g_ax0LimitRLatched = false;
		g_ax0BlockPlus = false;
		g_ax0BlockMinus = false;

		// Selected 그룹: 축별 파라미터/축 선택 UI 초기 반영
		RefreshSelAxisCombos(hWnd);
		for (int g = 0; g < 4; ++g) LoadAxisParamToUi(hWnd, g);

		// AutoStart + TCP 시작
		std::thread([](HWND hMain) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			AutoStart(hMain);
			StartTcpServer();
			std::this_thread::sleep_for(std::chrono::seconds(1));
			}, hWnd).detach();
	}
	return 0;

	case WM_APP_SHOW_DEMO_MIN:
		ShowDemoControlWindow(hWnd, true);
		return 0;

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);
		int code = HIWORD(wParam);

		// ======== Selected Axis Control: 파라미터 "즉시 저장" ========
		// 사용자가 축을 콤보박스로 바꾸지 않아도, 입력값이 바뀌는 즉시 해당 축 파라미터에 반영되도록 한다.
		// - EN_CHANGE: 타이핑 중에도 들어오므로, 숫자 파싱 실패 시엔 기존값을 유지(GetDlgDoubleOrDefIfInvalid)
		// - UI 로드 중(LoadAxisParamToUi 등)에는 notify를 무시(IsSelUiUpdating)
		if (!IsSelUiUpdating() && (code == EN_CHANGE || code == EN_KILLFOCUS)) {
			int gidx = -1;
			if (SelEditIdToGroupIdx(id, gidx)) {
				SaveAxisParamFromUi(hWnd, gidx);
				return 0;
			}
		}

		// 모드
		if (id == ID_BTN_MODE_MANUAL) {
			SwitchToManual(hWnd);
			if (HWND h = GetDlgItem(hWnd, ID_TXT_MODE_STATE)) SetWindowText(h, TEXT("MODE: MANUAL"));
			return 0;
		}
		if (id == ID_BTN_MODE_AUTO) {
			SwitchToAuto(hWnd);
			if (HWND h = GetDlgItem(hWnd, ID_TXT_MODE_STATE)) SetWindowText(h, TEXT("MODE: AUTO"));
			// Serial Monitor 창이 없으면 띄우기
			if (!g_hSerialWnd || !IsWindow(g_hSerialWnd)) {
				WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3SerialWnd");
				wc.lpfnWndProc = SerialWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hWnd, GWLP_HINSTANCE);
				wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
				wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
				RegisterClass(&wc);
				g_hSerialWnd = CreateWindow(TEXT("WMX3SerialWnd"), TEXT("Serial Monitor (TCP)"),
					WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
					CW_USEDEFAULT, CW_USEDEFAULT, 800, 400, hWnd, nullptr, wc.hInstance, nullptr);
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				UpdateWindow(g_hSerialWnd);
			}
			else {
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				SetForegroundWindow(g_hSerialWnd);
			}
			return 0;
		}

		if (id == ID_BTN_ESTOP_TOGGLE) { DoToggleEStop(hWnd, false); return 0; }

		if (id == ID_BTN_VELCONV_MAIN) { ShowVelConvWindow(hWnd); return 0; }

		// 체크박스 9축
		if (id >= ID_CHECK_AXIS(0) && id <= ID_CHECK_AXIS(kNumAxes - 3)) {
			if (code == BN_CLICKED) UpdateSelectedAxesTextOnDemand(hWnd);
			return 0;
		}

		if (id == ID_BTN_CREATE_DEVICE) {
			if (!IsManualAllowed(hWnd)) return 0;
			KillTimer(hWnd, ID_TIMER);
			bool ok = false;
			if (g_deviceOpened) { MessageBox(hWnd, TEXT("Device already created."), TEXT("Info"), MB_ICONINFORMATION); ok = true; }
			else { ok = InitDevice(); MessageBox(hWnd, ok ? TEXT("Success") : TEXT("Fail"), TEXT("Create Device"), ok ? MB_ICONINFORMATION : MB_ICONERROR); }
			SetTimer(hWnd, ID_TIMER, POLL_MS, nullptr);
			return 0;
		}
		if (id == ID_BTN_START_COMM) {
			if (!IsManualAllowed(hWnd)) return 0;
			KillTimer(hWnd, ID_TIMER);
			bool ok = false;
			if (!g_deviceOpened) MessageBox(hWnd, TEXT("Create device first."), TEXT("Start Communication"), MB_ICONWARNING);
			else if (g_commStarted) { MessageBox(hWnd, TEXT("Communication already started."), TEXT("Info"), MB_ICONINFORMATION); ok = true; }
			else { ok = StartComm(); MessageBox(hWnd, ok ? TEXT("Success") : TEXT("Fail"), TEXT("Start Communication"), ok ? MB_ICONINFORMATION : MB_ICONERROR); }
			SetTimer(hWnd, ID_TIMER, POLL_MS, nullptr);
			UpdateEStopUi(hWnd, false);
			return 0;
		}

		if (id == ID_BTN_DEMO_MAIN) {
			if (!IsManualAllowed(hWnd)) return 0;
			ShowDemoControlWindow(hWnd, false);
			return 0;
		}
		if (id == ID_BTN_GPIO_MAIN) {
			if (!IsManualAllowed(hWnd)) return 0;
			LaunchGPIOWindow();
			return 0;
		}
		if (id == ID_BTN_DEMO2) {
			if (!IsManualAllowed(hWnd)) return 0;
			ShowBarcodeDemoWindow(hWnd);
			return 0;
		}

		if (id == ID_BTN_SYNC_WINDOW) {
			if (!IsManualAllowed(hWnd)) return 0;
			if (!g_deviceOpened || !g_commStarted) {
				MessageBox(hWnd, TEXT("Create device and start communication first."), TEXT("Sync Group"), MB_ICONWARNING);
				return 0;
			}
			if (!g_hSyncWnd || !IsWindow(g_hSyncWnd)) {
				WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3SyncWnd");
				wc.lpfnWndProc = SyncWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hWnd, GWLP_HINSTANCE);
				wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
				wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
				RegisterClass(&wc);
				g_hSyncWnd = CreateWindow(TEXT("WMX3SyncWnd"), TEXT("Sync Group Control/Monitor"),
					WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
					CW_USEDEFAULT, CW_USEDEFAULT, 1050, 1000, hWnd, nullptr, wc.hInstance, nullptr);
				ShowWindow(g_hSyncWnd, SW_SHOWNORMAL);
				UpdateWindow(g_hSyncWnd);
				UpdateEStopUi(g_hSyncWnd, true);
			}
			else {
				ShowWindow(g_hSyncWnd, SW_SHOWNORMAL);
				SetForegroundWindow(g_hSyncWnd);
				UpdateEStopUi(g_hSyncWnd, true);
			}
			return 0;
		}
		if (id == ID_BTN_SERIAL_WINDOW) {
			if (!g_hSerialWnd || !IsWindow(g_hSerialWnd)) {
				WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3SerialWnd");
				wc.lpfnWndProc = SerialWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hWnd, GWLP_HINSTANCE);
				wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
				wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
				RegisterClass(&wc);
				g_hSerialWnd = CreateWindow(TEXT("WMX3SerialWnd"), TEXT("Serial Monitor (TCP)"),
					WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
					CW_USEDEFAULT, CW_USEDEFAULT, 800, 400, hWnd, nullptr, wc.hInstance, nullptr);
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				UpdateWindow(g_hSerialWnd);
			}
			else {
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				SetForegroundWindow(g_hSerialWnd);
			}
			return 0;
		}

		// ======== Selected Group A/B/C/D 버튼/이벤트 처리 ========

		// Combobox 변경: (1) 이전 축 파라미터 저장 → (2) 새 축 파라미터 로드 → (3) 4그룹 축 중복 방지
		if (!IsSelUiUpdating() && (id == ID_SEL_A_AXIS_COMBO || id == ID_SEL_B_AXIS_COMBO || id == ID_SEL_C_AXIS_COMBO || id == ID_SEL_D_AXIS_COMBO) && code == CBN_SELCHANGE) {
			int gidx = (id == ID_SEL_A_AXIS_COMBO) ? 0 : (id == ID_SEL_B_AXIS_COMBO ? 1 : (id == ID_SEL_C_AXIS_COMBO ? 2 : 3));
			OnSelGroupAxisChanged(hWnd, gidx);
			return 0;
		}


		// Apply Alt
		if (id == ID_SEL_A_BTN_APPLY_ALT) { SelectedGroup_ApplyAlt(hWnd, 0); return 0; }
		if (id == ID_SEL_B_BTN_APPLY_ALT) { SelectedGroup_ApplyAlt(hWnd, 1); return 0; }
		if (id == ID_SEL_C_BTN_APPLY_ALT) { SelectedGroup_ApplyAlt(hWnd, 2); return 0; }
		if (id == ID_SEL_D_BTN_APPLY_ALT) { SelectedGroup_ApplyAlt(hWnd, 3); return 0; }

		// Servo ON/OFF
		if (id == ID_SEL_A_BTN_SVON) { SelectedGroup_ServoOn(hWnd, 0); return 0; }
		if (id == ID_SEL_A_BTN_SVOFF) { SelectedGroup_ServoOff(hWnd, 0); return 0; }
		if (id == ID_SEL_B_BTN_SVON) { SelectedGroup_ServoOn(hWnd, 1); return 0; }
		if (id == ID_SEL_B_BTN_SVOFF) { SelectedGroup_ServoOff(hWnd, 1); return 0; }
		if (id == ID_SEL_C_BTN_SVON) { SelectedGroup_ServoOn(hWnd, 2); return 0; }
		if (id == ID_SEL_C_BTN_SVOFF) { SelectedGroup_ServoOff(hWnd, 2); return 0; }
		if (id == ID_SEL_D_BTN_SVON) { SelectedGroup_ServoOn(hWnd, 3); return 0; }
		if (id == ID_SEL_D_BTN_SVOFF) { SelectedGroup_ServoOff(hWnd, 3); return 0; }

		// Home
		if (id == ID_SEL_A_BTN_HOME) { SelectedGroup_Home(hWnd, 0); return 0; }
		if (id == ID_SEL_B_BTN_HOME) { SelectedGroup_Home(hWnd, 1); return 0; }
		if (id == ID_SEL_C_BTN_HOME) { SelectedGroup_Home(hWnd, 2); return 0; }
		if (id == ID_SEL_D_BTN_HOME) { SelectedGroup_Home(hWnd, 3); return 0; }

		// Abs/Rel/Stop
		if (id == ID_SEL_A_BTN_ABS) { SelectedGroup_Abs(hWnd, 0); return 0; }
		if (id == ID_SEL_A_BTN_REL) { SelectedGroup_Rel(hWnd, 0); return 0; }
		if (id == ID_SEL_A_BTN_STOP) { SelectedGroup_Stop(hWnd, 0); return 0; }

		if (id == ID_SEL_B_BTN_ABS) { SelectedGroup_Abs(hWnd, 1); return 0; }
		if (id == ID_SEL_B_BTN_REL) { SelectedGroup_Rel(hWnd, 1); return 0; }
		if (id == ID_SEL_B_BTN_STOP) { SelectedGroup_Stop(hWnd, 1); return 0; }

		if (id == ID_SEL_C_BTN_ABS) { SelectedGroup_Abs(hWnd, 2); return 0; }
		if (id == ID_SEL_C_BTN_REL) { SelectedGroup_Rel(hWnd, 2); return 0; }
		if (id == ID_SEL_C_BTN_STOP) { SelectedGroup_Stop(hWnd, 2); return 0; }

		if (id == ID_SEL_D_BTN_ABS) { SelectedGroup_Abs(hWnd, 3); return 0; }
		if (id == ID_SEL_D_BTN_REL) { SelectedGroup_Rel(hWnd, 3); return 0; }
		if (id == ID_SEL_D_BTN_STOP) { SelectedGroup_Stop(hWnd, 3); return 0; }


		// Jog +/- : "누르고 있는 동안만" 동작하도록
		// BN_CLICKED는 마우스/키보드가 "떼졌을 때" 들어오기 때문에, 시작은 BN_PUSHED에서만 한다.
		auto IsSelJogId = [&](int _id)->bool {
			return _id == ID_SEL_A_BTN_JOGP || _id == ID_SEL_A_BTN_JOGM ||
				_id == ID_SEL_B_BTN_JOGP || _id == ID_SEL_B_BTN_JOGM ||
				_id == ID_SEL_C_BTN_JOGP || _id == ID_SEL_C_BTN_JOGM ||
				_id == ID_SEL_D_BTN_JOGP || _id == ID_SEL_D_BTN_JOGM;
			};
		auto IsMultiJogId = [&](int _id)->bool { return _id == ID_MULTI_BTN_JOGP || _id == ID_MULTI_BTN_JOGM; };

		if (IsSelJogId(id) || IsMultiJogId(id)) {
			if (code == BN_PUSHED) {
				if (id == ID_SEL_A_BTN_JOGP) { SelectedGroup_Jog(hWnd, 0, +1); return 0; }
				if (id == ID_SEL_A_BTN_JOGM) { SelectedGroup_Jog(hWnd, 0, -1); return 0; }

				if (id == ID_SEL_B_BTN_JOGP) { SelectedGroup_Jog(hWnd, 1, +1); return 0; }
				if (id == ID_SEL_B_BTN_JOGM) { SelectedGroup_Jog(hWnd, 1, -1); return 0; }

				if (id == ID_SEL_C_BTN_JOGP) { SelectedGroup_Jog(hWnd, 2, +1); return 0; }
				if (id == ID_SEL_C_BTN_JOGM) { SelectedGroup_Jog(hWnd, 2, -1); return 0; }

				if (id == ID_SEL_D_BTN_JOGP) { SelectedGroup_Jog(hWnd, 3, +1); return 0; }
				if (id == ID_SEL_D_BTN_JOGM) { SelectedGroup_Jog(hWnd, 3, -1); return 0; }

				if (id == ID_MULTI_BTN_JOGP) { Multi_DoJog(hWnd, +1); return 0; }
				if (id == ID_MULTI_BTN_JOGM) { Multi_DoJog(hWnd, -1); return 0; }
			}

			if (code == BN_UNPUSHED) {
				StopJogIfActive();
				StopMultiJog();
				ReleaseCapture();
				return 0;
			}

			// BN_CLICKED 등은 무시(버튼을 '뗄 때' 조그가 다시 시작되는 버그 방지)
			return 0;
		}

		// ======== Checked Axis Control (버튼만) ========
		if (id == ID_MULTI_BTN_ALLCHECK) { Multi_SetAllAxisChecked(hWnd, true); return 0; }
		if (id == ID_MULTI_BTN_ALLCLEAR) { Multi_SetAllAxisChecked(hWnd, false); return 0; }
		if (id == ID_MULTI_BTN_ABS) { Multi_DoAbs(hWnd); return 0; }
		if (id == ID_MULTI_BTN_REL) { Multi_DoRel(hWnd); return 0; }
		if (id == ID_MULTI_BTN_JOGP) { Multi_DoJog(hWnd, +1); return 0; }
		if (id == ID_MULTI_BTN_JOGM) { Multi_DoJog(hWnd, -1); return 0; }
		if (id == ID_MULTI_BTN_HOME) { Multi_DoHome(hWnd); return 0; }
		if (id == ID_MULTI_BTN_SVON) { Multi_DoSvOn(hWnd); return 0; }
		if (id == ID_MULTI_BTN_SVOFF) { Multi_DoSvOff(hWnd); return 0; }
		if (id == ID_MULTI_BTN_STOP) { Multi_DoStop(hWnd); return 0; }
		if (id == ID_MULTI_BTN_ALARMRST) { Multi_DoAlarmReset(hWnd); return 0; }

		if (id == ID_BTN_MAP_WINDOW) { if (!IsManualAllowed(hWnd)) return 0; ShowMapWindow(hWnd); return 0; }
	}
	return 0;

	case WM_APP_MAP_GOTO:
	{
		MapGotoParam* p = reinterpret_cast<MapGotoParam*>(lParam);
		if (p) {
			if (g_commStarted) {
				int axis = 0;
				if (EnsureServoOn(axis) && EnsurePosModeNoStop(axis)) {
					// 여기서는 기본 SelectedA 파라미터로 속도/가감속을 가져와서 쓸 수도 있지만,
					// 현재는 Selected A 구조체를 사용하지 않고 기존 UI 필드가 제거되었으므로
					// 기본값으로 동작하도록 둔다.
					StartAbsMoveWithProfile(axis, p->target, 20000.0, 100.0, 100.0);
				}
			}
			delete p;
		}
		return 0;
	}

	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
	case WM_MBUTTONUP:
	case WM_KILLFOCUS:
	case WM_CANCELMODE:
		StopJogIfActive(); StopMultiJog(); ReleaseCapture(); return 0;

	case WM_APP_TCP_LOG:
	{
		wchar_t* p = (wchar_t*)lParam;
		if (g_hTcpLogList && p && IsWindow(g_hTcpLogList)) {
			SendMessage(g_hTcpLogList, LB_ADDSTRING, 0, (LPARAM)p);
			int cnt = (int)SendMessage(g_hTcpLogList, LB_GETCOUNT, 0, 0);
			SendMessage(g_hTcpLogList, LB_SETTOPINDEX, cnt - 1, 0);
		}
		if (p) free(p);
		return 0;
	}

	case WM_APP_TCP_STATE:
	{
		wchar_t* p = (wchar_t*)lParam;
		if (p) {
			if (HWND h = GetDlgItem(hWnd, ID_TXT_TCP_STATE)) {
				SetWindowTextW(h, p);
			}
			free(p);
		}
		else {
			UpdateTcpUiState(hWnd);
		}
		return 0;
	}

	case WM_TIMER:
		if (wParam == ID_TIMER)
		{
			// 통신 시작 전 표시 초기화
			if (!g_commStarted)
			{
				if (HWND h = GetDlgItem(hWnd, ID_TXT_ECAT_6063)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_ECAT_603F)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME))  SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX0_LIMIT_L)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX0_LIMIT_R))  SetWindowText(h, TEXT("-"));
				UpdateEStopUi(hWnd, false);
				UpdateTcpUiState(hWnd);
				return 0;
			}

			// EtherCAT 상태 갱신
			g_cm.GetStatus(&g_status);
			UpdateStatus(hWnd);

			// STO 펄스 자동 OFF
			if (g_ohtStoPulsePendingOff.load()) {
				DWORD now = GetTickCount();
				if (now - g_ohtStoPulseOnTick.load() >= kOhtStoPulseMs) {
					g_ohtStoPulsePendingOff = false;
					ToggleDO_HW(10, false, nullptr);
				}
			}

			// IO 신호 새로고침
			if (g_ioInitDone.load()) {
				RefreshLevels(hWnd);
			}

			// Axis2 ServoReady 지연 로직
			if (!g_ax2ServoReady.load())
			{
				if (IsAxis2ServoOn())
				{
					if (g_ax2ServoOnTime.load() == 0)
					{
						g_ax2ServoOnTime = GetTickCount();
					}
					else
					{
						DWORD diff = GetTickCount() - g_ax2ServoOnTime.load();
						if (diff >= AX2_SENSOR_ENABLE_DELAY_MS)
						{
							g_ax2ServoReady = true;
						}
					}
				}
			}

			// 센서 읽기
			bool limitRaw = ReadInputBit(AX2_LIMIT_ADDR, AX2_LIMIT_BIT, AX2_LIMIT_ACTIVE_HIGH);
			bool homeRaw = ReadInputBit(AX2_HOME_ADDR, AX2_HOME_BIT, AX2_HOME_ACTIVE_HIGH);
			g_ax2LimitOn = limitRaw;
			g_ax2HomeOn = homeRaw;

			if (!g_ax2ServoReady.load())
			{
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) SetWindowText(h, TEXT("WAIT"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME))  SetWindowText(h, TEXT("WAIT"));
			}
			else
			{
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) SetWindowText(h, limitRaw ? TEXT("ON") : TEXT("OFF"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME))  SetWindowText(h, homeRaw ? TEXT("ON") : TEXT("OFF"));
			}

			// Axis0 L/R Limit 처리
			{
				bool leftFree = ReadInputBit(AX0_LIMIT_L_ADDR, AX0_LIMIT_L_BIT, AX0_LIMIT_L_ACTIVE_HIGH);
				bool rightFree = ReadInputBit(AX0_LIMIT_R_ADDR, AX0_LIMIT_R_BIT, AX0_LIMIT_R_ACTIVE_HIGH);

				g_ax0LimitLFree = leftFree;
				g_ax0LimitRFree = rightFree;

				if (!leftFree)
				{
					if (!g_ax0LimitLLatched.exchange(true)) { StopAxis(0); }
					g_ax0BlockPlus = true;
				}
				else
				{
					g_ax0LimitLLatched = false;
					g_ax0BlockPlus = false;
				}

				if (!rightFree)
				{
					if (!g_ax0LimitRLatched.exchange(true)) { StopAxis(0); }
					g_ax0BlockMinus = true;
				}
				else
				{
					g_ax0LimitRLatched = false;
					g_ax0BlockMinus = false;
				}
			}

			return 0;
		}
		return 0;

	case WM_DESTROY:
		KillTimer(hWnd, ID_TIMER);
		StopJogIfActive(); StopMultiJog();
		g_demoRunning = false;
		for (int a = 0; a < kNumAxes - 3; ++a) StopAxis(a);
		StopTcpServer();

		ShutdownWMX();
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ------------------ WinMain ------------------
int APIENTRY _tWinMain(HINSTANCE hInst, HINSTANCE, LPTSTR, int nShow) {
	WNDCLASS wc{};
	wc.lpszClassName = TEXT("WMX3PracticeWnd");
	wc.lpfnWndProc = WndProc; wc.hInstance = hInst;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	RegisterClass(&wc);

	HWND h = CreateWindow(TEXT("WMX3PracticeWnd"), TEXT("WMX3 Practice - Demo + GPIO; Sync Group + ECAT6063/603F + AltTarget + E-Stop + SerialWnd + Log/Scope"),
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, 1850, 1080, nullptr, nullptr, hInst, nullptr);
	ShowWindow(h, nShow);
	UpdateWindow(h);

	MSG m;
	while (GetMessage(&m, nullptr, 0, 0)) { TranslateMessage(&m); DispatchMessage(&m); }
	return (int)m.wParam;
}

int main() { return _tWinMain(GetModuleHandle(nullptr), nullptr, GetCommandLine(), SW_SHOWNORMAL); }