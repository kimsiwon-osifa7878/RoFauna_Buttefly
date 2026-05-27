/*
  RoFauna_Buttefly
  ESP32-C3 SuperMini + Bluefruit Connect + 2 Servo Bionic Butterfly

  보드:
  - ESP32-C3 SuperMini
  - 왼쪽 서보 신호  : GPIO0
  - 오른쪽 서보 신호: GPIO1
  - 서보 전원은 별도 외부 전원을 사용
  - ESP32 GND와 서보 전원 GND는 공통 연결

  앱:
  - Adafruit Bluefruit Connect
  - BLE 장치명 "RoFauna_Buttefly"에 연결
  - Controller -> Control Pad 사용

  조작:
  - 대기 모드:
      Button 1 + Button 4 : ARM / 시작

  - 조종 모드:
      Button 1 + Button 4 : DISARM / 대기 모드

      UP          : 상승 / 스로틀 증가
      DOWN        : 하강 / 스로틀 감소
      LEFT        : 좌회전
      RIGHT       : 우회전
      Button 1    : 왼쪽 방향 트림
      Button 2    : 오른쪽 방향 트림
      Button 3    : 상승 방향 트림
      Button 4    : 하강 방향 트림

      Button 1 + Button 2 : 기본 스로틀 증가
      Button 3 + Button 4 : 기본 스로틀 감소

  참고:
  - Dabble 미사용
  - NimBLE-Arduino 미사용
  - ESP32 Arduino 내장 BLE 라이브러리 사용
*/

#include <Arduino.h>
#include <ESP32Servo.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// 장치 이름
// =====================================================

static const char* DEVICE_NAME = "RoFauna_Buttefly";

// =====================================================
// Nordic UART Service UUID
// Bluefruit Connect UART와 호환되는 UUID입니다.
// =====================================================

#define UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // 앱 -> ESP32
#define UART_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 -> 앱

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;

bool deviceConnected = false;
bool oldDeviceConnected = false;

// =====================================================
// 핀 설정
// =====================================================

const int SERVO_LEFT_PIN  = 0;
const int SERVO_RIGHT_PIN = 1;

// 많은 ESP32-C3 SuperMini 보드는 내장 LED가 GPIO8에 연결되어 있습니다.
// LED가 동작하지 않으면 실제 보드의 LED 핀에 맞게 8, 2, 10 등을 시험하세요.
const int LED_PIN = 8;

// LED 켜짐/꺼짐이 반대로 보이면 true로 바꿉니다.
const bool LED_ACTIVE_LOW = false;

// =====================================================
// 서보 설정
// =====================================================

Servo servoLeft;
Servo servoRight;

// PWM 펄스 폭 기준입니다. degree 값이 아닙니다.
const int SERVO_CENTER_US = 1500;
const int SERVO_MIN_US    = 500;
const int SERVO_MAX_US    = 2500;

// 11.1us를 약 1도로 보면 +-50도는 약 550us입니다.
int flapAmplitudeUs = 550;

// 한 루프에서 서보 명령이 변할 수 있는 최대 펄스 폭입니다.
const int MAX_SERVO_STEP_US = 100;

// 조종 입력의 최대 보정량입니다.
const int MAX_STEER_US = 160;
const int MAX_ELEV_US  = 160;

// 트림 값입니다.
int trimSteerUs = 0;
int trimElevUs  = 0;

const int TRIM_STEP_US  = 12;
const int TRIM_LIMIT_US = 160;

// 마지막으로 실제 출력한 서보 명령값입니다.
int lastLeftUs  = SERVO_CENTER_US;
int lastRightUs = SERVO_CENTER_US;

// =====================================================
// 스로틀 설정
// 기체가 충분히 날갯짓하지 않으면 먼저 이 값들을 조정합니다.
// =====================================================

const int BASE_THROTTLE_DEFAULT = 65;

const int THROTTLE_MIN = 35;
const int THROTTLE_MAX = 100;

const int THROTTLE_UP_BOOST   = 35;
const int THROTTLE_DOWN_DROP  = 30;
const int THROTTLE_STEP       = 10;

int baseThrottleLevel = BASE_THROTTLE_DEFAULT;

// =====================================================
// 버튼 상태
// Bluefruit Control Pad:
// 1,2,3,4 = 숫자 버튼
// 5,6,7,8 = Up, Down, Left, Right
// =====================================================

bool btnUp    = false;
bool btnDown  = false;
bool btnLeft  = false;
bool btnRight = false;

bool btn1 = false;
bool btn2 = false;
bool btn3 = false;
bool btn4 = false;

// 조합 버튼의 상승 에지를 감지하기 위한 이전 상태입니다.
bool prevCombo14 = false;

// =====================================================
// 비행 상태
// =====================================================

bool armed = false;

float phase = 0.0;
float currentOffsetUs = 0.0;

unsigned long lastBlinkMs       = 0;
unsigned long lastTrimMs        = 0;
unsigned long lastLogMs         = 0;
unsigned long lastThrottleAdjMs = 0;
unsigned long lastNeutralMs     = 0;

bool blinkState = false;

// =====================================================
// 함수 선언
// =====================================================

void setupBLE();
void handleBleConnection();

void standbyMode();
void armedMode();

void parseBluefruitPacket(const uint8_t* data, size_t len);
void updateButtonState(char buttonId, bool pressed);
String buttonName(char buttonId);

void updateTrimButtons(bool combo12, bool combo34, bool combo14);
void updateThrottleAdjust(bool combo12, bool combo34);
void updateFlapping(int throttleLevel);

void writeServosUs(int leftUs, int rightUs);
void writeServosUs(int leftUs, int rightUs, bool force);
void forceNeutralServos();
int limitStep(int current, int target, int maxStep);

void ledOn();
void ledOff();
void blinkLed(unsigned long intervalMs);

void sendToApp(const String& msg);
void logStatus(int throttleLevel, int steerUs, int elevUs, int targetLeftUs, int targetRightUs);
void savePreviousStates();

bool risingEdge(bool current, bool previous);
float approach(float current, float target, float step);

// =====================================================
// BLE 콜백
// =====================================================

class MyServerCallbacks : public BLEServerCallbacks {
public:
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE disconnected");
  }
};

class MyRxCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();

    if (value.length() == 0) {
      return;
    }

    parseBluefruitPacket((const uint8_t*)value.c_str(), value.length());
  }
};

// =====================================================
// 초기화
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  ledOff();

  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);

  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);

  // 전원 ON 직후 서보를 강제로 중심 위치에 둡니다.
  forceNeutralServos();

  Serial.println();
  Serial.println("==================================");
  Serial.println("RoFauna_Buttefly ESP32-C3 BLE");
  Serial.println("==================================");
  Serial.println("Initial: Servos forced to 1500us, LED blinking");
  Serial.println("Connect with Bluefruit Connect");
  Serial.println("Controller -> Control Pad");
  Serial.println("Press Button 1 + Button 4 to toggle ARM / STANDBY");
  Serial.print("Base throttle = ");
  Serial.println(baseThrottleLevel);
  Serial.println();

  setupBLE();
}

// =====================================================
// 메인 루프
// =====================================================

void loop() {
  handleBleConnection();

  if (!armed) {
    standbyMode();
  } else {
    armedMode();
  }

  savePreviousStates();

  delay(8);
}

// =====================================================
// BLE 설정
// =====================================================

void setupBLE() {
  BLEDevice::init(DEVICE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(UART_SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    UART_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );

  pTxCharacteristic->addDescriptor(new BLE2902());
  pTxCharacteristic->setValue("RoFauna_Buttefly ready\n");

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    UART_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );

  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.print("Advertising as ");
  Serial.println(DEVICE_NAME);
}

// =====================================================
// BLE 연결 관리
// =====================================================

void handleBleConnection() {
  // 연결이 끊기면 안전하게 DISARM하고 BLE 광고를 다시 시작합니다.
  if (!deviceConnected && oldDeviceConnected) {
    armed = false;
    trimSteerUs = 0;
    trimElevUs = 0;
    currentOffsetUs = 0;

    forceNeutralServos();

    delay(500);
    pServer->startAdvertising();

    Serial.println("BLE disconnected -> DISARM, force neutral, restart advertising");
    oldDeviceConnected = deviceConnected;
  }

  // 새로 연결되면 앱으로 기본 안내 메시지를 보냅니다.
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    sendToApp("RoFauna_Buttefly connected\n");
    sendToApp("Press Button 1 + Button 4 to toggle ARM / STANDBY\n");
  }
}

// =====================================================
// 대기 모드
// =====================================================

void standbyMode() {
  // 대기 모드에서는 주기적으로 서보를 중심 위치에 다시 씁니다.
  // 매 루프마다 쓰면 서보 떨림이나 소음이 생길 수 있어 간격을 둡니다.
  if (millis() - lastNeutralMs >= 250) {
    lastNeutralMs = millis();
    forceNeutralServos();
  }

  // 대기 중에는 LED를 점멸합니다.
  blinkLed(300);

  bool combo14 = btn1 && btn4;

  // Button 1 + Button 4 동시 입력의 상승 에지에서 ARM 상태로 전환합니다.
  if (risingEdge(combo14, prevCombo14)) {
    armed = true;

    phase = PI;
    currentOffsetUs = 0;

    trimSteerUs = 0;
    trimElevUs = 0;

    forceNeutralServos();
    ledOn();

    Serial.println("[ARMED] Button 1 + Button 4");
    sendToApp("[ARMED] Button 1 + Button 4\n");
  }
}

// =====================================================
// 조종 모드
// =====================================================

void armedMode() {
  ledOn();

  bool combo14 = btn1 && btn4;
  bool combo12 = btn1 && btn2;
  bool combo34 = btn3 && btn4;

  // Button 1 + Button 4 동시 입력의 상승 에지에서 DISARM합니다.
  if (risingEdge(combo14, prevCombo14)) {
    armed = false;

    trimSteerUs = 0;
    trimElevUs = 0;
    currentOffsetUs = 0;

    forceNeutralServos();

    Serial.println("[DISARMED] Button 1 + Button 4");
    sendToApp("[DISARMED] Back to standby\n");

    delay(250);
    return;
  }

  // 조종 모드에서 버튼 조합으로 기본 스로틀을 조정합니다.
  updateThrottleAdjust(combo12, combo34);

  // 조합 버튼이 아닐 때만 개별 트림 버튼을 처리합니다.
  updateTrimButtons(combo12, combo34, combo14);

  int steerUs = 0;
  int elevUs = 0;
  int throttleLevel = baseThrottleLevel;

  // 좌우 방향 버튼은 조향 입력입니다.
  if (btnLeft && !btnRight) {
    steerUs = -MAX_STEER_US;
  } else if (btnRight && !btnLeft) {
    steerUs = MAX_STEER_US;
  }

  // 상하 방향 버튼은 상승/하강 입력과 임시 스로틀 보정을 함께 적용합니다.
  if (btnUp && !btnDown) {
    throttleLevel = baseThrottleLevel + THROTTLE_UP_BOOST;
    elevUs = MAX_ELEV_US;
  } else if (btnDown && !btnUp) {
    throttleLevel = baseThrottleLevel - THROTTLE_DOWN_DROP;
    elevUs = -MAX_ELEV_US;
  } else {
    throttleLevel = baseThrottleLevel;
    elevUs = 0;
  }

  throttleLevel = constrain(throttleLevel, THROTTLE_MIN, THROTTLE_MAX);

  updateFlapping(throttleLevel);

  /*
    서보 믹싱:

    left  = center + flap + steer + elevator
    right = center - flap + steer - elevator

    trimSteerUs:
      좌우 서보에 같은 방향으로 더해져 좌우 치우침을 보정합니다.

    trimElevUs:
      좌우 서보에 반대 방향으로 더해져 상승/하강 쪽 수평을 보정합니다.
  */

  int finalSteer = steerUs + trimSteerUs;
  int finalElev  = elevUs  + trimElevUs;

  int leftUs  = SERVO_CENTER_US + (int)currentOffsetUs + finalSteer + finalElev;
  int rightUs = SERVO_CENTER_US - (int)currentOffsetUs + finalSteer - finalElev;
  
  writeServosUs(leftUs, rightUs);

  logStatus(throttleLevel, steerUs, elevUs, leftUs, rightUs);
}

// =====================================================
// Bluefruit 패킷 파서
// =====================================================

void parseBluefruitPacket(const uint8_t* data, size_t len) {
  /*
    Bluefruit Control Pad 패킷 형식:

    data[0] = '!'
    data[1] = 'B'
    data[2] = button id
    data[3] = state: '1' pressed, '0' released
    data[4] = checksum, 보통 존재함

    버튼 ID:
    '1' = Button 1
    '2' = Button 2
    '3' = Button 3
    '4' = Button 4
    '5' = Up
    '6' = Down
    '7' = Left
    '8' = Right
  */

  if (len >= 4 && data[0] == '!' && data[1] == 'B') {
    char buttonId = (char)data[2];
    char stateChar = (char)data[3];

    bool pressed = (stateChar == '1');

    updateButtonState(buttonId, pressed);

    Serial.print("Bluefruit: ");
    Serial.print(buttonName(buttonId));
    Serial.print(" ");
    Serial.println(pressed ? "PRESSED" : "RELEASED");

    String reply = "OK ";
    reply += buttonName(buttonId);
    reply += pressed ? " PRESSED\n" : " RELEASED\n";
    sendToApp(reply);

    return;
  }

  // Control Pad 패킷이 아닌 UART 텍스트나 원시 입력은 로그만 남깁니다.
  Serial.print("UART text/raw: ");
  for (size_t i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println();

  sendToApp("Text/raw received\n");
}

// =====================================================
// 버튼 처리
// =====================================================

void updateButtonState(char buttonId, bool pressed) {
  switch (buttonId) {
    case '1': btn1     = pressed; break;
    case '2': btn2     = pressed; break;
    case '3': btn3     = pressed; break;
    case '4': btn4     = pressed; break;
    case '5': btnUp    = pressed; break;
    case '6': btnDown  = pressed; break;
    case '7': btnLeft  = pressed; break;
    case '8': btnRight = pressed; break;
  }
}

String buttonName(char buttonId) {
  switch (buttonId) {
    case '1': return "BUTTON_1";
    case '2': return "BUTTON_2";
    case '3': return "BUTTON_3";
    case '4': return "BUTTON_4";
    case '5': return "UP";
    case '6': return "DOWN";
    case '7': return "LEFT";
    case '8': return "RIGHT";
    default: {
      String s = "UNKNOWN_";
      s += buttonId;
      return s;
    }
  }
}

void savePreviousStates() {
  prevCombo14 = btn1 && btn4;
}

bool risingEdge(bool current, bool previous) {
  return current && !previous;
}

// =====================================================
// 스로틀 조정
// =====================================================

void updateThrottleAdjust(bool combo12, bool combo34) {
  if (millis() - lastThrottleAdjMs < 180) {
    return;
  }

  bool changed = false;

  // Button 1 + Button 2: 기본 스로틀 증가
  if (combo12) {
    baseThrottleLevel += THROTTLE_STEP;
    changed = true;
  }

  // Button 3 + Button 4: 기본 스로틀 감소
  if (combo34) {
    baseThrottleLevel -= THROTTLE_STEP;
    changed = true;
  }

  baseThrottleLevel = constrain(baseThrottleLevel, THROTTLE_MIN, THROTTLE_MAX);

  if (changed) {
    lastThrottleAdjMs = millis();

    Serial.print("[BASE THROTTLE] ");
    Serial.println(baseThrottleLevel);

    String msg = "[BASE THROTTLE] ";
    msg += baseThrottleLevel;
    msg += "\n";
    sendToApp(msg);
  }
}

// =====================================================
// 트림
// =====================================================

void updateTrimButtons(bool combo12, bool combo34, bool combo14) {
  if (millis() - lastTrimMs < 120) {
    return;
  }

  // 조합 버튼을 누르는 동안에는 개별 트림 입력을 막습니다.
  if (combo12 || combo34 || combo14) {
    return;
  }

  bool changed = false;

  // Button 1: 왼쪽 방향으로 치우치게 보정
  if (btn1) {
    trimSteerUs -= TRIM_STEP_US;
    changed = true;
  }

  // Button 2: 오른쪽 방향으로 치우치게 보정
  if (btn2) {
    trimSteerUs += TRIM_STEP_US;
    changed = true;
  }

  // Button 3: 상승 방향 보정
  if (btn3) {
    trimElevUs += TRIM_STEP_US;
    changed = true;
  }

  // Button 4: 하강 방향 보정
  if (btn4) {
    trimElevUs -= TRIM_STEP_US;
    changed = true;
  }

  trimSteerUs = constrain(trimSteerUs, -TRIM_LIMIT_US, TRIM_LIMIT_US);
  trimElevUs  = constrain(trimElevUs,  -TRIM_LIMIT_US, TRIM_LIMIT_US);

  if (changed) {
    lastTrimMs = millis();

    Serial.print("[TRIM] steer=");
    Serial.print(trimSteerUs);
    Serial.print("us, elev=");
    Serial.print(trimElevUs);
    Serial.println("us");

    String msg = "[TRIM] steer=";
    msg += trimSteerUs;
    msg += "us elev=";
    msg += trimElevUs;
    msg += "us\n";
    sendToApp(msg);
  }
}

// =====================================================
// 날갯짓 계산
// =====================================================

void updateFlapping(int throttleLevel) {
  if (throttleLevel <= 0) {
    currentOffsetUs = approach(currentOffsetUs, 0, 18);
    return;
  }

  // throttleLevel이 높을수록 사인파 위상 진행 속도가 빨라집니다.
  float speedFactor = map(throttleLevel, 0, 100, 60, 100) / 1000.0;

  phase += speedFactor;

  if (phase >= TWO_PI) {
    phase -= TWO_PI;
  }

  currentOffsetUs = sin(phase) * flapAmplitudeUs;
}

// =====================================================
// 서보 출력
// =====================================================

void forceNeutralServos() {
  servoLeft.writeMicroseconds(SERVO_CENTER_US);
  servoRight.writeMicroseconds(SERVO_CENTER_US);

  lastLeftUs  = SERVO_CENTER_US;
  lastRightUs = SERVO_CENTER_US;
}

void writeServosUs(int leftUs, int rightUs) {
  writeServosUs(leftUs, rightUs, false);
}

void writeServosUs(int leftUs, int rightUs, bool force) {
  leftUs  = constrain(leftUs,  SERVO_MIN_US, SERVO_MAX_US);
  rightUs = constrain(rightUs, SERVO_MIN_US, SERVO_MAX_US);

  if (!force) {
    leftUs  = limitStep(lastLeftUs,  leftUs,  MAX_SERVO_STEP_US);
    rightUs = limitStep(lastRightUs, rightUs, MAX_SERVO_STEP_US);
  }

  if (force || abs(leftUs - lastLeftUs) >= 2) {
    servoLeft.writeMicroseconds(leftUs);
    lastLeftUs = leftUs;
  }

  if (force || abs(rightUs - lastRightUs) >= 2) {
    servoRight.writeMicroseconds(rightUs);
    lastRightUs = rightUs;
  }
}

int limitStep(int current, int target, int maxStep) {
  if (target > current + maxStep) {
    return current + maxStep;
  }

  if (target < current - maxStep) {
    return current - maxStep;
  }

  return target;
}

// =====================================================
// LED
// =====================================================

void ledOn() {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
}

void blinkLed(unsigned long intervalMs) {
  if (millis() - lastBlinkMs >= intervalMs) {
    lastBlinkMs = millis();
    blinkState = !blinkState;

    if (blinkState) {
      ledOn();
    } else {
      ledOff();
    }
  }
}

// =====================================================
// 유틸리티
// =====================================================

float approach(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target) {
      current = target;
    }
  } else if (current > target) {
    current -= step;
    if (current < target) {
      current = target;
    }
  }

  return current;
}

void sendToApp(const String& msg) {
  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    pTxCharacteristic->notify();
  }
}

void logStatus(int throttleLevel, int steerUs, int elevUs, int targetLeftUs, int targetRightUs) {
  if (millis() - lastLogMs < 80) {
    return;
  }

  lastLogMs = millis();

  Serial.print("[RUN] throttle=");
  Serial.print(throttleLevel);

  Serial.print(" baseThrottle=");
  Serial.print(baseThrottleLevel);

  Serial.print(" steer=");
  Serial.print(steerUs);

  Serial.print(" elev=");
  Serial.print(elevUs);

  Serial.print(" trimSteer=");
  Serial.print(trimSteerUs);

  Serial.print(" trimElev=");
  Serial.print(trimElevUs);

  Serial.print(" targetL=");
  Serial.print(targetLeftUs);

  Serial.print(" targetR=");
  Serial.print(targetRightUs);

  Serial.print(" actualL=");
  Serial.print(lastLeftUs);

  Serial.print(" actualR=");
  Serial.print(lastRightUs);

  Serial.print(" diffL=");
  Serial.print(targetLeftUs - lastLeftUs);

  Serial.print(" diffR=");
  Serial.print(targetRightUs - lastRightUs);

  Serial.print(" offsetTarget=");
  Serial.println(currentOffsetUs);
}