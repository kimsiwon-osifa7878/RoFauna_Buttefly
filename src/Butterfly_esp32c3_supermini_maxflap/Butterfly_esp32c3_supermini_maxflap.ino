/*
  RoFauna_Buttefly
  ESP32-C3 SuperMini + Bluefruit Connect + 2 Servo Bionic Butterfly

  보드:
  - ESP32-C3 SuperMini
  - 왼쪽 서보 신호  : GPIO21
  - 오른쪽 서보 신호: GPIO0
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

      UP          : 상승
      DOWN        : 하강
      LEFT        : 좌회전
      RIGHT       : 우회전
      Button 1    : 왼쪽 방향 트림
      Button 2    : 오른쪽 방향 트림
      Button 3    : 상승 방향 트림
      Button 4    : 하강 방향 트림

      Button 1 + Button 2 : 기본 주파수 증가
      Button 3 + Button 4 : 기본 주파수 감소
      Button 1 + Button 3 : 기본 진폭 감소
      Button 2 + Button 4 : 기본 진폭 증가
      Button 1 + Button 3 : 기본 진폭 감소 110us씩
      Button 2 + Button 4 : 기본 진폭 증가 110us씩

  참고:
  - Dabble 미사용
  - NimBLE-Arduino 미사용
  - ESP32 Arduino 내장 BLE 라이브러리 사용
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>

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
Preferences prefs;

bool deviceConnected = false;
bool oldDeviceConnected = false;

// =====================================================
// 핀 설정
// =====================================================

const int SERVO_LEFT_PIN  = 21;
const int SERVO_RIGHT_PIN = 0;

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

// 11.1us를 약 1도로 보면 기본값 550us는 약 +-50도입니다.
const int BASE_FLAP_AMPLITUDE_US = 550;
const int FLAP_AMPLITUDE_STEP_US = 110;
const int FLAP_AMPLITUDE_MIN_US  = FLAP_AMPLITUDE_STEP_US;
const int FLAP_AMPLITUDE_MAX_US  = SERVO_CENTER_US - SERVO_MIN_US;

int baseFlapAmplitudeUs = BASE_FLAP_AMPLITUDE_US;

// 한 루프에서 서보 명령이 변할 수 있는 최대 펄스 폭입니다.
const int MAX_SERVO_STEP_US = 100;

// 조종 입력의 최대 보정량입니다.
const int MAX_STEER_US = 160;
const int MAX_ELEV_US  = 160;

// 트림 값입니다.
int trimSteerUs = 24;
int trimElevUs  = -24;

const int TRIM_STEP_US  = 12;
const int TRIM_LIMIT_US = 160;
const unsigned long TRIM_SAVE_DELAY_MS = 1000;

const char* PREFS_NAMESPACE = "rofauna";
const char* PREF_TRIM_STEER = "trimSteer";
const char* PREF_TRIM_ELEV  = "trimElev";

bool trimSavePending = false;
unsigned long lastTrimChangeMs = 0;

// 마지막으로 실제 출력한 서보 명령값입니다.
int lastLeftUs  = SERVO_CENTER_US;
int lastRightUs = SERVO_CENTER_US;

// =====================================================
// 날갯짓 주파수 설정
// Hz는 날개가 +진폭과 -진폭을 한 번 왕복하는 주기 기준입니다.
// =====================================================

const float BASE_FLAP_FREQUENCY_HZ = 2.0;
const float FLAP_FREQUENCY_MIN_HZ  = 1.0;
const float FLAP_FREQUENCY_MAX_HZ  = 3.9;
const float FLAP_FREQUENCY_STEP_HZ = 0.1;

float baseFlapFrequencyHz = BASE_FLAP_FREQUENCY_HZ;

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

float currentOffsetUs = 0.0;
int flapSign = 1;

unsigned long lastBlinkMs       = 0;
unsigned long lastTrimMs        = 0;
unsigned long lastLogMs         = 0;
unsigned long lastFrequencyAdjMs = 0;
unsigned long lastAmplitudeAdjMs = 0;
unsigned long lastNeutralMs     = 0;
unsigned long lastFlapSwitchMs  = 0;

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

void updateTrimButtons(bool combo12, bool combo34, bool combo14, bool combo13, bool combo24);
void loadTrimSettings();
void scheduleTrimSave();
void updateTrimSave();
void updateFrequencyAdjust(bool combo12, bool combo34);
void updateAmplitudeAdjust(bool combo13, bool combo24);
void updateFlapping();

void writeServosUs(int leftUs, int rightUs);
void writeServosUs(int leftUs, int rightUs, bool force);
void forceNeutralServos();
int limitStep(int current, int target, int maxStep);

void ledOn();
void ledOff();
void blinkLed(unsigned long intervalMs);

void sendToApp(const String& msg);
void logStatus(int steerUs, int elevUs, int targetLeftUs, int targetRightUs);
void savePreviousStates();

bool risingEdge(bool current, bool previous);

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

  loadTrimSettings();

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
  Serial.print("Base frequency = ");
  Serial.print(baseFlapFrequencyHz, 1);
  Serial.println("Hz");
  Serial.print("Base amplitude = ");
  Serial.print(baseFlapAmplitudeUs);
  Serial.println("us");
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

  updateTrimSave();
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
    currentOffsetUs = 0;
    flapSign = 1;
    lastFlapSwitchMs = millis();

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

    currentOffsetUs = 0;
    flapSign = 1;
    lastFlapSwitchMs = millis();

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
  bool combo13 = btn1 && btn3;
  bool combo24 = btn2 && btn4;

  // Button 1 + Button 4 동시 입력의 상승 에지에서 DISARM합니다.
  if (risingEdge(combo14, prevCombo14)) {
    armed = false;

    currentOffsetUs = 0;

    forceNeutralServos();

    Serial.println("[DISARMED] Button 1 + Button 4");
    sendToApp("[DISARMED] Back to standby\n");

    delay(250);
    return;
  }

  // 조종 모드에서 버튼 조합으로 기본 날갯짓 주파수를 조정합니다.
  updateFrequencyAdjust(combo12, combo34);

  // 조종 모드에서 버튼 조합으로 기본 진폭을 조정합니다.
  updateAmplitudeAdjust(combo13, combo24);

  // 조합 버튼이 아닐 때만 개별 트림 버튼을 처리합니다.
  updateTrimButtons(combo12, combo34, combo14, combo13, combo24);

  int steerUs = 0;
  int elevUs = 0;

  // 좌우 방향 버튼은 조향 입력입니다.
  if (btnLeft && !btnRight) {
    steerUs = -MAX_STEER_US;
  } else if (btnRight && !btnLeft) {
    steerUs = MAX_STEER_US;
  }

  // 상하 방향 버튼은 상승/하강 입력입니다.
  if (btnUp && !btnDown) {
    elevUs = MAX_ELEV_US;
  } else if (btnDown && !btnUp) {
    elevUs = -MAX_ELEV_US;
  } else {
    elevUs = 0;
  }

  updateFlapping();

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
  
  writeServosUs(leftUs, rightUs, true);

  logStatus(steerUs, elevUs, leftUs, rightUs);
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
// 날갯짓 주파수 조정
// =====================================================

void updateFrequencyAdjust(bool combo12, bool combo34) {
  if (millis() - lastFrequencyAdjMs < 180) {
    return;
  }

  bool changed = false;

  // Button 1 + Button 2: 기본 주파수 증가
  if (combo12) {
    baseFlapFrequencyHz += FLAP_FREQUENCY_STEP_HZ;
    changed = true;
  }

  // Button 3 + Button 4: 기본 주파수 감소
  if (combo34) {
    baseFlapFrequencyHz -= FLAP_FREQUENCY_STEP_HZ;
    changed = true;
  }

  baseFlapFrequencyHz = constrain(baseFlapFrequencyHz, FLAP_FREQUENCY_MIN_HZ, FLAP_FREQUENCY_MAX_HZ);

  if (changed) {
    lastFrequencyAdjMs = millis();

    Serial.print("[BASE FREQUENCY] ");
    Serial.print(baseFlapFrequencyHz, 1);
    Serial.println("Hz");

    String msg = "[BASE FREQUENCY] ";
    msg += String(baseFlapFrequencyHz, 1);
    msg += "Hz\n";
    sendToApp(msg);
  }
}

// =====================================================
// 진폭 조정
// =====================================================

void updateAmplitudeAdjust(bool combo13, bool combo24) {
  if (millis() - lastAmplitudeAdjMs < 180) {
    return;
  }

  bool changed = false;

  // Button 1 + Button 3: 기본 진폭 감소
  if (combo13) {
    baseFlapAmplitudeUs -= FLAP_AMPLITUDE_STEP_US;
    changed = true;
  }

  // Button 2 + Button 4: 기본 진폭 증가
  if (combo24) {
    baseFlapAmplitudeUs += FLAP_AMPLITUDE_STEP_US;
    changed = true;
  }

  baseFlapAmplitudeUs = constrain(baseFlapAmplitudeUs, FLAP_AMPLITUDE_MIN_US, FLAP_AMPLITUDE_MAX_US);

  if (changed) {
    lastAmplitudeAdjMs = millis();

    Serial.print("[BASE AMPLITUDE] ");
    Serial.print(baseFlapAmplitudeUs);
    Serial.println("us");

    String msg = "[BASE AMPLITUDE] ";
    msg += baseFlapAmplitudeUs;
    msg += "us\n";
    sendToApp(msg);
  }
}

// =====================================================
// 트림
// =====================================================

void updateTrimButtons(bool combo12, bool combo34, bool combo14, bool combo13, bool combo24) {
  if (millis() - lastTrimMs < 120) {
    return;
  }

  // 조합 버튼을 누르는 동안에는 개별 트림 입력을 막습니다.
  if (combo12 || combo34 || combo14 || combo13 || combo24) {
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

    scheduleTrimSave();
  }
}

void loadTrimSettings() {
  prefs.begin(PREFS_NAMESPACE, false);

  trimSteerUs = prefs.getInt(PREF_TRIM_STEER, 0);
  trimElevUs = prefs.getInt(PREF_TRIM_ELEV, 0);

  trimSteerUs = constrain(trimSteerUs, -TRIM_LIMIT_US, TRIM_LIMIT_US);
  trimElevUs  = constrain(trimElevUs,  -TRIM_LIMIT_US, TRIM_LIMIT_US);

  Serial.print("[TRIM LOAD] steer=");
  Serial.print(trimSteerUs);
  Serial.print("us elev=");
  Serial.print(trimElevUs);
  Serial.println("us");
}

void scheduleTrimSave() {
  trimSavePending = true;
  lastTrimChangeMs = millis();
}

void updateTrimSave() {
  if (!trimSavePending) {
    return;
  }

  if (millis() - lastTrimChangeMs < TRIM_SAVE_DELAY_MS) {
    return;
  }

  prefs.putInt(PREF_TRIM_STEER, trimSteerUs);
  prefs.putInt(PREF_TRIM_ELEV, trimElevUs);

  trimSavePending = false;

  Serial.print("[TRIM SAVED] steer=");
  Serial.print(trimSteerUs);
  Serial.print("us elev=");
  Serial.print(trimElevUs);
  Serial.println("us");

  String msg = "[TRIM SAVED] steer=";
  msg += trimSteerUs;
  msg += "us elev=";
  msg += trimElevUs;
  msg += "us\n";
  sendToApp(msg);
}

// =====================================================
// 날갯짓 계산
// =====================================================

void updateFlapping() {
  unsigned long now = millis();
  unsigned long halfPeriodMs = (unsigned long)(500.0 / baseFlapFrequencyHz);

  if (halfPeriodMs < 1) {
    halfPeriodMs = 1;
  }

  if (now - lastFlapSwitchMs >= halfPeriodMs) {
    unsigned long steps = (now - lastFlapSwitchMs) / halfPeriodMs;

    if ((steps % 2) == 1) {
      flapSign = -flapSign;
    }

    lastFlapSwitchMs += steps * halfPeriodMs;
  }

  currentOffsetUs = flapSign * baseFlapAmplitudeUs;
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

void sendToApp(const String& msg) {
  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    pTxCharacteristic->notify();
  }
}

void logStatus(int steerUs, int elevUs, int targetLeftUs, int targetRightUs) {
  if (millis() - lastLogMs < 80) {
    return;
  }

  lastLogMs = millis();

  Serial.print("[RUN] baseHz=");
  Serial.print(baseFlapFrequencyHz, 1);

  Serial.print(" baseAmp=");
  Serial.print(baseFlapAmplitudeUs);

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
