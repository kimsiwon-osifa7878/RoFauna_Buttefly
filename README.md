# RoFauna_Buttefly

ESP32-C3 SuperMini와 Adafruit Bluefruit Connect 앱으로 좌우 2개의 서보를 제어하는 바이오닉 나비 프로젝트입니다.

현재 코드는 별도 PPM 수신기, Dabble, NimBLE-Arduino를 사용하지 않습니다. Bluefruit Connect의 BLE UART Control Pad 패킷을 ESP32 Arduino 내장 BLE 라이브러리로 받아 서보 PWM 펄스 폭을 직접 계산합니다.

> 저장소와 BLE 장치명은 현재 코드 기준으로 `RoFauna_Buttefly`입니다. `Butterfly`가 아니라 `Buttefly`로 표기되어 있으므로 앱에서 검색할 때 이 이름을 그대로 사용합니다.

## 프로젝트 구성

```text
.
├─ README.md
├─ LICENSE
└─ src/
   └─ Butterfly_esp32c3_supermini/
      └─ Butterfly_esp32c3_supermini.ino
```

## 하드웨어

| 항목 | 설정 |
|---|---|
| 보드 | ESP32-C3 SuperMini |
| 왼쪽 서보 신호선 | GPIO0 |
| 오른쪽 서보 신호선 | GPIO1 |
| 상태 LED | GPIO8 기본값 |
| BLE 장치명 | `RoFauna_Buttefly` |
| 조종 앱 | Adafruit Bluefruit Connect |
| 앱 화면 | Controller -> Control Pad |
| 서보 출력 | 50 Hz, `writeMicroseconds()` |
| 서보 펄스 범위 | 600 us ~ 2400 us |
| 서보 중심 | 1500 us |

서보 전원은 ESP32-C3 보드에서 직접 공급하지 않는 것을 권장합니다. 별도 BEC, DC-DC 컨버터, 또는 배터리 전원을 사용하고, ESP32-C3 GND와 서보 전원 GND는 반드시 공통으로 연결해야 합니다.

ESP32-C3 SuperMini 보드마다 내장 LED 핀이 다를 수 있습니다. LED가 동작하지 않으면 코드의 `LED_PIN` 값을 8, 2, 10 등 실제 보드 LED 핀에 맞게 바꾸세요. LED 동작이 반대로 보이면 `LED_ACTIVE_LOW`를 `true`로 변경합니다.

## 소프트웨어 요구사항

- Arduino IDE 또는 Arduino CLI
- ESP32 Arduino board package
- `ESP32Servo` 라이브러리
- ESP32 Arduino 내장 BLE 라이브러리
- Adafruit Bluefruit Connect 모바일 앱

스케치는 다음 헤더를 사용합니다.

```cpp
#include <Arduino.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
```

## 업로드와 실행

1. Arduino IDE에서 ESP32-C3 계열 보드를 선택합니다.
2. `src/Butterfly_esp32c3_supermini/Butterfly_esp32c3_supermini.ino`를 엽니다.
3. `ESP32Servo` 라이브러리가 설치되어 있는지 확인합니다.
4. 스케치를 ESP32-C3 SuperMini에 업로드합니다.
5. 서보 전원과 ESP32-C3 전원을 공급합니다.
6. Bluefruit Connect 앱을 실행하고 BLE 목록에서 `RoFauna_Buttefly`에 연결합니다.
7. 앱에서 `Controller -> Control Pad`로 들어갑니다.
8. `Button 1 + Button 4`를 동시에 눌러 조종 모드로 전환합니다.
9. 다시 `Button 1 + Button 4`를 동시에 누르면 대기 모드로 돌아갑니다.

## 동작 모드

### 대기 모드

전원 ON 직후 기본 상태입니다.

- `armed = false`
- 좌우 서보를 1500 us 중심값으로 유지
- 250 ms마다 `forceNeutralServos()`로 중심값 재출력
- LED 300 ms 간격 점멸
- 조종 입력은 ARM 토글 조합을 제외하고 실제 서보 조종에 반영되지 않음
- `Button 1 + Button 4` 상승 에지에서 조종 모드 진입

BLE 연결이 끊기면 코드가 자동으로 대기 모드로 돌아가고, 트림과 날갯짓 오프셋을 초기화한 뒤 BLE 광고를 다시 시작합니다.

### 조종 모드

`Button 1 + Button 4`로 진입하는 활성 상태입니다.

- `armed = true`
- LED 켜짐
- 사인파 기반 날갯짓 계산
- 방향 버튼으로 조향, 상승, 하강 제어
- 1~4번 버튼으로 트림 조정
- 버튼 조합으로 기본 스로틀 조정
- `Button 1 + Button 4` 상승 에지에서 대기 모드 복귀

## Bluefruit Control Pad 매핑

Bluefruit Control Pad는 `!B`로 시작하는 버튼 패킷을 보냅니다. 코드는 `data[2]`의 버튼 ID와 `data[3]`의 눌림 상태를 읽습니다.

| Bluefruit 입력 | ID | 조종 모드 기능 |
|---|---:|---|
| Button 1 | `1` | 왼쪽 방향 트림 |
| Button 2 | `2` | 오른쪽 방향 트림 |
| Button 3 | `3` | 상승 방향 트림 |
| Button 4 | `4` | 하강 방향 트림 |
| Up | `5` | 상승 자세, 스로틀 증가 |
| Down | `6` | 하강 자세, 스로틀 감소 |
| Left | `7` | 좌회전 |
| Right | `8` | 우회전 |

조합 버튼은 개별 트림과 충돌하지 않도록 별도로 처리합니다.

| 입력 | 기능 |
|---|---|
| Button 1 + Button 4 | 대기 모드 / 조종 모드 토글 |
| Button 1 + Button 2 | 기본 스로틀 증가 |
| Button 3 + Button 4 | 기본 스로틀 감소 |

## 서보 믹싱

모든 서보 출력 계산은 마이크로초 단위입니다.

```cpp
leftUs  = SERVO_CENTER_US + currentOffsetUs + finalSteer + finalElev;
rightUs = SERVO_CENTER_US - currentOffsetUs + finalSteer - finalElev;
```

| 값 | 의미 |
|---|---|
| `SERVO_CENTER_US` | 서보 중심값, 기본 1500 us |
| `currentOffsetUs` | 사인파 날갯짓 오프셋 |
| `finalSteer` | 좌우 조향 입력 + 좌우 트림 |
| `finalElev` | 상승/하강 입력 + 상하 트림 |

기본 구조는 다음과 같습니다.

- 날갯짓은 좌우 서보가 서로 반대 방향으로 움직이게 합니다.
- 조향은 좌우 서보에 같은 방향 오프셋을 더합니다.
- 상승/하강은 좌우 서보에 반대 방향 오프셋을 더합니다.
- 최종 출력은 600 us ~ 2400 us로 제한됩니다.
- 한 루프에서 서보 명령이 너무 크게 변하지 않도록 `MAX_SERVO_STEP_US`로 변화량을 제한합니다.

## 주요 조정값

스케치 상단의 값들을 먼저 조정합니다.

```cpp
const int SERVO_CENTER_US = 1500;
const int SERVO_MIN_US    = 500;
const int SERVO_MAX_US    = 2500;

int flapAmplitudeUs = 660;
const int MAX_SERVO_STEP_US = 100;

const int MAX_STEER_US = 160;
const int MAX_ELEV_US  = 160;

const int BASE_THROTTLE_DEFAULT = 55;
const int THROTTLE_MIN = 35;
const int THROTTLE_MAX = 100;
```

### 날갯짓 진폭

```cpp
int flapAmplitudeUs = 660;
```

서보 중심 1500 us 기준으로 좌우로 움직이는 최대 펄스 폭입니다. 500 us ~ 2500 us를 180도로 보는 단순 환산에서는 약 11.1 us가 1도에 해당하므로, 660 us는 약 59도 수준입니다. 실제 각도는 서보 모델, 링크 구조, 혼 길이에 따라 달라집니다.

### 기본 스로틀

```cpp
const int BASE_THROTTLE_DEFAULT = 65;
```

기본 날갯짓 속도입니다. 조종 모드에서 `Button 1 + Button 2`로 올리고 `Button 3 + Button 4`로 내릴 수 있습니다. 값은 `THROTTLE_MIN` 35부터 `THROTTLE_MAX` 100 사이로 제한됩니다.

Up 입력 중에는 기본 스로틀에 `THROTTLE_UP_BOOST` 35가 더해지고, Down 입력 중에는 `THROTTLE_DOWN_DROP` 30이 빠집니다.

### 날갯짓 속도 계산

현재 코드는 루프마다 스로틀을 사인파 위상 증가량으로 바꿉니다.

```cpp
float speedFactor = map(throttleLevel, 0, 100, 60, 100) / 1000.0;
phase += speedFactor;
currentOffsetUs = sin(phase) * flapAmplitudeUs;
```

메인 루프 끝에는 현재 `delay(8)`이 있습니다. 따라서 루프 주기가 바뀌면 실제 날갯짓 주파수도 함께 바뀝니다. `delay`, `speedFactor`, `MAX_SERVO_STEP_US`, `flapAmplitudeUs`는 함께 확인해야 합니다.

### 서보 변화량 제한

```cpp
const int MAX_SERVO_STEP_US = 70;
```

한 루프에서 명령 펄스 폭이 최대 몇 us까지 변할 수 있는지 제한합니다. 값이 너무 작으면 목표 진폭까지 도달하기 전에 사인파 방향이 바뀌어 실제 날갯짓이 작아질 수 있습니다. 값이 너무 크면 서보와 링크에 순간 부하가 커질 수 있습니다.

## 시리얼 로그

시리얼 모니터는 115200 baud로 설정합니다.

조종 모드에서는 약 800 ms마다 다음 항목을 출력합니다.

```text
[RUN] throttle=... baseThrottle=... steer=... elev=... trimSteer=... trimElev=... L=... R=...
offset=...
```

BLE 버튼 입력, ARM/DISARM 상태, 기본 스로틀 변경, 트림 변경도 시리얼과 Bluefruit UART 알림으로 출력합니다.

## 안전 테스트 순서

처음에는 날개 링크를 분리하고 서보만 테스트하세요.

1. ESP32-C3와 서보 전원을 켭니다.
2. 좌우 서보가 대기 모드에서 1500 us 중심으로 가는지 확인합니다.
3. Bluefruit Connect에서 `RoFauna_Buttefly`에 연결합니다.
4. Control Pad에서 `Button 1 + Button 4`로 조종 모드에 진입합니다.
5. 낮은 부하 상태에서 좌우 서보 방향과 진폭을 확인합니다.
6. 링크를 연결한 뒤 진폭과 스로틀을 단계적으로 올립니다.
7. 서보 발열, 소음, 전원 전압 강하, 링크 간섭을 확인합니다.

## 문제 해결

### BLE 장치가 보이지 않음

- 시리얼 모니터에서 `Advertising as RoFauna_Buttefly`가 출력되는지 확인합니다.
- 앱에서 장치명을 `RoFauna_Buttefly`로 찾습니다.
- 업로드 후 보드를 리셋합니다.
- 다른 BLE 앱이나 이전 연결이 장치를 잡고 있지 않은지 확인합니다.

### 서보가 움직이지 않음

- `ESP32Servo` 라이브러리가 설치되어 있는지 확인합니다.
- 왼쪽 신호선은 GPIO0, 오른쪽 신호선은 GPIO1에 연결했는지 확인합니다.
- 서보 전원이 별도로 공급되는지 확인합니다.
- ESP32-C3 GND와 서보 전원 GND가 공통인지 확인합니다.
- 대기 모드에서는 서보가 1500 us 중심에 고정되는 것이 정상입니다.

### 대기 모드에서 서보가 중심으로 가지 않음

- `forceNeutralServos()`가 호출되는지 시리얼 로그 흐름을 확인합니다.
- 서보 중심이 실제 기구 중심과 다를 수 있으므로 혼 조립 위치를 확인합니다.
- 서보 전원 전압이 부족하거나 순간 전류가 부족하지 않은지 확인합니다.

### 날갯짓이 너무 작음

- `flapAmplitudeUs`를 단계적으로 올립니다.
- `MAX_SERVO_STEP_US`가 너무 작지 않은지 확인합니다.
- `BASE_THROTTLE_DEFAULT` 또는 버튼 조합으로 기본 스로틀을 올립니다.
- 링크가 걸리거나 서보 토크가 부족하지 않은지 확인합니다.

### 날갯짓이 거칠거나 서보에 무리가 감

- `flapAmplitudeUs`를 낮춥니다.
- `BASE_THROTTLE_DEFAULT`를 낮춥니다.
- `MAX_SERVO_STEP_US`를 낮춰 명령 변화를 완만하게 만듭니다.
- 링크 간섭, 혼 길이, 전원 전류 용량을 확인합니다.

## 개발 메모

- BLE는 Nordic UART Service UUID를 사용해 Bluefruit Connect UART와 호환됩니다.
- RX 특성은 앱에서 ESP32로 들어오는 버튼 패킷을 받습니다.
- TX 특성은 ESP32에서 앱으로 상태 메시지를 notify합니다.
- BLE 연결 해제 시 자동 DISARM 처리합니다.
- 현재 트림 값은 메모리에만 유지되며 전원 재시작 시 초기화됩니다.
- 모든 서보 계산은 degree가 아니라 microseconds 기준입니다.

## 라이선스

MIT License
