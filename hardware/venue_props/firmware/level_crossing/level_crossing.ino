// 차단바 (Arduino Mega 2560, 박스 B1)
//
// 시뮬의 sim_level_crossing 과 같은 동작:
//   열림 상태에서 1번 센서가 로봇을 보면 바를 내리고(닫힘), CLOSED_MS 뒤 올린다.
//   내려가 있는 동안 2번 센서가 로봇을 보면 위반(미션 실패) — 부저와 시리얼로 알린다.
//   바가 올라간 뒤 1번 센서가 REARM_CLEAR_MS 동안 비어 있어야 다음 로봇을 받는다
//   (로봇이 센서 앞에 서 있다가 다시 닫히는 것을 막는다).
//
// 배선 (README "차단바 조립과 배선"):
//   D9  서보 신호(주황), 5 V, GND          MG90S
//   D2  1번 센서 OUT  INPUT_PULLUP, 감지 시 LOW  (E18-D80NK 검정선 / 수광 모듈 OUT)
//   D3  2번 센서 OUT  같음
//   D7  레이저 VCC    차광식(C8)일 때만. 반사식(C7)이면 비워 둔다
//   D8  부저 (선택)   능동 부저, HIGH 로 울림
//   D13 내장 LED      바 내려감 표시
//
// 서보 각도: 0° = 닫힘(바 수평), 90° = 열림(바 수직). 시뮬 ANGLE_CLOSED/OPEN 과 같다.
// 혼을 끼울 때는 시리얼로 c 를 보내 서보를 0° 에 세운 뒤 바가 수평이 되게 끼운다.
// 바가 반대쪽(도로 쪽)으로 올라가면 ANGLE_OPEN 을 -90 쪽, 즉 서보를 180° 로 잡게
// ANGLE_CLOSED = 180, ANGLE_OPEN = 90 으로 바꾼다.
//
// 시리얼 (115200): d 내림, u 올림, c 혼 조립 위치(0°, 센서 무시), a 자동 복귀,
//                  r 위반 지움, s 상태.

#include <Servo.h>

const uint8_t PIN_SERVO = 9;
const uint8_t PIN_SENSOR1 = 2;
const uint8_t PIN_SENSOR2 = 3;
const uint8_t PIN_LASER = 7;
const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_LED = 13;

const int ANGLE_CLOSED = 0;
const int ANGLE_OPEN = 90;
const unsigned long CLOSED_MS = 10000;      // sim: closed_duration 10.0
const unsigned long REARM_CLEAR_MS = 2000;  // 1번 센서가 이만큼 비어야 다시 무장
const unsigned long DEBOUNCE_MS = 30;       // 센서 LOW 가 이만큼 이어져야 감지
const unsigned long SERVO_STEP_MS = 15;     // 1° 마다 — 90° 에 약 1.4 s, 바가 튀지 않게
const unsigned long BUZZ_MS = 1500;

enum State { OPEN, CLOSED, MANUAL };
State state = OPEN;
bool armed = true;
bool violation = false;
unsigned long closedAt = 0;
unsigned long sensor1ClearSince = 0;
unsigned long buzzUntil = 0;

Servo servo;
int target = ANGLE_OPEN;
int angle = ANGLE_OPEN;
unsigned long lastStep = 0;

// 센서: 디바운스된 "로봇이 보인다"
struct Sensor {
  uint8_t pin;
  unsigned long lowSince;
  bool active;
  bool read(unsigned long now) {
    if (digitalRead(pin) == LOW) {
      if (lowSince == 0) lowSince = now;
      active = (now - lowSince >= DEBOUNCE_MS);
    } else {
      lowSince = 0;
      active = false;
    }
    return active;
  }
};
Sensor s1 = {PIN_SENSOR1, 0, false};
Sensor s2 = {PIN_SENSOR2, 0, false};

void moveServo(unsigned long now) {
  if (angle == target || now - lastStep < SERVO_STEP_MS) return;
  lastStep = now;
  angle += (target > angle) ? 1 : -1;
  servo.write(angle);
}

void setBar(int a) { target = a; }

void setClosed(unsigned long now, const __FlashStringHelper* why) {
  state = CLOSED;
  closedAt = now;
  setBar(ANGLE_CLOSED);
  digitalWrite(PIN_LED, HIGH);
  Serial.print(F("Closing the bar: ")); Serial.println(why);
}

void setOpen(unsigned long now, const __FlashStringHelper* why) {
  state = OPEN;
  armed = false;              // 1번이 비워진 뒤에 다시 무장
  sensor1ClearSince = now;
  setBar(ANGLE_OPEN);
  digitalWrite(PIN_LED, LOW);
  Serial.print(F("Opening the bar: ")); Serial.println(why);
}

void printStatus(unsigned long now) {
  Serial.print(F("state="));
  Serial.print(state == OPEN ? "open" : state == CLOSED ? "closed" : "manual");
  Serial.print(F(" angle=")); Serial.print(angle);
  Serial.print(F(" armed=")); Serial.print(armed);
  Serial.print(F(" s1=")); Serial.print(s1.active);
  Serial.print(F(" s2=")); Serial.print(s2.active);
  Serial.print(F(" violation=")); Serial.print(violation);
  if (state == CLOSED) {
    Serial.print(F(" opens_in_ms="));
    Serial.print((long)(CLOSED_MS - (now - closedAt)));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SENSOR1, INPUT_PULLUP);
  pinMode(PIN_SENSOR2, INPUT_PULLUP);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LASER, HIGH);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED, LOW);
  servo.attach(PIN_SERVO);
  servo.write(angle);
  Serial.println(F("level_crossing: d down, u up, c horn position, a auto, r clear violation, s status"));
}

void loop() {
  unsigned long now = millis();
  bool see1 = s1.read(now);
  bool see2 = s2.read(now);

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'd') { state = MANUAL; setBar(ANGLE_CLOSED); digitalWrite(PIN_LED, HIGH); Serial.println(F("manual: down")); }
    else if (c == 'u') { state = MANUAL; setBar(ANGLE_OPEN); digitalWrite(PIN_LED, LOW); Serial.println(F("manual: up")); }
    else if (c == 'c') { state = MANUAL; setBar(ANGLE_CLOSED); Serial.println(F("manual: horn position (0 deg, bar horizontal)")); }
    else if (c == 'a') { setOpen(now, F("auto")); }
    else if (c == 'r') { violation = false; Serial.println(F("violation cleared")); }
    else if (c == 's') { printStatus(now); }
  }

  if (state == OPEN) {
    if (!armed) {
      if (see1) sensor1ClearSince = now;
      else if (now - sensor1ClearSince >= REARM_CLEAR_MS) {
        armed = true;
        violation = false;    // 시뮬처럼 다음 로봇을 위해 지운다
        Serial.println(F("Sensor 1 clear, armed for the next robot."));
      }
    } else if (see1) {
      setClosed(now, F("sensor 1 triggered"));
    }
  } else if (state == CLOSED) {
    if (!violation && see2 && angle <= ANGLE_CLOSED + 10) {  // 바가 거의 다 내려온 뒤부터
      violation = true;
      buzzUntil = now + BUZZ_MS;
      Serial.println(F("Sensor 2 crossed while the bar is down: mission failed."));
    }
    if (now - closedAt >= CLOSED_MS) {
      setOpen(now, F("time is up"));
    }
  }

  digitalWrite(PIN_BUZZER, (long)(buzzUntil - now) > 0 ? HIGH : LOW);
  moveServo(now);
}
