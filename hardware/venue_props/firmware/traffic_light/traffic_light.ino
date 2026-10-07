// 신호등 (Arduino Mega 2560, 박스 B1)
//
// 시뮬의 sim_traffic_light 와 같은 순서·시간으로 램프를 돌린다:
// 적 5 s → 황 2 s → 녹 5 s → 적 ... (전원을 켠 뒤 START_DELAY 만큼은 꺼짐).
//
// 배선 (README "신호등 조립과 배선"):
//   D9  적 LED (+)  150 Ω 직렬
//   D10 황 LED (+)  150 Ω
//   D11 녹 LED (+)  100 Ω
//   GND LED (−) 공통
// 램프마다 WS2812 를 넣었다면 USE_WS2812 를 1 로 하고 D6 에 데이터선을 단다
// (Adafruit NeoPixel 라이브러리 필요, 픽셀 순서 적-황-녹, 아래부터 위로 배선).
//
// 시리얼 (115200): r / y / g 로 램프를 강제, a 로 자동 순환 복귀, s 상태 출력.

#define USE_WS2812 0

const unsigned long START_DELAY = 5000;  // ms, sim: start_delay 5.0
const unsigned long DURATION_MS[3] = {5000, 2000, 5000};  // 적, 황, 녹. sim: duration_*
const char* NAMES[3] = {"red", "yellow", "green"};

const uint8_t PIN_LED[3] = {9, 10, 11};  // 적, 황, 녹
const bool LED_ACTIVE_HIGH = true;        // 트랜지스터/ULN2003 로 끌면 배선에 따라 바꾼다

#if USE_WS2812
#include <Adafruit_NeoPixel.h>
const uint8_t PIN_PIXELS = 6;
const uint8_t PIXELS_PER_LAMP = 3;
Adafruit_NeoPixel pixels(3 * PIXELS_PER_LAMP, PIN_PIXELS, NEO_GRB + NEO_KHZ800);
const uint32_t LAMP_COLOR[3] = {0xFF0000, 0xFFA000, 0x00FF00};
#endif

int current = -1;              // 켜진 램프 (-1: 모두 꺼짐)
int manual = -1;               // 시리얼로 강제한 램프 (-1: 자동)
unsigned long nextSwitch = 0;

void show(int lamp) {
  for (int i = 0; i < 3; i++) {
    bool on = (i == lamp);
    digitalWrite(PIN_LED[i], on == LED_ACTIVE_HIGH ? HIGH : LOW);
  }
#if USE_WS2812
  pixels.clear();
  if (lamp >= 0) {
    for (int p = 0; p < PIXELS_PER_LAMP; p++) {
      pixels.setPixelColor(lamp * PIXELS_PER_LAMP + p, LAMP_COLOR[lamp]);
    }
  }
  pixels.show();
#endif
  if (lamp != current) {
    current = lamp;
    Serial.print(F("Traffic light: "));
    Serial.println(lamp < 0 ? "off" : NAMES[lamp]);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) pinMode(PIN_LED[i], OUTPUT);
#if USE_WS2812
  pixels.begin();
#endif
  show(-1);
  nextSwitch = millis() + START_DELAY;
  Serial.println(F("traffic_light: r/y/g force a lamp, a = auto, s = status"));
}

void loop() {
  unsigned long now = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') manual = 0;
    else if (c == 'y') manual = 1;
    else if (c == 'g') manual = 2;
    else if (c == 'a') { manual = -1; nextSwitch = now; current = -1; }
    else if (c == 's') {
      Serial.print(F("lamp=")); Serial.print(current < 0 ? "off" : NAMES[current]);
      Serial.print(F(" manual=")); Serial.println(manual >= 0);
    }
  }

  if (manual >= 0) {
    show(manual);
    return;
  }

  if ((long)(now - nextSwitch) >= 0) {
    int next = (current < 0) ? 0 : (current + 1) % 3;  // 꺼짐 → 적부터
    show(next);
    nextSwitch = now + DURATION_MS[next];
  }
}
