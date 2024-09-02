//acm1
#include "LedControl.h"

// 도트매트릭스 사용 준비
LedControl lc = LedControl(2,3,4,8);
int buzzer = 7; // 부저 출력용 디지털 IO 설정

byte arrow_up[] = {
  B00010000,
  B00111000,
  B01010100,
  B10010010,
  B00010000,
  B00010000,
  B00010000,
  B00010000
};

byte arrow_right[] = {
  B00001000,
  B00000100,
  B00000010,
  B11111111,
  B00000010,
  B00000100,
  B00001000,
  B00000000
};

byte arrow_down[] = {
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B10010010,
  B01010100,
  B00111000,
  B00010000
};

byte arrow_left[] = {
  B00010000,
  B00100000,
  B01000000,
  B11111111,
  B01000000,
  B00100000,
  B00010000,
  B00000000
};

byte arrow_all[] = {
  B00000000,
  B00000000,
  B00000000,
  B01111110,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};

byte arrow_fire[] = {
  B10000001,
  B01000010,
  B00100100,
  B00011000,
  B00011000,
  B00100100,
  B01000010,
  B10000001
};

void setup() {
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);
 
  for (int i = 0; i < 8; i++) {
    lc.shutdown(i, false);
    lc.setIntensity(i, 5);
    lc.clearDisplay(i);
  }
}

void processSerialInput() {
  char receivedChar[8];
  if (Serial.available() >= 8) {
    for (int i = 0; i < 8; ++i) {
      receivedChar[i] = Serial.read();
    }

    for (int i = 0; i < 8; i++) {
      displayArrow(i, receivedChar[i]);
    }
   
    // 데이터 수신 후 부저를 울림

      sound();
 
  }
}

void sound() {
  // 화재 경보음 패턴 생성
  for (int i = 0; i < 3; i++) {
    tone(buzzer, 1000); // 1000Hz 톤 재생
    delay(500); // 500ms 유지
    noTone(buzzer); // 톤 정지
    delay(500); // 500ms 대기
  }

  delay(1000); // 1초 대기 후 다음 경보음 패턴으로 전환
}

void displayArrow(int matrix_num, char direction) {
  byte* arrow = nullptr;
  switch (direction) {
    case 'U':
      arrow = arrow_up;
      break;
    case 'R':
      arrow = arrow_right;
      break;
    case 'D':
      arrow = arrow_down;
      break;
    case 'L':
      arrow = arrow_left;
      break;
    case 'O':
      arrow = arrow_all;
      break;
    case 'X':
      arrow = arrow_fire;
      break;
    default:
      return;
  }

  for (int i = 0; i < 8; i++) {
    lc.setRow(matrix_num, i, arrow[i]);
  }
}

void loop() {
  processSerialInput();
  delay(1000);
}
