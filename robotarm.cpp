#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 서보 각도 → PWM 값 매핑 범위 (모터 따라 조절 필요)
#define SERVO_MIN 100
#define SERVO_MAX 500

// 서보 각도 설정 함수
void setServoAngle(uint8_t ch, float angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(ch, 0, pulse);
}

void setup() {
  Wire.begin(21, 22);      // SDA = GPIO21, SCL = GPIO22
  pwm.begin();
  pwm.setPWMFreq(50);      // 서보는 50Hz 주파수 사용

  // 각 축 초기 위치 설정 (90도)
  setServoAngle(3, 90);    // Base
  setServoAngle(7, 90);    // Shoulder
  setServoAngle(11, 90);   // Elbow
  setServoAngle(15, 90);   // Wrist

  delay(1000);  // 1초 대기
}

void loop() {
  // Base 축만 좌우로 60~120도 반복 움직임
  for (int angle = 60; angle <= 120; angle += 5) {
    setServoAngle(3, angle);
    delay(100);
  }
  for (int angle = 120; angle >= 60; angle -= 5) {
    setServoAngle(3, angle);
    delay(100);
  }
}
