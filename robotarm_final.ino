#include <Wire.h>  // I2C 통신을 위한 라이브러리. PCA9685 서보 드라이버와의 통신에 사용
#include <Adafruit_PWMServoDriver.h>  // PCA9685를 제어하기 위한 라이브러리
#include <avr/io.h>  // 레지스터 직접 제어를 위한 헤더
#include <util/delay.h>  // 딜레이 함수 사용

#define F_CPU 16000000UL  // MCU CLOCK 속도 16MHz 정의
#define BAUD 9600         //UART 통신 속도 9600bps 설정
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1) // UART 통신속도 설정을 위한 값 계산
#define VOICE_START PB4  //ISD1820 시작 음성 모듈 제어 핀 정의
#define VOICE_FINISH PB5 //ISD1820 종료 음성 모듈 제어 핀 정의

const int RED = 8;   //RED LED 핀 번호 정의
const int BLUE = 7; // BLUE LED 핀 번호 정의

// 각 서보모터의 현재 각도를 저장하는 변수 지정
double currentBase = 90;
double currentShoulder = 135;
double currentElbow = 90;
double currentZ = 90;

const int steps = 50;  // 이동시 50단계로 나누어 이동
const int delayBetweenSteps = 15; // 각 이동 단계 간 시간 지연

//HC-06 블루투스 설정
void uart_init() {     
  UBRR0H = (unsigned char)(UBRR_VALUE >> 8); //UBRR 레지스터의 하위 8비트, UBRR_VALUE의 하위 바이트 저장
  UBRR0L = (unsigned char)UBRR_VALUE;        //UBRR 레지스터의 상위 8비트, UBRR_VALUE의 상위 바이트 저장
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);      //UART 수신 및 송신 기능 활성화
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);    //1 start bit + 8 data bit + 1 stop bit
}

char uart_receive_char() {
  while (!(UCSR0A & (1 << RXC0)));  // 수신이 완료될 때까지 대기(POLLING 방식)
  return UDR0;                      // 수신된 문자 반환
}

// PCA9685 설정
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // I2C로 PAC9685 서보드라이버 객체 생성

const int Z_CH = 1;     // Z축 서보모터 PCA9685의 1번핀에 연결
const int BASE_CH = 7;  // BASE축 서보모터 PCA9685의 7번핀에 연결


int angleToPWM(int angle) {
  return 150 + ((600 - 150) * angle) / 180;  
}  // 각도를 PCA9685 서보제어용 펄스 길이로 변환 (150 ~ 600, 0도 = 150, 180도 = 600)

// Timer1 설정 (Shoulder - OCR1A  ,   Elbow - OCR1B)
void setupTimer1() {
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  // FastPWM 모드, TOP = ICR1
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // 분주비 8 (010)
  ICR1 = 39999; // 20ms 주기 (50Hz)
}

void setShoulderAngle(double angle) {    // SHOULDER - DS3120MG (펄스폭 0.5ms ~ 2.5ms)
  OCR1A = ((angle * (5000 - 1000)) / 180) + 1000;  // 각도를 OCR1A 값으로 변환, OCR1A 1000 ~ 5000
}

void setElbowAngle(double angle) {       // ELBOW - MG996R (펄스폭 1ms ~ 2ms)
  OCR1B = ((angle * (4000 - 2000)) / 180) + 2000;  // 각도를 OCR1B 값으로 변환, OCR1B 2000 ~ 4000
}

//ISD1820 녹음모듈 설정
void setupVoice() {
  DDRB |= (1 << VOICE_START) | (1 << VOICE_FINISH);      // VOICE 핀을 출력으로 설정
  PORTB &= ~((1 << VOICE_START) | (1 << VOICE_FINISH));  // 초기 상태 LOW
}

void playVoice(int Voice_Pin) {
  PORTB |= (1 << Voice_Pin);   // 음성재생 버튼 누름
  _delay_ms(100);              // 100ms 유지
  PORTB &= ~(1 << Voice_Pin);  // 음성재생 버튼 뗌 (PLAYE 버튼은 한번 눌렀다 떼면 녹음된 음성 재생)
}

// ────────────────────────
void setup() {
  pinMode(RED, OUTPUT);   // 8번핀 - LED RED 출력으로 설정
  pinMode(BLUE, OUTPUT);  // 7번핀 - LED BLUE 출력으로 설정

  DDRB |= (1 << PB1); // D9 - OC1A (shoulder) 출력으로 설정
  DDRB |= (1 << PB2); // D10 - OC1B (elbow)) 출력으로 설정

  setupVoice(); // 음성 모듈 초기화

  Wire.begin(); //I2C 시작
  pwm.begin();  
  pwm.setPWMFreq(50); // PCA9685 주파수 50Hz 설정

  setupTimer1();  // TIMER1 설정

  //초기 위치로 초기화
  pwm.setPWM(BASE_CH, 0, angleToPWM(currentBase));
  setShoulderAngle(currentShoulder);
  setElbowAngle(currentElbow);
  pwm.setPWM(Z_CH, 0, angleToPWM(currentZ));

  uart_init(); // UART 통신 초기화
}

//BASE -> Z 모터이동 설정(+RED 깜빡임 설정)
void moveTo(double baseTarget, double shoulderTarget, double elbowTarget, double zTarget) {
  unsigned long lastBlink = millis();
  bool ledState = false;

  for (int i = 0; i <= steps; i++) {  // BASE - 이동을 50단계로 나누어 자연스럽게 이동
    double b = currentBase + (i * (baseTarget - currentBase) / steps); //한단계에 이동해야할 각도를 b에 저장
    pwm.setPWM(BASE_CH, 0, angleToPWM(b)); //서보를 b각도로 움직이게 명령
    if (millis() - lastBlink >= 500) {  //500ms마다 LED를 깜빡이기 위한 조건문
      ledState = !ledState;             //현재 LED 상태를 반전시킴(HIGH -> LOW, LOW -> HIGH)
      digitalWrite(RED, ledState);      //RED핀에 LED 상태를 출력
      lastBlink = millis();             //마지막 깜빡임 시간을 현재 시간으로 갱신
    }
    delay(delayBetweenSteps + i * i / 250); // 각 단계마다 딜레이를 점차 크게 만들어 점점 느려지는 이동 속도 
  }
  currentBase = baseTarget;  // 다음 이동을 위해 현재 각도 변수 변경
  delay(100);

for (int i = 0; i <= steps; i++) {  // SHOULDER - 이동을 50단계로 나누어 자연스럽게 이동
    double s = currentShoulder + (i * (shoulderTarget - currentShoulder) / steps);
    setShoulderAngle(s);  //서보를 s각도로 움직이게 명령
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250); // 각 단계마다 딜레이를 점차 크게 만들어 점점 느려지는 이동 속도 
  }
  currentShoulder = shoulderTarget;  // 다음 이동을 위해 현재 각도 변수 변경
  delay(100);

  for (int i = 0; i <= steps; i++) {  // ELBOW - 이동을 50단계로 나누어 자연스럽게 이동  
    double e = currentElbow + (i * (elbowTarget - currentElbow) / steps);
    setElbowAngle(e);    //서보를 e각도로 움직이게 명령
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250); // 각 단계마다 딜레이를 점차 크게 만들어 점점 느려지는 이동 속도 
  }
  currentElbow = elbowTarget;  // 다음 이동을 위해 현재 각도 변수 변경
  delay(100);

  for (int i = 0; i <= steps; i++) {  // Z - 이동을 50단계로 나누어 자연스럽게 이동  
    double z = currentZ + (i * (zTarget - currentZ) / steps);
    pwm.setPWM(Z_CH, 0, angleToPWM(z));   //서보를 z각도로 움직이게 명령
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250);  // 각 단계마다 딜레이를 점차 크게 만들어 점점 느려지는 이동 속도
  }
  currentZ = zTarget; // 다음 이동을 위해 현재 각도 변수 변경
}

//Z -> BASE 모터복귀 설정 (moveto()와 동일, 이동 순서만 반대) (+RED 깜빡임 설정)
void backTo(double baseTarget, double shoulderTarget, double elbowTarget, double zTarget) {
  unsigned long lastBlink = millis();
  bool ledState = false;

  for (int i = 0; i <= steps; i++) {  //Z
    double z = currentZ + (i * (zTarget - currentZ) / steps);
    pwm.setPWM(Z_CH, 0, angleToPWM(z));
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250);
  }
  currentZ = zTarget;
  delay(100);

  for (int i = 0; i <= steps; i++) {  //ELBOW
    double e = currentElbow + (i * (elbowTarget - currentElbow) / steps);
    setElbowAngle(e);
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250);
  }
  currentElbow = elbowTarget;
  delay(100);

  for (int i = 0; i <= steps; i++) {  //SHOULDER
    double s = currentShoulder + (i * (shoulderTarget - currentShoulder) / steps);
    setShoulderAngle(s);
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250);
  }
  currentShoulder = shoulderTarget;
  delay(100);

  for (int i = 0; i <= steps; i++) {  //BASE
   double b = currentBase + (i * (baseTarget - currentBase) / steps);
   pwm.setPWM(BASE_CH, 0, angleToPWM(b));
   if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(RED, ledState);
      lastBlink = millis();
    }
    delay(delayBetweenSteps + i * i / 250);
  }
  currentBase = baseTarget;

}

// ────────────────────────
void loop() {
  // 기본 상태 BLUE - ON
  digitalWrite(BLUE, HIGH); 

  char cmd = uart_receive_char();
    if (cmd == '1') {  // 블루투스에서 문자 '1' 대기

    // 주유시작 알림(음성 + BLUE OFF + RED TOGGLE)
    playVoice(VOICE_START);  
    digitalWrite(BLUE, LOW);
    delay(1000);

    // 목표 각도 설정
    double baseTarget = 0;
    double shoulderTarget = 90;
    double elbowTarget = 170;
    double zTarget = 0;

    // 목표 위치로 이동 (BASE -> Z)
    moveTo(baseTarget, shoulderTarget, elbowTarget, zTarget);

    delay(3000); 

    // 초기 위치로 복귀 (Z -> BASE)
    backTo(90, 135, 90, 90);  
    delay(1000);

    //주유종료 알림(음성 + RED OFF + BLUE ON)
    digitalWrite(RED, LOW);  
    digitalWrite(BLUE, HIGH);
    playVoice(VOICE_FINISH);  
  }
}