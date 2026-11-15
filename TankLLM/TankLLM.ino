/*
 * TankLLM — прошивка-парсер команд для LAFVIN Smart Robot Tank Kit (LA033 / Mini Tank)
 *
 * Связь: Bluetooth HC-06 на аппаратном Serial (пины 0/1), 9600 бод.
 * ВНИМАНИЕ: при заливке скетча по USB выньте модуль HC-06 из слота!
 *
 * Протокол (одна команда = одна строка, ответ = одна строка):
 *   PING                -> PONG
 *   FWD <ms>            -> OK / STOPPED_IR / STOPPED_SONAR / STOPPED_SONAR_NO_ECHO
 *   BACK <ms>           -> OK        назад
 *   LEFT <ms>           -> OK        разворот влево на месте
 *   RIGHT <ms>          -> OK        разворот вправо на месте
 *   STOP                -> OK        немедленный стоп
 *   SPEED <0..255>      -> OK        базовая скорость
 *   SCAN                -> DIST:d1,d2,d3,d4,d5   замер дистанции (см) в 5 точках
 *   STATUS              -> SPEED:<n> MAXDUR:<n>  текущие настройки
 *
 * Защита: длительность движения ограничена MAX_DURATION мс,
 * после каждого движения моторы останавливаются автоматически.
 */

#include <Servo.h>

// ---------- Распиновка расширительной платы LAFVIN ----------
const uint8_t PIN_L_DIR = 2;   // направление левых моторов
const uint8_t PIN_L_PWM = 5;   // скорость левых моторов
const uint8_t PIN_R_DIR = 4;   // направление правых моторов
const uint8_t PIN_R_PWM = 6;   // скорость правых моторов
const uint8_t PIN_TRIG  = 12;  // ультразвук: триггер
const uint8_t PIN_ECHO  = 13;  // ультразвук: эхо
const uint8_t PIN_SERVO = 10;  // серво с датчиком
const uint8_t PIN_IR_L  = A2;  // левый ИК-датчик (LOW = препятствие); замерено 15.11.2026
const uint8_t PIN_IR_R  = A1;  // правый ИК-датчик — на плате A1/A2 перепутаны
const uint8_t PIN_BATT  = A0;  // вольтметр: делитель 1:1 (10k+10k) с АКБ+ на A0

// ---------- Калибровка ----------
// У дешёвых моторов разбег большой: подберите PWM так, чтобы танк ехал прямо.
// Обычно одна сторона слабее — ей дают больший PWM.
int speedL = 168;              // PWM стороны A (пин 5) — физически правая гусеница
int speedR = 255;              // PWM стороны B (пин 6) — физически левая гусеница
// Калибровка от 15.11.2026: пины 5/6 на этой плате оказались перепутаны
// относительно названий; сторона на пине 5 быстрее — приглушена до 168.

unsigned long MAX_DURATION = 3000;   // предохранитель: максимум 3 с на команду
int servoCenter = 107;                // калибровка «прямо» — подобрано 15.11.2026
const unsigned long SONAR_STOP_CM = 20;  // предварительный порог: проверить на поднятом танке
const unsigned long SONAR_INTERVAL_MS = 60;
const unsigned long ECHO_TIMEOUT_US = 25000UL;  // ~4.3 м — дальность ультразвука

Servo servo;
String line;

// ---------------- Моторы ----------------
// Левая сторона: dir=LOW -> вперёд; правая: dir=HIGH -> вперёд (моторы стоят зеркально).
void motors(int dirL, int pwmL, int dirR, int pwmR) {
  digitalWrite(PIN_L_DIR, dirL);
  analogWrite(PIN_L_PWM, pwmL);
  digitalWrite(PIN_R_DIR, dirR);
  analogWrite(PIN_R_PWM, pwmR);
}

void stopMotors() {
  analogWrite(PIN_L_PWM, 0);
  analogWrite(PIN_R_PWM, 0);
}

void forward()  { motors(LOW,  speedL, HIGH, speedR); }
void backward() { motors(HIGH, speedL, LOW,  speedR); }
void rotLeft()  { motors(HIGH, speedL, HIGH, speedR); }
void rotRight() { motors(LOW,  speedL, LOW,  speedR); }

// Защита работает только при движении вперёд. Во время ожидания эха
// ИК-датчики и время движения продолжают проверяться, чтобы pulseIn не
// блокировал аварийную остановку.
enum DriveResult { DRIVE_OK, DRIVE_IR, DRIVE_SONAR, DRIVE_NO_ECHO };
DriveResult sonarGuard(bool moving, unsigned long started, unsigned long duration);
DriveResult driveFor(void (*move)(), unsigned long ms, bool fwd);
unsigned long lastSonarTrigger = 0;

bool irObstacle() {
  return digitalRead(PIN_IR_L) == LOW || digitalRead(PIN_IR_R) == LOW;
}

DriveResult sonarGuard(bool moving, unsigned long started, unsigned long duration) {
  if (irObstacle()) return DRIVE_IR;
  // Также между двумя последовательными командами FWD выдержать паузу.
  while (millis() - lastSonarTrigger < SONAR_INTERVAL_MS) {
    if (irObstacle()) return DRIVE_IR;
    if (moving && millis() - started >= duration) return DRIVE_OK;
    delay(1);
  }
  lastSonarTrigger = millis();
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long waitStart = micros();
  while (digitalRead(PIN_ECHO) == LOW) {
    if (irObstacle()) return DRIVE_IR;
    if (moving && millis() - started >= duration) return DRIVE_OK;
    if (micros() - waitStart >= ECHO_TIMEOUT_US) return DRIVE_NO_ECHO;
  }
  unsigned long echoStart = micros();
  while (digitalRead(PIN_ECHO) == HIGH) {
    if (irObstacle()) return DRIVE_IR;
    if (moving && millis() - started >= duration) return DRIVE_OK;
    if (micros() - waitStart >= ECHO_TIMEOUT_US) return DRIVE_NO_ECHO;
  }
  unsigned long echoUs = micros() - echoStart;
  // Целочисленное сравнение без округления дистанции в большую сторону.
  return echoUs <= SONAR_STOP_CM * 58UL ? DRIVE_SONAR : DRIVE_OK;
}

DriveResult driveFor(void (*move)(), unsigned long ms, bool fwd) {
  if (ms > MAX_DURATION) ms = MAX_DURATION;
  if (ms == 0) ms = 1;
  if (fwd) {
    DriveResult before = sonarGuard(false, 0, 0);
    if (before != DRIVE_OK) { stopMotors(); return before; }
  }
  move();
  unsigned long started = millis();
  // Отсчёт от предварительного замера исключает слишком частые импульсы HC-SR04.
  DriveResult result = DRIVE_OK;
  while (millis() - started < ms) {
    if (fwd && irObstacle()) { result = DRIVE_IR; break; }
    if (fwd && millis() - lastSonarTrigger >= SONAR_INTERVAL_MS) {
      result = sonarGuard(true, started, ms);
      if (result != DRIVE_OK) break;
    }
    delay(1);
  }
  stopMotors();
  return result;
}

// ---------------- Ультразвук ----------------
float distanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return 400.0;          // нет эха — препятствий нет / дальше 4 м
  float d = us / 58.0;
  if (d < 2.0) d = 2.0;
  return d;
}

// Замер в 5 точках: 20°, 60°, 90°, 120°, 160°
void scan() {
  const int offs[5] = {-70, -30, 0, 30, 70};   // углы относительно центра
  int angles[5];
  for (int i = 0; i < 5; i++) {
    angles[i] = constrain(servoCenter + offs[i], 5, 175);
  }
  Serial.print("DIST:");
  for (int i = 0; i < 5; i++) {
    servo.write(angles[i]);
    delay(350);                        // дать серве доехать
    float d = distanceCm();
    Serial.print(d, 0);
    if (i < 4) Serial.print(",");
  }
  Serial.println();
  servo.write(servoCenter);            // вернуть датчик вперёд
  delay(350);                        // голова должна успеть вернуться в центр
}

// ---------------- Команды ----------------
void handle(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  int sp = cmd.indexOf(' ');
  String op  = (sp < 0) ? cmd : cmd.substring(0, sp);
  long   arg = (sp < 0) ? 0   : cmd.substring(sp + 1).toInt();

  if (op == "PING") {
    Serial.println("PONG");
  } else if (op == "BATT") {
    // BATT -> BATT:<мВ> — напряжение АКБ через делитель 1:1 на A0.
    // 8 выборок для сглаживания; Vref = 5.0 В, делитель x2.
    unsigned long acc = 0;
    for (uint8_t i = 0; i < 8; i++) { acc += analogRead(PIN_BATT); delay(2); }
    unsigned long mv = (acc / 8) * 5000UL / 1023UL * 2UL;
    Serial.print("BATT:");
    Serial.println(mv);
  } else if (op == "FWD") {
    DriveResult result = driveFor(forward, arg, true);
    if (result == DRIVE_IR) Serial.println("STOPPED_IR");
    else if (result == DRIVE_SONAR) Serial.println("STOPPED_SONAR");
    else if (result == DRIVE_NO_ECHO) Serial.println("STOPPED_SONAR_NO_ECHO");
    else Serial.println("OK");
  } else if (op == "BACK") {
    driveFor(backward, arg, false); Serial.println("OK");
  } else if (op == "LEFT") {
    driveFor(rotLeft, arg, false);  Serial.println("OK");
  } else if (op == "RIGHT") {
    driveFor(rotRight, arg, false); Serial.println("OK");
  } else if (op == "STOP") {
    stopMotors();            Serial.println("OK");
  } else if (op == "SPEED") {
    if (arg >= 0 && arg <= 255) {
      speedL = arg;
      speedR = arg;
      Serial.println("OK");
    } else {
      Serial.println("ERR BAD_SPEED");
    }
  } else if (op == "TRIM") {
    // TRIM <левый> <правый> — раздельная настройка PWM сторон, 0..255
    int sp2 = cmd.indexOf(' ', sp + 1);
    if (sp < 0 || sp2 < 0) {
      Serial.println("ERR BAD_TRIM");
    } else {
      int l = cmd.substring(sp + 1, sp2).toInt();
      int r = cmd.substring(sp2 + 1).toInt();
      if (l >= 0 && l <= 255 && r >= 0 && r <= 255) {
        speedL = l;
        speedR = r;
        Serial.println("OK");
      } else {
        Serial.println("ERR BAD_TRIM");
      }
    }
  } else if (op == "CENTER") {
    if (arg >= 5 && arg <= 175) {
      servoCenter = arg;
      servo.write(servoCenter);
      delay(350); // дождаться центра перед следующей командой движения
      Serial.println("OK");
    } else {
      Serial.println("ERR BAD_CENTER");
    }
  } else if (op == "PINS") {
    Serial.print("PINS:");
    for (int p = A0; p <= A5; p++) {
      pinMode(p, INPUT);
      Serial.print(digitalRead(p));
      if (p < A5) Serial.print(",");
    }
    Serial.println();
  } else if (op == "IR") {
    Serial.print("IR:");
    Serial.print(digitalRead(PIN_IR_L) == LOW ? 1 : 0);
    Serial.print(",");
    Serial.println(digitalRead(PIN_IR_R) == LOW ? 1 : 0);
  } else if (op == "SCAN") {
    scan();
  } else if (op == "STATUS") {
    Serial.print("SPEED:");
    Serial.print(speedL);
    Serial.print("/");
    Serial.print(speedR);
    Serial.print(" CENTER:");
    Serial.print(servoCenter);
    Serial.print(" MAXDUR:");
    Serial.println(MAX_DURATION);
  } else {
    Serial.println("ERR UNKNOWN_CMD");
  }
}

void setup() {
  pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_R_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_L, INPUT);
  pinMode(PIN_IR_R, INPUT);
  servo.attach(PIN_SERVO);
  servo.write(servoCenter);
  stopMotors();
  delay(350); // сервопривод успевает направить датчик вперёд
  Serial.begin(9600);      // стандартная скорость HC-06
  Serial.setTimeout(50);
  Serial.println("READY TankLLM v1.0");
}

void loop() {
  if (Serial.available()) {
    line = Serial.readStringUntil('\n');
    handle(line);
  }
}
