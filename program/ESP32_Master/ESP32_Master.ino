#include <Ps3Controller.h>
#include <ESP32Servo.h>

// GPIO PIN
#define IN1 19  // Motor Kiri Depan (M1)
#define IN2 18
#define IN3 17  // Motor Kanan Depan (M2)
#define IN4 16

#define IN5 22  // Motor Kiri Belakang (M3)
#define IN6 23
#define IN7 26  // Motor Kanan Belakang (M4)
#define IN8 25

#define PWM 15  // PWM tunggal (INA)

#define BUZZER 13

#define conveyor 32

#define gripperLeftPin 2
#define gripperRightPin 4
#define servoLifter 5
#define stopperLifter 12


// STATE
#define KANAN 1
#define KIRI 0
#define BUKA 1
#define TUTUP 0
#define NAIK 1
#define TURUN 0

// STATE VALUE
#define IDLE 95
#define LIFTER_IDLE 90
#define OPEN 60
#define CLOSE 0
#define DOWN 0
#define UP 180
#define STOP 90
#define GO 0

#define RESET 0

Servo gripperLeft;
Servo gripperRight;
Servo lifter;
Servo stopper;

int servoGripperAngle[2];
int servoLifterState;
int servoStopperState;
int conveyorState;
int indexR, indexL, indexS, indexC, indexPwm;

int pwmLevel = 255;  // Default 100%

// Motor depan

void setMotorPWM(int pwm);
void motorStop();
void motorDepan(bool maju);
void motorBelakang(bool maju);
void reset();
void moveRight();
void moveLeft();
void rotateLeft();
void rotateRight();
void serongKananMaju();
void serongKiriMaju();
void buzzer(int, int, int);
void serongKananMundur();
void serongKiriMundur();


void setup() {
  Serial.begin(115200);
  Ps3.begin("b0:a7:32:2a:a3:3c");  // Ganti dengan MAC address ESP32 kamu
  Ps3.attach(onPS3Receive);

  gripperLeft.attach(gripperLeftPin);
  gripperRight.attach(gripperRightPin);
  lifter.attach(servoLifter);
  stopper.attach(stopperLifter);

  // Setup semua pin motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(conveyor, OUTPUT);
  pinMode(PWM, OUTPUT);  // PWM tunggal

  reset();
}

void loop() {
  // Tidak perlu polling, karena pakai callback
}

void onPS3Receive() {
  if (!Ps3.isConnected()) {
    buzzer(0, 0, 0);
    Serial.println("putus boss");
  } else {
    int lx = Ps3.data.analog.stick.lx;  // Joystick Kiri X (GESER)
    int ly = Ps3.data.analog.stick.ly;  // Joystick Kiri Y (MAJU/MUNDUR/SERONG)

    int rx = Ps3.data.analog.stick.rx;  // Joystick Kanan X (ROTASI)
    int ry = Ps3.data.analog.stick.ry;  // Joystick Kanan Y

    const int threshold = 15;  // Ambang batas yang lebih besar (misal 15) untuk menghindari drifting
    bool gerak = false;
    bool sweep = false;

    // --- LOGIKA UTAMA PERGERAKAN (Joystick Kiri) ---

    // 1. MAJU LURUS
    if (ly < -threshold && abs(lx) < threshold) {
      motorDepan(true);
      motorBelakang(true);
      gerak = true;
      Serial.println("maju");
    }
    // 2. MUNDUR LURUS
    else if (ly > threshold && abs(lx) < threshold) {
      motorDepan(false);
      motorBelakang(false);
      gerak = true;
      Serial.println("mundur");
    }
    // // 3. SERONG KIRI MAJU
    // else if (ly < -threshold && lx < -threshold) {
    //   serongKiriMaju();
    //   gerak = true;
    //   Serial.println("serong kiri maju");
    // }
    // // 4. SERONG KANAN MAJU
    // else if (ly < -threshold && lx > threshold) {
    //   serongKananMaju();
    //   gerak = true;
    //   Serial.println("serong kanan maju");
    // }
    // // 5. SERONG KIRI MUNDUR (Tambahan)
    // else if (ly > threshold && lx < -threshold) {
    //   serongKiriMundur();
    //   gerak = true;
    //   Serial.println("serong kiri mundur");
    // }
    // // 6. SERONG KANAN MUNDUR (Tambahan)
    // else if (ly > threshold && lx > threshold) {
    //   serongKananMundur();
    //   gerak = true;
    //   Serial.println("serong kanan mundur");
    // }
    // 7. GESER KIRI (Oleh LX)
    else if (abs(ly) < threshold && lx < -threshold) {
      moveLeft();
      gerak = true;
      Serial.println("geser kiri");
    }
    // 8. GESER KANAN (Oleh LX)
    else if (abs(ly) < threshold && lx > threshold) {
      moveRight();
      gerak = true;
      Serial.println("geser kanan");
    }

    // --- LOGIKA ROTASI (Joystick Kanan RX) ---
    // 9. ROTASI KIRI (Oleh RX)
    else if (abs(ry) < threshold && rx < -threshold) {
      rotateLeft();
      gerak = true;
      Serial.println("putar kiri");
    }
    // 10. ROTASI KANAN (Oleh RX)
    else if (abs(ry) < threshold && rx > threshold) {
      rotateRight();
      gerak = true;
      Serial.println("putar kanan");
    }


    // SET PWM DAN STOP
    if (gerak) {
      setMotorPWM(pwmLevel);
    } else {
      motorStop();
    }

    // // KONTROL LIFTER
    if (Ps3.data.button.down) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        sweep = true;
        servoLifterState = DOWN;

        lastPressTime = millis();
        lifter.write(servoLifterState);
        Serial.print("Lifter set to: ");
        Serial.println(servoLifterState);
      }
    } else if (Ps3.data.button.up) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        sweep = true;
        servoLifterState = UP;

        lastPressTime = millis();
        lifter.write(servoLifterState);
        Serial.print("Lifter set to: ");
        Serial.println(servoLifterState);
      }
    } else {
      servoLifterState = LIFTER_IDLE;
      lifter.write(servoLifterState);
      // Serial.print("Lifter set to: ");
      // Serial.println(servoLifterState);
    }

    // KONTROL GRIPPER
    if (Ps3.data.button.left) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        indexL = (indexL + 1) % 2;

        switch (indexL) {
          case 0: servoGripperAngle[KIRI] = CLOSE; break;  // tutup
          case 1: servoGripperAngle[KIRI] = OPEN; break;   // buka
        }
        lastPressTime = millis();
        gripperLeft.write(servoGripperAngle[KIRI]);
        Serial.print("Gripper Left set to: ");
        Serial.println(servoGripperAngle[KIRI]);
      }
    } else if (Ps3.data.button.right) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        indexR = (indexR + 1) % 2;

        switch (indexR) {
          case 0: servoGripperAngle[KANAN] = CLOSE; break;  // tutup
          case 1: servoGripperAngle[KANAN] = OPEN; break;   // buka
        }
        lastPressTime = millis();
        gripperRight.write(servoGripperAngle[KANAN]);
        Serial.print("Gripper Right set to: ");
        Serial.println(servoGripperAngle[KANAN]);
      }
    }

    // Ganti level PWM dengan tombol X
    if (Ps3.data.button.cross) { // Slow
      // Gunakan static unsigned long untuk debounce
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        // indexPwm = (indexPwm + 1) % 2;

        // switch (indexPwm) {
        //   case 0: pwmLevel = 128; break;  // 50%
        //   case 1: pwmLevel = 255; break;  // 100%
        // }

        pwmLevel = 190; // 70%
        lastPressTime = millis();
        Serial.print("PWM level set to: ");
        Serial.println(pwmLevel);
      }
    }

    if (Ps3.data.button.circle) {
      // Gunakan static unsigned long untuk debounce
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        indexPwm = (indexPwm + 1) % 2;

        // switch (indexPwm) {
        //   case 0: pwmLevel = 128; break;  // 50%
        //   case 1: pwmLevel = 255; break;  // 100%
        // }

        pwmLevel = 225; // 88%
        lastPressTime = millis();
        Serial.print("PWM level set to: ");
        Serial.println(pwmLevel);
      }
    }

    // STOP SEMUA (override manual)
    if (Ps3.data.button.select) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        reset();
        lastPressTime = millis();
      }
    }

    // KONTROL STOPPER
    if (Ps3.data.button.square) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        indexS = (indexS + 1) % 2;

        switch (indexS) {
          case 0: servoStopperState = STOP; break;  // tutup
          case 1: servoStopperState = GO; break;    // buka
        }
        lastPressTime = millis();
        stopper.write(servoStopperState);
        Serial.print("Stopper set to: ");
        Serial.println(servoStopperState);
      }
    }

    // kontrol conveyor
    if (Ps3.data.button.l1) {
      static unsigned long lastPressTime = 0;
      if (millis() - lastPressTime > 300) {
        indexC = (indexC + 1) % 2;

        switch (indexC) {
          case 0: conveyorState = 1; break;  // tutup
          case 1: conveyorState = 0; break;  // buka
        }
        lastPressTime = millis();
        digitalWrite(conveyor, conveyorState);
        Serial.print("Conveyor set to: ");
        Serial.println(conveyorState);
      }
    }
  }
}

// --- FUNGSI MOTOR ---

void setMotorPWM(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(PWM, pwm);
}

// Fungsi STOP yang lebih spesifik
void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
}

void reset() {
  servoGripperAngle[KIRI] = IDLE;
  servoGripperAngle[KANAN] = IDLE;
  servoLifterState = LIFTER_IDLE;
  indexL = 0;
  indexR = 0;
  indexPwm = 0;

  motorStop();
  gripperLeft.write(servoGripperAngle[KIRI]);
  gripperRight.write(servoGripperAngle[KANAN]);
  // lifter.write(servoLifterState);

  Serial.println("reset all");
}

void motorDepan(bool maju) {
  // Motor Kiri Depan (M1): IN1(maju) / IN2(mundur)
  digitalWrite(IN1, maju ? HIGH : LOW);
  digitalWrite(IN2, maju ? LOW : HIGH);
  // Motor Kanan Depan (M2): IN3(maju) / IN4(mundur)
  digitalWrite(IN3, maju ? HIGH : LOW);
  digitalWrite(IN4, maju ? LOW : HIGH);
}

void motorBelakang(bool maju) {
  // Motor Kiri Belakang (M3): IN5(maju) / IN6(mundur)
  digitalWrite(IN5, maju ? HIGH : LOW);
  digitalWrite(IN6, maju ? LOW : HIGH);
  // Motor Kanan Belakang (M4): IN7(maju) / IN8(mundur)
  digitalWrite(IN7, maju ? HIGH : LOW);
  digitalWrite(IN8, maju ? LOW : HIGH);
}

// GESER KANAN (Motor 1: MUNDUR, M2: MAJU, M3: MAJU, M4: MUNDUR)
void moveLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // M1 Mundur
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // M2 Maju

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);  // M3 Maju (Dibalik)
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);  // M4 Mundur (Dibalik)
  // Catatan: Arah motor Mecanum harus sesuai konfigurasi roda Anda
}

// GESER KIRI (Motor 1: MAJU, M2: MUNDUR, M3: MUNDUR, M4: MAJU)
void moveRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  // M1 Maju
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // M2 Mundur

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);  // M3 Mundur (Dibalik)
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);  // M4 Maju (Dibalik)
  // Catatan: Arah motor Mecanum harus sesuai konfigurasi roda Anda
}

// ROTASI KIRI (Motor Kiri: MUNDUR, Motor Kanan: MAJU)
void rotateLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // M1 Mundur
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // M2 Maju

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);  // M3 Mundur
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);  // M4 Maju
}

// ROTASI KANAN (Motor Kiri: MAJU, Motor Kanan: MUNDUR)
void rotateRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  // M1 Maju
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // M2 Mundur

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);  // M3 Maju
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);  // M4 Mundur
}

// SERONG KANAN MAJU (Motor Kanan Depan: STOP, sisanya MAJU)
void serongKananMaju() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  // M1 Maju
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);  // M2 STOP (Ini harus diuji)

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);  // M3 Maju
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);  // M4 Maju
}

// SERONG KIRI MAJU (Motor Kiri Depan: STOP, sisanya MAJU)
void serongKiriMaju() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  // M1 STOP
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // M2 Maju

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);  // M3 Maju
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);  // M4 Maju
}

// --- FUNGSI SERONG MUNDUR (Tambahan) ---

// SERONG KANAN MUNDUR (Motor Kiri Depan: MUNDUR, M. Kanan Belakang: MUNDUR)
void serongKananMundur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  // M1 STOP
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // M2 Maju

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);  // M3 Maju
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);  // M4 Maju
  // Catatan: Serong mundur di Mecanum/Omni lebih kompleks. Kita gunakan logika dasar.
}

// SERONG KIRI MUNDUR (Motor Kanan Depan: MUNDUR, M. Kiri Belakang: MUNDUR)
void serongKiriMundur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // M1 Maju
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);  // M2 STOP (Ini harus diuji)

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);  // M3 Maju
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);  // M4 Maju
  // Sama dengan mundur lurus, kita perlu logika Mecanum yang tepat di sini.
}

void buzzer(int count, int duration, int jeda) {
  if (count == 0) {
    tone(BUZZER, 1000);
    return;
  }

  for (int i = 0; i < count; i++) {
    tone(BUZZER, 1000);
    delay(duration);
    noTone(BUZZER);
    if (i < count - 1) {
      delay(jeda);
    }
  }
}

// void gripper(int index, int state){}

// void lifter(int state){}

// void stopper(int state){}