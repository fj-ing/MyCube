#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include <FastLED.h>

BluetoothSerial SerialBT;
CRGB leds[NUM_PIXELS];

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32-Cube"); // Bluetooth device name
  EEPROM.begin(EEPROM_SIZE);

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS); // GRB ordering is typical

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  // LED startup animation
  for (int i = 0; i <= 255; i += 10) {
    leds[0] = CRGB(i, 0, 0);
    leds[1] = CRGB(i, 0, 0);
    leds[2] = CRGB(i, 0, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i = 0; i <= 255; i += 10) {
    leds[0] = CRGB(0, i, 0);
    leds[1] = CRGB(0, i, 0);
    leds[2] = CRGB(0, i, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i = 0; i <= 255; i += 10) {
    leds[0] = CRGB(0, 0, i);
    leds[1] = CRGB(0, 0, i);
    leds[2] = CRGB(0, 0, i);
    FastLED.show();
    delay(5);
  }
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();

  // Motor 1
  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT_PULLUP);  // 加上拉，抗干扰
  pinMode(ENC1_2, INPUT_PULLUP);
  enc_count1 = 0;                 // 👈 清零
  motor1_speed = 0;
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcAttach(PWM1, BASE_FREQ, TIMER_BIT);
  Motor1_control(0);              // 现在真正输出 0

  // Motor 2
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT_PULLUP);
  pinMode(ENC2_2, INPUT_PULLUP);
  enc_count2 = 0;                 // 👈 清零
  motor2_speed = 0;
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcAttach(PWM2, BASE_FREQ, TIMER_BIT);
  Motor2_control(0);

  // Motor 3
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT_PULLUP);
  pinMode(ENC3_2, INPUT_PULLUP);
  enc_count3 = 0;                 // 👈 清零
  motor3_speed = 0;
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcAttach(PWM3, BASE_FREQ, TIMER_BIT);
  Motor3_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
    calibrated = true;

  delay(200);
  angle_setup();
}

void loop()
{
  currentT = millis();
  if (currentT - previousT_1 >= loop_time)
  {
    Tuning();
    SerialTuning();
    angle_calc();

    // 读取编码器速度
    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;

    if (vertical_vertex && calibrated && !calibrating)
    {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroZ = GyZ / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;

      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

      motors_speed_X += speed_X / 5;
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
    }
    else if (vertical_edge && calibrated && !calibrating)
    {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;

      // 🔧 临时关闭 eK3/eK4，避免正反馈
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt, -255, 255);

      motors_speed_X += motor3_speed / 5;

      // ✅ 关键：显式停止其他电机！
      Motor1_control(0);
      Motor2_control(0);
      Motor3_control(pwm_X);
    }
    else
    {
      // 完全停止
      Motor1_control(0);
      Motor2_control(0);
      Motor3_control(0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
    }
    previousT_1 = currentT;
  }

  if (currentT - previousT_2 >= 2000)
  {
    battVoltage((double)analogRead(VBAT) / 204);
    if (!calibrated && !calibrating)
    {
      SerialBT.println("first you need to calibrate the balancing points...");
      Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
      if (!calibrated_leds)
      {
        leds[0] = CRGB(0, 255, 0);
        leds[1] = CRGB(0, 255, 0);
        leds[2] = CRGB(0, 255, 0);
        FastLED.show();
        calibrated_leds = true;
      }
      else
      {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
        FastLED.show();
        calibrated_leds = false;
      }
    }
    previousT_2 = currentT;
  }
}