#include <math.h>
#include "HardwareSerial.h"
#ifndef ALPHA_H
#define ALPHA_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <MPU6050_light.h>



//==================TCA9548A==================
void tcaSelect(int ch) {
  if (ch > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
//==================Board==================
class AlphaBoard {
  int buttonPin;
public:
  AlphaBoard(int pin)
    : buttonPin(pin) {}
  void begin() {
    pinMode(buttonPin, INPUT_PULLUP);
  }
  void waitFor() {
    while (digitalRead(buttonPin) == HIGH) {
      delay(50);
    }
  }
};

//==================Motors==================
class AlphaMotor {
private:
  int in1, in2, pwm;

public:
  AlphaMotor(int in1pin, int in2pin, int pwmpin) {
    in1 = in1pin;
    in2 = in2pin;
    pwm = pwmpin;
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
  }

  void forward(int speed) {
    speed = constrain(speed, 0, 100);
    int pwmval = map(speed, 0, 100, 0, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, pwmval);
  }

  void backward(int speed) {
    speed = constrain(speed, 0, 100);
    int pwmval = map(speed, 0, 100, 0, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, pwmval);
  }

  void stop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
};
//==================ServoMotors==================
class ServoMotors {
private:
  int pin;
  Servo servo;
public:
  ServoMotors(int p) {
    pin = p;
  }
  void begin() {
    servo.attach(pin);
  }
  void moveToAngle(int angle) {
    servo.write(angle);
  }
};
//==================GyroSensor==================
class AlphaGyroSensor {
  MPU6050 mpu;
  bool present;
  int channel;
  float yaw;
  unsigned long lastTime;

public:
  AlphaGyroSensor(int ch)
    : mpu(Wire), present(false), channel(ch) {}
  void begin() {
    if (channel > 0) tcaSelect(channel);
    if (mpu.begin() == 0) {
      present = true;
      mpu.calcOffsets();
      lastTime = millis();
    }
  }
  float GetGyroZ() {
    tcaSelect(channel);
    if (present) mpu.update();
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    yaw += mpu.getGyroZ() * dt;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    return yaw;
  }
  float GetAngleZ() {
    tcaSelect(channel);
    if (present) mpu.update();
    mpu.getAngleZ();
  }
};
// ==================DistanceSensor==================
class AlphaDistanceSensor {
  int trig, echo;
public:
  AlphaDistanceSensor(int t, int e)
    : trig(t), echo(e) {}
  void begin() {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
  }
  long readDistance() {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH, 20000);
    float dis = duration * 0.034 / 2;
    if (dis > 250) return 0;
    return dis;
  }
};
// -------------------- Color Sensor --------------------
enum ColorMode { RGB,
                 HSV };
class AlphaColorSensor {
  Adafruit_TCS34725 tcs;
  uint8_t channel;
  ColorMode mode;
public:
  AlphaColorSensor(uint8_t tca_channel, ColorMode m = RGB)
    : tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X), channel(tca_channel), mode(m) {}
  void begin() {
    tcaSelect(channel);
    if (!tcs.begin())
      while (1)
        ;
  }
  int readColor() {
    tcaSelect(channel);
    uint16_t r, g, b, c;
    const int samples = 5;
    uint32_t sumR = 0, sumG = 0, sumB = 0;
    for (int i = 0; i < samples; i++) {
      tcs.getRawData(&r, &g, &b, &c);
      sumR += r;
      sumG += g;
      sumB += b;
      delay(20);
    }
    float avgR = sumR / float(samples);
    float avgG = sumG / float(samples);
    float avgB = sumB / float(samples);
    if (mode == RGB) {
      float total = avgR + avgG + avgB;
      if (total == 0) return 0;
      float ratioR = avgR / total;
      float ratioB = avgB / total;
      if (ratioB > 0.4) return 2;       // BLUE
      else if (ratioR > 0.4) return 5;  // RED
      else return 0;
    } else if (mode == HSV) {
      float R = avgR / 255.0, G = avgG / 255.0, B = avgB / 255.0;
      float maxV = max(R, max(G, B));
      float minV = min(R, min(G, B));
      float H, S, V;
      V = maxV;
      float delta = maxV - minV;
      S = (maxV == 0) ? 0 : delta / maxV;
      if (delta == 0) H = 0;
      else if (maxV == R) H = 60.0 * fmod(((G - B) / delta), 6.0);
      else if (maxV == G) H = 60.0 * (((B - R) / delta) + 2);
      else H = 60.0 * (((R - G) / delta) + 4);
      if (H < 0) H += 360;
      if (H >= 200 && H <= 350) return 2;    // BLUE
      else if (H >= 3 && H <= 15) return 5;  // RED
      else return 0;
    }
    return 0;
  }
};
#endif

