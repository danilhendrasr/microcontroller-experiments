#include <Arduino.h>
#include <analogWrite.h>

#define T1 34
#define T2 35
#define LED 26
#define Q1 32
#define Q2 33

#define BATAS_SUHU_ATAS 25

float cel, cel1, deg_c, deg_c1;

void q1_on() {
  analogWrite(Q1, 255);
}

void q1_off() {
  analogWrite(Q1, 0);
}

void q2_on() {
  analogWrite(Q2, 255);
}

void q2_off() {
  analogWrite(Q2, 0);
}

void led_on() {
  analogWrite(LED, 255);
}

void led_off() {
  analogWrite(LED, 0);
}

void temp_check() {
  deg_c = analogRead(T1) * 0.322265625;
  cel = deg_c / 10;
  deg_c1 = analogRead(T2) * 0.322265625;
  cel1 = deg_c / 10;

  Serial.print("Temperatureeee: ");
  Serial.print(cel);
  Serial.print("°C");
  Serial.print("  ~   ");
  Serial.print(cel1);
  Serial.println("°C");
}

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(5000);
  analogWriteResolution(LED, 10);
  analogWriteResolution(Q1, 10);
  analogWriteResolution(Q2, 10);
}

void loop() {
  temp_check();

  if (cel > BATAS_SUHU_ATAS) {
    q1_off();
    led_on();
  } else {
    q1_on();
    led_off();
  }

  if (cel1 > BATAS_SUHU_ATAS) {
    q2_off();
    led_on();
  } else {
    q2_on();
    led_off();
  }

  delay(100);
}
