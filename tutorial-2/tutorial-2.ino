#include <Arduino.h>
#include "lib.h"

#define BAUD_RATE 115200
#define PWM_FREQ 5000
#define UPPER_TEMP_BOUND 55

Q1 q1;
Q2 q2;
LED led;

Heater heater1(34);
Heater heater2(35);

float deg_heater1, deg_heater2;

struct HeaterTemps {
  float heater1;
  float heater2;
} current_temps;

void setup() {
  Serial.begin(BAUD_RATE);

  // wait for serial port to connect.
  while (!Serial) { ; }

  ledcSetup(q1.channel, PWM_FREQ, q1.channel_resolution);
  ledcAttachPin(q1.pin, q1.channel);

  ledcSetup(q2.channel, PWM_FREQ, q2.channel_resolution);
  ledcAttachPin(q2.pin, q2.channel);

  ledcSetup(led.channel, PWM_FREQ, led.channel_resolution);
  ledcAttachPin(led.pin, led.channel);

  ledcWrite(q1.channel, 0);
  ledcWrite(q2.channel, 0);
  ledcWrite(led.channel, 0);
}

// Get the temperature of both heaters
void poll_temp() {
  current_temps.heater1 = heater1.get_degree();
  current_temps.heater2 = heater2.get_degree();
}

// Print the current temperature of both heaters to serial console.
void print_temp() {
  Serial.print("Temperatureeee: ");
  Serial.print(current_temps.heater1);
  Serial.print("°C");
  Serial.print("  ~   ");
  Serial.print(current_temps.heater2);
  Serial.println("°C");
}

void loop() {
  poll_temp();
  print_temp();

  if (deg_heater1 > UPPER_TEMP_BOUND) {
    q1.off();
    led.on();
  } else {
    q1.on();
    led.off();
  }

  if (deg_heater2 > UPPER_TEMP_BOUND) {
    q2.off();
    led.on();
  } else {
    q2.on();
    led.off();
  }

  delay(500);
}