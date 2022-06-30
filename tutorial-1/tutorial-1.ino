#include <Arduino.h>
#include <analogWrite.h>
#include "lib.h"

#define UPPER_TEMP_BOUND 35

float deg_heater1, deg_heater2;

struct HeaterTemps {
  int heater1;
  int heater2;
} current_temps;

Q1 q1;
Q2 q2;
LED led;
Heater heater1(34);
Heater heater2(35);

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

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(5000);
  analogWriteResolution(led.pin, 10);
  analogWriteResolution(q1.pin, 10);
  analogWriteResolution(q2.pin, 10);
}

void loop() {
  poll_temp();
  print_temp();

  if (current_temps.heater1 > UPPER_TEMP_BOUND) {
    q1.off();
    led.on();
  } else {
    q1.on();
    led.off();
  }

  if (current_temps.heater2 > UPPER_TEMP_BOUND) {
    q2.off();
    led.on();
  } else {
    q2.on();
    led.off();
  }

  delay(100);
}
