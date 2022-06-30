#include "lib.h"
#include <analogWrite.h>

Heater::Heater(int pin) : pin(pin) {}

int Heater::get_pin() {
  return this->pin;
}

float Heater::get_degree() {
  float tmp = analogRead(this->pin) * 0.322265625;
  return tmp / 10;
}

void Toggleable::on(int channel) {
  ledcWrite(channel, 50);
}

void Toggleable::off(int channel) {
  ledcWrite(channel, 0);
}

void Q1::on() {
  Toggleable::on(this->pin);
}

void Q1::off() {
  Toggleable::off(this->pin);
}

void Q2::on() {
  Toggleable::on(this->pin);
}

void Q2::off() {
  Toggleable::off(this->pin);
}

void LED::on() {
  Toggleable::on(this->pin);
}

void LED::off() {
  Toggleable::off(this->pin);
}