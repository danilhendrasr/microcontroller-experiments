#include "lib.h"
#include <analogWrite.h>

Heater::Heater(int pin) : pin(pin) {}

int Heater::get_pin() {
  return this->pin;
}

int Heater::get_degree() {
  int tmp = analogRead(this->pin) * 0.322265625;
  return tmp / 10;
}

void Toggleable::on(int channel) {
  ledcWrite(channel, 25);
}

void Toggleable::off(int channel) {
  ledcWrite(channel, 0);
}

void Q1::on() {
  Toggleable::on(Q1::pin);
}

void Q1::off() {
  Toggleable::off(Q1::pin);
}

void Q2::on() {
  Toggleable::on(Q2::pin);
}

void Q2::off() {
  Toggleable::off(Q2::pin);
}

void LED::on() {
  Toggleable::on(LED::pin);
}

void LED::off() {
  Toggleable::off(LED::pin);
}