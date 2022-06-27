class Heater {
public:
  Heater(int pin);
  int get_pin();
  int get_degree();
private:
  int pin;
};

class Toggleable {
public:
  void on(int pin);
  void off(int pin);
};

class Q1 : Toggleable {
public:
  int pin = 32;
  virtual void on();
  virtual void off();
};

class Q2 : Toggleable {
public:
  int pin = 33;
  virtual void on();
  virtual void off();
};

class LED : Toggleable {
public:
  int pin = 26;
  virtual void on();
  virtual void off();
};