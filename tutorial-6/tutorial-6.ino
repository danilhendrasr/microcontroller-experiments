/*
  iTCLab Internet-Based Temperature Control Lab Firmware
  Jeffrey Kantor, Initial Version
  John Hedengren, Modified
  Oct 2017
  Basuki Rahmat, Modified
  April 2022

  This firmware is loaded into the Internet-Based Temperature Control Laboratory ESP32 to
  provide a high level interface to the Internet-Based Temperature Control Lab. The firmware
  scans the serial port looking for case-insensitive commands:

  Q1        set Heater 1, range 0 to 100% subject to limit (0-255 int)
  Q2        set Heater 2, range 0 to 100% subject to limit (0-255 int)
  T1        get Temperature T1, returns deg C as string
  T2        get Temperature T2, returns dec C as string
  VER       get firmware version string
  X         stop, enter sleep mode

  Limits on the heater can be configured with the constants below.
*/

#include <Arduino.h>

// constants
const String vers = "1.04";    // version of this firmware
const int baud = 115200;       // serial baud rate
const char sp = ' ';           // command separator
const char nl = '\n';          // command terminator

// pin numbers corresponding to signals on the iTCLab Shield
const int pinT1   = 34;         // T1
const int pinT2   = 35;         // T2
const int pinQ1   = 32;         // Q1
const int pinQ2   = 33;         // Q2
const int pinLED  = 26;         // LED

// setting PWM properties
const int freq = 5000; //5000
const int ledChannel = 0;
const int Q1Channel = 1;
const int Q2Channel = 2;
const int resolutionLedChannel = 8; //Resolution 8, 10, 12, 15
const int resolutionQ1Channel = 8; //Resolution 8, 10, 12, 15
const int resolutionQ2Channel = 8; //Resolution 8, 10, 12, 15

const double batas_suhu_atas = 59;

// global variables
char Buffer[64];               // buffer for parsing serial input
String cmd;                    // command 
double pv = 0;                 // pin value
float level;                   // LED level (0-100%)
double Q1 = 0;                 // value written to Q1 pin
double Q2 = 0;                 // value written to Q2 pin
int iwrite = 0;                // integer value for writing
float dwrite = 0;              // float value for writing
int n = 10;                    // number of samples for each temperature measurement

void parseSerial(void) {
  int ByteCount = Serial.readBytesUntil(nl,Buffer,sizeof(Buffer));
  String read_ = String(Buffer);
  memset(Buffer,0,sizeof(Buffer));
   
  // separate command from associated data
  int idx = read_.indexOf(sp);
  cmd = read_.substring(0,idx);
  cmd.trim();
  cmd.toUpperCase();

  // extract data. toInt() returns 0 on error
  String data = read_.substring(idx+1);
  data.trim();
  pv = data.toFloat();
}

// Q1_max = 100%
// Q2_max = 100%

void dispatchCommand(void) {
  if (cmd == "Q1") {   
    Q1 = max(0.0, min(25.0, pv));
    iwrite = int(Q1 * 2.0); // 10.? max
    iwrite = max(0, min(255, iwrite));    
    ledcWrite(Q1Channel,iwrite);
    Serial.println(Q1);
  }
  else if (cmd == "Q2") {
    Q2 = max(0.0, min(25.0, pv));
    iwrite = int(Q2 * 2.0); // 10.? max
    iwrite = max(0, min(255, iwrite));
    ledcWrite(Q2Channel,iwrite);   
    Serial.println(Q2);
  }
  else if (cmd == "T1") {
    float mV = 0.0;
    float degC = 0.0;
    for (int i = 0; i < n; i++) {
        mV = (float) analogRead(pinT1) * 0.322265625;
        degC = degC + mV/10.0;
    }
    degC = degC / float(n);   
    
    Serial.println(degC);
  }
  else if (cmd == "T2") {
    float mV = 0.0;
    float degC = 0.0;
    for (int i = 0; i < n; i++) {
         mV = (float) analogRead(pinT2) * 0.322265625;
         degC = degC + mV/10.0;
   }
    degC = degC / float(n);
    Serial.println(degC);
  }
  else if ((cmd == "V") or (cmd == "VER")) {
    Serial.println("TCLab Firmware Version " + vers);
  }
  else if (cmd == "LED") {
    level = max(0.0, min(100.0, pv));
    iwrite = int(level * 0.5);
    iwrite = max(0, min(50, iwrite));    
    ledcWrite(ledChannel, iwrite);      
    Serial.println(level);
  }  
  else if (cmd == "X") {
    ledcWrite(Q1Channel,0);
    ledcWrite(Q2Channel,0);
    Serial.println("Stop");
  }
}

// check temperature and shut-off heaters if above high limit
void checkTemp(void) {
    float mV = (float) analogRead(pinT1) * 0.322265625;
    //float degC = (mV - 500.0)/10.0;
    float degC = mV/10.0;
    if (degC >= batas_suhu_atas) {
      Q1 = 0.0;
      Q2 = 0.0;
      ledcWrite(Q1Channel,0);
      ledcWrite(Q2Channel,0);
      //Serial.println("High Temp 1 (> batas_suhu_atas): ");
      Serial.println(degC);
    }
    mV = (float) analogRead(pinT2) * 0.322265625;
    //degC = (mV - 500.0)/10.0;
    degC = mV/10.0;
    if (degC >= batas_suhu_atas) {
      Q1 = 0.0;
      Q2 = 0.0;
      ledcWrite(Q1Channel,0);
      ledcWrite(Q2Channel,0);
      //Serial.println("High Temp 2 (> batas_suhu_atas): ");
      Serial.println(degC);
    }
}

// arduino startup
void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(baud); 
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  // configure pinQ1 PWM functionalitites
  ledcSetup(Q1Channel, freq, resolutionQ1Channel);
  
  // attach the channel to the pinQ1 to be controlled
  ledcAttachPin(pinQ1, Q1Channel); 

  // configure pinQ2 PWM functionalitites
  ledcSetup(Q2Channel, freq, resolutionQ2Channel);
  
  // attach the channel to the pinQ2 to be controlled
  ledcAttachPin(pinQ2, Q2Channel);   

  // configure pinLED PWM functionalitites
  ledcSetup(ledChannel, freq, resolutionLedChannel);
  
  // attach the channel to the pinLED to be controlled
  ledcAttachPin(pinLED, ledChannel); 

  ledcWrite(Q1Channel,0);
  ledcWrite(Q2Channel,0);
}

// arduino main event loop
void loop() {
  parseSerial();
  dispatchCommand();
  checkTemp();
}
