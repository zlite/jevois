// JeVois control steering from the output of JeVois modules
//
// We handle messages "T2D <targetx> <targety>", "T1D <targetx>", 
// targetx is assumed to be in the -1000 ... 1000 range as output by the JeVois Kalman filters.
// Here we only do simple PD control under the assumption that target coordinates have already been filtered upstream.
#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial mySerial(2,3); // RX, TX
// Pin for LED, blinks as we receive serial commands:
#define LEDPIN 13
// Serial port to use: on chips with USB (e.g., 32u4), that usually is Serial1. On chips without USB, use Serial:
Servo steerservo;
#define STEERPIN 10
// Initial servo values in degrees:
#define STEERZERO 90
// With updates typically coming in at 60Hz or up to 120Hz, we will often need to move by a fraction of a
// degree. Hence we keep track of the pan and tilt values multiplied by SCALE. For the gains, a gain of 100
// means we will update servo angle by the 0.1*(target value/SCALE) degrees on each update. Higher gains mean
// larger angular updates.
// Buffer for received serial port bytes:
#define INLEN 128
char instr[INLEN + 1];
long steerval;
long steergain = 50;
void setup()
{
  Serial.begin(57600);
  mySerial.begin(38400);
  mySerial.setTimeout(100000);
  steerservo.attach(STEERPIN);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
// We are ready to rock, disable logs and turn on serial outputs on JeVois platform: 
//  mySerial.println("setpar serlog None");
//  mySerial.println("setpar serout Hard");
  Serial.print("Starting...");
}
void loop()
{
  byte len = mySerial.readBytesUntil('\n', instr, INLEN);
  Serial.print("Data: ");
  Serial.println(instr);
  instr[len] = 0;
  digitalWrite(LEDPIN, HIGH);
  char * tok = strtok(instr, " \r\n");
  int state = 0; int targx = 0;
  while (tok)
  {
    // State machine:
    // 0: start parsing
    // 4: T1D command, parse targx
    // 5: T1D command complete
    // 1000: unknown command
    switch (state)
    {
      case 0:
        if (strcmp(tok, "T1D") == 0) state = 4;
        else state = 1000;
        break;
      case 4: targx = atoi(tok); state = 5; break;
      default: break; // Skip any additional tokens
    }
    tok = strtok(0, " \r\n");
  }
  // Target coordinates are in range -1000 ... 1000. Servos want 0 ... 180.
  // We also need to negate as needed so that the servo turns to cancel any offset from center:
  if (state == 5)
  {
    steerval = ((targx * steergain)/1000) + 90;
    steerval = constrain(steerval, 0, 180);
    Serial.print("servo: ");
    Serial.println(steerval);
    steerservo.write(steerval);
  }
}
