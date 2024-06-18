

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <ArduinoJson.h>        //needed for unpacking of JSON records

#include "ESP32TimerInterrupt.h"    // needed for timer interrupts for smooth motion

#include "ConnectionManager.h"
#include "SettingsManager.h"

#define PORTHOROLO_8M  // FT Device Designator
#include ".\include\ftconfig.h"

//  Global Variables
DynamicJsonDocument jsonDoc(4096);
WiFiMulti wifiMulti;
long subcounter = 0;  // Used to count each time a stepper timer interrupt happens
int LEDsubcounter = 0;  // Used to count each time a LED timer interrupt happens
uint64_t chipid;  // ESP32 unique board identifier
String chipIdStr;
String route ="sea-bi";
bool HardwareInitialized = false;

// These objects should never be destroyed
ConnectionManager* conn = nullptr;

// Class Definitions
class Dial {
  public:
    int motorPin;
    int maxSteps;
    int currentPosition;
    int targetPosition;
    int dialPeriod;   // average period between clicks in 500uS increments
    long timeAtLastWebUpdate;
    int positionAtLastWebUpdate;
    bool inverted;

    Dial(int mP, int mS, bool mI) {   // Constructor with params
      motorPin = mP;
      maxSteps = mS;
      inverted = mI;

      if (motorPin == 0) return;
      pinMode(motorPin, OUTPUT);
      currentPosition = 0;  // dials are created at position 0
      targetPosition = 0;
      dialPeriod = 1;    // maximum speed

    }

    void moveTowardTarget() {                            //  Method to move needle toward target position at rate defined by dialPeriod but only 1 pulse at a time
      if (motorPin == 0) return;
      if (subcounter % dialPeriod == 0)  {                // Only do this at the dialPeriod frequency.
        int difference = targetPosition - currentPosition;         // find difference between new position and current position
        if (difference != 0) {                            // Only move if not in target position
          if (abs(difference) > 25)                     // If the target is a long way away, move fast until 25 ticks away (equivalent to 2 degrees)
            dialPeriod = 1;
          else
            dialPeriod = 600;                                // Otherwise move slow (was 600)

          if (difference > 0)   {     // set direction accordingly
            digitalWrite(DIRPIN, (inverted) ? HIGH : LOW); // Set LOW for CCW, HIGH for CW
            currentPosition++;
          }
          else {    // set direction accordingly
            digitalWrite(DIRPIN, (inverted) ? LOW : HIGH); // Set LOW for CCW, HIGH for CW
            currentPosition--;
          }
          digitalWrite(motorPin, LOW);  // deliver one pulse
          delayMicroseconds(1);
          digitalWrite(motorPin, HIGH);
        }
      }
    }
};




class DisplayLED {
  public:
    int LED_Pin;
    int offBrightnessValue;
    int onBrightnessValue;
    int value;

    DisplayLED(int lP, int fBV, int nBV) {   // Constructor with params
      LED_Pin = lP;
      if (LED_Pin == 0) return;
      pinMode(LED_Pin, OUTPUT);
      offBrightnessValue = fBV;
      onBrightnessValue = nBV;
      digitalWrite(LED_Pin, LOW);  // turn the LED off (LOW is the voltage level)
      value = 0;
    }

    void turnOn() {
      value = 1;
    };

    void turnOff() {
      value = 0;
    };

    void upDate() {
      if (value == 1) {
        if (LEDsubcounter < onBrightnessValue) {
          digitalWrite(LED_Pin, HIGH);
        } else {
          digitalWrite(LED_Pin, LOW);
        }
      }
      else {
        if (LEDsubcounter < offBrightnessValue) {
          digitalWrite(LED_Pin, HIGH);
        } else {
          digitalWrite(LED_Pin, LOW);
        }
      }
    }

};

// Define Global Dials
Dial Boat1ProgressDial(B1_PROGRESS,  B1_PROGRESS_RANGE,  B1_PROGRESS_INVERT  );
Dial Boat2ProgressDial(B2_PROGRESS,  B2_PROGRESS_RANGE ,  B2_PROGRESS_INVERT );
Dial Boat1_ATMinusDial(B1_ARRIVALTIMEMINUS , B1_ARRIVALTIMEMINUS_RANGE, B1_ARRIVALTIMEMINUS_INVERT);
Dial Boat2_ATMinusDial(B2_ARRIVALTIMEMINUS,  B2_ARRIVALTIMEMINUS_RANGE,  B2_ARRIVALTIMEMINUS_INVERT);
Dial Boat1_StopTimer(B1_STOPTIMER, B1_STOPTIMER_RANGE, B1_STOPTIMER_INVERT);
Dial Boat2_StopTimer(B2_STOPTIMER, B2_STOPTIMER_RANGE, B2_STOPTIMER_INVERT);
Dial Boat1_DDDial(B1_DEPARTUREDELAY , B1_DEPARTUREDELAY_RANGE, B1_DEPARTUREDELAY_INVERT);
Dial Boat2_DDDial(B2_DEPARTUREDELAY , B2_DEPARTUREDELAY_RANGE, B2_DEPARTUREDELAY_INVERT);
Dial Boat1_DDADial(B1_DEPARTUREDELAYAVERAGE , B1_DEPARTUREDELAYAVERAGE_RANGE, B1_DEPARTUREDELAYAVERAGE_INVERT);
Dial Boat2_DDADial(B2_DEPARTUREDELAYAVERAGE , B2_DEPARTUREDELAYAVERAGE_RANGE, B2_DEPARTUREDELAYAVERAGE_INVERT);
Dial Boat1SpeedDial(B1_SPEED,  B1_SPEED_RANGE ,  B1_SPEED_INVERT );
Dial Boat2SpeedDial(B2_SPEED,  B2_SPEED_RANGE ,  B2_SPEED_INVERT );
Dial Boat1HeadingDial(B1_HEADING,  B1_HEADING_RANGE ,  B1_HEADING_INVERT );
Dial Boat2HeadingDial(B2_HEADING,  B2_HEADING_RANGE,  B2_HEADING_INVERT);

Dial PortWN_ATMinusDial(PWN_ARRIVALTIMEMINUS,  PWN_ARRIVALTIMEMINUS_RANGE,  PWN_ARRIVALTIMEMINUS_INVERT);
Dial PortES_ATMinusDial(PES_ARRIVALTIMEMINUS , PES_ARRIVALTIMEMINUS_RANGE, PES_ARRIVALTIMEMINUS_INVERT);
Dial PortWN_StopTimer(PWN_STOPTIMER, PWN_STOPTIMER_RANGE, PWN_STOPTIMER_INVERT);
Dial PortES_StopTimer(PES_STOPTIMER, PES_STOPTIMER_RANGE, PES_STOPTIMER_INVERT);
Dial PortWN_DDDial(PWN_DEPARTUREDELAY, PWN_DEPARTUREDELAY_RANGE, PWN_DEPARTUREDELAY_INVERT);
Dial PortES_DDDial(PES_DEPARTUREDELAY, PES_DEPARTUREDELAY_RANGE, PES_DEPARTUREDELAY_INVERT);
Dial PortWN_DDADial(PWN_DEPARTUREDELAYAVERAGE, PWN_DEPARTUREDELAYAVERAGE_RANGE, PWN_DEPARTUREDELAYAVERAGE_INVERT);
Dial PortES_DDADial(PES_DEPARTUREDELAYAVERAGE, PES_DEPARTUREDELAYAVERAGE_RANGE, PES_DEPARTUREDELAYAVERAGE_INVERT);

//Define Global LEDs
DisplayLED B1NavLampWN(B1NavLampWNPIN, OFF_BRIGHTNESS, NAVLAMP_BRIGHTNESS );
DisplayLED B1NavLampES(B1NavLampESPIN, OFF_BRIGHTNESS, NAVLAMP_BRIGHTNESS );
DisplayLED B1DockLampWN(B1DockLampWNPIN, OFF_BRIGHTNESS, DOCKLAMP_BRIGHTNESS );
DisplayLED B1DockLampES(B1DockLampESPIN, OFF_BRIGHTNESS, DOCKLAMP_BRIGHTNESS );
DisplayLED B1ServiceLamp(B1ServiceLampPIN, OFF_BRIGHTNESS, SERVICELAMP_BRIGHTNESS );
DisplayLED B1LateLamp(B1LateLampPIN, OFF_BRIGHTNESS, LATELAMP_BRIGHTNESS );

DisplayLED B2NavLampWN(B2NavLampWNPIN, OFF_BRIGHTNESS, NAVLAMP_BRIGHTNESS);
DisplayLED B2NavLampES(B2NavLampESPIN, OFF_BRIGHTNESS, NAVLAMP_BRIGHTNESS);
DisplayLED B2DockLampWN(B2DockLampWNPIN, OFF_BRIGHTNESS, DOCKLAMP_BRIGHTNESS);
DisplayLED B2DockLampES(B2DockLampESPIN, OFF_BRIGHTNESS, DOCKLAMP_BRIGHTNESS);
DisplayLED B2ServiceLamp(B2ServiceLampPIN, OFF_BRIGHTNESS, SERVICELAMP_BRIGHTNESS );
DisplayLED B2LateLamp(B2LateLampPIN, OFF_BRIGHTNESS, LATELAMP_BRIGHTNESS );

DisplayLED PWNDockLamp(PWNDockLampPIN, OFF_BRIGHTNESS, DOCKLAMP_BRIGHTNESS);
DisplayLED PESDockLamp(PESDockLampPIN, OFF_BRIGHTNESS, DOCKLAMP_BRIGHTNESS);

DisplayLED PESLateLamp(PESLateLampPIN, OFF_BRIGHTNESS, LATELAMP_BRIGHTNESS);
DisplayLED PWNLateLamp(PWNLateLampPIN, OFF_BRIGHTNESS, LATELAMP_BRIGHTNESS);

bool IRAM_ATTR TimerHandler0(void * timerNo)
{

  subcounter++;
  Boat1ProgressDial.moveTowardTarget();
  Boat2ProgressDial.moveTowardTarget();
  Boat1_ATMinusDial.moveTowardTarget();
  Boat2_ATMinusDial.moveTowardTarget();
  Boat1_StopTimer.moveTowardTarget();
  Boat2_StopTimer.moveTowardTarget();
  Boat1_DDDial.moveTowardTarget();
  Boat2_DDDial.moveTowardTarget();
  Boat1_DDADial.moveTowardTarget();
  Boat2_DDADial.moveTowardTarget();
  Boat1SpeedDial.moveTowardTarget();
  Boat2SpeedDial.moveTowardTarget();
  Boat1SpeedDial.moveTowardTarget();
  Boat2SpeedDial.moveTowardTarget();
  PortWN_ATMinusDial.moveTowardTarget();
  PortES_ATMinusDial.moveTowardTarget();
  PortWN_StopTimer.moveTowardTarget();
  PortES_StopTimer.moveTowardTarget();
  PortWN_DDDial.moveTowardTarget();
  PortES_DDDial.moveTowardTarget();
  PortWN_DDADial.moveTowardTarget();
  PortES_DDADial.moveTowardTarget();



  return true;
}

bool IRAM_ATTR TimerHandler1(void * timerNo)
{
  LEDsubcounter = (LEDsubcounter + 1) % 32;
  B1NavLampWN.upDate();
  B1NavLampES.upDate();
  B1DockLampWN.upDate();
  B1DockLampES.upDate();
  B1ServiceLamp.upDate();
  B1LateLamp.upDate();
  B2NavLampWN.upDate();
  B2NavLampES.upDate();
  B2DockLampWN.upDate();
  B2DockLampES.upDate();
  B2ServiceLamp.upDate();
  B2LateLamp.upDate();
  PWNDockLamp.upDate();
  PESDockLamp.upDate();
  PESLateLamp.upDate();
  PWNLateLamp.upDate();
  return true;
}

ESP32Timer ITimer0(0);    // Init ESP32 timer 0 and 1  0 is for stepper motors
ESP32Timer ITimer1(1);    // 1 is for LED PWM
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  conn = new ConnectionManager("FERRY TEMPO Setup");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  conn->update();
  if (conn->ready()) {     // wait for WiFi connection
    if (!HardwareInitialized) {
      initializeHardware();
    }
    HTTPClient http;
    String QueryURL ="https://ftserver-4fqq.onrender.com/api/v1/route/"+route+"?cid="+chipIdStr+"&model="+MODEL;
    Serial.println(QueryURL);
    http.begin(QueryURL); //Get Latest update from web server
    int httpCode = http.GET();    // start connection and send HTTP header
    if (httpCode > 0) {               // httpCode will be negative on error
      if (httpCode == HTTP_CODE_OK) {      // file found at server
        String downloadedJsonString = http.getString();

        Serial.println(downloadedJsonString);
        Serial.println();
        DeserializationError err = deserializeJson(jsonDoc, downloadedJsonString);
        if (err) {
          Serial.print(F("deserializeJson() failed with code "));
          Serial.println(err.f_str());
        }
        else {
          UpdateDisplay();
        }
      }
      else
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());  // http code not OK
      http.end();
    }
    delay(4500);
  } else {
    //    Serial.println("*");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UpdateDisplay() {

  /////////////////////////////////////////////////////////////
  ///                                                       ///
  /// Boat 1 Update                                         ///
  ///                                                       ///
  /////////////////////////////////////////////////////////////

  ///
  /// Service Lamps
  ///
  if ((bool)jsonDoc["boatData"]["boat1"]["OnDuty"])
    B1ServiceLamp.turnOn();
  else
    B1ServiceLamp.turnOff();

  ///
  /// Nav Lamps
  ///
  B1NavLampWN.turnOff();
  B1NavLampES.turnOff();
  if (!jsonDoc["boatData"]["boat1"]["AtDock"]) {      // Only show Nav lamps when boat in motion
    if (jsonDoc["boatData"]["boat1"]["Direction"] == "WN")
      B1NavLampWN.turnOn();
    else
      B1NavLampES.turnOn();
  }

  ///
  /// Late Lamp
  ///
  if (jsonDoc["boatData"]["boat1"]["BoatDepartureDelay"].as<int>() > 300)
    B1LateLamp.turnOn();
  else
    B1LateLamp.turnOff();

  ///
  /// Dock Lamps & Progress
  ///
  float B1progress = jsonDoc["boatData"]["boat1"]["Progress"];
  if (jsonDoc["boatData"]["boat1"]["AtDock"]) {                                         // Boat1 AtDock
    if (jsonDoc["boatData"]["boat1"]["Direction"] == "WN") {
      B1DockLampES.turnOn();                                                            // Show which dock
      Boat1ProgressDial.targetPosition = B1progress * Boat1ProgressDial.maxSteps;       // Set Boat Progress target position
    }
    else {
      B1DockLampWN.turnOn();
      Boat1ProgressDial.targetPosition = (1 - B1progress) * Boat1ProgressDial.maxSteps; // Set Boat Progress target position
    }
  }
  else {
    // Boat not AtDock
    B1DockLampES.turnOff();
    B1DockLampWN.turnOff();

    if (jsonDoc["boatData"]["boat1"]["Direction"] == "WN")
      Boat1ProgressDial.targetPosition = B1progress * Boat1ProgressDial.maxSteps;       // Set Boat Progress target position
    else                                                                                // Set NavLamps for direction ES
      Boat1ProgressDial.targetPosition = (1 - B1progress) * Boat1ProgressDial.maxSteps; // Set Boat Progress target position
  }

  ///
  /// Boat Stop Timer
  ///
  /// Add code
  //  Serial.print("Boat 1 Stop Timer");
  //  Serial.println();

  ///
  /// Boat Departure Delay
  ///
  int Boat1DepartureDelaySecs = jsonDoc["boatData"]["boat1"]["BoatDepartureDelay"].as<long>();
  Boat1_DDDial.targetPosition = int(limitTime(Boat1DepartureDelaySecs) * Boat1_DDDial.maxSteps / 3600);

  ///
  /// Boat Departure Delay Average
  ///
  //  int Boat1DepartureDelayAverageSecs = jsonDoc["boatData"]["boat1"]["BoatDepartureDelayAverage"].as<long>();
  //  Boat1_DDDial.targetPosition = int(Boat1DepartureDelayAverageSecs * Boat1_DDADial.maxSteps / 3600);
  //  Serial.print("Boat 1 Departure Delay Average ");
  //  Serial.print(Boat1DepartureDelayAverageSecs);
  //  Serial.println();

  ///
  /// Boat Speed
  ///
  //  Serial.print("Boat 1 Speed ");
  float B1_Speed = jsonDoc["boatData"]["boat1"]["Speed"];
  //  Serial.println(B1_Speed);
  Boat1SpeedDial.targetPosition = int(B1_Speed * Boat1SpeedDial.maxSteps / 30);

  ///
  /// Boat Heading
  ///
  /// Add code
  //  Serial.print("Boat 1 Heading ");
  int B1_Heading = jsonDoc["boatData"]["boat1"]["Heading"];
  //  Serial.println(B1_Heading);


  /////////////////////////////////////////////////////////////
  ///                                                       ///
  /// Boat 2 Update                                         ///
  ///                                                       ///
  /////////////////////////////////////////////////////////////

  ///
  /// Service Lamps
  ///
  if ((bool)jsonDoc["boatData"]["boat2"]["OnDuty"])
    B2ServiceLamp.turnOn();
  else
    B2ServiceLamp.turnOff();

  ///
  /// Nav Lamps
  ///
  B2NavLampWN.turnOff();
  B2NavLampES.turnOff();
  if (!jsonDoc["boatData"]["boat2"]["AtDock"]) {      // Only show Nav lamps when boat in motion
    if (jsonDoc["boatData"]["boat2"]["Direction"] == "WN")
      B2NavLampWN.turnOn();
    else
      B2NavLampES.turnOn();
  }

  ///
  /// Late Lamp
  ///
  if (jsonDoc["boatData"]["boat2"]["BoatDepartureDelay"].as<int>() > 300)
    B2LateLamp.turnOn();
  else
    B2LateLamp.turnOff();

  ///
  /// Dock Lamps & Progress
  ///
  float B2progress = jsonDoc["boatData"]["boat2"]["Progress"];
  if (jsonDoc["boatData"]["boat2"]["AtDock"]) {                                         // Boat2 AtDock
    if (jsonDoc["boatData"]["boat2"]["Direction"] == "WN") {
      B2DockLampES.turnOn();                                                            // Show which dock
      Boat2ProgressDial.targetPosition = B2progress * Boat2ProgressDial.maxSteps;       // Set Boat Progress target position only in case of startup when boat is at dock
    }
    else {
      B2DockLampWN.turnOn();
      Boat2ProgressDial.targetPosition = (1 - B2progress) * Boat2ProgressDial.maxSteps; // Set Boat Progress target position
    }
  }
  else {                                                                                // Boat2 not AtDock
    B2DockLampES.turnOff();                                                             // Turn off AtDock LEDs
    B2DockLampWN.turnOff();
    if (jsonDoc["boatData"]["boat2"]["Direction"] == "WN")
      Boat2ProgressDial.targetPosition = B2progress * Boat2ProgressDial.maxSteps;       // Set Boat Progress target position
    else
      Boat2ProgressDial.targetPosition = (1 - B2progress) * Boat2ProgressDial.maxSteps; // Set Boat Progress target position
  }

  ///
  /// Boat Departure Delay
  ///
  int Boat2DepartureDelaySecs = jsonDoc["boatData"]["boat2"]["BoatDepartureDelay"].as<long>();
  Boat2_DDDial.targetPosition = int(limitTime(Boat2DepartureDelaySecs) * Boat2_DDDial.maxSteps / 3600);

  ///
  /// Boat Departure Delay Average
  ///
  //  int Boat2DepartureDelayAverageSecs = jsonDoc["boatData"]["boat2"]["BoatDepartureDelayAverage"].as<long>();
  //  Boat2_DDDial.targetPosition = int(Boat2DepartureDelayAverageSecs * Boat2_DDADial.maxSteps / 3600);
  //  Serial.print("Boat 2 Departure Delay Average ");
  //  Serial.print(Boat2DepartureDelayAverageSecs);
  //  Serial.println();

  ///
  /// Boat Speed
  ///
  /// Add code
  //  Serial.print("Boat 2 Speed ");
  float B2_Speed = jsonDoc["boatData"]["boat2"]["Speed"];
  //  Serial.println(B2_Speed);
  Boat2SpeedDial.targetPosition = int(B2_Speed * Boat2SpeedDial.maxSteps / 30);

  ///
  /// Boat Heading
  ///
  /// Add code
  //  Serial.print("Boat 2 Heading ");
  int B2_Heading = jsonDoc["boatData"]["boat2"]["Heading"];
  //  Serial.println(B2_Heading);

  /////////////////////////////////////////////////////////////
  ///                                                       ///
  /// Port ES Update                                        ///
  ///                                                       ///
  /////////////////////////////////////////////////////////////
  ///
  /// At Dock Lamp
  ///
  if ((bool)jsonDoc["portData"]["portES"]["BoatAtDock"])
    PESDockLamp.turnOn();
  else
    PESDockLamp.turnOff();

  ///
  /// Port Late Lamp
  ///
  if (jsonDoc["portData"]["portES"]["PortDepartureDelayAverage"].as<int>() > 300)
    PESLateLamp.turnOn();
  else
    PESLateLamp.turnOff();

  ///
  /// Port Arrival Time Minus
  ///
  int portESArrivalTimeMinusSecs = jsonDoc["portData"]["portES"]["PortETA"].as<long>() - (jsonDoc["lastUpdate"].as<long>());
  if (portESArrivalTimeMinusSecs > 0)
    PortES_ATMinusDial.targetPosition = int(limitTime(portESArrivalTimeMinusSecs) * PortES_ATMinusDial.maxSteps / 3600);
  else
    PortES_ATMinusDial.targetPosition = 0;

  ///
  /// Port Departure Delay
  ///
  int ESDepartureDelay = int(jsonDoc["portData"]["portES"]["PortDepartureDelay"].as<int>());
  PortES_DDDial.targetPosition = long(limitTime(ESDepartureDelay) * PortES_DDDial.maxSteps) / 3600;

  ///
  /// Port Stop Timer
  ///
  int ESStopTimer = int(jsonDoc["portData"]["portES"]["PortStopTimer"].as<int>());
  PortES_StopTimer.targetPosition = long(limitTime(ESStopTimer) * PortES_StopTimer.maxSteps) / 3600;


  /////////////////////////////////////////////////////////////
  ///                                                       ///
  /// Port WN Update                                        ///
  ///                                                       ///
  /////////////////////////////////////////////////////////////
  ///
  /// At Dock Lamp
  ///
  if ((bool)jsonDoc["portData"]["portWN"]["BoatAtDock"])
    PWNDockLamp.turnOn();
  else
    PWNDockLamp.turnOff();

  ///
  /// Port Late Lamp
  ///
  if (jsonDoc["portData"]["portWN"]["PortDepartureDelayAverage"].as<int>() > 300)
    PWNLateLamp.turnOn();
  else
    PWNLateLamp.turnOff();

  ///
  /// Port Arrival Time Minus
  ///
  int portWNArrivalTimeMinusSecs = jsonDoc["portData"]["portWN"]["PortETA"].as<long>() - (jsonDoc["lastUpdate"].as<long>());
  if (portWNArrivalTimeMinusSecs > 0)
    PortWN_ATMinusDial.targetPosition = int(limitTime(portWNArrivalTimeMinusSecs) * PortWN_ATMinusDial.maxSteps / 3600);
  else
    PortWN_ATMinusDial.targetPosition = 0;

  ///
  /// Port Departure Delay
  ///
  int WNDepartureDelay = int(jsonDoc["portData"]["portWN"]["PortDepartureDelay"].as<int>());  // Now in secs
  PortWN_DDDial.targetPosition = long(limitTime(WNDepartureDelay) * PortWN_DDDial.maxSteps) / 3600;  //360 degrees is 60 mins is 3600 secs

  ///
  /// Port Stop Timer
  ///
  int WNStopTimer = int(jsonDoc["portData"]["portWN"]["PortStopTimer"].as<int>());
  PortWN_StopTimer.targetPosition = long(limitTime(WNStopTimer) * PortWN_StopTimer.maxSteps) / 3600;
}

int limitTime(int seconds) {
  if (seconds > 45 * 60) seconds = 52.5 * 60; // ensures that all dials showing time do not wrap at 60 mins.  After 45 mins, we show 52 mins
  return seconds;
}

void InitializeInterruptTimers() {
  Serial.print(F("\nStarting Argument_None on "));
  Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))   // Interval in microsecs
    //if (ITimer0.attachInterrupt(1, TimerHandler0))
  {
    Serial.print(F("Starting  ITimer0 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer0. Select another Timer, freq. or timer"));

  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1))    // Interval in microsecs
    //if (ITimer1.attachInterrupt(2, TimerHandler1))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another Timer, freq. or timer"));
}

void dialInitialization() {

  pinMode(DIRPIN, OUTPUT);

  Boat1ProgressDial.targetPosition = B1_PROGRESS_RANGE;
  Boat2ProgressDial.targetPosition = B2_PROGRESS_RANGE ;
  Boat1_ATMinusDial.targetPosition = B1_ARRIVALTIMEMINUS_RANGE;
  Boat2_ATMinusDial.targetPosition = B2_ARRIVALTIMEMINUS_RANGE;
  Boat1_StopTimer.targetPosition = B1_STOPTIMER_RANGE;
  Boat2_StopTimer.targetPosition = B2_STOPTIMER_RANGE;
  Boat1_DDDial.targetPosition = B1_DEPARTUREDELAY_RANGE;
  Boat2_DDDial.targetPosition = B2_DEPARTUREDELAY_RANGE;
  Boat1_DDADial.targetPosition = B1_DEPARTUREDELAYAVERAGE_RANGE;
  Boat2_DDADial.targetPosition = B2_DEPARTUREDELAYAVERAGE_RANGE;
  Boat1SpeedDial.targetPosition = B1_SPEED_RANGE ;
  Boat2SpeedDial.targetPosition = B2_SPEED_RANGE ;
  Boat1HeadingDial.targetPosition = B1_HEADING_RANGE ;
  Boat2HeadingDial.targetPosition = B2_HEADING_RANGE;
  PortWN_ATMinusDial.targetPosition = PWN_ARRIVALTIMEMINUS_RANGE;
  PortES_ATMinusDial.targetPosition = PES_ARRIVALTIMEMINUS_RANGE;
  PortWN_StopTimer.targetPosition = PWN_STOPTIMER_RANGE;
  PortES_StopTimer.targetPosition = PES_STOPTIMER_RANGE;
  PortWN_DDDial.targetPosition = PWN_DEPARTUREDELAY_RANGE;
  PortES_DDDial.targetPosition = PES_DEPARTUREDELAY_RANGE;
  PortWN_DDADial.targetPosition = PWN_DEPARTUREDELAYAVERAGE_RANGE;

  delay(3000);

  Boat1ProgressDial.targetPosition = 0;
  Boat2ProgressDial.targetPosition = 0;
  Boat1_ATMinusDial.targetPosition = 0;
  Boat2_ATMinusDial.targetPosition = 0;
  Boat1_StopTimer.targetPosition = 0;
  Boat2_StopTimer.targetPosition = 0;
  Boat1_DDDial.targetPosition = 0;
  Boat2_DDDial.targetPosition = 0;
  Boat1_DDADial.targetPosition = 0;
  Boat2_DDADial.targetPosition = 0;
  Boat1SpeedDial.targetPosition = 0;
  Boat2SpeedDial.targetPosition = 0;
  Boat1HeadingDial.targetPosition = 0;
  Boat2HeadingDial.targetPosition = 0;
  PortWN_ATMinusDial.targetPosition = 0;
  PortES_ATMinusDial.targetPosition = 0;
  PortWN_StopTimer.targetPosition = 0;
  PortES_StopTimer.targetPosition = 0;
  PortWN_DDDial.targetPosition = 0;
  PortES_DDDial.targetPosition = 0;
  PortWN_DDADial.targetPosition = 0;

}


void lampTest() {
  B1NavLampWN.turnOn();
  delay(100);
  B1NavLampES.turnOn();
  delay(100);
  B1DockLampWN.turnOn();
  delay(100);
  B1DockLampES.turnOn();
  delay(100);
  B1ServiceLamp.turnOn();
  delay(100);
  B1LateLamp.turnOn();
  delay(100);
  B2NavLampWN.turnOn();
  delay(100);
  B2NavLampES.turnOn();
  delay(100);
  B2DockLampWN.turnOn();
  delay(100);
  B2DockLampES.turnOn();
  delay(100);
  B2ServiceLamp.turnOn();
  delay(100);
  B2LateLamp.turnOn();
  delay(100);
  PWNDockLamp.turnOn();
  delay(100);
  PESDockLamp.turnOn();
  delay(100);
  PESLateLamp.turnOn();
  delay(100);
  PWNLateLamp.turnOn();
  delay(100);

  B1NavLampWN.turnOff();
  delay(100);
  B1NavLampES.turnOff();
  delay(100);
  B1DockLampWN.turnOff();
  delay(100);
  B1DockLampES.turnOff();
  delay(100);
  B1ServiceLamp.turnOff();
  delay(100);
  B1LateLamp.turnOff();
  delay(100);
  B2NavLampWN.turnOff();
  delay(100);
  B2NavLampES.turnOff();
  delay(100);
  B2DockLampWN.turnOff();
  delay(100);
  B2DockLampES.turnOff();
  delay(100);
  B2ServiceLamp.turnOff();
  delay(100);
  B2LateLamp.turnOff();
  delay(100);
  PWNDockLamp.turnOff();
  delay(100);
  PESDockLamp.turnOff();
  delay(100);
  PESLateLamp.turnOff();
  delay(100);
  PWNLateLamp.turnOff();
  delay(100);

}


void status_report() {
  Serial.print("Model: ");
  Serial.println(MODEL);
  Serial.print("Board: ");
  Serial.println(BOARD);

  Serial.print("   B1NavLampWN  Pin: ");
  Serial.println(B1NavLampWNPIN);
  Serial.print("   B1NavLampES  Pin: ");
  Serial.println(B1NavLampESPIN);
  Serial.print("   B1DockLampWN  Pin: ");
  Serial.println(B1DockLampWNPIN);
  Serial.print("   B1DockLampES  Pin: ");
  Serial.println(B1DockLampESPIN);
  Serial.print("   B1ServiceLamp  Pin: ");
  Serial.println(B1ServiceLampPIN);
  Serial.print("   B1LateLamp  Pin: ");
  Serial.println(B1LateLampPIN);
  Serial.print("   B2NavLampWN  Pin: ");
  Serial.println(B2NavLampWNPIN);
  Serial.print("   B2NavLampES  Pin: ");
  Serial.println(B2NavLampESPIN);
  Serial.print("   B2DockLampWN  Pin: ");
  Serial.println(B2DockLampWNPIN);
  Serial.print("   B2DockLampES  Pin: ");
  Serial.println(B2DockLampESPIN);
  Serial.print("   B2ServiceLamp  Pin: ");
  Serial.println(B2ServiceLampPIN);
  Serial.print("   B2LateLamp  Pin: ");
  Serial.println(B2LateLampPIN);
  Serial.print("   PWNDockLamp  Pin: ");
  Serial.println(PWNDockLampPIN);
  Serial.print("   PESDockLamp  Pin: ");
  Serial.println(PESDockLampPIN);
  Serial.print("   PESLateLamp  Pin: ");
  Serial.println(PESLateLampPIN);
  Serial.print("   PWNLateLamp  Pin: ");
  Serial.println(PWNLateLampPIN);

  Serial.print("   Boat1ProgressDial  Pin: ");
  Serial.println(B1_PROGRESS);
  Serial.print("   Boat2ProgressDial  Pin: ");
  Serial.println(B2_PROGRESS);
  Serial.print("   Boat1_ATMinusDial  Pin: ");
  Serial.println(B1_ARRIVALTIMEMINUS );
  Serial.print("   Boat2_ATMinusDial  Pin: ");
  Serial.println(B2_ARRIVALTIMEMINUS);
  Serial.print("   Boat1_StopTimer  Pin: ");
  Serial.println(B1_STOPTIMER);
  Serial.print("   Boat2_StopTimer  Pin: ");
  Serial.println(B2_STOPTIMER);
  Serial.print("   Boat1_DDDial  Pin: ");
  Serial.println(B1_DEPARTUREDELAY );
  Serial.print("   Boat2_DDDial  Pin: ");
  Serial.println(B2_DEPARTUREDELAY );
  Serial.print("   Boat1_DDADial  Pin: ");
  Serial.println(B1_DEPARTUREDELAYAVERAGE );
  Serial.print("   Boat2_DDADial  Pin: ");
  Serial.println(B2_DEPARTUREDELAYAVERAGE );
  Serial.print("   Boat1SpeedDial  Pin: ");
  Serial.println(B1_SPEED);
  Serial.print("   Boat2SpeedDial  Pin: ");
  Serial.println(B2_SPEED);
  Serial.print("   Boat1HeadingDial  Pin: ");
  Serial.println(B1_HEADING);
  Serial.print("   Boat2HeadingDial  Pin: ");
  Serial.println(B2_HEADING);
  Serial.print("   PortWN_ATMinusDial  Pin: ");
  Serial.println(PWN_ARRIVALTIMEMINUS);
  Serial.print("   PortES_ATMinusDial  Pin: ");
  Serial.println(PES_ARRIVALTIMEMINUS );
  Serial.print("   PortWN_StopTimer  Pin: ");
  Serial.println(PWN_STOPTIMER);
  Serial.print("   PortES_StopTimer  Pin: ");
  Serial.println(PES_STOPTIMER);
  Serial.print("   PortWN_DDDial  Pin: ");
  Serial.println(PWN_DEPARTUREDELAY);
  Serial.print("   PortES_DDDial  Pin: ");
  Serial.println(PES_DEPARTUREDELAY);
  Serial.print("   PortWN_DDADial  Pin: ");
  Serial.println(PWN_DEPARTUREDELAYAVERAGE);
  Serial.print("   PortES_DDADial  Pin: ");
  Serial.println(PES_DEPARTUREDELAYAVERAGE);

}
void initializeHardware(){
      const uint32_t chipIdUint = ESP.getEfuseMac();
      char chipId[8] = {0};
      sprintf(chipId, "%06X", chipIdUint);
      chipIdStr = String(chipId);

      
      status_report();
      InitializeInterruptTimers();
      lampTest();
      dialInitialization();
      HardwareInitialized = true;
}
