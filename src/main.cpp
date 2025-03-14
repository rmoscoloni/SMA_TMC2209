#include <TMC2209.h>

// PINS
// 10uF CAP BETWEEN ESP32 PINS EN AND GND 
// POT GND 3V3 CENTER TO ESP32 PIN 32
// TMC2209 ENABLE TO ESP32 PIN 4 
// TMC2209 ENABLE 10K RESISTOR TO 3V3 (HARDWARE DISABLE)
// TMC2209 UART TO ESP32 PIN 16
// TMC2209 UART TO 1K RESISTOR OTHER SIDE TO ESP32 PIN 17
// TMC2209 LOGIC 3v3 AND GND FROM ESP32
// TMC2209 MOTOR POWER SUPPLY TO PINS GND AND SMD POT SIDE
// 100uF ELECTROLYTIC AND 100nF CERAMIC CAP ON MOTOR POWER SUPPLY
// TMC2209 TO MOTOR COILS

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
HardwareSerial & serial_stream = Serial2; //RX pin 16 y TX pin 17

const int ENABLE_PIN = 4;
const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 500;
int32_t VELOCITY = 0;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 80;
const uint8_t HOLD_CURRENT_PERCENT = 0;
const uint8_t STALL_GUARD_THRESHOLD = 250;

// Instantiate TMC2209
TMC2209 stepper_driver;

/*------------------------- MOVING AVG ------------------------*/
unsigned int movingAverage(unsigned int value) {
const byte nvalues = 30;  // Moving average window size

static byte current = 0;         // Index for current value
static byte values_counter = 0;  // Count of values read (<= nvalues)
static long sum = 0;             // Rolling sum
static int values[nvalues];

sum += value;

// If the window is full, adjust the sum by deleting the oldest value
if (values_counter == nvalues)
  sum -= values[current];
values[current] = value;  // Replace the oldest with the latest
if (++current >= nvalues)
  current = 0;
if (values_counter < nvalues)
  values_counter += 1;

return sum / values_counter;
}
/*------------------------- POTE      -------------------------*/
const int potPin = 32; // center pin
//variable for storing the potentiometer value
int pot_value = 0;
/*------------------------- TIME CONTROL ----------------------*/
unsigned int remaining_t(int update_t) {
  return millis() - update_t;
}
/*------------------------- CHECK VOLATILE ----------------------*/
uint8_t interfase_counter = false;
void CHECK_VOLATILE() {
  interfase_counter = stepper_driver.getInterfaceTransmissionCounter();
}
/*------------------------- WRITE VOLATILE ----------------------*/
void WRITE_VOLATILE() {
  //stepper_driver.setup(serial_stream, 9600, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
  stepper_driver.setHardwareEnablePin(ENABLE_PIN);
  stepper_driver.setHoldCurrent(HOLD_CURRENT_PERCENT);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
  // valid values = 1,2,4,8,...128,256, other values get rounded down
  stepper_driver.setMicrostepsPerStep(1);
  // stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  // stepper_driver.enableCoolStep();
  stepper_driver.disableStealthChop();
}
/*------------------------- SERIAL    -------------------------*/
unsigned long millisSerial_Last;
int serialInterval = 2000;  //2000 milliseconds

void SERIAL_PRINT() {
  if (remaining_t(millisSerial_Last) > serialInterval) {
    Serial.println(VELOCITY);
    millisSerial_Last = millis();
    
    if (stepper_driver.isSetupAndCommunicating())
      {
      Serial.println("Stepper driver is setup and communicating!");
      Serial.println("Try turning driver power off to see what happens.");
      }
    else if (stepper_driver.isCommunicatingButNotSetup())
      {
      Serial.println("Stepper driver is communicating but not setup!");
      Serial.println("Running setup again...");
      stepper_driver.setup(serial_stream);
      }
    else
      {
      Serial.println("Stepper driver is not communicating!");
      Serial.println("Try turning driver power on to see what happens.");
      }
    TMC2209::Settings settings = stepper_driver.getSettings();
    Serial.print("settings.software_enabled = ");
    Serial.print(settings.software_enabled);
    Serial.print(" (hardware_disabled = ");
    Serial.print(stepper_driver.hardwareDisabled());
    Serial.println(")");
    Serial.print("settings.microsteps_per_step = ");
    Serial.println(settings.microsteps_per_step);
    Serial.print("settings.stealth_chop_enabled = ");
    Serial.println(settings.stealth_chop_enabled);
    Serial.print("settings.irun_percent = ");
    Serial.println(settings.irun_percent);
    Serial.print("settings.irun_register_value = ");
    Serial.println(settings.irun_register_value);
    Serial.print("settings.ihold_percent = ");
    Serial.println(settings.ihold_percent);
    Serial.print("settings.ihold_register_value = ");
    Serial.println(settings.ihold_register_value);
    Serial.print("interfase_counter = ");
    Serial.println(interfase_counter);
    Serial.println("-----------------------------------");
    SerialBT.print(VELOCITY*60/200);
    SerialBT.println("RPM");
    CHECK_VOLATILE();
    if (!interfase_counter){
      Serial.println("WRITE VOLATILE");
      Serial.println("-----------------------------------");
      WRITE_VOLATILE();
    }
    if(stepper_driver.hardwareDisabled() && pot_value <= 10){
      Serial.println("P4 SOFTWARE ENABLE");
      Serial.println("-----------------------------------");
      stepper_driver.enable();
    }
  }
}
/*------------------------- SETUP     -------------------------*/
void setup()
{
  delay(DELAY*4);
  pinMode(potPin, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);
  stepper_driver.setup(serial_stream);
  WRITE_VOLATILE();
  if (analogRead(potPin <= 10))
    stepper_driver.enable();
  SerialBT.begin("SMA_TMC2209");
}
/*------------------------- LOOP      -------------------------*/
void loop()
{
  if(interfase_counter > 0){
  pot_value = (analogRead(potPin));
  int32_t avg_pot = movingAverage(pot_value);
  int32_t pot_maped = map(avg_pot, 0, 4095, 0, 6000);
  VELOCITY  = pot_maped;
  stepper_driver.moveAtVelocity(VELOCITY);
  }
  SERIAL_PRINT();
}