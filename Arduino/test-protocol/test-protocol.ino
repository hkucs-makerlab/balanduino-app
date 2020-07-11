#include <Wire.h>
#include "Kalman.h"
#include "EEPROMAnything.h"

#define __DEBUG__

#define __SOFTWARE_SERIAL__
#ifdef __SOFTWARE_SERIAL__
#include <SoftwareSerial.h>
#define BAUD_RATE 38400
#define Console Serial
#define BT_RX_PIN 3
#define BT_TX_PIN 4
SoftwareSerial BlueTooth(BT_RX_PIN, BT_TX_PIN);
#else
//#define BAUD_RATE 38400
#define BAUD_RATE 115200
#define Console Serial
#define BlueTooth Serial
#undef __DEBUG__
#endif

class Beep {
    static const int pin = 5;
  public:
    void beep() {
      tone(pin, 500, 250);
    }
};
Beep beep;

typedef struct {
  float P, I, D; // PID variables
  float targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  float Qangle, Qbias, Rmeasure; // Kalman filter values
  float accYzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
  bool bindSpektrum;
} cfg_t;

cfg_t cfg;
Kalman kalman;

static const char *version = "1.3.0";
static const uint8_t eepromVersion = 3;

static uint32_t receiveControlTimer;
static float sppData1, sppData2;


static bool sendPairConfirmation = false, sendPIDValues = false, sendSettings = false, sendInfo = true, sendKalmanValues = false, sendIMUValues = false, sendStatusReport = false;
static float batteryVoltage = 12;
static float accAngle = -1, gyroAngle = -1;
static float pitch = -1; // Result from Kalman filter
//
static uint32_t kalmanTimer; // Timer used for the Kalman filter
static uint32_t pidTimer; // Timer used for the PID loop
static uint32_t imuTimer; // This is used to set a delay between sending IMU values
static uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
static uint32_t reportTimer; // This is used to set a delay between sending report values
static uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
static uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming BlueTooth request
#ifdef __DEBUG__
static uint32_t debugTimer = 0;
#endif
//
/* IMU Data */
static float gyroXzero;
static uint8_t i2cBuffer[8]; // Buffer for I2C data
boolean imuReady = false;

void setup() {
  BlueTooth.begin(BAUD_RATE);
#ifdef __DEBUG__
#ifdef __SOFTWARE_SERIAL__
  Console.begin(115200);
#endif
  Console.println("in debugging mode");
#endif
  //restoreEEPROMValues();
  if (!checkInitializationFlags()) {
    readEEPROMValues(); // Only read the EEPROM values if they have not been restored
  } else { // Indicate that the EEPROM values have been reset by turning on the buzzer
    beep.beep();
    beep.beep();
    beep.beep();
  }
  //
  kalmanTimer = micros();
  pidTimer = kalmanTimer;
  imuTimer = millis();
  encoderTimer = imuTimer;
  reportTimer = imuTimer;
  ledTimer = imuTimer;
  blinkTimer = imuTimer;
#ifdef __DEBUG__
  debugTimer = millis();
#endif
  //
  imuSetup();
  if (!imuReady) {
    // fake the reading
    pitch = 190;
    accAngle = 180;
    gyroAngle = 170;
  }
  beep.beep();
#ifdef __DEBUG__
  printMenu();
#endif

}

void loop() {
  //
  if (imuReady) {
    control();
  }
  checkBlueToothData();
  printValues();
}


void control() {
  /* Calculate pitch */
  while (i2cRead(0x3D, i2cBuffer, 8));
  int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  int16_t gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;
  uint32_t timer = micros();
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    kalman.setAngle(accAngle);
    pitch = accAngle;
    gyroAngle = accAngle;
  } else {
    float gyroRate = ((float)gyroX - gyroXzero) / 131.0f; // Convert to deg/s
    float dt = (float)(timer - kalmanTimer) / 1000000.0f;
    gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
    pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter
  }
  kalmanTimer = timer;
#ifdef __DEBUG__
  //fake the reading if NaN
  if (isnan(pitch)) {
    pitch = 190;
  }
  if (isnan(gyroAngle)) {
    gyroAngle = 170;
  }
  if (millis() - debugTimer >= 1000) {
    debugTimer = millis();
    Console.print("AccAngle: ");  Console.print(accAngle);
    Console.print("\tGyro Angle:");  Console.print(gyroAngle);
    Console.print("\tPitch Angle: ");  Console.println(pitch);
  }
#else
 //-1 indicates incorrect reading
  if (isnan(pitch)) {
    pitch = -1;
  }
  if (isnan(gyroAngle)) {
    gyroAngle = -1;
  }
#endif

  if (0) { /* Drive motors */
    static bool layingDown = true;
    /* Drive motors */
    timer = micros();
    // If the robot is laying down, it has to be put in a vertical position before it starts balancing
    // If it's already balancing it has to be ±45 degrees before it stops trying to balance
    if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
      layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
      //stopAndReset();
    } else {
      layingDown = false; // It's no longer laying down
      //updatePID(cfg.targetAngle, targetOffset, turningOffset, (float)(timer - pidTimer) / 1000000.0f);
    }
    pidTimer = timer;
  }  /* Drive motors */

  if (0) { /* Update encoders */
    static int32_t lastWheelPosition; // Used to calculate the wheel velocity
    static int32_t wheelVelocity; // Wheel velocity based on encoder readings
    static int32_t targetPosition; // The encoder position the robot should be at
    static bool stopped;
    static uint8_t batteryCounter;

    timer = millis();
    if (timer - encoderTimer >= 100) { // Update encoder values every 100ms
      encoderTimer = timer;
      int32_t wheelPosition = getWheelsPosition();
      wheelVelocity = wheelPosition - lastWheelPosition;
      lastWheelPosition = wheelPosition;
      //BlueTooth.print(wheelPosition);BlueTooth.print('\t');BlueTooth.print(targetPosition);BlueTooth.print('\t');BlueTooth.println(wheelVelocity);
      if (abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
        targetPosition = wheelPosition;
        stopped = true;
      }

      batteryCounter++;
      if (batteryCounter >= 10) { // Measure battery every 1s
        batteryCounter = 0;
        batteryVoltage = (float)analogRead(A5) / 63.050847458f; // is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
        if (batteryVoltage < 10.2 && batteryVoltage > 5) {// Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
          beep.beep();
        } else {
          beep.beep();  beep.beep();
        }
      }
    }
  }  /* Update encoders */
}

void imuSetup() {
  int retry = 1;
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  while (i2cRead(0x75, i2cBuffer, 1)) {
    delay(1000);
    if (retry++ >= 3) {
      imuReady = false;
      beep.beep();
      beep.beep();
#ifdef __DEBUG__
      Console.print(F("Error, imu not found!"));
#endif
      delay(1000);
      return;
    }
  }
  if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
#ifdef __DEBUG__
    Console.print(F("Error, reading sensor"));
#endif
    beep.beep();
    beep.beep();
    imuReady = false;
    return;
  }
  while (i2cWrite(0x6B, 0x80, true)); // Reset device, this resets all internal registers to their default values
  do {
    while (i2cRead(0x6B, i2cBuffer, 1));
  } while (i2cBuffer[0] & 0x80); // Wait for the bit to clear
  delay(5);
  while (i2cWrite(0x6B, 0x09, true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
#if 1
  i2cBuffer[0] = 1; // Set the sample rate to 500Hz - 1kHz/(1+1) = 500Hz
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
#else
  i2cBuffer[0] = 15; // Set the sample rate to 500Hz - 8kHz/(15+1) = 500Hz
  i2cBuffer[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
#endif
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cBuffer, 4, true)); // Write to all four registers at once

  delay(100); // Wait for the sensor to get ready

  /* Set Kalman and gyro starting angle */
  //while (i2cRead(0x3D, i2cBuffer, 4));
  if (i2cRead(0x3D, i2cBuffer, 4)) {
    imuReady = false;
    return;
  }
  int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;
  kalman.setAngle(accAngle); // Set starting angle
  pitch = accAngle;
  gyroAngle = accAngle;
  /* Calibrate gyro zero value */
  //while (calibrateGyro()); // Run again if the robot is moved while calibrating
#ifdef __DEBUG__
  Console.println("imu ready!");
  delay(2000);
#endif
  imuReady = true;

}

void printValues() {
  if (sendPairConfirmation) {
    sendPairConfirmation = false;

    BlueTooth.println(F("PC"));
  } else if (sendPIDValues) {
    sendPIDValues = false;

    BlueTooth.print(F("P,"));
    BlueTooth.print(cfg.P);
    BlueTooth.print(F(","));
    BlueTooth.print(cfg.I);
    BlueTooth.print(F(","));
    BlueTooth.print(cfg.D);
    BlueTooth.print(F(","));
    BlueTooth.println(cfg.targetAngle);
  } else if (sendSettings) {
    sendSettings = false;

    BlueTooth.print(F("S,"));
    BlueTooth.print(cfg.backToSpot);
    BlueTooth.print(F(","));
    BlueTooth.print(cfg.controlAngleLimit);
    BlueTooth.print(F(","));
    BlueTooth.println(cfg.turningLimit);
  } else if (sendInfo) {
    sendInfo = false;

    BlueTooth.print(F("I,"));
    BlueTooth.print(version);
    BlueTooth.print(F(","));
    BlueTooth.print(eepromVersion);

#if defined(__AVR_ATmega644__)
    BlueTooth.println(F(",ATmega644"));
#elif defined(__AVR_ATmega1284P__)
    BlueTooth.println(F(",ATmega1284P"));
#elif defined(__AVR_ATmega328P__)
    BlueTooth.println(F(",ATmega328P"));
#else
    BlueTooth.println(F(",Unknown"));
#endif
  } else if (sendKalmanValues) {
    sendKalmanValues = false;

    BlueTooth.print(F("K,"));
    BlueTooth.print(kalman.getQangle(), 4);
    BlueTooth.print(F(","));
    BlueTooth.print(kalman.getQbias(), 4);
    BlueTooth.print(F(","));
    BlueTooth.println(kalman.getRmeasure(), 4);
  } else if (sendIMUValues && millis() - imuTimer > 50) { // Only send data every 50ms
    imuTimer = millis();

    BlueTooth.print(F("V,"));
    BlueTooth.print(accAngle);
    BlueTooth.print(F(","));
    BlueTooth.print(gyroAngle);
    BlueTooth.print(F(","));
    BlueTooth.println(pitch);
  } else if (sendStatusReport && millis() - reportTimer > 500) { // Send data every 500ms
    reportTimer = millis();

    BlueTooth.print(F("R,"));
    BlueTooth.print(batteryVoltage);
    BlueTooth.print(F(","));
    BlueTooth.println((float)reportTimer / 60000.0f);

  }
}


void setValues(char *input) {
  if (input[0] == 'A' && input[1] == ';') { // Abort
    //stopAndReset();
    while (BlueTooth.read() != 'C');
  } else if (input[0] == 'A' && input[1] == 'C') {// Accelerometer calibration
    //calibrateAcc();
  } else if (input[0] == 'M' && input[1] == 'C') {// Motor calibration
    //calibrateMotor();
  } else if (input[0] == 'G') { // The different application sends when it needs the PID, settings or info
    if (input[1] == 'P') // Get PID Values
      sendPIDValues = true;
    else if (input[1] == 'S') // Get settings
      sendSettings = true;
    else if (input[1] == 'I') // Get info
      sendInfo = true;
    else if (input[1] == 'K') // Get Kalman filter values
      sendKalmanValues = true;
  } else if (input[0] == 'S') { // Set different values
    /* Set PID and target angle */
    if (input[1] == 'P') {
      strtok(input, ","); // Ignore 'P'
      cfg.P = atof(strtok(NULL, ";"));
    } else if (input[1] == 'I') {
      strtok(input, ","); // Ignore 'I'
      cfg.I = atof(strtok(NULL, ";"));
    } else if (input[1] == 'D') {
      strtok(input, ","); // Ignore 'D'
      cfg.D = atof(strtok(NULL, ";"));
    } else if (input[1] == 'T') { // Target Angle
      strtok(input, ","); // Ignore 'T'
      cfg.targetAngle = atof(strtok(NULL, ";"));
    } else if (input[1] == 'K') { // Kalman values
      strtok(input, ","); // Ignore 'K'
      cfg.Qangle = atof(strtok(NULL, ","));
      cfg.Qbias = atof(strtok(NULL, ","));
      cfg.Rmeasure = atof(strtok(NULL, ";"));
    } else if (input[1] == 'A') { // Controlling max angle
      strtok(input, ","); // Ignore 'A'
      cfg.controlAngleLimit = atoi(strtok(NULL, ";"));
    } else if (input[1] == 'U') { // Turning max value
      strtok(input, ","); // Ignore 'U'
      cfg.turningLimit = atoi(strtok(NULL, ";"));
    } else if (input[1] == 'B') { // Set Back To Spot
      if (input[3] == '1')
        cfg.backToSpot = 1;
      else
        cfg.backToSpot = 0;
    }
    //updateConfig();
  } else if (input[0] == 'I') { // IMU transmitting states
    if (input[1] == 'B') {// Begin sending IMU values
      sendIMUValues = true; // Start sending output to application
#ifdef __DEBUG__
      Console.println("Graph start");
#endif
    } else if (input[1] == 'S') {// Stop sending IMU values
      sendIMUValues = false; // Stop sending output to application
#ifdef __DEBUG__
      Console.println("Graph stop");
#endif
    }
  } else if (input[0] == 'R') { // Report states
    if (input[1] == 'B') {// Begin sending report values
      sendStatusReport = true; // Start sending output to application
#ifdef __DEBUG__
      Console.println("Info start");
#endif
    } else if (input[1] == 'S') { // Stop sending report values
      sendStatusReport = false; // Stop sending output to application
#ifdef __DEBUG__
      Console.println("Info stop");
#endif
    }
  } else if (input[0] == 'C') { // Commands
    if (input[1] == 'S') {// Stop
      //steer(stop);
    } else if (input[1] == 'J') { // Joystick
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'J'
      sppData1 = atof(strtok(NULL, ",")); // x-axis
      sppData2 = atof(strtok(NULL, ";")); // y-axis
#ifdef __DEBUG__
      Console.print("joystick x="); Console.print(sppData1); Console.print(",y="); Console.println(sppData2);
#endif
      //steer(joystick);
    } else if (input[1] == 'M') { // IMU

      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'M'
      sppData1 = atof(strtok(NULL, ",")); // Pitch
      sppData2 = atof(strtok(NULL, ";")); // Roll
#ifdef __DEBUG__
      Console.print("motion pitch="); Console.print(sppData1); Console.print(", roll="); Console.println(sppData2);
#endif
      //steer(imu);
    } else if (input[1] == 'R') {
#ifdef __DEBUG__
      Console.println("restore EEPROM call");
#endif
      restoreEEPROMValues(); // Restore the default EEPROM values
      sendPIDValues = true;
      sendKalmanValues = true;
      sendSettings = true;
    }
  }
}


void checkBlueToothData() {
  static char dataInput[30];
  if (BlueTooth.available()) {
    int input = BlueTooth.read();
    //    if (input == 'm') {
    //      printMenu();
    //      return;
    //    }
    dataInput[0] = static_cast<char> (input); // Intentional cast
    delay(2); // Wait for rest of data

    uint8_t i = 1;
    while (1) {
      input = BlueTooth.read();
      if (input == -1) // Error while reading the string
        return;

      dataInput[i] = static_cast<char> (input); // Intentional cast
      if (dataInput[i] == ';') // Keep reading until it reads a semicolon
        break;
      if (++i >= sizeof(dataInput) / sizeof(dataInput[0])) // String is too long
        return;
    }
#ifdef __DEBUG__
    //    if ( dataInput[0] != 'C' && dataInput[1] != 'S') {
    //      Console.print("data coming: "); Console.println( dataInput[0]);
    //    }
#endif
    setValues(dataInput);

  }
}

void printMenu() {

  Console.println(F("\r\n========================================== Menu ==========================================\r\n"));

  Console.println(F("m\t\t\t\tSend to show this menu\r\n"));

  Console.println(F("A;\t\t\t\tSend to abort. Send 'C' again to continue\r\n"));

  Console.println(F("AC;\t\t\t\tSend to calibrate the accelerometer"));
  Console.println(F("MC;\t\t\t\tSend to calibrate the motors\r\n"));

  Console.println(F("GP;\t\t\t\tGet PID values"));
  Console.println(F("GK;\t\t\t\tGet Kalman filter values"));
  Console.println(F("GS;\t\t\t\tGet settings values"));
  Console.println(F("GI;\t\t\t\tGet info values\r\n"));

  Console.println(F("SP,Kp;\t\t\t\tUsed to set the Kp value"));
  Console.println(F("SI,Ki;\t\t\t\tUsed to set the Ki value"));
  Console.println(F("SD,Kd;\t\t\t\tUsed to set the Kd value"));
  Console.println(F("ST,targetAngle;\t\t\tUsed to set the target angle"));
  Console.println(F("SK,Qangle,Qbias,Rmeasure;\tUsed to set the Kalman filter values"));
  Console.println(F("SA,angle;\t\t\tUsed to set the maximum controlling angle"));
  Console.println(F("SU,value;\t\t\tUsed to set the maximum turning value"));
  Console.println(F("SB,value;\t\t\tUsed to set the back to spot value (true = 1, false = 0)\r\n"));

  Console.println(F("IB;\t\t\t\tStart sending IMU values"));
  Console.println(F("IS;\t\t\t\tStop sending IMU values"));
  Console.println(F("RB;\t\t\t\tStart sending report values"));
  Console.println(F("RS;\t\t\t\tStop sending report values\r\n"));

  Console.println(F("CS;\t\t\t\tSend stop command"));
  Console.println(F("CJ,x,y;\t\t\t\tSteer robot using x,y-coordinates"));
  Console.println(F("CM,pitch,roll;\t\t\tSteer robot using pitch and roll"));
#ifdef ENABLE_WII
  Console.println(F("CPW;\t\t\t\tStart paring sequence with Wiimote"));
#endif
#ifdef ENABLE_PS4
  Console.println(F("CPP;\t\t\t\tStart paring sequence with PS4 controller"));
#endif
  Console.println(F("CR;\t\t\t\tRestore default EEPROM values\r\n"));

#ifdef ENABLE_SPEKTRUM
  Console.println(F("BS;\t\t\t\tBind with Spektrum satellite receiver"));
#endif
  Console.println(F("\r\n==========================================================================================\r\n"));
}

static const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
static const uint16_t I2C_TIMEOUT = 100; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
#ifdef __DEBUG__
    Console.print(F("i2cWrite failed: "));
    Console.println(rcode);
#endif
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
#ifdef __DEBUG__
    Console.print(F("i2cRead failed: "));
    Console.println(rcode);
#endif
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }

  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Console.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

int32_t getWheelsPosition() {
  return 45;
}

/* EEPROM Address Definitions */
static const uint8_t initFlagsAddr = 0; // Set the first byte to the EEPROM version
static const uint8_t configAddr = 1; // Save the configuration starting from this location

bool checkInitializationFlags() {
  uint8_t initFlag;
  EEPROM_readAnything(initFlagsAddr, initFlag);
  if (initFlag != eepromVersion) { // Check if the EEPROM version matches the current one
    restoreEEPROMValues();
    EEPROM_updateAnything(initFlagsAddr, eepromVersion); // After the default values have been restored, set the flag
    return true; // Indicate that the values have been restored
  }
  return false;
}

void readEEPROMValues() {
  EEPROM_readAnything(configAddr, cfg);

  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void updateConfig() {
  EEPROM_updateAnything(configAddr, cfg);

  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void restoreEEPROMValues() {
  cfg.P = 9.0f;
  cfg.I = 2.0f;
  cfg.D = 3.0f;

  cfg.targetAngle = 180.0f;
  cfg.backToSpot = 1;
  cfg.controlAngleLimit = 7;
  cfg.turningLimit = 25;

  cfg.Qangle = 0.001f;
  cfg.Qbias = 0.003f;
  cfg.Rmeasure = 0.03f;

  cfg.accYzero = cfg.accZzero = 0.0f;
  cfg.leftMotorScaler = cfg.rightMotorScaler = 1.0f;

  cfg.bindSpektrum = false;

  updateConfig();
}
