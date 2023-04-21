// UpGoer5Instrumentation
// Version: Beta5
// Author: Bob Parker
// Date: 2/22/2023
// Tested: 
//
// Code for a model rocket instrumentation package. Measures and logs acceleration and turning
// rates along three axis. Measures and logs atmospheric pressure and computed altitude above
// ground. Measures and logs air temperature.
// In the loop function, getting new sensor readings and logging them to the SD card is taking
// between 15ms and 18ms. When logging is turned off then getting the sensor readings is taking
// about 850usec. TODO: Make new timing measurements for the SAMD21 chip

// Uses an SD card to write two files:
// eventFile.txt: Contains log of events including battery voltage, errors, state transitions, etc.
// dataFile.txt: Contains the sensor readings from a flight. Actual file name uses data/time to create a 
//               unique name.

// Peripherals used
// Internal peripherals
// TC3: 8-bit counter provides periodic interrupts to trigger sensor measurements
// ADC: Measure battery voltage
// SPI: Two SPI busses implemented. Bluetooth modem/SD card SPI bus and sensor SPI bus
// I2C: One I2C bus for the PCF8523 Real Time Clock
// External peripherals
// LM4040 Voltage Reference provides reference voltage for battery voltage measurements with ADC
// BMP390 Altimeter/Temperature sensor
// LSM6DSO32 Accelerometer/Gyro sensor
// Adafruit Adalogger feather wing with SD card and PCF8523 Real Time Clock with battery backup

// Beta 4:
// Added battery monitoring and status via NEO_PIXEL LED
// Added break-wire GPIO input to start data logging
// Added arm GPIO input
// Added ability to change settings via SD Card setting.txt file
// Added time limit for data logging
// Added data logging stop after period of no activity as measured by accelerometer/gyro
// Added Real Time Clock peripheral to support log file unique naming

// Beta 5:
// Ported to ATSAMD21 Arm M0+ on Adafruit Feather M0 Bluefruit LE
// Removed breakwire and arm loop hardware implementation. Arming is now done via ARM command
// sent via bluetooth. Launch event is now determined by looking at the vertical axis accelerometer.
// Discontinued the use of the RTC for periodic interrupts. Now using TC3.
// Discontinued using TCs for 32 bit millisecond counter. Now using millis() function

// ==============================  Include Files  ==================================
#include <SPI.h>                      // Sensor and micro SD card communication
#include <SD.h>                       // SD card library functions
#include <BMP3XX.h>                   // BMP390 sensor library code
#include <LSM6DSO32.h>                // LSM6DSO32 sensor library code
#include "wiring_private.h"           // pinPeripheral() function
#include "RTClib.h"                   // Real Time Clock library by Adafruit
#include "Adafruit_BLE.h"             // Bluetooth link library
#include "Adafruit_BluefruitLE_SPI.h" // SPI library for bluetooth module


// ===============================  Constants  =====================================
#define DEBUG // Uncomment to enable debug comments printed to the console
//#define PRINT_LOG_DATA  // Uncomment to enable printing of log data to the console
const String VERSION = "Beta 5.03";

// Arduino pin assignments
const uint8_t BMP390_CS = 5;
const uint8_t LSM6DSO32_CS = 6;
const uint8_t SD_CARD_CS = A5;   // Adalogger board must be modified to change CS from 10 (default) to A5
const uint8_t LED = 13;
const uint8_t BLUEFRUIT_SPI_CS = 8;
const uint8_t BLUEFRUIT_SPI_IRQ = 7;
const uint8_t BLUEFRUIT_SPI_RST = 4;
const uint8_t SENSOR_SPI_CLK = 12;
const uint8_t SENSOR_SPI_MOSI = 10;
const uint8_t SENSOR_SPI_MISO = 11;
const uint8_t BATTERY_MONITOR = 9;
const uint8_t PARACHUTE_RELEASE = A4;

// Other Constants
const uint8_t TC3_INT_PERIOD = 10;          // RTC interrupt period in milliseconds
const uint16_t LED_PPS_FAULT = 100;          // LED pulse rate while in FAULT state
const uint16_t LED_PPS_ARMED = 200;          // LED pulse rate while in ARMED state
const uint16_t LED_PPS_POST_FLIGHT = 10000;  // LED pulse period (ms) in POST_FLIGHT state
const int LED_PULSE_WIDTH = 50;             // LED pulse width in all states
const float SEALEVELPRESSURE = 1013.25;     // Sea level pressure in hpa
const float METERS_TO_FEET = 3.28084;       // Conversion from meters to feet
const long LOG_FILE_DURATION = 10;          // How long sensor data is logged to the log file in seconds
const float BAT_VOLT_LOW = 3.65;            // LiPo battery voltage below this level is at low charge
const uint16_t BAT_MEAS_PER  = 6000;       // LiPo battery voltage measurement period (ms)
const float NO_MOTION_UPPER = 0.1;          // Rate below which we are not moving
const float NO_MOTION_LOWER = -0.1;         // Rate above which we are not moving
const uint16_t TIME_OUT = 30;               // Number of seconds after which we will assume we have landed.
const float LAUNCH_ACCEL = 1.1;             // Threshold for launch determination in G's
const uint16_t PARACHUTE_SIGNAL_DURATION = 2000;  // Duration of parachute release signal in milliseconds

enum states {START_UP, FAULT, READY, ARMED, LOGGING, POST_FLIGHT};
enum errors {NONE, GYRO, ACCEL, ALT, BATTERY};

// =======================  Global Variables/Objects  ===============================
volatile bool rtcInterruptFlag = false;   // Flag set by rtc interrupt handler
 
// Driver class objects
SPIClass sensorSPI(&sercom1, 11, 12, 10, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0); // The sensor SPI bus instance
BMP3XX altimeter;  // Altimeter sensor object instance
LSM6DSO32 accelerometer_gyro;  //Accelerometer/Gyro sensor object instance
RTC_PCF8523 realTimeClock;
Adafruit_BluefruitLE_SPI bleRadio(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);  // Object manages the Bluetooth LE peripheral

double temperature = 0.0;       // Temperature in degrees C
double pressure = 0.0;          // Pressure in hpa
float altitude = 0.0;           // Altitude in meters
float baseAltitude = 0.0;       // The altitude of the rocket launch pad
int8_t altimeterErrorCode = 0;  // Result code returned from BMP390 API calls
bool readyForLaunch = false;
bool dataFileOpen = false;
File dataFile;                  // The data file object
File logFile;                   // The log file object
uint32_t launchTime = 0;        // Time at launch in millisends
float lipoBatteryVoltage = 0;   // Battery voltage of LiPo battery as measured by ADC
states state = START_UP;        // The current state
errors error = NONE;            // Error
float maxAcceleration = 0;      // The maximum axial acceleration measured during flight
float maxAltitude = 0;          // The maximum altitude measured during flight


// =======================  TC3 Interrupt Handler  ===================================
// TC3 will generate an interrupt at a rate determined by TC3_INT_PERIOD to drive the 
// measurement/logging cycle. The required interrupt handler name is specified in CMSIS 
// file samd21g18a.h. The interrupt handler will simply set a flag so the loop code 
// can respond accordingly.
void TC3_Handler(void) {
  // The first step is to clear the interrupt flag
  // The only interrupt enabled is MC0 so we don't need to read the interrupt flag
  // register to see which interrupt triggered
  // Writing a 1 to the flag bit will clear the interrupt flag
  TC3->COUNT8.INTFLAG.bit.MC0 = 1;

  // Now set the rtcInterruptFlag
  rtcInterruptFlag = true;
}

// =============================  Setup Function  ===================================
void setup() {
  // Initialize the Serial Console interface (used for debugging only)
#ifdef DEBUG
  Serial.begin(115200); // Make sure the Serial Monitor is set to 115200 baud
  while (!Serial) {
      ; 
  }
  Serial.println("UpGoer5 Intrumentation");
  Serial.print("Version: ");
  Serial.println(VERSION);
    
#endif

  // ======================== Bluetooth Radio Initialization =========================
  
#ifdef DEBUG
  Serial.print(F("Initializing Bluetooth LE module: "));
#endif
  if (!bleRadio.begin(false)) {
#ifdef DEBUG
    Serial.println(F("Could not initialize Bluefruit LE module"));
#endif
  }
#ifdef DEBUG
  Serial.println(F("OK"));
#endif
  // Disable command echo
  bleRadio.echo(false);
  // Change LED activity mode
  bleRadio.sendCommandCheckOK("AT+HWModeLED=MODE");

  // Wait for bluetooth radio connection
  while (!bleRadio.isConnected()) {
    delay(500);
  }

  delay(10000);

  bleRadio.println("AT+BLUEARTTX=Initializing");

#ifdef DEBUG
  Serial.println(F("Bluetooth link connected"));
#endif

  // ============================ Configure GPIO Pins ==================================

  // Configure the GPIO pins
  pinMode(LED, OUTPUT);   // Use this digital output to observe the TC3 interrupt
  PORT->Group[0].OUTCLR.reg = PORT_PA17; // Turn off the on-board LED
  pinMode(PARACHUTE_RELEASE, OUTPUT);
  digitalWrite(PARACHUTE_RELEASE, LOW);

  // ======================  Initialize the Sensor SPI Bus  ============================
  sensorSPI.begin();
  // These calls need to be here because the call to sensorSPI.begin() above will 
  // configure the pins to analog peripheral functions rather than the SERCOM-ALT that
  // we want.
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);

  // ============================ SD Card Library Setup ================================
  
  if (!SD.begin(SD_CARD_CS)) {
#ifdef DEBUG
    Serial.println("SD card failed to initialize");
#endif
    bleRadio.println("AT+BLEUARTTX=SD card initilization failure");
    while(1); // Hang here
  } else {
#ifdef DEBUG
    Serial.println("SD card initialized");
#endif
    bleRadio.println("AT+BLEUARTTX=SD card initialized");
  }

  // ======================  SAMD21 Generic Clock Setup  ====================
  // Configure Generic Clock 6. It will we used as the source clock for the
  // TC3 peripheral. Generic Clock 6 will output 16KHz clock.
  // ========================================================================

  GCLK->GENDIV.reg =  GCLK_GENDIV_ID(0x06) |     // Generic Clock 6
                      GCLK_GENDIV_DIV(0x02);     // Divide by 2 (16KHz)
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |       // Enable generic clock generator
                      GCLK_GENCTRL_SRC_XOSC32K | // External 32KHz crystal source
                      GCLK_GENCTRL_ID(0x06);     // Generic Clock 6
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |   // Generic Clock 6
                      GCLK_CLKCTRL_ID_TCC2_TC3 | // TC3 will use generic clock 6
                      GCLK_CLKCTRL_CLKEN;        // Enable generic clock

#ifdef DEBUG
  Serial.println("Generic clock initialized");
#endif

  // =============================  TC3 Setup  ==============================
  // TC3 is used to generate a periodic interrupt to drive all activity.
  // ========================================================================
  // Enable the TC3 bus clock
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;

  //Disable TC3
  TC3->COUNT8.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);

  // Enable the TC3 OVF (overflow) interrupt
  TC3->COUNT8.INTENSET.reg = TC_INTENSET_OVF;

  // Configure TC3 Control A register settings
  // PRESCALER set to divide by 16
  // MODE set to 8-bit counter
  TC3->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 |      // 8-bit counter mode
                          TC_CTRLA_WAVEGEN_NPWM |     // PER register used as TOP value
                          TC_CTRLA_PRESCALER_DIV16 |  // Divide GCLK by 16 (1KHz counter clock)
                          TC_CTRLA_PRESCSYNC_GCLK;    // Wrap around on next GCLK tick

  //Set the PER register value
  TC3->COUNT8.PER.reg = TC3_INT_PERIOD;
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);

  // Enable the TC3 interrupt in the NVIC
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

#ifdef DEBUG
  Serial.println("TC3 initialized");
#endif

  // =============================== ADC Setup ==========================
  // The ADC is used to measure the LiPo battery voltage level
  // The ADC is configured to use an external 2.048V reference
  // ====================================================================
  
  // Configure ADC to measure the LiPo battery power source
  // The LiPo battery is connected to D9 via a divide by 2 voltage divider
  // Will need to use an external reference of 2.048V connected to AREF 
  
  analogReadResolution(12);
  analogReference(AR_EXTERNAL);
  
#ifdef DEBUG
  Serial.println("ADC initialized");
#endif



  // ======================== Real Time Clock Setup =====================
  // The Real Time Clock on the Adalogger Feather Wing is used to create
  // a unique file name for the data log file. The file name uses the 
  // current date and minute to create a file name.
  // ====================================================================
  if (!realTimeClock.begin()) {
#ifdef DEBUG
    Serial.println("Real Time Clock not found");
#endif
    bleRadio.println("AT+BLEUARTTX=Real Time Clock not found");
    state = FAULT;
    logActivity("ERROR", "Real Time Clock not found");
  }

  if (!realTimeClock.initialized() || realTimeClock.lostPower())  {
    // Need to initialize the real time clock date and time
#ifdef DEBUG
    Serial.println("Initializing real time clock date and time");
#endif
    logActivity("STATUS", "Initializing real time clock date and time");
    delay(2000);  // Wait for real time clock crystal to stabilize befor calling adust()
    realTimeClock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Start the Real Time Clock
  realTimeClock.start();
  logActivity("STATUS", "Real Time Clock started");
  bleRadio.println("AT+BLEUARTTX=Real Time Clock started");

#ifdef DEBUG
  Serial.println("Real time clock initialized");
#endif



  // ===================== LSM6DSO32 Sensor Setup =======================
  // The LSM6DSO32 is a combined accelerometer and gyro. It is used to 
  // measure acceleration and turning rates along the 3 axis. The X-Axis
  // is aligned along the rockets longitudinal axis (top to bottom).
  // ====================================================================

  // Initialize the accelerometer/gyro sensor
#ifdef DEBUG
  Serial.println("Initializing accelerometer/gyro sensor");
#endif
  if (!accelerometer_gyro.begin(LSM6DSO32_CS, &sensorSPI)) {
#ifdef DEBUG
    Serial.println("Error initializing accelerometer/gyro sensor");
#endif
    state = FAULT;
    logActivity("ERROR", "Error initializing the accelerometer/gyro sensor");
    bleRadio.println("AT+BLEUARTTX=Inertial sensor initialization error");
  } else {
    bleRadio.println("AT+BLEUARTTX=Inertial sensors initialized");
#ifdef DEBUG
    Serial.println("Accelerometer/Gyro sensor initilaized");
#endif
  }

  // Configure accelerometer/gyro settings
  accelerometer_gyro.setAccelRange(LSM6DSO32_RANGE_32G);
  accelerometer_gyro.setGyroRange(LSM6DSO32_RANGE_500DPS);
  accelerometer_gyro.setAccelDataRate(LSM6DSO32_ACCEL_52_HZ);
  accelerometer_gyro.setGyroDataRate(LSM6DSO32_GYRO_52_HZ);

  logActivity("STATUS", "Accelerometer/gyro sensor initialized");

  // ====================== BMP390 Sensor Setup  ========================
  // The BMP390 is a barometric pressure sensor that is used to measure
  // air pressure and temperature. Altitude is computed from the air
  // pressure reading.
  // ====================================================================
  
#ifdef DEBUG
  Serial.println("Setup BMP390 Sensor");
#endif
  if(!altimeter.begin(BMP390_CS, &sensorSPI)) {
#ifdef DEBUG
    Serial.println("Error initializing Altimeter sensor");
#endif
    state = FAULT;
    logActivity("ERROR", "Error initializing the altimeter sensor");
    bleRadio.println("AT+BLEUARTTX=Altimeter initialization error");
  } else {
#ifdef DEBUG
    Serial.println("Accelerometer/Gyro sensor intiailaized");
#endif
  }
  
  // Set up oversampling and filter configuration
  altimeter.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  altimeter.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  altimeter.setOutputDataRate(BMP3_ODR_50_HZ);
  altimeterErrorCode = altimeter.writeSensorSettings(BMP3_NO_OVERSAMPLING, 
                                                     BMP3_OVERSAMPLING_8X, 
                                                     BMP3_IIR_FILTER_COEFF_3, 
                                                     BMP3_ODR_50_HZ);


  if (altimeterErrorCode == 0) {
#ifdef DEBUG
    Serial.println("Altimeter settings written");
#endif
  } else {
#ifdef DEBUG
    printAltimeterError("Error writing altimeter settings:", altimeterErrorCode);
#endif
    logActivity("ERROR", "Error writing altimeter settings");
  }

  // Set the power mode to Normal. It will start taking measurement now.
  altimeterErrorCode = altimeter.setPowerMode(BMP3_MODE_NORMAL);

  if (altimeterErrorCode == 0) {
#ifdef DEBUG
    Serial.println("Altimeter power mode written");
#endif
  } else {
#ifdef DEBUG
    printAltimeterError("Error writing altimeter power mode:", altimeterErrorCode);
#endif
    logActivity("ERROR", "Error writing altimeter power mode");
  }

  logActivity("STATUS", "Altimeter settings and power mode initialized");
  bleRadio.println(F("AT+BLEUARTTX=Altimeter sensor initialized"));
  

  // ======================= Ready State Check ===========================

  // Start-up is complete. Move to Ready state if no errors occurred during startup
  if (state != FAULT) {
    state = READY;
    logActivity("EVENT", "Transition to Ready state");
    bleRadio.println(F("AT+BLEUARTTX=READY - Not Armed"));
  }

  // Enable TC3. We will start getting TC3 interrupts at the TC3_INT_PERIOD
  TC3->COUNT8.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);
  
} // End setup code

// =======================  Loop Function  ===================================

void loop() {
  if (rtcInterruptFlag == true) {
    rtcInterruptFlag = false;
    // Check for the state
    switch (state) {
      case FAULT:
        // Note: Once we get in the FAULT state we are not getting out without a reset
        faultStateLoop();
        break;
      case READY:
        readyStateLoop();
        break;
      case ARMED:
        armedStateLoop();
        break;
      case LOGGING:
      loggingStateLoop();
        break;
      case POST_FLIGHT:
      postFlightStateLoop();
        break;
      default:
        break;
    }
  }
}

// ============================== Fault State Loop ==========================
// In the fault state the on-board LED is flashed at a high rate of 
// 1000/LED_PPS_FAULT pulses per second.
// No other processing or logging occurs.
// The application cannot exit the fault state.
// ==========================================================================
void faultStateLoop() {
  flashLED(LED_PPS_FAULT);

}

// ============================= Ready State Loop ===========================
// In the ready state the following activities occur
//   1. Read the LiPo Battery voltage every BAT_MEAS_PER milliseconds and 
//      log the voltage reading in the event log. If the battery voltage
//      falls below BAT_VOLT_LOW then the state is changed to fault state
//      and this event is noted in the event log and is reported via the
//      bluetooth radio.
//   2. Monitor the bluetooth radio link for an ARM command. If an ARM
//      command is received then transition to the armed state. Log the event 
//      in the event log and open the dataFile in preparation for logging data.
//   3. Before exiting to the armed state, compute the launch site elevation
//      for use during data acquisition.
// ==========================================================================
void readyStateLoop() {
  static uint16_t counter = 0;  // Used to compute when a battery check is needed

  accelerometer_gyro.readSensorData();
  
  // Check for an ARM event every time through the loop
  bleRadio.println("AT+BLEUARTRX");
  bleRadio.readline();
  if (!strcmp(bleRadio.buffer, "OK") == 0) {
    // Some data was received
    String message = bleRadio.buffer;
    message.toLowerCase();
    if (strcmp(message.c_str(), "arm") == 0 && accelerometer_gyro.getXAxisAccel() > 0.9) {
      // Arm command received
      state = ARMED;
      bleRadio.println("AT+BLEUARTTX=ARMED");
      logActivity("EVENT", "Armed");

      // Compute the launch point altitude
      delay(100);
      if (computeLaunchPointAlt() != 0) {
        // Go to fault state
        state = FAULT;
        logActivity("ERROR", "Error reading altimeter sensor data");
#ifdef DEBUG
        printAltimeterError("Error reading altimeter sensor data: ", altimeterErrorCode);
#endif
      }
      
      // Write the header row to the log file with labels for the data
      // File name to include the date from the Real Time Clock
      // Leave the file open. It will be closed later after all data is logged
      String headerRow = "Time(ms), Temp(degC), Pres(hPa), Alt(ft), X_Accel(Gs), Y_Accel(Gs), Z_Accel(Gs), X_Rate(dps), Y_Rate(dps), Z_Rate(dps)";
      openDataFile();
      // Write the header string to the data file
      if (dataFile) {
        dataFile.println(headerRow);
        // Leave the log file open to save time during logging of sensor readings
      } else {
        logActivity("ERROR", "SD Card write error");
    
#ifdef DEBUG
        Serial.println("SD card write error");
#endif
      }
    } // End arm command processing
  }

  // Check battery voltage and report via bluetooth radio
  counter++;
  if (counter >= (BAT_MEAS_PER / TC3_INT_PERIOD)) {
    counter = 0;  // Reset the counter
    lipoBatteryVoltage = readBatteryVoltage();

    // Log the battery reading in the event log
    String logString = "Battery voltage = ";
    logString += lipoBatteryVoltage;
    logString += " volts";
    logActivity("BATTERY", logString);
#ifdef DEBUG
    Serial.print("Battery Voltage: ");
    Serial.println(lipoBatteryVoltage);
#endif

    displayBatteryLevel();

    // Check if the battery voltage is low. Go to FAULT state if it is
    if (lipoBatteryVoltage < BAT_VOLT_LOW) {
      state = FAULT;
      logActivity("ERROR", "Battery voltage low");
      bleRadio.println("AT+BLEUARTTX=Battery Low");
      bleRadio.println("AT+BLEUARTTX=FAULT - Not Armed");
    }
  }
}

// ============================= Armed State Loop ===========================
// In the armed state the following activities occur
//   1. The on-board LED is flashed at a medium rate of 1000/LED_PPS_ARMED 
//      pulses per second.
//   2. Check for a disarm event in which case we transition to the ready
//      state. In the process, close the data logging file.
//   3. Check for a launch event in which case we take and log an initial set
//      of sensor readings before transitioning to the logging state.
// ==========================================================================
void armedStateLoop() {
  // Check for a DISARM event every time through the loop
  bleRadio.println("AT+BLEUARTRX");
  bleRadio.readline();
  if (!strcmp(bleRadio.buffer, "OK") == 0) {
    // Some data was received
    if (strcmp(bleRadio.buffer, "disarm") == 0) {
      // We are disarmed
      state = READY;
      bleRadio.println("AT+BLEUARTTX=READY");
      logActivity("EVENT", "Disarmed");
      closeDataFile();
      digitalWrite(LED, LOW);
    }
  }

  // Check for launch by reading the accelerometer
  if (isLaunched()) {
    // We are launched
    // Take the first sensor readings
    launchTime = millis();
    
    readAltimeterData();
    accelerometer_gyro.readSensorData();
    logData(launchTime);

    // Transistion to Data Logging state
    state = LOGGING;
    digitalWrite(LED, LOW); // Make sure the on-board LED is off
    logActivity("EVENT", "Launch");
    return;
  }
  
  
  // Flash LED
  flashLED(LED_PPS_ARMED);
}

// ============================= Logging State Loop =========================
// Data logging will progress in two different modes depending upon the value
// of the constant LOG_FILE_DURATION. If LOG_FILE_DURATION is equal to 0 then
// data logging continues until the model rocket has landed as determined by
// the angular rate readings. If LOG_FILE_DURATION is not equal to 0 then
// the logging continues only for the number of seconds specified by
// LOG_FILE_DURATION or until the model rocket lands.
// In the logging state the following activities occur
//   1. Measure 3-axis acceleration
//   2. Measure 3-axis angular rates
//   3. Measure temperature
//   4. Measure barometric pressure and compute altitude above ground level
//   5. Log all measured data to the data file
//   6. Detect apogee and trigger parachute release
// ==========================================================================
void loggingStateLoop() {
static uint16_t timeOutCounter = 0;
static uint16_t parachute_signal_timer = 0;
static bool parachute_released = false;
timeOutCounter++;

#ifdef DEBUG
  // Toggle D13 so we can measure when the interrupt processing starts
  PORT->Group[0].OUTTGL.reg = PORT_PA17;
#endif

  // Read the current time
  uint32_t timeInMS = millis();

  // Read the altimeter data
  readAltimeterData();

  // Check if we are at the apogee
  if(maxAltitude < 75) {
    if(altitude < (maxAltitude - 1)) {
      digitalWrite(PARACHUTE_RELEASE, HIGH); 
      parachute_released = true;
    }
  } else {
    if(altitude < (maxAltitude - 10)) {
      digitalWrite(PARACHUTE_RELEASE, HIGH); 
      parachute_released = true;
    }
  }

  // Read the accelerometer/gyro sensor data
  accelerometer_gyro.readSensorData();

  // Update maximum readings
  if (accelerometer_gyro.getXAxisAccel() > maxAcceleration) {
    maxAcceleration = accelerometer_gyro.getXAxisAccel();
  }
  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  }
  
  if ((LOG_FILE_DURATION == 0)  || ((timeInMS - launchTime) <= (LOG_FILE_DURATION * 1000))) {
    
      //Log the data
      logData(timeInMS);
  }

  // Check if we have landed (no more motion)
      if ((accelerometer_gyro.getXAxisRate() < NO_MOTION_UPPER) &&
          (accelerometer_gyro.getXAxisRate() > NO_MOTION_LOWER) && 
          (accelerometer_gyro.getYAxisRate() < NO_MOTION_UPPER) &&
          (accelerometer_gyro.getYAxisRate() > NO_MOTION_LOWER) &&
          (accelerometer_gyro.getZAxisRate() < NO_MOTION_UPPER) && 
          (accelerometer_gyro.getZAxisRate() > NO_MOTION_LOWER)) {
            state = POST_FLIGHT;
            if (dataFileOpen) {
              closeDataFile();
            }
            logActivity("EVENT", "Landed");
            // Log the maximum acceleration and altitude
            String logString = "Maximum acceleration = ";
            logString += maxAcceleration;
            logString += " Gs";
            logActivity("STATUS", logString);
            logString = "Maximum altitude = ";
            logString += maxAltitude;
            logString += " ft";
            logActivity("STATUS", logString);
          }

        // Check for timeout
        if (timeOutCounter * TC3_INT_PERIOD > TIME_OUT * 1000) {
          state = POST_FLIGHT;
          if (dataFileOpen) {
            closeDataFile();
          }
          logActivity("EVENT", "Landed");
          // Log the maximum acceleration and altitude
          String logString = "Maximum acceleration = ";
          logString += maxAcceleration;
          logString += " Gs";
          logActivity("STATUS", logString);
          logString = "Maximum altitude = ";
          logString += maxAltitude;
          logString += " ft";
          logActivity("STATUS", logString);
        }

    // Checks if we need to turn off the parachute release signal
    parachute_signal_timer++;
    if(parachute_signal_timer >= (PARACHUTE_SIGNAL_DURATION / TC3_INT_PERIOD)) {
      digitalWrite(PARACHUTE_RELEASE, LOW);
    }

#ifdef DEBUG
  // Toggle the D13 so we can measure how long the interrupt processing takes
  PORT->Group[0].OUTTGL.reg = PORT_PA17;
#endif
}

// ========================== Post Flight State Loop =========================
// In the post flight state the on-board LED is flashed at 
// 1000/LED_PPS_POST_FLIGHT pulses per second.
// No other processing or logging occurs.
// The application cannot exit the post flight state.
// ==========================================================================
void postFlightStateLoop() {
  static bool sent = false;
  flashLED(LED_PPS_POST_FLIGHT);
  if (bleRadio.isConnected() && sent != true) {
    delay(5000);
    bleRadio.print("AT+BLEUARTTX=");
    bleRadio.print("Max Alt: ");
    bleRadio.println(maxAltitude);
    
    bleRadio.print("AT+BLEUARTTX=");
    bleRadio.print("Max Accel: ");
    bleRadio.println(maxAcceleration);
    sent = true;
  }
}


//  =========================================================================
//  ======================= Helper Functions  ===============================
//  =========================================================================

// ===================== printAltimeterError ================================
// Prints an altimeter related error string to the Serial monitor console
// ==========================================================================
void printAltimeterError(String preamble, int8_t errorCode) {
  Serial.print(preamble);
  switch (errorCode) {
    case INVALID_POWER_MODE:
      Serial.println("Invalid power mode");
      break;
    case INVALID_TEMP_OS_VALUE:
      Serial.println("Invalid temperature oversampling value");
      break;
    case INVALID_PRES_OS_VALUE:
      Serial.println("Invalid pressure oversampling value");
      break;
    case INVALID_FILTER_COEF_VALUE:
      Serial.println("Invalid IIR Filter coefficient value");
      break;
    case INVALID_ODR_VALUE:
      Serial.println("Invalid output data rate value");
      break;
    case BMP3_E_NULL_PTR:
      Serial.println("BMP3 API Error - Null pointer");
      break;
    case BMP3_E_DEV_NOT_FOUND:
      Serial.println("BMP3 API Error - Sensor not found");
      break;
    case BMP3_E_INVALID_ODR_OSR_SETTINGS:
      Serial.println("BMP3 API Error - Invalid sensor settings");
      break;
    case BMP3_E_CMD_EXEC_FAILED:
      Serial.println("BMP3 API Error - Sensor command failed to execute");
      break;
    case BMP3_E_CONFIGURATION_ERR:
      Serial.println("BMP3 API Error - Sensor configuration error");
      break;
    case BMP3_E_INVALID_LEN:
      Serial.println("BMP3 API Error - Tried to read registers with length set to 0");
      break;
    case BMP3_E_COMM_FAIL:
      Serial.println("BMP3 API Error - Sensor communication error");
      break;
    case BMP3_E_FIFO_WATERMARK_NOT_REACHED:
      Serial.println("BMP3 API Error - FIFO Watermark error");
      break;
    case SPI_CNTRL_OBJECT_INIT_FAILED:
      Serial.println("Not able to initialize the SPI bus controller opject");
      break;
    default:
      Serial.print("(");
      Serial.print(errorCode);
      Serial.print(")");
      Serial.println("Undefined error");
      break;
  }
}

// =============================== logData ==================================
// Logs the sensor readings to the SD card with a time stamp
// ==========================================================================
void logData(uint32_t time) {
  // Empty string for log data
  String logData = "";

  // Create the logData string
  logData += time;
  logData += ",";
  logData += temperature;
  logData += ",";
  logData += pressure;
  logData += ",";
  logData += altitude;
  logData += ",";
  logData += accelerometer_gyro.getXAxisAccel();
  logData += ",";
  logData += accelerometer_gyro.getYAxisAccel();
  logData += ",";
  logData += accelerometer_gyro.getZAxisAccel();
  logData += ",";
  logData += accelerometer_gyro.getXAxisRate();
  logData += ",";
  logData += accelerometer_gyro.getYAxisRate();
  logData += ",";
  logData += accelerometer_gyro.getZAxisRate();
  logData += "\n";

  // Write the log data to the SD card
  openDataFile();
  if (dataFile) {
    dataFile.println(logData);
    //dataFile.close();
  } else {
#ifdef DEBUG
    Serial.println("SD card write error");
#endif
  }
  
#ifdef PRINT_LOG_DATA
  Serial.print(logData);
#endif
}

// ======================= readAltimeterData ================================
// Reads the altimeter data and computes the altitude above ground level
// ==========================================================================
void readAltimeterData() {
  altimeterErrorCode = altimeter.readSensorData();
  if (altimeterErrorCode != 0) {
      
#ifdef DEBUG
    printAltimeterError("Error reading altimeter sensor data: ", altimeterErrorCode);
#endif
      
  temperature = -100;
  pressure = -100;
  altitude = -100;
  } else {
    temperature = altimeter.getTemperature();
    pressure = altimeter.getPressure() / 100.0;
    altitude = ((altimeter.getAltitude(SEALEVELPRESSURE)) * METERS_TO_FEET) - baseAltitude;
  }
}

// ============================ openDataFile ================================
// Opens the data log file. Creates a unique file name based upon the current
// date and time as read from the Real Time Clock.
// ==========================================================================
void openDataFile() {
  // Make sure the file has not already been opened
  if (!dataFileOpen) {
    // Read the date/time from the real time clock to create a unique log file name
    DateTime now = realTimeClock.now();
    String dataFileName = "";
    dataFileName += now.year();
    dataFileName.remove(0, 2);
    dataFileName += now.month();
    dataFileName += now.day();
    dataFileName += now.minute();
    dataFileName += ".txt";
#ifdef DEBUG
    Serial.print("Log file name: ");
    Serial.println(dataFileName);
#endif

    dataFileOpen = true;
    dataFile =  SD.open(dataFileName, FILE_WRITE);
  }
}

// =========================== closeDataFile ================================
// Closes the data log file if it is open.
// ==========================================================================
void closeDataFile() {
  if (dataFileOpen) {
    dataFile.close();
    dataFileOpen = false;
  }
}

// ======================== readBatteryVoltage ==============================
// Reads and returns the LiPo battery voltage. The value returned is in volts
// ==========================================================================
float readBatteryVoltage(void) {
  int batteryVolt = analogRead(BATTERY_MONITOR);
  return ((float)(batteryVolt * 4.096) / 4096.0); // measurement in volts
}

// ======================= displayBatteryLevel ==============================
// Sends the current battery voltage reading to the connected bluetooth
// device.
// ==========================================================================
void displayBatteryLevel(void) {
  String voltageMessage = "AT+BLEUARTTX=Battery Voltage: ";
  voltageMessage += lipoBatteryVoltage;
  voltageMessage += " V";
  bleRadio.println(voltageMessage);
  
}

// ============================ logActivity =================================
// Create a log string and write it to the SD card along with a time stamp
// ==========================================================================
void logActivity(String type, String logEntry) {
  
  // Get the date/time
  DateTime now = realTimeClock.now();
  // Compose the log string
 
  String logString = "";
  
  logString += now.year();
  logString += "-";
  logString += now.month();
  logString += "-";
  logString += now.day();
  logString +="T";
  logString += now.hour();
  logString += ":";
  logString += now.minute();
  logString += ":";
  logString += now.second();
  logString += "PST|";
  logString += type;
  logString += "|";
  logString += logEntry;
  
#ifdef DEBUG
  Serial.println(logString);
#endif

  // Open the log file
  logFile = SD.open("logFile.txt", FILE_WRITE);
  
  // Write the log data to the SD card
  if (logFile) {
    logFile.println(logString);
    // Close the log file
    logFile.close();
  } else {
#ifdef DEBUG
    Serial.println("SD card write error");
#endif
  }
}

// ========================= flashLED =================================
// Flashes the LED with a pulse period of pulsePeriod milliseconds and
// a pulse width of LED_PULSE_WIDTH. This function is called as the 
// result of a periodic TC3 interrupt which occurs at the rate of 
// TC3_INT_PERIOD milliseconds.
// ====================================================================
void flashLED(uint16_t pulsePeriod) {
  static int ppCounter = 0; // Pulse period counter
  static int pwCounter = 0; // Pulse width counter
  static bool LED_On = false;

  // Count real time interrupts so we can compute the pulse period
  // and pulse width
  ppCounter++;
  if (LED_On) {
    // Only increment the pulse width counter if a pulse has started
    pwCounter++;
  }
  
  if (ppCounter >= (pulsePeriod / TC3_INT_PERIOD)) {
    // Time to start an LED pulse
    // Restart both counters
    ppCounter = 0;
    pwCounter = 0;
    
    // Toggle the LED on
    PORT->Group[0].OUTTGL.reg = PORT_PA17;
    LED_On = true;
  }

  if (LED_On && pwCounter >= (LED_PULSE_WIDTH / TC3_INT_PERIOD)) {
    // Time to stop the LED pulse
    // Reset the pwCounter
    pwCounter = 0;

    // Toggle the LED off
    PORT->Group[0].OUTTGL.reg = PORT_PA17;
    LED_On = false;
    
  }
}

// ========================= isLaunched ===============================
// Reads the accelerometer and returns true if the measurement
// indicates a launch has occurred.
// ====================================================================
bool isLaunched() {
  // Read the accelerometer/gyro sensor data
  accelerometer_gyro.readAccelData();
  // Check if acceleration measurement exceeds launch threshold
  if (accelerometer_gyro.getXAxisAccel() > LAUNCH_ACCEL) {
    
    return true;

  } else {
    return false;
  }
}

// ==================== computeLaunchPointAlt =========================
// Reads the barometric pressure and altitude and computes the launc
// point altitude in feet. Log the base altitude.
// ====================================================================
uint8_t computeLaunchPointAlt() {
  altimeterErrorCode = altimeter.readSensorData();
  if (altimeterErrorCode = 0) {
    pressure = altimeter.getPressure() / 100.0;
    baseAltitude = (altimeter.getAltitude(SEALEVELPRESSURE)) * METERS_TO_FEET;
    String logString = "Base Altitude: ";
    logString +=  baseAltitude;
    logActivity("STATUS", logString);
#ifdef DEBUG
    Serial.print("Base Altitude: ");
    Serial.println(baseAltitude);
#endif
  }

  return altimeterErrorCode;
}
