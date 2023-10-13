// UpGoer5Instrumentation (Green)
// Version: Beta 7
// Author: Bob Parker
// Date: 9/2/2023
// Tested: 
//
// Code for a model rocket instrumentation package. Measures and logs acceleration and turning
// rates along three axis. Measures and logs atmospheric pressure and computed altitude above
// ground. Measures and logs air temperature.
// In the loop function, getting new sensor readings and logging them to the SD card is taking
// between 15ms and 18ms. When logging is turned off then getting the sensor readings is taking
// about 850usec.

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

// Beta 6:
// Added integration with IOS UpGoV app.

// Beta 7: Replaced bluetooth radio with RFM95 LoRa radio.
//         Added GPS Sensor (Adafruit Ultimate GPS Featherwing). Lat/Lon/Alt datawill be reported via radio.
//         Added 400G Accelerometer sensor (Adafruit H3LIS331)
//         Added 2 addition in-flight event outputs (Now labeled event 1 through event 4)
//         Removed the Real Time Clock on the Adalogger Feather wing. Date/time data obtained from GPS
//         Now using an SD card breakout instead of the Adalogger featherwing
//         Added measurement of in-flight event battery voltage (7.4 volts). Report status via radio
//         Incorporate power saving measures

// ==============================  Include Files  ==================================
#include <SPI.h>                      // Sensor and micro SD card communication
#include <SD.h>                       // SD card library functions
#include <BMP3XX.h>                   // BMP390 sensor library code
#include <LSM6DSO32.h>                // LSM6DSO32 sensor library code
#include "wiring_private.h"           // pinPeripheral() function
#include "RTClib.h"                   // Real Time Clock library by Adafruit
#include <RWP_GPS.h>                  // GPS Library
#include <RH_RF95.h>                  // Radio Head driver class
#include <H3LIS331.hpp>               // H3LIS331 sensor library code


// ===============================  Constants  =====================================
#define DEBUG // Uncomment to enable debug comments printed to the console
//#define GPS_DEBUG // Uncomment to enable printing GPS NMEA senstences
//#define PRINT_LOG_DATA  // Uncomment to enable printing of log data to the console
#define GPSSerial Serial1
#define GPSECHO false

const String VERSION = "Beta 7.1";

// Arduino pin assignments
const uint8_t BMP390_CS = 5;
const uint8_t LSM6DSO32_CS = 6;
const uint8_t SD_CARD_CS = A5;
const uint8_t H3LIS331_CS = SDA;
const uint8_t GPS_ENABLE = SCL;
const uint8_t LED = 13;
const uint8_t RADIO_SPI_CS = 8;
const uint8_t RADIO_SPI_IRQ = 3;
const uint8_t RFM95_RST = 4;
const uint8_t SENSOR_SPI_CLK = 12;
const uint8_t SENSOR_SPI_MOSI = 10;
const uint8_t SENSOR_SPI_MISO = 11;
const uint8_t BATTERY_MONITOR = 9;
const uint8_t INFLIGHT_EVENT_BATTERY_MONITOR = A0;
const uint8_t INFLIGHT_EVENT_1 = A1;
const uint8_t INFLIGHT_EVENT_2 = A2;
const uint8_t INFLIGHT_EVENT_3 = A3;
const uint8_t INFLIGHT_EVENT_4 = A4;

// Other Constants
const uint8_t TC5_INT_PERIOD = 10;          // RTC interrupt period in milliseconds
const uint16_t LED_PPS_FAULT = 100;          // LED pulse rate while in FAULT state
const uint16_t LED_PPS_ARMED = 200;          // LED pulse rate while in ARMED state
const uint16_t LED_PPS_POST_FLIGHT = 10000;  // LED pulse period (ms) in POST_FLIGHT state
const int LED_PULSE_WIDTH = 50;             // LED pulse width in all states
const double SEALEVELPRESSURE = 1013.25;     // Sea level pressure in hpa
const double METERS_TO_FEET = 3.28084;       // Conversion from meters to feet
const long LOG_FILE_DURATION = 10;          // How long sensor data is logged to the log file in seconds
const double BAT_VOLT_LOW = 3.5;            // LiPo battery voltage below this level is at low charge
const double BAT_VOLT_FULL = 4.1;
const double BAT_VOLT_OK = 3.7;
const uint16_t BAT_MEAS_PER  = 10000;       // LiPo battery voltage measurement period (ms)
const double NO_MOTION_UPPER = 2.0;          // Rate below which we are not moving
const double NO_MOTION_LOWER = -2.0;         // Rate above which we are not moving
const uint16_t TIME_OUT = 30;               // Number of seconds after which we will assume we have landed.
const double LAUNCH_ACCEL = 0.2;             // Threshold for launch determination in G's
const uint16_t PARACHUTE_SIGNAL_DURATION = 2000;  // Duration of parachute release signal in milliseconds
const double GRAVITY_ACC = 32.174;          // The acceleration on the surface of the Earth in f/sec2
const uint8_t CLIENT_ADDRESS = 2;
const uint8_t SERVER_ADDRESS = 1;
const double RF95_FREQ = 915.0;
const uint8_t MAX_MESSAGE_LENGTH = 20;    // Maximum radio message length


enum states {START_UP, FAULT, READY, ARMED, LOGGING, POST_FLIGHT};
enum errors {NONE, GYRO, ACCEL, ALT, BATTERY};

// =======================  Global Variables/Objects  ===============================
volatile bool rtcInterruptFlag = false;   // Flag set by rtc interrupt handler
 
// Driver class objects
SPIClass sensorSPI(&sercom1, 11, 12, 10, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0); // The sensor SPI bus instance
BMP3XX altimeter;  // Altimeter sensor object instance
LSM6DSO32 accelerometer_gyro;  //Accelerometer/Gyro sensor object instance
H3LIS331 accelHighG;          // H3LIS331 high accelerometer object instance
RTC_PCF8523 realTimeClock;
RH_RF95 radioDriver = RH_RF95(RADIO_SPI_CS, RADIO_SPI_IRQ); // LoRa radio driver object
Adafruit_GPS gps(&GPSSerial);

double temperature = 0.0;       // Temperature in degrees C
double pressure = 0.0;          // Pressure in hpa
double altitude = 0.0;           // Altitude in meters
int8_t altimeterErrorCode = 0;  // Result code returned from BMP390 API calls
bool readyForLaunch = false;
bool dataFileOpen = false;
File dataFile;                  // The data file object
File logFile;                   // The log file object
uint32_t launchTime = 0;        // Time at launch in millisends
double lipoBatteryVoltage = 0;   // Battery voltage of LiPo battery as measured by ADC
states state = START_UP;        // The current state
errors error = NONE;            // Error
double maxAcceleration = 0;      // The maximum axial acceleration measured during flight
double maxAltitude = 0;          // The maximum altitude measured during flight
uint32_t flightLength = 0;
double velocity = 0;
double maxVelocity = 0;
double earthAccelerationMeasAvg = 1.0;  // Average measured earth acceleration
// Global vars used by the velocityCaculator() function
uint32_t prevTime = 0;
double prevAcc = 0.0; // f/sec2
double prevVelocity = 0;
bool GPS_FIX = false;
double launchPointLat;
double launchPointLon;
double launchPointAlt;           // Altitude above MSL in meters
double baroAltitudeError;        // Difference between baro computed alt and GPS alt
char buffer[MAX_MESSAGE_LENGTH]; // Message buffer
char radioPacket[20];            // Buffer containing radio message

// =======================  TC5 Interrupt Handler  ===================================
// TC5 will generate an interrupt at a rate determined by TC5_INT_PERIOD to drive the 
// measurement/logging cycle. The required interrupt handler name is specified in CMSIS 
// file samd21g18a.h. The interrupt handler will simply set a flag so the loop code 
// can respond accordingly.
void TC5_Handler(void) {
  // The first step is to clear the interrupt flag
  // The only interrupt enabled is MC0 so we don't need to read the interrupt flag
  // register to see which interrupt triggered
  // Writing a 1 to the flag bit will clear the interrupt flag
  TC5->COUNT8.INTFLAG.bit.MC0 = 1;

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

// ============================ Configure GPIO Pins ==================================

  // Configure the GPIO pins
  pinMode(LED, OUTPUT);   // Use this digital output to observe the TC3 interrupt
  PORT->Group[0].OUTCLR.reg = PORT_PA17; // Turn off the on-board LED
  pinMode(INFLIGHT_EVENT_1, OUTPUT);
  digitalWrite(INFLIGHT_EVENT_1, LOW);
  pinMode(INFLIGHT_EVENT_2, OUTPUT);
  digitalWrite(INFLIGHT_EVENT_2, LOW);
  pinMode(INFLIGHT_EVENT_3, OUTPUT);
  digitalWrite(INFLIGHT_EVENT_3, LOW);
  pinMode(INFLIGHT_EVENT_4, OUTPUT);
  digitalWrite(INFLIGHT_EVENT_4, LOW);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(20);

  // ========================== LoRa Radio Initialization ============================
  // Reset the LoRa radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the LoRa radio
  if (!radioDriver.init()) {
#ifdef DEBUG
    Serial.println("Radio init() error");
#endif
  }

  // Set the LoRa radio operating frequency
  if (!radioDriver.setFrequency(RF95_FREQ)) {
#ifdef DEBUG
    Serial.println("Failed to set LoRa radio frequency");
#endif
  }
  // Set the LoRa radio transmit power
  radioDriver.setTxPower(23, false);
#ifdef DEBUG
  Serial.println("Radio initialized");
  Serial.println("Waiting for ground station connect");
#endif

  // Block here until we connect to the UpGoV Ground Station
  bool connected = false;
  while (!connected) {
    if (radioDriver.available()) {
      // Message received. Check for a connect message
      char messageBuffer[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t messageLength = sizeof(messageBuffer);
      if (radioDriver.recv((uint8_t *)messageBuffer, &messageLength)) {
        // Check message length
        if (messageLength != 0) {
          if (!strncmp(messageBuffer, "connect", 7)) {
            // Connect message received
            connected = true;
            // Send the reply
            uint8_t data[] = "connect";
            radioDriver.send(data, sizeof(data));
            radioDriver.waitPacketSent();
            
          }
        }
      }
    }
  } // End radio connect loop

  delay(500);

  // ======================  Initialize the Sensor SPI Bus  ============================
  sensorSPI.begin();
  // These calls need to be here because the call to sensorSPI.begin() above will 
  // configure the pins to analog peripheral functions rather than the SERCOM-ALT that
  // we want.
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  sendRadioMessageNoAck("MS:Sensor SPI Init");

  // ============================ SD Card Library Setup ================================
  
  if (!SD.begin(SD_CARD_CS)) {
#ifdef DEBUG
    Serial.println("SD card failed to initialize");
#endif
    sendRadioMessageNoAck("ER:SD Card Init");
    delay(20);
    sendRadioMessageAckRetry("ST:ERROR", 3);
    delay(20);
    while(1); // Hang here
  } else {
#ifdef DEBUG
    Serial.println("SD card initialized");
#endif
    sendRadioMessageNoAck("MS:SD Card Init");
  }

  // ======================  SAMD21 Generic Clock Setup  ====================
  // Configure Generic Clock 6. It will we used as the source clock for the
  // TC5TCC2_ peripheral. Generic Clock 6 will output 16KHz clock.
  // ========================================================================

  GCLK->GENDIV.reg =  GCLK_GENDIV_ID(0x06) |     // Generic Clock 6
                      GCLK_GENDIV_DIV(0x02);     // Divide by 2 (16KHz)
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |       // Enable generic clock generator
                      GCLK_GENCTRL_SRC_XOSC32K | // External 32KHz crystal source
                      GCLK_GENCTRL_ID(0x06);     // Generic Clock 6
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |   // Generic Clock 6
                      GCLK_CLKCTRL_ID_TC4_TC5 | // TC3 will use generic clock 6
                      GCLK_CLKCTRL_CLKEN;        // Enable generic clock

#ifdef DEBUG
  Serial.println("Generic clock initialized");
#endif
  sendRadioMessageNoAck("MS:GCLK 6 Init");

  // =============================  TC5 Setup  ==============================
  // TC5 is used to generate a periodic interrupt to drive all activity.
  // ========================================================================
  // Enable the TC5 bus clock
  PM->APBCMASK.reg |= PM_APBCMASK_TC5;

  //Disable TC5
  TC5->COUNT8.CTRLA.bit.ENABLE = 0;
  while (TC5->COUNT8.STATUS.bit.SYNCBUSY);

  // Enable the TC5 OVF (overflow) interrupt
  TC5->COUNT8.INTENSET.reg = TC_INTENSET_OVF;

  // Configure TC5 Control A register settings
  // PRESCALER set to divide by 16
  // MODE set to 8-bit counter
  TC5->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 |      // 8-bit counter mode
                          TC_CTRLA_WAVEGEN_NPWM |     // PER register used as TOP value
                          TC_CTRLA_PRESCALER_DIV16 |  // Divide GCLK by 16 (1KHz counter clock)
                          TC_CTRLA_PRESCSYNC_GCLK;    // Wrap around on next GCLK tick

  //Set the PER register value
  TC5->COUNT8.PER.reg = TC5_INT_PERIOD;
  while (TC5->COUNT8.STATUS.bit.SYNCBUSY);

  // Enable the TC5 interrupt in the NVIC
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

#ifdef DEBUG
  Serial.println("TC5 initialized");
#endif
  sendRadioMessageNoAck("MS:TC5 Init");

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
  sendRadioMessageNoAck("MS:ADC Init");

  // ====================== GPS Setup  ==================================

  // Initialize the GPS module
  gps.begin(9600);
  // Turn on the RMC and GGA NMEA sentences from which we can parse
  // all of the GPS information that we need
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Hold here until we get a GPS fix
  while (!GPS_FIX) {
    //Check for a new NMEA sentence and parse the results if received
    checkGPS();
    //if (gps.fix) {
    if (true) {
      GPS_FIX = true;
#ifdef DEBUG
      Serial.println("GPS Fix");
#endif
    }
    delay(10);
  }

  sendRadioMessageNoAck("MS:GPS Fix");


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
    sendRadioMessageNoAck("ER:LSM6DS032 Error");
  } else {
    sendRadioMessageNoAck("MS:LSM6DS032 Init");
#ifdef DEBUG
    Serial.println("Accelerometer/Gyro sensor initilaized");
#endif
  }

  // Configure accelerometer/gyro settings
  accelerometer_gyro.setAccelRange(LSM6DSO32_RANGE_32G);
  accelerometer_gyro.setGyroRange(LSM6DSO32_RANGE_500DPS);
  accelerometer_gyro.setAccelDataRate(LSM6DSO32_ACCEL_104_HZ);
  accelerometer_gyro.setGyroDataRate(LSM6DSO32_GYRO_104_HZ);

  logActivity("STATUS", "Accelerometer/gyro sensor initialized");

// ======================== H3LIS331 Sensor Setup =======================
  // The H3LIS331 is a high G accelerometer. It is used to 
  // measure acceleration along the 3 axis. The X-Axis
  // is aligned along the rockets longitudinal axis (top to bottom).
  // ====================================================================

  // Initialize the accelerometer sensor
#ifdef DEBUG
  Serial.println("Initializing H3LIS331 accelerometer"); 
#endif
  if (!accelHighG.begin(H3LIS331_CS, &sensorSPI)) {
#ifdef DEBUG
    Serial.println("Error initializing H3LIS331 accelerometer sensor");
#endif
    state = FAULT;
    logActivity("ERROR", "Error initializing the H3LIS331 accelerometer sensor");
    sendRadioMessageNoAck("ER:H3LIS331 Error");
  } else {
    sendRadioMessageNoAck("MS:H3LIS331 Init");
#ifdef DEBUG
    Serial.println("H3LIS331 accelerometer sensor initilaized");
#endif
  }

  // Configure accelerometer settings
  accelHighG.setRange(H3LIS331DL_200g);
  accelHighG.setOutputDataRate(H3LIS331DL_ODR_100Hz);
  
  logActivity("STATUS", "H3LIS331 accelerometer sensor initialized");

  
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
    sendRadioMessageNoAck("ER:BMP390 Error");
  } else {
#ifdef DEBUG
    Serial.println("Altimeter sensor initialized");
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
    sendRadioMessageNoAck("ER:BMP390 Error");
    state = FAULT;
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
    sendRadioMessageNoAck("ER:BMP390 Error");
    state = FAULT;
  }

  logActivity("STATUS", "Altimeter settings and power mode initialized");
  sendRadioMessageNoAck("MS:BMP390 Init");


  // ======================= Ready State Check ===========================

  // Start-up is complete. Move to Ready state if no errors occurred during startup 
  if (state != FAULT) {
    state = READY;
    logActivity("EVENT", "Transition to Ready state");
    if (!sendRadioMessageAckRetry("ST:READY", 3)) {
      // The message was not confirmed as sent
#ifdef DEBUG
      Serial.println("Ready state transition not acknowledged");
#endif
      state = FAULT;
      logActivity("ERROR", "Ready state transition not acknowledged");
    }
    
  }

  // Enable TC5. We will start getting TC5 interrupts at the TC5_INT_PERIOD
  TC5->COUNT8.CTRLA.bit.ENABLE = 1;
  while (TC5->COUNT8.STATUS.bit.SYNCBUSY);
  
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

  // Read the new sensor data
  accelerometer_gyro.readSensorData();

  // Read GPS NMEA sentences and update GPS data values
  checkGPS();
  
  // Check for an ARM event every time through the loop
  checkArmCommand();

  // Check battery voltage and report via radio
  counter++;
  if (counter >= (BAT_MEAS_PER / TC5_INT_PERIOD)) {
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
      sendRadioMessageNoAck("MS:Battery Low");
      sendRadioMessageAckRetry("ST:ERROR", 3);
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
  // Compute a running average of the measured earth acceleration
  // from the last 5 x-axis acceleration measurements
  computeAverageAcceleration();

  // Read GPS NMEA sentences and update GPS data values
  checkGPS();
  
  // Check for a DISARM event every time through the loop
  if (checkRadioMessage("disarm")) {
    state = READY;
    sendRadioMessageAckRetry("ST:READY", 3);
    logActivity("EVENT", "Disarmed");
    closeDataFile();
    digitalWrite(LED, LOW);
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
    sendRadioMessageNoAck("ST:LAUNCHED");
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
static uint8_t apogeeDetectedCounter = 0;
static uint16_t parachute_signal_timer = 0;
static bool parachute_released = false;
timeOutCounter++;

#ifdef DEBUG
  // Toggle D13 so we can measure when the interrupt processing starts
  PORT->Group[0].OUTTGL.reg = PORT_PA17;
#endif

  // Read the current time
  uint32_t timeInMS = millis();

  // Read the altimeter data and correct altitude
  readAltimeterData();
  altitude = altitude - baroAltitudeError;

  // Update the maximum achieved altitude
  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  }

  // Check if we are at the apogee and release the parachute if we are
  if (atApogee()) {
    apogeeDetectedCounter++;
    if (apogeeDetectedCounter >= 5) {
      digitalWrite(INFLIGHT_EVENT_1, HIGH); 
      parachute_released = true;
      apogeeDetectedCounter = 0;
    }
  }

  // Read the accelerometer/gyro sensor and high G accelerometer sensor data
  accelerometer_gyro.readSensorData();
  accelHighG.getAcceleration();

  // Compute velocity. Use the high G accelerometer acceleration reading if the 
  // LSM6DSO32 sensor acceleration reading is greater than 30g
  float X_AxisAcceleration;
  if (accelerometer_gyro.getXAxisAccel() > 30.0) {
    X_AxisAcceleration = accelHighG.getX_Accel();
  } else {
    X_AxisAcceleration = accelerometer_gyro.getXAxisAccel();
  }
  
  velocity = velocityCalculator(timeInMS, X_AxisAcceleration - earthAccelerationMeasAvg);

  // Update the maximum velocity
  if (velocity > maxVelocity) {
    maxVelocity = velocity;
  }

  // Update maximum acceleration
  if (X_AxisAcceleration > maxAcceleration) {
    maxAcceleration = X_AxisAcceleration;
  }
  
  
  if ((LOG_FILE_DURATION == 0)  || ((timeInMS - launchTime) <= (LOG_FILE_DURATION * 1000))) {
    
      //Log the data
      logData(timeInMS);
  }

  // Check if we have landed (no more motion)
  if (isLanded) {
    state = POST_FLIGHT;
    postFlightDataLog("LANDED");
    sendRadioMessageAckRetry("ST:LANDED", 60);
  }
      

  // Check for timeout
  if (timeOutCounter * TC5_INT_PERIOD > TIME_OUT * 1000) {
    state = POST_FLIGHT;
    postFlightDataLog("TIMEOUT");
    sendRadioMessageAckRetry("ST:LANDED", 60);
    timeOutCounter = 0;
  }

    // Checks if we need to turn off the parachute release signal
    parachute_signal_timer++;
    if(parachute_signal_timer >= (PARACHUTE_SIGNAL_DURATION / TC5_INT_PERIOD)) {
      digitalWrite(INFLIGHT_EVENT_1, LOW);
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
  static bool connected = false;
  
  flashLED(LED_PPS_POST_FLIGHT);

  // Check for an incoming "connect" radio message
  if (checkRadioMessage("connect")) {
    sendRadioMessageNoAck("connect");
    connected = true;
  }

  if (connected && !sent) {
    delay(5000);

    String dataString = "AL:";
    dataString += String(maxAltitude);
    sendRadioMessageAckRetry(dataString.c_str(), 3);
    delay(20);
    
    dataString = "AC:";
    dataString += String(maxAcceleration);
    sendRadioMessageAckRetry(dataString.c_str(), 3);
    delay(20);

    dataString = "DU:";
    dataString += String(flightLength);
    sendRadioMessageAckRetry(dataString.c_str(), 3);
    delay(20);

    sendRadioMessageNoAck("AT:AGAIN");
    delay(20);

    
    sent = true;
  }

  // Check for the "again" message
  if (checkRadioMessage("again")) {
    state = READY;
    maxAltitude = 0;
    maxAcceleration = 0;
    maxVelocity = 0;
    flightLength = 0;
    prevTime = 0;
    prevAcc = 0.0; // f/sec2
    prevVelocity = 0;
    logActivity("EVENT", "READY");
    sent = false;
    connected = false;
    sendRadioMessageAckRetry("ST:READY", 3);
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
  logData += accelHighG.getX_Accel();
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
  logData += ",";
  logData += velocity;
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
// Reads the altimeter data
// Return: An error code reflecting the result of the altimeter read. A value
//         of 0 indicates no error during the read operation.
// ==========================================================================
int8_t readAltimeterData() {
  int8_t errorCode = altimeter.readSensorData();
  if (errorCode != 0) {
      
#ifdef DEBUG
    printAltimeterError("Error reading altimeter sensor data: ", errorCode);
#endif
      
  temperature = -100;
  pressure = -100;
  altitude = -100;
  } else {
    temperature = altimeter.getTemperature();
    pressure = altimeter.getPressure() / 100.0;
    altitude = (altimeter.getAltitude(SEALEVELPRESSURE));
  }
  return errorCode;
}

// ============================ openDataFile ================================
// Opens the data log file. Creates a unique file name based upon the current
// date and time (GMT) as read from the GPS. YYMMDDMM.txt
// ==========================================================================
void openDataFile() {
  // Make sure the file has not already been opened
  if (!dataFileOpen) {
    String dataFileName = "";
    dataFileName += gps.year;
    dataFileName.remove(0, 2);
    dataFileName += gps.month;
    dataFileName += gps.day;
    dataFileName += gps.minute;
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
double readBatteryVoltage(void) {
  int batteryVolt = analogRead(BATTERY_MONITOR);
  return ((batteryVolt * 4.096) / 4096.0); // measurement in volts
}

// ======================= displayBatteryLevel ==============================
// Sends the current battery voltage reading to the connected bluetooth
// device.
// ==========================================================================
void displayBatteryLevel(void) {
  if (lipoBatteryVoltage >= BAT_VOLT_FULL) {
    sendRadioMessageNoAck("BT:FULL");
  } else if (lipoBatteryVoltage >= BAT_VOLT_OK) {
    sendRadioMessageNoAck("BT:OK");
  } else if (lipoBatteryVoltage >= BAT_VOLT_LOW) {
    sendRadioMessageNoAck("BT:LOW");
  } else {
    sendRadioMessageNoAck("BT:CRITICAL");
  }
  
}

// ============================ logActivity =================================
// Create a log string and write it to the SD card along with a time stamp
// ==========================================================================
void logActivity(String type, String logEntry) {
  
  // Compose the log string
  String logString = "";
  
  logString += gps.year;
  logString += "-";
  logString += gps.month;
  logString += "-";
  logString += gps.day;
  logString +="T";
  logString += gps.hour;
  logString += ":";
  logString += gps.minute;
  logString += ":";
  logString += gps.seconds;
  logString += "PST|";
  logString += type;
  logString += "|";
  logString += logEntry;
  
#ifdef PRINT_LOG_DATA
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
  
  if (ppCounter >= (pulsePeriod / TC5_INT_PERIOD)) {
    // Time to start an LED pulse
    // Restart both counters
    ppCounter = 0;
    pwCounter = 0;
    
    // Toggle the LED on
    PORT->Group[0].OUTTGL.reg = PORT_PA17;
    LED_On = true;
  }

  if (LED_On && pwCounter >= (LED_PULSE_WIDTH / TC5_INT_PERIOD)) {
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
  if ((accelerometer_gyro.getXAxisAccel() - earthAccelerationMeasAvg) > LAUNCH_ACCEL) {
    
    return true;

  } else {
    return false;
  }
}

// ==================== postFlightDataLog =========================
// Prints to the SD card max altitude, max acceleration, and flight
// duration after timeout\landing.
// ====================================================================
 void postFlightDataLog(String flightEndCause) {
  state = POST_FLIGHT;
  if (dataFileOpen) {
    closeDataFile();
  }
  logActivity("EVENT", flightEndCause);
  // Log the maximum acceleration and altitude
  uint32_t landTime = millis();
  flightLength = landTime - launchTime;
  flightLength /= 1000;
  String logString = "Maximum acceleration = ";
  logString += maxAcceleration;
  logString += " Gs";
  logActivity("STATUS", logString);
  logString = "Maximum altitude = ";
  logString += maxAltitude;
  logString += " ft";
  logActivity("STATUS", logString);
  logString = "FLight duration = ";
  logString += flightLength;
  logString += " sec";
  logActivity("STATUS", logString);
  logString = "Maximum velocity = ";
  logString += maxVelocity;
  logString += " fps";
  logActivity("STATUS", logString);
 }

// ==================== velocityCalculator =========================
// Calculates the rocket's velocity.
// ====================================================================
double velocityCalculator(uint32_t currentTime, double currentAcc) {
 
 double timeInterval = 0.0;

 double  scaledAcc = currentAcc * GRAVITY_ACC; // Acceleration to f/sec2

 double avgAcc = (prevAcc + scaledAcc) / 2;

 if (prevTime == 0) {
  timeInterval = (double)TC5_INT_PERIOD;
 } else {
  timeInterval = (double)currentTime - (double)prevTime;
 }
 

 timeInterval /= 1000.0; // TimeInterval in sec

 double deltaV = avgAcc * timeInterval; // f/sec

 double velocity = deltaV + prevVelocity; // f/sec

 prevTime = currentTime;
 prevAcc = scaledAcc;
 prevVelocity = velocity;
 
 return velocity;

 
}

// ==================== computeAverageAcceleration ===================
// Calculates the running average of the measured acceleration over
// 10 samples. Used to compute the average earth acceleration prior to 
// launch.
// ===================================================================
void computeAverageAcceleration() {
  // Array to hold the ten acceleration measurements
  static double accelMeasurements[10] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  static int accelMeasIndex = 0;  // Next measurement to be saved

  if (accelMeasIndex < 10) {
    // Read the accelerometer/gyro sensor data...
    accelerometer_gyro.readSensorData();
    // ... and save the latest measurement
    accelMeasurements[accelMeasIndex] = accelerometer_gyro.getXAxisAccel();

    ++accelMeasIndex;

    // Compute the average acceleration
    double accelSum = 0;
    for (int index = 0; index < 10; ++index) {
      accelSum += accelMeasurements[index];
    }

  // Save the result to the global variable
  earthAccelerationMeasAvg = accelSum / 10.0;

#ifdef DEBUG
  Serial.print("Average Earth Accel: ");
  Serial.println(earthAccelerationMeasAvg);
#endif
  }

  
}

// ======================== checkGPS =================================
// Checks for and processes GPS messages.
// ===================================================================
void checkGPS() {
  // Read data from the GPS
  char c = gps.read();
  
#ifdef GPS_DEBUG
  if (c) {
    Serial.print(c);
  }
#endif

  if (gps.newNMEAreceived()) {
#ifdef GPS_DEBUG
    Serial.print(gps.lastNMEA());
#endif
    if (!gps.parse(gps.lastNMEA())) {
#ifdef GPS_DEBUG
      Serial.println("Failed to parse GPS sentence");
#endif
    }

  }
}

// ======================== atApogee =================================
// Checks if the rocket has reached apogee (highest point)
// Return:  True if apogee is detected. False otherwise.
// ===================================================================
bool atApogee() {
  // Check for low altitude apogee
  if((maxAltitude < (75.0 + launchPointAlt)) && (altitude < (maxAltitude - 1))) {
      return true;
  }
  // Check for high altitude apogee
    if((maxAltitude >= (75.0 + launchPointAlt)) && (altitude < (maxAltitude - 10))) {
      return true;
    }
    return false;
  }

// ======================== isLanded =================================
// Checks if the rocket has landed
// Return:  True if a landing is detected. False otherwise.
// ===================================================================
bool isLanded() {
  if ((accelerometer_gyro.getXAxisRate() < NO_MOTION_UPPER) &&
          (accelerometer_gyro.getXAxisRate() > NO_MOTION_LOWER) && 
          (accelerometer_gyro.getYAxisRate() < NO_MOTION_UPPER) &&
          (accelerometer_gyro.getYAxisRate() > NO_MOTION_LOWER) &&
          (accelerometer_gyro.getZAxisRate() < NO_MOTION_UPPER) && 
          (accelerometer_gyro.getZAxisRate() > NO_MOTION_LOWER)) {
            return true;
          } else {
            return false;
          }
}

//=================== sendRadioMessageNoAck ======================
// Sends a radio message with no acknowledment required.
// Parameters:
//  message:  The radio message that is to be sent
// Return: None
//=================================================================
void sendRadioMessageNoAck(const char * message) {
  strncpy(radioPacket, message, sizeof(radioPacket));
  radioDriver.send((uint8_t *)radioPacket, sizeof(radioPacket));
  delay(10);

#ifdef DEBUG
  Serial.print("Radio message sent: ");
  Serial.println(message);
#endif
  // Block here until the packet has been sent
  radioDriver.waitPacketSent();
  delay(200);
}


//=================== sendRadioMessageAckRetry ====================
// Sends a radio message with acknowledment required. Resends the
// message if no acknowledgment is received.
// Parameters:
//  message:  The radio message that is to be sent
//  retries:  The number of times that the message will be resent
// Return: True if a acknowledment was received
//=================================================================
bool sendRadioMessageAckRetry(const char * message, uint8_t retries) {
  strncpy(radioPacket, message, sizeof(radioPacket));

  uint8_t attempt = 0;
  while (attempt <= retries) {
    // Send the message
    radioDriver.send((uint8_t *)radioPacket, sizeof(radioPacket));
    delay(10);
    radioDriver.waitPacketSent();

    // Listen for an acknowledgment
    uint8_t length;
    if (radioDriver.waitAvailableTimeout(1000)) {
      if (radioDriver.recv((uint8_t *)buffer, &length)) {
        if (length != 0) {
          if (!strncmp(buffer, message, sizeof(message))) {
            return true;
          }
        }
      }
    } // End listen for acknowledgement

    // Try again
    attempt++;
  } // End while loop

  return false;
  
}


//======================= checkRadioMessage =======================
// Checks if the specified radio message has been received.
// Parameters:
//  message:  The radio message to check for
// Return: True if the message was received
//=================================================================
bool checkRadioMessage(const char * message) {
  uint8_t messageLength;
  char messageBuffer[MAX_MESSAGE_LENGTH];
  
  if (radioDriver.available()) {
    if (radioDriver.recv((uint8_t *)messageBuffer, &messageLength)) {
      if (messageLength != 0) {
        if (!strncmp(messageBuffer, message, sizeof(message))) {
          return true;
        }
      }
    }
  }
  return false;
}


//======================= getLaunchPointFix =======================
// Reads, saves, and logs the current GPS latitude, longitude, and 
// altitude data. Computes the baro altitude error  assuming GPS 
// altitude is truth.
// Parameters: None
// Return: True if the baro altitude was read successfully
//=================================================================
bool getLaunchPointFix() {
  // Save the GPS location data
  launchPointLat = gps.latitude;
  launchPointLon = gps.longitude;
  launchPointAlt = gps.altitude;

  // Log the GPS location data
  String logString = "Launch Point Latitude: ";
  logString += launchPointLat;
  logActivity("STATUS", logString);
  logString = "Launch Point Longitude: ";
  logString += launchPointLon;
  logActivity("STATUS", logString);
  logString = "Launch Point Altitude: ";
  logString += launchPointAlt;
  logActivity("STATUS", logString);

  // Initialize maximum altitude
  maxAltitude = launchPointAlt;

  // Compute the bara altitude error
  if (!(readAltimeterData() == 0)) {
    //Failed to read altimeter data
    return false;
    }
    
  baroAltitudeError = altitude - launchPointAlt;

  return true;
}


//======================= checkArmCommand =========================
// Checks for a valid arm command from the ground station. Validity
// check includes checking if the rocket is vertical, checking
// that the arm command could be acknowledged, the launch point
// baro altitude error could be computed, and the data file could
// be opened. If an error is detected then the state is changed to
// FAULT.
// Parameters: None
// Return: None
//=================================================================
void checkArmCommand() {
  if (checkRadioMessage("arm")) {
    if (accelerometer_gyro.getXAxisAccel() > 0.9) {

      // Acknowledge receipt of the arm command
      if (sendRadioMessageAckRetry("ST:ARMED", 3)) {
        state = ARMED;
        logActivity("Event", "Armed");
#ifdef DEBUG
        Serial.println("Arm command received and acknowledged");
#endif
        // Get GPS fix of launch point & compute baro altitude error
        if (!getLaunchPointFix()) {
          state = FAULT;
          sendRadioMessageAckRetry("ST:FAULT", 3);
#ifdef DEBUG
          Serial.println("Error reading altimeter while arming");
#endif
        }
        // Open data file in preparation for data logging
        openDataFile();
        if (dataFile) {
          String headerRow = "Time(ms), Temp(degC), Pres(hPa), Alt(ft), XH_Accel(Gs), X_Accel(Gs), Y_Accel(Gs), "
                             "Z_Accel(Gs), X_Rate(dps), Y_Rate(dps), Z_Rate(dps), Velocity(fps)";
          dataFile.println(headerRow);
        } else {
          // Data file could not be opened
          logActivity("ERROR", "SD Card write error");
          sendRadioMessageNoAck("ER:SD Card Write");
#ifdef DEBUG
          Serial.println("SD card write error");
#endif
        }
      } else {
        // Could not acknowledge arm command
        sendRadioMessageAckRetry("ST:READY", 3);
#ifdef DEBUG
        Serial.println("Could not acknowledge arm command");
#endif  
      }

    } else { 
      // The rocket is not vertical
      delay(200);
      sendRadioMessageNoAck("ER:Not Vertical");
    } // End check if rocket is vertical
  } // End checkRadioMessage
}
