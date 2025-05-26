#include <DHT.h>
#include <MQ135.h>
#include <SD.h>
#include <RTClib.h>
#include <EEPROM.h>

// Pin Configuration
#define TRIGGER_PIN1 5
#define ECHO_PIN1 4
#define TRIGGER_PIN2 7
#define ECHO_PIN2 6
#define DHTPIN 3
#define DHTTYPE DHT11
#define SD_CS_PIN 10 
#define MQ135_PIN A0
#define RESET_BUTTON_PIN 2
#define LED_ENTER 8
#define LED_EXIT 9
#define STATUS_LED 13  // Using built-in LED for status

// System Constants
const unsigned long LOG_INTERVAL = 300000;      // 10 seconds between logs
const unsigned long DEBUG_INTERVAL = 3000;     // 5 seconds between debug outputs
const unsigned long RECALIBRATION_INTERVAL = 1800000; // Recalibrate every hour
const unsigned long SENSOR_CHECK_INTERVAL = 30000;    // Check sensors every 30 seconds
const unsigned long WATCHDOG_INTERVAL = 60000;        // Sensor watchdog timer (1 minute)

// Occupancy Detection Parameters
const int SENSITIVITY = 20;                   // Detection threshold in cm
const unsigned long MIN_TRANSIT_TIME = 70;    // Minimum time between sensors (80 ms)
const unsigned long MAX_TRANSIT_TIME = 2000;  // Max time between sensors (1.5 seconds)
const int MAX_READING_ATTEMPTS = 5;           // Maximum sensor reading attempts before error
const int MIN_VALID_DISTANCE = 2;             // Minimum valid reading in cm
const int MAX_VALID_DISTANCE = 500;           // Maximum valid reading in cm
const int EEPROM_OCCUPANCY_ADDR = 0;          // EEPROM address to store occupancy

// Object Initialization
DHT dht(DHTPIN, DHTTYPE);
MQ135 gasSensor(MQ135_PIN);
RTC_DS3231 rtc;

// Global Variables
int baseline1, baseline2;
int occupancy = 0;
bool sensorFault = false;
bool sensor1Fault = false;
bool sensor2Fault = false;
unsigned long lastDetectionTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastDebugTime = 0;
unsigned long lastRecalibrationTime = 0;
unsigned long lastSensorCheckTime = 0;
int consecutiveErrors1 = 0;
int consecutiveErrors2 = 0;

// Tracking state machine
enum PassageState {
  IDLE,
  SENSOR1_TRIGGERED,
  SENSOR2_TRIGGERED,
  ERROR_STATE
};
PassageState currentState = IDLE;

// Function Prototypes
long measureDistance(int trigPin, int echoPin, bool *error = NULL);
void calibrateSensors();
void createLogFileIfNeeded();
void logSensorData(bool forceLog = false);
bool validateUltrasonicSensor(int trigPin, int echoPin);
void recoverFromFault();
void saveOccupancyToEEPROM();
void loadOccupancyFromEEPROM();
void handleButtonPress();
void errorBlink(int pin, int times);
void statusLED(bool state);
bool isPersonDetected(int currentReading, int baseline, int sensitivity, int trigPin, int echoPin);

// Setup function
void setup() {
  Serial.begin(9600);
  Serial.println(F("\n\n----- Occupancy Counter System -----"));
  Serial.println(F("Initializing..."));
  
  // Pin setup
  pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIGGER_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_ENTER, OUTPUT);
  pinMode(LED_EXIT, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  
  // Indicate startup
  digitalWrite(STATUS_LED, HIGH);
  
  // Initialize Sensors
  dht.begin();
  
  // RTC initialization with error handling
  if (!rtc.begin()) {
    Serial.println(F("RTC Module NOT FOUND"));
    errorBlink(STATUS_LED, 5);
    // Continue anyway with reduced functionality
  } else {
    // Only set RTC time if it's lost power
    if (rtc.lostPower()) {
     }
    
    DateTime now = rtc.now();
    Serial.print(F("RTC Date: "));
    Serial.print(now.year());
    Serial.print(F("-"));
    Serial.print(now.month());
    Serial.print(F("-"));
    Serial.println(now.day());
  }
  
  // SD Card initialization with error handling
  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD Card Initialization FAILED"));
    errorBlink(STATUS_LED, 5);
    // Continue anyway with reduced functionality
  } else {
    createLogFileIfNeeded();
    Serial.println(F("SD Card initialized successfully"));
  }
  
  // Load saved occupancy from EEPROM
  loadOccupancyFromEEPROM();
  
  // Calibrate sensors
  Serial.println(F("Calibrating sensors..."));
  calibrateSensors();
  
  // Mark system start times
  lastLogTime = millis();
  lastDebugTime = millis();
  lastRecalibrationTime = millis();
  lastSensorCheckTime = millis();
  
  Serial.println(F("System Initialized and Ready"));
  Serial.println(F("------------------------------"));
  
  // Flash LEDs to indicate system ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_ENTER, HIGH);
    digitalWrite(LED_EXIT, HIGH);
    delay(100);
    digitalWrite(LED_ENTER, LOW);
    digitalWrite(LED_EXIT, LOW);
    delay(100);
  }
  
  // Turn status LED off to indicate normal operation
  digitalWrite(STATUS_LED, LOW);
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  
  // Check reset button
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    handleButtonPress();
  }
  
  // If in error state, try to recover periodically
  if (currentState == ERROR_STATE && (currentTime - lastSensorCheckTime >= SENSOR_CHECK_INTERVAL)) {
    recoverFromFault();
    lastSensorCheckTime = currentTime;
    return;  // Skip the rest of this loop iteration
  }
  
  // Check sensors health periodically
  if (currentTime - lastSensorCheckTime >= SENSOR_CHECK_INTERVAL) {
    sensor1Fault = !validateUltrasonicSensor(TRIGGER_PIN1, ECHO_PIN1);
    sensor2Fault = !validateUltrasonicSensor(TRIGGER_PIN2, ECHO_PIN2);
    
    if (sensor1Fault || sensor2Fault) {
      Serial.println(F("SENSOR FAULT DETECTED"));
      if (sensor1Fault) Serial.println(F("Sensor 1 fault"));
      if (sensor2Fault) Serial.println(F("Sensor 2 fault"));
      sensorFault = true;
      currentState = ERROR_STATE;
      logSensorData(true);  // Force logging on error
      // Blink status LED rapidly to indicate error
      errorBlink(STATUS_LED, 3);
    }
    lastSensorCheckTime = currentTime;
  }
  
  // Periodic recalibration 
  if (currentTime - lastRecalibrationTime >= RECALIBRATION_INTERVAL && currentState == IDLE) {
    Serial.println(F("Performing periodic recalibration..."));
    calibrateSensors();
    lastRecalibrationTime = currentTime;
  }
  
  // Do regular sensor readings if not in error state
  if (currentState != ERROR_STATE) {
    bool error1 = false, error2 = false;
    int current1 = measureDistance(TRIGGER_PIN1, ECHO_PIN1, &error1);
    int current2 = measureDistance(TRIGGER_PIN2, ECHO_PIN2, &error2);
    
    // Track consecutive errors for each sensor
    if (error1) {
      consecutiveErrors1++;
      if (consecutiveErrors1 >= MAX_READING_ATTEMPTS) {
        sensor1Fault = true;
        sensorFault = true;
        currentState = ERROR_STATE;
        Serial.println(F("Sensor 1 consecutive errors threshold reached"));
        errorBlink(STATUS_LED, 3);
        return;
      }
    } else {
      consecutiveErrors1 = 0;
    }
    
    if (error2) {
      consecutiveErrors2++;
      if (consecutiveErrors2 >= MAX_READING_ATTEMPTS) {
        sensor2Fault = true;
        sensorFault = true;
        currentState = ERROR_STATE;
        Serial.println(F("Sensor 2 consecutive errors threshold reached"));
        errorBlink(STATUS_LED, 3);
        return;
      }
    } else {
      consecutiveErrors2 = 0;
    }
    
    // Debug output with expanded information
    if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
      // Read environmental data
      float temperature = dht.readTemperature();
      float humidity = dht.readHumidity();
      float gasPpm = gasSensor.getPPM();
      
      Serial.println(F("\n--- Environmental Monitoring ---"));
      Serial.print(F("Temperature: "));
      Serial.print(temperature);
      Serial.println(F(" °C"));
      
      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.println(F(" %"));
      
      Serial.print(F("Gas PPM: "));
      Serial.print(gasPpm);
      Serial.println(F(" ppm"));
      
      Serial.println(F("\n--- Occupancy Tracking ---"));
      Serial.print(F("S1: "));
      Serial.print(current1);
      Serial.print(F("cm, S2: "));
      Serial.print(current2);
      Serial.print(F("cm, Occupancy: "));
      Serial.println(occupancy);
      
      Serial.println(F("-----------------------------"));
      lastDebugTime = currentTime;
    }
    
    // Occupancy tracking state machine
    switch(currentState) {
      case IDLE:
        // Check if first sensor is triggered
        if (!error1 && isPersonDetected(current1, baseline1, SENSITIVITY, TRIGGER_PIN1, ECHO_PIN1)) {
          currentState = SENSOR1_TRIGGERED;
          lastDetectionTime = currentTime;
          digitalWrite(LED_ENTER, HIGH);
          Serial.println(F("Sensor 1 Triggered"));
        }
        // Check if second sensor is triggered
        else if (!error2 && isPersonDetected(current2, baseline2, SENSITIVITY, TRIGGER_PIN2, ECHO_PIN2)) {
          currentState = SENSOR2_TRIGGERED;
          lastDetectionTime = currentTime;
          digitalWrite(LED_EXIT, HIGH);
          Serial.println(F("Sensor 2 Triggered"));
        }
        break;
  
      case SENSOR1_TRIGGERED:
        // Timeout if too much time has passed
        if (currentTime - lastDetectionTime > MAX_TRANSIT_TIME) {
          currentState = IDLE;
          digitalWrite(LED_ENTER, LOW);
          Serial.println(F("Sensor 1 Timeout"));
          break;
        }
  
        // Check if second sensor is triggered within valid time window
        if (!error2 && current2 < (baseline2 - SENSITIVITY)) {
          // Validate transit time
          unsigned long transitTime = currentTime - lastDetectionTime;
          if (transitTime >= MIN_TRANSIT_TIME && transitTime <= MAX_TRANSIT_TIME) {
            occupancy++;
            saveOccupancyToEEPROM();
            Serial.print(F("Person Entered - Occupancy: "));
            Serial.println(occupancy);
            logSensorData(true);  // Force log on occupancy change
          } else {
            Serial.print(F("Invalid transit time: "));
            Serial.print(transitTime);
            Serial.println(F("ms - entry ignored"));
          }
          currentState = IDLE;
          digitalWrite(LED_ENTER, LOW);
          lastDetectionTime = currentTime;
        }
        break;
  
      case SENSOR2_TRIGGERED:
        // Timeout if too much time has passed
        if (currentTime - lastDetectionTime > MAX_TRANSIT_TIME) {
          currentState = IDLE;
          digitalWrite(LED_EXIT, LOW);
          Serial.println(F("Sensor 2 Timeout"));
          break;
        }
  
        // Check if first sensor is triggered within valid time window
        if (!error1 && current1 < (baseline1 - SENSITIVITY)) {
          // Validate transit time
          unsigned long transitTime = currentTime - lastDetectionTime;
          if (transitTime >= MIN_TRANSIT_TIME && transitTime <= MAX_TRANSIT_TIME) {
            if (occupancy > 0) {
              occupancy--;
              saveOccupancyToEEPROM();
              Serial.print(F("Person Exited - Occupancy: "));
              Serial.println(occupancy);
              logSensorData(true);  // Force log on occupancy change
            } else {
              Serial.println(F("Exit detected but occupancy already at 0"));
            }
          } else {
            Serial.print(F("Invalid transit time: "));
            Serial.print(transitTime);
            Serial.println(F("ms - exit ignored"));
          }
          currentState = IDLE;
          digitalWrite(LED_EXIT, LOW);
          lastDetectionTime = currentTime;
        }
        break;
        
      case ERROR_STATE:
        // Handled at the beginning of the loop
        break;
    }
  }
  
  // Periodic logging
  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    logSensorData();
    lastLogTime = currentTime;
  }
  
  delay(20);
}

// Helper function to measure distance with error detection
long measureDistance(int trigPin, int echoPin, bool *error) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Set timeout to about 23ms (400cm max sensing distance)
  unsigned long duration = pulseIn(echoPin, HIGH, 25000);
  
  if (duration == 0) {
    if (error != NULL) *error = true;
    return -1; // No echo received
  }
  
  long distance = duration * 0.034 / 2;
  
  // Check for valid range
  if (distance < MIN_VALID_DISTANCE || distance > MAX_VALID_DISTANCE) {
    if (error != NULL) *error = true;
    return distance; // Return the invalid reading
  }
  
  if (error != NULL) *error = false;
  return distance;
}

// Fixed isPersonDetected function
bool isPersonDetected(int currentReading, int baseline, int sensitivity, int trigPin, int echoPin) {
  // Take multiple readings to confirm
  const int confirmationReadings = 3;
  int confirmedCount = 0;
  
  for (int i = 0; i < confirmationReadings; i++) {
    bool error = false;
    int reading = measureDistance(trigPin, echoPin, &error);
    
    if (!error && reading < (baseline - sensitivity)) {
      confirmedCount++;
    }
    delay(10);  // Small delay between readings
  }
  
  // Require majority of readings to confirm detection
  return (confirmedCount >= (confirmationReadings / 2 + 1));
}

// Function to get stable readings
int getStableReading(int trigPin, int echoPin, bool *errorFlag) {
  const int numReadings = 5;
  long readings[numReadings];
  int validReadings = 0;
  bool errors[numReadings] = {false};
  
  for (int i = 0; i < numReadings; i++) {
    bool error = false;
    readings[i] = measureDistance(trigPin, echoPin, &error);
    errors[i] = error;
    
    if (!error && readings[i] > MIN_VALID_DISTANCE && readings[i] < MAX_VALID_DISTANCE) {
      validReadings++;
    }
    delay(50);
  }
  
  // If more than half of readings were errors, report error
  if (validReadings < numReadings / 2) {
    if (errorFlag != NULL) *errorFlag = true;
    return -1;
  }
  
  // Sort and find median of valid readings
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = 0; j < numReadings - i - 1; j++) {
      if (!errors[j] && !errors[j+1] && readings[j] > readings[j+1]) {
        long temp = readings[j];
        readings[j] = readings[j+1];
        readings[j+1] = temp;
      }
    }
  }
  
  // Find median of valid readings
  int medianIndex = 0;
  for (int i = 0; i < numReadings; i++) {
    if (!errors[i]) {
      medianIndex = i + validReadings / 2;
      break;
    }
  }
  
  if (errorFlag != NULL) *errorFlag = false;
  return readings[medianIndex];
}

// Function to perform sensor calibration
void calibrateSensors() {
  // Flash status LED to indicate calibration in progress
  digitalWrite(STATUS_LED, HIGH);
  
  Serial.println(F("--- Starting Sensor Calibration ---"));
  
  bool error1 = false, error2 = false;
  int reading1 = getStableReading(TRIGGER_PIN1, ECHO_PIN1, &error1);
  int reading2 = getStableReading(TRIGGER_PIN2, ECHO_PIN2, &error2);
  
  if (error1 || error2) {
    if (error1) {
      Serial.println(F("Calibration error: Sensor 1"));
      sensor1Fault = true;
    }
    
    if (error2) {
      Serial.println(F("Calibration error: Sensor 2"));
      sensor2Fault = true;
    }
    
    if (sensor1Fault || sensor2Fault) {
      sensorFault = true;
      currentState = ERROR_STATE;
      digitalWrite(STATUS_LED, LOW);
      return;
    }
  }
  
  // Set baselines
  baseline1 = reading1;
  baseline2 = reading2;
  
  Serial.print(F("Calibrated Baselines - S1: "));
  Serial.print(baseline1);
  Serial.print(F("cm, S2: "));
  Serial.print(baseline2);
  Serial.println(F("cm"));
  
  // Reset fault flags after successful calibration
  sensor1Fault = false;
  sensor2Fault = false;
  sensorFault = false;
  consecutiveErrors1 = 0;
  consecutiveErrors2 = 0;
  
  // Safe exit from error state if all is well
  if (currentState == ERROR_STATE) {
    currentState = IDLE;
  }
  
  digitalWrite(STATUS_LED, LOW);
}

// Function to create or update log file
void createLogFileIfNeeded() {
  char filename[] = "LOG.csv";
  
  // Check if file exists
  if (!SD.exists(filename)) {
    File logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      logFile.println(F("Timestamp,Occupancy,Temperature(C),Humidity(%),Gas(ppm),Sensor1(cm),Sensor2(cm),Status"));
      logFile.close();
      Serial.println(F("Log file created"));
    } else {
      Serial.println(F("Failed to create log file"));
    }
  }
}

// Function to log sensor data
void logSensorData(bool forceLog) {
  // Skip if SD is not available
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD Card not available for logging"));
    return;
  }
  
  char filename[] = "LOG.csv";
  File logFile = SD.open(filename, FILE_WRITE);
  
  if (!logFile) {
    Serial.println(F("Cannot open log file"));
    return;
  }
  
  // Get timestamp
  char timestamp[25];
  if (rtc.begin()) {
    DateTime now = rtc.now();
    
    if (now.isValid()) {
      snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d", 
               now.year(), now.month(), now.day(), 
               now.hour(), now.minute(), now.second());
    } else {
      snprintf(timestamp, sizeof(timestamp), "RTC_INVALID");
    }
  } else {
    // Use millis if RTC is not available
    unsigned long uptime = millis() / 1000; // Convert to seconds
    snprintf(timestamp, sizeof(timestamp), "UPTIME:%lu", uptime);
  }
  
  // Read sensor data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float gasPpm = gasSensor.getPPM();
  
  bool error1 = false, error2 = false;
  int distance1 = measureDistance(TRIGGER_PIN1, ECHO_PIN1, &error1);
  int distance2 = measureDistance(TRIGGER_PIN2, ECHO_PIN2, &error2);
  
  String status = "OK";
  if (error1 || error2 || sensorFault) {
    status = "ERROR:";
    if (error1 || sensor1Fault) status += "S1";
    if (error2 || sensor2Fault) status += "S2";
  }
  
  // Write data to log file
  logFile.print(timestamp);
  logFile.print(',');
  logFile.print(occupancy);
  logFile.print(',');
  
  if (isnan(temperature)) logFile.print("ERR");
  else logFile.print(temperature, 1);
  
  logFile.print(',');
  
  if (isnan(humidity)) logFile.print("ERR");
  else logFile.print(humidity, 1);
  
  logFile.print(',');
  logFile.print(gasPpm, 2);
  logFile.print(',');
  
  if (error1) logFile.print("ERR"); 
  else logFile.print(distance1);
  
  logFile.print(',');
  
  if (error2) logFile.print("ERR"); 
  else logFile.print(distance2);
  
  logFile.print(',');
  logFile.println(status);
  
  logFile.close();
  Serial.println(F("Data logged"));
}

// Function to validate sensor health
bool validateUltrasonicSensor(int trigPin, int echoPin) {
  const int testReadings = 3;
  int validCount = 0;
  long readings[testReadings];
  
  for (int i = 0; i < testReadings; i++) {
    bool error = false;
    readings[i] = measureDistance(trigPin, echoPin, &error);
    
    if (!error && readings[i] >= MIN_VALID_DISTANCE && readings[i] <= MAX_VALID_DISTANCE) {
      validCount++;
    }
    delay(20);
  }
  
  // Consider sensor healthy if at least 2/3 readings are valid
  return (validCount >= testReadings * 2 / 3);
}

// Function to attempt recovery from faults
void recoverFromFault() {
  Serial.println(F("Attempting to recover from sensor fault..."));
  
  // Flash status LED to show recovery attempt
  for (int i = 0; i < 2; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  // Turn off indicator LEDs
  digitalWrite(LED_ENTER, LOW);
  digitalWrite(LED_EXIT, LOW);
  
  // Try recalibrating sensors
  calibrateSensors();
  
  // Check sensor status
  sensor1Fault = !validateUltrasonicSensor(TRIGGER_PIN1, ECHO_PIN1);
  sensor2Fault = !validateUltrasonicSensor(TRIGGER_PIN2, ECHO_PIN2);
  
  if (!sensor1Fault && !sensor2Fault) {
    sensorFault = false;
    currentState = IDLE;
    
    // Blink all LEDs to show recovery success
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_ENTER, HIGH);
      digitalWrite(LED_EXIT, HIGH);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(LED_ENTER, LOW);
      digitalWrite(LED_EXIT, LOW);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
    }
    
    Serial.println(F("System successfully recovered from fault"));
  } else {
    // Keep status LED on to indicate persisting error
    digitalWrite(STATUS_LED, HIGH);
    
    Serial.println(F("Recovery failed - sensors still faulty"));
    if (sensor1Fault) Serial.println(F("Sensor 1 still faulty"));
    if (sensor2Fault) Serial.println(F("Sensor 2 still faulty"));
  }
}

// Function to save occupancy to EEPROM
void saveOccupancyToEEPROM() {
  EEPROM.write(EEPROM_OCCUPANCY_ADDR, occupancy);
}

// Function to load occupancy from EEPROM
void loadOccupancyFromEEPROM() {
  occupancy = EEPROM.read(EEPROM_OCCUPANCY_ADDR);
  // Sanity check - if occupancy is unrealistically high, reset it
  if (occupancy > 50) {
    occupancy = 0;
    saveOccupancyToEEPROM();
  }
  Serial.print(F("Loaded occupancy from EEPROM: "));
  Serial.println(occupancy);
}

// Function to handle reset button press
void handleButtonPress() {
  // Debounce
  delay(50);
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    unsigned long pressStartTime = millis();
    
    // Print indication of button press
    Serial.println(F("Reset button pressed - hold for 3 seconds to reset occupancy"));
    
    // Wait for button release or long press
    while (digitalRead(RESET_BUTTON_PIN) == LOW) {
      unsigned long pressDuration = millis() - pressStartTime;
      
      // Every 500ms, blink status LED to show progress
      if (pressDuration % 500 < 250) {
        digitalWrite(STATUS_LED, HIGH);
      } else {
        digitalWrite(STATUS_LED, LOW);
      }
      
      // Long press = reset occupancy
      if (pressDuration > 3000) {
        occupancy = 0;
        saveOccupancyToEEPROM();
        
        Serial.println(F("OCCUPANCY RESET TO ZERO"));
        
        // Indicate reset with LEDs
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_ENTER, HIGH);
          digitalWrite(LED_EXIT, HIGH);
          digitalWrite(STATUS_LED, HIGH);
          delay(200);
          digitalWrite(LED_ENTER, LOW);
          digitalWrite(LED_EXIT, LOW);
          digitalWrite(STATUS_LED, LOW);
          delay(200);
        }
        
        // Wait for button release
        while (digitalRead(RESET_BUTTON_PIN) == LOW) {
          delay(10);
        }
        
        digitalWrite(STATUS_LED, LOW);
        return;
      }
      
      delay(10);
    }
    
    digitalWrite(STATUS_LED, LOW);
    
    // Short press = force recalibration
    Serial.println(F("Short press detected - performing manual recalibration"));
    calibrateSensors();
  }
}

// Function to indicate errors with LED blinking
void errorBlink(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(200);
    digitalWrite(pin, LOW);
    delay(200);
  }
}