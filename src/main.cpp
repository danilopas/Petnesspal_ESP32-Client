#include <Firebase_ESP_Client.h>
#include <Arduino.h>
#include <HX711.h>
#include "Stepper.h" // stepper motor
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include "soc/rtc.h"
// #include <deque>
#include <LiquidCrystal_I2C.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

using namespace std;

// Initialize the LCD, change the address 0x27 if necessary
LiquidCrystal_I2C lcd(0x27, 16, 2); // Pins for LCD - SDA = 21, SCL = 22

// Definitions for HX711_petTray
const int petTray_dout = 16;
const int petTray_sck = 4;
const int platformLoadCell_dout = 32;
const int platformLoadCell_sck = 33;
// Calibration values
float calibrationValue_petTray = 1955.32;  // petTray Calibration - Past Values: 2291.28 , 2642.85, 2542.704,  1955.32
float calibrationValue_platform = -21.114; // platform Calibration - Past Values: 21.68
// LoadCell objects+
HX711 LoadCell_petTray;
HX711 LoadCell_platform;

boolean petTray_newDataReady = false;

boolean dispenseMotor1ACounter = false;

unsigned long t = 0;

// Relay config/constructor
int motor1Relay = 25; // Relay for motor1
int motor2Relay = 13; // Relay for motor2

// Motor Config/constructor
const int motorSpeed = 100;
const int stepsPerRevolution = 200;
const int smallStep = 250;                                    // Small step size for motor control
Stepper motor1 = Stepper(stepsPerRevolution, 12, 14, 27, 26); // motor1
Stepper motor2 = Stepper(stepsPerRevolution, 17, 5, 18, 19);  // motor2

// AmountToDispense
float amountToDispense = 0;           // Initialized to 0, will be updated by HTTP request
String userName = "";                 // Initialized to 0, will be updated by HTTP request
String selectedFood = "";             // Initialized to 0, will be updated by HTTP request
String cageID = "";                   // Initialized to 0, will be updated by HTTP request
String feedingModeType = "";          // Initialized to 0, will be updated by HTTP request
String scheduleDate = "";             // Initialized to 0, will be updated by HTTP request
String scheduledTime = "";            // Initialized to 0, will be updated by HTTP request
const char *clientCageID = "cage_01"; // Predefined cageID
boolean dataReceived = false;         // Flag to check if data is received
float initialPlatformWeight = 0.0;    // Initialized to 0, will be updated by load cell
float initialTrayWeight = 0.0;        // Initialized to 0, will be updated by load cell

// Ultrasonic Sensor Config
const int foodOneTrigPin = 23;
const int foodOneEchoPin = 34;
const int foodTwoTrigPin = 15;
const int foodTwoEchoPin = 35;

// Fluctuation handling
vector<float> weightReadings;
int consistentCount = 0;
const int smoothingWindow = 5;             // Number of readings to average
const int consistentReadingsThreshold = 3; // Number of consistent readings before stopping
const float tolerance = 0.7;               // Allowable error margin in grams

// Selected Motor for Dispensing Food
int selectedMotor = 0;

// Define the base distance (cm) for 0% output
const int baseDistance = 20; // Modify this value to set the base distance for 0%

// Definitions for WiFi
const char *WIFI_SSID = "Smartbro-4BC4"; // Baklang Theozoids // GlobeAtHome_8B831 // Test_3
const char *WIFI_PASSWORD = "smartbro";  // Badingakomalala@123 // 4588CDE3 // test3null

// Definitions for Firebase Database
const char *FIREBASE_HOST = "https://petness-92c55-default-rtdb.asia-southeast1.firebasedatabase.app/";
const char *FIREBASE_AUTH = "bhvzGLuvbjReHlQjk77UwWGtCVdBBUBABE3X4PQ2";
// Definitions for Firestore Database
const char *FIREBASE_FIRESTORE_HOST = "petness-92c55";
// Define the API Key
const char *API_KEY = "AIzaSyDPcMRU9x421wP0cS1sRHwEvi57W8NoLiE";
// Define the user Email and password that already registerd or added in your project
const char *USER_EMAIL = "petnessadmin@gmail.com";
const char *USER_PASSWORD = "petness";

// Declare the FirebaseData objects at the global scope
FirebaseData fbdo1; // For Realtime Database Weight Stream
FirebaseData fbdo2; // For Firestore Record
FirebaseData fbdo3; // For Realtime Database Food Stream
FirebaseConfig config;
FirebaseAuth auth;
FirebaseJson content;

// Declare the paths at the global scope
String pathGetPetWeight = "/trigger/getPetWeight/status";
String pathGetFoodAmount = "/trigger/getPetFood/status";

WebServer server(80);

// Additional Global Variables for Motor Control and Data Handling
static bool newDataReady = false;
int rotationCount = 0;
bool isClockwise = true;

// function prototypes
void handleReceive();
void dispenseMotor1A();
void dispenseMotor1B();
void stopDispenseMotor1();
void recordFeedingDataToFirestore(const String &mode, const String &userName, float amount, const String &scheduleDate, const String &scheduledTime, const String &cageID, float initialWeight, float foodConsumed);
String generateRandomString(int length);
void connectToWifi();
void configureLoadCells();
void initializeFirebase();
void initializeMotor();
float samplesForGettingWeight();
void setPetWeight();
void petWeightStream();
void printLoadCellWeights();
bool checkConsistentWeight(float targetWeight, float tolerance);
float calculateFoodConsumption();
int selectMotor(const String &selectedFood);
void dispenseMotor2A();
void dispenseMotor2B();
void stopDispenseMotor2();
void activateMotor1Relay();
void activateMotor2Relay();
void ultrasonicConfig();
int measureDistance1();
int measureDistance2();
int calculatePercentage(int distance, int baseDistance);
void petFoodStream();
void setFoodAmount();
void calibrateLoadCells();
void updateWeightReadings();
bool checkThresholdDuringDispense(float targetWeight, float tolerance);

void setup()
{
  Serial.begin(115200);

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Welcome to"); // Print first part of message
  lcd.setCursor(0, 1);     // Move cursor to the beginning of the second line
  lcd.print("PetnessPal"); // Print second part of message

  delay(3000);

  lcd.clear();

  connectToWifi();
  delay(2000);
  configureLoadCells();
  delay(2000);
  // calibrateLoadCells();
  // delay(2000);
  initializeFirebase();
  delay(2000);
  initializeMotor();
  delay(2000);
  ultrasonicConfig();

  // randomSeed(analogRead(0));

  // Initialize server routes
  server.on("/receive", HTTP_POST, handleReceive);
  server.begin();
  Serial.println("Server started");

  delay(2000);

  // Display server status on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Server started");
  lcd.setCursor(0, 1);
  lcd.print("Listening...");
}

unsigned long lastPetWeightStreamTime = 0;
unsigned long lastPetFoodStreamTime = 0;

void loop()
{
  server.handleClient();

  selectedMotor = selectMotor(selectedFood); // Example: change "food_01" to your actual variable

  unsigned long currentMillis = millis();
  const int serialPrintInterval = 100;
  static unsigned long lastPrintTime = 0;

  if (LoadCell_petTray.is_ready())
  {
    newDataReady = true;
  }

  if (dataReceived)
  {
    if (newDataReady)
    {
      static int rotationCount = 0;
      static bool isClockwise = true;

      if (millis() > t + serialPrintInterval)
      {
        float petTrayAmount = LoadCell_petTray.get_units(1);
        Serial.println(petTrayAmount);
        weightReadings.push_back(petTrayAmount);
        // long rawValue = LoadCell_petTray.read();
        // float petTrayAmount = (float)(rawValue - LoadCell_petTray.get_offset()) / LoadCell_petTray.get_scale();
        // weightReadings.push_back(petTrayAmount);

        if (weightReadings.size() > smoothingWindow)
        {
          weightReadings.erase(weightReadings.begin()); // Remove the oldest reading
        }

        if (!weightReadings.empty())
        {
          float sum = accumulate(weightReadings.begin(), weightReadings.end(), 0.0);
          float smoothedWeight = sum / weightReadings.size();

          Serial.print("Current Smoothed Weight: ");
          Serial.println(smoothedWeight);

          if (checkThresholdDuringDispense(smoothedWeight, tolerance) && checkConsistentWeight(smoothedWeight, tolerance))
          {
            if (smoothedWeight >= amountToDispense)
            {
              if (selectedMotor == 1)
              {
                stopDispenseMotor1();
              }
              else if (selectedMotor == 2)
              {
                stopDispenseMotor2();
              }
              Serial.println(" -- Reached the Threshold, STOP");
              Serial.print("Smoothed load cell output val: ");
              Serial.println(smoothedWeight);
              Serial.println("Dispensing Finished");
              Serial.println("Calculating Food Consumed");
              float foodConsumed = calculateFoodConsumption();
              Serial.println("Recording Feeding Data to Firestore");
              recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialPlatformWeight, foodConsumed);
              Serial.println("Resetting the Data Received Flag");
              dataReceived = false;
              newDataReady = false;
            }
            else
            {
              // Adjust rotation limit based on selected motor
              int rotationLimit = (selectedMotor == 1) ? 3 : 4;

              if (rotationCount < rotationLimit)
              {
                if (isClockwise)
                {
                  if (selectedMotor == 1)
                  {
                    activateMotor1Relay();
                    Serial.println(" Rotating Clockwise Relay 1 (a)--");
                    dispenseMotor1A();
                  }
                  else if (selectedMotor == 2)
                  {
                    activateMotor2Relay();
                    Serial.println(" Rotating Clockwise Relay 2 (a)--");
                    dispenseMotor2A();
                  }
                  Serial.print("Smoothed load cell output val: ");
                  Serial.println(smoothedWeight);
                }
                else
                {
                  if (selectedMotor == 1)
                  {
                    activateMotor1Relay();
                    Serial.println("Counter Clockwise Relay 1 (b)--");
                    dispenseMotor1B();
                  }
                  else if (selectedMotor == 2)
                  {
                    activateMotor2Relay();
                    Serial.println("Counter Clockwise Relay 2 (b)--");
                    dispenseMotor2B();
                  }
                  Serial.print("Smoothed load cell output val: ");
                  Serial.println(smoothedWeight);
                }
                rotationCount++;
              }
              else
              {
                rotationCount = 0;
                isClockwise = !isClockwise; // Toggle direction
              }
              Serial.print("Smoothed load cell output val: ");
              Serial.println(smoothedWeight);
              newDataReady = false;
              t = millis();
            }
          }
          else
          {
            Serial.println("Not enough consistent readings.");
          }
        }
        else
        {
          Serial.println("Weight readings array is empty, skipping smoothed weight calculation.");
        }
      }
    }
  }
  else
  {
    if (millis() - lastPrintTime >= 1000)
    {
      Serial.println("Waiting for data...");
      lastPrintTime = millis();
    }
  }

  if (currentMillis - lastPetWeightStreamTime >= 250)
  {
    petWeightStream();
    lastPetWeightStreamTime = currentMillis;
  }

  if (currentMillis - lastPetFoodStreamTime >= 250)
  {
    petFoodStream();
    lastPetFoodStreamTime = currentMillis;
  }
}

bool checkConsistentWeight(float targetWeight, float tolerance)
{
  consistentCount = 0;
  for (float reading : weightReadings)
  {
    if (abs(reading - targetWeight) <= tolerance)
    {
      consistentCount++;
    }
  }
  Serial.print("Consistent Count: ");
  Serial.println(consistentCount);
  return consistentCount >= consistentReadingsThreshold;
}

bool checkThresholdDuringDispense(float threshold, float tolerance)
{
  if (weightReadings.empty())
  {
    return false; // Ensure weightReadings is not empty
  }

  float smoothedWeight = accumulate(weightReadings.begin(), weightReadings.end(), 0.0) / weightReadings.size();

  Serial.print("Current Smoothed Weight: ");
  Serial.println(smoothedWeight);
  lcd.setCursor(0, 0);
  lcd.print("Amount Dispensed");
  lcd.setCursor(0, 1);
  lcd.print(String(smoothedWeight));

  if (abs(smoothedWeight - threshold) <= tolerance)
  {
    consistentCount++;
    Serial.print("Consistent Count: ");
    Serial.println(consistentCount);

    if (consistentCount >= consistentReadingsThreshold)
    { // Adjust consistentCountThreshold if needed
      return true;
    }
  }
  else
  {
    consistentCount = 0; // Reset if the weight is not within the tolerance
  }

  return false;
}

void updateWeightReadings()
{
  float petTrayAmount = LoadCell_petTray.get_units();
  weightReadings.push_back(petTrayAmount);
  if (weightReadings.size() > smoothingWindow)
  {
    weightReadings.erase(weightReadings.begin()); // Remove the oldest reading
  }
  Serial.print("Current Weight Reading: ");
  Serial.println(petTrayAmount);
}

int selectMotor(const String &selectedFood)
{
  if (selectedFood == "food_01")
  {
    return 1;
  }
  else if (selectedFood == "food_02")
  {
    return 2;
  }
  else
  {
    return 0;
  }
}

void dispenseMotor1A()
{
  motor1.setSpeed(motorSpeed);
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor1.step(smallStep);
    delay(10);

    if (!weightReadings.empty())
    {
      float sum = std::accumulate(weightReadings.begin(), weightReadings.end(), 0.0);
      size_t readingSize = weightReadings.size();

      if (readingSize > 0)
      {
        float smoothedWeight = sum / readingSize;

        if (checkThresholdDuringDispense(amountToDispense, tolerance))
        {
          if (checkConsistentWeight(amountToDispense, tolerance))
          {
            stopDispenseMotor1();
            Serial.print("Reached threshold during dispenseMotor1A: ");
            lcd.setCursor(0, 0);
            lcd.print("Amount Dispensed");
            lcd.setCursor(0, 1);
            lcd.print(String(amountToDispense));
            Serial.println(amountToDispense);
            Serial.println("Calculating Food Consumed");
            float foodConsumed = calculateFoodConsumption();
            // LoadCell_petTray.tare();
            Serial.println("Recording Feeding Data to Firestore");
            recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialPlatformWeight, foodConsumed);
            lcd.setCursor(0, 0);
            lcd.print("Stored Data");
            lcd.setCursor(0, 1);
            lcd.print("Successfully");
            dataReceived = false;
            return;
          }
        }
      }
    }
    else
    {
      Serial.println("Weight readings array is empty, skipping smoothed weight calculation.");
    }
  }
}

void dispenseMotor1B()
{
  motor1.setSpeed(motorSpeed);
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor1.step(-smallStep);
    delay(10);

    if (!weightReadings.empty())
    {
      float sum = std::accumulate(weightReadings.begin(), weightReadings.end(), 0.0);
      size_t readingSize = weightReadings.size();

      if (readingSize > 0)
      {
        float smoothedWeight = sum / readingSize;

        if (checkThresholdDuringDispense(amountToDispense, tolerance))
        {
          if (checkConsistentWeight(amountToDispense, tolerance))
          {
            stopDispenseMotor1();
            Serial.print("Reached threshold during dispenseMotor1B: ");
            lcd.setCursor(0, 0);
            lcd.print("Amount Dispensed");
            lcd.setCursor(0, 1);
            lcd.print(String(amountToDispense));
            Serial.println(amountToDispense);
            Serial.println("Calculating Food Consumed");
            float foodConsumed = calculateFoodConsumption();
            // LoadCell_petTray.tare();
            Serial.println("Recording Feeding Data to Firestore");
            recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialPlatformWeight, foodConsumed);
            lcd.setCursor(0, 0);
            lcd.print("Stored Data");
            lcd.setCursor(0, 1);
            lcd.print("Successfully");
            dataReceived = false;
            return;
          }
        }
      }
    }
    else
    {
      Serial.println("Weight readings array is empty, skipping smoothed weight calculation.");
    }
  }
}

void dispenseMotor2A()
{
  motor2.setSpeed(motorSpeed);
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor2.step(smallStep);
    delay(10);

    if (!weightReadings.empty())
    {
      float sum = std::accumulate(weightReadings.begin(), weightReadings.end(), 0.0);
      size_t readingSize = weightReadings.size();

      if (readingSize > 0)
      {
        float smoothedWeight = sum / readingSize;

        if (checkThresholdDuringDispense(amountToDispense, tolerance))
        {
          if (checkConsistentWeight(amountToDispense, tolerance))
          {
            stopDispenseMotor2();
            Serial.print("Reached threshold during dispenseMotor2A: ");
            lcd.setCursor(0, 0);
            lcd.print("Amount Dispensed");
            lcd.setCursor(0, 1);
            lcd.print(String(amountToDispense));
            Serial.println(amountToDispense);
            Serial.println("Calculating Food Consumed");
            float foodConsumed = calculateFoodConsumption();
            // LoadCell_petTray.tare();
            Serial.println("Recording Feeding Data to Firestore");
            recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialPlatformWeight, foodConsumed);
            lcd.setCursor(0, 0);
            lcd.print("Stored Data");
            lcd.setCursor(0, 1);
            lcd.print("Successfully");
            dataReceived = false;
            return;
          }
        }
      }
    }
    else
    {
      Serial.println("Weight readings array is empty, skipping smoothed weight calculation.");
    }
  }
}

void dispenseMotor2B()
{
  motor2.setSpeed(motorSpeed);
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor2.step(-smallStep);
    delay(10);

    if (!weightReadings.empty())
    {
      float sum = std::accumulate(weightReadings.begin(), weightReadings.end(), 0.0);
      size_t readingSize = weightReadings.size();

      if (readingSize > 0)
      {
        float smoothedWeight = sum / readingSize;

        if (checkThresholdDuringDispense(amountToDispense, tolerance))
        {
          if (checkConsistentWeight(amountToDispense, tolerance))
          {
            stopDispenseMotor2();
            Serial.print("Reached threshold during dispenseMotor2B: ");
            lcd.setCursor(0, 0);
            lcd.print("Amount Dispensed");
            lcd.setCursor(0, 1);
            lcd.print(String(amountToDispense));
            Serial.println(amountToDispense);
            Serial.println("Calculating Food Consumed");
            float foodConsumed = calculateFoodConsumption();
            // LoadCell_petTray.tare();
            Serial.println("Recording Feeding Data to Firestore");
            recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialPlatformWeight, foodConsumed);
            lcd.setCursor(0, 0);
            lcd.print("Stored Data");
            lcd.setCursor(0, 1);
            lcd.print("Successfully");
            dataReceived = false;
            return;
          }
        }
      }
    }
    else
    {
      Serial.println("Weight readings array is empty, skipping smoothed weight calculation.");
    }
  }
}

void stopDispenseMotor1()
{
  // motor1.setSpeed(0);              // Stop motor1
  digitalWrite(motor1Relay, HIGH); // Turn off the relay to stop the motor
  digitalWrite(motor1Relay, HIGH); // Turn off the relay to stop the motor
}

void stopDispenseMotor2()
{
  // motor2.setSpeed(0);              // Stop motor2
  digitalWrite(motor2Relay, HIGH); // Turn off the relay to stop the motor
  digitalWrite(motor2Relay, HIGH); // Turn off the relay to stop the motor
}

void initializeMotor()
{
  // Relay Config
  pinMode(motor1Relay, OUTPUT);
  pinMode(motor2Relay, OUTPUT);
  digitalWrite(motor1Relay, HIGH);
  digitalWrite(motor2Relay, HIGH);

  // Motor Config
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Motors Status:");
  lcd.setCursor(0, 1);
  lcd.print("Ready");
}

void activateMotor1Relay()
{
  digitalWrite(motor1Relay, LOW);
  digitalWrite(motor2Relay, HIGH);
}

void activateMotor2Relay()
{
  digitalWrite(motor1Relay, HIGH);
  digitalWrite(motor2Relay, LOW);
}

void calibrateLoadCells()
{
  // For Tray Calibration

  // LoadCell_petTray.begin(petTray_dout, petTray_sck);
  // if (LoadCell_petTray.is_ready())
  // {
  //   LoadCell_petTray.set_scale();
  //   Serial.println("Tare... remove any weights from the scale.");
  //   delay(5000);
  //   LoadCell_petTray.tare();
  //   Serial.println("LoadCell_petTray Tare done...");
  //   Serial.print("Place a known weight on the scale...");
  //   delay(10000);
  //   long reading = LoadCell_petTray.get_units(10);
  //   Serial.print("TRAY Result: ");
  //   Serial.println(reading);
  // }
  // else
  // {
  //   Serial.println("HX711 not found.");
  // }

  // delay(1000);

  // For Platform Calibration
  LoadCell_platform.begin(platformLoadCell_dout, platformLoadCell_sck);

  if (LoadCell_platform.is_ready())
  {
    LoadCell_platform.set_scale();
    Serial.println("Tare... remove any weights from the scale.");
    delay(5000);
    LoadCell_platform.tare();
    Serial.println("LoadCell_platform Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(10000);
    long reading = LoadCell_platform.get_units(10);
    Serial.print("PLATFORM Result: ");
    Serial.println(reading);
  }
  else
  {
    Serial.println("HX711 not found.");
  }
}

void printLoadCellWeights()
{
  Serial.print("PET TRAY WEIGHT: ");
  float petTrayWeight = LoadCell_petTray.get_units();
  Serial.println(petTrayWeight);

  Serial.print("PLATFORM WEIGHT: ");
  float platformWeight = LoadCell_platform.get_units();
  Serial.println(platformWeight);
}

void initializeFirebase()
{
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Serial.println("Settting Up Firebase");
  Firebase.begin(&config, &auth);
  Serial.println("Done Firebase Configure");

  if (Firebase.RTDB.beginStream(&fbdo1, pathGetPetWeight.c_str()))
  {
    Serial.println("Stream getPetWeight begin, success");
    // Display Firebase initialization status on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Firebase Ready");
  }
  else
  {
    Serial.print("Stream begin failed, reason: ");
    Serial.println(fbdo1.errorReason());
    // Display Firebase initialization failure on LCD
    lcd.setCursor(0, 1);
    lcd.print("Firebase Failed");
  }

  if (Firebase.RTDB.beginStream(&fbdo3, pathGetFoodAmount.c_str()))
  {
    Serial.println("Stream getPetFood begin, success");
    // Display Firebase initialization status on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Firebase Ready");
  }
  else
  {
    Serial.print("Stream begin failed, reason: ");
    Serial.println(fbdo3.errorReason());
    // Display Firebase initialization failure on LCD
    lcd.setCursor(0, 1);
    lcd.print("Firebase Failed");
  }
}

void connectToWifi()
{
  // Initialize WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Display connecting status on LCD
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  delay(500);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  lcd.clear();
  // Display connected status on LCD
  lcd.setCursor(0, 0);
  lcd.print("IP Address: ");

  // Optionally, display the IP address on the second line of the LCD
  lcd.setCursor(0, 1);
  // lcd.print("IP:");
  lcd.print(WiFi.localIP().toString()); // Convert IP address to string and display
}

void handleReceive()
{
  if (server.hasArg("plain"))
  {
    String body = server.arg("plain");

    // Print received data to Serial
    Serial.print("Received data: ");
    Serial.println(body);

    // Increase the size of the JsonDocument (adjust as needed)
    StaticJsonDocument<512> jsonDoc;

    // Parse received data to JSON
    DeserializationError error = deserializeJson(jsonDoc, body);

    // Check for errors
    if (error)
    {
      Serial.print("JSON deserialization failed: ");
      Serial.println(error.c_str());
      server.send(400, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid JSON\"}");
      return;
    }

    // Extract fields from JSON
    const char *receivedCageID = jsonDoc["cageID"];
    const char *receivedUserName = jsonDoc["userName"];
    float receivedAmountToDispense = jsonDoc["amountToDispense"].as<float>(); // Extract amountToDispense as float
    const char *receivedSelectedFood = jsonDoc["selectedFood"];
    const char *receivedFeedingModeType = jsonDoc["feedingModeType"];
    const char *receivedScheduleDate = jsonDoc["scheduledDate"]; // Corrected the field name
    const char *receivedScheduledTime = jsonDoc["scheduledTime"];

    // Check if the received cageID matches the predefined cageID
    if (strcmp(receivedCageID, clientCageID) == 0)
    {
      // Update global variables with received data
      cageID = receivedCageID;
      amountToDispense = receivedAmountToDispense;
      userName = receivedUserName;
      selectedFood = receivedSelectedFood;
      feedingModeType = receivedFeedingModeType;
      scheduleDate = receivedScheduleDate; // Corrected the field name
      scheduledTime = receivedScheduledTime;

      Serial.println("Receiving Data...");
      // Display received data on LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Data Received");
      lcd.setCursor(0, 1);
      lcd.print("Fetching Weight");

      Serial.println("Getting Initial Weight...");

      // Get initial weight before dispensing
      float initialWeightGramsToKG = samplesForGettingWeight() / 1000;    // Convert to kilograms
      initialPlatformWeight = roundf(initialWeightGramsToKG * 100) / 100; // Round to nearest 0.01
      Serial.println("Getting Initial Tray Weight...");
      initialTrayWeight = LoadCell_petTray.get_units(1);
      lcd.clear();

      // Reset state for new dispensing session
      weightReadings.clear();
      dataReceived = true;
      newDataReady = false;
      rotationCount = 0;
      isClockwise = true;

      // Prepare the response JSON
      StaticJsonDocument<256> responseDoc;
      responseDoc["status"] = "success";
      responseDoc["clientCageID"] = clientCageID;
      responseDoc["userName"] = userName;
      responseDoc["selectedFood"] = selectedFood;
      responseDoc["feedingModeType"] = feedingModeType;
      responseDoc["amountToDispense"] = amountToDispense;
      responseDoc["scheduleDate"] = scheduleDate; // Corrected the field name
      responseDoc["scheduledTime"] = scheduledTime;

      // Serialize response JSON to string
      String response;
      serializeJson(responseDoc, response);

      server.send(200, "application/json", response);
    }
    else
    {
      server.send(403, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid Cage ID\"}");
    }
  }
  else
  {
    server.send(400, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid request\"}");
  }

  Serial.print("Free heap after handling request: ");
  Serial.println(ESP.getFreeHeap());
}

void configureLoadCells()
{
  LoadCell_petTray.begin(petTray_dout, petTray_sck);
  LoadCell_petTray.set_scale(calibrationValue_petTray);
  LoadCell_petTray.tare();
  Serial.println("Pet tray load cell initialized");

  LoadCell_platform.begin(platformLoadCell_dout, platformLoadCell_sck);
  LoadCell_platform.set_scale(calibrationValue_platform);
  LoadCell_platform.tare();
  Serial.println("Platform load cell initialized");

  // Display load cell configuration status on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Status: Done");
}

String generateRandomString(int length)
{
  String chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  String randomString = "";
  for (int i = 0; i < length; i++)
  {
    int index = random(0, chars.length());
    randomString += chars[index];
  }
  return randomString;
}

void recordFeedingDataToFirestore(const String &mode, const String &userName, float amount, const String &scheduleDate, const String &scheduledTime, const String &cageID, float initialPlatformWeight, float foodConsumed)
{
  // Debugging: Print the values of userName and scheduleDate
  Serial.print("userName: ");
  Serial.println(userName);
  Serial.print("scheduleDate: ");
  Serial.println(scheduleDate);

  // Check if userName or scheduleDate is empty
  if (userName.length() == 0 || scheduleDate.length() == 0)
  {
    Serial.println("Failed to add feeding record, reason: userName or scheduleDate is empty.");
    return;
  }

  content.clear(); // Clear previous content to avoid conflicts
  content.set("fields/mode/stringValue", mode);
  content.set("fields/userName/stringValue", userName);
  content.set("fields/amount/doubleValue", amount);
  content.set("fields/date/stringValue", scheduleDate); // Corrected the field name
  content.set("fields/time/stringValue", scheduledTime);
  content.set("fields/cageID/stringValue", cageID);
  content.set("fields/weight/doubleValue", initialPlatformWeight);
  content.set("fields/foodConsumed/doubleValue", foodConsumed);

  // Debugging: Print the content JSON
  Serial.println("Content JSON to be sent:");
  String contentString;
  content.toString(contentString);
  Serial.println(contentString);

  String documentPath = "pets/";
  documentPath.concat(userName);
  documentPath.concat("/records/");
  documentPath.concat(generateRandomString(20)); // Adding a unique identifier to the record path

  Serial.print("Document path: ");
  Serial.println(documentPath);

  if (Firebase.Firestore.createDocument(&fbdo2, FIREBASE_FIRESTORE_HOST, "", documentPath.c_str(), content.raw()))
  {
    Serial.println("Feeding record added successfully.");
    Serial.printf("ok\n%s\n\n", fbdo2.payload().c_str());
    // Display successful recording on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dispensing Done");
    lcd.setCursor(0, 1);
    lcd.print("Data Recorded");

    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waiting for");
    lcd.setCursor(0, 1);
    lcd.print("New Schedule...");
  }
  else
  {
    Serial.print("Failed to add feeding record, reason: ");
    Serial.println(fbdo2.errorReason());
    // Display error message on LCD
    lcd.setCursor(0, 1);
    lcd.print("Record Failed");
  }
}

float samplesForGettingWeight()
{
  const int numSamples = 10;
  float totalWeight = 0.0;
  for (int i = 0; i < numSamples; ++i)
  {
    totalWeight += LoadCell_platform.get_units(10);
    delay(100);
  }
  return totalWeight / numSamples;
}

void setPetWeight()
{
  float petsWeight = samplesForGettingWeight() / 1000;      // Convert to kilograms
  float petsWeightRounded = roundf(petsWeight * 100) / 100; // Round to nearest 0.1

  Serial.println("Weight data received from load cell");
  Serial.print("Weight: ");
  Serial.println(petsWeightRounded);

  String path = "/getWeight/loadCell/weight";
  if (Firebase.RTDB.setFloat(&fbdo1, path.c_str(), petsWeightRounded))
  {
    Serial.println("Weight data updated successfully.");
  }
  else
  {
    Serial.print("Failed to update weight data, reason: ");
    Serial.println(fbdo1.errorReason());
  }

  if (Firebase.RTDB.setBool(&fbdo1, "/trigger/getPetWeight/status", false))
  {
    Serial.println("Status of Weight Stream set to false");
  }
  else
  {
    Serial.print("Failed to set status of Weight Stream, reason: ");
    Serial.println(fbdo1.errorReason());
  }
}

void petWeightStream()
{
  if (Firebase.RTDB.readStream(&fbdo1))
  {
    if (fbdo1.streamTimeout())
    {
      Serial.println("Stream timeout, no data received from Firebase");
    }
    else if (fbdo1.dataType() == "boolean")
    {
      bool status = fbdo1.boolData();
      Serial.print("Status of Weight Stream: ");
      Serial.println(String(status).c_str());
      if (status)
      {
        setPetWeight();
      }
    }
  }
  else
  {
    Serial.print("Failed to read stream, reason: ");
    Serial.println(fbdo1.errorReason());
  }
}

float calculateFoodConsumption()
{
  float petTrayWeight = LoadCell_petTray.get_units(1);
  float foodConsumed = initialTrayWeight - petTrayWeight;
  return foodConsumed;
}

void ultrasonicConfig()
{
  pinMode(foodOneTrigPin, OUTPUT);
  pinMode(foodOneEchoPin, INPUT);
  pinMode(foodTwoTrigPin, OUTPUT);
  pinMode(foodTwoEchoPin, INPUT);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ultrasonic");
  lcd.setCursor(0, 1);
  lcd.print("Configured");
}

// Function to measure distance1 using the ultrasonic sensor
int measureDistance1()
{
  // Clear the trigPin
  digitalWrite(foodOneTrigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(foodOneTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(foodOneTrigPin, LOW);

  // Read the echoPin and calculate the duration of the pulse
  long duration = pulseIn(foodOneEchoPin, HIGH);

  // Calculate the distance in centimeters
  int distance1 = duration * 0.034 / 2;

  return distance1;
}
// Function to measure distance2 using the ultrasonic sensor
int measureDistance2()
{
  // Clear the trigPin
  digitalWrite(foodTwoTrigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(foodTwoTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(foodTwoTrigPin, LOW);

  // Read the echoPin and calculate the duration of the pulse
  long duration = pulseIn(foodTwoEchoPin, HIGH);

  // Calculate the distance in centimeters
  int distance2 = duration * 0.034 / 2;

  return distance2;
}

// Function to calculate the percentage based on the distance
int calculatePercentage(int distance, int baseDistance)
{
  // Ensure distance doesn't exceed the base distance
  if (distance > baseDistance)
  {
    distance = baseDistance;
  }

  // Map the distance to a percentage where 0 cm is 100% and baseDistance cm is 0%
  int percentage = map(distance, 0, baseDistance, 100, 0);

  return percentage;
}

void petFoodStream()
{
  if (Firebase.RTDB.readStream(&fbdo3))
  {
    if (fbdo3.streamTimeout())
    {
      Serial.println("Stream timeout, no data received from Firebase");
    }
    else if (fbdo3.dataType() == "boolean")
    {
      bool status = fbdo3.boolData();
      Serial.print("Status of Food Stream: ");
      Serial.println(String(status).c_str());
      if (status)
      {
        setFoodAmount();
      }
    }
  }
  else
  {
    Serial.print("Failed to read stream, reason: ");
    Serial.println(fbdo3.errorReason());
  }
}

void setFoodAmount()
{
  Serial.println("Getting Food Amount Status");

  int distance1 = measureDistance1();
  int distance2 = measureDistance2();

  int percentage1 = calculatePercentage(distance1, baseDistance);
  int percentage2 = calculatePercentage(distance2, baseDistance);

  Serial.print("Percentage1: ");
  Serial.print(percentage1);
  Serial.println(" %");
  Serial.print("Percentage2: ");
  Serial.print(percentage2);
  Serial.println(" %");

  // Update Firebase with the new percentages for Food1
  if (Firebase.RTDB.setInt(&fbdo3, "/foods/food1/percentage", percentage1))
  {
    Serial.println("Food1 percentage updated");
    // Update Firebase with the new percentages for Food2
    if (Firebase.RTDB.setInt(&fbdo3, "/foods/food2/percentage", percentage2))
    {
      Serial.println("Food2 percentage updated");
      // Make the firebase path false only if both updates are successful
      if (Firebase.RTDB.setBool(&fbdo3, "/trigger/getPetFood/status", false))
      {
        Serial.println("Status of Food Stream set to false");
      }
      else
      {
        Serial.print("Failed to set status to false, reason: ");
        Serial.println(fbdo3.errorReason());
      }
    }
    else
    {
      Serial.print("Failed to update Food2 percentage, reason: ");
      Serial.println(fbdo3.errorReason());
    }
  }
  else
  {
    Serial.print("Failed to update Food1 percentage, reason: ");
    Serial.println(fbdo3.errorReason());
  }
}