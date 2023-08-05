
//ESP32
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

//Firebase
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

//Sensors
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <SPI.h>
//#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Insert your network credentials
#define WIFI_SSID "Samsung_S21"
#define WIFI_PASSWORD "test1111"

// Insert Firebase project API Key
#define API_KEY "AIzaSyB4xNMsU5d-OoBFP86ota6FBSM_t6Jlaw0"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://taylor-hackathon-default-rtdb.firebaseio.com/"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

Adafruit_BMP085 bmp;
// Set the LCD address to 0x27 for a 16 chars and 2 line display
//LiquidCrystal_I2C lcd(0x27, 16, 2);

MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

const byte RATE_SIZE = 4;       //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE] = { 0 };  //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

int noti;

void setup() {
  Serial.begin(115200);
  sensorInitialization();
  connectToWiFi();
  configFB();
}

void loop() {

  unsigned long currentTime = millis();
  float temp = bmp.readTemperature();
  float alt = bmp.readAltitude();


  //if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 500 || sendDataPrevMillis == 0)) {

  for (int i = 0; i < 100; i++) {
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true) {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    if (irValue < 50000) {
      Serial.println(" No finger?");
      beatAvg = 0;
      beatsPerMinute = 0;
      break;
    }
  }


  // Upload data to Firebase
  if (Firebase.ready() && signupOK && (currentTime - sendDataPrevMillis > 500 || sendDataPrevMillis == 0)) {
    uploadData(temp, alt, beatsPerMinute, beatAvg);
    displayData(temp, alt, beatsPerMinute, beatAvg);
    //getNotification();
    sendDataPrevMillis = currentTime;
  }



  //delay(500);
}

// Connect to WiFi
void connectToWiFi() {

  display.clearDisplay();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Wifi connecting: ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("WiFi\nConnection\nSuccess");
  display.display();
}

//Setup for FireBase
void configFB() {

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("FB Connection okay");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Sensor initialization
void sensorInitialization() {

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();

  // BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  // initialize the LCD
  //lcd.begin();

  // Turn on the blacklight and print a message.
  // lcd.backlight();
  // lcd.print("Starting");
  // delay(1000);

  // Heart Sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the Heart rate sensor with steady pressure.");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Place your index\nfinger on the Heart\nrate sensor\nwith steady pressure.");
  display.display();



  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED
  delay(2000);
}

void uploadData(float temperature, float altitude, float heartRate, int AvgHeartRate) {

  // Upload data to Firebase
  if (Firebase.RTDB.setInt(&fbdo, "sensor/temperature", temperature)) {
    Serial.print("TEMPERATURE: ");
    Serial.println(temperature);
  } else {
    Serial.println("TEMPERATURE Failed to upload data to Firebase!");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setInt(&fbdo, "sensor/altitude", altitude)) {
    Serial.print("ALTITUDE: ");
    Serial.println(altitude);
  } else {
    Serial.println("ALTITUDE Failed to upload data to Firebase!");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setInt(&fbdo, "sensor/heart_rate", heartRate)) {
    Serial.print("Heart Rate: ");
    Serial.println(heartRate);
  } else {
    Serial.println("TEMPERATURE Failed to upload data to Firebase!");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setInt(&fbdo, "sensor/avg_heart_rate", AvgHeartRate)) {
    Serial.print("AvgHeartRate: ");
    Serial.println(AvgHeartRate);
  } else {
    Serial.println("AvgHeartRate Failed to upload data to Firebase!");
    Serial.println(fbdo.errorReason());
  }

  Serial.println();
}

void displayData(float temperature, float altitude, float heartRate, int AvgHeartRate) {
  // lcd.clear();
  // lcd.print("Temp: ");
  // lcd.print(temperature);
  // lcd.print(" C");

  // lcd.setCursor(0, 1);
  // lcd.print("Alt: ");
  // lcd.print(altitude);
  // lcd.print(" m");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,15);
  display.print("Avg BPM: ");
  display.println(AvgHeartRate);
  display.setCursor(0,30);
  display.print("Temperature: ");
  display.print(temperature);
  display.print(" *C");
  display.setCursor(0,45);
  display.print("Altitude: ");
  display.print(altitude);
  display.println(" m");
  display.display();

}

// void getNotification() {
//   if (Firebase.RTDB.getInt(&fbdo, "/notification/medication")) {
//     if (fbdo.dataType() == "int") {
//       noti = fbdo.intData();
//       Serial.print("Notification status: ");
//       Serial.println(noti);
//     }
//   } else {
//     Serial.println(fbdo.errorReason());
//   }
// }