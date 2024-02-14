#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "MPU9250.h"
#if defined(ESP32)
#include <WiFi.h>
#endif
float gps_latitude = 0;
float gps_longitude = 0;
float gps_speed = 0;
float gps_altitude = 0;
unsigned long getDataPrevMillis = 0;
static const int RXPin = 4, TXPin = 0;
static const uint32_t GPSBaud = 9600;
//String lat_str, lng_str;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Galaxy M21A1AA"
#define WIFI_PASSWORD "uchd4839"
#define SERVER_URL "http://192.168.219.184:3000/alert/"

MPU9250 mpu;

float YAW_new;
float PITCH_new;
float ROLL_new;

float YAW_previous;
float PITCH_previous;
float ROLL_previous;

float YAW_difference;
float PITCH_difference;
float ROLL_difference;

float ALA;
float ACCX;
float ACCY;
float ACCZ;

float max_difference = 10.0;
float ALA_THRESH = 2;
float PITCH_THRESH = 45;
float ROLL_THRESH = 45;

int tracking = 1;
int accident_happened = 0;
int accident_happened0 = 0;
int accident_detected = 0;

unsigned long startTime;
unsigned long endTime;
const int communication_pin1 = 23;
const int communication_pin2 = 15;

void mpu_core( void *pvParameters );
void gps_core( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Wire.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();



  pinMode(communication_pin1, OUTPUT);
  digitalWrite(communication_pin1, LOW);
//  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // calibrate anytime you want to

  mpu.verbose(true);
//  delay(5000);
  mpu.calibrateAccelGyro();
//  delay(5000);
  mpu.calibrateMag();
  mpu.verbose(false);

  pinMode(communication_pin2, INPUT);

  gpsSerial.begin(GPSBaud);
  Serial.println(TinyGPSPlus::libraryVersion());

  xTaskCreatePinnedToCore(
    mpu_core
    ,  "mpu_core"   // A name just for humans
    ,  20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  //
    ,  NULL
    ,  0);

  xTaskCreatePinnedToCore(
    gps_core
    ,  "gps_core"
    ,  85000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  1);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void mpu_core(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  Serial.print("Setup: mpu9250Executing on core ");
  Serial.println(xPortGetCoreID());
  int count = 0;
  

  // initialize digital LED_BUILTIN on pin 13 as an output.


  for (;;) // A Task shall never return or exit.
  {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {

        YAW_new = mpu.getYaw();
        PITCH_new = mpu.getPitch();
        ROLL_new = mpu.getRoll();
        YAW_difference = abs(YAW_new - YAW_previous);
        PITCH_difference = abs(PITCH_new - PITCH_previous);
        ROLL_difference = abs(ROLL_new - ROLL_previous);
        ACCX = mpu.getAccX();
        ACCY = mpu.getAccY();
        ACCZ = mpu.getAccZ();
//        Serial.println("MPU9250 Data:");
//        Serial.print("Yaw: "); Serial.println(YAW_new);
//        Serial.print("Pitch: "); Serial.println(PITCH_new);
//        Serial.print("Roll: "); Serial.println(ROLL_new);


        ALA = sqrt(sq(ACCX) + sq(ACCY) + sq(ACCZ));
        Serial.println(ALA);
        //Serial.println(ROTATION_FILTERED);
        if (ALA > ALA_THRESH || PITCH_difference > max_difference || ROLL_difference > max_difference || abs(PITCH_new) > PITCH_THRESH || abs(ROLL_new) > ROLL_THRESH || ALA < 0.1 || YAW_difference > 45 ) {
          if (count != 0) {
            digitalWrite(communication_pin1, HIGH);
            accident_happened0 = 1;
            startTime =  millis();
            Serial.println("ACCIDENT");
            if ((millis() - getDataPrevMillis > 1500000 || getDataPrevMillis == 0)) {
              getDataPrevMillis = millis();
              HTTPClient http;
              http.begin(SERVER_URL);
              http.addHeader("Content-Type", "application/json");
             
              String dataToSend = "{\"lat\":" + String(gps.location.lat(), 7) +",\"long\":" + String(gps.location.lng(), 7) +",\"mobile\":\"1234567890\"}";
        
              int httpCode = http.POST(dataToSend);
        
              if (httpCode > 0) {
                String response = http.getString();
                Serial.println("HTTP Response: " + response);
              } else {
                Serial.println("HTTP POST failed with error: " + http.errorToString(httpCode));
              }
          
              http.end();
            }
          }
          else {
            count = 1;
          }
        }

        else {

          if (accident_happened0 == 1) {
            endTime = millis();

            if (endTime - startTime > 20000) {
              digitalWrite(communication_pin1, LOW);
              accident_happened0 = 0;
              delay(10);
            }

          }

        }
        YAW_previous = YAW_new;
        PITCH_previous = PITCH_new;
        ROLL_previous = ROLL_new;

        prev_ms = millis();
      }
    }// one tick delay (15ms) in between reads for stability
  }
}

void gps_core(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  Serial.print("Setup: gps and firebase Executing on core ");
  Serial.println(xPortGetCoreID());


  

  for (;;)
  { smartdelay_gps(1000);
  
      if (gps.location.isValid()) {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat());

        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng());

        Serial.print(F("- altitude: "));
        if (gps.altitude.isValid())
          Serial.println(gps.altitude.meters());
        else
          Serial.println(F("INVALID"));
      } else {
        Serial.println(F("- location: INVALID"));
      }

      Serial.print(F("- speed: "));
      if (gps.speed.isValid()) {
        Serial.print(gps.speed.kmph());
        Serial.println(F(" km/h"));
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.println();
    
  

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));


   
    if (digitalRead(communication_pin2) == LOW) {
      accident_detected = 0;

      delay(500);
    }


  }
}



static void smartdelay_gps(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}
