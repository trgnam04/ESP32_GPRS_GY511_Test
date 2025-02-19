#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <HTTPClient.h>

#include <SPI.h>
#include <HttpClient.h>
#include <WiFi.h>

#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22
/* Assign a unique ID to this sensor at the same time */
// Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(54321);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

WiFiClient wifi;
HttpClient http(wifi);
const char* ssid = "v";
const char* password = "12345678";
// Name of the server we want to connect to
const char kHostname[] = "ems.thebestits.vn";
const char kPath[] = "/Value/GetData"; // ?stationcode=K13_TEST&data=kaka_nam

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30 * 1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

void lsm303_init(){
  Wire.begin(SDA, SCL);
  Serial.println("Accelerometer Test"); Serial.println("");
  if(!mag.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  };
  displaySensorDetails();
};

void wifi_init(){
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected!");
};

int http_get(const char * hostname, const char * path, char * data, char * retData, int * retDataSize){
  int err = 0;
  *retDataSize = 0;
  // path processing 
  char newpath[200];
  sprintf(newpath, "%s?stationcode=K13_TEST&data=%s", path, data);
  err = http.get(hostname, newpath);
  if (err == 0) {
    Serial.println("start request");
    err = http.responseStatusCode();
    if (err >= 0) {
      Serial.print("Status code: ");
      Serial.println(err);
      int bodyLen;
      err = http.skipResponseHeaders();
      if (err >= 0) {
        bodyLen = http.contentLength();
        // *size = bodyLen;
        Serial.print("Content length is: ");
        Serial.println(bodyLen);
        Serial.println();
        Serial.println("Body returned follows:");

        unsigned long timeoutStart = millis();

        while ((http.connected() || http.available()) &&
               ((millis() - timeoutStart) < kNetworkTimeout)) {
          if (http.available()) {
            retData[(*retDataSize)++] = http.read();
            bodyLen--;
            timeoutStart = millis();
          } else {
            delay(kNetworkDelay);
          }
        }
        Serial.println(); // Add a newline after the body

      } else {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
        return -1;
      }
    } else {
      Serial.print("Getting response failed: ");
      Serial.println(err);
      return -1;
    }
  } else {
    Serial.print("Connect failed: ");
    Serial.println(err);
    return -1;
  };
  retData[*retDataSize] = 0;
  return 0;
};

void setup(void)
{
  Serial.begin(9600);
  // LSM303
  lsm303_init();
  // WIFI
  wifi_init();
}

void loop(void)
{
  /* Get a new sensor event */
  // sensors_event_t event;
  // mag.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
//   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  // Serial.print("__X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  // Serial.print("__Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  // Serial.print("__Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("Gauss ");
  /* Delay before the next sample */
  // delay(2000);
  // ****************************************************************************************************
  char data[] = "bkitk22";
  char retData[100];
  int retDataSize;

  int err = http_get(kHostname, kPath, data, retData, &retDataSize);
  
  Serial.println(retData);
  http.stop();

  while (1);
}