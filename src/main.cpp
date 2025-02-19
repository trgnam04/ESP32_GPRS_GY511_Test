// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_LSM303_U.h>
// #include <HTTPClient.h>

// #include <SPI.h>
// #include <HttpClient.h>
// #include <WiFi.h>

// #define SDA GPIO_NUM_21
// #define SCL GPIO_NUM_22
// /* Assign a unique ID to this sensor at the same time */
// // Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
// Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(54321);

// WiFiClient wifi;
// HttpClient http(wifi);


// {
//   sensor_t sensor;
//   mag.getSensor(&sensor);
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
//   Serial.println("------------------------------------");
//   Serial.println("");
//   delay(500);
// }

// const char* ssid = "v";
// const char* password = "12345678";
// // Name of the server we want to connect to
// const char kHostname[] = "ems.thebestits.vn";
// const char kPath[] = "/Value/GetData"; // ?stationcode=K13_TEST&data=kaka_nam

// // Number of milliseconds to wait without receiving any data before we give up
// const int kNetworkTimeout = 30 * 1000;
// // Number of milliseconds to wait if no data is available before trying again
// const int kNetworkDelay = 1000;

// void lsm303_init(){
//   Wire.begin(SDA, SCL);
//   Serial.println("Accelerometer Test"); Serial.println("");
//   if(!mag.begin())
//   {
//     /* There was a problem detecting the ADXL345 ... check your connections */
//     Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//     while(1);
//   };
//   displaySensorDetails();
// };

// void wifi_init(){
//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to Wi-Fi");
//   while (WiFi.status() != WL_CONNECTED) {
//     Serial.print(".");
//     delay(500);
//   }
//   Serial.println("Connected!");
// };

// int http_get(const char * hostname, const char * path, char * data, char * retData, int * retDataSize){
//   int err = 0;
//   *retDataSize = 0;
//   // path processing 
//   char newpath[200];
//   sprintf(newpath, "%s?stationcode=K13_TEST&data=%s", path, data);
//   err = http.get(hostname, newpath);
//   if (err == 0) {
//     Serial.println("start request");
//     err = http.responseStatusCode();
//     if (err >= 0) {
//       Serial.print("Status code: ");
//       Serial.println(err);
//       int bodyLen;
//       err = http.skipResponseHeaders();
//       if (err >= 0) {
//         bodyLen = http.contentLength();
//         // *size = bodyLen;
//         Serial.print("Content length is: ");
//         Serial.println(bodyLen);
//         Serial.println();
//         Serial.println("Body returned follows:");

//         unsigned long timeoutStart = millis();

//         while ((http.connected() || http.available()) &&
//                ((millis() - timeoutStart) < kNetworkTimeout)) {
//           if (http.available()) {
//             retData[(*retDataSize)++] = http.read();
//             bodyLen--;
//             timeoutStart = millis();
//           } else {
//             delay(kNetworkDelay);
//           }
//         }
//         Serial.println(); // Add a newline after the body

//       } else {
//         Serial.print("Failed to skip response headers: ");
//         Serial.println(err);
//         return -1;
//       }
//     } else {
//       Serial.print("Getting response failed: ");
//       Serial.println(err);
//       return -1;
//     }
//   } else {
//     Serial.print("Connect failed: ");
//     Serial.println(err);
//     return -1;
//   };
//   retData[*retDataSize] = 0;
//   return 0;
// };

// void setup(void)
// {
//   Serial.begin(9600);
//   // LSM303
//   lsm303_init();
//   // WIFI
//   wifi_init();
// }

// void loop(void)
// {
//   /* Get a new sensor event */
//   // sensors_event_t event;
//   // mag.getEvent(&event);

//   /* Display the results (acceleration is measured in m/s^2) */
// //   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
// //   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
// //   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

//   // Serial.print("__X: "); Serial.print(event.magnetic.x); Serial.print("  ");
//   // Serial.print("__Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
//   // Serial.print("__Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("Gauss ");
//   /* Delay before the next sample */
//   // delay(2000);
//   // ****************************************************************************************************
//   char data[] = "bkitk22";
//   char retData[100];
//   int retDataSize;

//   int err = http_get(kHostname, kPath, data, retData, &retDataSize);
  
//   Serial.println(retData);
//   http.stop();

//   while (1);
// }


#ifdef ESP32
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif // ESP32


#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>
#include <ArduinoJson.h>

constexpr char WIFI_SSID[] = "v";
constexpr char WIFI_PASSWORD[] = "12345678";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
constexpr char TOKEN[] = "0EbSWCgO5wPgnRfAhUwr";

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

// DEFINE CONST
#define NEOPIXEL GPIO_NUM_45
#define SDA GPIO_NUM_11
#define SCL GPIO_NUM_12

constexpr uint16_t THINGSBOARD_PORT = 1883U;

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "example_set_switch";
constexpr const char RPC_TEMPERATURE_KEY[] = "temp";
constexpr const char RPC_SWITCH_KEY[] = "switch";

constexpr const char RPC_NEO_RED_METHOD[] = "set_neo_red";
constexpr const char RPC_NEO_GREEN_METHOD[] = "set_neo_green";
constexpr const char RPC_NEO_BLUE_METHOD[] = "set_neo_blue";

constexpr const char RPC_NEO_RED_KEY[] = "data";
constexpr const char RPC_NEO_GREEN_KEY[] = "data";
constexpr const char RPC_NEO_BLUE_KEY[] = "data";

constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 10U;
constexpr uint8_t MAX_RPC_RESPONSE = 10U;

#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize used apis
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// Statuses for subscribing to rpc
bool subscribed = false;

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

/// @brief Processes function for RPC call "example_json"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processGetJson(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the json RPC method");

  // Size of the response document needs to be configured to the size of the innerDoc + 1.
  StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
  innerDoc["string"] = "exampleResponseString";
  innerDoc["int"] = 5;
  innerDoc["float"] = 5.0f;
  innerDoc["bool"] = true;
  response["json_data"] = innerDoc;
}

void processTemperatureChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set temperature RPC method");

  // Process data
  const float example_temperature = data[RPC_TEMPERATURE_KEY];

  Serial.print("Example temperature: ");
  Serial.println(example_temperature);

  // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC template parameter accordingly to ensure that the value can be deserialized.RPC_Callback.
  // See https://arduinojson.org/v6/api/jsondocument/add/ for more information on which variables cause a copy to be created
  response["string"] = "exampleResponseString";
  response["int"] = 5;
  response["float"] = 5.0f;
  response["double"] = 10.0;
  response["bool"] = true;
}

void processSwitchChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set switch method");

  // Process data
  const int switch_state = data[RPC_TEMPERATURE_KEY];
  Serial.print("Example switch state: ");
  Serial.println(switch_state);
  response.set(22.02);
}

void processNeoDefault(const JsonVariantConst &data, JsonDocument &response){
  response.set(0);
}

void setup() {
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
}

void loop() {
  delay(1000);
  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  if (!subscribed) {
    Serial.println("Subscribing for RPC...");
    const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
      // Requires additional memory in the JsonDocument for the JsonDocument that will be copied into the response
      RPC_Callback{ RPC_JSON_METHOD,           processGetJson },
      // Requires additional memory in the JsonDocument for 5 key-value pairs that do not copy their value into the JsonDocument itself
      RPC_Callback{ RPC_TEMPERATURE_METHOD,    processTemperatureChange },
       // Internal size can be 0, because if we use the JsonDocument as a JsonVariant and then set the value we do not require additional memory
      RPC_Callback{ RPC_SWITCH_METHOD,         processSwitchChange }
    };
    if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }
  float lng = 50.12345;
  float lat = 20.12345;
  float magX = 0.123;
  float magY = 0.234;
  float magZ = 0.345;
  float accX = 0.000;
  float accY = 0.000;
  float accZ = 9.801;
  DynamicJsonDocument doc(1024);
  doc[String("lng")] = lng;
  
  tb.sendTelemetryJson(doc, 1024);
  tb.loop();
}