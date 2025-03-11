#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>

#define MS_TO_KMH 18.0f / 5.0f
#define KMH_TO_MS 5.0f / 18.0f

// ================================================================ CONST
const int RXPin = 17, TXPin = 18;
const uint32_t GPSBaud = 9600;
const float GRAVITY = 9.81;

constexpr char WIFI_SSID[] = "271104E";
constexpr char WIFI_PASSWORD[] = "1234567890";

constexpr char TOKEN[] = "COLLECTOR";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 10U;
constexpr uint8_t MAX_RPC_RESPONSE = 10U;

constexpr char COLLECTOR_KEY_LAT[] = "lat";
constexpr char COLLECTOR_KEY_LNG[] = "lng";
constexpr char COLLECTOR_KEY_ACCEL_X[] = "accX";
constexpr char COLLECTOR_KEY_ACCEL_Y[] = "accY";
constexpr char COLLECTOR_KEY_ACCEL_Z[] = "accZ";
constexpr char COLLECTOR_KEY_MAG_X[] = "magX";
constexpr char COLLECTOR_KEY_MAG_Y[] = "magY";
constexpr char COLLECTOR_KEY_MAG_Z[] = "magZ";
constexpr char COLLECTOR_KEY_GYRO_X[] = "gyroX";
constexpr char COLLECTOR_KEY_GYRO_Y[] = "gyroY";
constexpr char COLLECTOR_KEY_GYRO_Z[] = "gyroZ";
constexpr char COLLECTOR_KEY_DELTA_T[] = "deltaT";
// =============================================================== Object
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
QueueHandle_t gpsQueue;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);



// =================================================================== Funciton prototype
void InitWiFi();
bool reconnect();
void sendTelemetryData();
// ==================================================================================== SETUP
void setup(void)
{
  Serial.begin(9600);
  InitWiFi();
  ss.begin(GPSBaud);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}
sensors_event_t a, g, temp;

//================================================ variable for data
double ax = 0.0f, ay = 0.0f, az = 0.0f, gx = 0.0f, gy = 0.0f, gz = 0.0f;
double lat = 0.0f, lng = 0.0f;

DynamicJsonDocument data(256);
//================================================ variable for timer
static unsigned long taskMillis = 0;
static unsigned long getDataMillis = 0;
const long taskInterval = 200;
unsigned long currentMillis = 0;
unsigned long gps_process_millis = 0;

const long getDataInterval = 10;

const float dt = float(taskInterval) / 1000.0;

//================================================ variable for calculate
unsigned int count_for_mean = 0;
float roll, pitch; // after complementary with ax, ay, az

float ax_filtered = 0.0, ay_filtered = 0.0, az_filtered = 0.0;
float ax_linear = 0.0f, ay_linear=0.0f;

float ax_temp = 0.0f, ay_temp = 0.0f, az_temp = 0.0f;
float gx_temp = 0.0f, gy_temp = 0.0f, gz_temp = 0.0f;


// TODO
// ax_filter, ay_filter, lat, lon, deltaT, real_velocity
// ==================================================================================== LOOP
void loop()
{
  // Get data
  mpu.getEvent(&a, &g, &temp);
  currentMillis = millis();

  ax_temp = a.acceleration.x; ay_temp = a.acceleration.y; az_temp = a.acceleration.z + 0.1;
  gx_temp = g.gyro.x; gy_temp = g.gyro.y; gz_temp = g.gyro.z;

  if (currentMillis - getDataMillis >= getDataInterval)
  { // ============================================ MEAN
    getDataMillis = currentMillis;

    ax += ax_temp;
    ay += ay_temp;
    az += az_temp;
    gx += gx_temp;
    gy += gy_temp;
    gz += gz_temp;
    count_for_mean += 1;
  }

  if(currentMillis - gps_process_millis >= 10){ // ==================================================== ENCODE GPS
    gps_process_millis = currentMillis;
    while (ss.available() > 0)
    if (gps.encode(ss.read()))
    {
      if (gps.location.isValid())
      {
        lat = gps.location.lat();
        lng = gps.location.lng();
      }
  };
  }

  if (currentMillis - taskMillis >= taskInterval)
  { // ============================================= FILTER Gravity
    taskMillis = currentMillis;
    // average from 100 measurement
    ax = ax / (float)count_for_mean;
    ay = ay / (float)count_for_mean;
    az = az / (float)count_for_mean;
    gx = gx / (float)count_for_mean;
    gy = gy / (float)count_for_mean;
    gz = gz / (float)count_for_mean;
    count_for_mean = 1;

    float accelRoll = atan2(ay, sqrt(ax * ax + az * az));
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az));

    // (b) Integrate gyro angles
    roll += gx * dt;
    pitch += gy * dt;

    // (c) Simple complementary filter
    float alpha = 0.5f;
    roll = alpha * roll + (1 - alpha) * accelRoll;
    pitch = alpha * pitch + (1 - alpha) * accelPitch;

    // Rotate gravity (0, 0, +GRAVITY) in sensor frame
    float gx_comp = sin(pitch) * GRAVITY * -1.0f;              // X comp
    float gy_comp = -cos(pitch) * sin(roll) * GRAVITY * -1.0f; // Y comp
    float gz_comp = -cos(pitch) * cos(roll) * GRAVITY * -1.0f; // Z comp

    ax_linear = ax - gx_comp;
    ay_linear = ay - gy_comp;
    float az_linear = az - gz_comp;

    // 5) Filter the linear acceleration
    float beta = 0.9f; // tune
    ax_filtered = beta * ax_filtered + (1.0f - beta) * ax_linear;
    ay_filtered = beta * ay_filtered + (1.0f - beta) * ay_linear;
    az_filtered = beta * az_filtered + (1.0f - beta) * az_linear;


    if (gps.location.isUpdated()) // Nếu có dữ liệu mới
    {
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", Longitude: ");
      Serial.println(gps.location.lng(), 6);
    }
    // Plot
    Serial.print(ax);Serial.print("\t");Serial.print(ay);Serial.print("\t");Serial.print(az);Serial.print("\t");
    // Serial.print(gx); Serial.print("\t"); Serial.print(gy); Serial.print("\t"); Serial.println(gz);
    // Serial.print(ax_linear); Serial.print("\t"); Serial.print(ay_linear); Serial.print("\t"); Serial.println(az_linear); Serial.print("\t");
    // Serial.print(vx); Serial.print("\t"); Serial.print(vy); Serial.print("\t"); Serial.println(vz);
    // Serial.print(offset_ax); Serial.print("\t"); Serial.print(offset_ay); Serial.print("\t"); Serial.print(offset_az); Serial.print("\t");
    // Serial.print(offset_gx); Serial.print("\t"); Serial.print(offset_gy); Serial.print("\t"); Serial.println(offset_gz);
    Serial.print(ax_linear); Serial.print("\t"); Serial.print(ay_linear); Serial.print("\t");
    Serial.print(lat, 6); Serial.print("\t"); Serial.println(lng, 6);
  };

  // publish to server
  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  };
  sendTelemetryData();
  tb.loop();
};

void sendTelemetryData(){
  data[COLLECTOR_KEY_LNG] = lng;
  data[COLLECTOR_KEY_LAT] = lat;
  data[COLLECTOR_KEY_ACCEL_X] = ax_linear;
  data[COLLECTOR_KEY_ACCEL_Y] = ay_linear;
  data[COLLECTOR_KEY_DELTA_T] = dt;
  size_t data_size = Helper::Measure_Json(data);
  tb.sendTelemetryJson(data, data_size);
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
};

bool reconnect() {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  InitWiFi();
  return true;
};


  

  