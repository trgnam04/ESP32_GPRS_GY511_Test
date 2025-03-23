#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <U8g2lib.h>


#define MS_TO_KMH 18.0f / 5.0f
#define KMH_TO_MS 5.0f / 18.0f

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
QueueHandle_t gpsQueue;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation *, 1U> apis = {
    &rpc};
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// =================================================================== Funciton prototype
void InitWiFi();
bool reconnect();
void sendTelemetryData();
void printEvent(sensors_event_t *event);
float calculate_std_deviation(float arr[], int n);
// ==================================================================================== SETUP

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

uint16_t BNO055_SAMPLERATE_DELAY_MS = 20;
//================================================ variable for calculate
unsigned int count_for_mean = 0;
float roll, pitch; // after complementary with ax, ay, az

float ax_filtered = 0.0, ay_filtered = 0.0, az_filtered = 0.0;
float ax_linear = 0.0f, ay_linear = 0.0f;

float ax_temp = 0.0f, ay_temp = 0.0f, az_temp = 0.0f;
float gx_temp = 0.0f, gy_temp = 0.0f, gz_temp = 0.0f;


//================================================ Display support function
void displayMessage(const char *message) {
  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(10, 30);
      u8g2.print(message);
  } while (u8g2.nextPage());
}

void displayCalibrationStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  
  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.setCursor(25, 15);
      u8g2.print("Calibrating...");
      u8g2.setCursor(10, 30);
      u8g2.print("Sys: "); u8g2.print(sys);
      u8g2.setCursor(10, 40);
      u8g2.print("Gyro: "); u8g2.print(gyro); 
      u8g2.setCursor(10, 50);
      u8g2.print("Accel: "); u8g2.print(accel);
      u8g2.setCursor(10, 60);
      u8g2.print("Mag: "); u8g2.print(mag);
  } while (u8g2.nextPage());
}

void displayVelocity(float Vx, float Vy) {
  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(10, 20);
      u8g2.print("Vx: ");
      u8g2.print(Vx);
      u8g2.print(" Km/h");
      
      u8g2.setCursor(10, 40);
      u8g2.print("Vy: ");
      u8g2.print(Vy);
      u8g2.print(" Km/h");
  } while (u8g2.nextPage());
}

void setup() {
  u8g2.begin();
  u8g2.enableUTF8Print();
  Serial.begin(9600);
  
  if (!bno.begin()) {
      Serial.println("Không tìm thấy BNO055, kiểm tra kết nối!");
      while (1);
  }
  
  
  // Hiển thị trạng thái hiệu chỉnh
  while (true) {
      uint8_t sys, gyro, accel, mag;
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      displayCalibrationStatus();
      
      if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
          break;
      }
      delay(500);
  }
  
  displayMessage("Calibration Complete!");
  delay(2000);
}


// TODO
// ax_filter, ay_filter, lat, lon, deltaT, real_velocity
// ==================================================================================== LOOP
// Khai báo biến toàn cục
// Khai báo biến toàn cục
float vx = 0.0f, vy = 0.0f;
float prevAx = 0.0f, prevAy = 0.0f; // Lưu giá trị gia tốc trước đó
float arrDataX[10], arrDataY[10];
int arr_countX = 0, arr_countY = 0;
int stop_count = 0, moving_count = 0;
unsigned long lastTime = 0; // Thời gian lần đo trước đó

void loop() {
  // Lấy dữ liệu gia tốc
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Tính thời gian delta (Δt) theo giây
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Chuyển ms thành giây
  lastTime = currentTime;

  // Lưu giá trị gia tốc hiện tại
  float ax = linearAccelData.acceleration.x;
  float ay = linearAccelData.acceleration.y;

  arrDataX[arr_countX++] = ax;
  arrDataY[arr_countY++] = ay;

  // Tính gia tốc trung điểm (a_mid)
  float a_mid_x = (ax + prevAx) / 2.0f;
  float a_mid_y = (ay + prevAy) / 2.0f;

  // Tính vận tốc theo Midpoint Riemann Sum
  vx += a_mid_x * dt;
  vy += a_mid_y * dt;

  // Cập nhật giá trị gia tốc trước đó
  prevAx = ax;
  prevAy = ay;

  // Hiển thị kết quả vận tốc
  Serial.print("vx: "); Serial.print(vx);
  Serial.print("\tvy: "); Serial.println(vy);

  // Kiểm tra khi mảng đủ dữ liệu
  if (arr_countX == 10 && arr_countY == 10) {
    float stdX = calculate_std_deviation(arrDataX, 10);
    float stdY = calculate_std_deviation(arrDataY, 10);

    // Kiểm tra trạng thái dừng hay di chuyển
    if (stdX < 0.25 && stdY < 0.21) stop_count++;
    else moving_count++;

    // Reset vận tốc khi phát hiện dừng
    if (stop_count == 3) {
      Serial.println("STOP");
      vx = 0.0f; vy = 0.0f; // Reset vận tốc
      stop_count = 0;
    }
    if (moving_count == 3) {
      Serial.println("MOVING");
      moving_count = 0;
    }

    // Reset bộ đếm mảng
    arr_countX = 0;
    arr_countY = 0;
  }

  // Hiển thị vận tốc lên màn hình
  displayVelocity(vx, vy);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}


float calculate_std_deviation(float arr[], int n)
{
  float sum = 0.0f;
  for (int i = 0; i < n; i++)
  {
    sum += arr[i];
  }
  float mean = sum / n;
  float sum_deviation = 0.0f;
  for (int i = 0; i < n; i++)
  {
    sum_deviation += (arr[i] - mean) * (arr[i] - mean);
  }
  return sqrt(sum_deviation / n);
}

void sendTelemetryData()
{
  data[COLLECTOR_KEY_LNG] = lng;
  data[COLLECTOR_KEY_LAT] = lat;
  data[COLLECTOR_KEY_ACCEL_X] = ax_linear;
  data[COLLECTOR_KEY_ACCEL_Y] = ay_linear;
  data[COLLECTOR_KEY_DELTA_T] = dt;
  size_t data_size = Helper::Measure_Json(data);
  tb.sendTelemetryJson(data, data_size);
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
};

bool reconnect()
{
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    return true;
  }
  InitWiFi();
  return true;
};

void printEvent(sensors_event_t *event)
{
  double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    // Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE)
  {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    // Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY)
  {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else
  {
    Serial.print("Unk:");
  }

  // Serial.print("\tx= ");
  // Serial.print(x);
  // Serial.print(" |\ty= ");
  // Serial.print(y);
  // Serial.print(" |\tz= ");
  // Serial.println(z);
  Serial.print(x, 6);
  Serial.print("\t");
  Serial.print(y, 6);
  Serial.print("\t");
  Serial.println(z, 6);
}
