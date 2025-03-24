/*
  Thống kê để  chỉnh sửa lại các thông số sau
  +) Tinh chỉnh lại các thông số bias trong kalman filter 
  +) Tìm kiếm thông số run trên xe để chỉnh lại threshold, nhằm mục đích phát hiện xe đang dừng 
  +) Khi xe giảm tốc quá nhanh gia tốc detech không kịp hoặc do hiện tượng nghẽn, nó chỉ về dưới 0 trong một vài giá trị
  sau đó lại tăng lên. 


*/

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

#include <utils.h>


#define MS_TO_KMH 18.0f / 5.0f
#define KMH_TO_MS 5.0f / 18.0f

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ================================================================ CONST

// =============================================================== Object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
// =================================================================== Funciton prototype
void kalman_filter(float a_measured, float dt, float& v, float& b_a, float P[2][2]);
float calculate_std_deviation(float arr[], int n);
// ==================================================================================== SETUP

sensors_event_t a, g, temp;

//================================================ variable for data
double ax = 0.0f, ay = 0.0f, az = 0.0f, gx = 0.0f, gy = 0.0f, gz = 0.0f;
double lat = 0.0f, lng = 0.0f;
//================================================ variable for timer
static unsigned long taskMillis = 0;
static unsigned long getDataMillis = 0;
const long taskInterval = 200;
unsigned long currentMillis = 0;
unsigned long gps_process_millis = 0;

const long getDataInterval = 10;

const float dt = float(taskInterval) / 1000.0;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 30;

adafruit_bno055_opmode_t opmode = OPERATION_MODE_NDOF;
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

void displayVelocity(float V_B, float V_D, float Speed) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x12_tf);
      u8g2.setCursor(10, 20);
      u8g2.print("V_B: ");
      u8g2.print(V_B * MS_TO_KMH);
      u8g2.print(" Km/h");
      
      u8g2.setCursor(10, 35);
      u8g2.print("V_D: ");
      u8g2.print(V_D * MS_TO_KMH);
      u8g2.print(" Km/h");

      u8g2.setCursor(10, 50);
      u8g2.print("Speed: ");
      u8g2.print(Speed * MS_TO_KMH);
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
  
  bno.setMode(opmode);
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
// ==================================================================================== LOOP
// Khai báo biến toàn cục
float speed = 0.0;
float vx = 0.0f, vy = 0.0f;
float prevAx = 0.0f, prevAy = 0.0f; // Lưu giá trị gia tốc trước đó
unsigned long lastTime = 0; // Thời gian lần đo trước đó
unsigned long velocityResetTimer = 0; // Bộ đếm thời gian reset vận tốc
float b_ax = -0.045f, b_ay = 0.0334f; // Bias gia tốc trục x, y
float Px[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // Ma trận hiệp phương sai cho trục x
float Py[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // Ma trận hiệp phương sai cho trục y
float Q[2][2] = {{0.002f, 0.0f}, {0.0f, 0.0002f}}; // Process noise
float R = 0.0f; // Measurement noise

// Ngưỡng để phát hiện trạng thái dừng
const float threshold_accel = 0.3f;  // m/s²
const float threshold_gyro = 5.5f;  // rad/s
const unsigned long stop_detection_time = 2000;  // 1 giây
unsigned long stop_timer = 0;  // Bộ đếm thời gian để xác định trạng thái dừng
bool is_stopped = false;  // Trạng thái xe dừng

// Để phát hiện chuyển động đều
const int speed_window_size = 25;  // 5 giây với delay 200ms (25 mẫu x 200ms = 5s)
float speed_history[speed_window_size];  // Mảng lưu lịch sử vận tốc
int speed_index = 0;  // Chỉ số hiện tại trong mảng
bool speed_history_full = false;  // Cờ kiểm tra mảng đã đầy chưa

void loop() {
  // Lấy dữ liệu từ BNO055
  imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  // Tính thời gian delta (Δt)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // Gia tốc đo được
  float ax_measured = linearAccel.x();
  float ay_measured = linearAccel.y();
  float az_measured = linearAccel.z();

  // Tính norm của gia tốc tuyến tính và tốc độ góc
  float norm_accel = sqrt(ax_measured * ax_measured + ay_measured * ay_measured);
  float gx = gyro.x();
  float gy = gyro.y();
  float gz = gyro.z();
  float norm_gyro = sqrt(gx * gx + gy * gy);

  // Kiểm tra trạng thái dừng
  if (norm_accel < threshold_accel && norm_gyro < threshold_gyro){
    if (stop_timer == 0) {
      stop_timer = currentTime;
    } else if (currentTime - stop_timer >= stop_detection_time) {
      is_stopped = true;
      vx = 0.0f;
      vy = 0.0f;
      Serial.print("Xe dung");
    }
  } else {
    stop_timer = 0;
    is_stopped = false;
  }

  // Cập nhật Kalman nếu xe không dừng
  if (!is_stopped) {
    kalman_filter(ax_measured, dt, vx, b_ax, Px);
    kalman_filter(ay_measured, dt, vy, b_ay, Py);
  }

  // Tính góc heading từ quaternion
  float headingRadians = atan2(2.0f * (quat.x() * quat.w() + quat.y() * quat.z()), 
                               1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
  
  // Tính vận tốc hướng Bắc và Đông
  float v_B = 0.0f, v_D = 0.0f;
  calculateNorthEastVelocity(vx, vy, headingRadians, v_B, v_D);

  // Tính tốc độ tổng hợp
  speed = sqrt(v_B * v_B + v_D * v_D);

  // Lưu trữ vận tốc để kiểm tra chuyển động đều
  speed_history[speed_index] = speed;
  speed_index = (speed_index + 1) % speed_window_size;
  if (speed_index == 0) speed_history_full = true;

  // Kiểm tra chuyển động đều
  if (speed_history_full && !is_stopped) {
    float std_dev = calculate_std_deviation(speed_history, speed_window_size);
    if (std_dev < 0.25f) {
      Serial.print("Xe chuyen dong deu");
    } else {  
      Serial.print("Xe dang chuyen dong");
    }
  }

  // Hiển thị kết quả
  Serial.printf("\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n", v_B, v_D, speed, norm_accel, norm_gyro, ax_measured, ay_measured, gx, gy);
  // Serial.printf("\t%.6f\t%.6f\n", ax_measured, ay_measured);
  
  // // // Reset vận tốc sau 5 giây (giữ nguyên logic cũ)
  // if (currentTime - velocityResetTimer >= 5000) {
  //   vx = 0.0f;
  //   vy = 0.0f;
  //   velocityResetTimer = currentTime;
  //   Serial.println("Velocity reset to 0");
  // }

  // Hiển thị lên màn hình
  displayVelocity(v_B, v_D, speed);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// Hàm bộ lọc Kalman cải tiến cho một trục
void kalman_filter(float a_measured, float dt, float& v, float& b_a, float P[2][2]) {
  // Ma trận chuyển tiếp F
  float F[2][2] = {{1, -dt}, {0, 1}};

  // Dự đoán trạng thái
  float v_pred = v + (a_measured - b_a) * dt;
  float b_a_pred = b_a;

  // Cập nhật ma trận hiệp phương sai dự đoán P_pred = F * P * F^T + Q
  float P_pred[2][2];
  P_pred[0][0] = P[0][0] + dt * (P[0][1] + P[1][0] + dt * P[1][1]) + Q[0][0];
  P_pred[0][1] = P[0][1] + dt * P[1][1];
  P_pred[1][0] = P[1][0] + dt * P[1][1];
  P_pred[1][1] = P[1][1] + Q[1][1];

  // Ma trận quan sát H (giả định a = (v_k - v_{k-1})/dt + b_a)
  float H[2] = {1/dt, 1};

  // Tính innovation
  float a_pred = (v_pred - v) / dt + b_a_pred;
  float innovation = a_measured - a_pred;

  // Tính S = H * P_pred * H^T + R
  float S = H[0] * (H[0] * P_pred[0][0] + H[1] * P_pred[0][1]) +
            H[1] * (H[0] * P_pred[1][0] + H[1] * P_pred[1][1]) + R;

  // Tính Kalman Gain K = P_pred * H^T / S
  float K[2];
  K[0] = (P_pred[0][0] * H[0] + P_pred[0][1] * H[1]) / S;
  K[1] = (P_pred[1][0] * H[0] + P_pred[1][1] * H[1]) / S;

  // Cập nhật trạng thái
  v = v_pred + K[0] * innovation;
  b_a = b_a_pred + K[1] * innovation;

  // Cập nhật ma trận hiệp phương sai P = (I - K*H) * P_pred
  float temp_P00 = P_pred[0][0];
  float temp_P01 = P_pred[0][1];
  float temp_P10 = P_pred[1][0];
  float temp_P11 = P_pred[1][1];
  P[0][0] = (1 - K[0] * H[0]) * temp_P00 - K[0] * H[1] * temp_P10;
  P[0][1] = (1 - K[0] * H[0]) * temp_P01 - K[0] * H[1] * temp_P11;
  P[1][0] = -K[1] * H[0] * temp_P00 + (1 - K[1] * H[1]) * temp_P10;
  P[1][1] = -K[1] * H[0] * temp_P01 + (1 - K[1] * H[1]) * temp_P11;
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

