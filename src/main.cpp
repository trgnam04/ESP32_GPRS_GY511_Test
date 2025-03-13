#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}
sensors_event_t a, g, temp;
float ax, ay, az, gx, gy, gz;

#define MS_TO_KMH 18.0f/5.0f
#define KMH_TO_MS 5.0f/18.0f

static unsigned long taskMillis = 0;
static unsigned long getDataMillis = 0;  unsigned int count = 0;
const long taskInterval = 100;
unsigned long currentMillis = 0;

const long getDataInterval = 5;

const float dt = float(taskInterval)/1000.0;
const float alpha = 0.98;
const float GRAVITY = 9.81;
float prev_ax, prev_ay, prev_az;

float roll, pitch; // after complementary with ax, ay, az
float alpha_complementary = 0.98;

float ax_filtered = 0.0;
float ay_filtered = 0.0;
float az_filtered = 0.0;

float vx = 0.0;
float vy = 0.0;
float vz = 0.0;

float l_o = 0.0f;
float l_f = 0.0f;

float velocity_stop = 10.0f;

float ax_temp = 0.0f;
float ay_temp = 0.0f;
float az_temp = 0.0f;
float gx_temp = 0.0f;
float gy_temp = 0.0f;
float gz_temp = 0.0f;
// ax       ay      az      gx      gy      gz
// 0.79    -0.19   9.33    -0.01   0.02    0.00
void loop() {
  // Get data
  mpu.getEvent(&a, &g, &temp);
  currentMillis = millis();

  ax_temp = a.acceleration.x + 0.3;
  ay_temp = a.acceleration.y;
  az_temp = a.acceleration.z - 0.1;
  gx_temp = g.gyro.x;
  gy_temp = g.gyro.y;
  gz_temp = g.gyro.z;

  if(currentMillis - getDataMillis >= getDataInterval){
    getDataMillis = currentMillis;

    ax += ax_temp;
    ay += ay_temp;
    az += az_temp;
    gx += gx_temp;
    gy += gy_temp;
    gz += gz_temp;
    count += 1;
  }

  if (currentMillis - taskMillis >= taskInterval) {
    taskMillis = currentMillis;
    // average from 100 measurement
    ax = ax / (float)count;
    ay = ay / (float)count;
    az = az / (float)count;
    gx = gx / (float)count;
    gy = gy / (float)count;
    gz = gz / (float)count;
    count = 1;

    // 3) Estimate orientation
  // (a) Get pitch/roll from accel (in radians)
  float accelRoll  = atan2(ay, sqrt(ax*ax + az*az));
  float accelPitch = atan2(-ax, sqrt(ay*ay + az*az));

  // (b) Integrate gyro angles 
  roll  += gx * dt;
  pitch += gy * dt;
  // yaw   += gz * dt;  // won't be stable w/o magnetometer

  // (c) Simple complementary filter
  float alpha = 0.9f;
  roll  = alpha * roll  + (1 - alpha) * accelRoll; 
  pitch = alpha * pitch + (1 - alpha) * accelPitch;

  // Rotate gravity (0, 0, +GRAVITY) in sensor frame
  float gx_comp = sin(pitch) * GRAVITY * -1.0f;                          // X comp
  float gy_comp = -cos(pitch) * sin(roll) * GRAVITY * -1.0f;          // Y comp
  float gz_comp = -cos(pitch) * cos(roll) * GRAVITY * -1.0f;          // Z comp

  float ax_linear = ax - gx_comp;
  float ay_linear = ay - gy_comp;
  float az_linear = az - gz_comp;

  // 5) Filter the linear acceleration
  float beta = 0.9f; // tune
  ax_filtered = beta * ax_filtered + (1.0f - beta) * ax_linear;
  ay_filtered = beta * ay_filtered + (1.0f - beta) * ay_linear;
  az_filtered = beta * az_filtered + (1.0f - beta) * az_linear;

  // 6) Integrate to get velocity
  vx += ax_filtered * dt;
  vy += ay_filtered * dt;
  // vz += az_filtered * dt;

  // zero velocity or zero acceleration
  // if (fabs(ax_filtered) < 0.2 && fabs(ay_filtered) < 0.2) {
  //   vx = 0.0f;
  //   vy = 0.0f;
  // }
  
        // Plot
    Serial.print(ax); Serial.print("\t"); Serial.print(ay); Serial.print("\t"); Serial.print(az); Serial.print("\t");
    // Serial.print(gx); Serial.print("\t"); Serial.print(gy); Serial.print("\t"); Serial.println(gz);
    // Serial.print(ax_linear); Serial.print("\t"); Serial.print(ay_linear); Serial.print("\t"); Serial.println(az_linear); Serial.print("\t");
    // Serial.print(vx); Serial.print("\t"); Serial.print(vy); Serial.print("\t"); Serial.println(vz);
    // Serial.print(offset_ax); Serial.print("\t"); Serial.print(offset_ay); Serial.print("\t"); Serial.print(offset_az); Serial.print("\t");
    // Serial.print(offset_gx); Serial.print("\t"); Serial.print(offset_gy); Serial.print("\t"); Serial.println(offset_gz);
    Serial.print(ax_filtered); Serial.print("\t"); Serial.print(ay_filtered); Serial.print("\t"); Serial.println(az_filtered);
  };
}