/*
 * ESP32 Kalman Filter for Position Fusion – Đọc dữ liệu JSON từ Serial
 * Phiên bản tối ưu cho môi trường vi điều khiển với bộ nhớ hạn chế.
 *
 * Yêu cầu: ArduinoJson library (version 6) được cài đặt trong Arduino IDE.
 */

 #include <Arduino.h>
 #include <math.h>
 #include <ArduinoJson.h>
 
 // -------------------------
 // Các hằng số & Kiểu dữ liệu
 // -------------------------
 using Degrees = double;
 using Radians = double;
 
 const double EARTH_RADIUS = 6371 * 1000.0; // meters
 const double ACTUAL_GRAVITY = 9.80665;
 
 // -------------------------
 // Cấu trúc GeoPoint
 // -------------------------
 struct GeoPoint {
   double Latitude;
   double Longitude;
 };
 
 // -------------------------
 // Các hàm chuyển đổi góc
 // -------------------------
 Radians DegreesToRadians(Degrees deg) {
   return deg * M_PI / 180.0;
 }
 
 Degrees RadiansToDegrees(Radians rad) {
   return rad * 180.0 / M_PI;
 }
 
 double geoAngle(double angleInDegrees) {
   return DegreesToRadians(angleInDegrees);
 }
 
 // -------------------------
 // Các hàm hình học địa lý
 // -------------------------
 GeoPoint GetPointAhead(GeoPoint fromCoordinate, double distanceMeters, Degrees azimuth) {
   double radiusFraction = distanceMeters / EARTH_RADIUS;
   double bearing = DegreesToRadians(azimuth);
 
   double lat1 = geoAngle(fromCoordinate.Latitude);
   double lng1 = geoAngle(fromCoordinate.Longitude);
 
   double lat2 = asin(sin(lat1) * cos(radiusFraction) +
                      cos(lat1) * sin(radiusFraction) * cos(bearing));
 
   double lng2 = lng1 + atan2(sin(bearing) * sin(radiusFraction) * cos(lat1),
                              cos(radiusFraction) - sin(lat1) * sin(lat2));
   lng2 = fmod((lng2 + 3 * M_PI), (2 * M_PI)) - M_PI;
 
   GeoPoint result;
   result.Latitude = RadiansToDegrees(lat2);
   result.Longitude = RadiansToDegrees(lng2);
   return result;
 }
 
 GeoPoint PointPlusDistanceEast(GeoPoint fromCoordinate, double distance) {
   return GetPointAhead(fromCoordinate, distance, 90.0);
 }
 
 GeoPoint PointPlusDistanceNorth(GeoPoint fromCoordinate, double distance) {
   return GetPointAhead(fromCoordinate, distance, 0.0);
 }
 
 GeoPoint MetersToGeopoint(double latAsMeters, double lonAsMeters) {
   GeoPoint point = {0.0, 0.0};
   GeoPoint pointEast = PointPlusDistanceEast(point, lonAsMeters);
   GeoPoint pointNorthEast = PointPlusDistanceNorth(pointEast, latAsMeters);
   return pointNorthEast;
 }
 
 double GetDistanceMeters(GeoPoint fromCoordinate, GeoPoint toCoordinate) {
   double deltaLon = geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude);
   double deltaLat = geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude);
 
   double a = pow(sin(deltaLat / 2.0), 2) +
              cos(geoAngle(fromCoordinate.Latitude)) *
              cos(geoAngle(toCoordinate.Latitude)) *
              pow(sin(deltaLon / 2.0), 2);
   double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
   return EARTH_RADIUS * c;
 }
 
 double LatitudeToMeters(double latitude) {
   GeoPoint p1 = {latitude, 0.0};
   GeoPoint p2 = {0.0, 0.0};
   double distance = GetDistanceMeters(p1, p2);
   if (latitude < 0)
     distance *= -1;
   return distance;
 }
 
 double LongitudeToMeters(double longitude) {
   GeoPoint p1 = {0.0, longitude};
   GeoPoint p2 = {0.0, 0.0};
   double distance = GetDistanceMeters(p1, p2);
   if (longitude < 0)
     distance *= -1;
   return distance;
 }
 
 // -------------------------
 // Định nghĩa các cấu trúc ma trận tĩnh
 // -------------------------
 struct Vector2 {
   double x;
   double y;
   
   Vector2() : x(0.0), y(0.0) {}
   Vector2(double _x, double _y) : x(_x), y(_y) {}
   
   Vector2 operator+(const Vector2 &other) const {
     return Vector2(x + other.x, y + other.y);
   }
   
   Vector2 operator-(const Vector2 &other) const {
     return Vector2(x - other.x, y - other.y);
   }
   
   Vector2 operator*(double scalar) const {
     return Vector2(x * scalar, y * scalar);
   }
 };
 
 struct Matrix2x2 {
   double m00, m01;
   double m10, m11;
   
   Matrix2x2() : m00(0), m01(0), m10(0), m11(0) {}
   Matrix2x2(double a, double b, double c, double d)
     : m00(a), m01(b), m10(c), m11(d) {}
     
   Matrix2x2 operator+(const Matrix2x2 &other) const {
     return Matrix2x2(m00 + other.m00, m01 + other.m01,
                      m10 + other.m10, m11 + other.m11);
   }
   
   Matrix2x2 operator-(const Matrix2x2 &other) const {
     return Matrix2x2(m00 - other.m00, m01 - other.m01,
                      m10 - other.m10, m11 - other.m11);
   }
   
   Matrix2x2 operator*(const Matrix2x2 &other) const {
     return Matrix2x2(
       m00 * other.m00 + m01 * other.m10,
       m00 * other.m01 + m01 * other.m11,
       m10 * other.m00 + m11 * other.m10,
       m10 * other.m01 + m11 * other.m11
     );
   }
   
   Vector2 operator*(const Vector2 &v) const {
     return Vector2(
       m00 * v.x + m01 * v.y,
       m10 * v.x + m11 * v.y
     );
   }
   
   Matrix2x2 transpose() const {
     return Matrix2x2(m00, m10, m01, m11);
   }
   
   double determinant() const {
     return m00 * m11 - m01 * m10;
   }
   
   Matrix2x2 inverse() const {
     double det = determinant();
     if (fabs(det) < 1e-9) {
       return Matrix2x2();  // Trả về ma trận 0 nếu không khả nghịch
     }
     double invDet = 1.0 / det;
     return Matrix2x2(
       m11 * invDet, -m01 * invDet,
       -m10 * invDet, m00 * invDet
     );
   }
 };
 
 Matrix2x2 IdentityMatrix() {
   return Matrix2x2(1, 0, 0, 1);
 }
 
 // -------------------------
 // Lớp Kalman Filter (phiên bản tối ưu cho ESP32)
 // -------------------------
 class KalmanFilterFusedPositionAccelerometer {
 public:
   Matrix2x2 I;  // Ma trận đơn vị
   Matrix2x2 P;  // Ma trận hiệp phương sai lỗi
   Matrix2x2 Q;  // Ma trận nhiễu hệ thống
   Matrix2x2 R;  // Ma trận nhiễu đo lường
   double u;     // Đầu vào (accelerometer)
   Vector2 z;    // Vector đo lường (GPS: position, velocity)
   Matrix2x2 A;  // Ma trận chuyển trạng thái
   Vector2 B;    // Ma trận kiểm soát dạng vector 2x1
   Vector2 currentState; // Trạng thái hiện tại [position; velocity]
   double currentStateTimestampSeconds;
   
   // Lưu lại giá trị độ lệch chuẩn accelerometer ban đầu (sigma_a)
   double sigma_a;
   
   KalmanFilterFusedPositionAccelerometer(double initialPosition, double initialVelocity,
                                            double positionStdDev, double accelerometerStdDev,
                                            double currentTimestampSeconds)
   {
     currentState = Vector2(initialPosition, initialVelocity);
     currentStateTimestampSeconds = currentTimestampSeconds;
     I = IdentityMatrix();
     P = IdentityMatrix();  // Có thể tinh chỉnh theo ứng dụng cụ thể
     sigma_a = accelerometerStdDev; // Lưu lại sigma_a ban đầu
     Q = Matrix2x2(sigma_a * sigma_a, 0,
                   0, sigma_a * sigma_a); // Khởi tạo ban đầu (sẽ được tính lại theo dt trong Predict)
     R = Matrix2x2(positionStdDev * positionStdDev, 0,
                   0, positionStdDev * positionStdDev);
     A = Matrix2x2(); // Sẽ được cập nhật trong recreateStateTransitionMatrix
     B = Vector2();   // Sẽ được cập nhật trong recreateControlMatrix
     u = 0.0;
     z = Vector2();
   }
   
   // Cập nhật ma trận kiểm soát: B = [0.5*Δt²; Δt]
   void recreateControlMatrix(double deltaSeconds) {
     double dtSquared = 0.5 * deltaSeconds * deltaSeconds;
     B = Vector2(dtSquared, deltaSeconds);
   }
   
   // Cập nhật ma trận chuyển trạng thái: A = [[1, Δt], [0, 1]]
   void recreateStateTransitionMatrix(double deltaSeconds) {
     A = Matrix2x2(1.0, deltaSeconds, 0.0, 1.0);
   }
   
   // Prediction step: tính Q theo Δt và cập nhật trạng thái, hiệp phương sai
   void Predict(double accelerationThisAxis, double timestampNow) {
     double deltaT = timestampNow - currentStateTimestampSeconds;
     recreateControlMatrix(deltaT);
     recreateStateTransitionMatrix(deltaT);
     u = accelerationThisAxis;
     
     // Tính lại Q theo Δt dựa trên sigma_a:
     double dt2 = deltaT * deltaT;
     double dt3 = dt2 * deltaT;
     double dt4 = dt3 * deltaT;
     Q = Matrix2x2(0.25 * dt4 * sigma_a * sigma_a, 0.5 * dt3 * sigma_a * sigma_a,
                   0.5 * dt3 * sigma_a * sigma_a, dt2 * sigma_a * sigma_a);
     
     // Dự đoán trạng thái: x̂ₖ₊₁|ₖ = A*xₖ|ₖ + B*u
     currentState = A * currentState + B * u;
     // Dự đoán hiệp phương sai: Pₖ₊₁|ₖ = A*Pₖ|ₖ*Aᵀ + Q
     P = A * P * A.transpose() + Q;
     currentStateTimestampSeconds = timestampNow;
   }
   
   // Update step: sử dụng dạng Joseph Form để cập nhật P
   void Update(double measuredPosition, double measuredVelocity, double* positionError, double velocityError) {
     // Thiết lập vector đo lường: z = [position; velocity]
     z = Vector2(measuredPosition, measuredVelocity);
     // Cập nhật ma trận nhiễu đo R (nếu có lỗi đo vị trí được cung cấp)
     if (positionError != nullptr) {
       R.m00 = (*positionError) * (*positionError);
     }
     R.m11 = velocityError * velocityError;
     
     // Giả sử mô hình đo lường là tuyến tính: H = I
     Matrix2x2 H = IdentityMatrix();
     
     // Tính đổi mới: y = z - H*currentState = z - currentState
     Vector2 y = z - currentState;
     
     // Tính hiệp phương sai đổi mới: S = H*P*Hᵀ + R = P + R
     Matrix2x2 S = P + R;
     Matrix2x2 SInv = S.inverse();
     
     // Kalman Gain: K = P*Hᵀ*S⁻¹ = P*S⁻¹
     Matrix2x2 K = P * SInv;
     
     // Cập nhật trạng thái theo dạng chuẩn: x̂ₖ₊₁ = x̂ₖ₊₁ + K*y
     currentState = currentState + K * y;
     
     // Cập nhật ma trận hiệp phương sai bằng dạng Joseph form:\n    // P = (I - K*H)*P*(I - K*H)ᵀ + K*R*Kᵀ, với H = I
     Matrix2x2 I = IdentityMatrix();
     Matrix2x2 IK = I - K;
     P = IK * P * IK.transpose() + K * R * K.transpose();
   }
   
   double GetPredictedPosition() {
     return currentState.x;
   }
   
   double GetPredictedVelocity() {
     return currentState.y;
   }
 };
  
 // -------------------------
 // Cấu trúc dữ liệu cảm biến được gửi qua serial (JSON)
 // -------------------------
 struct sensorData {
   double Timestamp;
   double GpsLat;
   double GpsLon;
   double GpsAlt;
   float Pitch;
   float Yaw;
   float Roll;
   float AbsNorthAcc;
   float AbsEastAcc;
   float AbsUpAcc;
   double VelNorth;
   double VelEast;
   double VelDown;
   double VelError;
   double AltitudeError;
 };
  
 // -------------------------
 // Các biến toàn cục cho Kalman Filter
 // -------------------------
 KalmanFilterFusedPositionAccelerometer* kfEast = nullptr;
 KalmanFilterFusedPositionAccelerometer* kfNorth = nullptr;
 KalmanFilterFusedPositionAccelerometer* kfAlt = nullptr;
 bool filterInitialized = false;
  
 // Hàm cập nhật giá trị của Kalman Filter dựa trên dữ liệu JSON đã parse
 void updateKalmanFilter(const sensorData &data) {
   // Nếu chưa khởi tạo, khởi tạo Kalman Filter dựa trên dữ liệu đầu tiên
   if (!filterInitialized) {
     double latLonStdDev = 2.0;
     double altStdDev = (double)3.518522;
     double accelEastStdDev = ACTUAL_GRAVITY * 0.033436;
     double accelNorthStdDev = ACTUAL_GRAVITY * 0.053553;
     double accelUpStdDev = ACTUAL_GRAVITY * 0.208868;
      
     double initialLonMeters = LongitudeToMeters(data.GpsLon);
     double initialLatMeters = LatitudeToMeters(data.GpsLat);
      
     kfEast = new KalmanFilterFusedPositionAccelerometer(initialLonMeters, data.VelEast,
                                                          latLonStdDev, accelEastStdDev,
                                                          data.Timestamp);
     kfNorth = new KalmanFilterFusedPositionAccelerometer(initialLatMeters, data.VelNorth,
                                                           latLonStdDev, accelNorthStdDev,
                                                           data.Timestamp);
     kfAlt = new KalmanFilterFusedPositionAccelerometer(data.GpsAlt, -data.VelDown,
                                                         altStdDev, accelUpStdDev,
                                                         data.Timestamp);
     filterInitialized = true;
   }
    
   // Prediction step: sử dụng dữ liệu gia tốc (đã nhân với trọng lực)
   kfEast->Predict(data.AbsEastAcc * ACTUAL_GRAVITY, data.Timestamp);
   kfNorth->Predict(data.AbsNorthAcc * ACTUAL_GRAVITY, data.Timestamp);
   kfAlt->Predict(data.AbsUpAcc * ACTUAL_GRAVITY, data.Timestamp);
    
   // Nếu dữ liệu GPS hợp lệ, thực hiện Update step
   if (data.GpsLat != 0.0) {
     double lonMeters = LongitudeToMeters(data.GpsLon);
     kfEast->Update(lonMeters, data.VelEast, nullptr, data.VelError);
      
     double latMeters = LatitudeToMeters(data.GpsLat);
     kfNorth->Update(latMeters, data.VelNorth, nullptr, data.VelError);
      
     double vUp = -data.VelDown;
     kfAlt->Update(data.GpsAlt, vUp, (double*)&data.AltitudeError, data.VelError);
   }
    
   // Sau khi cập nhật, in ra kết quả dự đoán
   double predictedLonMeters = kfEast->GetPredictedPosition();
   double predictedLatMeters = kfNorth->GetPredictedPosition();
   double predictedAlt = kfAlt->GetPredictedPosition();
   GeoPoint point = MetersToGeopoint(predictedLatMeters, predictedLonMeters);
    
   double predictedVE = kfEast->GetPredictedVelocity();
   double predictedVN = kfNorth->GetPredictedVelocity();
   double resultantV = sqrt(predictedVE * predictedVE + predictedVN * predictedVN);
    
   Serial.print("Time: ");
   Serial.print(data.Timestamp, 3);
   Serial.print(" sec, Lat: ");
   Serial.print(point.Latitude, 7);
   Serial.print(", Lon: ");
   Serial.print(point.Longitude, 7);
   Serial.print(", Alt: ");
   Serial.print(predictedAlt, 2);
   Serial.print(", V(mph): ");
   Serial.println(2.23694 * resultantV, 2);
 }
  
 // -------------------------
 // Hàm đọc JSON từ Serial
 // -------------------------
 bool readJsonFromSerial(sensorData &data) {
   // Giả sử mỗi JSON được gửi trên một dòng kết thúc bằng '\\n'
   if (Serial.available()) {
     String jsonString = Serial.readStringUntil('\n');
     if (jsonString.length() > 0) {
       // Tạo một StaticJsonDocument với dung lượng phù hợp (có thể tinh chỉnh)
       DynamicJsonDocument doc(1024);
       DeserializationError error = deserializeJson(doc, jsonString);
       if (error) {
         Serial.print("JSON parse error: ");
         Serial.println(error.f_str());
         return false;
       }
       // Lấy các trường từ JSON
       data.Timestamp = doc["timestamp"] | 0.0;
       data.GpsLat = doc["gps_lat"] | 0.0;
       data.GpsLon = doc["gps_lon"] | 0.0;
       data.GpsAlt = doc["gps_alt"] | 0.0;
       data.Pitch = doc["pitch"] | 0.0;
       data.Yaw = doc["yaw"] | 0.0;
       data.Roll = doc["roll"] | 0.0;
       data.AbsNorthAcc = doc["abs_north_acc"] | 0.0;
       data.AbsEastAcc = doc["abs_east_acc"] | 0.0;
       data.AbsUpAcc = doc["abs_up_acc"] | 0.0;
       data.VelNorth = doc["vel_north"] | 0.0;
       data.VelEast = doc["vel_east"] | 0.0;
       data.VelDown = doc["vel_down"] | 0.0;
       data.VelError = doc["vel_error"] | 0.0;
       data.AltitudeError = doc["altitude_error"] | 0.0;
       return true;
     }
   }
   return false;
 }
  
 // -------------------------
 // Hàm setup() – khởi tạo ESP32
 // -------------------------
 void setup() {
   Serial.begin(115200);
   Serial.println("ESP32 Kalman Filter Test Started");
   // Chờ Serial sẵn sàng (nếu cần)
   delay(1000);
 }
  
 // -------------------------
 // Hàm loop() – vòng lặp chính
 // -------------------------
 void loop() {
   sensorData currentData;
   // Đọc dữ liệu JSON từ Serial (nếu có)
   if (readJsonFromSerial(currentData)) {
     updateKalmanFilter(currentData);
   }
   // Có thể thêm delay ngắn nếu cần
   delay(10);
 }
 