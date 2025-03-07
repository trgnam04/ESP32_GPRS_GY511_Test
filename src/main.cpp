/*
 * ESP32 Kalman Filter for Position Fusion
 * Phiên bản tối ưu cho môi trường vi điều khiển với bộ nhớ hạn chế.
 */

 #include <Arduino.h>
 #include <math.h>
 
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
 
 // Vector 2 phần tử
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
 
 // Ma trận 2x2 tĩnh
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
       // Trên vi điều khiển, bạn có thể xử lý lỗi theo cách đơn giản
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
   
   KalmanFilterFusedPositionAccelerometer(double initialPosition, double initialVelocity,
                                            double positionStdDev, double accelerometerStdDev,
                                            double currentTimestampSeconds)
   {
     currentState = Vector2(initialPosition, initialVelocity);
     currentStateTimestampSeconds = currentTimestampSeconds;
     I = IdentityMatrix();
     P = IdentityMatrix();  // Ở đây khởi tạo đơn giản bằng ma trận đơn vị (có thể tinh chỉnh)
     Q = Matrix2x2(accelerometerStdDev * accelerometerStdDev, 0,
                   0, accelerometerStdDev * accelerometerStdDev);
     R = Matrix2x2(positionStdDev * positionStdDev, 0,
                   0, positionStdDev * positionStdDev);
     A = Matrix2x2(); // Sẽ được tính lại trong recreateStateTransitionMatrix
     B = Vector2();   // Sẽ được tính lại trong recreateControlMatrix
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
   
   // Prediction step
   void Predict(double accelerationThisAxis, double timestampNow) {
     double deltaT = timestampNow - currentStateTimestampSeconds;
     recreateControlMatrix(deltaT);
     recreateStateTransitionMatrix(deltaT);
     u = accelerationThisAxis;
     // x̂ₖ₊₁|ₖ = A * xₖ|ₖ + B * u
     currentState = A * currentState + B * u;
     // Pₖ₊₁|ₖ = A * Pₖ|ₖ * Aᵀ + Q
     P = A * P * A.transpose() + Q;
     currentStateTimestampSeconds = timestampNow;
   }
   
   // Update step
   void Update(double position, double velocityThisAxis, double* positionError, double velocityError) {
     // Thiết lập vector đo lường: z = [position; velocity]
     z = Vector2(position, velocityThisAxis);
     if (positionError != nullptr) {
       R.m00 = (*positionError) * (*positionError);
     }
     R.m11 = velocityError * velocityError;
     // Đổi mới: y = z - x̂ₖ₊₁|ₖ
     Vector2 y = z - currentState;
     // Hiệp phương sai đổi mới: S = P + R (với H = I)
     Matrix2x2 S = P + R;
     Matrix2x2 SInv = S.inverse();
     // Kalman Gain: K = P * S⁻¹
     Matrix2x2 K = P * SInv;
     // Cập nhật trạng thái: x̂ₖ₊₁|ₖ₊₁ = x̂ₖ₊₁|ₖ + K * y
     currentState = currentState + K * y;
     // Cập nhật hiệp phương sai: Pₖ₊₁|ₖ₊₁ = (I - K) * P
     P = (I - K) * P;
   }
   
   double GetPredictedPosition() {
     return currentState.x;
   }
   
   double GetPredictedVelocity() {
     return currentState.y;
   }
 };
 
 // -------------------------
 // Giả lập dữ liệu cảm biến (đối với ESP32, thay thế bằng đọc từ cảm biến thật)
 // -------------------------
 struct sensorData {
   double Timestamp;
   double GpsLat;
   double GpsLon;
   double GpsAlt;
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
 // Biến toàn cục (cho ví dụ đơn giản)
 // -------------------------
 const int numSamples = 10; // Giả sử có 10 mẫu dữ liệu
 sensorData sensorSamples[numSamples];  // Bạn có thể cập nhật giá trị thực từ cảm biến
 
 // -------------------------
 // Hàm setup() – chạy một lần khi khởi động ESP32
 // -------------------------
 void setup() {
   Serial.begin(9600);
   // Giả lập dữ liệu mẫu (trong thực tế, bạn sẽ lấy từ cảm biến)
   for (int i = 0; i < numSamples; i++) {
     sensorSamples[i].Timestamp = i * 0.1;  // 0.1 giây giữa các mẫu
     sensorSamples[i].GpsLat = 10.0 + 0.001 * i;
     sensorSamples[i].GpsLon = 106.0 + 0.001 * i;
     sensorSamples[i].GpsAlt = 50.0;
     sensorSamples[i].AbsNorthAcc = 0.01;
     sensorSamples[i].AbsEastAcc = 0.02;
     sensorSamples[i].AbsUpAcc = 0.005;
     sensorSamples[i].VelNorth = 0.0;
     sensorSamples[i].VelEast = 0.0;
     sensorSamples[i].VelDown = 0.0;
     sensorSamples[i].VelError = 0.5;
     sensorSamples[i].AltitudeError = 1.0;
   }
   Serial.println("Setup complete.");
 }
 
 // -------------------------
 // Hàm loop() – chạy liên tục
 // -------------------------
 void loop() {
   static bool initialized = false;
   static KalmanFilterFusedPositionAccelerometer* kfEast = nullptr;
   static KalmanFilterFusedPositionAccelerometer* kfNorth = nullptr;
   static KalmanFilterFusedPositionAccelerometer* kfAlt = nullptr;
   
   // Khởi tạo bộ lọc Kalman dựa trên mẫu đầu tiên (nếu chưa khởi tạo)
   if (!initialized) {
     sensorData initData = sensorSamples[0];
     double latLonStdDev = 2.0; // +/- 1m, tăng thêm cho an toàn
     double altStdDev = 3.5;
     double accelEastStdDev = ACTUAL_GRAVITY * 0.0334;
     double accelNorthStdDev = ACTUAL_GRAVITY * 0.0536;
     double accelUpStdDev = ACTUAL_GRAVITY * 0.2089;
     
     double initialLonMeters = LongitudeToMeters(initData.GpsLon);
     double initialLatMeters = LatitudeToMeters(initData.GpsLat);
     
     kfEast = new KalmanFilterFusedPositionAccelerometer(initialLonMeters, initData.VelEast,
                                                         latLonStdDev, accelEastStdDev,
                                                         initData.Timestamp);
     kfNorth = new KalmanFilterFusedPositionAccelerometer(initialLatMeters, initData.VelNorth,
                                                          latLonStdDev, accelNorthStdDev,
                                                          initData.Timestamp);
     kfAlt = new KalmanFilterFusedPositionAccelerometer(initData.GpsAlt, -initData.VelDown,
                                                        altStdDev, accelUpStdDev,
                                                        initData.Timestamp);
     initialized = true;
   }
   
   // Giả sử mỗi 100ms cập nhật một mẫu dữ liệu (vòng lặp đơn giản)
   static int sampleIndex = 1;
   if (sampleIndex >= numSamples) {
     sampleIndex = 1; // reset
   }
   
   sensorData data = sensorSamples[sampleIndex];
   
   // Prediction step (chia tỷ lệ gia tốc theo trọng lực)
   kfEast->Predict(data.AbsEastAcc * ACTUAL_GRAVITY, data.Timestamp);
   kfNorth->Predict(data.AbsNorthAcc * ACTUAL_GRAVITY, data.Timestamp);
   kfAlt->Predict(data.AbsUpAcc * ACTUAL_GRAVITY, data.Timestamp);
   
   // Update step nếu có dữ liệu GPS hợp lệ (giả sử GpsLat != 0)
   if (data.GpsLat != 0.0) {
     double lonMeters = LongitudeToMeters(data.GpsLon);
     kfEast->Update(lonMeters, data.VelEast, nullptr, data.VelError);
     
     double latMeters = LatitudeToMeters(data.GpsLat);
     kfNorth->Update(latMeters, data.VelNorth, nullptr, data.VelError);
     
     double vUp = -data.VelDown;
     kfAlt->Update(data.GpsAlt, vUp, &data.AltitudeError, data.VelError);
   }
   else{
    // do something in order to not affect the covariance.
   }
   
   
   // Lấy kết quả dự đoán và chuyển đổi về tọa độ địa lý
   double predictedLonMeters = kfEast->GetPredictedPosition();
   double predictedLatMeters = kfNorth->GetPredictedPosition();
   double predictedAlt = kfAlt->GetPredictedPosition();
   GeoPoint point = MetersToGeopoint(predictedLatMeters, predictedLonMeters);
   
   double predictedVE = kfEast->GetPredictedVelocity();
   double predictedVN = kfNorth->GetPredictedVelocity();
   double resultantV = sqrt(predictedVE * predictedVE + predictedVN * predictedVN);
   
   Serial.print(data.Timestamp - sensorSamples[0].Timestamp, 3);
   Serial.print(" sec, Lat: ");
   Serial.print(point.Latitude, 6);
   Serial.print(", Lon: ");
   Serial.print(point.Longitude, 6);
   Serial.print(", Alt: ");
   Serial.print(predictedAlt, 2);
   Serial.print(", V(mph): ");
   Serial.print(2.23694 * resultantV, 2);
   Serial.println();
   
   sampleIndex++;
   delay(100);  // 100ms delay
 }
 