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
struct GeoPoint
{
    double Latitude;
    double Longitude;
};

// -------------------------
// Các hàm chuyển đổi góc
// -------------------------
Radians DegreesToRadians(Degrees deg)
{
    return deg * M_PI / 180.0;
}

Degrees RadiansToDegrees(Radians rad)
{
    return rad * 180.0 / M_PI;
}

double geoAngle(double angleInDegrees)
{
    return DegreesToRadians(angleInDegrees);
}

// -------------------------
// Các hàm hình học địa lý
// -------------------------
GeoPoint GetPointAhead(GeoPoint fromCoordinate, double distanceMeters, Degrees azimuth)
{
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

GeoPoint PointPlusDistanceEast(GeoPoint fromCoordinate, double distance)
{
    return GetPointAhead(fromCoordinate, distance, 90.0);
}

GeoPoint PointPlusDistanceNorth(GeoPoint fromCoordinate, double distance)
{
    return GetPointAhead(fromCoordinate, distance, 0.0);
}

GeoPoint MetersToGeopoint(double latAsMeters, double lonAsMeters)
{
    GeoPoint point = {0.0, 0.0};
    GeoPoint pointEast = PointPlusDistanceEast(point, lonAsMeters);
    GeoPoint pointNorthEast = PointPlusDistanceNorth(pointEast, latAsMeters);
    return pointNorthEast;
}

double GetDistanceMeters(GeoPoint fromCoordinate, GeoPoint toCoordinate)
{
    double deltaLon = geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude);
    double deltaLat = geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude);

    double a = pow(sin(deltaLat / 2.0), 2) +
               cos(geoAngle(fromCoordinate.Latitude)) *
                   cos(geoAngle(toCoordinate.Latitude)) *
                   pow(sin(deltaLon / 2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS * c;
}

double LatitudeToMeters(double latitude)
{
    GeoPoint p1 = {latitude, 0.0};
    GeoPoint p2 = {0.0, 0.0};
    double distance = GetDistanceMeters(p1, p2);
    if (latitude < 0)
        distance *= -1;
    return distance;
}

double LongitudeToMeters(double longitude)
{
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
struct Vector2
{
    double x;
    double y;

    Vector2() : x(0.0), y(0.0) {}
    Vector2(double _x, double _y) : x(_x), y(_y) {}

    Vector2 operator+(const Vector2 &other) const
    {
        return Vector2(x + other.x, y + other.y);
    }

    Vector2 operator-(const Vector2 &other) const
    {
        return Vector2(x - other.x, y - other.y);
    }

    Vector2 operator*(double scalar) const
    {
        return Vector2(x * scalar, y * scalar);
    }
};

// Ma trận 2x2 tĩnh
struct Matrix2x2
{
    double m00, m01;
    double m10, m11;

    Matrix2x2() : m00(0), m01(0), m10(0), m11(0) {}
    Matrix2x2(double a, double b, double c, double d)
        : m00(a), m01(b), m10(c), m11(d) {}

    Matrix2x2 operator+(const Matrix2x2 &other) const
    {
        return Matrix2x2(m00 + other.m00, m01 + other.m01,
                         m10 + other.m10, m11 + other.m11);
    }

    Matrix2x2 operator-(const Matrix2x2 &other) const
    {
        return Matrix2x2(m00 - other.m00, m01 - other.m01,
                         m10 - other.m10, m11 - other.m11);
    }

    Matrix2x2 operator*(const Matrix2x2 &other) const
    {
        return Matrix2x2(
            m00 * other.m00 + m01 * other.m10,
            m00 * other.m01 + m01 * other.m11,
            m10 * other.m00 + m11 * other.m10,
            m10 * other.m01 + m11 * other.m11);
    }

    Vector2 operator*(const Vector2 &v) const
    {
        return Vector2(
            m00 * v.x + m01 * v.y,
            m10 * v.x + m11 * v.y);
    }

    Matrix2x2 transpose() const
    {
        return Matrix2x2(m00, m10, m01, m11);
    }

    double determinant() const
    {
        return m00 * m11 - m01 * m10;
    }

    Matrix2x2 inverse() const
    {
        double det = determinant();
        if (fabs(det) < 1e-9)
        {
            // Trên vi điều khiển, bạn có thể xử lý lỗi theo cách đơn giản
            return Matrix2x2(); // Trả về ma trận 0 nếu không khả nghịch
        }
        double invDet = 1.0 / det;
        return Matrix2x2(
            m11 * invDet, -m01 * invDet,
            -m10 * invDet, m00 * invDet);
    }
};

Matrix2x2 IdentityMatrix()
{
    return Matrix2x2(1, 0, 0, 1);
}

double gps_lon, gps_lat;
double gps_lonMeters = 0.0;
double gps_latMeters = 0.0;
double gps_old_lonMeters = 0.0;
double gps_old_latMeters = 0.0;
double gps_v_north, gps_v_east;

double imu_a_north, imu_a_east; // north, east

double t_gps = 200.0f;
double t_imu = 50.0f;

double currentMillis = 0.0f;
double predict_millis = 0.0f;
int counter_for_update = 0;

class KalmanFilterFusedPositionAccelerometer
{
public:
    Matrix2x2 I;          // Ma trận đơn vị
    Matrix2x2 P;          // Ma trận hiệp phương sai lỗi
    Matrix2x2 Q;          // Ma trận nhiễu hệ thống (predict acc)
    Matrix2x2 R;          // Ma trận nhiễu đo lường (measure gps)
    double u;             // Đầu vào (accelerometer)
    Vector2 z;            // Vector đo lường (GPS: position, velocity)
    Matrix2x2 A;          // Ma trận chuyển trạng thái
    Vector2 B;            // Ma trận kiểm soát dạng vector 2x1
    Vector2 currentState; // Trạng thái hiện tại [position; velocity]

    KalmanFilterFusedPositionAccelerometer(double initialPosition, double initialVelocity,
                                           double positionStdDev, double accelerometerStdDev)
    {
        currentState = Vector2(initialPosition, initialVelocity);
        I = IdentityMatrix();
        P = IdentityMatrix(); // Ở đây khởi tạo đơn giản bằng ma trận đơn vị (có thể tinh chỉnh)
        Q = Matrix2x2(accelerometerStdDev * accelerometerStdDev, 0,
                      0, accelerometerStdDev * accelerometerStdDev); // acc
        R = Matrix2x2(positionStdDev * positionStdDev, 0,
                      0, positionStdDev * positionStdDev); // gps
        A = Matrix2x2();                                   // Sẽ được tính lại trong recreateStateTransitionMatrix
        B = Vector2();                                     // Sẽ được tính lại trong recreateControlMatrix
        u = 0.0;
        z = Vector2();
    }

    // Cập nhật ma trận kiểm soát: B = [0.5*Δt²; Δt]
    void recreateControlMatrix(double deltaSeconds)
    {
        double dtSquared = 0.5 * deltaSeconds * deltaSeconds;
        B = Vector2(dtSquared, deltaSeconds);
    }

    // Cập nhật ma trận chuyển trạng thái: A = [[1, Δt], [0, 1]]
    void recreateStateTransitionMatrix(double deltaSeconds)
    {
        A = Matrix2x2(1.0, deltaSeconds, 0.0, 1.0);
    }

    // Prediction step
    void Predict(double accelerationThisAxis)
    {
        recreateControlMatrix(t_imu);
        recreateStateTransitionMatrix(t_imu);
        u = accelerationThisAxis;
        // x̂ₖ₊₁|ₖ = A * xₖ|ₖ + B * u
        currentState = A * currentState + B * u;
        // Pₖ₊₁|ₖ = A * Pₖ|ₖ * Aᵀ + Q
        P = A * P * A.transpose() + Q;
    }

    // Update step
    void Update(double position, double velocityThisAxis, int update_with_no_effect)
    {
        // Thiết lập vector đo lường: z = [position; velocity]
        z = Vector2(position, velocityThisAxis);
        // Đổi mới: y = z - x̂ₖ₊₁|ₖ
        Vector2 y = z - currentState;
        // Hiệp phương sai đổi mới: S = P + R (với H = I)
        Matrix2x2 S;
        if(update_with_no_effect == 1){
            Matrix2x2 tempR = Matrix2x2(1000, 0, 0, 1000); // R with infinity standar varian
            S = P + tempR;
        }
        S = P + R;
        Matrix2x2 SInv = S.inverse();
        // Kalman Gain: K = P * S⁻¹
        Matrix2x2 K = P * SInv;
        // Cập nhật trạng thái: x̂ₖ₊₁|ₖ₊₁ = x̂ₖ₊₁|ₖ + K * y
        currentState = currentState + K * y;
        // Cập nhật hiệp phương sai: Pₖ₊₁|ₖ₊₁ = (I - K) * P
        P = (I - K) * P;
    }

    double GetPredictedPosition()
    {
        return currentState.x;
    }

    double GetPredictedVelocity()
    {
        return currentState.y;
    }
};


struct sensorData
{
    double Timestamp;
    double GpsLat;
    double GpsLon;
    float AbsNorthAcc;
    float AbsEastAcc;
    double VelNorth;
    double VelEast;
    double VelError;
};


void setup()
{
    Serial.begin(9600);
    Serial.println("Setup complete.");
}


void loop()
{
    currentMillis = millis();
    static bool initialized = false;
    static KalmanFilterFusedPositionAccelerometer *kfEast = nullptr;
    static KalmanFilterFusedPositionAccelerometer *kfNorth = nullptr;

    if (!initialized) // long press button => set current state (v = 0; lon, lat = mean of GPS measure)
    {
        double latLonStdDev = 2.0;      // +/- 1m, tăng thêm cho an toàn
        double accelEastStdDev = 0.22;  // do lech chuan accel                       m/s^2
        double accelNorthStdDev = 0.22; // do lech chuan accel, tu do roi tinh lai

        double initialLonMeters = LongitudeToMeters(gps_lon);
        double initialLatMeters = LatitudeToMeters(gps_lat);

        double init_velocity_east = 0.0f;
        double init_velocity_north = 0.0f;

        kfNorth = new KalmanFilterFusedPositionAccelerometer(initialLatMeters, init_velocity_north,
                                                            latLonStdDev, accelNorthStdDev);
        kfEast = new KalmanFilterFusedPositionAccelerometer(initialLonMeters, init_velocity_east,
                                                            latLonStdDev, accelEastStdDev);
        
        initialized = true;
    }

    { // get new data from sensor
        gps_lon = 0;
        gps_lat = 0;
        imu_a_north = 0;
        imu_a_east = 0;
    }

    if(currentMillis - predict_millis >= t_imu){
        predict_millis = currentMillis;
        kfNorth->Predict(imu_a_north);
        kfEast->Predict(imu_a_east);
        counter_for_update++;
    };
    
    if(counter_for_update == (int)t_gps / (int)t_imu){
        counter_for_update = 0;
        gps_old_lonMeters = gps_lonMeters;
        gps_lonMeters = LongitudeToMeters(gps_lon);
        gps_v_north = (gps_lonMeters - gps_old_lonMeters) / t_gps;
        kfNorth->Update(gps_lonMeters, gps_v_north, 0);

        gps_old_latMeters = gps_latMeters;
        gps_latMeters = LatitudeToMeters(gps_lat);
        gps_v_east = (gps_latMeters - gps_old_latMeters) / t_gps;
        kfEast->Update(gps_latMeters, gps_v_east, 0);
    }else{
        counter_for_update = 0;
        gps_old_lonMeters = gps_lonMeters;
        gps_lonMeters = LongitudeToMeters(gps_lon);
        gps_v_north = (gps_lonMeters - gps_old_lonMeters) / t_gps;
        kfNorth->Update(gps_lonMeters, gps_v_north, 1);

        gps_old_latMeters = gps_latMeters;
        gps_latMeters = LatitudeToMeters(gps_lat);
        gps_v_east = (gps_latMeters - gps_old_latMeters) / t_gps;
        kfEast->Update(gps_latMeters, gps_v_east, 1);
    }

    double predictedLonMeters = kfEast->GetPredictedPosition();
    double predictedLatMeters = kfNorth->GetPredictedPosition();
    GeoPoint point = MetersToGeopoint(predictedLatMeters, predictedLonMeters);
    double predictedVE = kfEast->GetPredictedVelocity();
    double predictedVN = kfNorth->GetPredictedVelocity();
    double resultantV = sqrt(predictedVE * predictedVE + predictedVN * predictedVN);

    // Serial.print(data.Timestamp - sensorSamples[0].Timestamp, 3);
    Serial.print(" sec, Lat: ");
    Serial.print(point.Latitude, 6);
    Serial.print(", Lon: ");
    Serial.print(point.Longitude, 6);

    Serial.print(", V(mph): ");
    Serial.print(resultantV, 3);
    Serial.println();

};