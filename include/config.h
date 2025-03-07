#define SERIAL_BAUDRATE 9600
#define TX_PIN 18
#define RX_PIN 17
#define SDA 21
#define SCK 22
#undef HTTP    
#define MQTT   1

// set up Wifi
constexpr char WIFI_SSID[] = "v";
constexpr char WIFI_PASSWORD[] = "12345678";
constexpr char TOKEN[] = "COLLECTOR";


// set up key
constexpr char COLLECTOR_KEY_TRIP_NUMBER[] = "Trip-Number";
constexpr char COLLECTOR_KEY_ROUTE_ID[] = "Route-ID";
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
constexpr char COLLECTOR_KEY_STATION_ID[] = "Station-ID";
