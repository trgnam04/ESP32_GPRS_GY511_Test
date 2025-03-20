#define SERIAL_BAUDRATE 115200
#define TX_PIN 18
#define RX_PIN 17
#define SDA 21
#define SCK 22
#undef HTTP    
#define MQTT   1

#define MS_TO_KMH 18.0f / 5.0f
#define KMH_TO_MS 5.0f / 18.0f

const float GRAVITY = 9.81;

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
constexpr char COLLECTOR_KEY_VELOCITY_NORTH[] = "v_n";
constexpr char COLLECTOR_KEY_VELOCITY_EAST[] = "v_e";
constexpr char COLLECTOR_KEY_STATION_ID[] = "Station-ID";
