import json
import math
import numpy as np

# Constants
EARTH_RADIUS = 6371 * 1000.0  # in meters
ACTUAL_GRAVITY = 9.80665
FOUR_SPACES = "    "

# Geospatial functions
def degrees_to_radians(deg):
    return deg * math.pi / 180.0

def radians_to_degrees(rad):
    return rad * 180.0 / math.pi

def geo_angle(lat_or_lon):
    return degrees_to_radians(lat_or_lon)

def get_distance_meters(from_coordinate, to_coordinate):
    # Haversine formula
    delta_lat = geo_angle(to_coordinate['Latitude'] - from_coordinate['Latitude'])
    delta_lon = geo_angle(to_coordinate['Longitude'] - from_coordinate['Longitude'])
    lat1 = geo_angle(from_coordinate['Latitude'])
    lat2 = geo_angle(to_coordinate['Latitude'])
    
    a = (math.sin(delta_lat / 2.0) ** 2 +
         math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

def get_point_ahead(from_coordinate, distance_meters, azimuth_deg):
    radius_fraction = distance_meters / EARTH_RADIUS
    bearing = degrees_to_radians(azimuth_deg)
    
    lat1 = geo_angle(from_coordinate['Latitude'])
    lng1 = geo_angle(from_coordinate['Longitude'])
    
    lat2_part = math.sin(lat1) * math.cos(radius_fraction) + math.cos(lat1) * math.sin(radius_fraction) * math.cos(bearing)
    lat2 = math.asin(lat2_part)
    
    lng2_part1 = math.sin(bearing) * math.sin(radius_fraction) * math.cos(lat1)
    lng2_part2 = math.cos(radius_fraction) - math.sin(lat1) * math.sin(lat2)
    lng2 = lng1 + math.atan2(lng2_part1, lng2_part2)
    lng2 = (lng2 + 3*math.pi) % (2*math.pi) - math.pi  # normalize
    return {
        'Latitude': radians_to_degrees(lat2),
        'Longitude': radians_to_degrees(lng2)
    }

def point_plus_distance_east(from_coordinate, distance):
    # 90 degrees is east
    return get_point_ahead(from_coordinate, distance, 90.0)

def point_plus_distance_north(from_coordinate, distance):
    # 0 degrees is north
    return get_point_ahead(from_coordinate, distance, 0.0)

def meters_to_geopoint(lat_meters, lon_meters):
    # Starting at (0,0)
    origin = {'Latitude': 0.0, 'Longitude': 0.0}
    point_east = point_plus_distance_east(origin, lon_meters)
    point_north_east = point_plus_distance_north(point_east, lat_meters)
    return point_north_east

def latitude_to_meters(latitude):
    # Calculate meters from equator along latitude
    point = {'Latitude': latitude, 'Longitude': 0.0}
    origin = {'Latitude': 0.0, 'Longitude': 0.0}
    distance = get_distance_meters(point, origin)
    return -distance if latitude < 0 else distance

def longitude_to_meters(longitude):
    point = {'Latitude': 0.0, 'Longitude': longitude}
    origin = {'Latitude': 0.0, 'Longitude': 0.0}
    distance = get_distance_meters(point, origin)
    return -distance if longitude < 0 else distance

# Kalman Filter Class for Fused Position and Accelerometer Data
class KalmanFilterFusedPositionAccelerometer:
    def __init__(self, initial_position, initial_velocity,
                 position_std, accel_std, current_timestamp):
        # The state is a 2x1 vector [position, velocity]
        self.current_state = np.array([[initial_position], [initial_velocity]], dtype=float)
        # Identity matrix I
        self.I = np.eye(2)
        # Transformation matrix; for now, using identity.
        self.H = np.eye(2)
        self.P = np.eye(2)
        # Process error covariance matrix Q
        self.Q = np.array([[accel_std**2, 0],
                           [0, accel_std**2]], dtype=float)
        # Measurement error covariance matrix R
        # Note: both rows use positionStd (could adjust the second row if needed)
        self.R = np.array([[position_std**2, 0],
                           [0, position_std**2]], dtype=float)
        # Control input matrix u (acceleration)
        self.u = np.zeros((1, 1), dtype=float)
        # Measurement vector matrix z (GPS measurement, position, and velocity)
        self.z = np.zeros((2, 1), dtype=float)
        # Control matrix B (will be updated)
        self.B = np.zeros((2, 1), dtype=float)
        # State transition matrix A (will be updated)
        self.A = np.zeros((2, 2), dtype=float)
        self.current_timestamp = current_timestamp

    def predict(self, acceleration, timestamp_now):
        deltaT = timestamp_now - self.current_timestamp
        # Update control and state transition matrices based on deltaT
        self.recreate_control_matrix(deltaT)
        self.recreate_state_transition_matrix(deltaT)
        
        self.u[0, 0] = acceleration
        
        # Predict state: current_state = A*current_state + B*u
        self.current_state = np.add(np.dot(self.A, self.current_state),
                                    np.dot(self.B, self.u))
        # Update error covariance: P = A*P*A^T + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        
        self.current_timestamp = timestamp_now

    def update(self, position, velocity, position_error, velocity_error):
        self.z[0, 0] = position
        self.z[1, 0] = velocity

        if position_error is not None:
            self.R[0, 0] = position_error * position_error
        # Always update the second diagonal element
        self.R[1, 1] = velocity_error * velocity_error
        
        # y = z - current_state (residual)
        y = self.z - self.current_state
        s = self.P + self.R
        # Compute inverse of s; if not invertible, skip update
        try:
            s_inv = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            return
        
        # Kalman gain
        K = np.dot(self.P, s_inv)
        # Update state estimate
        self.current_state = self.current_state + np.dot(K, y)
        # Update error covariance
        self.P = np.dot((self.I - K), self.P)

    def recreate_control_matrix(self, deltaT):
        dt_squared = 0.5 * deltaT * deltaT
        self.B[0, 0] = dt_squared
        self.B[1, 0] = deltaT

    def recreate_state_transition_matrix(self, deltaT):
        self.A[0, 0] = 1.0
        self.A[0, 1] = deltaT
        self.A[1, 0] = 0.0
        self.A[1, 1] = 1.0

    def get_predicted_position(self):
        return self.current_state[0, 0]

    def get_predicted_velocity(self):
        return self.current_state[1, 0]

# Utility functions for file IO
def read_file_as_json(filename):
    with open(filename, 'r') as f:
        return json.load(f)

def write_json_serializable_to_file(json_entity, filename):
    with open(filename, 'w') as f:
        json.dump(json_entity, f, indent=4)

# Main function to process sensor data and apply filters
def main():
    # Load sensor data from JSON file ("pos_final.json")
    collection = read_file_as_json("pos_final.json")
    if not collection:
        print("No sensor data found!")
        return

    # The first sensor reading is used to initialize filters.
    initial_sensor_data = collection[0]

    # Standard deviations (adjusted as per original code comments)
    lat_lon_std = 2.0        # +/- 1m, increased for safety
    altitude_std = 3.518522417151836

    # Accelerometer standard deviations derived from ACTUAL_GRAVITY
    accel_east_std = ACTUAL_GRAVITY * 0.033436506994600976
    accel_north_std = ACTUAL_GRAVITY * 0.05355371135598354
    accel_up_std = ACTUAL_GRAVITY * 0.2088683796078286

    # Initialize Kalman filters for longitude, latitude and altitude
    longitude_filter = KalmanFilterFusedPositionAccelerometer(
        longitude_to_meters(initial_sensor_data['GpsLon']),
        initial_sensor_data['VelEast'],
        lat_lon_std,
        accel_east_std,
        initial_sensor_data['timestamp']
    )
    latitude_filter = KalmanFilterFusedPositionAccelerometer(
        latitude_to_meters(initial_sensor_data['GpsLat']),
        initial_sensor_data['VelNorth'],
        lat_lon_std,
        accel_north_std,
        initial_sensor_data['timestamp']
    )
    altitude_filter = KalmanFilterFusedPositionAccelerometer(
        initial_sensor_data['GpsAlt'],
        -initial_sensor_data['VelDown'],  # note inversion as in original
        altitude_std,
        accel_up_std,
        initial_sensor_data['timestamp']
    )

    outputs = []
    for i in range(1, len(collection)):
        data = collection[i]
        timestamp = data['timestamp']

        # Predict step (accelerometer data used here)
        longitude_filter.predict(data['AbsEastAcc'] * ACTUAL_GRAVITY, timestamp)
        latitude_filter.predict(data['AbsNorthAcc'] * ACTUAL_GRAVITY, timestamp)
        altitude_filter.predict(data['AbsUpAcc'] * ACTUAL_GRAVITY, timestamp)

        # If GPS data is available (non-zero latitude assumed)
        if data['GpsLat'] != 0.0:
            position_err = None  # equivalent to nil
            longitude_meters = longitude_to_meters(data['GpsLon'])
            longitude_filter.update(
                longitude_meters,
                data['VelEast'],
                position_err,
                data['VelError']
            )

            latitude_meters = latitude_to_meters(data['GpsLat'])
            latitude_filter.update(
                latitude_meters,
                data['VelNorth'],
                position_err,
                data['VelError']
            )

            altitude_filter.update(
                data['GpsAlt'],
                -data['VelDown'],   # inversion as in original code
                data.get('AltitudeError', 0),
                data['VelError']
            )

        predicted_lon_meters = longitude_filter.get_predicted_position()
        predicted_lat_meters = latitude_filter.get_predicted_position()
        predicted_alt = altitude_filter.get_predicted_position()

        point = meters_to_geopoint(predicted_lat_meters, predicted_lon_meters)
        predicted_lon = point['Longitude']
        predicted_lat = point['Latitude']

        predicted_v_east = longitude_filter.get_predicted_velocity()
        predicted_v_north = latitude_filter.get_predicted_velocity()
        resultant_v = math.sqrt(predicted_v_east ** 2 + predicted_v_north ** 2)

        deltaT = timestamp - initial_sensor_data['timestamp']
        print(f"{deltaT:.6f} seconds in, Lat: {predicted_lat:.6f}, Lon: {predicted_lon:.6f}, Alt: {predicted_alt:.6f}, V(mph): {2.23694 * resultant_v:.6f}")

        out_packet = {
            "timestamp": data['timestamp'],
            "GpsLat": data['GpsLat'],
            "GpsLon": data['GpsLon'],
            "PredictedLat": predicted_lat,
            "PredictedLon": predicted_lon,
            "PredictedAlt": predicted_alt,
            "ResultantMPH": 2.23694 * resultant_v,
            # include other sensor data if needed
        }
        outputs.append(out_packet)
    
    # Write outputs to file
    write_json_serializable_to_file(outputs, "finalOut.json")
    print("Finished processing without crash.")

if __name__ == "__main__":
    main()