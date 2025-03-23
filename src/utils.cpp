#include "utils.h"
float extractVectorToNorthVolume(float velocity, float headingRadians)
{
    return velocity * cos(headingRadians);
}
float extractVectorToEastVolume(float velocity, float headingRadians)
{
    return velocity * sin(headingRadians);
}
void calculateNorthEastVelocity(float v_x, float v_y, float headingRadians, float &v_B, float &v_D)
{
    v_B = extractVectorToNorthVolume(v_x, headingRadians) + extractVectorToNorthVolume(v_y, headingRadians);
    v_D = extractVectorToEastVolume(v_x, headingRadians) + extractVectorToEastVolume(v_y, headingRadians);
}
// Reference : https://gis.stackexchange.com/questions/312831/get-lat-long-given-current-point-distance-and-bearing/313006#313006
void calculateNextLocation(double latitude, double longtitude, float v_B, float v_D, float delta, double &next_latitude, double &next_longtitude)
{
    // Convert to radians
    latitude = latitude / 180 * PI;
    longtitude = longtitude / 180 * PI;
    // Calculate bearing and distance
    float bearing = atan2(v_D, v_B);
    float d = sqrt(v_B * v_B + v_D * v_D) / 1000;
    // Calculate next location
    next_latitude = asin(sin(latitude) * cos(d / 6371) + cos(latitude) * sin(d / 6371) * cos(bearing));
    next_longtitude = longtitude + atan2(sin(bearing) * sin(d / 6371) * cos(latitude), cos(d / 6371) - sin(latitude) * sin(next_latitude));
    // Check if next location is out of range
    if (next_latitude < 0)
    {
        next_latitude += 2 * PI;
    }
    else if (next_latitude > 2 * PI)
    {
        next_latitude += 2 * PI;
    }
    if (next_longtitude < 0)
    {
        next_longtitude += 2 * PI;
    }
    else if (next_longtitude > 2 * PI)
    {
        next_longtitude += 2 * PI;
    }
    // Convert to degrees
    next_latitude = next_latitude * 180 / PI;
    next_longtitude = next_longtitude * 180 / PI;
}