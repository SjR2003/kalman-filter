#pragma once

#include <random>
#include <vector>

struct GPSMeasurement
{
    double x, y;
};
struct GyroMeasurement
{
    double psi_dot;
};
struct LidarMeasurement
{
    double range, theta;
    int id;
};

class BeaconMap;

class GPSSensor
{
public:
    GPSSensor();
    void reset();
    void set_gps_noise_std(double std);
    void set_gps_error_prob(double prob);
    void set_gps_denied_zone(double x, double y, double r);
    GPSMeasurement generate_gps_measurement(double sensor_x, double sensor_y);

private:
    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_error_prob;
    double m_gps_denied_x, m_gps_denied_y, m_gps_denied_range;
};

class GyroSensor
{
public:
    GyroSensor();
    void reset();
    void set_gyro_noise_std(double std);
    void set_gyro_bias(double bias);
    GyroMeasurement generate_gyro_measurement(double sensor_yaw_rate);

private:
    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_bias;
};

class LidarSensor
{
public:
    LidarSensor();
    void reset();
    void set_lidar_noise_std(double range_std, double theta_std);
    void set_lidar_max_range(double range);
    void set_lidar_da_enabled(bool id_enabled);
    std::vector<LidarMeasurement> generate_lidar_measurements(double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap &map);

private:
    std::mt19937 m_rand_gen;
    double m_range_noise_std;
    double m_theta_noise_std;
    double m_max_range;
    bool m_id_enabled;
};