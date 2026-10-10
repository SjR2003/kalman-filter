#pragma once

#include <vector>
#include <Eigen/Dense>

#include "simulation/car.h"
#include "simulation/beacons.h"
#include "utils/sensors.h"

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

class KalmanFilterBase
{
public:
    KalmanFilterBase() : m_init(false) {}
    virtual ~KalmanFilterBase() {}
    void reset() { m_init = false; }
    bool is_init() const { return m_init; }

protected:
    VectorXd get_state() const { return m_state; }
    MatrixXd get_covariance() const { return m_covariance; }
    void set_state(const VectorXd &state)
    {
        m_state = state;
        m_init = true;
    }
    void set_covariance(const MatrixXd &cov) { m_covariance = cov; }

private:
    bool m_init;
    VectorXd m_state;
    MatrixXd m_covariance;
};

class KalmanFilter : public KalmanFilterBase
{
public:
    VehicleState get_vehicle_state();
    Matrix2d get_vehicle_state_position_covariance();

    void predict(double dt);
    void predict(GyroMeasurement gyro, double dt);
    void handle_lidar_measurements(const std::vector<LidarMeasurement> &meas, const BeaconMap &map);
    void handle_lidar_measurement(LidarMeasurement meas, const BeaconMap &map);
    void handle_gps_measurement(GPSMeasurement meas);
};