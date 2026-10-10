#include "math/kalmanfilter.h"
#include "utils/utils.h"

constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01 / 180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0 / 180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;

void KalmanFilter::handle_lidar_measurements(const std::vector<LidarMeasurement> &dataset, const BeaconMap &map)
{
    for (const auto &meas : dataset)
    {
        handle_lidar_measurement(meas, map);
    }
}

void KalmanFilter::handle_lidar_measurement(LidarMeasurement meas, const BeaconMap &map)
{
    if (is_init())
    {
        VectorXd state = get_state();
        MatrixXd cov = get_covariance();

        BeaconData map_beacon = map.get_beacon_with_id(meas.id);
        if (meas.id != -1 && map_beacon.id != -1)
        {
            VectorXd z = Vector2d::Zero();
            z << meas.range, meas.theta;

            VectorXd z_hat = Vector2d::Zero();
            double delta_x = map_beacon.x - state[0];
            double delta_y = map_beacon.y - state[1];
            double zhat_range = sqrt(delta_x * delta_x + delta_y * delta_y);
            double zhat_theta = wrap_angle(atan2(delta_y, delta_x) - state[2]);
            z_hat << zhat_range, zhat_theta;

            MatrixXd H = MatrixXd(2, 4);
            H << -delta_x / zhat_range, -delta_y / zhat_range, 0, 0, delta_y / zhat_range / zhat_range, -delta_x / zhat_range / zhat_range, -1, 0;

            MatrixXd R = Matrix2d::Zero();
            R(0, 0) = LIDAR_RANGE_STD * LIDAR_RANGE_STD;
            R(1, 1) = LIDAR_THETA_STD * LIDAR_THETA_STD;

            VectorXd y = z - z_hat;
            MatrixXd S = H * cov * H.transpose() + R;
            MatrixXd K = cov * H.transpose() * S.inverse();

            y(1) = wrap_angle(y(1));

            state = state + K * y;
            cov = (Matrix4d::Identity() - K * H) * cov;
        }

        set_state(state);
        set_covariance(cov);
    }
}

void KalmanFilter::predict(GyroMeasurement gyro, double dt)
{
    if (is_init())
    {
        VectorXd state = get_state();
        MatrixXd cov = get_covariance();

        double x = state(0);
        double y = state(1);
        double psi = state(2);
        double v = state(3);
        double x_new = x + dt * v * cos(psi);
        double y_new = y + dt * v * sin(psi);
        double psi_new = wrap_angle(psi + dt * gyro.psi_dot);
        double V_new = v;
        state << x_new, y_new, psi_new, V_new;

        MatrixXd F = Matrix4d::Zero();
        F << 1, 0, -dt * v * sin(psi), dt * cos(psi), 0, 1, dt * v * cos(psi), dt * sin(psi), 0, 0, 1, 0, 0, 0, 0, 1;

        MatrixXd Q = Matrix4d::Zero();
        Q(2, 2) = dt * dt * GYRO_STD * GYRO_STD;
        Q(3, 3) = dt * dt * ACCEL_STD * ACCEL_STD;

        cov = F * cov * F.transpose() + Q;

        set_state(state);
        set_covariance(cov);
    }
}

void KalmanFilter::handle_gps_measurement(GPSMeasurement meas)
{
    if (is_init())
    {
        VectorXd state = get_state();
        MatrixXd cov = get_covariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2, 4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x, meas.y;
        H << 1, 0, 0, 0, 0, 1, 0, 0;
        R(0, 0) = GPS_POS_STD * GPS_POS_STD;
        R(1, 1) = GPS_POS_STD * GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov * H.transpose() * S.inverse();

        state = state + K * y;
        cov = (Matrix4d::Identity() - K * H) * cov;

        set_state(state);
        set_covariance(cov);
    }
    else
    {
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state(0) = meas.x;
        state(1) = meas.y;
        cov(0, 0) = GPS_POS_STD * GPS_POS_STD;
        cov(1, 1) = GPS_POS_STD * GPS_POS_STD;
        cov(2, 2) = INIT_PSI_STD * INIT_PSI_STD;
        cov(3, 3) = INIT_VEL_STD * INIT_VEL_STD;

        set_state(state);
        set_covariance(cov);
    }
}

Matrix2d KalmanFilter::get_vehicle_state_position_covariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = get_covariance();
    if (is_init() && cov.size() != 0)
    {
        pos_cov << cov(0, 0), cov(0, 1), cov(1, 0), cov(1, 1);
    }
    return pos_cov;
}

VehicleState KalmanFilter::get_vehicle_state()
{
    if (is_init())
    {
        VectorXd state = get_state();
        return VehicleState(state[0], state[1], state[2], state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predict(double dt) {}
