#include "math/kalmanfilter.h"
#include "utils/utils.h"

constexpr bool INIT_ON_FIRST_PREDICTION = true;
constexpr double INIT_POS_STD = 0;
constexpr double INIT_VEL_STD = 15;
constexpr double ACCEL_STD = 0.1;
constexpr double GPS_POS_STD = 3.0;

void KalmanFilter::predict(double dt)
{
    if (!is_init() && INIT_ON_FIRST_PREDICTION)
    {
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state << 0, 0, 5.0 * cos(M_PI / 4), 5.0 * sin(M_PI / 4);

        set_state(state);
        set_covariance(cov);
    }

    if (is_init())
    {
        VectorXd state = get_state();
        MatrixXd cov = get_covariance();

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

        set_state(state);
        set_covariance(cov);
    }
    else
    {
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

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
        double psi = std::atan2(state[3], state[2]);
        double v = std::sqrt(state[2] * state[2] + state[3] * state[3]);
        return VehicleState(state[0], state[1], psi, v);
    }
    return VehicleState();
}

void KalmanFilter::predict(GyroMeasurement gyro, double dt) { predict(dt); }
void KalmanFilter::handle_lidar_measurements(const std::vector<LidarMeasurement> &dataset, const BeaconMap &map) {}
void KalmanFilter::handle_lidar_measurement(LidarMeasurement meas, const BeaconMap &map) {}
