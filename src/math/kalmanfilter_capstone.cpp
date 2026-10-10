#include "math/kalmanfilter.h"
#include "utils/utils.h"

constexpr double ACCEL_STD = 1.0;
constexpr double BIAS_STD = 1e-3;
constexpr double GYRO_STD = 0.01 / 180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0 / 180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;

VectorXd normalize_state(VectorXd state)
{
    state(2) = wrap_angle(state(2));
    return state;
}

VectorXd normalize_lidar_measurement(VectorXd meas)
{
    meas(1) = wrap_angle(meas(1));
    return meas;
}

std::vector<VectorXd> generate_sigma_points(VectorXd state, MatrixXd cov)
{
    std::vector<VectorXd> sigma_points;
    int num_states = state.size();
    double lambda = 3.0 - num_states;
    MatrixXd sqrt_cov = cov.llt().matrixL();
    sigma_points.push_back(state);
    for (int i_state = 0; i_state < num_states; i_state++)
    {
        sigma_points.push_back(state + sqrt(lambda + num_states) * sqrt_cov.col(i_state));
        sigma_points.push_back(state - sqrt(lambda + num_states) * sqrt_cov.col(i_state));
    }

    return sigma_points;
}

std::vector<double> generate_sigma_weights(unsigned int num_states)
{
    std::vector<double> weights;

    double lambda = 3.0 - num_states;
    double w0 = lambda / (lambda + num_states);
    double wi = 0.5 / (num_states + lambda);
    weights.push_back(w0);
    for (int i = 0; i < 2 * num_states; ++i)
    {
        weights.push_back(wi);
    }

    return weights;
}

VectorXd lidar_measurement_model(VectorXd aug_state, double beaconX, double beaconY)
{
    VectorXd z_hat = VectorXd::Zero(2);

    double x = aug_state(0);
    double y = aug_state(1);
    double psi = aug_state(2);
    double range_noise = aug_state(5);
    double theta_noise = aug_state(6);

    double delta_x = beaconX - x;
    double delta_y = beaconY - y;
    double zhat_range = sqrt(delta_x * delta_x + delta_y * delta_y) + range_noise;
    double zhat_theta = atan2(delta_y, delta_x) - aug_state[2] + theta_noise;
    z_hat << zhat_range, zhat_theta;

    return z_hat;
}

VectorXd vehicle_process_model(VectorXd aug_state, double psi_dot, double dt)
{
    VectorXd new_state = VectorXd::Zero(5);

    double x = aug_state(0);
    double y = aug_state(1);
    double psi = aug_state(2);
    double v = aug_state(3);
    double bias = aug_state(4);
    double psi_dot_noise = aug_state(5);
    double accel_noise = aug_state(6);

    double x_new = x + dt * v * cos(psi);
    double y_new = y + dt * v * sin(psi);
    double psi_new = psi + dt * (psi_dot + psi_dot_noise - bias);
    double v_new = v + dt * accel_noise;
    double bias_new = bias;
    new_state << x_new, y_new, psi_new, v_new, bias_new;

    return new_state;
}

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

            MatrixXd R = Matrix2d::Zero();
            R(0, 0) = LIDAR_RANGE_STD * LIDAR_RANGE_STD;
            R(1, 1) = LIDAR_THETA_STD * LIDAR_THETA_STD;

            int n_x = state.size();
            int n_v = 2;
            int n_z = 2;
            int n_aug = n_x + n_v;
            VectorXd x_aug = VectorXd::Zero(n_aug);
            MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
            x_aug.head(n_x) = state;
            P_aug.topLeftCorner(n_x, n_x) = cov;
            P_aug.bottomRightCorner(n_v, n_v) = R;
            std::vector<VectorXd> sigma_points = generate_sigma_points(x_aug, P_aug);
            std::vector<double> sigma_weights = generate_sigma_weights(n_aug);

            std::vector<VectorXd> z_sig;
            for (const auto &sigma_point : sigma_points)
            {
                z_sig.push_back(lidar_measurement_model(sigma_point, map_beacon.x, map_beacon.y));
            }

            VectorXd z_mean = VectorXd::Zero(n_z);
            for (unsigned int i = 0; i < z_sig.size(); ++i)
            {
                z_mean += sigma_weights[i] * z_sig[i];
            }

            MatrixXd Py = MatrixXd::Zero(n_z, n_z);
            for (unsigned int i = 0; i < z_sig.size(); ++i)
            {
                VectorXd diff = normalize_lidar_measurement(z_sig[i] - z_mean);
                Py += sigma_weights[i] * diff * diff.transpose();
            }

            MatrixXd Pxy = MatrixXd::Zero(n_x, n_z);
            for (unsigned int i = 0; i < 2 * n_x + 1; ++i)
            {
                VectorXd x_diff = normalize_state(sigma_points[i].head(n_x) - state);
                VectorXd z_diff = normalize_lidar_measurement(z_sig[i] - z_mean);
                Pxy += sigma_weights[i] * x_diff * z_diff.transpose();
            }

            MatrixXd K = Pxy * Py.inverse();
            VectorXd y = normalize_lidar_measurement(z - z_mean);
            state = state + K * y;
            cov = cov - K * Py * K.transpose();
        }
        else
        {
            if (meas.id != -1 && map_beacon.id != -1)
                state(2) = meas.theta;
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

        MatrixXd Q = MatrixXd::Zero(3, 3);
        Q(0, 0) = GYRO_STD * GYRO_STD;
        Q(1, 1) = ACCEL_STD * ACCEL_STD;
        Q(2, 2) = BIAS_STD * BIAS_STD;

        int n_x = state.size();
        int n_w = 3;
        int n_aug = n_x + n_w;
        VectorXd x_aug = VectorXd::Zero(n_aug);
        MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
        x_aug.head(n_x) = state;
        P_aug.topLeftCorner(n_x, n_x) = cov;
        P_aug.bottomRightCorner(n_w, n_w) = Q;

        std::vector<VectorXd> sigma_points = generate_sigma_points(x_aug, P_aug);
        std::vector<double> sigma_weights = generate_sigma_weights(n_aug);

        std::vector<VectorXd> sigma_points_predict;
        for (const auto &sigma_point : sigma_points)
        {
            sigma_points_predict.push_back(vehicle_process_model(sigma_point, gyro.psi_dot, dt));
        }

        state = VectorXd::Zero(n_x);
        for (unsigned int i = 0; i < sigma_points_predict.size(); ++i)
        {
            state += sigma_weights[i] * sigma_points_predict[i];
        }
        state = normalize_state(state);

        cov = MatrixXd::Zero(n_x, n_x);
        for (unsigned int i = 0; i < sigma_points_predict.size(); ++i)
        {
            VectorXd diff = normalize_state(sigma_points_predict[i] - state);
            cov += sigma_weights[i] * diff * diff.transpose();
        }

        set_state(state);
        set_covariance(cov);
    }
}

void KalmanFilter::handle_gps_measurement(GPSMeasurement meas)
{
    if (std::isnan(meas.x) || std::isnan(meas.y))
    {
        return;
    }

    if (is_init())
    {
        VectorXd state = get_state();
        MatrixXd cov = get_covariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2, 5);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x, meas.y;
        H << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        R(0, 0) = GPS_POS_STD * GPS_POS_STD;
        R(1, 1) = GPS_POS_STD * GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;

        double mahalanobis = y.transpose() * S.inverse() * y;

        //  95% : 5.991
        //  99% : 9.210
        //  99.9% : 13.816
        const double GATING_THRESHOLD = 9.210;

        if (mahalanobis > GATING_THRESHOLD)
        {
            return;
        }

        MatrixXd K = cov * H.transpose() * S.inverse();

        state = state + K * y;
        cov = (MatrixXd::Identity(5, 5) - K * H) * cov;

        set_state(state);
        set_covariance(cov);
    }
    else
    {
        VectorXd state = VectorXd::Zero(5);
        MatrixXd cov = MatrixXd::Zero(5, 5);

        state(0) = meas.x;
        state(1) = meas.y;
        cov(0, 0) = GPS_POS_STD * GPS_POS_STD;
        cov(1, 1) = GPS_POS_STD * GPS_POS_STD;
        cov(2, 2) = INIT_PSI_STD * INIT_PSI_STD;
        cov(3, 3) = INIT_VEL_STD * INIT_VEL_STD;
        cov(4, 4) = 0.1 * 0.1;

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