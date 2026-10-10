
#include "simulation/simulation.h"
#include "utils/utils.h"

Simulation::Simulation()
    : m_sim_parameters(SimulationParams()),
      m_is_paused(false),
      m_is_running(false),
      m_time_multiplier(1),
      m_view_size(100),
      m_time(0.0),
      m_time_till_gyro_measurement(0.0),
      m_time_till_gps_measurement(0.0),
      m_time_till_lidar_measurement(0.0),
      m_logging_enabled(true),
      m_log_file_path("simulation_log.csv")
{
}

void Simulation::reset()
{
    m_time = 0.0;
    m_time_till_gyro_measurement = 0.0;
    m_time_till_gps_measurement = 0.0;
    m_time_till_lidar_measurement = 0.0;

    m_is_running = true;
    m_is_paused = false;

    m_kalman_filter.reset();

    m_gps_sensor.reset();
    m_gps_sensor.set_gps_noise_std(m_sim_parameters.gps_position_noise_std);
    m_gps_sensor.set_gps_error_prob(m_sim_parameters.gps_error_probability);
    m_gps_sensor.set_gps_denied_zone(m_sim_parameters.gps_denied_x, m_sim_parameters.gps_denied_y, m_sim_parameters.gps_denied_range);

    m_gyro_sensor.reset();
    m_gyro_sensor.set_gyro_noise_std(m_sim_parameters.gyro_noise_std);
    m_gyro_sensor.set_gyro_bias(m_sim_parameters.gyro_bias);

    m_lidar_sensor.reset();
    m_lidar_sensor.set_lidar_noise_std(m_sim_parameters.lidar_range_noise_std, m_sim_parameters.lidar_theta_noise_std);
    m_lidar_sensor.set_lidar_da_enabled(m_sim_parameters.lidar_id_enabled);

    m_car.reset(m_sim_parameters.car_initial_x, m_sim_parameters.car_initial_y, m_sim_parameters.car_initial_psi, m_sim_parameters.car_initial_velocity);

    for (auto &cmd : m_sim_parameters.car_commands)
    {
        m_car.add_vehicle_command(cmd.get());
    }
    m_gps_measurement_history.clear();
    m_lidar_measurement_history.clear();
    m_vehicle_position_history.clear();
    m_filter_position_history.clear();
    m_filter_error_x_position_history.clear();
    m_filter_error_y_position_history.clear();
    m_filter_error_heading_history.clear();
    m_filter_error_velocity_history.clear();

    close_log_file();
    open_log_file();

    std::cout << "Simulation: Reset" << std::endl;
}

void Simulation::update()
{
    if (m_is_running && !m_is_paused)
    {
        for (unsigned i = 0; i < m_time_multiplier; ++i)
        {
            if (m_time >= m_sim_parameters.end_time)
            {
                m_is_running = false;
                std::cout << "Simulation: Reached End of Simulation Time (" << m_time << ")" << std::endl;
                return;
            }

            m_car.update(m_time, m_sim_parameters.time_step);
            m_vehicle_position_history.push_back(Vector2(m_car.get_vehicle_state().x, m_car.get_vehicle_state().y));

            if (m_sim_parameters.gyro_enabled)
            {
                if (m_time_till_gyro_measurement <= 0)
                {
                    GyroMeasurement meas = m_gyro_sensor.generate_gyro_measurement(m_car.get_vehicle_state().yaw_rate);
                    m_kalman_filter.predict(meas, m_sim_parameters.time_step);
                    m_time_till_gyro_measurement += 1.0 / m_sim_parameters.gyro_update_rate;
                }
                m_time_till_gyro_measurement -= m_sim_parameters.time_step;
            }

            if (m_sim_parameters.gps_enabled)
            {
                if (m_time_till_gps_measurement <= 0)
                {
                    GPSMeasurement gps_meas = m_gps_sensor.generate_gps_measurement(m_car.get_vehicle_state().x, m_car.get_vehicle_state().y);
                    m_kalman_filter.handle_gps_measurement(gps_meas);
                    m_gps_measurement_history.push_back(gps_meas);
                    m_time_till_gps_measurement += 1.0 / m_sim_parameters.gps_update_rate;
                }
                m_time_till_gps_measurement -= m_sim_parameters.time_step;
            }
            if (m_sim_parameters.lidar_enabled)
            {
                if (m_time_till_lidar_measurement <= 0)
                {
                    std::vector<LidarMeasurement> lidar_measurements = m_lidar_sensor.generate_lidar_measurements(m_car.get_vehicle_state().x, m_car.get_vehicle_state().y, m_car.get_vehicle_state().psi, m_beacons);
                    m_kalman_filter.handle_lidar_measurements(lidar_measurements, m_beacons);
                    m_lidar_measurement_history = lidar_measurements;
                    m_time_till_lidar_measurement += 1.0 / m_sim_parameters.lidar_update_rate;
                }
                m_time_till_lidar_measurement -= m_sim_parameters.time_step;
            }

            if (m_kalman_filter.is_init())
            {
                VehicleState vehicle_state = m_car.get_vehicle_state();
                VehicleState filter_state = m_kalman_filter.get_vehicle_state();
                m_filter_position_history.push_back(Vector2(filter_state.x, filter_state.y));
                m_filter_error_x_position_history.push_back(filter_state.x - vehicle_state.x);
                m_filter_error_y_position_history.push_back(filter_state.y - vehicle_state.y);
                m_filter_error_heading_history.push_back(wrap_angle(filter_state.psi - vehicle_state.psi));
                m_filter_error_velocity_history.push_back(filter_state.v - vehicle_state.v);
            }

            m_time += m_sim_parameters.time_step;
            log_state();
        }
    }
}

void Simulation::render(Display &disp)
{
    std::vector<Vector2> marker_lines1 = {{0.5, 0.5}, {-0.5, -0.5}};
    std::vector<Vector2> marker_lines2 = {{0.5, -0.5}, {-0.5, 0.5}};

    disp.set_view(m_view_size * disp.get_screen_aspect_ratio(), m_view_size, m_car.get_vehicle_state().x, m_car.get_vehicle_state().y);

    m_car.render(disp);
    m_beacons.render(disp);

    disp.set_draw_color(0, 100, 0);
    disp.draw_lines(m_vehicle_position_history);

    disp.set_draw_color(100, 0, 0);
    disp.draw_lines(m_filter_position_history);

    if (m_kalman_filter.is_init())
    {
        VehicleState filter_state = m_kalman_filter.get_vehicle_state();
        Eigen::Matrix2d cov = m_kalman_filter.get_vehicle_state_position_covariance();

        double x = filter_state.x;
        double y = filter_state.y;
        double sigma_xx = cov(0, 0);
        double sigma_yy = cov(1, 1);
        double sigma_xy = cov(0, 1);

        std::vector<Vector2> marker_lines1_world = offset_points(marker_lines1, Vector2(x, y));
        std::vector<Vector2> marker_lines2_world = offset_points(marker_lines2, Vector2(x, y));
        disp.set_draw_color(255, 0, 0);
        disp.draw_lines(marker_lines1_world);
        disp.draw_lines(marker_lines2_world);

        std::vector<Vector2> cov_world = generate_ellipse(x, y, sigma_xx, sigma_yy, sigma_xy);
        disp.set_draw_color(255, 0, 0);
        disp.draw_lines(cov_world);
    }

    std::vector<std::vector<Vector2>> m_gps_marker = {{{0.5, 0.5}, {-0.5, -0.5}}, {{0.5, -0.5}, {-0.5, 0.5}}};
    disp.set_draw_color(255, 255, 255);
    for (const auto &meas : m_gps_measurement_history)
    {
        disp.draw_lines(offset_points(m_gps_marker, Vector2(meas.x, meas.y)));
    }

    if (m_sim_parameters.gps_denied_range > 0)
    {
        std::vector<Vector2> zone_lines = generate_circle(m_sim_parameters.gps_denied_x, m_sim_parameters.gps_denied_y, m_sim_parameters.gps_denied_range);
        disp.set_draw_color(255, 150, 0);
        disp.draw_lines(zone_lines);
    }

    for (const auto &meas : m_lidar_measurement_history)
    {
        double x0 = m_car.get_vehicle_state().x;
        double y0 = m_car.get_vehicle_state().y;
        double delta_x = meas.range * cos(meas.theta + m_car.get_vehicle_state().psi);
        double delta_y = meas.range * sin(meas.theta + m_car.get_vehicle_state().psi);
        disp.set_draw_color(201, 201, 0);
        disp.draw_line(Vector2(x0, y0), Vector2(x0 + delta_x, y0 + delta_y));
    }

    int x_offset, y_offset;
    int stride = 20;
    x_offset = 10;
    y_offset = 30;
    std::string time_string = string_format("Time: %0.2f (x%d)", m_time, m_time_multiplier);
    std::string profile_string = string_format("Profile: %s", m_sim_parameters.profile_name.c_str());
    std::string gps_string = string_format("GPS: %s (%0.1f Hz)", (m_sim_parameters.gps_enabled ? "ON" : "OFF"), m_sim_parameters.gps_update_rate);
    std::string lidar_string = string_format("LIDAR: %s (%0.1f Hz)", (m_sim_parameters.lidar_enabled ? "ON" : "OFF"), m_sim_parameters.lidar_update_rate);
    std::string gyro_string = string_format("GYRO: %s (%0.1f Hz)", (m_sim_parameters.gyro_enabled ? "ON" : "OFF"), m_sim_parameters.gyro_update_rate);
    disp.draw_text_main_font(profile_string, Vector2(x_offset, y_offset + stride * -1), 1.0, {255, 255, 255});
    disp.draw_text_main_font(time_string, Vector2(x_offset, y_offset + stride * 0), 1.0, {255, 255, 255});
    disp.draw_text_main_font(gps_string, Vector2(x_offset, y_offset + stride * 1), 1.0, {255, 255, 255});
    disp.draw_text_main_font(lidar_string, Vector2(x_offset, y_offset + stride * 2), 1.0, {255, 255, 255});
    disp.draw_text_main_font(gyro_string, Vector2(x_offset, y_offset + stride * 3), 1.0, {255, 255, 255});
    if (m_is_paused)
    {
        disp.draw_text_main_font("PAUSED", Vector2(x_offset, y_offset + stride * 4), 1.0, {255, 0, 0});
    }
    if (!m_is_running)
    {
        disp.draw_text_main_font("FINISHED", Vector2(x_offset, y_offset + stride * 5), 1.0, {255, 0, 0});
    }

    x_offset = 800;
    y_offset = 10;
    std::string velocity_string = string_format("    Velocity: %0.2f m/s", m_car.get_vehicle_state().v);
    std::string yaw_string = string_format("   Heading: %0.2f deg", m_car.get_vehicle_state().psi * 180.0 / M_PI);
    std::string xpos = string_format("X Position: %0.2f m", m_car.get_vehicle_state().x);
    std::string ypos = string_format("Y Position: %0.2f m", m_car.get_vehicle_state().y);
    disp.draw_text_main_font("Vehicle State", Vector2(x_offset - 5, y_offset + stride * 0), 1.0, {255, 255, 255});
    disp.draw_text_main_font(velocity_string, Vector2(x_offset, y_offset + stride * 1), 1.0, {255, 255, 255});
    disp.draw_text_main_font(yaw_string, Vector2(x_offset, y_offset + stride * 2), 1.0, {255, 255, 255});
    disp.draw_text_main_font(xpos, Vector2(x_offset, y_offset + stride * 3), 1.0, {255, 255, 255});
    disp.draw_text_main_font(ypos, Vector2(x_offset, y_offset + stride * 4), 1.0, {255, 255, 255});

    std::string kf_velocity_string = string_format("    Velocity: %0.2f m/s", m_kalman_filter.get_vehicle_state().v);
    std::string kf_yaw_string = string_format("   Heading: %0.2f deg", m_kalman_filter.get_vehicle_state().psi * 180.0 / M_PI);
    std::string kf_xpos = string_format("X Position: %0.2f m", m_kalman_filter.get_vehicle_state().x);
    std::string kf_ypos = string_format("Y Position: %0.2f m", m_kalman_filter.get_vehicle_state().y);
    disp.draw_text_main_font("Filter State", Vector2(x_offset, y_offset + stride * 6), 1.0, {255, 255, 255});
    disp.draw_text_main_font(kf_velocity_string, Vector2(x_offset, y_offset + stride * 7), 1.0, {255, 255, 255});
    disp.draw_text_main_font(kf_yaw_string, Vector2(x_offset, y_offset + stride * 8), 1.0, {255, 255, 255});
    disp.draw_text_main_font(kf_xpos, Vector2(x_offset, y_offset + stride * 9), 1.0, {255, 255, 255});
    disp.draw_text_main_font(kf_ypos, Vector2(x_offset, y_offset + stride * 10), 1.0, {255, 255, 255});

    x_offset = 10;
    y_offset = 650;
    disp.draw_text_main_font("Reset Key: r", Vector2(x_offset, y_offset + stride * 0), 1.0, {255, 255, 255});
    disp.draw_text_main_font("Pause Key: [space bar]", Vector2(x_offset, y_offset + stride * 1), 1.0, {255, 255, 255});
    disp.draw_text_main_font("Speed Multiplier (+/-) Key: [ / ] ", Vector2(x_offset, y_offset + stride * 2), 1.0, {255, 255, 255});
    disp.draw_text_main_font("Zoom (+/-) Key: + / - (keypad)", Vector2(x_offset, y_offset + stride * 3), 1.0, {255, 255, 255});
    disp.draw_text_main_font("Motion Profile Key: 1 - 9,0", Vector2(x_offset, y_offset + stride * 4), 1.0, {255, 255, 255});

    x_offset = 750;
    y_offset = 650;
    std::string xpos_error_string = string_format("X Position RMSE: %0.2f m", calculate_rmse(m_filter_error_x_position_history));
    std::string ypos_error_string = string_format("Y Position RMSE: %0.2f m", calculate_rmse(m_filter_error_y_position_history));
    std::string heading_error_string = string_format("   Heading RMSE: %0.2f deg", 180.0 / M_PI * calculate_rmse(m_filter_error_heading_history));
    std::string velocity_error_string = string_format("    Velocity RMSE: %0.2f m/s", calculate_rmse(m_filter_error_velocity_history));
    disp.draw_text_main_font(xpos_error_string, Vector2(x_offset, y_offset + stride * 0), 1.0, {255, 255, 255});
    disp.draw_text_main_font(ypos_error_string, Vector2(x_offset, y_offset + stride * 1), 1.0, {255, 255, 255});
    disp.draw_text_main_font(heading_error_string, Vector2(x_offset, y_offset + stride * 2), 1.0, {255, 255, 255});
    disp.draw_text_main_font(velocity_error_string, Vector2(x_offset, y_offset + stride * 3), 1.0, {255, 255, 255});
}

void Simulation::reset(SimulationParams sim_params)
{
    m_sim_parameters = sim_params;
    reset();
}
void Simulation::increase_time_multiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier << ")" << std::endl;
}
void Simulation::decrease_time_multiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")" << std::endl;
    }
}
void Simulation::set_time_multiplier(unsigned int multiplier) { m_time_multiplier = static_cast<int>(multiplier); }
void Simulation::increase_zoom()
{
    if (m_view_size > 25)
    {
        m_view_size -= 25;
    }
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}
void Simulation::decrease_zoom()
{
    if (m_view_size < 400)
    {
        m_view_size += 25;
    }
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}
void Simulation::toggle_pause_simulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused ? "True" : "False") << ")" << std::endl;
}
bool Simulation::is_paused() { return m_is_paused; }
bool Simulation::is_running() { return m_is_running; }
void Simulation::open_log_file()
{
    if (m_logging_enabled && m_log_file.is_open())
    {
        m_log_file.close();
    }

    m_log_file.open(m_log_file_path, std::ios::out | std::ios::trunc);
    if (!m_log_file.is_open())
    {
        std::cerr << "Simulation: Failed to open log file: " << m_log_file_path << std::endl;
        m_logging_enabled = false;
        return;
    }

    m_logging_enabled = true;
    write_log_header();
    std::cout << "Simulation: Logging to " << m_log_file_path << std::endl;
}

void Simulation::close_log_file()
{
    if (m_log_file.is_open())
    {
        m_log_file.flush();
        m_log_file.close();
        std::cout << "Simulation: Log file closed" << std::endl;
    }
    m_logging_enabled = false;
}
void Simulation::log_state()
{
    if (!m_logging_enabled || !m_log_file.is_open())
        return;

    const VehicleState veh = m_car.get_vehicle_state();
    const VehicleState kf = m_kalman_filter.get_vehicle_state();

    const double rmse_x = calculate_rmse(m_filter_error_x_position_history);
    const double rmse_y = calculate_rmse(m_filter_error_y_position_history);
    const double rmse_psi = 180.0 / M_PI * calculate_rmse(m_filter_error_heading_history);
    const double rmse_v = calculate_rmse(m_filter_error_velocity_history);

    m_log_file << m_time << ","
               << m_time_multiplier << ","
               << veh.x << "," << veh.y << ","
               << (veh.psi * 180.0 / M_PI) << ","
               << veh.v << ","
               << kf.x << "," << kf.y << ","
               << (kf.psi * 180.0 / M_PI) << ","
               << kf.v << ","
               << rmse_x << "," << rmse_y << ","
               << rmse_psi << "," << rmse_v << ","
               << (m_sim_parameters.gps_enabled ? 1 : 0) << ","
               << m_sim_parameters.gps_update_rate << ","
               << (m_sim_parameters.lidar_enabled ? 1 : 0) << ","
               << m_sim_parameters.lidar_update_rate << ","
               << (m_sim_parameters.gyro_enabled ? 1 : 0) << ","
               << m_sim_parameters.gyro_update_rate << ","
               << m_sim_parameters.profile_name
               << "\n";
}

void Simulation::write_log_header()
{
    if (!m_log_file.is_open())
        return;

    m_log_file << "time,"
               << "time_multiplier,"
               << "veh_x,veh_y,veh_psi_deg,veh_v,"
               << "kf_x,kf_y,kf_psi_deg,kf_v,"
               << "rmse_x,rmse_y,rmse_psi_deg,rmse_v,"
               << "gps_enabled,gps_rate,"
               << "lidar_enabled,lidar_rate,"
               << "gyro_enabled,gyro_rate,"
               << "profile\n";
}