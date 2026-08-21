#include "car.hpp"

float carBase::brake_per_wheel() {
    return brake * 10.0f / 4.0f;
}

float carBase::drag_force() {
    return d * vl * std::abs(vl);
}

float carBase::brake_force() {
    float v_sign = 0.0;
    if (std::abs(vl) > vel_eps)
        v_sign = vl / std::abs(vl);

    return brake_per_wheel() * (2 + std::cos(tr) + std::cos(tl));
}

float carBase::dvl_dt() {
    return accel() - drag_force() - brake_force();
}

std::tuple<float, float> carBase::wheel_rots(float curr_ts) {
    float tr;
    float tl;
    if (std::abs(curr_ts) < steer_eps) {
        tr = 0.0;
        tl = 0.0;
    } else {
        float ti =
            pi / 2 - std::atan(1 / std::tan(std::abs(curr_ts)) - w / (2 * l)); // inside wheel angle
        float to = pi / 2 -
                   std::atan(1 / std::tan(std::abs(curr_ts)) + w / (2 * l)); // outside wheel angle
        if (curr_ts > 0.0) {
            tr = to;
            tl = ti;
        } else {
            tr = -ti;
            tl = -to;
        }
    }
    return {tr, tl};
}

void carBase::update_control(float throttle_command, float brake_command, float steer_command,
                             int gear_command, float clutch_command) {
    ts = steer_command;
    std::tie(tr, tl) = wheel_rots(ts);

    drivetrain.throttle = throttle_command;
    brake = brake_command;

    drivetrain.gear = gear_command;
    drivetrain.clutch = clutch_command;
}

float carBase::accel() { // longitudonal acceleration [m/s^2]
    float N = drivetrain.final_drive_ratio * drivetrain.ratios[drivetrain.gear + 1];
    float wheel_ratio = N / wheel_radius;
    float m_eff = m + I_wheels / (wheel_radius * wheel_radius);

    float torque = drivetrain.clutch_torque(vl, wheel_radius) * wheel_ratio / m_eff;
    return torque;
}

float carBase::lag_alpha(float tau) {
    if (tau <= 0.0f)
        return 1.0f; // immediate reaction
    return 1.0f - std::exp(-dt / tau);
}

carBase::carBase(float timestep, float disp_fps, float length, float width, float mass, float drag,
                 Eigen::Vector3f init_pos) {
    dt = timestep;

    l = length;
    w = width;
    m = mass;
    d = drag;

    wheel_positions = {{l / 2, w / 2, 0.0},    // front right
                       {l / 2, -w / 2, 0.0},   // front left
                       {-l / 2, w / 2, 0.0},   // back right
                       {-l / 2, -w / 2, 0.0}}; // back left

    wheel_width = l / 10;
    wheel_radius = l / 10;

    com_pos = init_pos;

    I_wheels = 0.5f * wheel_radius * wheel_radius * m * 0.01;

    sound = carSoundEmitter(44100, disp_fps);
}

void carBase::input_control(float throttle_command, float brake_command, float steer_command,
                            int gear_command, float clutch_command, int starter_command) {

    // time constants [s] (0: immediate reaction)
    float thr_tau = 0.1f;
    float str_tau = 0.5f;
    float brk_tau = 0.1f;
    float clt_tau = 0.1f;

    float thr_a = lag_alpha(thr_tau);
    float brk_a = lag_alpha(brk_tau);
    float str_a = lag_alpha(str_tau);
    float clt_a = lag_alpha(clt_tau);

    float thr_smooth = drivetrain.throttle + thr_a * (throttle_command - drivetrain.throttle);
    float brk_smooth = brake + brk_a * (brake_command - brake);
    float str_smooth = ts + str_a * (steer_command - ts);
    float clt_smooth = drivetrain.clutch + clt_a * (clutch_command - drivetrain.clutch);

    update_control(thr_smooth, brk_smooth, str_smooth, gear_command, clt_smooth);

    if (starter_command == 1)
        drivetrain.start_engine();
    else if (starter_command == -1)
        drivetrain.kill_engine();
}

void carBase::update_state() {
    drivetrain.w += dt * drivetrain.dw_engine_dt(vl, wheel_radius, dt);
    vl += dt * dvl_dt();
    float vt = 0.0;

    if (std::abs(ts) > steer_eps) {
        float rn = l / tan(ts);
        float dtc = vl / rn;
        tc += dtc * dt;
        vt = dtc * l / 2;
    }

    com_pos += Eigen::Vector3f(vl * std::cos(tc) - vt * std::sin(tc),
                               vl * std::sin(tc) + vt * std::cos(tc), 0.0) *
               dt;
}

void carBase::draw() {

    // wheels
    int wheel_idx = 0;
    for (Eigen::Vector3f p : wheel_positions) {

        Eigen::Vector3f start(0.0f, -wheel_width / 2, 0.0f);
        Eigen::Vector3f end(0.0, wheel_width / 2, 0.0f);

        float angle_base = tc;
        float angle_wheel = 0.0f;
        if (wheel_idx == 0)
            angle_wheel = tr;
        if (wheel_idx == 1)
            angle_wheel = tl;
        Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);

        Eigen::Vector3f start_trans =
            com_pos + Eigen::AngleAxisf(angle_base, axis) *
                          (p + Eigen::AngleAxisf(angle_wheel, axis) * start);
        Eigen::Vector3f end_trans = com_pos + Eigen::AngleAxisf(angle_base, axis) *
                                                  (p + Eigen::AngleAxisf(angle_wheel, axis) * end);

        Vector3 start_v = {start_trans.x(), start_trans.z(), start_trans.y()};
        Vector3 end_v = {end_trans.x(), end_trans.z(), end_trans.y()};

        DrawCylinderWiresEx(start_v, end_v, wheel_radius, wheel_radius, 100, WHITE);
        wheel_idx++;
    }
}

short carBase::play_sound() {
    return sound.emit(drivetrain.throttle, drivetrain.w);
}
