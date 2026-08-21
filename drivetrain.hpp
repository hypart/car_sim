#include "deps.hpp"

struct driveTrain {

  float throttle = 0.0f; // throttle command
  float clutch = 0.0f;   // 0 disengaged 1 fully engaged
  bool clutch_locked = false;
  int gear = 0;

  float throttle_min = 0.0f;
  float idle_throttle = 0.08f; // minimum throttle needed to idle
  float throttle_resp =
      0.8f; // how responsive freewheel rpm are to throttle input

  float w = 0.0f; // rotational velocity of engine [rad/s]
  float w_peak = 400.0f;
  float torque_peak = 450.0f;

  std::vector<float> ratios = {-4.0f, 0.0f, 4.0f, 2.5f, 1.5f, 1.0f, 0.75f};
  float final_drive_ratio = 4.0;
  int max_gear = 5;
  int min_gear = -1;

  float I_engine = 0.25f;

  float clutch_capacity = 600.0f; // max transmissible torque [Nm]
  float w_slip_eps = 20.0f;

  float engine_torque() {
    const double thr_command = std::max(throttle, throttle_min);
    const double t_s = throttle_resp * thr_command + 1 - throttle_resp;

    const double b = 2.0 * torque_peak / std::pow(w_peak, 3);
    const double c = -3.0 * torque_peak / std::pow(w_peak, 2);
    const double d = c * c / (4 * b * (1 - idle_throttle));

    const double w_t = std::clamp(w / t_s, -1.5 * w_peak, 5.0 * w_peak);
    float power_torque = -b * std::pow(w_t, 3) - c * std::pow(w_t, 2) +
                         (thr_command - 1) * d * w_t;
    // engine braking with this torque model is very aggressive, soften the
    // curve at negative values
    if (power_torque < 0.0)
      return 0.05 * power_torque;
    return power_torque;
  }

  void update_clutch_lock_state(float car_speed, float wheel_radius) {
    float N = final_drive_ratio * ratios[gear + 1] / wheel_radius;
    float slip = w - car_speed * N;
    if (clutch >= 0.98 && std::abs(slip) < w_slip_eps)
      clutch_locked = true;
    else if (std::abs(engine_torque()) > clutch_capacity || clutch < 0.98)
      clutch_locked = false;
  }

  // TODO add clutch lockup functionality
  float clutch_torque(float car_speed, float wheel_radius) {
    float N = final_drive_ratio * ratios[gear + 1] / wheel_radius;
    float slip = w - car_speed * N;
    if (clutch_locked)
      return engine_torque();

    if (ratios[gear + 1] == 0.0f || clutch <= 0.0f)
      return 0.0f;

    return clutch * clutch_capacity * std::tanh(slip / w_slip_eps);
  }

  float dw_engine_dt(float car_speed, float wheel_radius, float dt) {

    update_clutch_lock_state(car_speed, wheel_radius);

    float N = final_drive_ratio * ratios[gear + 1] / wheel_radius;
    if (clutch_locked)
      return (car_speed * N - w) / dt;

    return (engine_torque() - clutch_torque(car_speed, wheel_radius)) /
           I_engine;
  }

  void start_engine() {
    // start at 1.1 * idle rpm
    w = 1.1 * 0.75 * w_peak *
        (throttle_resp * std::max(throttle, throttle_min) + 1 - throttle_resp);
    throttle_min = 1.01 * idle_throttle;
  }

  void kill_engine() { throttle_min = 0.0; }

  void
  configure_engine(float _peak_torque,   // maximum torque [Nm]
                   float _peak_rpm,      // rpm at max torque [min^-1]
                   float _idle_throttle, // throttle needed to idle [0.0-1.0]
                   float _throttle_response, // how responsive the freewheel rpm
                                             // should be to throttle [0.0-1.0]
                   float _inertia_engine     // inertia of the engine [Nm^2]
  ) {
    idle_throttle = _idle_throttle;
    throttle_resp = _throttle_response;
    w_peak = _peak_rpm / 60 * 2 * pi;
    torque_peak = _peak_torque;
    I_engine = _inertia_engine;
  }

  void configure_transmission(std::vector<float> _gear_ratios,
                              int _neutral_gear_idx, float _final_drive_ratio,
                              float _clutch_max_torque,
                              float _clutch_slip_transition) {

    ratios = _gear_ratios;
    final_drive_ratio = _final_drive_ratio;
    max_gear = ratios.size() - 1 - _neutral_gear_idx;
    min_gear = -_neutral_gear_idx;

    clutch_capacity = _clutch_max_torque;
    w_slip_eps = _clutch_slip_transition;
  }
};
