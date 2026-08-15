#include "deps.hpp"

struct driveTrain{

    float throttle = 0.0f; //throttle command
    float clutch = 0.0f; //0 disengaged 1 fully engaged
    int gear = 0;

    float throttle_min = 0.0f;
    float idle_throttle = 0.08f; // minimum throttle needed to idle
    float throttle_resp = 0.8f; // how responsive freewheel rpm are to throttle input

    float w = 0.0f; //rotational velocity of engine [rad/s]
    float w_peak = 400.0f;
    float torque_peak = 450.0f;

    std::vector<float> ratios = {-4.0f, 0.0f, 4.0f, 2.5f, 1.5f, 1.0f, 0.75f};
    float final_drive_ratio = 4.0;

    float I_engine = 0.25f;

    float clutch_capacity = 600.0f; //max transmissible torque [Nm]
    float w_slip_eps = 20.0f;

    driveTrain(){
        ratios = {-4.0f, 0.0f, 4.0f, 2.5f, 1.5f, 1.0f, 0.75f};
        final_drive_ratio = 4.0;

        I_engine = 0.25f;

        clutch_capacity = 600.0f; //max transmissible torque [Nm]
        w_slip_eps = 20.0f;
    }
    
    float engine_torque(){
        const double thr_command = std::max(throttle, throttle_min);
        const double t_s = throttle_resp * thr_command + 1 - throttle_resp;

        const double b = 2.0 * torque_peak / std::pow(w_peak, 3);
        const double c = -3.0 * torque_peak / std::pow(w_peak, 2);
        const double d = c*c / (4*b*(1-idle_throttle));

        const double w_t = std::clamp(w / t_s, 0.0, 1.5*w_peak);
        return -b*std::pow(w_t, 3) - c*std::pow(w_t, 2) + (thr_command - 1)*d*w_t;
    }

    float clutch_torque(float car_speed, float wheel_radius){
        if (ratios[gear+1] == 0.0f || clutch <= 0.0f) return 0.0f;

        float N = final_drive_ratio * ratios[gear+1] / wheel_radius;
        float slip = w - car_speed * N;
        return clutch * clutch_capacity * std::tanh(slip / w_slip_eps);
    }

    float dw_engine_dt(float car_speed, float wheel_radius){
        return (engine_torque() - clutch_torque(car_speed, wheel_radius))/I_engine;
    }

    void start_engine(){
        // start at 1.1 * idle rpm
        w = 1.1 * 0.75 * w_peak * (throttle_resp * std::max(throttle, throttle_min) + 1 - throttle_resp);
        throttle_min = 1.01*idle_throttle;
    }

    void kill_engine(){
        throttle_min = 0.0;
    }

    float accel(float car_mass, float I_wheels, float car_speed, float wheel_radius){ //longitudonal acceleration [m/s^2]
        float N = final_drive_ratio * ratios[gear+1];
        float wheel_ratio = N/wheel_radius;
        float m_eff = car_mass + I_wheels/(wheel_radius*wheel_radius);

        return clutch_torque(car_speed, wheel_radius) * wheel_ratio / m_eff;
    }
};