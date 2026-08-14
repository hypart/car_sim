#include "deps.hpp"

struct driveTrain{

    float throttle = 0.0f; //throttle command
    float clutch = 0.0f; //0 disengaged 1 fully engaged
    int gear = 0;

    float max_torque;
    float throttle_min = 0.0f;
    float idle_throttle = 0.09f; //throttle needed to hold w_idle against drag

    float w = 0.0f; //rotational velocity of engine [rad/s]
    float w_max = 750.0f;
    float w_peak = 524.0f;
    float t_peak = 440.0f;
    float w_idle = 10.0f;

    std::vector<float> ratios = {-4.0f, 0.0f, 4.0f, 2.5f, 1.5f, 1.0f, 0.75f};
    float final_drive_ratio = 4.0;

    float I_engine = 0.25f;

    float clutch_capacity = 600.0f; //max transmissible torque [Nm]
    float w_slip_eps = 20.0f;

    driveTrain(){

        max_torque = 400.0f;

        ratios = {-4.0f, 0.0f, 4.0f, 2.5f, 1.5f, 1.0f, 0.75f};
        final_drive_ratio = 4.0;

        I_engine = 0.25f;

        clutch_capacity = 600.0f; //max transmissible torque [Nm]
        w_slip_eps = 20.0f;
    }
    // TODO: find better torque curve (maybe from dataset)
    float engine_torque(){
        const double x  = w / w_max;
        const double xp = w_peak / w_max;
        const double d  = (x - xp) / (1.0 - xp);
        const double shape = 1.0 - 0.2 * d * d;
        const double drag  = 0.15 * t_peak * (0.25 + 0.75 * x * x);
        return std::max(throttle, throttle_min) * t_peak * shape - drag * std::tanh(w / 5.0);
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
        w = w_idle;
        throttle_min = idle_throttle;
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