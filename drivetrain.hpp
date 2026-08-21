#include "deps.hpp"

struct driveTrain {

    float throttle = 0.0f; // throttle command
    float clutch = 0.0f;   // 0 disengaged 1 fully engaged
    bool clutch_locked = false;
    int gear = 0;

    float throttle_min = 0.0f;
    float idle_throttle = 0.08f; // minimum throttle needed to idle
    float throttle_resp = 0.8f;  // how responsive freewheel rpm are to throttle input

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

    float engine_torque();

    void update_clutch_lock_state(float car_speed, float wheel_radius);

    float clutch_torque(float car_speed, float wheel_radius);

    float dw_engine_dt(float car_speed, float wheel_radius, float dt);

    void start_engine();

    void kill_engine();

    void configure_engine(float _peak_torque,       // maximum torque [Nm]
                          float _peak_rpm,          // rpm at max torque [min^-1]
                          float _idle_throttle,     // throttle needed to idle [0.0-1.0]
                          float _throttle_response, // how responsive the freewheel rpm
                                                    // should be to throttle [0.0-1.0]
                          float _inertia_engine     // inertia of the engine [Nm^2]
    );

    void configure_transmission(std::vector<float> _gear_ratios, int _neutral_gear_idx,
                                float _final_drive_ratio, float _clutch_max_torque,
                                float _clutch_slip_transition);
};
