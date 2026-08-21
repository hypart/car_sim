#pragma once
#include "deps.hpp"
#include "drivetrain.hpp"
#include "sounds.hpp"

constexpr float steer_eps = 1e-10;
constexpr float vel_eps = 1e-10;

struct wireMesh {

    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::tuple<uint, uint>> connectivity;

    wireMesh(std::vector<Eigen::Vector3f> _vertices,
             std::vector<std::tuple<uint, uint>> _connectivity) {
        vertices = _vertices;
        connectivity = _connectivity;
    }

    wireMesh() {
        vertices = {};
        connectivity = {};
    }
};

class carBase {
  public:
    float dt; // physics time step

    std::vector<Eigen::Vector3f> wheel_positions;
    float wheel_width;
    float wheel_radius;

    Eigen::Vector3f com_pos; // position of the center of mass [m]
    float tc = 0.0f;         // rotation of chassis with respect to global x axis [rad]

    float vl = 0.0f; // longitudonal velocity component of chassis CoM [m/s]
    float ts = 0.0f; // steer control rotation, at the center of the front axle
    float tr = 0.0f; // rotation of right tire with respect to longitudonal
                     // chassis axis [rad]
    float tl = 0.0f; // rotation of left tire with respect to longitudonal chassis
                     // axis [rad]

    float w; // wheelbase width
    float l; // wheelbase length
    float m; // car mass
    float d; // drag coefficient

    float I_wheels;

    float brake = 0.0f; // braking command

    driveTrain drivetrain;
    carSoundEmitter sound;

    // TODO add braking curve
    float brake_per_wheel();

    float drag_force();

    float brake_force();

    float dvl_dt();

    std::tuple<float, float> wheel_rots(float curr_ts);

    void update_control(float throttle_command, float brake_command, float steer_command,
                        int gear_command, float clutch_command);

    float accel();

    float lag_alpha(float tau);

  public:
    carBase(float timestep, float disp_fps, float length, float width, float mass, float drag,
            Eigen::Vector3f init_pos);

    void input_control(float throttle_command, float brake_command, float steer_command,
                       int gear_command, float clutch_command, int starter_command);

    void update_state();

    void draw();

    short play_sound();
};
