#include "deps.hpp"
#include "drivetrain.hpp"

constexpr float pi =  3.14159265358979323846f;
constexpr float steer_eps = 1e-10;
constexpr float vel_eps = 1e-10;

struct wireMesh{

    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::tuple<uint, uint>> connectivity;

    wireMesh(std::vector<Eigen::Vector3f> _vertices, std::vector<std::tuple<uint, uint>> _connectivity){
        vertices = _vertices;
        connectivity = _connectivity;
    }

    wireMesh(){
        vertices = {};
        connectivity = {};
    }

};

class carBase{
public:

    float dt; //physics time step

    std::vector<Eigen::Vector3f> wheel_positions;
    float wheel_width;
    float wheel_radius;

    Eigen::Vector3f com_pos; //position of the center of mass [m]
    float tc = 0.0f; //rotation of chassis with respect to global x axis [rad]

    float vl = 0.0f; //longitudonal velocity component of chassis CoM [m/s]
    float ts = 0.0f; //steer control rotation, at the center of the front axle
    float tr = 0.0f; //rotation of right tire with respect to longitudonal chassis axis [rad]
    float tl = 0.0f; //rotation of left tire with respect to longitudonal chassis axis [rad]

    float w; //wheelbase width
    float l; //wheelbase length
    float m; //car mass
    float d; //drag coefficient

    float I_wheels;

    float brake = 0.0f; //braking command

    driveTrain drivetrain;

    // TODO add braking curve
    float brake_per_wheel(){
        return brake*10.0f/4.0f;
    }

    float dvl_dt(){
        float v_sign = 0.0;
        if (std::abs(vl) > vel_eps) v_sign = vl/std::abs(vl);
        return drivetrain.accel(m, I_wheels, vl, wheel_radius) - d*vl*std::abs(vl) - v_sign*brake_per_wheel()*(2 + std::cos(tr) + std::cos(tl));
    }

    std::tuple<float, float> wheel_rots(float curr_ts){
        float tr;
        float tl;
        if(std::abs(curr_ts) < steer_eps){
            tr = 0.0;
            tl = 0.0;
        }
        else{
            float ti = pi/2 - std::atan(1/std::tan(std::abs(curr_ts)) - w/(2*l)); //inside wheel angle
            float to = pi/2 - std::atan(1/std::tan(std::abs(curr_ts)) + w/(2*l)); //outside wheel angle
            if(curr_ts > 0.0){
                tr = to;
                tl = ti;
            }
            else{
                tr = -ti;
                tl = -to;
            }
        }
        return {tr, tl};
    }

    void update_control(float throttle_command, float brake_command, float steer_command, int gear_command, float clutch_command){
        ts = steer_command;
        std::tie(tr, tl) = wheel_rots(ts);
        
        drivetrain.throttle = throttle_command;
        brake = brake_command;

        drivetrain.gear = gear_command;
        drivetrain.clutch = clutch_command;
    }

public:

    carBase(float timestep, float length, float width, float mass, float drag, Eigen::Vector3f init_pos){
        dt = timestep;
        
        l = length;
        w = width;
        m = mass;
        d = drag;

        wheel_positions = {
            {l/2, w/2, 0.0}, // front right
            {l/2, -w/2, 0.0}, // front left
            {-l/2, w/2, 0.0}, // back right
            {-l/2, -w/2, 0.0}}; // back left

        wheel_width = l/10;
        wheel_radius = l/10;

        com_pos = init_pos;

        I_wheels = 0.5f * wheel_radius*wheel_radius * m*0.01;
    }

    float lag_alpha(float tau){
        if(tau <= 0.0f) return 1.0f; //immediate reaction
        return 1.0f - std::exp(-dt/tau);
    }

    void input_control(
        float throttle_command, 
        float brake_command, 
        float steer_command, 
        int gear_command, 
        float clutch_command,
        int starter_command){

        //time constants [s] (0: immediate reaction)
        float thr_tau = 0.0f;
        float str_tau = 0.5f;
        float brk_tau = 0.1f;
        float clt_tau = 0.1f;

        float thr_a = lag_alpha(thr_tau);
        float brk_a = lag_alpha(brk_tau);
        float str_a = lag_alpha(str_tau);
        float clt_a = lag_alpha(clt_tau);

        float thr_smooth = drivetrain.throttle   + thr_a * (throttle_command - drivetrain.throttle);
        float brk_smooth = brake            + brk_a * (brake_command    - brake);
        float str_smooth = ts               + str_a * (steer_command    - ts);
        float clt_smooth = drivetrain.clutch     + clt_a * (clutch_command   - drivetrain.clutch);

        update_control(thr_smooth, brk_smooth, str_smooth, gear_command, clt_smooth);

        if(starter_command == 1) drivetrain.start_engine();
        else if(starter_command == -1) drivetrain.kill_engine();
    }

    void update_state(){
        drivetrain.w += dt * drivetrain.dw_engine_dt(vl, wheel_radius);
        vl += dt * dvl_dt();
        float vt = 0.0;

        if(std::abs(ts) > steer_eps){
            float rn = l/tan(ts);
            float dtc = vl/rn;
            tc += dtc * dt;
            vt = dtc * l/2;
        }

        com_pos += Eigen::Vector3f(
            vl * std::cos(tc) - vt * std::sin(tc),
            vl * std::sin(tc) + vt * std::cos(tc),
            0.0
        )* dt;
    }

    void draw(){

        //wheels
        int wheel_idx = 0;
        for(Eigen::Vector3f p : wheel_positions){

            Eigen::Vector3f start(0.0f, -wheel_width/2, 0.0f);
            Eigen::Vector3f end(0.0, wheel_width/2, 0.0f);

            float angle_base = tc;
            float angle_wheel = 0.0f;
            if (wheel_idx == 0) angle_wheel = tr;
            if (wheel_idx == 1) angle_wheel = tl;
            Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);

            Eigen::Vector3f start_trans = com_pos + Eigen::AngleAxisf(angle_base, axis) * (p + Eigen::AngleAxisf(angle_wheel, axis) * start);
            Eigen::Vector3f end_trans = com_pos + Eigen::AngleAxisf(angle_base, axis) * (p + Eigen::AngleAxisf(angle_wheel, axis) * end);
            
            Vector3 start_v = {start_trans.x(), start_trans.z(), start_trans.y()};
            Vector3 end_v = {end_trans.x(), end_trans.z(), end_trans.y()};

            DrawCylinderWiresEx(start_v, end_v, wheel_radius, wheel_radius, 100, WHITE);
            wheel_idx++;
        }
    }
};