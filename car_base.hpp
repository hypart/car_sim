#include "deps.hpp"

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

    float w_engine = 0.0f; //rotational velocity of engine [rad/s]

    float w; //wheelbase width
    float l; //wheelbase length
    float m; //car mass
    float d; //drag coefficient

    float max_torque;

    float throttle = 0.0f; //throttle command
    float throttle_min = 0.0f;
    float brake = 0.0f; //braking command

    float clutch = 0.0f; //0 disengaged 1 fully engaged
    int gear = 0;
    std::vector<float> ratios = {-4.0f, 0.0f, 4.0f, 2.5f, 1.5f, 1.0f, 0.75f};
    float final_drive_ratio = 4.0;

    float I_engine = 0.25f;
    float I_wheels;

    float engine_friction; //derived in the constructor from max_torque and w_peak
    float drag_exp = 3.0;

    float w_idle = 90.0f;
    float w_redline = 700.0f;

    float clutch_capacity = 600.0f; //max transmissible torque [Nm]
    float w_slip_eps = 20.0f;

    float throttle_scale;
    float w_peak; //engine speed of peak torque [rad/s]
    float idle_throttle; //throttle needed to hold w_idle against drag

    float engine_drag(){
        return engine_friction*std::pow(w_engine, drag_exp);
    }

    float engine_torque(){
        float rot = w_engine / w_redline;
        float torque = std::max(throttle, throttle_min) * throttle_scale * rot - engine_drag();
        return torque;
    }

    float clutch_torque(){
        if (ratios[gear+1] == 0.0f || clutch <= 0.0f) return 0.0f;

        float N = final_drive_ratio * ratios[gear+1] / wheel_radius;
        float slip = w_engine - vl * N;
        std::cout << slip << "\n";
        return clutch * clutch_capacity * std::tanh(slip / w_slip_eps);
    }

    float dw_engine_dt(){
        return (engine_torque() - clutch_torque())/I_engine;
    }

    void start_engine(){
        w_engine = w_idle;
        throttle_min = idle_throttle;
    }

    void kill_engine(){
        throttle_min = 0.0;
    }

    float accel(){ //longitudonal acceleration [m/s^2]
        float N = final_drive_ratio * ratios[gear+1];
        float wheel_ratio = N/wheel_radius;
        float m_eff = m + I_wheels/(wheel_radius*wheel_radius);

        return clutch_torque() * wheel_ratio / m_eff;
    }

    // TODO add braking curve
    float brake_per_wheel(){
        return brake*10.0f/4.0f;
    }

    float dvl_dt(){
        float v_sign = 0.0;
        if (std::abs(vl) > vel_eps) v_sign = vl/std::abs(vl);
        return accel() - d*vl*std::abs(vl) - v_sign*brake_per_wheel()*(2 + std::cos(tr) + std::cos(tl));
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
        
        throttle = throttle_command;
        brake = brake_command;

        gear = gear_command;
        clutch = clutch_command;
    }

public:

    carBase(float timestep, float length, float width, float _max_torque, float mass, float drag, Eigen::Vector3f init_pos){
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

        max_torque = _max_torque;

        I_wheels = 0.5f * wheel_radius*wheel_radius * m*0.01;
        
        //Build the torque curve from parameters
        w_peak = w_redline / std::pow(drag_exp, 1/(drag_exp - 1));

        float k = (drag_exp/(drag_exp - 1)) * max_torque / w_peak;
        throttle_scale = k * w_redline;
        engine_friction = k / (drag_exp * std::pow(w_peak, drag_exp - 1));

        //throttle that balances drag at idle: k*throttle*w_idle = f*w_idle^n
        idle_throttle = engine_friction * std::pow(w_idle, drag_exp - 1) / k;
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

        float thr_smooth = throttle + thr_a * (throttle_command - throttle);
        float brk_smooth = brake    + brk_a * (brake_command    - brake);
        float str_smooth = ts       + str_a * (steer_command    - ts);
        float clt_smooth = clutch   + clt_a * (clutch_command   - clutch);

        update_control(thr_smooth, brk_smooth, str_smooth, gear_command, clt_smooth);

        if(starter_command == 1) start_engine();
        else if(starter_command == -1) kill_engine();
    }

    void update_state(){
        w_engine += dt * dw_engine_dt();
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