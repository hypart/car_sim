#include "deps.hpp"
#include "vec.hpp"

constexpr float pi =  3.14159265358979323846f;
constexpr float steer_eps = 1e-6;
constexpr float vel_eps = 1e-6;

struct wireMesh{

std::vector<Vec3D> vertices;
std::vector<std::tuple<uint, uint>> connectivity;

wireMesh(std::vector<Vec3D> _vertices, std::vector<std::tuple<uint, uint>> _connectivity){
    vertices = _vertices;
    connectivity = _connectivity;
}

wireMesh(){
    vertices = {};
    connectivity = {};
}

};

struct carDynState{
    float dvl, dvt;
};

struct carState{
    float vl; //longitudonal velocity component of chassis CoM [m/s]
    float vt; //transverse velocity component of chassis CoM [m/s]
    float ts; //steer control rotation, at the center of the front axle
    float tr; //rotation of right tire with respect to longitudonal chassis axis [rad]
    float tl; //rotation of left tire with respect to longitudonal chassis axis [rad]

    carState apply_timestep(float dt, const carDynState& ds){
        carState s{
            vl + dt * ds.dvl,
            vt + dt * ds.dvt,
            ts,
            tr,
            tl
        };
        return s;
    }
};

class carBase{
public:
    wireMesh bodymesh;
    wireMesh wheelmesh;

    Vec3D com_pos; //position of the center of mass [m]
    float tc; //rotation of chassis with respect to global x axis [rad]

    carState state;

    float w; //wheelbase width
    float l; //wheelbase length
    float m; //car mass
    float d; //drag coefficient

    float throttle; //throttle command
    float brake; //braking command

    // TODO add power curve
    float accel(){ //longitudonal acceleration [m/s^2]
        return throttle;
    }

    // TODO add braking curve
    float brake_per_wheel(){
        return brake/4;
    }

    float dtc_dt(const carState& s){
        if (abs(s.ts) < steer_eps) return 0.0;
        float r = l * std::hypot(1/std::tan(s.ts), 0.25);
        float ts_sign = s.ts/std::abs(s.ts);

        float bpw = brake_per_wheel();

        // braking acceleration caused by angled wheels
        float brake_accel_right = bpw * l / std::sin(-s.tr);
        float brake_accel_left = bpw * l / std::sin(-s.tl);

        return ts_sign * (std::hypot(s.vt, s.vl) / r) + brake_accel_left + brake_accel_right;
    }

    float dvl_dt(const carState& s){
        float v_sign = 0.0;
        if (std::abs(s.vl) > vel_eps) v_sign = s.vl/std::abs(s.vl);
        return accel()/m - d*s.vl - v_sign*brake_per_wheel()*(2 + std::cos(s.tr) + std::cos(s.tl));
    }

    float dvt_dt(const carState& s){
        if (abs(s.ts) < steer_eps) return 0.0;
        float r = l * std::hypot(1/std::tan(s.ts), 0.25);
        float ts_sign = s.ts/std::abs(s.ts);
        return ts_sign * ((std::pow(s.vl, 2) + std::pow(s.vt, 2)) / r) - brake_per_wheel()*(std::sin(s.tr) + std::sin(s.tl));
    }

    carDynState derivatives(const carState& s){
        float dvl = dvl_dt(s);
        float dvt = dvt_dt(s);
        carDynState ds{dvl, dvt};
        return ds;
    }

    std::tuple<float, float> wheel_rots(float curr_ts){
        float tr;
        float tl;
        if(abs(curr_ts) < steer_eps){
            tr = 0.0;
            tl = 0.0;
        }
        else{
            float ti = pi/2 - std::atan(1/std::tan(curr_ts) - w/(2*l)); //inside wheel angle
            float to = pi/2 - std::atan(1/std::tan(curr_ts) + w/(2*l)); //outside wheel angle
            if(curr_ts > 0.0){
                tr = ti;
                tl = to;
            }
            else{
                tr = to;
                tl = ti;
            }
        }
        return {tr, tl};
    }

    void update_control(float throttle_command, float brake_command, float steer_command){
        state.ts = steer_command;
        std::tie(state.tr, state.tl) = wheel_rots(state.ts);
        
        throttle = throttle_command;
        brake = brake_command;
    }

public:

    carBase(float length, float width, float mass, float drag, wireMesh body, wireMesh wheel, Vec3D init_pos){
        l = length;
        w = width;
        m = mass;
        d = drag;

        bodymesh = body;
        wheelmesh = wheel;

        com_pos = init_pos;

        state = carState{0.0, 0.0, 0.0, 0.0, 0.0};

        throttle = 0.0;
        brake = 0.0;
    }

    void input_control(float throttle_command, float brake_command, float steer_command){

        //between 0 and 1 (0: immediate reaction)
        float thr_delay = 0.5;
        float str_delay = 0.5;
        float brk_delay = 0.5;

        float thr_smooth = throttle_command * (1-thr_delay) + throttle * thr_delay;
        float brk_smooth = brake_command * (1-brk_delay) + brake * brk_delay;
        float str_smooth = steer_command * (1-str_delay) + state.ts * str_delay;
        
        update_control(thr_smooth, brk_smooth, str_smooth);
    }

    void update_state(float dt){
        state = state.apply_timestep(dt, derivatives(state));

        if(abs(state.vt) < vel_eps){
            tc = 0.0;
        }
        else{
            float tv = std::atan2(state.vt, state.vl);
            float sgn_rot = state.vt / std::abs(state.vt);
            float r = l/(2*tan(tv));
            tc += sgn_rot*std::hypot(state.vl, state.vt)/r * dt;
        }
        
        float st = std::sin(tc);
        float ct = std::cos(tc);
        com_pos += Vec3D(
            state.vl * ct - state.vt * st,
            state.vl * st + state.vt * ct,
            0.0
        ) * dt;
    }

};