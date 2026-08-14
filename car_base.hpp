#include "deps.hpp"

constexpr float pi =  3.14159265358979323846f;
constexpr float steer_eps = 1e-6;
constexpr float vel_eps = 1e-6;

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
    wireMesh bodymesh;
    wireMesh wheelmesh;

    Eigen::Vector3f com_pos; //position of the center of mass [m]
    float tc; //rotation of chassis with respect to global x axis [rad]

    float vl; //longitudonal velocity component of chassis CoM [m/s]
    float ts; //steer control rotation, at the center of the front axle
    float tr; //rotation of right tire with respect to longitudonal chassis axis [rad]
    float tl; //rotation of left tire with respect to longitudonal chassis axis [rad]

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

    float dvl_dt(){
        float v_sign = 0.0;
        if (std::abs(vl) > vel_eps) v_sign = vl/std::abs(vl);
        return accel()/m - d*vl - v_sign*brake_per_wheel()*(2 + std::cos(tr) + std::cos(tl));
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

    void update_control(float throttle_command, float brake_command, float steer_command){
        ts = steer_command;
        std::tie(tr, tl) = wheel_rots(ts);
        
        throttle = throttle_command;
        brake = brake_command;
    }

public:

    carBase(float length, float width, float mass, float drag, Eigen::Vector3f init_pos){
        l = length;
        w = width;
        m = mass;
        d = drag;

        // TODO: quick fix to shift the mesh so the axle is at the com, fix this in the kinematics later
        std::vector<Eigen::Vector3f> body_corners = {{0.0, -w/2, 0.0}, {l, -w/2, 0.0}, {l, w/2, 0.0}, {0.0, w/2, 0.0}};
        std::vector<std::tuple<uint, uint>> body_conn = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};

        std::vector<Eigen::Vector3f> wheel_corners = {{-l/10, -l/20, 0.0}, {l/10, -l/20, 0.0}, {l/10, l/20, 0.0}, {-l/10, l/20, 0.0}};
        std::vector<std::tuple<uint, uint>> wheel_conn = body_conn;

        bodymesh = wireMesh(body_corners, body_conn);
        wheelmesh = wireMesh(wheel_corners, wheel_conn);

        com_pos = init_pos;
        tc = 0.0;

        vl = 0.0;
        ts = 0.0;
        tr = 0.0;
        tl = 0.0;

        throttle = 0.0;
        brake = 0.0;
    }

    void input_control(float throttle_command, float brake_command, float steer_command){

        //between 0 and 1 (0: immediate reaction)
        float thr_delay = 0.0;
        float str_delay = 0.999;
        float brk_delay = 0.999;

        float thr_smooth = throttle_command * (1-thr_delay) + throttle * thr_delay;
        float brk_smooth = brake_command * (1-brk_delay) + brake * brk_delay;
        float str_smooth = steer_command * (1-str_delay) + ts * str_delay;
        
        update_control(thr_smooth, brk_smooth, str_smooth);
    }

    void update_state(float dt){
        vl += dt * dvl_dt();

        if(std::abs(ts) > steer_eps){
            float rn = l/tan(ts);
            tc += vl/rn * dt;
        }

        com_pos += Eigen::Vector3f(
            vl * std::cos(tc),
            vl * std::sin(tc),
            0.0
        )* dt;
    }

    void draw(float zoom){

        std::vector<Eigen::Vector3f> body_corners;

        //body
        for(std::tuple<uint, uint> idcs : bodymesh.connectivity){
            Eigen::Vector3f start = bodymesh.vertices[std::get<0>(idcs)];
            Eigen::Vector3f end = bodymesh.vertices[std::get<1>(idcs)];

            Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);

            start = com_pos + Eigen::AngleAxisf(tc, axis) * start;
            end = com_pos + Eigen::AngleAxisf(tc, axis) * end;

            body_corners.push_back(start);

            DrawLine(start.x()*zoom, start.y()*zoom, end.x()*zoom, end.y()*zoom, WHITE);
        }

        //wheels
        int wheel_idx = 0;
        for(Eigen::Vector3f p : body_corners){
            for(std::tuple<uint, uint> idcs : wheelmesh.connectivity){
                Eigen::Vector3f start = wheelmesh.vertices[std::get<0>(idcs)];
                Eigen::Vector3f end = wheelmesh.vertices[std::get<1>(idcs)];

                float rot = tc;
                if (wheel_idx == 1) rot += tr;
                if (wheel_idx == 2) rot += tl;

                Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);

                start = p + Eigen::AngleAxisf(rot, axis) * start;
                end = p + Eigen::AngleAxisf(rot, axis) * end;

                DrawLine(start.x()*zoom, start.y()*zoom, end.x()*zoom, end.y()*zoom, WHITE);
            }
            wheel_idx++;
        }
    }
};