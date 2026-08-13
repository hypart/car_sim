#include "deps.hpp"
#include "car_base.hpp"

double dispFPS = 60;
uint phys_per_disp_FPS = 10; //physics frames per displayed frame


int main(int argc, const char* argv[]){
    std::vector<Vec3D> body_nodes;
    std::vector<std::tuple<uint, uint>> body_connectivity;
    wireMesh body = wireMesh(body_nodes, body_connectivity);
    wireMesh wheel = wireMesh(body_nodes, body_connectivity);

    int h = 432;
    int w = 768;

    carBase car = carBase(2.0, 1.0, 1.0, 0.01, body, wheel, Vec3D(1.0f, 1.0f, 0.0f));

    float throttle = 0.0;
    float steer = 0.0;
    float brake = 0.0;

    InitWindow(w, h, "car");
    SetTargetFPS(dispFPS);

    uint frame_count = 0;
    float dist_scale = 50.0;

    while(!WindowShouldClose()){

        if(IsKeyDown(KEY_W)) throttle = 0.1;
        if(IsKeyDown(KEY_A)) steer = -1.0;
        if(IsKeyDown(KEY_D)) steer = 1.0;
        if(IsKeyDown(KEY_S)) brake = 0.1;

        for(int i = 0; i < phys_per_disp_FPS; i++){
            car.input_control(throttle, brake, steer);
            car.update_state(1/(dispFPS * phys_per_disp_FPS));
        }

        if(!(frame_count % phys_per_disp_FPS)){
            BeginDrawing();
                ClearBackground(BLACK);
                std::cout   << car.com_pos.x << "\n"
                            << car.com_pos.y << "\n"
                            << car.tc<< "\n";
                DrawLine(
                    car.com_pos.x*dist_scale, car.com_pos.y*dist_scale,
                    (car.com_pos.x + std::cos(car.tc))*dist_scale, (car.com_pos.y + std::sin(car.tc))*dist_scale,
                    WHITE
                );
            EndDrawing();
        }

        throttle = 0.0;
        steer = 0.0;
        brake = 0.0;
        
        frame_count++;
    }
    CloseWindow();
    return 0;
}