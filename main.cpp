#include "deps.hpp"
#include "car.hpp"
#include "environment.hpp"

double dispFPS = 60;
uint phys_per_disp_FPS = 10; //physics frames per displayed frame


int main(int argc, const char* argv[]){

    int h = 500;
    int w = 1000;

    carBase car = carBase(1/(dispFPS * phys_per_disp_FPS), 4.0, 1.75, 1500.0f, 0.0005, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    Environment scene = Environment(0.0f, 0.0f, 1.0f, 1.0f, 100.0f);

    float throttle = 0.0;
    float steer = 0.0;
    float brake = 0.0;

    InitWindow(w, h, "car");
    SetTargetFPS(dispFPS);

    uint frame_count = 0;
    float dist_scale = 50.0;

    float cam_yaw_default = 0.0;
    float cam_pitch_default = pi/16;

    float cam_yaw_lim = 3*pi/4;
    float cam_pitch_lim = 3*pi/8;

    float cam_yaw = cam_yaw_default;
    float cam_pitch = cam_pitch_default;

    float cam_alpha = 1.0 - std::exp(-1.0/(dispFPS * 0.1));

    int clutch_state = -1;
    int gear_state = 0;

    while(!WindowShouldClose()){

        if(IsKeyDown(KEY_W)) throttle = 1.0;
        if(IsKeyDown(KEY_A)) steer = -1.0;
        if(IsKeyDown(KEY_D)) steer = 1.0;
        if(IsKeyDown(KEY_S)) brake = 1.0;

        if(IsKeyDown(KEY_DOWN)) cam_pitch -= 2.0*std::abs(cam_pitch_lim + cam_pitch) / dispFPS;
        else if(IsKeyDown(KEY_UP)) cam_pitch += 2.0*std::abs(cam_pitch_lim - cam_pitch) / dispFPS;
        else cam_pitch += cam_alpha * (cam_pitch_default - cam_pitch);

        if(IsKeyDown(KEY_RIGHT)) cam_yaw -= 2.0*std::abs(cam_yaw_lim + cam_yaw) / dispFPS;
        else if(IsKeyDown(KEY_LEFT)) cam_yaw += 2.0*std::abs(cam_yaw_lim - cam_yaw) / dispFPS;
        else cam_yaw += cam_alpha * (cam_yaw_default - cam_yaw);

        if(IsKeyReleased(KEY_E)) gear_state = std::min(gear_state+1, car.drivetrain.max_gear);
        if(IsKeyReleased(KEY_Q)) gear_state = std::max(gear_state-1, car.drivetrain.min_gear);

        if(IsKeyReleased(KEY_C)) clutch_state *= -1;

        float clutch_command = clutch_state == 1 ? 1.0f : 0.0f;

        int starter_command = 0;
        if(IsKeyReleased(KEY_L)) starter_command = 1;
        if(IsKeyReleased(KEY_K)) starter_command = -1;


        for(int i = 0; i < phys_per_disp_FPS; i++){
            car.input_control(throttle, brake, steer, gear_state, clutch_command, starter_command);
            car.update_state();
        }

        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(scene.camera);
                Eigen::Vector3f cam_dir = Eigen::AngleAxisf(cam_yaw, Eigen::Vector3f(0.0f, 0.0f, 1.0f)) * Eigen::Vector3f(std::cos(car.tc), std::sin(car.tc), 0.0f);
                scene.draw(car.com_pos, cam_dir, 8.0f, cam_pitch);
                car.draw();
            EndMode3D();
            DrawFPS(10, 10);
            DrawText(TextFormat("speed: %.2f km/h", car.vl*3.6f), 10, 40, 20, WHITE);
            DrawText(TextFormat("gear: %i", car.drivetrain.gear), 10, 70, 20, WHITE);
            DrawText(TextFormat("rpm: %.0f", car.drivetrain.w*60/(2*pi)), 10, 100, 20, WHITE);
            DrawText(TextFormat("throttle: %.0f%%", car.drivetrain.throttle * 100.0f), 220, 10, 20, WHITE);
            DrawText(TextFormat("clutch: %.0f%% engaged", car.drivetrain.clutch * 100.0f), 220, 40, 20, WHITE);
        EndDrawing();

        throttle = 0.0;
        steer = 0.0;
        brake = 0.0;
        
        frame_count++;
    }
    CloseWindow();
    return 0;
}