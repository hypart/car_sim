#include "deps.hpp"

struct carSoundEmitter{

    int SAMPLE_RATE;
    int BUFFER_SIZE;

    int num_cylinders = 4;
    float max_cyl_offset = 0.025f;
    std::vector<float> cyl_firing_offsets;

    float lp_tau = 5e-5;

    float final_amp = 16000.0f;
    float phase = 0.0f;

    float lp_sound_cache = 100000.0f;

    float random(){
        static std::mt19937 rng{std::random_device{}()};
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        float noise = dist(rng);
        return noise;
    }

    carSoundEmitter(int sample_rate, float fps){
        SAMPLE_RATE = sample_rate;
        float min_buf_size = SAMPLE_RATE/fps;
        BUFFER_SIZE = 1;
        while(BUFFER_SIZE < min_buf_size) BUFFER_SIZE *= 2;

        cyl_firing_offsets.clear();
        for(int i = 0; i < num_cylinders; i++){
            cyl_firing_offsets.push_back(random() * max_cyl_offset);
        }
    }

    carSoundEmitter(){
        carSoundEmitter(44100, 144);
    }

    float combustion(float throttle, float engine_rot){
        // one unit of cyl_phase == one firing interval; 0..num_cylinders per 720 deg cycle
        float cyl_phase = phase * 0.5f * num_cylinders;

        int cyl = int(cyl_phase) % num_cylinders;
        float local = cyl_phase - std::floor(cyl_phase) - cyl_firing_offsets[cyl];
        local -= std::floor(local);

        float decay = 8.0f + 20.0f * throttle;
        float floor_ = std::exp(-decay);
        float pulse = (std::exp(-decay * local) - floor_) / (1.0f - floor_);

        float mean = ((1.0f - floor_) / decay - floor_) / (1.0f - floor_);
        pulse = 2.0f * (pulse - mean);

        float amp = 0.15f + 0.85f * throttle;
        return pulse * amp;
    }

    float low_pass(float sound){
        float lp_alpha = 1.0 - std::exp(-1.0/(SAMPLE_RATE * lp_tau));
        lp_sound_cache = lp_sound_cache*(1-lp_alpha) + sound*lp_alpha;
        return lp_sound_cache;
    }

    void update_phase(float engine_rot){
        phase += engine_rot / (2*pi) / SAMPLE_RATE;
        phase = std::fmod(phase, 2.0f); // two revolutions due to four stroke
    }

    short emit(float throttle, float engine_rot){
        update_phase(engine_rot);
        float sound = low_pass(combustion(throttle, engine_rot));
        return static_cast<short>(sound * final_amp);
    }

};