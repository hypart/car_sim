#pragma once
#include "deps.hpp"

struct onePole {
    float y1 = 0.0f;
    float r = 0.0f;
    float amp = 1.0f;

    void set(float amp_factor, float cutoff_hz, int sample_rate) {
        r = std::exp(-2.0f * pi * cutoff_hz / sample_rate);
        amp = amp_factor;
    }

    onePole(float amp_factor, float cutoff_hz, int sample_rate) {
        set(amp_factor, cutoff_hz, sample_rate);
    }

    onePole() = default;

    float operator()(float x) {
        float y = y1 * r + x * (1.0f - r);
        y1 = y;
        return y * amp;
    }
};

struct twoPole {
    float y1 = 0.0f, y2 = 0.0f;
    float b0 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    float amp = 1.0f;

    void set(float amp_factor, float freq_hz, float t60_ms, int sample_rate) {
        float r = std::exp(-6.9078f / (sample_rate * t60_ms / 1000.0f));
        float theta = 2.0f * pi * freq_hz / sample_rate;
        a1 = -2.0f * r * std::cos(theta);
        a2 = r * r;
        b0 = (1.0f - r) * std::sqrt(1.0f - 2.0f * r * std::cos(2.0f * theta) + r * r);
        amp = amp_factor;
    }

    twoPole() = default;

    twoPole(float amp_factor, float freq_hz, float t60_ms, int sample_rate) {
        set(amp_factor, freq_hz, t60_ms, sample_rate);
    }

    float operator()(float x) {
        float y = b0 * x - a1 * y1 - a2 * y2;
        y2 = y1;
        y1 = y;
        return y * amp;
    }
};

struct carSoundEmitter {

    int SAMPLE_RATE;
    int BUFFER_SIZE;

    int num_cylinders;
    float max_cyl_offset;
    std::vector<float> cyl_firing_offsets;

    std::vector<twoPole> exhaust_res;
    onePole exhaust_lp;

    float final_amp;
    float phase = 0.0f;

    carSoundEmitter(float _loudness, int _num_cylinders, float _max_cyl_offset,
                    std::vector<twoPole> _exhaust_resonance, onePole _exhaust_low_pass,
                    int _sample_rate, float _fps);

    carSoundEmitter(int _sample_rate, float _fps);

    carSoundEmitter();

    float combustion(float throttle, float engine_rot);

    float exhaust_pipe(float x);

    void update_phase(float engine_rot);

    short emit(float throttle, float engine_rot);
};
