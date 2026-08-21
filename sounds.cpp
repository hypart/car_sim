#include "sounds.hpp"

carSoundEmitter::carSoundEmitter(float _loudness, int _num_cylinders, float _max_cyl_offset,
                                 std::vector<twoPole> _exhaust_resonance, onePole _exhaust_low_pass,
                                 int _sample_rate, float _fps) {
    SAMPLE_RATE = _sample_rate;
    float min_buf_size = SAMPLE_RATE / _fps;
    BUFFER_SIZE = 1;
    while (BUFFER_SIZE < min_buf_size)
        BUFFER_SIZE *= 2;

    final_amp = _loudness;
    num_cylinders = _num_cylinders;
    max_cyl_offset = _max_cyl_offset;

    cyl_firing_offsets.clear();
    for (int i = 0; i < num_cylinders; i++) {
        cyl_firing_offsets.push_back(rand_uniform() * max_cyl_offset);
    }

    exhaust_res = _exhaust_resonance;
    exhaust_lp = _exhaust_low_pass;
}

carSoundEmitter::carSoundEmitter(int _sample_rate, float _fps)
    : carSoundEmitter(10.0f, 4, 0.025f,
                      {twoPole(0.75f, 120.0f, 80.0f, _sample_rate),
                       twoPole(0.35f, 340.0f, 20.0f, _sample_rate),
                       twoPole(0.08f, 1200.0f, 6.0f, _sample_rate)},
                      onePole(1.0f, 3600.0f, _sample_rate), _sample_rate, _fps) {}

carSoundEmitter::carSoundEmitter() : carSoundEmitter(14400, 144.0f) {}

float carSoundEmitter::combustion(float throttle, float engine_rot) {
    // one unit of cyl_phase == one firing interval; 0..num_cylinders per 720
    // deg cycle
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

float carSoundEmitter::exhaust_pipe(float x) {
    float s = 0.0f;
    for (twoPole& p : exhaust_res) {
        s += p(x);
    }
    return exhaust_lp(s);
}

void carSoundEmitter::update_phase(float engine_rot) {
    phase += engine_rot / (2 * pi) / SAMPLE_RATE;
    phase = std::fmod(phase, 2.0f); // two revolutions due to four stroke
}

short carSoundEmitter::emit(float throttle, float engine_rot) {
    update_phase(engine_rot);
    float sound = exhaust_pipe(combustion(throttle, engine_rot));
    return static_cast<short>(std::clamp(sound * final_amp * 16000.0f, -32000.0f, 32000.0f));
}