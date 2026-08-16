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

  ///@param amp_factor volume amplification factor
  ///@param freq_hz resonant frequency in Hertz
  ///@param t60_ms time to decay by 60 decibels, in miliseconds
  void set(float amp_factor, float freq_hz, float t60_ms, int sample_rate) {
    float r = std::exp(-6.9078f / (sample_rate * t60_ms / 1000.0f));
    float theta = 2.0f * pi * freq_hz / sample_rate;
    a1 = -2.0f * r * std::cos(theta);
    a2 = r * r;
    b0 = (1.0f - r) *
         std::sqrt(1.0f - 2.0f * r * std::cos(2.0f * theta) + r * r);
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

  float random() {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    float noise = dist(rng);
    return noise;
  }

  carSoundEmitter(float _loudness, int _num_cylinders, float _max_cyl_offset,
                  std::vector<twoPole> _exhaust_resonance,
                  onePole _exhaust_low_pass, int _sample_rate, float _fps) {
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
      cyl_firing_offsets.push_back(random() * max_cyl_offset);
    }

    exhaust_res = _exhaust_resonance;
    exhaust_lp = _exhaust_low_pass;
  }

  carSoundEmitter(int _sample_rate, float _fps)
      : carSoundEmitter(10.0f, 4, 0.025f,
                        {twoPole(0.75f, 120.0f, 80.0f, _sample_rate),
                         twoPole(0.35f, 340.0f, 20.0f, _sample_rate),
                         twoPole(0.08f, 1200.0f, 6.0f, _sample_rate)},
                        onePole(1.0f, 3600.0f, _sample_rate), _sample_rate,
                        _fps) {}

  carSoundEmitter() : carSoundEmitter(14400, 144.0f) {}

  float combustion(float throttle, float engine_rot) {
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

  float exhaust_pipe(float x) {
    float s = 0.0f;
    for (twoPole &p : exhaust_res) {
      s += p(x);
    }
    return exhaust_lp(s);
  }

  void update_phase(float engine_rot) {
    phase += engine_rot / (2 * pi) / SAMPLE_RATE;
    phase = std::fmod(phase, 2.0f); // two revolutions due to four stroke
  }

  short emit(float throttle, float engine_rot) {
    update_phase(engine_rot);
    float sound = exhaust_pipe(combustion(throttle, engine_rot));
    return static_cast<short>(
        std::clamp(sound * final_amp * 16000.0f, -32000.0f, 32000.0f));
  }
};
