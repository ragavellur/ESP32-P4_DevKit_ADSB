#pragma once

#include <cstddef>
#include <cstdint>

namespace adsb_radar::adsb_rx {

struct Frame {
  uint8_t bytes[14]{};
  uint8_t bit_length = 0;
  uint32_t icao = 0;
  uint8_t type_code = 0;
  char callsign[9]{};
  bool has_callsign = false;
  int altitude_ft = 0;
  bool has_altitude = false;
  int speed_kts = 0;
  bool has_speed = false;
  int heading_deg = 0;
  bool has_heading = false;
  int vertical_rate_fpm = 0;
  bool has_vertical_rate = false;
  uint32_t cpr_latitude = 0;
  uint32_t cpr_longitude = 0;
  bool cpr_odd = false;
  bool has_cpr = false;
  uint16_t signal = 0;
  /* Preamble start position in magnitudes buffer (sample index) */
  uint32_t preamble_offset_samples = 0;
};

bool decode_global_cpr(const Frame& first, const Frame& second, bool use_odd,
                       double* latitude, double* longitude);

struct Stats {
  uint32_t preambles = 0;
  uint32_t frames = 0;
  uint32_t df17 = 0;
  uint32_t crc_ok = 0;
  uint16_t magnitude_min = 0xffff;
  uint16_t magnitude_max = 0;
};

using FrameCallback = void (*)(const Frame&, void*);

class Decoder {
  public:
    void reset();
    void process_cu8(const uint8_t* data, size_t bytes, FrameCallback callback, void* context);
    const Stats& stats() const { return stats_; }
    static bool self_check();
    void set_dc_filter(bool enable);
    void set_aggressive(bool enable);
    void set_crc_fix(bool enable);
    void set_sample_rate(uint32_t sample_rate_sps);
    /* Noise-adaptive preamble gate (readsb --preamble-threshold). Higher = stricter. */
    void set_preamble_threshold(uint32_t threshold);
    uint32_t preamble_threshold() const { return preamble_threshold_; }

  private:
    /* Process a run of sample pairs (cu8). May be called repeatedly by
     * process_cu8() to consume blocks larger than magnitudes_ can hold; the
     * carry-over overlap_ and DC-filter state are preserved across calls. */
    void process_pairs(const uint8_t* data, size_t pairs, FrameCallback callback,
                       void* context);

    /* dump1090/readsb demodulate2400 port.
     *
     * Geometry at 2.4 MSPS: preamble occupies 20 samples (19 + phase/5), a long
     * message is 112 bits * 12/5 = 268 samples. The slicer reads up to
     * pPtr+20 for the last correlation tap, so keep ample trailing room. */
    static constexpr size_t kMagnitudeCapacity = 16640;
    static constexpr size_t kLongMsgSamples = 268;
    static constexpr size_t kShortMsgSamples = 135;
    static constexpr size_t kMaxFrameSamples = kLongMsgSamples;
    /* Trailing samples carried into the next block (preamble + longest message). */
    static constexpr size_t kTrailingSamples = kLongMsgSamples + 24;
    uint16_t magnitudes_[kMagnitudeCapacity]{};
    size_t overlap_ = 0;
    Stats stats_{};
    bool dc_filter_enabled_ = false;
    float dc_z1_i_ = 0.0f;
    float dc_z1_q_ = 0.0f;
    float dc_a_ = 0.0f;
    float dc_b_ = 1.0f;
    bool aggressive_ = false;
    bool crc_fix_enabled_ = false;
    uint32_t sample_rate_sps_ = 2400000;
    uint32_t preamble_threshold_ = 100;
};

}  // namespace adsb_radar::adsb_rx
