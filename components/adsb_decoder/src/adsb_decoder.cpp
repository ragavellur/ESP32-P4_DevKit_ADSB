#include "adsb_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace adsb_radar::adsb_rx {
namespace {

constexpr uint32_t kCrcPolynomial = 0xfff409u;

uint32_t bits(const uint8_t* frame, int first, int length) {
  uint32_t value = 0;
  for (int i = 0; i < length; ++i)
    value = (value << 1) | ((frame[(first + i) / 8] >> (7 - (first + i) % 8)) & 1u);
  return value;
}

uint32_t crc_payload(const uint8_t* frame, int data_bits) {
  uint32_t crc = 0;
  for (int i = 0; i < data_bits; ++i) {
    const bool feedback = ((crc >> 23) & 1u) != ((frame[i / 8] >> (7 - i % 8)) & 1u);
    crc = (crc << 1) & 0xffffffu;
    if (feedback) crc ^= kCrcPolynomial;
  }
  return crc;
}



bool parse(const uint8_t* bytes, uint8_t bit_length, uint16_t signal, Frame* out) {
  const uint8_t df = bytes[0] >> 3;
  const int data_bits = bit_length - 24;
  const uint32_t syndrome = crc_payload(bytes, data_bits) ^ bits(bytes, data_bits, 24);
  if (!((df == 17 && bit_length == 112 && syndrome == 0) ||
        (df == 11 && bit_length == 56 && syndrome <= 0x7f)))
    return false;
  std::memcpy(out->bytes, bytes, bit_length / 8);
  out->bit_length = bit_length;
  out->icao = bits(bytes, 8, 24);
  if (df != 17) {
    out->signal = signal;
    return true;
  }
  out->type_code = static_cast<uint8_t>(bits(bytes, 32, 5));
  out->signal = signal;

  if (out->type_code >= 1 && out->type_code <= 4) {
    size_t length = 8;
    for (size_t i = 0; i < 8; ++i) {
      const uint8_t code = static_cast<uint8_t>(bits(bytes, 40 + static_cast<int>(i) * 6, 6));
      out->callsign[i] = code >= 1 && code <= 26 ? static_cast<char>('A' + code - 1)
                         : code >= 48 && code <= 57 ? static_cast<char>('0' + code - 48)
                                                    : ' ';
    }
    while (length && out->callsign[length - 1] == ' ') --length;
    out->callsign[length] = '\0';
    out->has_callsign = length != 0;
  } else if (out->type_code >= 9 && out->type_code <= 18) {
    const uint16_t encoded = static_cast<uint16_t>(bits(bytes, 40, 12));
    if ((encoded & 0x10u) != 0) {
      const int n = ((encoded & 0xfe0u) >> 1) | (encoded & 0x0fu);
      out->altitude_ft = n * 25 - 1000;
      out->has_altitude = true;
    }
    out->cpr_odd = bits(bytes, 53, 1) != 0;
    out->cpr_latitude = bits(bytes, 54, 17);
    out->cpr_longitude = bits(bytes, 71, 17);
    out->has_cpr = true;
  } else if (out->type_code == 19) {
    const uint8_t subtype = static_cast<uint8_t>(bits(bytes, 37, 3));
    const int scale = (subtype == 2 || subtype == 4) ? 4 : 1;
    if (subtype == 1 || subtype == 2) {
      const int ew_raw = static_cast<int>(bits(bytes, 46, 10));
      const int ns_raw = static_cast<int>(bits(bytes, 57, 10));
      if (ew_raw && ns_raw) {
        const int ew = (bits(bytes, 45, 1) ? -1 : 1) * (ew_raw - 1) * scale;
        const int ns = (bits(bytes, 56, 1) ? -1 : 1) * (ns_raw - 1) * scale;
        out->speed_kts = static_cast<int>(std::lround(std::sqrt(ew * ew + ns * ns)));
        out->heading_deg = static_cast<int>(std::lround(
            std::fmod(std::atan2(static_cast<double>(ew), static_cast<double>(ns)) *
                              180.0 / 3.14159265358979323846 + 360.0,
                      360.0)));
        out->has_speed = out->has_heading = true;
      }
    }
    const int vr = static_cast<int>(bits(bytes, 69, 9));
    if (vr) {
      out->vertical_rate_fpm = (bits(bytes, 68, 1) ? -1 : 1) * (vr - 1) * 64;
      out->has_vertical_rate = true;
    }
  }
  return true;
}

// Try to fix single-bit CRC error by flipping each bit
bool try_crc_fix(uint8_t* bytes, int bit_length, Frame* out) {
  uint8_t original[14];
  std::memcpy(original, bytes, bit_length / 8);
  
  for (int bit = 0; bit < bit_length; ++bit) {
    // Flip bit
    bytes[bit / 8] ^= (0x80u >> (bit % 8));
    
    // Check CRC
    const uint8_t df = bytes[0] >> 3;
    const int data_bits = bit_length - 24;
    const uint32_t syndrome = crc_payload(bytes, data_bits) ^ bits(bytes, data_bits, 24);
    
    if ((df == 17 && bit_length == 112 && syndrome == 0) ||
        (df == 11 && bit_length == 56 && syndrome <= 0x7f)) {
      // CRC fixed! Parse the corrected frame
      return parse(bytes, bit_length, 0, out);
    }
    
    // Restore and try next bit
    std::memcpy(bytes, original, bit_length / 8);
  }
  return false;
}

}  // namespace

bool decode_global_cpr(const Frame& first, const Frame& second, bool use_odd,
                       double* latitude, double* longitude) {
  if (!latitude || !longitude || !first.has_cpr || !second.has_cpr ||
      first.cpr_odd == second.cpr_odd || first.icao != second.icao)
    return false;
  const Frame& even = first.cpr_odd ? second : first;
  const Frame& odd = first.cpr_odd ? first : second;
  const double yz_even = even.cpr_latitude / 131072.0;
  const double yz_odd = odd.cpr_latitude / 131072.0;
  const int j = static_cast<int>(std::floor(59.0 * yz_even - 60.0 * yz_odd + 0.5));
  auto positive_mod = [](int value, int divisor) {
    const int result = value % divisor;
    return result < 0 ? result + divisor : result;
  };
  double lat_even = 6.0 * (positive_mod(j, 60) + yz_even);
  double lat_odd = (360.0 / 59.0) * (positive_mod(j, 59) + yz_odd);
  if (lat_even >= 270.0) lat_even -= 360.0;
  if (lat_odd >= 270.0) lat_odd -= 360.0;
  auto longitude_zones = [](double latitude_value) {
    constexpr double kPi = 3.14159265358979323846;
    const double latitude_radians = std::abs(latitude_value) * kPi / 180.0;
    if (latitude_radians >= 87.0 * kPi / 180.0) return 1;
    const double numerator = 1.0 - std::cos(kPi / 30.0);
    const double denominator = std::cos(latitude_radians) * std::cos(latitude_radians);
    return static_cast<int>(std::floor(2.0 * kPi / std::acos(1.0 - numerator / denominator)));
  };
  const int nl_even = longitude_zones(lat_even);
  const int nl_odd = longitude_zones(lat_odd);
  if (nl_even != nl_odd) return false;
  const double xz_even = even.cpr_longitude / 131072.0;
  const double xz_odd = odd.cpr_longitude / 131072.0;
  const int m = static_cast<int>(
      std::floor(xz_even * (nl_even - 1) - xz_odd * nl_even + 0.5));
  const int ni = std::max(nl_even - (use_odd ? 1 : 0), 1);
  double lon = (360.0 / ni) *
               (positive_mod(m, ni) + (use_odd ? xz_odd : xz_even));
  if (lon > 180.0) lon -= 360.0;
  *latitude = use_odd ? lat_odd : lat_even;
  *longitude = lon;
  return true;
}

void Decoder::reset() {
  overlap_ = 0;
  stats_ = {};
  dc_z1_i_ = 0.0f;
  dc_z1_q_ = 0.0f;
}

void Decoder::set_dc_filter(bool enable) {
  dc_filter_enabled_ = enable;
  if (!enable) {
    dc_z1_i_ = 0.0f;
    dc_z1_q_ = 0.0f;
  }
}

void Decoder::set_aggressive(bool enable) {
  aggressive_ = enable;
}

void Decoder::set_crc_fix(bool enable) {
  crc_fix_enabled_ = enable;
}

void Decoder::set_sample_rate(uint32_t sample_rate_sps) {
  sample_rate_sps_ = sample_rate_sps;
}

void Decoder::set_preamble_threshold(uint32_t threshold) {
  preamble_threshold_ = threshold;
}

void Decoder::process_cu8(const uint8_t* data, size_t bytes, FrameCallback callback,
                          void* context) {
  if (!data) return;
  bytes &= ~size_t{1};
  const size_t pairs = bytes / 2;
  if (pairs == 0) return;
  /* magnitudes_ is a fixed buffer. Consume arbitrarily large blocks in
   * sub-runs that leave room for the carry-over overlap, so no samples are
   * silently dropped (a 64 KiB block = 32768 pairs > kMagnitudeCapacity). */
  const size_t max_new_pairs = kMagnitudeCapacity - kTrailingSamples;
  size_t off = 0;
  while (off < pairs) {
    const size_t n = std::min(pairs - off, max_new_pairs);
    process_pairs(data + off * 2, n, callback, context);
    off += n;
  }
}

/* --------------------------------------------------------------------------
 * dump1090 / readsb demodulate2400 (2.4 MSPS), ported.
 *
 * Reference: readsb demod_2400.c + convert.c (uc8 magnitude) + mode_s.c
 * (CRC). Behaviour differences from the previous hand-rolled detector:
 *   - magnitude is linear amplitude  sqrt((I-127.5)^2 + (Q-127.5)^2)
 *     normalized to 0..65535, not |I|+|Q|.
 *   - DC removal is a 1 Hz single-pole high-pass (readsb exp(-2*pi/sr)),
 *     not a 1/4096 running mean.
 *   - preamble uses readsb's pulse-shape pre-check and a noise-adaptive
 *     threshold ref_level = base_noise * preamble_threshold / 32.
 *   - bits are recovered by 5-phase DC-balanced correlation (slice_phase0..4)
 *     instead of a single fixed-phase 2-point compare.
 * -------------------------------------------------------------------------- */
namespace {

constexpr uint32_t kModesLongMsgBytes = 14;
constexpr uint32_t kModesShortMsgBytes = 7;

// DC-balanced 3/4-tap correlation slicers (verbatim from readsb demod_2400.c).
inline int slice_phase0(const uint16_t* m) { return 18 * m[0] - 15 * m[1] - 3 * m[2]; }
inline int slice_phase1(const uint16_t* m) { return 14 * m[0] - 5 * m[1] - 9 * m[2]; }
inline int slice_phase2(const uint16_t* m) { return 16 * m[0] + 5 * m[1] - 20 * m[2]; }
inline int slice_phase3(const uint16_t* m) { return 7 * m[0] + 11 * m[1] - 18 * m[2]; }
inline int slice_phase4(const uint16_t* m) { return 4 * m[0] + 15 * m[1] - 20 * m[2] + m[3]; }

inline uint8_t slice_byte(const uint16_t** pPtr, int* phase) {
  const uint16_t* p = *pPtr;
  uint8_t b = 0;
  switch (*phase) {
    case 0:
      b = (slice_phase0(p) > 0 ? 0x80 : 0) | (slice_phase2(p + 2) > 0 ? 0x40 : 0) |
          (slice_phase4(p + 4) > 0 ? 0x20 : 0) | (slice_phase1(p + 7) > 0 ? 0x10 : 0) |
          (slice_phase3(p + 9) > 0 ? 0x08 : 0) | (slice_phase0(p + 12) > 0 ? 0x04 : 0) |
          (slice_phase2(p + 14) > 0 ? 0x02 : 0) | (slice_phase4(p + 16) > 0 ? 0x01 : 0);
      *phase = 1; *pPtr = p + 19; break;
    case 1:
      b = (slice_phase1(p) > 0 ? 0x80 : 0) | (slice_phase3(p + 2) > 0 ? 0x40 : 0) |
          (slice_phase0(p + 5) > 0 ? 0x20 : 0) | (slice_phase2(p + 7) > 0 ? 0x10 : 0) |
          (slice_phase4(p + 9) > 0 ? 0x08 : 0) | (slice_phase1(p + 12) > 0 ? 0x04 : 0) |
          (slice_phase3(p + 14) > 0 ? 0x02 : 0) | (slice_phase0(p + 17) > 0 ? 0x01 : 0);
      *phase = 2; *pPtr = p + 19; break;
    case 2:
      b = (slice_phase2(p) > 0 ? 0x80 : 0) | (slice_phase4(p + 2) > 0 ? 0x40 : 0) |
          (slice_phase1(p + 5) > 0 ? 0x20 : 0) | (slice_phase3(p + 7) > 0 ? 0x10 : 0) |
          (slice_phase0(p + 10) > 0 ? 0x08 : 0) | (slice_phase2(p + 12) > 0 ? 0x04 : 0) |
          (slice_phase4(p + 14) > 0 ? 0x02 : 0) | (slice_phase1(p + 17) > 0 ? 0x01 : 0);
      *phase = 3; *pPtr = p + 19; break;
    case 3:
      b = (slice_phase3(p) > 0 ? 0x80 : 0) | (slice_phase0(p + 3) > 0 ? 0x40 : 0) |
          (slice_phase2(p + 5) > 0 ? 0x20 : 0) | (slice_phase4(p + 7) > 0 ? 0x10 : 0) |
          (slice_phase1(p + 10) > 0 ? 0x08 : 0) | (slice_phase3(p + 12) > 0 ? 0x04 : 0) |
          (slice_phase0(p + 15) > 0 ? 0x02 : 0) | (slice_phase2(p + 17) > 0 ? 0x01 : 0);
      *phase = 4; *pPtr = p + 19; break;
    default:
      b = (slice_phase4(p) > 0 ? 0x80 : 0) | (slice_phase1(p + 3) > 0 ? 0x40 : 0) |
          (slice_phase3(p + 5) > 0 ? 0x20 : 0) | (slice_phase0(p + 8) > 0 ? 0x10 : 0) |
          (slice_phase2(p + 10) > 0 ? 0x08 : 0) | (slice_phase4(p + 12) > 0 ? 0x04 : 0) |
          (slice_phase1(p + 15) > 0 ? 0x02 : 0) | (slice_phase3(p + 17) > 0 ? 0x01 : 0);
      *phase = 0; *pPtr = p + 20; break;
  }
  return b;
}

uint32_t valid_df_short_bitset = 0;
uint32_t valid_df_long_bitset = 0;

void init_bitsets_once() {
  if (valid_df_short_bitset != 0) return;
  /* DFs understood without correction. */
  valid_df_short_bitset = (1u << 0) | (1u << 4) | (1u << 5) | (1u << 11);
  valid_df_long_bitset = (1u << 16) | (1u << 17) | (1u << 18) | (1u << 20) | (1u << 21);
}

}  // namespace

void Decoder::process_pairs(const uint8_t* data, size_t pairs, FrameCallback callback,
                            void* context) {
  init_bitsets_once();

  /* --- magnitude: readsb uc8 conversion (linear amplitude, 1 Hz DC high-pass) --- */
  const float inv = 1.0f / 127.5f;
  if (dc_filter_enabled_) {
    /* readsb: dc_b = exp(-2*pi*1.0/sample_rate); dc_a = 1 - dc_b */
    dc_b_ = std::exp(-2.0f * 3.14159265358979323846f / static_cast<float>(sample_rate_sps_));
    dc_a_ = 1.0f - dc_b_;
  }
  size_t count = overlap_;
  for (size_t i = 0; i < pairs && count < kMagnitudeCapacity; ++i) {
    float fI = (static_cast<float>(data[i * 2]) - 127.5f) * inv;
    float fQ = (static_cast<float>(data[i * 2 + 1]) - 127.5f) * inv;
    if (dc_filter_enabled_) {
      dc_z1_i_ = fI * dc_a_ + dc_z1_i_ * dc_b_;
      dc_z1_q_ = fQ * dc_a_ + dc_z1_q_ * dc_b_;
      fI -= dc_z1_i_;
      fQ -= dc_z1_q_;
    }
    float magsq = fI * fI + fQ * fQ;
    if (magsq > 1.0f) magsq = 1.0f;
    const uint16_t mag = static_cast<uint16_t>(std::sqrt(magsq) * 65535.0f + 0.5f);
    stats_.magnitude_min = std::min(stats_.magnitude_min, mag);
    stats_.magnitude_max = std::max(stats_.magnitude_max, mag);
    magnitudes_[count++] = mag;
  }

  /* --- preamble detection + multi-phase slicing (readsb demodulate2400) --- */
  const uint32_t threshold = preamble_threshold_;
  /* Trailing samples needed for the slicer past the last preamble candidate. */
  const size_t need = kMaxFrameSamples + 24;
  if (count > need) {
    const size_t last = count - need;
    for (size_t pos = 0; pos <= last; ++pos) {
      const uint16_t* pa = magnitudes_ + pos;
      if (!(pa[1] > pa[7] && pa[12] > pa[14] && pa[12] > pa[15])) continue;

      const int base_noise = static_cast<int>(pa[5]) + pa[8] + pa[16] + pa[17] + pa[18];
      const int ref_level = static_cast<int>((static_cast<int64_t>(base_noise) * threshold) >> 5);

      const int diff_2_3 = static_cast<int>(pa[2]) - pa[3];
      const int sum_1_4 = static_cast<int>(pa[1]) + pa[4];
      const int diff_10_11 = static_cast<int>(pa[10]) - pa[11];
      const int common3456 = sum_1_4 - diff_2_3 + pa[9] + pa[12];

      Frame best{};
      int best_bit_length = 0;
      bool have_candidate = false;

      auto score_phase = [&](int try_phase) {
        const uint16_t* pPtr = pa + 19 + (try_phase / 5);
        int phase = try_phase % 5;
        uint8_t msg[14]{};
        msg[0] = slice_byte(&pPtr, &phase);
        const uint32_t df = msg[0] >> 3;
        uint32_t bytelen;
        if (valid_df_long_bitset & (1u << df)) {
          bytelen = kModesLongMsgBytes;
        } else if (valid_df_short_bitset & (1u << df)) {
          bytelen = kModesShortMsgBytes;
        } else {
          return;
        }
        for (uint32_t i = 1; i < bytelen; ++i) msg[i] = slice_byte(&pPtr, &phase);

        uint8_t frame_copy[14];
        std::memcpy(frame_copy, msg, bytelen);
        Frame decoded{};
        if (!parse(frame_copy, bytelen * 8, 0, &decoded)) {
          if (!crc_fix_enabled_) return;
          std::memcpy(frame_copy, msg, bytelen);
          if (!try_crc_fix(frame_copy, bytelen * 8, &decoded)) return;
        }
        if (have_candidate && best_bit_length >= static_cast<int>(bytelen * 8)) return;
        best = decoded;
        best_bit_length = static_cast<int>(bytelen * 8);
        have_candidate = true;
      };

      int pa_mag = common3456 - diff_10_11;
      if (pa_mag >= ref_level) { score_phase(4); score_phase(5); }
      pa_mag = common3456 + diff_10_11;
      if (pa_mag >= ref_level) { score_phase(6); score_phase(7); }
      pa_mag = sum_1_4 + 2 * diff_2_3 + diff_10_11 + pa[12];
      if (pa_mag >= ref_level) score_phase(8);

      if (!have_candidate) continue;
      ++stats_.preambles;
      ++stats_.frames;
      const uint8_t df = best.bytes[0] >> 3;
      if (df == 17) ++stats_.df17;
      ++stats_.crc_ok;
      if (callback) callback(best, context);
      /* Skip most of the message so we don't re-decode it at every bit
       * offset (readsb: pa += msglen * 8 / 4). */
      pos += (best_bit_length == 112 ? 224u : 112u) - 1;
    }
  }

  overlap_ = std::min(count, kTrailingSamples);
  std::memmove(magnitudes_, magnitudes_ + count - overlap_, overlap_ * sizeof(uint16_t));
}

bool Decoder::self_check() {
  const uint8_t identity[] = {0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3,
                              0x71, 0xc3, 0x2c, 0xe0, 0x57, 0x60, 0x98};
  const uint8_t position[] = {0x8d, 0x40, 0x62, 0x1d, 0x58, 0xc3, 0x82,
                              0xd6, 0x90, 0xc8, 0xac, 0x28, 0x63, 0xa7};
  const uint8_t odd_position[] = {0x8d, 0x40, 0x62, 0x1d, 0x58, 0xc3, 0x86,
                                  0x43, 0x5c, 0xc4, 0x12, 0x69, 0x2a, 0xd6};
  const uint8_t velocity[] = {0x8d, 0x45, 0x1d, 0xbd, 0x99, 0x05, 0xb5,
                              0x01, 0x80, 0x04, 0x00, 0x59, 0x79, 0xc5};
  // Live OrcSDR RF capture, 2026-08-11: ASA1310 / ICAO A29551 at 35,000 ft.
  const uint8_t live_position[] = {0x8d, 0xa2, 0x95, 0x51, 0x58, 0xb5, 0x05,
                                   0x03, 0x6b, 0xfb, 0x54, 0xbc, 0x90, 0xac};
  uint8_t all_call[] = {0x5d, 0x48, 0x40, 0xd6, 0, 0, 0};
  const uint32_t all_call_crc = crc_payload(all_call, 32);
  all_call[4] = static_cast<uint8_t>(all_call_crc >> 16);
  all_call[5] = static_cast<uint8_t>(all_call_crc >> 8);
  all_call[6] = static_cast<uint8_t>(all_call_crc);
  Frame a{}, b{}, c{}, odd{}, live{}, short_reply{};
  double latitude = 0.0, longitude = 0.0;
  if (!(parse(identity, 112, 1, &a) && a.icao == 0x4840d6 && a.has_callsign &&
         std::strcmp(a.callsign, "KLM1023") == 0 && parse(position, 112, 1, &b) &&
         b.has_altitude && b.altitude_ft == 38000 && parse(velocity, 112, 1, &c) &&
         c.icao == 0x451dbd && c.has_speed && c.speed_kts == 436 &&
         c.has_heading && c.heading_deg == 271 && parse(odd_position, 112, 1, &odd) &&
         decode_global_cpr(b, odd, true, &latitude, &longitude) &&
         std::abs(latitude - 52.2658) < 0.001 && std::abs(longitude - 3.9389) < 0.001 &&
         parse(live_position, 112, 1, &live) &&
         live.icao == 0xa29551 && live.has_altitude && live.altitude_ft == 35000 &&
         parse(all_call, 56, 1, &short_reply) && short_reply.bit_length == 56 &&
         short_reply.icao == 0x4840d6))
    return false;
  /* The IQ->bits->frame path is validated offline against a real 30 s
   * 2.4 MSPS capture (650 CRC / 289 distinct aircraft vs readsb 883 / 419);
   * see verify_port harness + docs/LEARNING.md L28. It is deliberately not
   * re-run here: a synthetic-waveform test needs bit-exact sample alignment
   * with the 5-phase slicer and would make boot self_check fragile.
   * Below are the deterministic parse/CRC/CPR unit checks. */
  return true;
}

}  // namespace adsb_radar::adsb_rx
