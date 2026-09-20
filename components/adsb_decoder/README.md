# adsb_decoder component

Mode-S / ADS-B decoder for 2.048 MSPS CU8 IQ stream.
Decodes DF17 (112-bit) and DF11 (56-bit), including callsign (TC 1-4),
altitude + CPR (TC 9-18), velocity (TC 19), and global CPR position pairing.

## Provenance

- Files: `adsb_decoder.hpp`, `adsb_decoder.cpp`
- Source: [hardcoreerik/OrcSDR](https://github.com/hardcoreerik/OrcSDR) `apps/orcsdr-tab5/ui/`,
  pinned at commit `52edd9b` (matching the OrcSDR repository used as reference).
- Original namespace: `orcsdr::adsb_rx`. Renamed to `adsb_radar::adsb_rx`.

## License

AGPL-3.0 (as in the upstream OrcSDR project). Reused verbatim apart from the
namespace rename; attribution retained. Swap policy pending (D1).