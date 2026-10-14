idf_component_register(
    SRCS "src/adsb_decoder.cpp" "src/adsb_decoder_bridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES log esp_timer
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)

# The demodulator is a per-sample DSP hot loop (magnitude + preamble scan for
# every one of ~2.4 M samples/s). The project default build is -Og
# (CONFIG_COMPILER_OPTIMIZATION_DEBUG=y), which is far too slow for it and
# leaves adsb_dsp CPU-saturated, so IQ is dropped before it is decoded
# (ring_free pinned at 0). Optimize just this component - no global change,
# no assertions/size trade-off elsewhere.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)