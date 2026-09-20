idf_component_register(
    SRCS "src/adsb_decoder.cpp" "src/adsb_decoder_bridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES log esp_timer
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)