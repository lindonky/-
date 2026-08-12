idf_component_register(
    SRCS "thruster.c" "cmd_parser.c" "attitude_control.c" "stabilize.c" "control_mode.c" "training_session.c" "calibration_capture.c"
    INCLUDE_DIRS "."
    REQUIRES bsp esp_timer
)
