idf_component_register(
    SRCS "bsp.c" "driver/pwm_channel.c" "driver/uart_console.c" "driver/jy61p.c" "driver/imu_uart.c"
    INCLUDE_DIRS "." "driver"
    REQUIRES driver vfs
)
