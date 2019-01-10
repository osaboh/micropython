#define MICROPY_HW_BOARD_NAME       "PYBv1.1"
#define MICROPY_HW_MCU_NAME         "STM32F405RG"

#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_USB       (0)

// HSE is 12MHz
#define MICROPY_HW_CLK_PLLM (12)
#define MICROPY_HW_CLK_PLLN (336)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ (7)
#define MICROPY_HW_CLK_LAST_FREQ (1)

// The pyboard has a 32kHz crystal for the RTC

// UART config
#define MICROPY_HW_UART6_NAME   "YA"
#define MICROPY_HW_UART6_TX     (pin_C6)
#define MICROPY_HW_UART6_RX     (pin_C7)

// I2C busses

// SPI busses

// CAN busses

// USRSW has no pullup or pulldown, and pressing the switch makes the input go low

// The pyboard has 4 LEDs

// SD card detect switch

// USB config

// MMA accelerometer config

// pyboard uart repl
#define MICROPY_HW_UART_REPL       PYB_UART_6
#define MICROPY_HW_UART_REPL_BAUD  115200
