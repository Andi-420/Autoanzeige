#pragma once
#include <cstdint>
typedef enum { ESP_SLEEP_WAKEUP_UNDEFINED = 0, ESP_SLEEP_WAKEUP_EXT0 = 2, ESP_SLEEP_WAKEUP_TIMER = 4 } esp_sleep_wakeup_cause_t;
typedef int gpio_num_t;
#define GPIO_NUM_6  6
#define GPIO_NUM_16 16
#define RTC_DATA_ATTR
inline esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause() { return ESP_SLEEP_WAKEUP_UNDEFINED; }
inline int esp_sleep_enable_timer_wakeup(uint64_t) { return 0; }
inline int esp_sleep_enable_ext0_wakeup(gpio_num_t, int) { return 0; }
inline void esp_deep_sleep_start() {}
