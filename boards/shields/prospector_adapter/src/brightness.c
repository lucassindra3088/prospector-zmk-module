#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdlib.h>
#include <display_power.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(als, 4);

static const struct device *pwm_leds_dev = DEVICE_DT_GET_ONE(pwm_leds);
#define DISP_BL DT_NODE_CHILD_IDX(DT_NODELABEL(disp_bl))

static uint8_t current_brightness = 0;

// --- Threaded fade system ---

struct fade_request_t {
    uint8_t from;
    uint8_t to;
};

#define FADE_QUEUE_SIZE 4
K_MSGQ_DEFINE(fade_msgq, sizeof(struct fade_request_t), FADE_QUEUE_SIZE, 4);

static float ease_in_out(float t) {
    if (t < 0.5f)
        return 4.0f * t * t * t;
    float f = -2.0f * t + 2.0f;
    return 1.0f - (f * f * f) / 2.0f;
}

static void apply_brightness(uint8_t value) {
    led_set_brightness(pwm_leds_dev, DISP_BL, value);
    current_brightness = value;
}

void fade_thread(void) {
    struct fade_request_t req;

    while (1) {
        if (k_msgq_get(&fade_msgq, &req, K_FOREVER) == 0) {
            if (req.from == req.to || abs(req.to - req.from) <= 1) {
                apply_brightness(req.to);
                continue;
            }

            int diff = abs(req.to - req.from);
            int steps = CLAMP(diff * 2, 6, 32);
            int total_duration_ms = CLAMP(diff * 20, 500, 1000);
            int delay_us = (total_duration_ms * 1000) / steps;

            uint8_t last_applied = 255;

            for (int i = 0; i <= steps; i++) {
                float t = (float)i / steps;
                float eased = ease_in_out(t);
                float interpolated = req.from + (req.to - req.from) * eased;
                uint8_t brightness = (uint8_t)(interpolated + 0.5f);

                if (brightness != last_applied) {
                    apply_brightness(brightness);
                    last_applied = brightness;
                }

                k_usleep(delay_us);
            }

            if (last_applied != req.to) {
                apply_brightness(req.to);
            }
        }
    }
}

K_THREAD_DEFINE(fade_tid, 768, fade_thread, NULL, NULL, NULL, 6, 0, 0);

static void fade_to_brightness(uint8_t from, uint8_t to) {
    struct fade_request_t req = {.from = from, .to = to};
    k_msgq_purge(&fade_msgq);
    k_msgq_put(&fade_msgq, &req, K_NO_WAIT);
}

// --- Public API for display_idle.c ---

void prospector_brightness_fade_off(void) {
    fade_to_brightness(current_brightness, 0);
}

void prospector_brightness_fade_on(uint8_t target) {
    fade_to_brightness(0, target);
}

uint8_t prospector_brightness_get_current(void) {
    return current_brightness;
}

// --- ALS or fixed brightness ---

#ifdef CONFIG_PROSPECTOR_USE_AMBIENT_LIGHT_SENSOR

#define SENSOR_MIN      0
#define SENSOR_MAX      100
#define PWM_MIN         15
#define PWM_MAX         100

#define FADE_THRESHOLD                   10
#define NORMAL_SAMPLE_SLEEP_MS           100
#define BURST_SAMPLE_SLEEP_MS            30
#define BURST_SAMPLE_TIMEOUT             10
#define BURST_SAMPLE_CONSECUTIVE         3

uint8_t map_light_to_pwm(int32_t sensor_reading) {
    if (sensor_reading < SENSOR_MIN) {
        return PWM_MIN;
    }
    if (sensor_reading > SENSOR_MAX) {
        sensor_reading = SENSOR_MAX;
    }
    uint8_t pwm_value = (uint8_t)(
        PWM_MIN + ((PWM_MAX - PWM_MIN) *
        (sensor_reading - SENSOR_MIN)) / (SENSOR_MAX - SENSOR_MIN)
    );
    return pwm_value;
}

extern void als_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    const struct device *dev;
    struct sensor_value intensity;
    uint8_t mapped_brightness;

    dev = DEVICE_DT_GET_ONE(avago_apds9960);
    if (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
    }

    while (1) {
        if (prospector_display_is_sleeping()) {
            if (current_brightness != 0) {
                apply_brightness(0);
            }
            k_msleep(NORMAL_SAMPLE_SLEEP_MS);
            continue;
        }

        k_msleep(NORMAL_SAMPLE_SLEEP_MS);

        if (sensor_sample_fetch(dev)) {
            LOG_ERR("sensor_sample fetch failed\n");
        }
        if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
            LOG_ERR("Cannot read ALS data.\n");
        }

        mapped_brightness = map_light_to_pwm(intensity.val1);

        if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
            uint8_t integrator = 0;

            for (int i = 0; i < BURST_SAMPLE_TIMEOUT; i++) {
                k_msleep(BURST_SAMPLE_SLEEP_MS);

                if (sensor_sample_fetch(dev)) {
                    LOG_ERR("sensor_sample fetch failed\n");
                }
                if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &intensity)) {
                    LOG_ERR("Cannot read ALS data.\n");
                }

                mapped_brightness = map_light_to_pwm(intensity.val1);

                if (abs(mapped_brightness - current_brightness) > FADE_THRESHOLD) {
                    integrator++;
                    if (integrator >= BURST_SAMPLE_CONSECUTIVE) {
                        fade_to_brightness(current_brightness, mapped_brightness);
                        break;
                    }
                }
            }
        }
    }
}

K_THREAD_DEFINE(als_tid, 1024, als_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

#else

static int init_fixed_brightness(void) {
    fade_to_brightness(0, CONFIG_PROSPECTOR_FIXED_BRIGHTNESS);
    return 0;
}

SYS_INIT(init_fixed_brightness, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
