/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <math.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/led_strip.h>
#include <drivers/ext_power.h>

#include <zmk/rgb_underglow.h>

#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/workqueue.h>
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/hid_indicators.h>
#endif
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/battery.h>
#include <zmk/keymap.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>

#if ZMK_BLE_IS_CENTRAL
#include <zmk/split/bluetooth/central.h>
#else
#include <zmk/split/bluetooth/peripheral.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
#include <zmk/split/bluetooth/central.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)

#error "A zmk,underglow chosen node must be declared"

#endif

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)

#define HUE_MAX 360
#define SAT_MAX 100
#define BRT_MAX 100

BUILD_ASSERT(CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN <= CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX,
             "ERROR: RGB underglow maximum brightness is less than minimum brightness");

enum rgb_underglow_effect {
    UNDERGLOW_EFFECT_SOLID,
    UNDERGLOW_EFFECT_BREATHE,
    UNDERGLOW_EFFECT_SPECTRUM,
    UNDERGLOW_EFFECT_SWIRL,
    UNDERGLOW_EFFECT_KINESIS,
    UNDERGLOW_EFFECT_BATTERY,
    UNDERGLOW_EFFECT_TEST,
    UNDERGLOW_EFFECT_NUMBER // Used to track number of underglow effects
};

struct rgb_underglow_state {
    struct zmk_led_hsb color;
    uint8_t animation_speed;
    uint8_t current_effect;
    uint16_t animation_step;
    bool on;
    bool status_active;
    uint16_t status_animation_step;
};

static const struct device *led_strip;

static struct led_rgb pixels[STRIP_NUM_PIXELS];
static struct led_rgb status_pixels[STRIP_NUM_PIXELS];

static struct rgb_underglow_state state;

static struct zmk_periph_led led_data;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
static bool last_ble_state[2];
#endif

static bool triggered;

#if ZMK_BLE_IS_CENTRAL
static struct zmk_periph_led old_led_data;
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *const ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
#endif

static struct zmk_led_hsb hsb_scale_min_max(struct zmk_led_hsb hsb) {
    hsb.b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN +
            (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX - CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN) * hsb.b / BRT_MAX;
    return hsb;
}

static struct zmk_led_hsb hsb_scale_zero_max(struct zmk_led_hsb hsb) {
    hsb.b = hsb.b * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / BRT_MAX;
    return hsb;
}

static struct led_rgb hsb_to_rgb(struct zmk_led_hsb hsb) {
    float r = 0, g = 0, b = 0;

    uint8_t i = hsb.h / 60;
    float v = hsb.b / ((float)BRT_MAX);
    float s = hsb.s / ((float)SAT_MAX);
    float f = hsb.h / ((float)HUE_MAX) * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    }

    struct led_rgb rgb = {r : r * 255, g : g * 255, b : b * 255};

    return rgb;
}

#define LED_RGB_SCALING_MULTIPLE (((float)CONFIG_ZMK_RGB_UNDERGLOW_BRT_SCALE) / 250.)

#define LED_RGB(hex)                                                                               \
    ((struct led_rgb){                                                                             \
        r : LED_RGB_SCALING_MULTIPLE * (((hex) & 0xFF0000) >> 16),                                 \
        g : LED_RGB_SCALING_MULTIPLE * (((hex) & 0x00FF00) >> 8),                                  \
        b : LED_RGB_SCALING_MULTIPLE * (((hex) & 0x0000FF) >> 0)                                   \
    })

int zmk_rgb_underglow_set_periph(struct zmk_periph_led periph) {
    led_data = periph;
    if (!state.on && led_data.on)
        zmk_rgb_underglow_on();
    else if (state.on && !led_data.on)
        zmk_rgb_underglow_off();

    state.current_effect = led_data.effect;
    LOG_DBG("Update led_data %d %d %d", led_data.layer, led_data.indicators, led_data.on);
    return 0;
}

static void zmk_rgb_underglow_effect_solid(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
    }
}

static void zmk_rgb_underglow_effect_breathe(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.b = abs(state.animation_step - 1200) / 12;

        pixels[i] = hsb_to_rgb(hsb_scale_zero_max(hsb));
    }

    state.animation_step += state.animation_speed * 10;

    if (state.animation_step > 2400) {
        state.animation_step = 0;
    }
}

static void zmk_rgb_underglow_effect_spectrum(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed;
    state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (HUE_MAX / STRIP_NUM_PIXELS * i + state.animation_step) % HUE_MAX;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed * 2;
    state.animation_step = state.animation_step % HUE_MAX;
}

#if ZMK_BLE_IS_CENTRAL
static struct k_work_delayable led_update_work;

static void zmk_rgb_underglow_central_send() {
    int err = zmk_split_bt_update_led(&led_data);
    if (err) {
        LOG_ERR("send failed (err %d)", err);
    }
}

#define NUM_BT_COLORS 4

static const struct led_rgb BT_COLORS[NUM_BT_COLORS] = {LED_RGB(0xFFFFFF), LED_RGB(0x0000FF),
                                                        LED_RGB(0xFF0000), LED_RGB(0x00FF00)};
#endif

static const struct led_rgb LAYER_COLORS[8] = {
    LED_RGB(0x000000), LED_RGB(0xFFFFFF), LED_RGB(0x0000FF), LED_RGB(0x00FF00),
    LED_RGB(0xFF0000), LED_RGB(0xFF00FF), LED_RGB(0x00FFFF), LED_RGB(0xFFFF00)};

// Formulas chosen so that for the first 8 layers both left and right modules show the same color,
// then as the layer number increases the right module color cycles through until "wrapping around",
// at which point the left module colour is advanced by one as well. We skip over the off/black
// state while we do this. (The right module also skips over the current left module color each
// loop, since those combinations correspond to the first 8 layers.)
static void zmk_led_layer_to_colors(uint8_t layer, uint8_t *left, uint8_t *right) {
    if (layer < 8) {
        *left = layer;
        *right = layer;
        return;
    }

    *left = 1 + ((layer - 8) / 6);
    *right = 1 + ((layer - 8) % 6);

    if (*left <= *right)
        *right += 1;
}

static bool zmk_kinesis_blink_step(uint8_t idx, uint8_t limit) {
    state.animation_step++;
    if (state.animation_step > limit) {
        last_ble_state[idx] = !last_ble_state[idx];
        state.animation_step = 0;
    }

    return !last_ble_state[idx];
}

static struct led_rgb zmk_get_indicator_color(zmk_hid_indicators_t bit) {
    return led_data.indicators & bit ? LED_RGB(CONFIG_ZMK_RGB_UNDERGLOW_MOD_COLOR)
                                     : LED_RGB(0x000000);
}

static void zmk_rgb_underglow_effect_kinesis() {
#if ZMK_BLE_IS_CENTRAL
    // update state and propagate to peripheral if necessary
    old_led_data.layer = led_data.layer;
    old_led_data.indicators = led_data.indicators;
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    led_data.indicators = zmk_hid_indicators_get_current_profile();
#else
    led_data.indicators = 0;
#endif
    led_data.layer = zmk_keymap_highest_layer_active();

    if (old_led_data.layer != led_data.layer || old_led_data.indicators != led_data.indicators) {
        zmk_rgb_underglow_central_send();
    }
#endif

    bool bt_blinking = false;

    uint8_t layer_color_left, layer_color_right;
    zmk_led_layer_to_colors(led_data.layer, &layer_color_left, &layer_color_right);

#if UNDERGLOW_INDICATORS_ENABLED
    // Use configurable positions when available
    
#if ZMK_BLE_IS_CENTRAL
    // leds for central (left) side
    
    // Caps lock indicator
    if (DT_NODE_HAS_PROP(UNDERGLOW_INDICATORS, capslock)) {
        pixels[DT_PROP(UNDERGLOW_INDICATORS, capslock)] = zmk_get_indicator_color(ZMK_LED_CAPSLOCK_BIT);
    } else {
        pixels[0] = zmk_get_indicator_color(ZMK_LED_CAPSLOCK_BIT);
    }
    
    // Bluetooth state indicator
    int bt_idx = zmk_ble_active_profile_index();
    if (zmk_ble_active_profile_is_open()) {
        bt_blinking = zmk_kinesis_blink_step(0, 2);
    } else if (!zmk_ble_active_profile_is_connected()) {
        bt_blinking = zmk_kinesis_blink_step(1, 13);
    }
    
    // Use first BLE state position for active profile
    if (DT_PROP_LEN(UNDERGLOW_INDICATORS, ble_state) > 0) {
        pixels[underglow_ble_state[0]] = (bt_idx < NUM_BT_COLORS && !bt_blinking) ? BT_COLORS[bt_idx] : LED_RGB(0x000000);
    } else {
        pixels[1] = (bt_idx < NUM_BT_COLORS && !bt_blinking) ? BT_COLORS[bt_idx] : LED_RGB(0x000000);
    }
    
    // Layer state - use first position
    if (DT_PROP_LEN(UNDERGLOW_INDICATORS, layer_state) > 0) {
        pixels[underglow_layer_state[0]] = LAYER_COLORS[layer_color_left];
    } else {
        pixels[2] = LAYER_COLORS[layer_color_left];
    }
#else
    // leds for peripheral (right) side
    
    // Num lock and scroll lock indicators
    if (DT_NODE_HAS_PROP(UNDERGLOW_INDICATORS, numlock)) {
        pixels[DT_PROP(UNDERGLOW_INDICATORS, numlock)] = zmk_get_indicator_color(ZMK_LED_NUMLOCK_BIT);
    } else {
        pixels[2] = zmk_get_indicator_color(ZMK_LED_NUMLOCK_BIT);
    }
    
    if (DT_NODE_HAS_PROP(UNDERGLOW_INDICATORS, scrolllock)) {
        pixels[DT_PROP(UNDERGLOW_INDICATORS, scrolllock)] = zmk_get_indicator_color(ZMK_LED_SCROLLLOCK_BIT);
    } else {
        pixels[1] = zmk_get_indicator_color(ZMK_LED_SCROLLLOCK_BIT);
    }
    
    // Layer state position
    if (DT_PROP_LEN(UNDERGLOW_INDICATORS, layer_state) > 0) {
        pixels[underglow_layer_state[0]] = LAYER_COLORS[layer_color_right];
    } else {
        pixels[0] = LAYER_COLORS[layer_color_right];
    }
#endif

#else
    // Fallback to hardcoded positions when indicators not configured
    
#if ZMK_BLE_IS_CENTRAL
    // leds for central (left) side

    // set first led to caps lock state
    pixels[0] = zmk_get_indicator_color(ZMK_LED_CAPSLOCK_BIT);

    // set second led to bluetooth state, blinking quickly if bluetooth not paired,
    // and slowly if not connected
    int bt_idx = zmk_ble_active_profile_index();
    if (zmk_ble_active_profile_is_open()) {
        bt_blinking = zmk_kinesis_blink_step(0, 2);
    } else if (!zmk_ble_active_profile_is_connected()) {
        bt_blinking = zmk_kinesis_blink_step(1, 13);
    }
    pixels[1] = (bt_idx < NUM_BT_COLORS && !bt_blinking) ? BT_COLORS[bt_idx] : LED_RGB(0x000000);

    // set third led to layer state
    pixels[2] = LAYER_COLORS[layer_color_left];
#else
    // leds for peripheral (right) side

    // set first and second leds to num lock and scroll lock state, respectively
    pixels[2] = zmk_get_indicator_color(ZMK_LED_NUMLOCK_BIT);
    pixels[1] = zmk_get_indicator_color(ZMK_LED_SCROLLLOCK_BIT);

    // set third led to layer state
    pixels[0] = LAYER_COLORS[layer_color_right];
#endif

#endif // UNDERGLOW_INDICATORS_ENABLED

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && !ZMK_BLE_IS_CENTRAL
    bool bt_alert = false;
    if (!zmk_split_bt_peripheral_is_bonded()) {
        bt_alert = true;
        bt_blinking = zmk_kinesis_blink_step(0, 2);
    } else if (!zmk_split_bt_peripheral_is_connected()) {
        bt_alert = true;
        bt_blinking = zmk_kinesis_blink_step(1, 13);
    }

    if (bt_alert) {
        // override all leds to blinking red due to bluetooth problem
        struct led_rgb color = bt_blinking ? LED_RGB(0x000000) : LED_RGB(0xFF0000);
        for (int i = 0; i < STRIP_NUM_PIXELS; i++)
            pixels[i] = color;
    }
#endif
}

static void zmk_rgb_underglow_effect_test() {
    triggered = true;
    struct led_rgb rgb;
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 0;

    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
    if (state.animation_step < (HUE_MAX * 3)) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;
        rgb.r = 0;

        pixels[0] = rgb;
        pixels[1] = rgb;
        pixels[2] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
    if (state.animation_step < (HUE_MAX * 2)) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step - HUE_MAX;
        rgb.r = 0;
        rgb.g = 0;
        rgb.b = 0;
        pixels[0] = rgb;
        pixels[1] = hsb_to_rgb(hsb_scale_min_max(hsb));
        pixels[2] = rgb;
    }
    if (state.animation_step < HUE_MAX) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;
        rgb.r = 0;
        rgb.g = 0;
        rgb.b = 0;
        pixels[0] = hsb_to_rgb(hsb_scale_min_max(hsb));
        pixels[1] = rgb;
        pixels[2] = rgb;
    }

    state.animation_step += 20;
    if (state.animation_step > (HUE_MAX * 3)) {
        rgb.r = 255;
        rgb.g = 255;
        rgb.b = 255;
        for (int i = 0; i < STRIP_NUM_PIXELS; i++)
            pixels[i] = rgb;
    }
}

#define NUM_BATTERY_LEVELS 3

static const uint8_t BATTERY_LEVELS[NUM_BATTERY_LEVELS] = {80, 50, 20};

static const struct led_rgb BATTERY_COLORS[NUM_BATTERY_LEVELS + 1] = {
    LED_RGB(0x00FF00), LED_RGB(0xFFFF00), LED_RGB(0xFF8C00), LED_RGB(0xFF0000)};

static void zmk_rgb_underglow_effect_battery() {
    uint8_t soc = zmk_battery_state_of_charge();

    int color = 0;
    for (; color < NUM_BATTERY_LEVELS && soc < BATTERY_LEVELS[color]; color++)
        ;

    struct led_rgb rgb = BATTERY_COLORS[color];
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = rgb;
    }
}

// RGB underglow status support
#define UNDERGLOW_INDICATORS DT_PATH(underglow_indicators)

#if defined(DT_N_S_underglow_indicators_EXISTS)
#define UNDERGLOW_INDICATORS_ENABLED 1
#else
#define UNDERGLOW_INDICATORS_ENABLED 0
#endif

#if UNDERGLOW_INDICATORS_ENABLED
const uint8_t underglow_layer_state[] = DT_PROP(UNDERGLOW_INDICATORS, layer_state);
const uint8_t underglow_ble_state[] = DT_PROP(UNDERGLOW_INDICATORS, ble_state);
const uint8_t underglow_bat_lhs[] = DT_PROP(UNDERGLOW_INDICATORS, bat_lhs);
const uint8_t underglow_bat_rhs[] = DT_PROP(UNDERGLOW_INDICATORS, bat_rhs);

#define HEXRGB(R, G, B)                                                                            \
    ((struct led_rgb){                                                                             \
        r : (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX * (R)) / 0xff,                                       \
        g : (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX * (G)) / 0xff,                                       \
        b : (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX * (B)) / 0xff                                        \
    })

const struct led_rgb red = HEXRGB(0xff, 0x00, 0x00);
const struct led_rgb yellow = HEXRGB(0xff, 0xff, 0x00);
const struct led_rgb green = HEXRGB(0x00, 0xff, 0x00);
const struct led_rgb dull_green = HEXRGB(0x00, 0xff, 0x68);
const struct led_rgb magenta = HEXRGB(0xff, 0x00, 0xff);
const struct led_rgb white = HEXRGB(0xff, 0xff, 0xff);
const struct led_rgb lilac = HEXRGB(0x6b, 0x1f, 0xce);

static void zmk_led_battery_level(int bat_level, const uint8_t *addresses, size_t addresses_len) {
    struct led_rgb bat_colour;

    if (bat_level > 40) {
        bat_colour = green;
    } else if (bat_level > 20) {
        bat_colour = yellow;
    } else {
        bat_colour = red;
    }

    for (int i = 0; i < addresses_len; i++) {
        int min_level = (i * 100) / (addresses_len - 1);
        if (bat_level >= min_level) {
            status_pixels[addresses[i]] = bat_colour;
        }
    }
}

static void zmk_led_fill(struct led_rgb color, const uint8_t *addresses, size_t addresses_len) {
    for (int i = 0; i < addresses_len; i++) {
        status_pixels[addresses[i]] = color;
    }
}

static int zmk_led_generate_status(void) {
    // Clear all status pixels
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        status_pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    // BATTERY STATUS
    zmk_led_battery_level(zmk_battery_state_of_charge(), underglow_bat_lhs,
                          DT_PROP_LEN(UNDERGLOW_INDICATORS, bat_lhs));

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    uint8_t peripheral_level = 0;
    int rc = zmk_split_get_peripheral_battery_level(0, &peripheral_level);

    if (rc == 0) {
        zmk_led_battery_level(peripheral_level, underglow_bat_rhs,
                              DT_PROP_LEN(UNDERGLOW_INDICATORS, bat_rhs));
    } else if (rc == -ENOTCONN) {
        zmk_led_fill(red, underglow_bat_rhs, DT_PROP_LEN(UNDERGLOW_INDICATORS, bat_rhs));
    } else if (rc == -EINVAL) {
        LOG_ERR("Invalid peripheral index requested for battery level read: 0");
    }
#endif

    // CAPSLOCK/NUMLOCK/SCROLLOCK STATUS
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    zmk_hid_indicators_t led_flags = zmk_hid_indicators_get_current_profile();

    if (led_flags & ZMK_LED_CAPSLOCK_BIT)
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, capslock)] = red;
    if (led_flags & ZMK_LED_NUMLOCK_BIT)
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, numlock)] = red;
    if (led_flags & ZMK_LED_SCROLLLOCK_BIT)
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, scrolllock)] = red;
#endif

    // LAYER STATUS
    for (uint8_t i = 0; i < DT_PROP_LEN(UNDERGLOW_INDICATORS, layer_state); i++) {
        if (zmk_keymap_layer_active(i))
            status_pixels[underglow_layer_state[i]] = magenta;
    }

    // CONNECTION STATUS
    struct zmk_endpoint_instance active_endpoint = zmk_endpoints_selected();

    if (!zmk_endpoints_preferred_transport_is_active())
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, output_fallback)] = red;

    int active_ble_profile_index = zmk_ble_active_profile_index();
    for (uint8_t i = 0;
         i < MIN(ZMK_BLE_PROFILE_COUNT, DT_PROP_LEN(UNDERGLOW_INDICATORS, ble_state)); i++) {
        int8_t status = zmk_ble_profile_status(i);
        int ble_pixel = underglow_ble_state[i];
        if (status == 2 && active_endpoint.transport == ZMK_TRANSPORT_BLE &&
            active_ble_profile_index == i) { // connected AND active
            status_pixels[ble_pixel] = white;
        } else if (status == 2) { // connected - not active
            status_pixels[ble_pixel] = dull_green;
        } else if (status == 1) { // paired
            status_pixels[ble_pixel] = dull_green;
        } else if (status == 0) { // disconnected
            status_pixels[ble_pixel] = red;
        }
    }

    if (active_endpoint.transport == ZMK_TRANSPORT_USB) {
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, usb_state)] = white;
    }

    // Calculate blend value based on animation step
    int blend;
    if (state.status_animation_step < (500 / 25)) {
        // fade in for 500ms
        blend = ((state.status_animation_step * 256) / (500 / 25));
    } else if (state.status_animation_step < (8000 / 25)) {
        // full for 7500ms  
        blend = 256;
    } else if (state.status_animation_step < (10000 / 25)) {
        // fade out for 2000ms
        blend = 256 - (((state.status_animation_step - (8000 / 25)) * 256) / (2000 / 25));
    } else {
        // done
        state.status_active = false;
        state.status_animation_step = 0;
        blend = 0;
        
        // Turn off timer and external power if main underglow is off
        if (!state.on) {
            k_timer_stop(&underglow_tick);
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
            if (ext_power != NULL) {
                int rc = ext_power_disable(ext_power);
                if (rc != 0) {
                    LOG_ERR("Unable to disable EXT_POWER after status: %d", rc);
                }
            }
#endif
        }
    }

    return blend;
}
#else
static int zmk_led_generate_status(void) { return 0; }
#endif

static void zmk_led_write_pixels(void) {
    static struct led_rgb led_buffer[STRIP_NUM_PIXELS];
    int bat_level = zmk_battery_state_of_charge();
    int blend = 0;
    
    if (state.status_active) {
        blend = zmk_led_generate_status();
        state.status_animation_step++;
    }

    // Fast path: no status indicators, battery level OK
    if (blend == 0 && bat_level >= 20) {
        int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
        if (err < 0) {
            LOG_ERR("Failed to update the RGB strip (%d)", err);
        }
        return;
    }

    // Apply battery dimming and/or status blending
    if (blend == 0) {
        // Just copy pixels, battery dimming will be applied below
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i] = pixels[i];
        }
    } else if (blend >= 256) {
        // Full status display
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i] = status_pixels[i];
        }
    } else {
        // Blend status with regular effect
        uint16_t blend_l = blend;
        uint16_t blend_r = 256 - blend;
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i].r = ((status_pixels[i].r * blend_l) >> 8) + ((pixels[i].r * blend_r) >> 8);
            led_buffer[i].g = ((status_pixels[i].g * blend_l) >> 8) + ((pixels[i].g * blend_r) >> 8);
            led_buffer[i].b = ((status_pixels[i].b * blend_l) >> 8) + ((pixels[i].b * blend_r) >> 8);
        }
    }

    // Battery level dimming applies to the final output
    if (bat_level < 10) {
        memset(led_buffer, 0, sizeof(struct led_rgb) * STRIP_NUM_PIXELS);
    } else if (bat_level < 20) {
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i].r = led_buffer[i].r >> 1;
            led_buffer[i].g = led_buffer[i].g >> 1;
            led_buffer[i].b = led_buffer[i].b >> 1;
        }
    }

    int err = led_strip_update_rgb(led_strip, led_buffer, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

static void zmk_rgb_underglow_tick(struct k_work *work) {
    switch (state.current_effect) {
    case UNDERGLOW_EFFECT_SOLID:
        zmk_rgb_underglow_effect_solid();
        break;
    case UNDERGLOW_EFFECT_BREATHE:
        zmk_rgb_underglow_effect_breathe();
        break;
    case UNDERGLOW_EFFECT_SPECTRUM:
        zmk_rgb_underglow_effect_spectrum();
        break;
    case UNDERGLOW_EFFECT_SWIRL:
        zmk_rgb_underglow_effect_swirl();
        break;
    case UNDERGLOW_EFFECT_KINESIS:
        zmk_rgb_underglow_effect_kinesis();
        break;
    case UNDERGLOW_EFFECT_BATTERY:
        zmk_rgb_underglow_effect_battery();
        break;
    case UNDERGLOW_EFFECT_TEST:
        zmk_rgb_underglow_effect_test();
        break;
    }

    // Call the blending function
    zmk_led_write_pixels();
}

K_WORK_DEFINE(underglow_tick_work, zmk_rgb_underglow_tick);

static void zmk_rgb_underglow_tick_handler(struct k_timer *timer) {
    if (!state.on && !state.status_active) {
        return;
    }

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_tick_work);
}

K_TIMER_DEFINE(underglow_tick, zmk_rgb_underglow_tick_handler, NULL);

int zmk_rgb_underglow_save_state(void) { return 0; }

static int zmk_rgb_underglow_init(void) {
    led_strip = DEVICE_DT_GET(STRIP_CHOSEN);

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (!device_is_ready(ext_power)) {
        LOG_ERR("External power device \"%s\" is not ready", ext_power->name);
        return -ENODEV;
    }
#endif

    state = (struct rgb_underglow_state){
        color : {
            h : CONFIG_ZMK_RGB_UNDERGLOW_HUE_START,
            s : CONFIG_ZMK_RGB_UNDERGLOW_SAT_START,
            b : CONFIG_ZMK_RGB_UNDERGLOW_BRT_START,
        },
        animation_speed : CONFIG_ZMK_RGB_UNDERGLOW_SPD_START,
        current_effect : CONFIG_ZMK_RGB_UNDERGLOW_EFF_START,
        animation_step : 0,
        on : IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START)
    };
    led_data.indicators = 0;
    led_data.on = IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START);
    led_data.effect = state.current_effect;

#if ZMK_BLE_IS_CENTRAL
    k_work_init_delayable(&led_update_work, zmk_rgb_underglow_central_send);
#endif

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_tick_work);
    zmk_rgb_underglow_off();
    if (IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START)) {
        ext_power_enable(ext_power);
        zmk_rgb_underglow_on();
    }
    triggered = false;
    return 0;
}

int zmk_rgb_underglow_get_state(bool *on_off) {
    if (!led_strip)
        return -ENODEV;

    *on_off = state.on;
    return 0;
}

int zmk_rgb_underglow_on(void) {
    if (!led_strip)
        return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL) {
        int rc = ext_power_enable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to enable EXT_POWER: %d", rc);
        }
    }
#endif

    state.on = true;
    state.animation_step = 0;
    k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(25));

#if ZMK_BLE_IS_CENTRAL
    led_data.on = true;
    zmk_rgb_underglow_central_send();
#endif

    return 0;
}

static void zmk_rgb_underglow_off_handler(struct k_work *work) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
}

K_WORK_DEFINE(underglow_off_work, zmk_rgb_underglow_off_handler);

int zmk_rgb_underglow_off(void) {
    if (!led_strip)
        return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL) {
        int rc = ext_power_disable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to disable EXT_POWER: %d", rc);
        }
    }
#endif

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_off_work);

    k_timer_stop(&underglow_tick);
    state.on = false;
#if ZMK_BLE_IS_CENTRAL
    led_data.on = false;
    zmk_rgb_underglow_central_send();
#endif
    return 0;
}

int zmk_rgb_underglow_calc_effect(int direction) {
    return (state.current_effect + UNDERGLOW_EFFECT_NUMBER + direction) % UNDERGLOW_EFFECT_NUMBER;
}

int zmk_rgb_underglow_select_effect(int effect) {
    if (!led_strip)
        return -ENODEV;

    if (effect < 0 || effect >= UNDERGLOW_EFFECT_NUMBER) {
        return -EINVAL;
    }

    state.current_effect = effect;
    state.animation_step = 0;

#if ZMK_BLE_IS_CENTRAL
    led_data.effect = effect;

    zmk_rgb_underglow_central_send();
#endif

    return 0;
}

int zmk_rgb_underglow_cycle_effect(int direction) {
    return zmk_rgb_underglow_select_effect(zmk_rgb_underglow_calc_effect(direction));
}

int zmk_rgb_underglow_toggle(void) {
    return state.on ? zmk_rgb_underglow_off() : zmk_rgb_underglow_on();
}

int zmk_rgb_underglow_set_hsb(struct zmk_led_hsb color) {
    if (color.h > HUE_MAX || color.s > SAT_MAX || color.b > BRT_MAX) {
        return -ENOTSUP;
    }

    state.color = color;

    return 0;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_hue(int direction) {
    struct zmk_led_hsb color = state.color;

    color.h += HUE_MAX + (direction * CONFIG_ZMK_RGB_UNDERGLOW_HUE_STEP);
    color.h %= HUE_MAX;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_sat(int direction) {
    struct zmk_led_hsb color = state.color;

    int s = color.s + (direction * CONFIG_ZMK_RGB_UNDERGLOW_SAT_STEP);
    if (s < 0) {
        s = 0;
    } else if (s > SAT_MAX) {
        s = SAT_MAX;
    }
    color.s = s;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_brt(int direction) {
    struct zmk_led_hsb color = state.color;

    int b = color.b + (direction * CONFIG_ZMK_RGB_UNDERGLOW_BRT_STEP);
    color.b = CLAMP(b, 0, BRT_MAX);

    return color;
}

int zmk_rgb_underglow_change_hue(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_hue(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_sat(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_sat(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_brt(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_brt(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_spd(int direction) {
    if (!led_strip)
        return -ENODEV;

    if (state.animation_speed == 1 && direction < 0) {
        return 0;
    }

    state.animation_speed += direction;

    if (state.animation_speed > 5) {
        state.animation_speed = 5;
    }

    return zmk_rgb_underglow_save_state();
}

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||                                          \
    IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
struct rgb_underglow_sleep_state {
    bool is_awake;
    bool rgb_state_before_sleeping;
};

static int rgb_underglow_auto_state(bool target_wake_state) {
    static struct rgb_underglow_sleep_state sleep_state = {
        is_awake : true,
        rgb_state_before_sleeping : false
    };

    // wake up event while awake, or sleep event while sleeping -> no-op
    if (target_wake_state == sleep_state.is_awake) {
        return 0;
    }
    sleep_state.is_awake = target_wake_state;

    if (sleep_state.is_awake) {
        if (sleep_state.rgb_state_before_sleeping) {
            return zmk_rgb_underglow_on();
        } else {
            return zmk_rgb_underglow_off();
        }
    } else {
        sleep_state.rgb_state_before_sleeping = state.on;
        return zmk_rgb_underglow_off();
    }
}
#endif
static int rgb_underglow_event_listener(const zmk_event_t *eh) {

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
    if (as_zmk_activity_state_changed(eh)) {
        return rgb_underglow_auto_state(zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    if (as_zmk_usb_conn_state_changed(eh)) {
    #if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    led_data.indicators = zmk_hid_indicators_get_current_profile();
#else
    led_data.indicators = 0;
#endif
        led_data.layer = zmk_keymap_highest_layer_active();
        led_data.on = state.on;
        int err = zmk_split_bt_update_led(&led_data);
        if (err) {
            LOG_ERR("send failed (err %d)", err);
        }
        static bool prev_state = false;
        return rgb_underglow_auto_state(zmk_usb_is_powered());
    }
#endif

#if ZMK_BLE_IS_CENTRAL
    if (as_zmk_split_peripheral_status_changed(eh)) {
        LOG_DBG("event called");
        const struct zmk_split_peripheral_status_changed *ev;
        ev = as_zmk_split_peripheral_status_changed(eh);
        if (ev->connected) {
            k_work_reschedule(&led_update_work, K_MSEC(2500));
            return 0;
        } else {
            k_work_cancel_delayable(&led_update_work);
            return 0;
        }
    }
#endif

    return -ENOTSUP;
}

ZMK_LISTENER(rgb_underglow, rgb_underglow_event_listener);
// IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_activity_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_usb_conn_state_changed);
#endif

#if ZMK_BLE_IS_CENTRAL
ZMK_SUBSCRIPTION(rgb_underglow, zmk_split_peripheral_status_changed);
#endif

int zmk_rgb_underglow_status(void) {
    if (!led_strip)
        return -ENODEV;

    state.status_active = true;
    state.status_animation_step = 0;
    
    // Ensure timer is running for status animation
    if (!state.on) {
        k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(25));
    }
    
    // Enable external power if needed
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL && !state.on) {
        int rc = ext_power_enable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to enable EXT_POWER for status: %d", rc);
        }
    }
#endif
    
    return 0;
}

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
