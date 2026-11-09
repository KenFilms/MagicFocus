/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * BrightnessControl.h: sets the screen's brightness from the CoreS3's ambient light sensor. A Core2
 * has none, and keeps its brightness.
 */
#pragma once

#include <math.h>

#if CONFIG_IDF_TARGET_ESP32S3
#include <LTR5XX.h>

static LTR5XX coreS3LightSensor;
static bool coreS3LightReady = false;
static uint8_t ambientBrightness = 128;
static uint32_t lastAmbientSampleMs = 0;

// Starts the CoreS3's LTR-553 ambient-light channel.
static bool beginAmbientBrightness() {
    Ltr5xx_Init_Basic_Para parameters = LTR5XX_BASE_PARA_CONFIG_DEFAULT;
    parameters.als_gain = LTR5XX_ALS_GAIN_1X;
    parameters.als_integration_time = LTR5XX_ALS_INTEGRATION_TIME_100MS;
    parameters.als_measurement_rate = LTR5XX_ALS_MEASUREMENT_RATE_100MS;
    if (!coreS3LightSensor.begin(&parameters) ||
        !coreS3LightSensor.setAlsMode(LTR5XX_ALS_ACTIVE_MODE)) return false;
    ambientBrightness = M5.Display.getBrightness();
    coreS3LightReady = true;
    return true;
}

// Applies a smoothed logarithmic curve because the sensor spans several orders of magnitude.
static void updateAmbientBrightness() {
    if (!coreS3LightReady) return;
    const uint32_t now = millis();
    if (now - lastAmbientSampleMs < 250) return;
    lastAmbientSampleMs = now;

    const uint16_t als = coreS3LightSensor.getAlsValue();
    const int target = constrain((int)lroundf(AMBIENT_BRIGHTNESS_MIN +
                                               log10f((float)als + 1.0f) * AMBIENT_BRIGHTNESS_SCALE),
                                 (int)AMBIENT_BRIGHTNESS_MIN, (int)AMBIENT_BRIGHTNESS_MAX);
    const int next = (int)ambientBrightness + (target - (int)ambientBrightness) / 4;
    if (next != ambientBrightness) {
        ambientBrightness = (uint8_t)next;
        M5.Display.setBrightness(ambientBrightness);
    }
}
#else
// No ambient-light hardware adjustment is used on the Core2.
static bool beginAmbientBrightness() { return false; }
// No-op because ambient brightness adjustment is CoreS3-only.
static void updateAmbientBrightness() {}
#endif
