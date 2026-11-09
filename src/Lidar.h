/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * Lidar.h: reads the TFmini-S lidar and keeps the readings it can trust: strong enough and within
 * range, sudden jumps only once they hold, then the median of the last few.
 */
#pragma once

static uint16_t lidarCm = 0;          // median of the recent valid samples
static uint32_t lastLidarMs = 0;
static uint16_t lidarHistory[LIDAR_MEDIAN];
static uint8_t lidarCount = 0, lidarNext = 0;

// True if a lidar sample has arrived within the last LIDAR_TIMEOUT_MS.
static bool lidarValid() {
    return lidarCount > 0 && millis() - lastLidarMs < LIDAR_TIMEOUT_MS;
}

// Rejects unconfirmed jumps, then folds a raw sample into the median filter.
// @param cm Valid raw distance sample in centimetres.
static void addLidarSample(uint16_t cm) {
    static uint16_t pendingCm = 0;
    static bool pending = false;
    static uint32_t pendingSinceMs = 0;
    const uint32_t now = millis();

    if (!lidarValid()) {
        lidarCount = lidarNext = 0;  // don't mix in samples from before a dropout
        pending = false;
    } else {
        uint16_t tolerance = (uint16_t)((uint32_t)lidarCm * LIDAR_JUMP_PERCENT / 100);
        if (tolerance < LIDAR_JUMP_MIN_CM) tolerance = LIDAR_JUMP_MIN_CM;
        if (abs((int)cm - (int)lidarCm) > tolerance) {
            if (!pending || abs((int)cm - (int)pendingCm) > (int)tolerance) {
                pendingCm = cm;
                pending = true;
                pendingSinceMs = now;
            }
            if (now - pendingSinceMs < LIDAR_JUMP_CONFIRM_MS) {
                lastLidarMs = now;  // the sensor is alive, this sample just isn't trusted yet
                return;
            }
            lidarCount = lidarNext = 0;  // confirmed, let the filter snap to the new distance
        }
        pending = false;
    }
    lidarHistory[lidarNext] = cm;
    lidarNext = (lidarNext + 1) % LIDAR_MEDIAN;
    if (lidarCount < LIDAR_MEDIAN) ++lidarCount;

    uint16_t sorted[LIDAR_MEDIAN];
    memcpy(sorted, lidarHistory, sizeof(sorted));
    for (uint8_t i = 1; i < lidarCount; ++i) {
        for (uint8_t j = i; j > 0 && sorted[j - 1] > sorted[j]; --j) {
            const uint16_t swap = sorted[j];
            sorted[j] = sorted[j - 1];
            sorted[j - 1] = swap;
        }
    }
    lidarCm = sorted[lidarCount / 2];
    lastLidarMs = now;
}

// Parses the 9-byte TFmini-S frames: 0x59 0x59, distance (cm), strength, temperature, checksum.
static void readLidar() {
    static uint8_t frame[9];
    static uint8_t count = 0;
    static uint32_t bytesSinceDebug = 0, badChecksums = 0, lastDebugMs = 0;
    while (Serial2.available()) {
        const uint8_t value = (uint8_t)Serial2.read();
        ++bytesSinceDebug;
        if (count < 2 && value != 0x59) {
            count = 0;
            continue;
        }
        frame[count++] = value;
        if (count < sizeof(frame)) continue;
        count = 0;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 8; ++i) checksum += frame[i];
        if (checksum != frame[8]) {
            ++badChecksums;
            continue;
        }
        const uint16_t distance = frame[2] | (frame[3] << 8);
        const uint16_t strength = frame[4] | (frame[5] << 8);
        if (strength < LIDAR_MIN_STRENGTH || strength == 0xFFFF || distance < LIDAR_MIN_CM || distance > LIDAR_MAX_CM) continue;
        addLidarSample(distance);
    }
    // TEMPORARY: remove once the lidar is confirmed working. Reports what's arriving on UART2.
    const uint32_t now = millis();
    if (now - lastDebugMs >= 1000) {
        Serial.printf("[LIDAR] bytes=%lu badChecksum=%lu valid=%d cm=%u\n",
                      (unsigned long)bytesSinceDebug, (unsigned long)badChecksums, lidarValid(), lidarCm);
        bytesSinceDebug = badChecksums = 0;
        lastDebugMs = now;
    }
}
