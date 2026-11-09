/*
 * Core2Programmer
 *
 * Turns an M5Stack CoreS3 into a USB programmer for the Module Gateway H2 stacked on it, so the H2's
 * bridge firmware (h2_firmware/TiltaBridgeH2) can be flashed without an ESP32 Downloader. At startup
 * it switches on the stack's power and resets the H2 with its boot pin held low, so the H2 starts in
 * its ROM download mode. From then on it passes bytes both ways between the CoreS3's USB serial port
 * and the H2's UART at 115200 baud, and esptool on the computer writes the H2 through it; the
 * command is in the README, "Flash the Gateway H2". The screen stays black while it runs.
 *
 * The pins for a Core2's stack are here too, but with a Core2 the README flashes the H2 with an ESP32
 * Downloader instead.
 *
 * Build with Arduino IDE:
 *   Tools -> Board:             M5CoreS3
 *   Tools -> USB CDC On Boot:   Enabled (the default)
 *   Library:                    M5Unified
 * Afterwards, upload MagicFocus to the CoreS3 again: at startup it restarts the H2 into the bridge.
 */

#include <Arduino.h>
#include <M5Unified.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static const uint8_t H2_UART_RX_PIN = 10;
static const uint8_t H2_UART_TX_PIN = 17;
static const uint8_t H2_BOOT_PIN = 18;
static const uint8_t H2_ENABLE_PIN = 7;
#else
static const uint8_t H2_UART_RX_PIN = 35;
static const uint8_t H2_UART_TX_PIN = 14;
static const uint8_t H2_BOOT_PIN = 13;
static const uint8_t H2_ENABLE_PIN = 19;
#endif
static const uint32_t H2_UART_BAUD = 115200;

static void enterH2DownloadMode() {
    digitalWrite(H2_BOOT_PIN, LOW);
    pinMode(H2_BOOT_PIN, OUTPUT);
    digitalWrite(H2_ENABLE_PIN, LOW);
    pinMode(H2_ENABLE_PIN, OUTPUT);
    delay(50);
    digitalWrite(H2_ENABLE_PIN, HIGH);
    delay(150);
    digitalWrite(H2_BOOT_PIN, HIGH);
}

void setup() {
    // esptool sends the H2's ROM loader 1 KB packets far faster than 115200 baud passes them on, and
    // the CoreS3's USB serial drops whatever doesn't fit its receive buffer, so make room for whole packets.
    Serial.setRxBufferSize(4096);
    Serial1.setRxBufferSize(1024);
    auto cfg = M5.config();
    cfg.serial_baudrate = H2_UART_BAUD;
    M5.begin(cfg);  // also switches on the 5V the stacked H2 runs from; without it a CoreS3 leaves the H2 unpowered
    delay(100);     // let the H2 power up before resetting it into download mode
    Serial1.begin(H2_UART_BAUD, SERIAL_8N1, H2_UART_RX_PIN, H2_UART_TX_PIN);
    enterH2DownloadMode();
}

void loop() {
    uint8_t buffer[128];

    size_t count = 0;
    while (count < sizeof(buffer) && Serial.available()) {
        buffer[count++] = (uint8_t)Serial.read();
    }
    if (count > 0) Serial1.write(buffer, count);

    count = 0;
    while (count < sizeof(buffer) && Serial1.available()) {
        buffer[count++] = (uint8_t)Serial1.read();
    }
    if (count > 0) Serial.write(buffer, count);
}
