/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * BridgeUpdate.h: keeps the Gateway H2 stacked on a CoreS3 on the bridge firmware this build carries,
 * so installing MagicFocus on a CoreS3 installs the H2 too.
 *
 * The bridge reports its build in every status frame: the first 4 bytes of its ELF SHA-256. The
 * CoreS3 build embeds h2_firmware/TiltaBridgeH2/TiltaBridgeH2.bin (see platformio.ini), whose app
 * description holds the same 4 bytes. If the H2 reports a different build, or doesn't answer at all,
 * the CoreS3 resets it into its ROM download mode with the boot and enable pins it controls, and
 * writes the image through the ROM loader's serial protocol, the one esptool speaks with --no-stub.
 * That download mode lives in the H2's ROM, so an interrupted update is harmless: the next start
 * finds no bridge answering and writes it again.
 *
 * A Core2 can't reach the H2's boot pin; there the H2 is installed from a computer (README, step 1).
 */
#pragma once

#if MAGICFOCUS_H2_IMAGE
#include <MD5Builder.h>

extern const uint8_t H2_IMAGE_START[] asm("_binary_h2_firmware_TiltaBridgeH2_TiltaBridgeH2_bin_start");
extern const uint8_t H2_IMAGE_END[] asm("_binary_h2_firmware_TiltaBridgeH2_TiltaBridgeH2_bin_end");

// The image is merged for flashing at 0: bootloader at 0, partition table at 0x8000, the app at 0x10000.
static const uint32_t H2_APP_OFFSET = 0x10000;
static const uint32_t H2_APP_DESC = H2_APP_OFFSET + 0x20;      // esp_app_desc_t, after the image and segment headers
static const uint32_t H2_APP_ELF_SHA = H2_APP_OFFSET + 0xB0;   // its app_elf_sha256 field
static const uint32_t H2_FLASH_BYTES = 2 * 1024 * 1024;        // ESP32-H2-MINI-1-N2

// ROM loader commands (esptool's serial protocol) and its flash write block size.
static const uint8_t ROM_FLASH_BEGIN = 0x02, ROM_FLASH_DATA = 0x03, ROM_SYNC = 0x08, ROM_SPI_SET_PARAMS = 0x0B,
                     ROM_SPI_ATTACH = 0x0D, ROM_CHANGE_BAUDRATE = 0x0F, ROM_SPI_FLASH_MD5 = 0x13;
static const uint32_t ROM_BLOCK = 0x400;
static const uint8_t SLIP_END = 0xC0, SLIP_ESC = 0xDB, SLIP_ESC_END = 0xDC, SLIP_ESC_ESC = 0xDD;

static size_t bridgeImageSize() { return (size_t)(H2_IMAGE_END - H2_IMAGE_START); }

// The build the embedded image reports once it runs.
static uint32_t embeddedBridgeBuild() {
    const uint8_t* sha = H2_IMAGE_START + H2_APP_ELF_SHA;
    return (uint32_t)sha[0] << 24 | (uint32_t)sha[1] << 16 | (uint32_t)sha[2] << 8 | sha[3];
}

// True when the embedded image is a whole H2 image: bootloader header, app header and app description.
static bool embeddedBridgeImageValid() {
    const uint8_t* image = H2_IMAGE_START;
    const uint8_t* desc = image + H2_APP_DESC;
    return bridgeImageSize() > H2_APP_ELF_SHA + 32 && bridgeImageSize() <= H2_FLASH_BYTES && image[0] == 0xE9 &&
           image[H2_APP_OFFSET] == 0xE9 && desc[0] == 0x32 && desc[1] == 0x54 && desc[2] == 0xCD && desc[3] == 0xAB;
}

static void putLe32(uint8_t* out, uint32_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

// ----------------------------------------------------------------------------
// ROM loader protocol: SLIP-framed command and reply packets
// ----------------------------------------------------------------------------

// Writes bytes with SLIP escaping.
static void slipWrite(const uint8_t* data, size_t length) {
    uint8_t out[128];
    size_t used = 0;
    for (size_t index = 0; index < length; ++index) {
        const uint8_t value = data[index];
        if (value == SLIP_END) {
            out[used++] = SLIP_ESC;
            out[used++] = SLIP_ESC_END;
        } else if (value == SLIP_ESC) {
            out[used++] = SLIP_ESC;
            out[used++] = SLIP_ESC_ESC;
        } else {
            out[used++] = value;
        }
        if (used >= sizeof(out) - 2) {
            bridgeSerial.write(out, used);
            used = 0;
        }
    }
    if (used > 0) bridgeSerial.write(out, used);
}

// Sends one command packet: direction 0, command, data length, checksum (only FLASH_DATA uses it), data.
static void romCommand(uint8_t command, const uint8_t* data, uint16_t length, uint32_t checksum = 0) {
    uint8_t header[8] = {0x00, command, (uint8_t)length, (uint8_t)(length >> 8)};
    putLe32(header + 4, checksum);
    bridgeSerial.write(SLIP_END);
    slipWrite(header, sizeof(header));
    slipWrite(data, length);
    bridgeSerial.write(SLIP_END);
}

// Waits for the reply to a command and checks its status. Bytes outside SLIP frames, such as the
// ROM's boot banner, are skipped.
// @param command Command the reply must answer.
// @param timeoutMs How long to wait.
// @param data Receives the reply's data before the status bytes (the MD5 reply's 32 hex digits); may be nullptr.
// @param dataSize Capacity of data.
// @return True for a reply with a success status.
static bool romReply(uint8_t command, uint32_t timeoutMs, uint8_t* data = nullptr, size_t dataSize = 0) {
    uint8_t packet[64];
    size_t count = 0;
    bool inFrame = false, escaped = false;
    const uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (!bridgeSerial.available()) {
            delay(1);
            continue;
        }
        uint8_t value = (uint8_t)bridgeSerial.read();
        if (value == SLIP_END) {
            const size_t size = count >= 8 ? (size_t)(packet[2] | packet[3] << 8) : 0;
            if (inFrame && count >= 10 && packet[0] == 0x01 && packet[1] == command && count >= 8 + size) {
                // The data ends in status bytes (2, or 4 on the original ESP32's ROM); the first is 0 for success.
                const size_t extra = command == ROM_SPI_FLASH_MD5 ? 32 : 0;
                if (size < extra + 2) return false;
                if (data != nullptr) memcpy(data, packet + 8, extra < dataSize ? extra : dataSize);
                return packet[8 + extra] == 0;
            }
            inFrame = true;  // the end of one packet can also be the start of the next
            count = 0;
            escaped = false;
            continue;
        }
        if (!inFrame) continue;
        if (escaped) {
            value = value == SLIP_ESC_END ? SLIP_END : value == SLIP_ESC_ESC ? SLIP_ESC : value;
            escaped = false;
        } else if (value == SLIP_ESC) {
            escaped = true;
            continue;
        }
        if (count < sizeof(packet)) packet[count++] = value;
    }
    return false;
}

static void dropSerialInput() {
    while (bridgeSerial.available()) bridgeSerial.read();
}

// Syncs with the ROM loader, as esptool does after a reset into download mode.
static bool romSync() {
    uint8_t data[36] = {0x07, 0x07, 0x12, 0x20};
    memset(data + 4, 0x55, 32);
    for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        dropSerialInput();
        romCommand(ROM_SYNC, data, sizeof(data));
        if (romReply(ROM_SYNC, 100)) {
            delay(50);  // the ROM answers a sync several times; drop the rest
            dropSerialInput();
            return true;
        }
    }
    return false;
}

// Switches the ROM loader and our UART to a faster rate.
static bool romChangeBaud(uint32_t baud) {
    uint8_t data[8];
    putLe32(data, baud);
    putLe32(data + 4, 0);  // the ROM takes 0 for the current rate
    romCommand(ROM_CHANGE_BAUDRATE, data, sizeof(data));
    if (!romReply(ROM_CHANGE_BAUDRATE, 1000)) return false;
    bridgeSerial.updateBaudRate(baud);
    delay(50);
    dropSerialInput();
    return true;
}

// Attaches the H2's SPI flash and tells the ROM its size, as esptool does with --no-stub.
static bool romPrepareFlash() {
    uint8_t attach[8] = {0};
    romCommand(ROM_SPI_ATTACH, attach, sizeof(attach));
    if (!romReply(ROM_SPI_ATTACH, 3000)) return false;
    // flash id, total size, block, sector and page size, status mask
    const uint32_t params[6] = {0, H2_FLASH_BYTES, 64 * 1024, 4 * 1024, 256, 0xFFFF};
    uint8_t data[24];
    for (uint8_t index = 0; index < 6; ++index) putLe32(data + 4 * index, params[index]);
    romCommand(ROM_SPI_SET_PARAMS, data, sizeof(data));
    return romReply(ROM_SPI_SET_PARAMS, 3000);
}

// ----------------------------------------------------------------------------
// Progress on screen
// ----------------------------------------------------------------------------

static const int16_t UPDATE_BAR_X = 40, UPDATE_BAR_Y = 150, UPDATE_BAR_W = 240, UPDATE_BAR_H = 14;
static int bridgeProgressShown = -1;  // percentage the bar shows; -1 = empty

// Clears the screen to a title, a line below it and, with a bar, an empty progress bar.
static void showBridgeUpdate(const char* title, const char* detail, bool bar) {
    M5.Display.startWrite();
    M5.Display.fillScreen(COLOR_BACKGROUND);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(WHITE, COLOR_BACKGROUND);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.drawString(title, 160, 88);
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.setTextColor(LIGHTGREY, COLOR_BACKGROUND);
    M5.Display.drawString(detail, 160, 120);
    if (bar) M5.Display.drawRect(UPDATE_BAR_X, UPDATE_BAR_Y, UPDATE_BAR_W, UPDATE_BAR_H, COLOR_ITEM);
    M5.Display.endWrite();
    bridgeProgressShown = -1;
}

// Fills the progress bar and shows the percentage under it.
static void showBridgeProgress(size_t done, size_t total) {
    const int percent = (int)(done * 100 / total);
    if (percent == bridgeProgressShown) return;
    M5.Display.startWrite();
    if (percent < bridgeProgressShown) {  // a second attempt starts over
        M5.Display.fillRect(UPDATE_BAR_X + 2, UPDATE_BAR_Y + 2, UPDATE_BAR_W - 4, UPDATE_BAR_H - 4, COLOR_BACKGROUND);
    }
    bridgeProgressShown = percent;
    M5.Display.fillRect(UPDATE_BAR_X + 2, UPDATE_BAR_Y + 2, (UPDATE_BAR_W - 4) * percent / 100, UPDATE_BAR_H - 4, UI_SEL);
    char text[8];
    snprintf(text, sizeof(text), "%d %%", percent);
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(WHITE, COLOR_BACKGROUND);
    M5.Display.fillRect(130, UPDATE_BAR_Y + 20, 60, 22, COLOR_BACKGROUND);
    M5.Display.drawString(text, 160, UPDATE_BAR_Y + 31);
    M5.Display.endWrite();
}

// ----------------------------------------------------------------------------
// The update
// ----------------------------------------------------------------------------

// Writes the embedded image to the H2 from offset 0 and checks it with the ROM's MD5 of the flash.
static bool romWriteBridgeImage() {
    const uint8_t* image = H2_IMAGE_START;
    const uint32_t size = (uint32_t)bridgeImageSize();
    const uint32_t blocks = (size + ROM_BLOCK - 1) / ROM_BLOCK;

    // erase size, blocks, block size, offset, not encrypted; the ROM erases the area before answering
    uint8_t begin[20];
    putLe32(begin, size);
    putLe32(begin + 4, blocks);
    putLe32(begin + 8, ROM_BLOCK);
    putLe32(begin + 12, 0);
    putLe32(begin + 16, 0);
    romCommand(ROM_FLASH_BEGIN, begin, sizeof(begin));
    if (!romReply(ROM_FLASH_BEGIN, 3000 + size / 1024 * 40)) {
        Serial.println("[UPDATE] the H2 didn't accept the flash erase");
        return false;
    }

    static uint8_t packet[16 + ROM_BLOCK];
    for (uint32_t sequence = 0; sequence < blocks; ++sequence) {
        const uint32_t offset = sequence * ROM_BLOCK;
        const uint32_t length = size - offset < ROM_BLOCK ? size - offset : ROM_BLOCK;
        putLe32(packet, ROM_BLOCK);
        putLe32(packet + 4, sequence);
        putLe32(packet + 8, 0);
        putLe32(packet + 12, 0);
        memcpy(packet + 16, image + offset, length);
        memset(packet + 16 + length, 0xFF, ROM_BLOCK - length);
        uint8_t checksum = 0xEF;
        for (uint32_t index = 0; index < ROM_BLOCK; ++index) checksum ^= packet[16 + index];
        bool written = false;
        for (uint8_t attempt = 0; attempt < 3 && !written; ++attempt) {  // esptool also tries a block 3 times
            romCommand(ROM_FLASH_DATA, packet, sizeof(packet), checksum);
            written = romReply(ROM_FLASH_DATA, 3000);
        }
        if (!written) {
            Serial.printf("[UPDATE] writing block %lu of %lu failed\n", (unsigned long)sequence + 1, (unsigned long)blocks);
            return false;
        }
        showBridgeProgress(offset + length, size);
    }

    uint8_t md5Args[16];
    putLe32(md5Args, 0);
    putLe32(md5Args + 4, size);
    putLe32(md5Args + 8, 0);
    putLe32(md5Args + 12, 0);
    uint8_t flashMd5[32];
    romCommand(ROM_SPI_FLASH_MD5, md5Args, sizeof(md5Args));
    if (!romReply(ROM_SPI_FLASH_MD5, 3000 + size / 1024 * 10, flashMd5, sizeof(flashMd5))) {
        Serial.println("[UPDATE] the H2 didn't report the MD5 of what was written");
        return false;
    }
    MD5Builder md5;
    md5.begin();
    md5.add(image, size);
    md5.calculate();
    const String expected = md5.toString();
    if (strncasecmp(expected.c_str(), (const char*)flashMd5, 32) != 0) {
        Serial.printf("[UPDATE] MD5 mismatch: wrote %s, the H2's flash has %.32s\n", expected.c_str(), (const char*)flashMd5);
        return false;
    }
    return true;
}

// Resets the H2 into download mode and writes the bridge, first at H2_UPDATE_BAUD, then once more at
// H2_BAUD if that fails. Shows nothing if the ROM doesn't answer, i.e. no H2 is stacked.
static bool flashBridge() {
    bool screenShown = false;
    for (uint8_t pass = 0; pass < 2; ++pass) {
        const uint32_t baud = pass == 0 ? H2_UPDATE_BAUD : H2_BAUD;
        bridgeSerial.updateBaudRate(H2_BAUD);
        resetBridge(true);
        if (!romSync()) {
            Serial.println("[UPDATE] no answer from the H2's download mode");
            return false;
        }
        if (!screenShown) {
            showBridgeUpdate("Updating radio bridge", "Keep MagicFocus switched on", true);
            screenShown = true;
        }
        Serial.printf("[UPDATE] writing %u bytes to the H2 at %lu baud\n", (unsigned)bridgeImageSize(), (unsigned long)baud);
        if (baud != H2_BAUD && !romChangeBaud(baud)) {
            Serial.println("[UPDATE] the H2's ROM didn't change its baud rate");
            continue;
        }
        if (!romPrepareFlash()) {
            Serial.println("[UPDATE] the H2's ROM didn't attach its flash");
            continue;
        }
        if (romWriteBridgeImage()) return true;
    }
    return false;
}

// Waits for a status frame from the bridge.
// @param timeoutMs How long to wait.
// @return True if the bridge answered.
static bool waitForBridge(uint32_t timeoutMs) {
    const uint32_t start = millis();
    while (lastBridgeStatusMs == 0 && millis() - start < timeoutMs) {
        pollMotorLink();
        delay(5);
    }
    return lastBridgeStatusMs != 0;
}

// Called once at startup, after beginMotorLink(): writes the bridge firmware this build carries to
// the H2 if the H2 runs another build, an old bridge that doesn't report one, or nothing that answers.
static void updateBridgeIfNeeded() {
    if (!H2_AUTO_UPDATE || board.bridgeEnable < 0 || board.bridgeBoot < 0) return;
    if (!embeddedBridgeImageValid()) {
        Serial.println("[UPDATE] the embedded H2 image isn't valid; rebuild it with tools/build_release.sh");
        return;
    }
    const uint32_t wanted = embeddedBridgeBuild();
    const bool answered = waitForBridge(H2_BOOT_MS);  // beginMotorLink() may have just restarted it
    if (answered && bridgeSniffing) return;            // h2_firmware/TiltaSnifferH2, put there on purpose
    if (answered && bridgeBuildKnown && bridgeBuild == wanted) {
        Serial.printf("[UPDATE] the H2 runs this firmware's bridge build %08lX\n", (unsigned long)wanted);
        return;
    }
    if (!answered) Serial.printf("[UPDATE] the H2 doesn't answer; writing bridge build %08lX\n", (unsigned long)wanted);
    else if (!bridgeBuildKnown) Serial.printf("[UPDATE] the H2 runs an older bridge; writing build %08lX\n", (unsigned long)wanted);
    else Serial.printf("[UPDATE] the H2 runs bridge build %08lX; writing %08lX\n", (unsigned long)bridgeBuild, (unsigned long)wanted);

    const uint32_t start = millis();
    const bool written = flashBridge();
    bridgeSerial.updateBaudRate(H2_BAUD);
    resetBridge(false);
    dropSerialInput();
    lastBridgeStatusMs = 0;
    bridgeReady = motorConnected = bridgeSniffing = bridgeBuildKnown = false;
    pingBridge();
    const bool running = waitForBridge(H2_BOOT_MS) && bridgeBuildKnown && bridgeBuild == wanted;
    screenDirty = true;
    if (!written && !answered) return;  // nothing stacked, or nothing that answers: the footer says No radio bridge
    if (written && running) {
        Serial.printf("[UPDATE] the H2 runs bridge build %08lX after %lu s\n", (unsigned long)wanted,
                      (unsigned long)((millis() - start) / 1000));
        showBridgeUpdate("Radio bridge updated", "Starting MagicFocus", false);
        delay(1000);
    } else {
        Serial.printf("[UPDATE] the H2 update failed%s\n", written ? ": the H2 doesn't report the new build" : "");
        showBridgeUpdate("Radio bridge update failed", "It is tried again at the next start", false);
        delay(3000);
    }
}

#else
static void updateBridgeIfNeeded() {}  // a Core2 build carries no H2 image
#endif
