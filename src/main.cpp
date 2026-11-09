/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * main.cpp: starts the hardware and runs the main loop. The rest of the firmware is in the headers
 * it includes, one part each:
 *  - A TFmini-S lidar on UART2 (Port C on a Core2, Port B on a CoreS3; 115200 baud) measures the subject distance.
 *  - The board is detected at startup and picks its pin layout from Board.h.
 *  - A per-lens calibration (up to CAL_POINTS distance/focus pairs, kept in the SD card's lens list) maps
 *    that distance to a motor position.
 *  - The position is handed to a Module Gateway H2 over a serial link; the H2 impersonates the Nucleus
 *    Nano II hand wheel on IEEE 802.15.4 and keeps retransmitting the position to the motor.
 *    Its firmware lives in h2_firmware/TiltaBridgeH2, and the air protocol comes from
 *    https://github.com/strawlab/tilta-n2-control.
 */

#include <M5Unified.h>
#include <SdFat.h>
#include <SPI.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// Split into per-module headers for readability; all still compile as one
// translation unit, so include order below matters (each module only uses
// state/functions defined in headers included before it).
// ============================================================================
#include "Config.h"
#include "Board.h"
#include "BrightnessControl.h"
#include "TiltaMotorLink.h"
#include "Lidar.h"
#include "LensCalibration.h"
#include "FocusControl.h"
#include "Ui.h"
#include "TouchInput.h"
#include "SerialControl.h"
#include "BlackmagicCamera.h"
#include "PinKeypad.h"

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

// Initializes hardware, loads calibration, and opens the link to the radio bridge.
void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
    // CoreS3: Serial is the USB port itself. With a cable in but no terminal reading, every print would
    // block for 100 ms and the loop would miss short taps, so drop log output instead of waiting.
    Serial.setTxTimeoutMs(0);
#endif
    M5.Display.setBrightness(UI_DEFAULT_BRIGHTNESS);
    M5.Display.setRotation(3);
    selectBoard();
    if (strcmp(board.name, "CoreS3") == 0) beginAmbientBrightness();
    Serial.printf("[BOARD] %s\n", board.name);
    Serial2.begin(LIDAR_BAUD, SERIAL_8N1, board.lidarRx, board.lidarTx);

    const int8_t sdCs = M5.getPin(m5::pin_name_t::sd_spi_cs);
    SPI.begin(M5.getPin(m5::pin_name_t::sd_spi_sclk), M5.getPin(m5::pin_name_t::sd_spi_cipo),
              M5.getPin(m5::pin_name_t::sd_spi_copi), sdCs);
    mountSdCard();

    menu_top.createSprite(menu_top_WIDTH, menu_top_HEIGHT);
    menu_upper_body.createSprite(menu_body_WIDTH, menu_upper_body_HEIGHT);
    menu_lower_body.createSprite(menu_body_WIDTH, menu_lower_body_HEIGHT);

    loadCalibration();
    loadAfCurve();
    loadLensLock();
    beginMotorLink();
    beginCameraLink();
    readBattery();
    drawScreen();
}

// Main loop: lidar reading, autofocus, bridge traffic, touch handling and display updates.
void loop() {
    M5.update();

    if (M5.BtnC.pressedFor(2000)) M5.Power.powerOff();  // Core2 only; the CoreS3 has no touch buttons

    readLidar();
    updateAmbientBrightness();
    pollMotorLink();
    pollSerialCommands();
    updateAutofocus();
    updateFocusMotion();
    pushFocusTarget();
    updateCameraLink();

    // The camera's PIN keypad takes over the whole screen until pairing is answered.
    if (pinScreenDrawn) {
        handleKeypadTouch();
        delay(2);
        return;
    }
    if (screenDirty) {
        screenDirty = false;
        statusDirty = false;
        drawScreen();
    }
    if (statusDirty) {
        statusDirty = false;
        drawStatus();
    }
    sampleDistanceGraph();
    refreshBody();
    updateBattery();
    handleTouch();
    delay(2);
}
