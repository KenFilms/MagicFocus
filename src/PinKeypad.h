/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * PinKeypad.h: the keypad for the PIN the camera shows the first time it pairs over Bluetooth. It
 * takes over the screen until the PIN is answered. Core2 only.
 */
#pragma once

#ifdef CONFIG_BT_BLUEDROID_ENABLED  // only the Bluedroid camera link asks for a PIN (see BlackmagicCamera.h)

static const int PIN_KEY_X[3] = {4, 109, 214};
static const int PIN_KEY_Y[4] = {30, 83, 136, 189};
static const int PIN_KEY_W = 101, PIN_KEY_H = 49;
static uint8_t pinDigits = 0;
static char pinEntry[7] = {};

// Starts PIN entry; the camera's passkey request lands here.
// @param parameter Passkey request holding the camera's address.
static void beginPinEntry(esp_ble_gap_cb_param_t* parameter) {
    memcpy(pairingBda, parameter->ble_security.ble_req.bd_addr, sizeof(pairingBda));
    pinDigits = 0;
    memset(pinEntry, 0, sizeof(pinEntry));
    pinSubmitted = false;
    passkeyReplyPending = true;
    pinRequested = true;
}

// Draws the whole keypad, including the six digit boxes at the top.
static void drawPinScreen() {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextDatum(MC_DATUM);

    M5.Display.setFont(&fonts::Font2);
    for (uint8_t i = 0; i < 6; ++i) {
        const int x = 18 + i * 48;
        M5.Display.drawRoundRect(x, 3, 44, 24, 3, i < pinDigits ? UI_SEL : GREY);
        const char digit[2] = {i < pinDigits ? pinEntry[i] : '-', '\0'};
        M5.Display.setTextColor(WHITE, BLACK);
        M5.Display.drawString(digit, x + 22, 15);
    }

    const char* labels[4][3] = {{"1", "2", "3"}, {"4", "5", "6"}, {"7", "8", "9"}, {"DEL", "0", "OK"}};
    M5.Display.setFont(&fonts::Font4);
    for (uint8_t row = 0; row < 4; ++row) {
        for (uint8_t column = 0; column < 3; ++column) {
            const int x = PIN_KEY_X[column];
            const int y = PIN_KEY_Y[row];
            const bool confirmButton = row == 3 && column == 2;
            const uint16_t buttonColor = confirmButton ? (pinDigits == 6 ? COLOR_ACTIVE : DARK_GREY) : GREY;
            const uint16_t buttonFill = confirmButton && pinDigits == 6 ? UI_CONNECT : UI_BAR;
            M5.Display.fillRoundRect(x, y, PIN_KEY_W, PIN_KEY_H, 6, buttonFill);
            M5.Display.drawRoundRect(x, y, PIN_KEY_W, PIN_KEY_H, 6, buttonColor);
            M5.Display.setTextColor(WHITE, buttonFill);
            M5.Display.drawString(labels[row][column], x + PIN_KEY_W / 2, y + PIN_KEY_H / 2);
        }
    }
    pinScreenDrawn = true;
}

// Adds digits or deletes, and submits the PIN once all 6 digits are entered.
// @param x Touch x coordinate in screen space.
// @param y Touch y coordinate in screen space.
static void handlePinTouch(int x, int y) {
    int column = -1, row = -1;
    for (int index = 0; index < 3; ++index) {
        if (x >= PIN_KEY_X[index] && x < PIN_KEY_X[index] + PIN_KEY_W) column = index;
    }
    for (int index = 0; index < 4; ++index) {
        if (y >= PIN_KEY_Y[index] && y < PIN_KEY_Y[index] + PIN_KEY_H) row = index;
    }
    if (row < 0 || column < 0) return;

    if (row == 3 && column == 2) {
        if (pinDigits == 6) pinSubmitted = true;
        return;
    }
    if (row == 3 && column == 0) {
        if (pinDigits > 0) pinEntry[--pinDigits] = '\0';
    } else if (pinDigits < 6) {
        pinEntry[pinDigits++] = row == 3 ? '0' : (char)('0' + row * 3 + column + 1);
    } else {
        return;
    }
    drawPinScreen();
}

// Feeds new presses to the keypad while the camera waits for its PIN.
static void handleKeypadTouch() {
    const auto detail = M5.Touch.getDetail();
    if (detail.wasPressed()) handlePinTouch(detail.x, detail.y);
}

// Answers the camera's passkey request with the digits typed on the keypad.
static void replyWithEnteredPin() {
    uint32_t pin = 0;
    for (uint8_t i = 0; i < 6; ++i) pin = pin * 10 + (uint32_t)(pinEntry[i] - '0');
    const esp_err_t result = esp_ble_passkey_reply(pairingBda, true, pin);
    passkeyReplyPending = false;
    pinSubmitted = false;
    pinRequested = false;
    if (result != ESP_OK) setStatus("Camera PIN reply failed");
}

#else

static void handleKeypadTouch() {}

#endif
