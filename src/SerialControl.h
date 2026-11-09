/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * SerialControl.h: commands from the computer over USB serial. tools/lenses.py uses them to read and
 * send the lens list, and test scripts to drive the motor through MagicFocus, for example to replay
 * recorded controller traffic (see docs/tilta-motor-protocol.md). One command per line:
 *     N                       list the lens slots: "[LENS] slot <n>, <points> points: <name>", then "[LENS] end"
 *     N <slot> <name>         name lens slot <slot> (1..8, as on the screen); without a name, clear it
 *     F                       print the lens list with every lens's points, as the SD card's lenses.txt
 *                             holds it: each line after "[LIST] ", then "[LIST] end"
 *     L                       apply a lens list, the lines that follow up to a line ".", and save the
 *                             lenses to the SD card. A list with points replaces all lenses; one without
 *                             only names them, and every lens keeps its points.
 *     P <position> <motion>   hold the motor at <position> (0..4095), with <motion> (hex) as the
 *                             direction-and-speed byte; turns autofocus off
 *     S <text>                show <text> in the footer
 *     R <hex bytes>           have the bridge send one packet with this MAC payload, e.g. a
 *                             recorded idle packet: R aa ae ff 01 83 db
 *     H                       stop sending positions to the bridge, e.g. while it plays a sequence
 *     C                       clear the bridge's sequence
 *     Q <ms> <position> <motion>  add a step to it: <ms> after the previous one, <motion> in hex
 *     G                       have the bridge play the sequence with exact timing
 *     X                       hand the motor back to MagicFocus
 * MagicFocus also takes the motor back by itself once the script has been quiet for a few seconds
 * (a minute after H).
 */
#pragma once

static const uint32_t SERIAL_CONTROL_TIMEOUT_MS = 3000;
static const uint32_t SERIAL_HANDS_OFF_TIMEOUT_MS = 60000;
static const uint32_t SERIAL_LIST_TIMEOUT_MS = 3000;   // a list upload with no line for this long is dropped
static uint32_t lastSerialCommandMs = 0;

// A lens names list arriving after L, until its "." line.
static bool receivingList = false;
static bool listTooLong = false;
static char listUpload[LENS_LIST_MAX];
static size_t listUploadLength = 0;
static uint32_t lastListLineMs = 0;

// While it lives, prints wait for the computer to take them. Answers to a command shouldn't be dropped
// the way unread log output is on a CoreS3 (see setup() in main.cpp): the computer that sent the
// command is reading.
struct SerialAnswer {
    SerialAnswer() {
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(200);
#endif
    }
    ~SerialAnswer() {
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(0);
#endif
    }
};

// Lists the lens slots, or names one: "N", "N <slot> <name>", or "N <slot>" to clear the name.
// @param arguments What follows the N.
static void handleLensCommand(char* arguments) {
    SerialAnswer answer;
    while (*arguments == ' ') ++arguments;
    if (*arguments == '\0') {
        printLensSlots();
        return;
    }
    char* name = nullptr;
    const long slot = strtol(arguments, &name, 10);
    if (name == arguments || slot < 1 || slot > MAX_LENSES) {
        Serial.printf("[LENS] no slot \"%s\": slots are 1 to %u\n", arguments, (unsigned)MAX_LENSES);
        return;
    }
    while (*name == ' ' || *name == ':') ++name;
    size_t length = strlen(name);
    while (length > 0 && isspace((unsigned char)name[length - 1])) name[--length] = '\0';
    const char* error = nameLensSlot((uint8_t)(slot - 1), name);
    if (error) Serial.printf("[LENS] slot %ld not saved: %s\n", slot, error);
    else if (name[0]) Serial.printf("[LENS] slot %ld named \"%s\"\n", slot, lenses[slot - 1].name);
    else Serial.printf("[LENS] slot %ld name cleared\n", slot);
    if (slot - 1 == lens) drawLensSelector();
}

// Takes one line of a list sent after L; the "." line saves the list and applies it.
// @param line The line, without its line ending.
static void handleListLine(const char* line) {
    lastListLineMs = millis();
    if (strcmp(line, ".") != 0) {
        const size_t length = strlen(line);
        if (listUploadLength + length + 2 > sizeof(listUpload)) {
            listTooLong = true;
            return;
        }
        memcpy(listUpload + listUploadLength, line, length);
        listUploadLength += length;
        listUpload[listUploadLength++] = '\n';
        listUpload[listUploadLength] = '\0';
        return;
    }
    receivingList = false;
    SerialAnswer answer;
    if (listTooLong) {
        Serial.println("[LENS] list not saved: list too long");
        return;
    }
    listUpload[listUploadLength] = '\0';
    applyLensList(listUpload, listHasPoints(listUpload) ? LIST_DATABASE : LIST_MERGE);
    const char* error = saveLenses();
    drawLensSelector();
    drawBody();
    if (error) Serial.printf("[LENS] list not saved: %s\n", error);
    else Serial.println("[LENS] list saved");
}

// Applies one command line from the computer.
// @param line Command text, without the line ending.
static void handleSerialCommand(char* line) {
    if (line[0] == 'N') {
        handleLensCommand(line + 1);
    } else if (line[0] == 'F' && line[1] == '\0') {
        SerialAnswer answer;
        printLensList();
    } else if (line[0] == 'L' && line[1] == '\0') {
        receivingList = true;
        listTooLong = false;
        listUploadLength = 0;
        lastListLineMs = millis();
    } else if (line[0] == 'P') {
        int position = -1;
        unsigned motion = 0;
        if (sscanf(line + 1, "%d %x", &position, &motion) < 1 || position < 0 || position > FOCUS_MAX) return;
        afEnabled = false;
        afDriving = false;
        focusTarget = position;
        stopFocus();
        motorBusyUntilMs = millis();  // the script paces its own moves
        motionOverride = true;
        overrideMotion = (uint8_t)motion;
        lastSerialCommandMs = millis();
    } else if (line[0] == 'S') {
        setStatus(line[1] == ' ' ? line + 2 : line + 1);
        scriptStatusMs = millis();
    } else if (line[0] == 'R') {
        uint8_t payload[16];
        uint8_t count = 0;
        char* cursor = line + 1;
        while (count < sizeof(payload)) {
            char* end = nullptr;
            const unsigned long value = strtoul(cursor, &end, 16);
            if (end == cursor || value > 0xFF) break;
            payload[count++] = (uint8_t)value;
            cursor = end;
        }
        if (count > 0) sendLinkFrame(LINK_CMD_RAW, payload, count);
    } else if (line[0] == 'H') {
        positionsHandsOff = true;
        lastSerialCommandMs = millis();
    } else if (line[0] == 'C') {
        sendLinkFrame(LINK_CMD_SEQUENCE_CLEAR, nullptr, 0);
    } else if (line[0] == 'Q') {
        unsigned delayMs = 0, position = 0, motion = 0;
        if (sscanf(line + 1, "%u %u %x", &delayMs, &position, &motion) < 2 || position > (unsigned)FOCUS_MAX) return;
        const uint8_t step[5] = {(uint8_t)(delayMs & 0xFF), (uint8_t)(delayMs >> 8), (uint8_t)(position & 0xFF),
                                 (uint8_t)(position >> 8), (uint8_t)motion};
        sendLinkFrame(LINK_CMD_SEQUENCE_ADD, step, sizeof(step));
    } else if (line[0] == 'G') {
        sendLinkFrame(LINK_CMD_SEQUENCE_PLAY, nullptr, 0);
        lastSerialCommandMs = millis();
    } else if (line[0] == 'X') {
        motionOverride = false;
        positionsHandsOff = false;
    }
}

// Reads command lines from USB serial, and hands the motor back once the script goes quiet.
static void pollSerialCommands() {
    static char line[64];
    static uint8_t length = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c != '\n' && c != '\r') {
            if (length < sizeof(line) - 1) line[length++] = c;
            continue;
        }
        if (length == 0) continue;
        line[length] = '\0';
        length = 0;
        if (receivingList) handleListLine(line);
        else handleSerialCommand(line);
    }
    if (receivingList && millis() - lastListLineMs > SERIAL_LIST_TIMEOUT_MS) {
        receivingList = false;
        SerialAnswer answer;
        Serial.println("[LENS] list not saved: it never ended with a \".\" line");
    }
    if (motionOverride && millis() - lastSerialCommandMs > SERIAL_CONTROL_TIMEOUT_MS) motionOverride = false;
    if (positionsHandsOff && millis() - lastSerialCommandMs > SERIAL_HANDS_OFF_TIMEOUT_MS) positionsHandsOff = false;
}
