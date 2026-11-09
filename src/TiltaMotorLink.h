/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * TiltaMotorLink.h: the serial link to the Gateway H2, which radios the motor positions. Neither
 * controller has an IEEE 802.15.4 radio, so the H2 (ESP32-H2) does the transmitting. This file only
 * speaks the serial protocol to that bridge; the packet format that goes on the air lives in
 * h2_firmware/TiltaBridgeH2.
 *
 * Serial frame, both directions:
 *     0xA5 0x5A <length> <type> <payload...> <checksum>
 * where length counts the type byte plus the payload, and checksum makes the sum of length, type
 * and payload bytes zero.
 */
#pragma once

static const uint8_t LINK_SYNC0 = 0xA5, LINK_SYNC1 = 0x5A;
static const uint8_t LINK_CMD_POSITION = 0x01;   // uint16 little-endian, 0..FOCUS_MAX, then the direction-and-speed byte
static const uint8_t LINK_CMD_CHANNEL = 0x02;    // uint8 802.15.4 channel
static const uint8_t LINK_CMD_PING = 0x03;       // no payload
static const uint8_t LINK_CMD_OPTIONS = 0x04;    // uint8 flags, see LINK_OPTION_*
static const uint8_t LINK_CMD_RAW = 0x05;        // MAC payload bytes for one extra packet, sent once with the usual header
// A sequence of position packets the bridge plays itself with exact timing: clear, add steps (uint16
// ms after the previous step, uint16 position, motion byte), then play.
static const uint8_t LINK_CMD_SEQUENCE_CLEAR = 0x06;
static const uint8_t LINK_CMD_SEQUENCE_ADD = 0x07;
static const uint8_t LINK_CMD_SEQUENCE_PLAY = 0x08;
static const uint8_t LINK_OPTION_DITHER = 0x01;  // flip the lowest position bit on every packet
static const uint8_t LINK_OPTION_HANDLE_TIMING = 0x02;  // send with the Control Handle's timing
static const uint8_t LINK_OPTION_RESEND = 0x04;  // send unacknowledged packets again
static const uint8_t LINK_EVT_STATUS = 0x81;     // radioReady, motorAck, uint16 position, channel; the sniffer adds its mode
static const uint8_t LINK_EVT_SNIFF = 0x82;      // sniffer firmware: one 802.15.4 frame it heard
static const uint8_t LINK_EVT_SCAN = 0x83;       // sniffer firmware: frames heard on each channel 11..26
static const uint8_t LINK_MAX_PAYLOAD = 140;     // a sniffed frame carries up to 127 bytes
static const uint8_t SNIFFER_SCANNING = 2;       // sniffer status mode while it sweeps the channels
// The type C packet the Control Handle sends every 2 s while idle (docs/tilta-motor-protocol.md, "Other
// controller traffic"). The motor acknowledges it and doesn't move.
static const uint8_t IDLE_PACKET[] = {0xC9, 0x80, 0x00, 0x00, 0x00, 0xB7};

static HardwareSerial bridgeSerial(1);

static bool screenDirty = true;        // full redraw pending
static bool statusDirty = true;        // footer redraw pending
static bool bridgeReady = false;       // the H2 is answering on the serial link
static bool motorConnected = false;    // the motor is acknowledging the packets the H2 sends
static int motorPosition = -1;         // position the H2 reports it is holding; -1 = unknown
static uint32_t lastBridgeStatusMs = 0;
static uint16_t lastAutofocusCm = 0;   // distance autofocus last acted on; 0 = none yet
static char statusMessage[48] = "Starting radio bridge...";
static uint32_t scriptStatusMs = 0;    // when a test script last put a message in the footer (SerialControl.h)

// Set while the H2 runs h2_firmware/TiltaSnifferH2, which only listens and never drives the motor.
static bool bridgeSniffing = false;
static bool snifferScanning = false;
static uint8_t snifferChannel = 0;
static uint32_t sniffedFrames = 0;
static char scanSummary[24] = "";      // channels the last scan heard traffic on, e.g. " 12 15"

// The bridge's packet counters as last reported: sent, acknowledged by the motor, resent, lost.
static uint16_t linkSent = 0, linkAcked = 0, linkResent = 0, linkLost = 0;

// Updates the footer message and marks it for redraw.
// @param message New status text to display.
static void setStatus(const char* message) {
    snprintf(statusMessage, sizeof(statusMessage), "%s", message);
    statusDirty = true;
}

// Checksum over the length, type and payload bytes of a serial frame.
// @param bytes First byte of the data to checksum.
// @param length Number of bytes to include.
static uint8_t linkChecksum(const uint8_t* bytes, size_t length) {
    uint8_t sum = 0;
    for (size_t index = 0; index < length; ++index) sum = (uint8_t)(sum - bytes[index]);
    return sum;
}

// Wraps a command in a serial frame and sends it to the bridge.
// @param type Command or event type byte.
// @param payload Bytes following the type, or nullptr when empty.
// @param payloadLength Number of payload bytes.
static void sendLinkFrame(uint8_t type, const uint8_t* payload, uint8_t payloadLength) {
    uint8_t frame[4 + LINK_MAX_PAYLOAD + 1];
    frame[0] = LINK_SYNC0;
    frame[1] = LINK_SYNC1;
    frame[2] = (uint8_t)(payloadLength + 1);
    frame[3] = type;
    for (uint8_t index = 0; index < payloadLength; ++index) frame[4 + index] = payload[index];
    frame[4 + payloadLength] = linkChecksum(frame + 2, payloadLength + 2);
    bridgeSerial.write(frame, 5 + payloadLength);
}

// Asks the bridge to hold the motor at a position; it keeps retransmitting until told otherwise.
// @param position Motor position in the 0..FOCUS_MAX range.
// @param motion Direction-and-speed byte for the packet: bit 7 while the position falls, bits 6-0 the speed.
static void sendMotorPosition(uint16_t position, uint8_t motion) {
    const uint8_t payload[3] = {(uint8_t)(position & 0xFF), (uint8_t)(position >> 8), motion};
    sendLinkFrame(LINK_CMD_POSITION, payload, sizeof(payload));
}

// Tells the bridge which 802.15.4 channel the motor is paired on and how to send.
static void sendBridgeSettings() {
    const uint8_t channel = TILTA_CHANNEL;
    const uint8_t options = (BRIDGE_DITHER ? LINK_OPTION_DITHER : 0) | (BRIDGE_HANDLE_TIMING ? LINK_OPTION_HANDLE_TIMING : 0) |
                            (BRIDGE_RESEND ? LINK_OPTION_RESEND : 0);
    sendLinkFrame(LINK_CMD_CHANNEL, &channel, 1);
    sendLinkFrame(LINK_CMD_OPTIONS, &options, 1);
}

// Sends the settings and asks the bridge for a status frame.
static void pingBridge() {
    sendBridgeSettings();
    sendLinkFrame(LINK_CMD_PING, nullptr, 0);
}

// Applies one decoded status frame from the bridge.
// @param payload Status payload bytes, excluding serial framing.
// @param payloadLength Number of available payload bytes.
static void handleStatusFrame(const uint8_t* payload, uint8_t payloadLength) {
    if (payloadLength < 5) return;
    const bool wasReady = bridgeReady;
    const bool wasConnected = motorConnected;
    const bool wasSniffing = bridgeSniffing;
    bridgeReady = payload[0] != 0;
    motorConnected = payload[1] != 0;
    const uint16_t reportedPosition = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    motorPosition = reportedPosition == 0xFFFF ? -1 : reportedPosition;
    bridgeSniffing = payloadLength >= 6 && payload[5] != 0;
    snifferScanning = bridgeSniffing && payload[5] == SNIFFER_SCANNING;
    snifferChannel = payload[4];
    if (payloadLength >= 14) {
        linkSent = (uint16_t)(payload[6] | payload[7] << 8);
        linkAcked = (uint16_t)(payload[8] | payload[9] << 8);
        linkResent = (uint16_t)(payload[10] | payload[11] << 8);
        linkLost = (uint16_t)(payload[12] | payload[13] << 8);
    }
    // A bridge that has just (re)started answering runs on its defaults until it hears the settings.
    if (lastBridgeStatusMs == 0 || millis() - lastBridgeStatusMs > H2_STATUS_TIMEOUT_MS) sendBridgeSettings();
    lastBridgeStatusMs = millis();
    const bool awaitingCheck = strcmp(statusMessage, "Starting radio bridge...") == 0 ||
                               strcmp(statusMessage, "Re-checking the radio...") == 0 ||
                               strcmp(statusMessage, "No radio bridge") == 0;
    const bool changed = wasReady != bridgeReady || wasConnected != motorConnected || wasSniffing != bridgeSniffing;
    if (changed) {
        Serial.printf("[BRIDGE] %s\n", !bridgeReady ? "radio not ready" : bridgeSniffing ? "sniffer running" :
                                       motorConnected ? "motor acknowledging" : "no motor");
    }
    if (changed || awaitingCheck) {
        if (!bridgeReady) setStatus("Radio not ready");
        else if (bridgeSniffing) setStatus("Sniffer running");
        else if (motorConnected) setStatus("Motor connected");
        else setStatus("Searching for motor...");
        screenDirty = true;
    }
}

// ----------------------------------------------------------------------------
// Sniffer output (h2_firmware/TiltaSnifferH2), printed on USB serial for docs/tilta-motor-protocol.md
// ----------------------------------------------------------------------------

// Appends printf-style text to a line buffer without overrunning it.
// @param line Buffer being built.
// @param size Capacity of the buffer in bytes.
// @param used Characters already in the buffer; advanced by what was appended.
// @param format printf format string.
static void appendf(char* line, size_t size, size_t& used, const char* format, ...) {
    if (used + 1 >= size) return;
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(line + used, size - used, format, args);
    va_end(args);
    if (written > 0) used = used + (size_t)written < size ? used + (size_t)written : size - 1;
}

// Prints one frame the sniffer heard as a [SNIFF] line: seconds since the H2 started, channel, signal
// strength, then the frame. Data frames with the Knob's addressing show their payload bytes, the
// command nibble, the 12-bit value and the payload sum (00 when the checksum is right); anything else
// is printed as raw bytes.
// @param payload Sniff event: uint32 µs, int8 RSSI, LQI, channel, uint16 frames dropped so far, MAC frame without FCS.
// @param payloadLength Number of payload bytes.
static void printSniffedFrame(const uint8_t* payload, uint8_t payloadLength) {
    static uint16_t reportedDrops = 0;
    if (payloadLength < 9) return;
    ++sniffedFrames;
    const uint16_t drops = (uint16_t)(payload[7] | payload[8] << 8);
    if (drops != reportedDrops) {
        Serial.printf("[SNIFF] the H2 dropped %u frames\n", (unsigned)(uint16_t)(drops - reportedDrops));
        reportedDrops = drops;
    }
    const uint32_t micros = (uint32_t)payload[0] | (uint32_t)payload[1] << 8 | (uint32_t)payload[2] << 16 |
                            (uint32_t)payload[3] << 24;
    const uint8_t* mac = payload + 9;
    const uint8_t length = (uint8_t)(payloadLength - 9);
    const uint16_t control = length >= 2 ? (uint16_t)(mac[0] | mac[1] << 8) : 0;

    char line[480];
    size_t used = 0;
    appendf(line, sizeof(line), used, "[SNIFF] %lu.%06lu ch%u %ddBm", (unsigned long)(micros / 1000000),
            (unsigned long)(micros % 1000000), payload[6], (int8_t)payload[4]);
    if (length == 3 && (control & 0x07) == 2) {
        appendf(line, sizeof(line), used, " ack seq=%02X%s", mac[2], control & 0x10 ? " pending" : "");
    } else if (length >= 9 && control == 0x8861) {
        // Data frame, PAN id compressed, short addresses: the layout of the Knob's position packets.
        appendf(line, sizeof(line), used, " data seq=%02X pan=%04X dst=%04X src=%04X |", mac[2], mac[3] | mac[4] << 8,
                mac[5] | mac[6] << 8, mac[7] | mac[8] << 8);
        uint8_t sum = 0;
        for (uint8_t index = 9; index < length; ++index) {
            appendf(line, sizeof(line), used, " %02X", mac[index]);
            sum = (uint8_t)(sum + mac[index]);
        }
        if (length >= 11) {
            appendf(line, sizeof(line), used, " | cmd=%X val=%u sum=%02X", mac[9] >> 4, (mac[9] & 0x0F) << 8 | mac[10], sum);
        }
    } else {
        appendf(line, sizeof(line), used, " frame |");
        for (uint8_t index = 0; index < length; ++index) appendf(line, sizeof(line), used, " %02X", mac[index]);
    }
    appendf(line, sizeof(line), used, "\n");
    Serial.write((const uint8_t*)line, used);
}

// Prints how many frames the sniffer heard on each channel during its scan as a [SCAN] line, and keeps
// the channels that had any traffic for the footer.
// @param payload uint16 little-endian frame count for each channel from 11 up.
// @param payloadLength Number of payload bytes.
static void printScanResult(const uint8_t* payload, uint8_t payloadLength) {
    char line[256];
    size_t used = 0;
    size_t summaryUsed = 0;
    scanSummary[0] = '\0';
    appendf(line, sizeof(line), used, "[SCAN]");
    for (uint8_t index = 0; 2 * index + 1 < payloadLength; ++index) {
        const unsigned count = payload[2 * index] | payload[2 * index + 1] << 8;
        appendf(line, sizeof(line), used, " ch%u:%u", 11 + index, count);
        if (count > 0) appendf(scanSummary, sizeof(scanSummary), summaryUsed, " %u", 11 + index);
    }
    appendf(line, sizeof(line), used, "\n");
    Serial.write((const uint8_t*)line, used);
}

// Keeps the footer showing what the sniffer is doing, once a second, unless a test script is using it.
static void updateSnifferStatus() {
    static uint32_t lastUpdateMs = 0;
    if (!bridgeSniffing || millis() - lastUpdateMs < 1000) return;
    if (scriptStatusMs != 0 && millis() - scriptStatusMs < 5000) return;
    lastUpdateMs = millis();
    char message[sizeof(statusMessage)];
    if (snifferScanning) {
        snprintf(message, sizeof(message), "Sniffer: scanning channels 11-26");
    } else if (scanSummary[0]) {
        snprintf(message, sizeof(message), "Sniff ch%u: %lu frames, heard on%s", snifferChannel,
                 (unsigned long)sniffedFrames, scanSummary);
    } else {
        snprintf(message, sizeof(message), "Sniff ch%u: %lu frames", snifferChannel, (unsigned long)sniffedFrames);
    }
    setStatus(message);
}

// Prints the bridge's packet counters for the last few seconds, while it is sending.
static void logLinkStats() {
    static uint32_t lastLogMs = 0;
    static uint16_t sent0 = 0, acked0 = 0, resent0 = 0, lost0 = 0;
    if (millis() - lastLogMs < 5000) return;
    lastLogMs = millis();
    const unsigned sent = (uint16_t)(linkSent - sent0), acked = (uint16_t)(linkAcked - acked0);
    const unsigned resent = (uint16_t)(linkResent - resent0), lost = (uint16_t)(linkLost - lost0);
    if (sent > 0) {
        Serial.printf("[LINK] last 5 s: %u packets, %u acknowledged, %u resent, %u lost\n", sent, acked, resent, lost);
    }
    sent0 = linkSent;
    acked0 = linkAcked;
    resent0 = linkResent;
    lost0 = linkLost;
}

// While the bridge holds no position it transmits nothing, so the motor couldn't show as connected
// before its first move. Has the bridge send the handle's idle packet instead now and then; the
// motor's acknowledgements of it show that it is there.
static void probeMotor() {
    static uint32_t lastProbeMs = 0;
    if (!bridgeReady || bridgeSniffing || motorPosition >= 0) return;
    const uint32_t now = millis();
    if (now - lastProbeMs < MOTOR_PROBE_MS) return;
    lastProbeMs = now;
    sendLinkFrame(LINK_CMD_RAW, IDLE_PACKET, sizeof(IDLE_PACKET));
}

// Reads whatever the bridge has sent and keeps the link state up to date.
static void pollMotorLink() {
    static uint8_t frame[4 + LINK_MAX_PAYLOAD + 1];
    static uint8_t count = 0;
    static uint32_t lastPingMs = 0;

    while (bridgeSerial.available()) {
        const uint8_t value = (uint8_t)bridgeSerial.read();
        if (count == 0 && value != LINK_SYNC0) continue;
        if (count == 1 && value != LINK_SYNC1) {
            count = 0;
            continue;
        }
        if (count == 2 && (value == 0 || value > LINK_MAX_PAYLOAD + 1)) {
            count = 0;
            continue;
        }
        frame[count++] = value;
        if (count < 3) continue;

        const uint8_t bodyLength = frame[2];
        if (count < (uint8_t)(bodyLength + 4)) continue;
        count = 0;
        if (linkChecksum(frame + 2, bodyLength + 1) != frame[bodyLength + 3]) continue;
        if (frame[3] == LINK_EVT_STATUS) handleStatusFrame(frame + 4, (uint8_t)(bodyLength - 1));
        else if (frame[3] == LINK_EVT_SNIFF) printSniffedFrame(frame + 4, (uint8_t)(bodyLength - 1));
        else if (frame[3] == LINK_EVT_SCAN) printScanResult(frame + 4, (uint8_t)(bodyLength - 1));
    }
    updateSnifferStatus();
    logLinkStats();

    const uint32_t now = millis();
    if (now - lastBridgeStatusMs > H2_STATUS_TIMEOUT_MS) {
        const bool hadLink = bridgeReady || motorConnected;
        const bool awaitingCheck = strcmp(statusMessage, "Starting radio bridge...") == 0 ||
                                   strcmp(statusMessage, "Re-checking the radio...") == 0;
        if (hadLink) {
            bridgeReady = motorConnected = bridgeSniffing = false;
            motorPosition = -1;
            lastAutofocusCm = 0;
            screenDirty = true;
            Serial.println("[BRIDGE] stopped answering");
        }
        if (hadLink || awaitingCheck) setStatus("No radio bridge");
        if (now - lastPingMs >= H2_PING_INTERVAL_MS) {
            lastPingMs = now;
            pingBridge();
        }
    }
    probeMotor();
}

// Restarts the H2 into its firmware, on boards where its chip enable and boot pins are wired.
static void restartBridge() {
    if (board.bridgeEnable < 0 || board.bridgeBoot < 0) return;
    Serial.println("[BRIDGE] no answer, restarting the H2");
    pinMode(board.bridgeBoot, OUTPUT);
    digitalWrite(board.bridgeBoot, HIGH);  // boot the firmware, not the ROM's download mode
    pinMode(board.bridgeEnable, OUTPUT);
    digitalWrite(board.bridgeEnable, LOW);
    delay(50);
    digitalWrite(board.bridgeEnable, HIGH);
    delay(100);
    // Hand both lines back to the H2 module's own pull-ups, as before.
    pinMode(board.bridgeBoot, INPUT);
    pinMode(board.bridgeEnable, INPUT);
}

// Opens the serial link to the H2 and announces the channel. A running bridge keeps the last position
// it was sent, so it is only restarted if it doesn't answer, e.g. because Core2Programmer left it in
// download mode, where it says nothing.
static void beginMotorLink() {
    // The sniffer firmware sends far more than the bridge; this rides out the loop's screen redraws.
    bridgeSerial.setRxBufferSize(4096);
    bridgeSerial.begin(H2_BAUD, SERIAL_8N1, board.bridgeRx, board.bridgeTx);
    pingBridge();
    const uint32_t start = millis();
    while (lastBridgeStatusMs == 0 && millis() - start < H2_ANSWER_MS) {
        pollMotorLink();
        delay(5);
    }
    if (lastBridgeStatusMs == 0) restartBridge();
}
