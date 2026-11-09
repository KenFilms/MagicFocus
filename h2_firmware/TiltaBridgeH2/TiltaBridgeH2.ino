/*
 * TiltaBridgeH2
 *
 * Firmware for an M5Stack Module Gateway H2 (ESP32-H2), turning it into an IEEE 802.15.4
 * transmitter for an external lens focus motor.
 *
 * The host (an M5Stack Core2 or CoreS3 running MagicFocus) sends focus positions over UART0, each with
 * the direction-and-speed byte the Nucleus-N II Control Handle puts in its packets (see
 * docs/tilta-motor-protocol.md); this firmware keeps transmitting the current position to the motor,
 * either every 10 ms or with the handle's timing as the host chooses, and reports back whether the
 * motor is acknowledging.
 *
 * The air protocol was reverse engineered by https://github.com/strawlab/tilta-n2-control.
 *
 * Build with Arduino IDE (arduino-esp32 >= 3.1.1):
 *   Tools -> Board:                 ESP32H2 Dev Module
 *   Tools -> Flash Size:            2MB
 *   Tools -> Partition Scheme:      custom      (uses the partitions.csv next to this sketch)
 *   Tools -> Erase All Flash ...:   Enable
 * Flash it through the module's program/download header with an M5Stack ESP32 Downloader.
 */

#include <Arduino.h>
#include "esp_ieee802154.h"

// UART0 is the module's download header, which is also the only UART broken out for the host link.
#if ARDUINO_USB_CDC_ON_BOOT
#define Bridge Serial0
#else
#define Bridge Serial
#endif

// The 802.15.4 driver has these but its header doesn't declare them. IDF 5.5's promiscuous mode turns
// both automatic ACKs off and earlier versions leave both on; setting them keeps the bridge the same
// either way: wait for the motor's ACK after each packet, and never acknowledge anything itself.
extern "C" esp_err_t esp_ieee802154_set_auto_ack_rx(bool enable);
extern "C" esp_err_t esp_ieee802154_set_auto_ack_tx(bool enable);

// ----------------------------------------------------------------------------
// Air protocol
// ----------------------------------------------------------------------------

// A position command as the Knob sends it. The PAN id, source address and channel are the ones the
// strawlab project sniffed off a paired Knob; if your motor ignores this, sniff your own Knob with
// Nordic's 802.15.4 sniffer and copy the values from a position packet into the fields below.
static uint8_t txFrame[] = {
    0x0F,        // PHR: 13 MAC bytes plus the 2 FCS bytes the radio appends
    0x61, 0x88,  // frame control: data frame, ack requested, PAN id compressed, short addresses
    0x00,        // sequence number, incremented per packet
    0xE7, 0x01,  // destination PAN id (0x01E7)
    0xFF, 0xFF,  // destination address: broadcast
    0x96, 0xF0,  // source address (0xF096): the Knob we impersonate
    0x44,        // command type 4 (position) in the high nibble, position bits 11..8 in the low one
    0x05,        // position bits 7..0
    0x00,        // direction and speed: bit 7 while the position falls, bits 6..0 the speed
    0xB7,        // payload checksum
    0x00, 0x00   // FCS, filled in by the radio
};

static const uint8_t FRAME_SEQUENCE = 3;
static const uint8_t FRAME_COMMAND = 10;
static const uint8_t FRAME_POSITION_LOW = 11;
static const uint8_t FRAME_MOTION = 12;
static const uint8_t FRAME_CHECKSUM = 13;

static const uint16_t TILTA_PAN_ID = 0x01E7;
static const uint16_t TILTA_SOURCE_ADDRESS = 0xF096;
static const uint8_t DEFAULT_CHANNEL = 12;      // "CH1" in the Nucleus Nano II menus
static const uint16_t POSITION_MAX = 4095;      // the position field is 12 bits

static const uint32_t TX_INTERVAL_MS = 10;      // continuous packet rate, as strawlab's dongle sends
static const uint32_t TX_TIMEOUT_MS = 50;       // give up waiting for a transmit to complete
static const uint32_t ACK_TIMEOUT_MS = 1500;    // no ack for this long = the motor is gone
// With handle timing the bridge sends like the Nucleus-N II Control Handle: a packet about every 35 ms
// while the position or motion changes, the final one repeated a few times, then quiet apart from a
// keepalive that keeps the ACK status fresh.
static const uint32_t HANDLE_TX_INTERVAL_MS = 35;
static const uint8_t HANDLE_SETTLE_PACKETS = 20;
static const uint32_t HANDLE_KEEPALIVE_MS = 1000;
// A packet the motor doesn't acknowledge is sent again, unchanged, up to this many times: what the
// 802.15.4 MAC normally does, and what the handle was seen doing.
static const uint8_t TX_MAX_RESENDS = 3;

// ----------------------------------------------------------------------------
// Host protocol: 0xA5 0x5A <length> <type> <payload...> <checksum>
// ----------------------------------------------------------------------------

static const uint8_t LINK_SYNC0 = 0xA5, LINK_SYNC1 = 0x5A;
static const uint8_t LINK_CMD_POSITION = 0x01;
static const uint8_t LINK_CMD_CHANNEL = 0x02;
static const uint8_t LINK_CMD_PING = 0x03;
static const uint8_t LINK_CMD_OPTIONS = 0x04;
static const uint8_t LINK_OPTION_DITHER = 0x01;
static const uint8_t LINK_OPTION_HANDLE_TIMING = 0x02;
static const uint8_t LINK_OPTION_RESEND = 0x04;
static const uint8_t LINK_CMD_RAW = 0x05;       // MAC payload for one extra packet, e.g. a recorded idle packet
static const uint8_t RAW_MAX_PAYLOAD = 16;
// A sequence of position packets the bridge plays itself with exact timing, e.g. a recorded handle
// sweep: add steps (uint16 ms after the previous step, uint16 position, motion byte), then play.
static const uint8_t LINK_CMD_SEQUENCE_CLEAR = 0x06;
static const uint8_t LINK_CMD_SEQUENCE_ADD = 0x07;
static const uint8_t LINK_CMD_SEQUENCE_PLAY = 0x08;
static const uint16_t SEQUENCE_MAX_STEPS = 800;
static const uint8_t LINK_EVT_STATUS = 0x81;
static const uint8_t LINK_MAX_PAYLOAD = 16;
static const uint32_t STATUS_INTERVAL_MS = 100;
static const uint32_t HOST_BAUD = 115200;

static bool radioReady = false;
static bool hasPosition = false;                // nothing is transmitted until the host asks for a position
static uint16_t position = 0;
static uint8_t motion = 0;                      // direction-and-speed byte sent with the position
static bool dither = true;                      // flip the lowest position bit on every packet
static bool handleTiming = false;               // send like the Control Handle rather than continuously
static uint16_t lastTxPosition = 0xFFFF;        // position and motion of the last packet, before dithering
static uint8_t lastTxMotion = 0;
static uint8_t settlePackets = 0;               // repeats of a final position still to send
static uint8_t channel = DEFAULT_CHANNEL;
static volatile bool txBusy = false;
static volatile bool txNoAck = false;           // the last transmit went unacknowledged (set from the radio ISR)
static bool resend = false;                     // send unacknowledged packets again
static uint8_t resendsUsed = 0;                 // resends already spent on the current packet
// Packet counters since the bridge started, reported in the status frame.
static uint16_t statSent = 0, statAcked = 0, statResent = 0, statLost = 0;
// One extra packet from the host: length byte, the position packets' header, payload, FCS.
static uint8_t rawFrame[1 + 9 + RAW_MAX_PAYLOAD + 2];
static bool rawPending = false;
static bool lastTxWasRaw = false;               // the frame in flight is a raw one: never resent or counted

struct SequenceStep {
    uint16_t delayMs;                           // after the previous step
    uint16_t position;
    uint8_t motion;
};
static SequenceStep sequence[SEQUENCE_MAX_STEPS];
static uint16_t sequenceLength = 0;
static uint16_t sequenceNext = 0;
static bool sequencePlaying = false;
static uint32_t sequenceDueMs = 0;              // when the next step goes out
static volatile bool ackReceived = false;       // set from the radio ISR, timestamped in loop()
static volatile bool motorReplyReceived = false;
static volatile uint8_t lastTxSequence = 0;
static uint32_t lastAckMs = 0;
static bool everAcked = false;
static uint32_t lastTxMs = 0;
static uint32_t lastStatusMs = 0;
static uint8_t noise = 0;

// ----------------------------------------------------------------------------
// Radio
// ----------------------------------------------------------------------------

// Writes a 12-bit position and the direction-and-speed byte into the frame and fixes up the payload
// checksum.
static void setFramePosition(uint16_t value, uint8_t motionByte) {
    txFrame[FRAME_COMMAND] = (uint8_t)(0x40 | ((value >> 8) & 0x0F));
    txFrame[FRAME_POSITION_LOW] = (uint8_t)(value & 0xFF);
    txFrame[FRAME_MOTION] = motionByte;
    txFrame[FRAME_CHECKSUM] = (uint8_t)(0 - txFrame[FRAME_COMMAND] - txFrame[FRAME_POSITION_LOW] - txFrame[FRAME_MOTION]);
}

// Both of these run in the radio's ISR context, so they only flag what loop() has to act on.
extern "C" void esp_ieee802154_transmit_done(const uint8_t* frame, const uint8_t* ack,
                                             esp_ieee802154_frame_info_t* ackFrameInfo) {
    (void)frame;
    (void)ackFrameInfo;
    txBusy = false;
    if (ack != nullptr) {
        ackReceived = true;
        esp_ieee802154_receive_handle_done(ack);
    }
}

extern "C" void esp_ieee802154_transmit_failed(const uint8_t* frame, esp_ieee802154_tx_error_t error) {
    (void)frame;
    if (error == ESP_IEEE802154_TX_ERR_NO_ACK) txNoAck = true;
    txBusy = false;
}

extern "C" void esp_ieee802154_receive_done(uint8_t* frame, esp_ieee802154_frame_info_t* frameInfo) {
    (void)frameInfo;
    if (frame != nullptr && frame[0] >= 3 && frame[1] == 0x02 && frame[2] == 0x00 &&
        frame[3] == lastTxSequence) {
        motorReplyReceived = true;
    }
    if (frame != nullptr) esp_ieee802154_receive_handle_done(frame);
}

// Brings the radio up on the configured channel.
static void startRadio() {
    radioReady = esp_ieee802154_enable() == ESP_OK;
    if (!radioReady) return;
    // Promiscuous mode keeps the driver from filtering out the Knob's addressing, which does not
    // follow the pairing we would otherwise have to perform.
    esp_ieee802154_set_promiscuous(true);
    esp_ieee802154_set_auto_ack_rx(true);
    esp_ieee802154_set_auto_ack_tx(false);
    esp_ieee802154_set_rx_when_idle(true);
    esp_ieee802154_set_panid(TILTA_PAN_ID);
    esp_ieee802154_set_short_address(TILTA_SOURCE_ADDRESS);
    esp_ieee802154_set_channel(channel);
}

// Sends one position packet. With dithering on, the value is nudged by one count every other packet:
// strawlab found the motor only engages full speed while it believes a human is turning the Knob.
static void transmitPosition() {
    noise = dither ? (uint8_t)(noise ^ 1) : 0;
    setFramePosition((uint16_t)(position ^ noise), motion);
    lastTxPosition = position;
    lastTxMotion = motion;
    ++txFrame[FRAME_SEQUENCE];
    lastTxSequence = txFrame[FRAME_SEQUENCE];
    resendsUsed = 0;
    ++statSent;
    lastTxWasRaw = false;
    txBusy = true;
    if (esp_ieee802154_transmit(txFrame, false) != ESP_OK) txBusy = false;
}

// Sends the queued raw packet, numbered in the same sequence as the position packets.
static void transmitRaw() {
    ++txFrame[FRAME_SEQUENCE];
    rawFrame[FRAME_SEQUENCE] = txFrame[FRAME_SEQUENCE];
    lastTxSequence = rawFrame[FRAME_SEQUENCE];
    lastTxWasRaw = true;
    txBusy = true;
    if (esp_ieee802154_transmit(rawFrame, false) != ESP_OK) txBusy = false;
}

// Sends the next step of a playing sequence when it is due, one packet per step and no others, then
// lets the last position settle as usual.
// @param now Current time from millis().
static void playSequence(uint32_t now) {
    if ((int32_t)(now - sequenceDueMs) < 0 || txBusy || txNoAck) return;
    const SequenceStep& step = sequence[sequenceNext];
    position = step.position;
    motion = step.motion;
    hasPosition = true;
    lastTxMs = now;
    transmitPosition();
    if (++sequenceNext >= sequenceLength) {
        sequencePlaying = false;
        settlePackets = HANDLE_SETTLE_PACKETS;
        return;
    }
    sequenceDueMs += sequence[sequenceNext].delayMs;
}

// Deals with a packet the motor didn't acknowledge: sends it again unchanged, sequence number and
// all, until TX_MAX_RESENDS is used up, then counts it as lost.
static void handleMissingAck() {
    if (lastTxWasRaw) return;
    if (!resend || resendsUsed >= TX_MAX_RESENDS) {
        ++statLost;
        return;
    }
    ++resendsUsed;
    ++statResent;
    txBusy = true;
    if (esp_ieee802154_transmit(txFrame, false) != ESP_OK) txBusy = false;
}

// Whether a position packet is due now. Continuously, one goes out every TX_INTERVAL_MS. With handle
// timing, one goes out every HANDLE_TX_INTERVAL_MS while the position or motion is changing, the
// final one is repeated HANDLE_SETTLE_PACKETS times, and after that only a keepalive is sent.
// @param now Current time from millis().
static bool positionPacketDue(uint32_t now) {
    if (!handleTiming) return now - lastTxMs >= TX_INTERVAL_MS;
    if (now - lastTxMs < HANDLE_TX_INTERVAL_MS) return false;
    if (position != lastTxPosition || motion != lastTxMotion) {
        settlePackets = HANDLE_SETTLE_PACKETS;
        return true;
    }
    if (settlePackets > 0) {
        --settlePackets;
        return true;
    }
    return now - lastTxMs >= HANDLE_KEEPALIVE_MS;
}

// ----------------------------------------------------------------------------
// Host link
// ----------------------------------------------------------------------------

// Checksum over the length, type and payload bytes of a serial frame.
static uint8_t linkChecksum(const uint8_t* bytes, size_t length) {
    uint8_t sum = 0;
    for (size_t index = 0; index < length; ++index) sum = (uint8_t)(sum - bytes[index]);
    return sum;
}

static void sendLinkFrame(uint8_t type, const uint8_t* payload, uint8_t payloadLength) {
    uint8_t frame[4 + LINK_MAX_PAYLOAD + 1];
    frame[0] = LINK_SYNC0;
    frame[1] = LINK_SYNC1;
    frame[2] = (uint8_t)(payloadLength + 1);
    frame[3] = type;
    for (uint8_t index = 0; index < payloadLength; ++index) frame[4 + index] = payload[index];
    frame[4 + payloadLength] = linkChecksum(frame + 2, payloadLength + 2);
    Bridge.write(frame, 5 + payloadLength);
}

// Status: radio ready, motor acking, uint16 position, channel, mode 0 (the bridge, not the sniffer),
// then uint16 counters of packets sent, acknowledged, resent and lost since the bridge started.
static void sendStatus() {
    const bool motorAck = everAcked && millis() - lastAckMs < ACK_TIMEOUT_MS;
    const uint8_t payload[14] = {
        (uint8_t)(radioReady ? 1 : 0),
        (uint8_t)(motorAck ? 1 : 0),
        (uint8_t)(hasPosition ? (position & 0xFF) : 0xFF),
        (uint8_t)(hasPosition ? (position >> 8) : 0xFF),
        channel,
        0,
        (uint8_t)(statSent & 0xFF), (uint8_t)(statSent >> 8),
        (uint8_t)(statAcked & 0xFF), (uint8_t)(statAcked >> 8),
        (uint8_t)(statResent & 0xFF), (uint8_t)(statResent >> 8),
        (uint8_t)(statLost & 0xFF), (uint8_t)(statLost >> 8),
    };
    sendLinkFrame(LINK_EVT_STATUS, payload, sizeof(payload));
}

static void handleCommand(uint8_t type, const uint8_t* payload, uint8_t payloadLength) {
    switch (type) {
        case LINK_CMD_POSITION:
            if (payloadLength < 2 || sequencePlaying) break;
            position = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            if (position > POSITION_MAX) position = POSITION_MAX;
            motion = payloadLength >= 3 ? payload[2] : 0;  // hosts from before the motion byte send "not moving"
            hasPosition = true;
            break;
        case LINK_CMD_OPTIONS:
            if (payloadLength < 1) break;
            dither = (payload[0] & LINK_OPTION_DITHER) != 0;
            handleTiming = (payload[0] & LINK_OPTION_HANDLE_TIMING) != 0;
            resend = (payload[0] & LINK_OPTION_RESEND) != 0;
            break;
        case LINK_CMD_RAW:
            if (payloadLength < 1 || payloadLength > RAW_MAX_PAYLOAD) break;
            // Same header as the position packets; the sequence number goes in when it is sent.
            rawFrame[0] = (uint8_t)(9 + payloadLength + 2);
            memcpy(rawFrame + 1, txFrame + 1, 9);
            memcpy(rawFrame + 10, payload, payloadLength);
            rawPending = true;
            break;
        case LINK_CMD_SEQUENCE_CLEAR:
            sequenceLength = 0;
            sequencePlaying = false;
            break;
        case LINK_CMD_SEQUENCE_ADD:
            if (payloadLength < 5 || sequenceLength >= SEQUENCE_MAX_STEPS || sequencePlaying) break;
            sequence[sequenceLength].delayMs = (uint16_t)(payload[0] | payload[1] << 8);
            sequence[sequenceLength].position = (uint16_t)((payload[2] | payload[3] << 8) & POSITION_MAX);
            sequence[sequenceLength].motion = payload[4];
            ++sequenceLength;
            break;
        case LINK_CMD_SEQUENCE_PLAY:
            if (sequenceLength == 0) break;
            sequenceNext = 0;
            sequenceDueMs = millis() + sequence[0].delayMs;
            sequencePlaying = true;
            break;
        case LINK_CMD_CHANNEL:
            if (payloadLength < 1 || payload[0] < 11 || payload[0] > 26 || payload[0] == channel) break;
            channel = payload[0];
            if (radioReady) esp_ieee802154_set_channel(channel);
            break;
        case LINK_CMD_PING:
            sendStatus();
            break;
        default:
            break;
    }
}

static void pollHost() {
    static uint8_t frame[4 + LINK_MAX_PAYLOAD + 1];
    static uint8_t count = 0;

    while (Bridge.available()) {
        const uint8_t value = (uint8_t)Bridge.read();
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
        handleCommand(frame[3], frame + 4, (uint8_t)(bodyLength - 1));
    }
}

// ----------------------------------------------------------------------------

void setup() {
    Bridge.begin(HOST_BAUD);
    startRadio();
    sendStatus();
}

void loop() {
    pollHost();

    const uint32_t now = millis();
    if (ackReceived) {
        ackReceived = false;
        everAcked = true;
        lastAckMs = now;
        if (!lastTxWasRaw) ++statAcked;
    }
    if (motorReplyReceived) {
        motorReplyReceived = false;
        everAcked = true;
        lastAckMs = now;
    }
    if (txNoAck) {
        txNoAck = false;
        handleMissingAck();
    }
    if (radioReady && rawPending && !txBusy && !txNoAck) {
        rawPending = false;
        transmitRaw();
    }
    if (radioReady && sequencePlaying) {
        playSequence(now);
    } else if (radioReady && hasPosition && !txNoAck && (!txBusy || now - lastTxMs >= TX_TIMEOUT_MS) && positionPacketDue(now)) {
        lastTxMs = now;
        transmitPosition();
    }
    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = now;
        sendStatus();
    }
}
