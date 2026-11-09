/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * Board.h: the Core2's and the CoreS3's pins for the lidar and the Gateway H2, and picking the board
 * at startup.
 */
#pragma once

struct BoardPins {
    const char* name;
    int8_t lidarRx, lidarTx;    // lidar TX -> lidarRx
    int8_t bridgeRx, bridgeTx;  // H2 U0TXD -> bridgeRx
    int8_t bridgeEnable, bridgeBoot;  // H2 chip enable and boot pins, or -1 if they aren't wired
};

// Core2: lidar on Port C (G13/G14). The stacked H2's UART and boot pin land on G35/G14/G13, which
// Port C shares, so the H2 is wired to Port A instead: G32 -> H2 U0RXD, G33 <- H2 U0TXD.
static const BoardPins CORE2_PINS = {"Core2", 13, 14, 33, 32, -1, -1};
// CoreS3: the stacked H2 talks on G10/G17 with no extra wiring. Port C (G18/G17) carries the H2's
// boot and RX lines, so the lidar moves to Port B with the same cable: lidar TX -> G8, lidar RX <- G9.
// G7 is the H2's chip enable.
static const BoardPins CORES3_PINS = {"CoreS3", 8, 9, 10, 17, 7, 18};

#if CONFIG_IDF_TARGET_ESP32S3
static constexpr BoardModel CHIP_DEFAULT_BOARD = BOARD_CORES3;
#else
static constexpr BoardModel CHIP_DEFAULT_BOARD = BOARD_CORE2;
#endif
static_assert(BOARD_SELECTION == BOARD_AUTO || BOARD_SELECTION == CHIP_DEFAULT_BOARD,
              "BOARD_SELECTION doesn't match this chip; build the m5stack-core2 or m5stack-cores3 environment instead");

static BoardPins board = CORE2_PINS;

// Picks the pin layout from BOARD_SELECTION, or from the board M5Unified detected at startup.
static void selectBoard() {
    BoardModel model = BOARD_SELECTION;
    if (model == BOARD_AUTO) {
        const m5::board_t detected = M5.getBoard();
        if (detected == m5::board_t::board_M5StackCoreS3 || detected == m5::board_t::board_M5StackCoreS3SE) model = BOARD_CORES3;
        else if (detected == m5::board_t::board_M5StackCore2) model = BOARD_CORE2;
        else model = CHIP_DEFAULT_BOARD;
    }
    board = model == BOARD_CORES3 ? CORES3_PINS : CORE2_PINS;
}
