/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * Config.h: the settings, in one place: the board, the lens slots and calibration steps, the lidar,
 * the Gateway H2 bridge and how it sends, focus moves and their speeds, and the screen.
 */
#pragma once

enum BoardModel : uint8_t { BOARD_AUTO, BOARD_CORE2, BOARD_CORES3 };
// BOARD_AUTO uses the board M5Unified detects at startup; set it to force a pin layout (see Board.h).
static constexpr BoardModel BOARD_SELECTION = BOARD_AUTO;

// Lenses are picked with the prev/next arrows, or automatically from the lens name a paired BMPCC
// reports over Bluetooth when the Camera button is on (see BlackmagicCamera.h).
static const uint8_t MAX_LENSES = 8;
// Holding the lens name this long locks the lens, so the arrows can't switch it, or unlocks it again.
// A lens the camera reports still switches it, and the lock moves along. The lock is remembered across
// restarts.
static const uint32_t LENS_LOCK_HOLD_MS = 1000;
static const uint32_t CAMERA_RETRY_MS = 10000;      // gap between scans while no camera is connected

// The lens list to start from when the SD card has none of its own (/tiltafocus/lenses.txt): lenses.txt in
// the repository's top folder, built into the firmware by board_build.embed_txtfiles in platformio.ini, so
// a change there takes a reflash; tools/lenses.py push sends the same file to the card instead. Each lens
// is "<slot>: <name>", 1 to MAX_LENSES as on the screen and at most 31 characters, followed by up to
// CAL_POINTS points as "<metres> m: <motor position>"; see LENS_NAMES_PATH in LensCalibration.h. Built in,
// it only fills in: a lens whose slot already holds a calibration goes to a free slot instead.
extern const char LENS_NAMES_TXT[] asm("_binary_lenses_txt_start");

// The Calib. tab's steps, labelled with the distance the subject should roughly be at. Any two are
// enough, so skip the ones a lens can't focus at. A point saved on the tab replaces the lens's point
// nearest to it among those for the same step, which are the ones closer to that step's distance than
// to its neighbours' (in 1/distance).
static const uint8_t CAL_STEPS = 3;
static const char* CAL_STEP_LABELS[CAL_STEPS] = {"0.5 m", "2 m", "4 m"};
static const uint16_t CAL_STEP_CM[CAL_STEPS] = {50, 200, 400};
// The most calibration points a lens can hold. lenses.txt can give a lens more points than the steps.
static const uint8_t CAL_POINTS = 6;

static const uint32_t LIDAR_BAUD = 115200;          // TFmini-S default
static const uint16_t LIDAR_MIN_STRENGTH = 100;     // below this the TFmini-S reading is unreliable
static const uint16_t LIDAR_MIN_CM = 10, LIDAR_MAX_CM = 1200;
static const uint32_t LIDAR_TIMEOUT_MS = 300;       // no valid frame for this long = no reading
static const uint8_t LIDAR_MEDIAN = 5;              // samples in the median filter
// The beam clipping an edge, or losing the subject, shows up as a brief burst of much longer
// readings. Anything past this tolerance has to be confirmed before the lens is allowed to follow.
static const uint8_t LIDAR_JUMP_PERCENT = 15;
static const uint16_t LIDAR_JUMP_MIN_CM = 10;       // tolerance floor, for close subjects
// A jump must keep reading consistently for this long before it is believed. Timed rather than counted
// so it doesn't depend on the lidar's frame rate; a short multipath glitch (e.g. a reflective edge) can
// still read consistently for a few frames, so raise this if the lens chases false readings.
static const uint32_t LIDAR_JUMP_CONFIRM_MS = 30;

// ----------------------------------------------------------------------------
// Module Gateway H2 radio bridge
// ----------------------------------------------------------------------------
// Neither controller has an IEEE 802.15.4 radio, so the Module Gateway H2 (ESP32-H2) transmits the
// Tilta packets for us; its firmware lives in h2_firmware/ and its wiring per board is in Board.h.
static const uint32_t H2_BAUD = 115200;
static const uint32_t H2_STATUS_TIMEOUT_MS = 500;   // no status frame for this long = bridge missing
static const uint32_t H2_PING_INTERVAL_MS = 1000;   // how often to re-announce ourselves while the bridge is quiet

// IEEE 802.15.4 channel the motor was paired on. 12 is "CH1" in the Nucleus Nano II menus; change it
// here and in the Knob together if you use a different one.
static const uint8_t TILTA_CHANNEL = 12;
static const uint32_t H2_ANSWER_MS = 500;           // at startup, a running H2 answers within this; otherwise it is restarted
// Until MagicFocus sends a position, the bridge has nothing to transmit and can't tell whether the motor
// is there. This often it sends the handle's idle packet instead, which the motor acknowledges without
// moving; the H2 counts the motor as gone after 1.5 s without an acknowledgement.
static const uint32_t MOTOR_PROBE_MS = 500;

// How the bridge sends (see docs/tilta-motor-protocol.md); these reach the H2 with every ping, so they
// can be changed without reflashing it.
// Flip the lowest position bit on every packet, as strawlab's dongle does. Off: every flip is a new
// position, and the motor drops what is left of its move each time it gets one.
static const bool BRIDGE_DITHER = false;
// Send like the handle: a packet about every 35 ms while the position changes, then quiet, instead of
// every 10 ms.
static const bool BRIDGE_HANDLE_TIMING = true;
// Send a packet the motor didn't acknowledge again, up to 3 times, as the handle does.
static const bool BRIDGE_RESEND = true;
// While a Smooth or Linear glide moves the position, send the direction-and-speed byte as the handle
// does while it is turned. Finished moves always send 0. Applied by the Core, not the H2.
static const bool BRIDGE_SEND_SPEED = true;
static const float BRIDGE_SPEED_UNIT_COUNTS_PER_S = 185.0f;  // one step of the handle's speed value

static const int16_t FOCUS_MAX = 4095;              // the Tilta position field is 12 bits
static const bool INFINITY_AT_FOCUS_MAX = true;     // infinity end for lenses with <2 calibration points; flip if your gear runs the other way
// Closer than the nearest saved point or farther than the farthest, continue the line through the two
// outermost points (in 1/distance) instead of holding the outermost point's position. Positions stay
// within 0..FOCUS_MAX, so this relies on the motor's range ending at the lens's ends, as calibrated in
// the README; set it false if yours doesn't.
static const bool AF_EXTRAPOLATE = true;
static const int FOCUS_DEADBAND = 24;               // ignore autofocus changes smaller than this, so lidar noise doesn't chatter the motor
// The focus curve is steepest near minimum focus distance, where 1 cm of lidar quantization noise can
// exceed FOCUS_DEADBAND on its own. Requiring a minimum real distance change first stops that regardless
// of how steep the calibration curve is at the current distance.
static const uint16_t AF_DISTANCE_HYSTERESIS_CM = 2;
static const uint32_t FOCUS_SEND_INTERVAL_MS = 10;  // minimum gap between position updates to the bridge
// Finished moves (everything but the Smooth and Linear glides) are sent once, and the next only after
// this estimate of how long the motor takes. Err on the long side: too long only adds delay, too short
// may lose movement.
static const uint32_t MOTOR_SPEED_COUNTS_PER_S = 2000;
static const uint32_t MOTOR_MOVE_BASE_MS = 150;     // start-up and settling time added to every move
static const int AF_SLOW_CHUNK_COUNTS = 256;        // Slow autofocus moves in finished steps of this size
// Smooth and Linear autofocus glide at these speeds; Smooth eases in and out at these accelerations.
static const uint32_t AF_SPEED_SLOW_COUNTS_PER_S = 500;     // full range (4095) takes ~8 s
static const uint32_t AF_SPEED_FAST_COUNTS_PER_S = 4000;    // full range takes ~1.4 s
static const uint32_t AF_ACCEL_SLOW_COUNTS_PER_S2 = 3000;
static const uint32_t AF_ACCEL_FAST_COUNTS_PER_S2 = 12000;
// The Infinity buttons glide there with Smooth's easing rather than jumping, so the motor doesn't run
// flat out: full range takes about 1.7 s.
static const uint32_t INFINITY_SPEED_COUNTS_PER_S = 3000;
static const uint32_t INFINITY_ACCEL_COUNTS_PER_S2 = 8000;
// Autofocus curves (AF Curve page). Overshoot first stops past the goal by a share of the travel, then
// moves back; moves shorter than AF_OVERSHOOT_MIN_TRAVEL go straight there so tracking corrections don't wobble.
static const int AF_OVERSHOOT_PERCENT = 8;
static const int AF_OVERSHOOT_MAX_COUNTS = 120;
static const int AF_OVERSHOOT_MIN_TRAVEL = 150;
// Creep first stops this far short of the goal, then makes the last short move.
static const int AF_CREEP_ZONE_COUNTS = 150;
static const uint32_t FOCUS_REPEAT_MS = 500;       // resend an unchanged position this often, in case one was lost
static const int CAL_STEP_FINE = 2;                 // a tap on a calibration arrow moves this many counts
static const uint32_t CAL_HOLD_AFTER_MS = 300;      // a held arrow starts moving the motor after this
static const int CAL_JOG_FINE_COUNTS = 8;           // a held arrow moves this much per step at first
static const int CAL_JOG_COARSE_COUNTS = 64;        // and this much after CAL_COARSE_AFTER_MS
static const uint32_t CAL_COARSE_AFTER_MS = 1000;
static const uint32_t UI_REFRESH_MS = 100;
static const uint32_t BATTERY_REFRESH_MS = 2000;    // how often the battery readout reads the power chip again
static const int BATTERY_LOW_PERCENT = 15;          // the battery icon turns red below this charge
static const uint32_t GRAPH_WINDOW_MS = 10000;      // history the distance graph on the MagicFocus page shows

static const uint8_t UI_DEFAULT_BRIGHTNESS = 200;
static const uint8_t AMBIENT_BRIGHTNESS_MIN = 35;
static const uint8_t AMBIENT_BRIGHTNESS_MAX = 255;
static const float AMBIENT_BRIGHTNESS_SCALE = 60.0f;
