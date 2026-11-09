/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * LensCalibration.h: the lens database. Reads and writes the SD card's lens list, with each lens's
 * name and calibration points, and turns a lidar distance into a motor position and back.
 */
#pragma once

#include <hal/spi_ll.h>

struct LensCalibration {
    char name[32];                    // "" = no name; set from the lens list or the lens the camera reports
    uint16_t distanceCm[CAL_POINTS];  // 0 = no point here; the points are in no particular order
    int16_t focus[CAL_POINTS];
};

// Lens files from earlier versions, lens0.bin to lens7.bin, which held a name and three or four points:
// 0.5, 2 and 4 m, and for a while also 0.2 m. They are read for the slots lenses.txt gives no points.
template <uint8_t POINTS>
struct LegacyLensFile {
    char name[32];
    uint16_t distanceCm[POINTS];
    int16_t focus[POINTS];
};
static_assert(CAL_POINTS >= 4, "a lens must hold the points of the older lens files");

static LensCalibration lenses[MAX_LENSES] = {};
// Separate from the old /magicfocus directory on purpose: those curves were calibrated against the
// camera's 0..2048 focus scale and would be read as nonsense on the motor's 0..4095 one.
static const char* CALIBRATION_DIR = "/tiltafocus";

// The lens list on the card: each lens's name and calibration points, which MagicFocus reads at startup
// and writes back whenever a point is saved or a slot named. Each lens is a line "<slot>: <name>", the
// slot numbered 1 to MAX_LENSES as on the screen (or just a name), followed by its points as lines
// "<metres> m: <motor position>". Without it on the card, the list built into the firmware is used
// (LENS_NAMES_TXT in Config.h).
static const char* LENS_NAMES_PATH = "/tiltafocus/lenses.txt";
static const size_t LENS_LIST_MAX = 2048;   // longest list read or written, in bytes

// Use the display's bus owner for both devices; Arduino SPI can leave incompatible register state.
class CalibrationSpiDriver final : public SdSpiBaseClass {
public:
    void begin(SdSpiConfig) override {}
    void activate() override {
        M5.Display.waitDisplay();
        m5gfx::spi::beginTransaction(HOST);
        auto& spi = hardware();
        savedUser = spi.user.val;
        savedClock = spi.clock.val;
        savedCtrl = spi.ctrl.val;
#if CONFIG_IDF_TARGET_ESP32S3
        savedPin = spi.misc.val;
        savedLength = spi.ms_dlen.val;
        savedDma = spi.dma_conf.val;
        spi.dma_conf.val = 0;
        spi.misc.val = SPI_CS0_DIS | SPI_CS1_DIS | SPI_CS2_DIS | SPI_CS3_DIS | SPI_CS4_DIS | SPI_CS5_DIS;
#else
        savedPin = spi.pin.val;
        savedLength = spi.mosi_dlen.val;
        savedReadLength = spi.miso_dlen.val;
        spi.pin.val = SPI_CS0_DIS | SPI_CS1_DIS | SPI_CS2_DIS;
#endif
        spi.ctrl.wr_bit_order = 0;
        spi.ctrl.rd_bit_order = 0;
        spi.user.val = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
        spi.clock.val = m5gfx::FreqToClockDiv(m5gfx::getSpiClockFrequency(HOST), frequency);
    }
    void deactivate() override {
        auto& spi = hardware();
        // M5GFX caches display state: return every register we changed, not just the bus lock.
        spi.user.val = savedUser;
        spi.clock.val = savedClock;
        spi.ctrl.val = savedCtrl;
#if CONFIG_IDF_TARGET_ESP32S3
        spi.misc.val = savedPin;
        spi.ms_dlen.val = savedLength;
        spi.dma_conf.val = savedDma;
        spi.cmd.update = 1;
        while (spi.cmd.update) {}
#else
        spi.pin.val = savedPin;
        spi.mosi_dlen.val = savedLength;
        spi.miso_dlen.val = savedReadLength;
#endif
        m5gfx::spi::endTransaction(HOST);
    }
    void setSckSpeed(uint32_t hz) override { frequency = hz; }
    uint8_t receive() override { return spiTransferByteNL(SPI.bus(), 0xFF); }
    uint8_t receive(uint8_t* data, size_t size) override {
        alignas(4) uint8_t buffer[64];
        while (size) {
            const size_t chunk = size < sizeof(buffer) ? size : sizeof(buffer);
            spiTransferBytesNL(SPI.bus(), nullptr, buffer, chunk);
            memcpy(data, buffer, chunk);
            data += chunk;
            size -= chunk;
        }
        return 0;
    }
    void send(uint8_t value) override { spiTransferByteNL(SPI.bus(), value); }
    void send(const uint8_t* data, size_t size) override {
        // The HAL's unlocked transfers run under M5GFX's lock and access whole words.
        alignas(4) uint8_t buffer[64] = {};
        while (size) {
            const size_t chunk = size < sizeof(buffer) ? size : sizeof(buffer);
            memcpy(buffer, data, chunk);
            spiTransferBytesNL(SPI.bus(), buffer, nullptr, chunk);
            data += chunk;
            size -= chunk;
        }
    }
private:
#if CONFIG_IDF_TARGET_ESP32S3
    static constexpr int HOST = SPI2_HOST;
    uint32_t savedDma = 0;
#else
    static constexpr int HOST = SPI3_HOST;
    uint32_t savedReadLength = 0;
#endif
    static spi_dev_t& hardware() { return *SPI_LL_GET_HW(HOST); }
    uint32_t frequency = 400000;
    uint32_t savedUser = 0, savedClock = 0, savedCtrl = 0, savedPin = 0, savedLength = 0;
};

static CalibrationSpiDriver calibrationSpi;
static SdFat32 calibrationSd;
static bool sdMounted = false;

// Mounts the SD card, stepping the bus speed down for cards that can't keep up with the fastest one.
// The card shares the bus M5GFX initialized for the display.
static bool mountSdCard() {
    static const uint32_t SPEEDS_HZ[] = {10000000, 4000000};
    const int8_t cs = M5.getPin(m5::pin_name_t::sd_spi_cs);
    M5.Display.waitDisplay();
    for (uint32_t hz : SPEEDS_HZ) {
        calibrationSd.vol()->end();
        if (calibrationSd.begin(SdSpiConfig(cs, SHARED_SPI, hz, &calibrationSpi))) {
            Serial.printf("[SD] mounted at %lu Hz, %llu MB\n", (unsigned long)hz,
                          ((uint64_t)calibrationSd.card()->sectorCount() * 512) >> 20);
            return sdMounted = true;
        }
        Serial.printf("[SD] mount failed at %lu Hz: code=%u data=%u\n", (unsigned long)hz,
                      calibrationSd.sdErrorCode(), calibrationSd.sdErrorData());
    }
    calibrationSd.vol()->end();
    Serial.println("[SD] card initialization or FAT32 mount failed");
    return sdMounted = false;
}

// Builds the SD card path for a lens's calibration file.
// @param lens Lens slot whose file path is requested.
// @param path Output buffer for the generated path.
// @param size Capacity of the output buffer in bytes.
static void lensPath(uint8_t lens, char* path, size_t size) {
    snprintf(path, size, "%s/lens%u.bin", CALIBRATION_DIR, (unsigned)lens);
}

// Reads a lens file from an earlier version, of either layout.
// @param file Open lens file.
// @param cal Calibration to fill.
// @return The number of points the layout holds, or 0 if the file is neither layout.
template <uint8_t POINTS>
static uint8_t readLegacyLensFile(File32& file, LensCalibration& cal) {
    LegacyLensFile<POINTS> legacy;
    if (file.size() != sizeof(legacy) || file.read((uint8_t*)&legacy, sizeof(legacy)) != sizeof(legacy)) return 0;
    memset(&cal, 0, sizeof(cal));
    memcpy(cal.name, legacy.name, sizeof(cal.name) - 1);
    for (uint8_t i = 0; i < POINTS; ++i) {
        cal.distanceCm[i] = legacy.distanceCm[i];
        cal.focus[i] = legacy.focus[i];
    }
    return POINTS;
}

static uint8_t savedPointCount(const LensCalibration& cal);
static uint8_t sortedPoints(const LensCalibration& cal, uint16_t* distance, int16_t* focus);
static bool legacyLensFiles = false;   // lens files of earlier versions were read at startup

// Copies a name into a lens slot, cut short to fit and without trailing spaces.
// @param slot Slot to name.
// @param name Name to give it; "" clears it.
static void setLensName(LensCalibration& slot, const char* name) {
    memset(slot.name, 0, sizeof(slot.name));
    strncpy(slot.name, name, sizeof(slot.name) - 1);
    for (size_t length = strlen(slot.name); length > 0 && isspace((unsigned char)slot.name[length - 1]);) {
        slot.name[--length] = '\0';
    }
}

// Trims a line of a lens list in place: the byte order mark some editors put at the start of a file,
// and whitespace at both ends.
// @param line The line.
// @return Where the trimmed line starts.
static char* trimListLine(char* line) {
    if ((uint8_t)line[0] == 0xEF && (uint8_t)line[1] == 0xBB && (uint8_t)line[2] == 0xBF) line += 3;
    while (isspace((unsigned char)*line)) ++line;
    size_t length = strlen(line);
    while (length > 0 && isspace((unsigned char)line[length - 1])) line[--length] = '\0';
    return line;
}

// Reads a calibration point line of a lens list, "<metres> m: <motor position>", such as "0.5 m: 1950".
// @param line Trimmed line.
// @param cm Set to the distance in centimetres.
// @param position Set to the motor position.
// @return 1 for a point, -1 for a point line out of range (0.1 to 100 m, 0 to FOCUS_MAX), 0 for any other line.
static int parsePointLine(const char* line, uint16_t& cm, int16_t& position) {
    char* cursor = nullptr;
    const float metres = strtof(line, &cursor);
    if (cursor == line) return 0;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor != 'm') return 0;
    for (++cursor; *cursor == ' ' || *cursor == '\t'; ++cursor) {}
    if (*cursor != ':') return 0;
    char* end = nullptr;
    const long value = strtol(cursor + 1, &end, 10);
    if (end == cursor + 1 || *end != '\0') return 0;
    if (!(metres >= 0.1f && metres <= 100.0f) || value < 0 || value > FOCUS_MAX) return -1;
    cm = (uint16_t)lroundf(metres * 100.0f);
    position = (int16_t)value;
    return 1;
}

// Whether a lens list holds any calibration points.
// @param text The list.
static bool listHasPoints(const char* text) {
    for (const char* cursor = text; *cursor != '\0';) {
        const char* end = strchr(cursor, '\n');
        const size_t length = end ? (size_t)(end - cursor) : strlen(cursor);
        char buffer[64] = {};
        memcpy(buffer, cursor, length < sizeof(buffer) - 1 ? length : sizeof(buffer) - 1);
        cursor += length + (end ? 1 : 0);
        uint16_t cm;
        int16_t position;
        if (parsePointLine(trimListLine(buffer), cm, position) == 1) return true;
    }
    return false;
}

// How a lens list changes the lenses already there.
enum ListMode : uint8_t {
    LIST_DEFAULTS,   // the list built into the firmware: it fills in, but never renames a lens or changes its points
    LIST_MERGE,      // a lens the list gives points gets just those; every other lens keeps its points
    LIST_DATABASE,   // the list is the whole lens database, replacing whatever was there
};

// The slot a lens without a slot number belongs to: the one with its name, or else the first with
// neither a name nor points, which takes the name.
// @param name The lens's name; longer than a slot's name, it is cut short.
// @return The slot, or -1 if there is no room.
static int slotForName(const char* name) {
    LensCalibration listed = {};
    setLensName(listed, name);
    int freeSlot = -1;
    for (uint8_t i = 0; i < MAX_LENSES; ++i) {
        if (strcasecmp(lenses[i].name, listed.name) == 0) return i;
        if (freeSlot < 0 && lenses[i].name[0] == '\0' && savedPointCount(lenses[i]) == 0) freeSlot = i;
    }
    if (freeSlot < 0) {
        Serial.printf("[LENS] no free slot for \"%s\"\n", listed.name);
        return -1;
    }
    setLensName(lenses[freeSlot], listed.name);
    Serial.printf("[LENS] slot %d named \"%s\"\n", freeSlot + 1, listed.name);
    return freeSlot;
}

// Gives a lens one more point, if it has room for it.
// @param slot The lens.
// @param cm Distance in centimetres.
// @param position Motor position.
static void addListedPoint(uint8_t slot, uint16_t cm, int16_t position) {
    LensCalibration& cal = lenses[slot];
    for (uint8_t i = 0; i < CAL_POINTS; ++i) {
        if (cal.distanceCm[i] != 0) continue;
        cal.distanceCm[i] = cm;
        cal.focus[i] = position;
        return;
    }
    Serial.printf("[LENS] slot %u has %u points already; %.2f m left out\n", (unsigned)(slot + 1), (unsigned)CAL_POINTS,
                  cm / 100.0f);
}

// Applies a lens list (see LENS_NAMES_PATH). Lenses with a slot number are placed on a first pass and
// those without on a second, so the numbered slots are taken before the others look for free ones.
// Each lens's point lines follow it, up to the next lens.
// @param text The list, its lines separated by '\n'.
// @param mode How it changes the lenses already there.
static void applyLensList(const char* text, ListMode mode) {
    if (mode == LIST_DATABASE) memset(lenses, 0, sizeof(lenses));
    for (int pass = 0; pass < 2; ++pass) {
        int slot = -1;                // the lens this pass placed, which the next point lines belong to
        bool takesPoints = false;     // it takes the list's points
        bool pointsCleared = false;   // its points from before have been dropped for the list's
        for (const char* cursor = text; *cursor != '\0';) {
            const char* end = strchr(cursor, '\n');
            const size_t length = end ? (size_t)(end - cursor) : strlen(cursor);
            char buffer[64] = {};
            memcpy(buffer, cursor, length < sizeof(buffer) - 1 ? length : sizeof(buffer) - 1);
            cursor += length + (end ? 1 : 0);
            char* line = trimListLine(buffer);
            if (line[0] == '\0' || line[0] == '#') continue;

            uint16_t cm = 0;
            int16_t position = 0;
            const int point = parsePointLine(line, cm, position);
            if (point < 0 && pass == 0) Serial.printf("[LENS] \"%s\" left out: out of range\n", line);
            if (point != 0) {
                if (point > 0 && slot >= 0 && takesPoints) {
                    if (!pointsCleared) {
                        memset(lenses[slot].distanceCm, 0, sizeof(lenses[slot].distanceCm));
                        memset(lenses[slot].focus, 0, sizeof(lenses[slot].focus));
                        pointsCleared = true;
                    }
                    addListedPoint((uint8_t)slot, cm, position);
                }
                continue;
            }

            // A lens: "<slot>: <name>", or just a name. A name that starts with a number, such as
            // "35mm f1.4", has no colon after the number.
            slot = -1;
            long number = 0;
            char* name = line;
            if (isdigit((unsigned char)line[0])) {
                char* after = nullptr;
                const long value = strtol(line, &after, 10);
                while (*after == ' ' || *after == '\t') ++after;
                if (*after == ':') {
                    number = value;
                    for (name = after + 1; isspace((unsigned char)*name); ++name) {}
                }
            }
            if (number == 0) {
                if (pass == 1) slot = slotForName(name);
            } else if (number < 1 || number > MAX_LENSES) {
                if (pass == 0) Serial.printf("[LENS] no slot \"%s\": slots are 1 to %u\n", line, (unsigned)MAX_LENSES);
                continue;
            } else if (mode != LIST_DEFAULTS) {
                if (pass == 0) {
                    slot = (int)number - 1;
                    if (name[0] != '\0') setLensName(lenses[slot], name);
                }
            } else {
                // A built-in lens never takes a slot from another; if its slot is in use, it goes in a free one.
                const LensCalibration& there = lenses[number - 1];
                const bool empty = there.name[0] == '\0' && savedPointCount(there) == 0;
                const bool same = name[0] != '\0' && strcasecmp(there.name, name) == 0;
                if (pass == 0 && (empty || same)) {
                    slot = (int)number - 1;
                    if (empty && name[0] != '\0') setLensName(lenses[slot], name);
                } else if (pass == 1 && !same && name[0] != '\0') {
                    slot = slotForName(name);
                }
            }
            if (slot >= 0) {
                takesPoints = mode != LIST_DEFAULTS || savedPointCount(lenses[slot]) == 0;
                pointsCleared = mode != LIST_MERGE;
            }
        }
    }
}

// Reads the card's lens list.
// @param text Buffer for it, LENS_LIST_MAX bytes.
// @return false if the card has none.
static bool readLensList(char* text) {
    File32 file = sdMounted ? calibrationSd.open(LENS_NAMES_PATH, O_RDONLY) : File32();
    if (!file) return false;
    const int length = file.read(text, LENS_LIST_MAX - 1);
    file.close();
    text[length > 0 ? length : 0] = '\0';
    if (length == (int)LENS_LIST_MAX - 1) {
        Serial.printf("[SD] %s is longer than %u bytes; the rest is left out\n", LENS_NAMES_PATH, (unsigned)LENS_LIST_MAX - 1);
    }
    return true;
}

// Loads the lenses at startup: from the lens files of earlier versions, then the card's lens list over
// them, or without one the list built into the firmware. A card list with points is the whole lens
// database, once the earlier lens files have been retired (see retireLegacyLensFiles()).
static void loadCalibration() {
    memset(lenses, 0, sizeof(lenses));
    char path[32];
    for (uint8_t lens = 0; lens < MAX_LENSES; ++lens) {
        lensPath(lens, path, sizeof(path));
        File32 file = sdMounted ? calibrationSd.open(path, O_RDONLY) : File32();
        if (!file) continue;
        const uint8_t layout = readLegacyLensFile<3>(file, lenses[lens]) ? 3 : readLegacyLensFile<4>(file, lenses[lens]);
        file.close();
        if (layout == 0) {
            memset(&lenses[lens], 0, sizeof(LensCalibration));
            continue;
        }
        legacyLensFiles = true;
        Serial.printf("[SD] lens%u.bin read: %u points\n", (unsigned)lens, (unsigned)savedPointCount(lenses[lens]));
    }
    static char text[LENS_LIST_MAX];
    if (readLensList(text)) {
        applyLensList(text, listHasPoints(text) && !legacyLensFiles ? LIST_DATABASE : LIST_MERGE);
    } else {
        applyLensList(LENS_NAMES_TXT, LIST_DEFAULTS);
    }
}

// One attempt at writing the lens list; returns nullptr on success or a short reason on failure.
// @param text The list's contents.
// @param length Its length in bytes.
static const char* writeLensListFile(const char* text, size_t length) {
    if (!calibrationSd.exists(CALIBRATION_DIR) && !calibrationSd.mkdir(CALIBRATION_DIR)) return "can't create folder";
    File32 file = calibrationSd.open(LENS_NAMES_PATH, O_WRONLY | O_CREAT | O_TRUNC);
    if (!file) return "can't open file";
    const bool ok = file.write((const uint8_t*)text, length) == length;
    const bool closed = file.close();
    if (!ok || !closed) return "write error";
    File32 saved = calibrationSd.open(LENS_NAMES_PATH, O_RDONLY);
    if (!saved) return "can't verify file";
    bool matches = saved.size() == length;
    char chunk[64];
    for (size_t done = 0; matches && done < length;) {
        const size_t part = length - done < sizeof(chunk) ? length - done : sizeof(chunk);
        matches = saved.read(chunk, part) == (int)part && memcmp(chunk, text + done, part) == 0;
        done += part;
    }
    saved.close();
    return matches ? nullptr : "verification failed";
}

// Writes the lens list to LENS_NAMES_PATH, creating the folder if needed. Remounts the card and retries
// once, so a card inserted after boot or a glitch on the shared bus doesn't lose the change.
// @param text The list's contents.
// @param length Its length in bytes.
// @return nullptr on success, otherwise a short reason suitable for the status line.
static const char* saveLensList(const char* text, size_t length) {
    if (!sdMounted && !mountSdCard()) return "SD mount failed";
    M5.Display.waitDisplay();
    const char* error = writeLensListFile(text, length);
    if (error) {
        Serial.printf("[SD] saving the lens list: %s; remounting and retrying\n", error);
        error = mountSdCard() ? writeLensListFile(text, length) : "SD mount failed";
    }
    if (error) Serial.printf("[SD] saving the lens list failed: %s\n", error);
    else Serial.println("[SD] lens list saved and verified");
    return error;
}

// Formats a distance for the lens list in metres, with as few decimals as it needs: 0.5, 2, 1.35.
static void formatListMetres(char* out, size_t size, uint16_t cm) {
    if (cm % 100 == 0) snprintf(out, size, "%u", (unsigned)(cm / 100));
    else if (cm % 10 == 0) snprintf(out, size, "%u.%u", (unsigned)(cm / 100), (unsigned)(cm % 100 / 10));
    else snprintf(out, size, "%u.%02u", (unsigned)(cm / 100), (unsigned)(cm % 100));
}

// Writes the lenses as a lens list: every lens with a name or points, its points nearest first.
// @param text Buffer for the list.
// @param size Its size in bytes.
// @return The list's length.
static size_t formatLensList(char* text, size_t size) {
    size_t length = snprintf(text, size, "# MagicFocus lenses: <slot>: <name>, then each of its\n"
                                         "# calibration points as <metres> m: <motor position>\n");
    for (uint8_t lens = 0; lens < MAX_LENSES && length < size; ++lens) {
        const LensCalibration& cal = lenses[lens];
        uint16_t distance[CAL_POINTS];
        int16_t focus[CAL_POINTS];
        const uint8_t count = sortedPoints(cal, distance, focus);
        if (cal.name[0] == '\0' && count == 0) continue;
        if (cal.name[0]) length += snprintf(text + length, size - length, "%u: %s\n", (unsigned)(lens + 1), cal.name);
        else length += snprintf(text + length, size - length, "%u:\n", (unsigned)(lens + 1));
        for (uint8_t i = 0; i < count && length < size; ++i) {
            char metres[12];
            formatListMetres(metres, sizeof(metres), distance[i]);
            length += snprintf(text + length, size - length, "  %s m: %d\n", metres, focus[i]);
        }
    }
    return length < size ? length : size - 1;
}

// Renames the lens files of earlier versions to lens<n>.old once the lens list holds their points, so
// they are kept but no longer read.
static void retireLegacyLensFiles() {
    if (!legacyLensFiles) return;
    char path[32], retired[32];
    for (uint8_t lens = 0; lens < MAX_LENSES; ++lens) {
        lensPath(lens, path, sizeof(path));
        if (!calibrationSd.exists(path)) continue;
        snprintf(retired, sizeof(retired), "%s/lens%u.old", CALIBRATION_DIR, (unsigned)lens);
        calibrationSd.remove(retired);
        if (calibrationSd.rename(path, retired)) Serial.printf("[SD] %s retired as %s\n", path, retired);
        else Serial.printf("[SD] couldn't rename %s\n", path);
    }
    legacyLensFiles = false;
}

// Saves every lens's name and points as the card's lens list.
// @return nullptr on success, otherwise a short reason suitable for the status line.
static const char* saveLenses() {
    static char text[LENS_LIST_MAX];
    const char* error = saveLensList(text, formatLensList(text, sizeof(text)));
    if (!error) retireLegacyLensFiles();
    return error;
}

// Names a lens slot, or clears its name, and saves the lens list.
// @param lens Lens slot to name, 0-based.
// @param name The new name, cut short to fit; "" clears it.
// @return nullptr on success, otherwise a short reason.
static const char* nameLensSlot(uint8_t lens, const char* name) {
    setLensName(lenses[lens], name);
    return saveLenses();
}

// The Calib. tab's step a distance belongs to: the steps' ranges meet halfway between their distances,
// in 1/distance.
// @param cm Distance in centimetres.
static uint8_t calStepFor(uint16_t cm) {
    uint8_t step = 0;
    while (step + 1 < CAL_STEPS && 2.0f / cm < 1.0f / CAL_STEP_CM[step] + 1.0f / CAL_STEP_CM[step + 1]) ++step;
    return step;
}

// Stores a point saved on the Calib. tab. It replaces the lens's point nearest to it among those for the
// same step, or else takes a free place, or else replaces the lens's nearest point.
// @param cal The lens.
// @param cm Lidar distance in centimetres.
// @param focus Motor position.
static void storeCalibrationPoint(LensCalibration& cal, uint16_t cm, int16_t focus) {
    const uint8_t step = calStepFor(cm);
    int sameStep = -1, freePlace = -1, nearest = -1;
    float sameStepGap = INFINITY, nearestGap = INFINITY;
    for (uint8_t i = 0; i < CAL_POINTS; ++i) {
        if (cal.distanceCm[i] == 0) {
            if (freePlace < 0) freePlace = i;
            continue;
        }
        const float gap = fabsf(1.0f / cal.distanceCm[i] - 1.0f / cm);
        if (gap < nearestGap) {
            nearestGap = gap;
            nearest = i;
        }
        if (calStepFor(cal.distanceCm[i]) == step && gap < sameStepGap) {
            sameStepGap = gap;
            sameStep = i;
        }
    }
    const int place = sameStep >= 0 ? sameStep : freePlace >= 0 ? freePlace : nearest;
    cal.distanceCm[place] = cm;
    cal.focus[place] = focus;
}

// Prints every slot's number, its saved points and its name for the computer, then an end line.
static void printLensSlots() {
    for (uint8_t lens = 0; lens < MAX_LENSES; ++lens) {
        const uint8_t points = savedPointCount(lenses[lens]);
        Serial.printf("[LENS] slot %u, %u %s: %s\n", (unsigned)(lens + 1), (unsigned)points, points == 1 ? "point" : "points",
                      lenses[lens].name);
    }
    Serial.println("[LENS] end");
}

// Prints the lens list as saveLenses() would write it, each line marked for the computer, then an end line.
static void printLensList() {
    static char text[LENS_LIST_MAX];
    formatLensList(text, sizeof(text));
    for (const char* line = text; *line != '\0';) {
        const char* end = strchr(line, '\n');
        const int length = end ? (int)(end - line) : (int)strlen(line);
        Serial.printf("[LIST] %.*s\n", length, line);
        line += length + (end ? 1 : 0);
    }
    Serial.println("[LIST] end");
}

// Saved points sorted by distance; returns how many there are.
// @param cal Lens calibration to read.
// @param distance Output array receiving sorted distances in centimetres.
// @param focus Output array receiving the corresponding motor positions.
static uint8_t sortedPoints(const LensCalibration& cal, uint16_t* distance, int16_t* focus) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < CAL_POINTS; ++i) {
        if (cal.distanceCm[i] == 0) continue;
        uint8_t j = count++;
        for (; j > 0 && distance[j - 1] > cal.distanceCm[i]; --j) {
            distance[j] = distance[j - 1];
            focus[j] = focus[j - 1];
        }
        distance[j] = cal.distanceCm[i];
        focus[j] = cal.focus[i];
    }
    return count;
}

// Focus position for a distance, or -1 if the lens has fewer than two points. Between saved points it
// interpolates; closer than the nearest or farther than the farthest it extrapolates along the line through
// the two outermost points, or holds the outermost point's position if AF_EXTRAPOLATE is off.
// @param cal Lens calibration curve to use.
// @param cm Subject distance in centimetres.
static int focusForDistance(const LensCalibration& cal, uint16_t cm) {
    uint16_t distance[CAL_POINTS];
    int16_t focus[CAL_POINTS];
    const uint8_t count = sortedPoints(cal, distance, focus);
    if (count < 2) return -1;
    if (!AF_EXTRAPOLATE && cm <= distance[0]) return focus[0];
    if (!AF_EXTRAPOLATE && cm >= distance[count - 1]) return focus[count - 1];
    // The pair of points either side of the reading, or the outermost pair beyond them.
    uint8_t i = 1;
    while (i < count - 1 && cm > distance[i]) ++i;
    const float span = 1.0f / distance[i] - 1.0f / distance[i - 1];
    if (span == 0.0f) return focus[i];  // two points saved at the same distance
    // Lens focus travel is close to linear in 1/distance, so interpolate and extrapolate there.
    const float t = (1.0f / cm - 1.0f / distance[i - 1]) / span;
    const long position = lroundf(focus[i - 1] + t * (focus[i] - focus[i - 1]));
    return (int)constrain(position, 0L, (long)FOCUS_MAX);
}

// Distance in metres the lens is focused at, or a negative value if unknown. Beyond the outermost points it
// reads along the same extrapolated lines as focusForDistance(), up to infinity.
// @param cal Lens calibration curve to use.
// @param focusValue Motor position to convert.
static float distanceForFocus(const LensCalibration& cal, int focusValue) {
    uint16_t distance[CAL_POINTS];
    int16_t focus[CAL_POINTS];
    const uint8_t count = sortedPoints(cal, distance, focus);
    for (uint8_t i = 1; i < count; ++i) {
        if (focus[i] == focus[i - 1]) continue;
        const float t = (float)(focusValue - focus[i - 1]) / (focus[i] - focus[i - 1]);
        const bool between = t >= 0.0f && t <= 1.0f;
        const bool beyond = AF_EXTRAPOLATE && ((i == 1 && t < 0.0f) || (i == count - 1 && t > 1.0f));
        if (!between && !beyond) continue;
        const float inverse = 1.0f / distance[i - 1] + t * (1.0f / distance[i] - 1.0f / distance[i - 1]);
        return inverse > 0.0f ? 1.0f / inverse / 100.0f : -1.0f;
    }
    return -1.0f;
}

// Motor position at the lens's infinity end, judged from which way focus moves as distance grows.
// @param cal Lens calibration used to determine which motor end is infinity.
static int infinityPosition(const LensCalibration& cal) {
    uint16_t distance[CAL_POINTS];
    int16_t focus[CAL_POINTS];
    const uint8_t count = sortedPoints(cal, distance, focus);
    const bool atMax = count >= 2 && focus[count - 1] != focus[0] ? focus[count - 1] > focus[0] : INFINITY_AT_FOCUS_MAX;
    return atMax ? FOCUS_MAX : 0;
}

// Number of calibration points saved for a lens.
// @param cal Lens calibration to count.
static uint8_t savedPointCount(const LensCalibration& cal) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < CAL_POINTS; ++i) count += cal.distanceCm[i] != 0;
    return count;
}
