/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * Ui.h: draws the screen: the tabs, the lens selector, the MagicFocus, Calib. and AF Curve pages, the
 * distance graph, the info page, and the footer with the status, the (i) and the battery. The layout
 * and colours come from MagicFocusEF_menu_test.ino.
 */
#pragma once

static const uint16_t COLOR_BACKGROUND = 0x18E3;
static const uint16_t COLOR_ITEM = 0x4229;
static const uint16_t COLOR_BUTTON = 0x31C7;
static const uint16_t COLOR_BUTTON_TEXT = 0x7C31;
static const uint16_t COLOR_ACTIVE = 0x07E0;
static const uint16_t GREY = 0x8410;
static const uint16_t DARK_GREY = 0x4208;
static const uint16_t UI_BAR = 0x1082;
static const uint16_t UI_CONNECT = 0x1A91;
static const uint16_t UI_SEL = 0xFD20;

#define menu_top_WIDTH  320
#define menu_top_HEIGHT 50
#define menu_body_WIDTH 300
#define menu_upper_body_HEIGHT 40
#define menu_lower_body_HEIGHT 110

#define FSS9 (&fonts::FreeSans9pt7b)
#define FSSB9 (&fonts::FreeSansBold9pt7b)
#define FSSB12 (&fonts::FreeSansBold12pt7b)

static M5Canvas menu_top(&M5.Display);
static M5Canvas menu_upper_body(&M5.Display);
static M5Canvas menu_lower_body(&M5.Display);

struct TouchPoint {
    int x, y;
};

// A screen rectangle, inclusive on all sides.
struct HotZone {
    int x0, y0, x1, y1;
    // Tests whether a screen-space touch falls inside this inclusive rectangle.
    // @param p Touch coordinates in screen space.
    bool inHotZone(const TouchPoint& p) const { return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1; }
};

// Screen layout, top to bottom: the tabs, the lens selector, the page's body and the footer.
static const int BODY_X = 10, BODY_Y = 105;           // where the lower body sprite sits on the screen
static const int FOOTER_Y = 217, FOOTER_H = 23;       // the status line, the (i) and the battery
static const int FOOTER_MID = FOOTER_Y + FOOTER_H / 2;

// Screen coordinates. The lower body's buttons are offset by where its sprite sits.
static const HotZone menu_mf = {0, 0, 116, 50};
static const HotZone menu_calib = {116, 0, 218, 50};
static const HotZone menu_char = {218, 0, 320, 50};
static const HotZone lensPrevBtn = {10, 60, 160, 100};
static const HotZone lensNextBtn = {160, 60, 310, 100};
static const HotZone afSpeedBtn = {10, BODY_Y, 75, BODY_Y + 56};
static const HotZone afCameraBtn = {245, BODY_Y, 310, BODY_Y + 56};
static const HotZone afBtn = {10, BODY_Y + 62, 105, BODY_Y + 106};
static const HotZone radioBtn = {110, BODY_Y + 62, 210, BODY_Y + 106};
static const HotZone afInfinityBtn = {215, BODY_Y + 62, 310, BODY_Y + 106};
static const HotZone calPointBtn = {10, BODY_Y, 110, BODY_Y + 70};
static const HotZone calLeftBtn = {120, BODY_Y, 175, BODY_Y + 70};
static const HotZone calRightBtn = {255, BODY_Y, 310, BODY_Y + 70};
static const HotZone calSaveBtn = {100, BODY_Y + 76, 215, BODY_Y + 110};
static const HotZone calInfinityBtn = {225, BODY_Y + 76, 310, BODY_Y + 110};
// Indexed like AF_CURVES: Snap, Overshoot and Creep across the top, the Smooth and Linear glides below.
static const HotZone curveBtns[AF_CURVE_COUNT] = {
    {10, BODY_Y, 105, BODY_Y + 51}, {112, BODY_Y, 207, BODY_Y + 51}, {214, BODY_Y, 309, BODY_Y + 51},
    {10, BODY_Y + 58, 156, BODY_Y + 109}, {163, BODY_Y + 58, 309, BODY_Y + 109}};

// Draws the MagicFocus / Calib. / AF Curve folder tabs: the open tab stands taller and runs into the page
// below, the others sit lower and darker behind it.
static void drawTabs() {
    static const HotZone* const tabs[] = {&menu_mf, &menu_calib, &menu_char};
    static const char* const labels[] = {"MagicFocus", "Calib.", "AF Curve"};
    const int radius = 8;
    int openX = 0, openW = 0;
    menu_top.fillSprite(BLACK);
    menu_top.setTextDatum(MC_DATUM);
    for (uint8_t i = 0; i < 3; ++i) {
        const bool active = page == i;
        const int x = tabs[i]->x0 + 2, w = tabs[i]->x1 - tabs[i]->x0 - 4, top = active ? 4 : 12;
        // The shape runs past the bottom of the sprite so only its top corners are rounded.
        const int h = menu_top_HEIGHT - top + radius;
        menu_top.fillRoundRect(x, top, w, h, radius, active ? COLOR_BACKGROUND : UI_BAR);
        menu_top.drawRoundRect(x, top, w, h, radius, active ? COLOR_ITEM : DARK_GREY);
        menu_top.setFreeFont(active ? FSSB9 : FSS9);
        menu_top.setTextColor(active ? WHITE : GREY);
        menu_top.drawString(labels[i], x + w / 2, (top + menu_top_HEIGHT) / 2);
        if (active) {
            openX = x;
            openW = w;
        }
    }
    // Folder edge along the bottom, open under the active tab.
    menu_top.drawFastHLine(0, menu_top_HEIGHT - 1, menu_top_WIDTH, COLOR_ITEM);
    if (openW > 0) menu_top.drawFastHLine(openX + 1, menu_top_HEIGHT - 1, openW - 2, COLOR_BACKGROUND);
    menu_top.pushSprite(0, 0);
}

// Draws the current lens name and its prev/next arrows, unless the info page covers them.
static void drawLensSelector() {
    static const int NAME_LEFT = 56;    // the name starts right of the slot number
    static const int LOCK_X = 258;      // a locked lens's padlock, left of the right arrow
    const int NAME_RIGHT = lensLocked ? LOCK_X - 8 : 272;
    if (page == PAGE_INFO) return;
    menu_upper_body.fillSprite(COLOR_ITEM);
    menu_upper_body.setTextDatum(MC_DATUM);
    menu_upper_body.setTextColor(WHITE);
    // The slot's number in a ring, as lenses.txt and the N command number it.
    char number[4];
    snprintf(number, sizeof(number), "%u", (unsigned)(lens + 1));
    menu_upper_body.drawCircle(40, 20, 11, WHITE);
    menu_upper_body.setFreeFont(FSS9);
    menu_upper_body.drawString(number, 40, 21);
    // The name in bold, in regular if bold is too wide, and cut short with ".." if even that is.
    const char* name = lenses[lens].name[0] ? lenses[lens].name : "(no lens)";
    menu_upper_body.setFreeFont(FSSB9);
    if (menu_upper_body.textWidth(name) > NAME_RIGHT - NAME_LEFT) menu_upper_body.setFreeFont(FSS9);
    char shown[sizeof(LensCalibration::name) + 2];
    const size_t full = strlen(name);
    for (size_t length = full; length > 0; --length) {
        snprintf(shown, sizeof(shown), length < full ? "%.*s.." : "%.*s", (int)length, name);
        if (menu_upper_body.textWidth(shown) <= NAME_RIGHT - NAME_LEFT) break;
    }
    menu_upper_body.drawString(shown, (NAME_LEFT + NAME_RIGHT) / 2, 20);
    if (lensLocked) {
        // A red padlock: a two-pixel shackle above the body.
        menu_upper_body.drawRoundRect(LOCK_X + 2, 11, 9, 11, 4, RED);
        menu_upper_body.drawRoundRect(LOCK_X + 3, 12, 7, 9, 3, RED);
        menu_upper_body.fillRoundRect(LOCK_X, 17, 13, 11, 2, RED);
        menu_upper_body.fillRect(LOCK_X + 6, 21, 1, 3, COLOR_ITEM);   // keyhole
    }
    menu_upper_body.fillTriangle(20, 15, 15, 20, 20, 25, WHITE);
    menu_upper_body.fillTriangle(280, 15, 285, 20, 280, 25, WHITE);
    menu_upper_body.pushSprite(10, 60);
}

// Formats a distance in metres, or "--" if it is unknown.
// @param out Destination text buffer.
// @param size Capacity of the destination buffer in bytes.
// @param metres Distance to format; negative values display as unknown.
static void formatMetres(char* out, size_t size, float metres) {
    if (metres < 0) snprintf(out, size, "--");
    else snprintf(out, size, "%.2f m", metres);
}

// ----------------------------------------------------------------------------
// Distance graph: the lidar and lens distances over the last GRAPH_WINDOW_MS. A tap on the readout of the
// MagicFocus page shows it in the readout's place, and another tap brings the numbers back.
// ----------------------------------------------------------------------------
static const HotZone afReadoutBtn = {80, BODY_Y, 239, BODY_Y + 56};
static const int GRAPH_X = 70, GRAPH_W = 160;           // one sample per column, in the lower body sprite
static const int GRAPH_TOP = 14, GRAPH_BOTTOM = 55;     // the plot, below the legend
static const int GRAPH_LEGEND_Y = 10;                   // baseline of the legend
static const float GRAPH_NEAR_CM = 12.5f, GRAPH_FAR_CM = 1600.0f;  // log scale: a factor of 2 every 1/7 of the height
static const uint16_t COLOR_LIDAR = UI_SEL, COLOR_LENS = 0x4CFE;
static bool afGraph = false;
static uint16_t graphLidarCm[GRAPH_W], graphLensCm[GRAPH_W];  // 0 = no reading
static uint16_t graphNext = 0, graphCount = 0;
static uint32_t graphSamples = 0;  // grows with every sample, so the page knows when to redraw

// Records the lidar and lens distances once per graph column, whether or not the graph is showing.
static void sampleDistanceGraph() {
    static uint32_t lastSampleMs = 0;
    const uint32_t now = millis();
    if (now - lastSampleMs < GRAPH_WINDOW_MS / GRAPH_W) return;
    lastSampleMs = now;
    graphLidarCm[graphNext] = lidarValid() ? lidarCm : 0;
    const float lensMetres = motorPosition >= 0 ? distanceForFocus(lenses[lens], motorPosition) : -1.0f;
    graphLensCm[graphNext] = lensMetres < 0 ? 0 : (uint16_t)constrain(lroundf(lensMetres * 100.0f), 1L, (long)GRAPH_FAR_CM);
    graphNext = (graphNext + 1) % GRAPH_W;
    if (graphCount < GRAPH_W) ++graphCount;
    ++graphSamples;
}

// Height in the sprite for a distance, on the graph's log scale.
// @param cm Distance in centimetres.
static int graphY(float cm) {
    const float t = constrain(log2f(cm / GRAPH_NEAR_CM) / log2f(GRAPH_FAR_CM / GRAPH_NEAR_CM), 0.0f, 1.0f);
    return GRAPH_BOTTOM - (int)lroundf(t * (GRAPH_BOTTOM - GRAPH_TOP));
}

// Formats a legend entry: the name and the distance in metres, without the unit, which wouldn't fit, and with
// one decimal from 10 m on so the width stays the same; "--" without a reading.
// @param out Destination text buffer.
// @param size Capacity of the destination buffer in bytes.
// @param name Series name.
// @param cm Distance in centimetres; 0 = no reading.
static void formatLegend(char* out, size_t size, const char* name, uint16_t cm) {
    if (cm == 0) snprintf(out, size, "%s --", name);
    else snprintf(out, size, cm < 1000 ? "%s %.2f" : "%s %.1f", name, cm / 100.0f);
}

// Draws the graph in the readout's place: gridlines at 1 and 4 m, both distances with the newest at the right,
// and a legend with the current values.
static void drawDistanceGraph() {
    menu_lower_body.setFont(&fonts::DejaVu12);
    menu_lower_body.setTextDatum(L_BASELINE);
    menu_lower_body.setTextColor(GREY);
    for (const int cm : {100, 400}) {
        menu_lower_body.drawFastHLine(GRAPH_X, graphY(cm), GRAPH_W, DARK_GREY);
        menu_lower_body.drawString(cm == 100 ? "1m" : "4m", GRAPH_X + 1, graphY(cm) - 1);
    }
    const int oldest = graphNext + GRAPH_W - graphCount;  // ring index of the oldest sample, plus GRAPH_W
    for (const uint16_t* series : {graphLidarCm, graphLensCm}) {
        const uint16_t color = series == graphLidarCm ? COLOR_LIDAR : COLOR_LENS;
        for (int i = 1; i < graphCount; ++i) {
            const uint16_t from = series[(oldest + i - 1) % GRAPH_W], to = series[(oldest + i) % GRAPH_W];
            if (from == 0 || to == 0) continue;  // a gap where there was no reading
            const int x = GRAPH_X + GRAPH_W - graphCount + i;
            menu_lower_body.drawLine(x - 1, graphY(from), x, graphY(to), color);
        }
    }

    char text[20];
    const uint16_t lidarNow = graphCount ? graphLidarCm[(graphNext + GRAPH_W - 1) % GRAPH_W] : 0;
    const uint16_t lensNow = graphCount ? graphLensCm[(graphNext + GRAPH_W - 1) % GRAPH_W] : 0;
    formatLegend(text, sizeof(text), "Lidar", lidarNow);
    menu_lower_body.setTextDatum(L_BASELINE);
    menu_lower_body.setTextColor(COLOR_LIDAR);
    menu_lower_body.drawString(text, GRAPH_X, GRAPH_LEGEND_Y);
    formatLegend(text, sizeof(text), "Lens", lensNow);
    menu_lower_body.setTextDatum(R_BASELINE);
    menu_lower_body.setTextColor(COLOR_LENS);
    menu_lower_body.drawString(text, GRAPH_X + GRAPH_W - 1, GRAPH_LEGEND_Y);
}

// Draws the autofocus page: lidar/motor distance readout, or the distance graph, and the AF/radio buttons.
static void drawAfBody() {
    char text[24];
    menu_lower_body.fillSprite(COLOR_BACKGROUND);
    if (afGraph) {
        drawDistanceGraph();
    } else {
        menu_lower_body.setFreeFont(FSS9);
        menu_lower_body.setTextColor(WHITE);
        menu_lower_body.setTextDatum(ML_DATUM);

        menu_lower_body.drawString("Lidar:", 75, 15);
        formatMetres(text, sizeof(text), lidarValid() ? lidarCm / 100.0f : -1.0f);
        menu_lower_body.drawString(text, 150, 15);

        menu_lower_body.drawString("Lens:", 75, 43);
        const float lensMetres = motorPosition >= 0 ? distanceForFocus(lenses[lens], motorPosition) : -1.0f;
        if (lensMetres >= 0 || motorPosition < 0) formatMetres(text, sizeof(text), lensMetres);
        else snprintf(text, sizeof(text), "pos %d", motorPosition);
        menu_lower_body.drawString(text, 150, 43);
    }
    menu_lower_body.setFreeFont(FSS9);

    menu_lower_body.fillRect(0, 0, 65, 56, COLOR_BUTTON);
    menu_lower_body.setTextDatum(MC_DATUM);
    menu_lower_body.setTextColor(COLOR_BUTTON_TEXT);
    menu_lower_body.drawString("Speed", 32, 16);
    menu_lower_body.setFreeFont(FSSB9);
    menu_lower_body.setTextColor(afFast ? UI_SEL : COLOR_ACTIVE);
    menu_lower_body.drawString(afFast ? "Fast" : "Slow", 32, 40);

    menu_lower_body.fillRect(235, 0, 65, 56, COLOR_BUTTON);
    menu_lower_body.setFreeFont(FSS9);
    menu_lower_body.setTextColor(COLOR_BUTTON_TEXT);
    menu_lower_body.drawString("Camera", 267, 16);
    menu_lower_body.setFreeFont(FSSB9);
    menu_lower_body.setTextColor(!cameraEnabled ? GREY : (cameraConnected ? COLOR_ACTIVE : UI_SEL));
    menu_lower_body.drawString(!cameraEnabled ? "Off" : (cameraConnected ? "OK" : "Search"), 267, 40);

    const char* label = afEnabled ? "AF: ON" : "AF: OFF";
    const uint16_t labelColor = afEnabled ? COLOR_ACTIVE : COLOR_BUTTON_TEXT;
    menu_lower_body.fillRect(0, 62, 95, 44, COLOR_BUTTON);
    menu_lower_body.setFreeFont(FSSB9);
    menu_lower_body.setTextDatum(MC_DATUM);
    menu_lower_body.setTextColor(labelColor);
    menu_lower_body.drawString(label, 47, 84);

    // The bridge has to answer before anything can reach the motor, so both states get their own label.
    const char* radioLabel = !bridgeReady ? "No bridge" : (bridgeSniffing ? "Sniffer" : (motorConnected ? "Motor OK" : "No motor"));
    menu_lower_body.fillRect(100, 62, 100, 44, COLOR_BUTTON);
    menu_lower_body.setTextColor(motorConnected ? COLOR_ACTIVE : (bridgeReady ? UI_SEL : GREY));
    menu_lower_body.drawString(radioLabel, 150, 84);

    menu_lower_body.fillRect(205, 62, 95, 44, COLOR_BUTTON);
    menu_lower_body.setTextColor(COLOR_BUTTON_TEXT);
    menu_lower_body.drawString("Infinity", 252, 84);
}

// Draws the calibration page: point selector, focus arrows and Save button.
static void drawCalBody() {
    char text[16];
    const LensCalibration& cal = lenses[lens];
    menu_lower_body.fillSprite(COLOR_BACKGROUND);
    menu_lower_body.setTextDatum(MC_DATUM);
    menu_lower_body.setTextColor(WHITE);

    // Calibration step: its distance and one dot per step; filled = selected, green = the lens has a
    // point for that step.
    menu_lower_body.fillRect(0, 0, 100, 70, COLOR_ITEM);
    menu_lower_body.setFreeFont(FSSB12);
    menu_lower_body.drawString(CAL_STEP_LABELS[calStep], 50, 26);
    for (uint8_t step = 0; step < CAL_STEPS; ++step) {
        bool saved = false;
        for (uint8_t i = 0; i < CAL_POINTS && !saved; ++i) saved = cal.distanceCm[i] != 0 && calStepFor(cal.distanceCm[i]) == step;
        const int x = 50 - (CAL_STEPS - 1) * 7 + step * 14;
        const uint16_t color = saved ? COLOR_ACTIVE : WHITE;
        if (step == calStep) menu_lower_body.fillCircle(x, 55, 4, color);
        else menu_lower_body.drawCircle(x, 55, 4, color);
    }

    // Focus position with the arrows that move the lens.
    menu_lower_body.fillRect(110, 0, 55, 70, COLOR_ITEM);
    menu_lower_body.fillRect(170, 0, 70, 70, COLOR_ITEM);
    menu_lower_body.fillRect(245, 0, 55, 70, COLOR_ITEM);
    menu_lower_body.fillTriangle(143, 25, 133, 35, 143, 45, WHITE);
    menu_lower_body.fillTriangle(267, 25, 277, 35, 267, 45, WHITE);
    if (focusTarget >= 0) snprintf(text, sizeof(text), "%d", focusTarget);
    else snprintf(text, sizeof(text), "--");
    menu_lower_body.drawString(text, 205, 35);

    // The lidar reading, then the buttons, Save to SD widest for its label.
    menu_lower_body.fillRect(0, 76, 80, 34, COLOR_ITEM);
    menu_lower_body.setFreeFont(FSS9);
    formatMetres(text, sizeof(text), lidarValid() ? lidarCm / 100.0f : -1.0f);
    menu_lower_body.drawString(text, 40, 93);

    menu_lower_body.fillRect(90, 76, 115, 34, COLOR_BUTTON);
    menu_lower_body.setFreeFont(FSSB9);
    menu_lower_body.setTextColor(COLOR_BUTTON_TEXT);
    menu_lower_body.drawString("Save to SD", 147, 93);

    menu_lower_body.fillRect(215, 76, 85, 34, COLOR_BUTTON);
    menu_lower_body.drawString("Infinity", 257, 93);
}

// Lens position along a curve's preview sketch.
// @param curve Curve to sketch.
// @param t Time through the move, 0 to 1.
// @return Share of the move covered; 1 is the goal, above 1 is past it.
static float curvePreview(FocusCurve curve, float t) {
    // An eased move from a to b between t0 and t1; finished moves are short and quick.
    auto move = [](float t, float t0, float t1, float a, float b) {
        if (t <= t0) return a;
        if (t >= t1) return b;
        const float s = (t - t0) / (t1 - t0);
        return a + (b - a) * s * s * (3 - 2 * s);
    };
    switch (curve) {
        case CURVE_SMOOTH: return move(t, 0.0f, 1.0f, 0.0f, 1.0f);
        case CURVE_LINEAR: return min(t * 1.6f, 1.0f);
        case CURVE_OVERSHOOT: return t < 0.45f ? move(t, 0.0f, 0.3f, 0.0f, 1.2f) : move(t, 0.45f, 0.65f, 1.2f, 1.0f);
        case CURVE_CREEP: return t < 0.45f ? move(t, 0.0f, 0.3f, 0.0f, 0.8f) : move(t, 0.45f, 0.6f, 0.8f, 1.0f);
        default: return move(t, 0.0f, 0.3f, 0.0f, 1.0f);
    }
}

// Draws the AF Curve page: one button per autofocus curve, each with a sketch of its moves.
static void drawCharBody() {
    menu_lower_body.fillSprite(COLOR_BACKGROUND);
    menu_lower_body.setFreeFont(FSSB9);
    menu_lower_body.setTextDatum(MC_DATUM);
    for (uint8_t i = 0; i < AF_CURVE_COUNT; ++i) {
        // The hot zones are in screen space; the sprite sits at (BODY_X, BODY_Y).
        const int x = curveBtns[i].x0 - BODY_X, y = curveBtns[i].y0 - BODY_Y;
        const int w = curveBtns[i].x1 - curveBtns[i].x0 + 1, h = curveBtns[i].y1 - curveBtns[i].y0 + 1;
        const bool selected = afCurve == AF_CURVES[i];
        const uint16_t color = selected ? COLOR_ACTIVE : COLOR_BUTTON_TEXT;
        menu_lower_body.fillRect(x, y, w, h, selected ? COLOR_ITEM : COLOR_BUTTON);
        menu_lower_body.setTextColor(color);
        menu_lower_body.drawString(curveName(AF_CURVES[i]), x + w / 2, y + 13);

        // Sketch of lens position over time below the name, with the goal as a grey line.
        const int gw = 40, gh = 18, gx = x + (w - gw) / 2, gy = y + h - 6 - gh;
        menu_lower_body.drawFastHLine(gx, gy, gw, DARK_GREY);
        int px = gx, py = gy + gh;
        for (int s = 1; s <= gw; ++s) {
            const int nx = gx + s;
            const int ny = gy + gh - (int)lroundf(curvePreview(AF_CURVES[i], (float)s / gw) * gh);
            menu_lower_body.drawLine(px, py, nx, ny, color);
            px = nx;
            py = ny;
        }
    }
}

// ----------------------------------------------------------------------------
// Info page: the things to remember, opened with the (i) next to the battery and scrolled by dragging
// ----------------------------------------------------------------------------

struct InfoParagraph {
    bool heading;
    const char* text;
};

// Plain ASCII only: the screen's fonts have no other characters.
static const InfoParagraph INFO_TEXT[] = {
    {true, "Lens and camera"},
    {false, "Lens switch on MF."},
    {false, "By-wire lenses: Linked to rotation degree, no auto power-off."},
    {true, "Motor"},
    {false, "Pair on Tilta CH1, then switch the Knob or handle off."},
    {false, "Each lens: hold the motor button 3 s. No end stops? Stop it by hand at both ends."},
    {true, "Calibration"},
    {false, "Calib. tab: aim at 0.5, 2 or 4 m, focus, Save to SD (2+ points)."},
    {false, "Test with AF: ON."},
    {true, "Lens list"},
    {false, "lenses.txt on the SD card: names and up to six points per lens."},
};

static const int INFO_X = 10, INFO_Y = 54, INFO_W = 300, INFO_H = FOOTER_Y - 4 - INFO_Y;   // ends above the footer
static const int INFO_HEADER_H = 30, INFO_TEXT_TOP = 36;   // the header, then the text below it
static const int INFO_LINE_H = 19, INFO_GAP = 7;            // between lines, and more between paragraphs
static const int INFO_TEXT_W = INFO_W - 30;                 // leaves room for the scrollbar
static const HotZone infoCloseBtn = {INFO_X + INFO_W - 44, INFO_Y, INFO_X + INFO_W, INFO_Y + INFO_HEADER_H + 6};

struct InfoLine {
    const char* text;   // within INFO_TEXT
    uint8_t length;
    bool heading;
    int16_t y;          // from the top of the text
};
static InfoLine infoLines[96];
static uint8_t infoLineCount = 0;
static int infoMaxScroll = 0;
static int infoScroll = 0;
static M5Canvas menu_info(&M5.Display);

// Wraps the info text to the page's width once, measuring each line in the font it is drawn in.
static void layoutInfo() {
    if (infoLineCount > 0) return;
    int y = 0;
    for (const InfoParagraph& paragraph : INFO_TEXT) {
        menu_info.setFreeFont(paragraph.heading ? FSSB9 : FSS9);
        if (paragraph.heading && y > 0) y += INFO_GAP;
        for (const char* start = paragraph.text; *start && infoLineCount < sizeof(infoLines) / sizeof(*infoLines);) {
            // The most whole words that fit, or one word alone if it doesn't.
            const char* fit = nullptr;
            for (const char* next = start; *next;) {
                while (*next == ' ') ++next;
                while (*next && *next != ' ') ++next;
                char candidate[96];
                const size_t length = (size_t)(next - start);
                if (length >= sizeof(candidate)) break;
                memcpy(candidate, start, length);
                candidate[length] = '\0';
                if (fit && menu_info.textWidth(candidate) > INFO_TEXT_W) break;
                fit = next;
            }
            if (!fit) break;
            infoLines[infoLineCount++] = {start, (uint8_t)(fit - start), paragraph.heading, (int16_t)y};
            y += INFO_LINE_H;
            for (start = fit; *start == ' '; ++start) {}
        }
        y += INFO_GAP;
    }
    infoMaxScroll = y + 4 > INFO_H - INFO_TEXT_TOP ? y + 4 - (INFO_H - INFO_TEXT_TOP) : 0;
}

// Draws the info page: its header with a close button, the text from infoScroll down, and a scrollbar.
static void drawInfoBody() {
    if (!menu_info.getBuffer()) {
        menu_info.setPsram(true);   // keeps the internal heap for Bluetooth and the other sprites
        if (!menu_info.createSprite(INFO_W, INFO_H)) {
            menu_info.setPsram(false);
            if (!menu_info.createSprite(INFO_W, INFO_H)) return;
        }
    }
    layoutInfo();
    infoScroll = constrain(infoScroll, 0, infoMaxScroll);
    menu_info.fillSprite(COLOR_BACKGROUND);
    menu_info.fillRoundRect(0, 0, INFO_W, INFO_HEADER_H, 6, COLOR_ITEM);
    menu_info.setTextDatum(ML_DATUM);
    menu_info.setTextColor(WHITE);
    menu_info.setFreeFont(FSSB9);
    menu_info.drawString("Good to know", 12, INFO_HEADER_H / 2 + 1);
    const int cx = INFO_W - 18, cy = INFO_HEADER_H / 2;
    for (int d = -1; d <= 1; ++d) {
        menu_info.drawLine(cx - 6 + d, cy - 6, cx + 6 + d, cy + 6, WHITE);
        menu_info.drawLine(cx - 6 + d, cy + 6, cx + 6 + d, cy - 6, WHITE);
    }

    const int visible = INFO_H - INFO_TEXT_TOP;
    menu_info.setClipRect(0, INFO_TEXT_TOP, INFO_W, visible);
    menu_info.setTextDatum(TL_DATUM);
    for (uint8_t i = 0; i < infoLineCount; ++i) {
        const InfoLine& line = infoLines[i];
        const int y = INFO_TEXT_TOP + line.y - infoScroll;
        if (y + INFO_LINE_H < INFO_TEXT_TOP || y > INFO_H) continue;
        char text[96];
        memcpy(text, line.text, line.length);
        text[line.length] = '\0';
        menu_info.setFreeFont(line.heading ? FSSB9 : FSS9);
        menu_info.setTextColor(line.heading ? UI_SEL : WHITE);
        menu_info.drawString(text, 10, y);
    }
    menu_info.clearClipRect();
    if (infoMaxScroll > 0) {
        const int trackTop = INFO_TEXT_TOP + 2, trackH = visible - 4;
        const int thumbH = max(16, trackH * visible / (visible + infoMaxScroll));
        const int thumbY = trackTop + (trackH - thumbH) * infoScroll / infoMaxScroll;
        menu_info.fillRoundRect(INFO_W - 7, trackTop, 4, trackH, 2, UI_BAR);
        menu_info.fillRoundRect(INFO_W - 7, thumbY, 4, thumbH, 2, GREY);
    }
    menu_info.pushSprite(INFO_X, INFO_Y);
}

// Scrolls the info page by a drag, redrawing it if the text moved.
// @param dy Pixels the finger moved up since the last call; negative when it moved down.
static void scrollInfo(int dy) {
    const int scrolled = constrain(infoScroll + dy, 0, infoMaxScroll);
    if (scrolled == infoScroll) return;
    infoScroll = scrolled;
    drawInfoBody();
}

// Draws whichever page is active and pushes it to the screen.
static void drawBody() {
    if (page == PAGE_INFO) {
        drawInfoBody();
        return;
    }
    if (page == PAGE_AF) drawAfBody();
    else if (page == PAGE_CAL) drawCalBody();
    else drawCharBody();
    menu_lower_body.pushSprite(BODY_X, BODY_Y);
}

// Battery icon in the footer's lower right corner, drawn like MagicPilot Remote's: the voltage inside, or the
// charge in percent after a tap.
static const HotZone batteryBtn = {262, FOOTER_Y, 319, 239};
static bool batteryShowsPercent = false;
static int16_t batteryMillivolts = -1;  // last reading; -1 until read
static int32_t batteryPercent = -1;     // last reading; -1 until read or if the power chip can't tell
static bool batteryCharging = false;

// Reads the battery voltage, charge and charging state from the power chip.
static void readBattery() {
    batteryMillivolts = M5.Power.getBatteryVoltage();
    batteryPercent = M5.Power.getBatteryLevel();
    batteryCharging = M5.Power.isCharging() == m5::Power_Class::is_charging;
}

// Formats the battery readout, or dashes if the power chip gave no reading.
// @param out Destination text buffer.
// @param size Capacity of the destination buffer in bytes.
static void formatBattery(char* out, size_t size) {
    if (batteryShowsPercent) {
        if (batteryPercent < 0) snprintf(out, size, "--%%");
        else snprintf(out, size, "%ld%%", (long)batteryPercent);
    } else {
        if (batteryMillivolts <= 0) snprintf(out, size, "-.-V");
        else snprintf(out, size, "%.1fV", batteryMillivolts / 1000.0f);
    }
}

// Colour of the battery icon: grey without a reading, green while charging, red when the charge is low.
static uint16_t batteryColor() {
    if (batteryShowsPercent ? batteryPercent < 0 : batteryMillivolts <= 0) return GREY;
    if (batteryCharging) return COLOR_ACTIVE;
    if (batteryPercent >= 0 && batteryPercent < BATTERY_LOW_PERCENT) return RED;
    return WHITE;
}

// Draws the battery icon in the lower right corner: a 34x14 pill with a tip, the readout centred inside in bold.
static void drawBattery() {
    char text[12];
    formatBattery(text, sizeof(text));
    const uint16_t color = batteryColor();
    const int x = 273, y = FOOTER_MID - 7;
    M5.Display.fillRect(batteryBtn.x0, FOOTER_Y, batteryBtn.x1 - batteryBtn.x0 + 1, FOOTER_H, COLOR_BACKGROUND);
    M5.Display.fillRoundRect(x, y, 34, 14, 2, color);
    M5.Display.fillRect(x + 34, y + 4, 3, 6, color);
    // The 6x8 font has no bold face, so draw the text twice a pixel apart, without a background that would
    // paint over the first pass.
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextDatum(MC_DATUM);
    M5.Display.setTextColor(BLACK);
    M5.Display.drawString(text, x + 16, y + 7);
    M5.Display.drawString(text, x + 17, y + 7);
}

// Reads the battery every BATTERY_REFRESH_MS and redraws the icon when its text or colour changes.
static void updateBattery() {
    static uint32_t lastReadMs = 0;
    static char drawn[12] = "";
    static uint16_t drawnColor = BLACK;  // never a battery colour, so the first reading is drawn
    const uint32_t now = millis();
    if (now - lastReadMs < BATTERY_REFRESH_MS) return;
    lastReadMs = now;
    readBattery();
    char text[12];
    formatBattery(text, sizeof(text));
    const uint16_t color = batteryColor();
    if (strcmp(text, drawn) == 0 && color == drawnColor) return;
    strcpy(drawn, text);
    drawnColor = color;
    drawBattery();
}

// Switches the battery readout between the voltage and the charge in percent.
static void toggleBatteryReadout() {
    batteryShowsPercent = !batteryShowsPercent;
    drawBattery();
}

// The (i) left of the battery that opens the info page; orange while the page is open.
static const HotZone infoBtn = {232, FOOTER_Y, 259, 239};

static void drawInfoIcon() {
    const int cx = 248, cy = FOOTER_MID;
    const uint16_t color = page == PAGE_INFO ? UI_SEL : WHITE;
    M5.Display.fillRect(infoBtn.x0, FOOTER_Y, infoBtn.x1 - infoBtn.x0 + 1, FOOTER_H, COLOR_BACKGROUND);
    M5.Display.drawCircle(cx, cy, 7, color);
    M5.Display.fillRect(cx - 1, cy - 4, 2, 2, color);
    M5.Display.fillRect(cx - 1, cy - 1, 2, 5, color);
}

// Draws the footer: the status message, stopping short of the (i), then the (i) and the battery readout.
static void drawStatus() {
    M5.Display.fillRect(0, FOOTER_Y, 320, FOOTER_H, COLOR_BACKGROUND);
    M5.Display.setTextFont(2);
    M5.Display.setTextDatum(ML_DATUM);
    M5.Display.setTextColor(motorConnected ? COLOR_ACTIVE : GREY, COLOR_BACKGROUND);
    M5.Display.setClipRect(0, FOOTER_Y, infoBtn.x0 - 4, FOOTER_H);
    M5.Display.drawString(statusMessage, 10, FOOTER_MID);
    M5.Display.clearClipRect();
    drawInfoIcon();
    drawBattery();
}

// Redraws the whole screen: tabs, lens selector, body and status.
static void drawScreen() {
    M5.Display.fillRect(0, 50, 320, 190, COLOR_BACKGROUND);
    drawTabs();
    drawLensSelector();
    drawBody();
    drawStatus();
}
