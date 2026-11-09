/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * TouchInput.h: turns touches into actions: the tabs and buttons, the held calibration arrows, and
 * dragging the info page.
 */
#pragma once

static bool touchWasDown = false;
static uint32_t touchStartMs = 0;
static int heldArrow = 0;         // calibration arrow the current touch started on: -1 left, +1 right, 0 none
static bool jogging = false;      // a held arrow is moving the motor step by step
static int lastTouchY = 0;        // where the current touch was last seen, for dragging the info page
static int pendingLensStep = 0;   // a touch on the lens selector, waiting to be a tap (-1, +1) or a long press
static Page pageBeforeInfo = PAGE_AF;

static void setCameraEnabled(bool enabled);

// Switches between the autofocus, calibration, characteristics and info pages.
// @param next Page to display.
static void openPage(Page next) {
    if (page == next) return;
    const bool infoChanged = page == PAGE_INFO || next == PAGE_INFO;
    page = next;
    if (page == PAGE_CAL && motorPosition >= 0) initFocusTarget(motorPosition);
    if (infoChanged) {
        // The info page covers the lens selector and changes the (i) in the footer.
        drawScreen();
        return;
    }
    drawTabs();
    drawBody();
}

// Opens the info page from the top, or closes it again back to the tab it was opened from.
static void toggleInfo() {
    if (page == PAGE_INFO) {
        openPage(pageBeforeInfo);
        return;
    }
    pageBeforeInfo = page;
    infoScroll = 0;
    openPage(PAGE_INFO);
}

// Makes a lens slot current, turning autofocus off if that lens isn't calibrated.
// @param index Lens slot to select.
static void activateLens(uint8_t index) {
    lens = index;
    if (afEnabled && savedPointCount(lenses[lens]) < 2) {
        afEnabled = false;
        setStatus("Lens not calibrated, AF off");
    }
    lastAutofocusCm = 0;  // a different lens has its own calibration curve
    drawLensSelector();
    drawBody();
}

// Locks the lens shown, or unlocks it. The arrows can't switch a locked lens; the camera's lens still does.
static void toggleLensLock() {
    setLensLock(!lensLocked);
    setStatus(lensLocked ? "Lens locked" : "Lens unlocked");
    drawLensSelector();
}

// Selects the previous or next lens.
// @param direction -1 for the previous lens, +1 for the next lens.
static void selectLens(int direction) {
    activateLens((uint8_t)((lens + MAX_LENSES + direction) % MAX_LENSES));
}

// Turns autofocus on or off, refusing to enable it if the lens isn't calibrated.
static void toggleAutofocus() {
    if (afEnabled) {
        afEnabled = false;
        setStatus("Autofocus off");
    } else if (savedPointCount(lenses[lens]) < 2) {
        setStatus("Calibrate 2+ points for this lens");
    } else if (!bridgeReady) {
        setStatus("No radio bridge");
    } else {
        afEnabled = true;
        setStatus("Autofocus on");
    }
    drawBody();
}

// Saves the current lidar distance and focus position as a calibration point.
static void saveCalibrationPoint() {
    if (!lidarValid()) {
        setStatus("No lidar reading");
        return;
    }
    if (focusTarget < 0) {
        setStatus("Set focus with the arrows first");
        return;
    }
    storeCalibrationPoint(lenses[lens], lidarCm, (int16_t)focusTarget);
    char message[sizeof(statusMessage)];
    const char* error = saveLenses();
    if (!error) snprintf(message, sizeof(message), "Point saved: %.2f m = %d", lidarCm / 100.0f, focusTarget);
    else snprintf(message, sizeof(message), "Saving the point failed: %s", error);
    setStatus(message);
    drawBody();
}

// Routes a touch to a hot zone on the current page.
// @param point Touch coordinates in screen space.
static void handlePress(const TouchPoint& point) {
    // TEMPORARY: remove once the unexplained lens movements are tracked down. Reveals phantom touches.
    Serial.printf("[TOUCH] x=%d y=%d page=%d\n", point.x, point.y, page);
    if (menu_mf.inHotZone(point)) openPage(PAGE_AF);
    else if (menu_calib.inHotZone(point)) openPage(PAGE_CAL);
    else if (menu_char.inHotZone(point)) openPage(PAGE_CHAR);
    else if (infoBtn.inHotZone(point)) toggleInfo();
    else if (batteryBtn.inHotZone(point)) toggleBatteryReadout();
    else if (page == PAGE_INFO) {
        // Only the close button acts; anywhere else a touch starts dragging the text.
        if (infoCloseBtn.inHotZone(point)) toggleInfo();
    } else if (lensPrevBtn.inHotZone(point) || lensNextBtn.inHotZone(point)) {
        // Decided when the touch ends or has lasted LENS_LOCK_HOLD_MS: a tap switches the lens, a long
        // press locks or unlocks it.
        pendingLensStep = lensPrevBtn.inHotZone(point) ? -1 : 1;
    }
    else if (page == PAGE_AF && afSpeedBtn.inHotZone(point)) {
        afFast = !afFast;
        setStatus(afFast ? "AF speed: fast" : "AF speed: slow");
        drawBody();
    } else if (page == PAGE_AF && afCameraBtn.inHotZone(point)) {
        setCameraEnabled(!cameraEnabled);
        drawBody();
    } else if (page == PAGE_AF && afReadoutBtn.inHotZone(point)) {
        afGraph = !afGraph;
        drawBody();
    } else if (page == PAGE_AF && afBtn.inHotZone(point)) toggleAutofocus();
    else if (page == PAGE_AF && radioBtn.inHotZone(point)) {
        pingBridge();
        setStatus("Re-checking the radio...");
    } else if (page == PAGE_AF && afInfinityBtn.inHotZone(point)) {
        driveToInfinity();
    } else if (page == PAGE_CAL) {
        if (calPointBtn.inHotZone(point)) {
            calStep = (calStep + 1) % CAL_STEPS;
            drawBody();
        } else if (calLeftBtn.inHotZone(point)) {
            heldArrow = -1;
            nudgeFocus(-1, CAL_STEP_FINE);
        } else if (calRightBtn.inHotZone(point)) {
            heldArrow = 1;
            nudgeFocus(1, CAL_STEP_FINE);
        }
        else if (calSaveBtn.inHotZone(point)) saveCalibrationPoint();
        else if (calInfinityBtn.inHotZone(point)) driveToInfinity();
    } else if (page == PAGE_CHAR) {
        for (uint8_t i = 0; i < AF_CURVE_COUNT; ++i) {
            if (!curveBtns[i].inHotZone(point)) continue;
            setAfCurve(AF_CURVES[i]);
            char message[sizeof(statusMessage)];
            snprintf(message, sizeof(message), "AF curve: %s", curveName(AF_CURVES[i]));
            setStatus(message);
            drawBody();
        }
    }
}

// Ends a continuous calibration-arrow move, if one is running.
static void endJog() {
    if (!jogging) return;
    jogging = false;
    stopFocus();
}

// Acts once per press; a held calibration arrow keeps moving the motor in small steps, larger after a second.
static void handleTouch() {
    const auto detail = M5.Touch.getDetail();
    const bool touchDown = M5.Touch.getCount() > 0 && detail.isPressed();
    const TouchPoint point = {detail.x, detail.y};
    const uint32_t now = millis();
    if (!touchDown) {
        // TEMPORARY: remove once the unexplained motor movements are tracked down.
        if (touchWasDown) Serial.printf("[TOUCH] released after %lu ms\n", (unsigned long)(now - touchStartMs));
        if (touchWasDown && pendingLensStep != 0) {
            if (lensLocked) setStatus("Lens locked: hold its name to unlock");
            else selectLens(pendingLensStep);
        }
        pendingLensStep = 0;
        touchWasDown = false;
        heldArrow = 0;
        endJog();
        return;
    }
    if (!touchWasDown) {
        touchWasDown = true;
        touchStartMs = now;
        heldArrow = 0;
        pendingLensStep = 0;
        lastTouchY = point.y;
        handlePress(point);
        return;
    }
    if (pendingLensStep != 0 && now - touchStartMs >= LENS_LOCK_HOLD_MS) {
        pendingLensStep = 0;   // the long press is used up; letting go does nothing more
        toggleLensLock();
        return;
    }
    if (page == PAGE_INFO) {
        // Dragging moves the text with the finger.
        scrollInfo(lastTouchY - point.y);
        lastTouchY = point.y;
        return;
    }
    if (heldArrow == 0 || now - touchStartMs < CAL_HOLD_AFTER_MS) return;
    const bool onArrow = page == PAGE_CAL && (heldArrow < 0 ? calLeftBtn : calRightBtn).inHotZone(point);
    if (!onArrow) {
        endJog();
        return;
    }
    jogging = true;
    jogFocus(heldArrow, now - touchStartMs > CAL_COARSE_AFTER_MS ? CAL_JOG_COARSE_COUNTS : CAL_JOG_FINE_COUNTS);
}

// Redraws the body only when one of its values changed, or the distance graph showing has a new sample.
static void refreshBody() {
    static uint32_t lastRefreshMs = 0;
    static int32_t drawn[9] = {};
    const uint32_t now = millis();
    if (page == PAGE_INFO || now - lastRefreshMs < UI_REFRESH_MS) return;
    lastRefreshMs = now;
    const int32_t current[9] = {lidarValid() ? lidarCm : -1, motorPosition, focusTarget, afEnabled, bridgeReady, motorConnected,
                                cameraEnabled, cameraConnected, page == PAGE_AF && afGraph ? (int32_t)graphSamples : 0};
    if (memcmp(current, drawn, sizeof(current)) == 0) return;
    memcpy(drawn, current, sizeof(drawn));
    drawBody();
}
