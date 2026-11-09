/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * FocusControl.h: moves the focus: autofocus from the lidar distance along the chosen focus curve,
 * the calibration arrows and the Infinity buttons, and sending the positions to the bridge.
 *
 * Snap, Overshoot and Creep and the calibration arrows make finished moves: each move is sent once, and
 * the next only when the motor should have finished the last one. The Smooth and Linear autofocus
 * curves and the Infinity buttons glide instead, sending a stream of positions as the handle does. In the
 * drift tests (docs/tilta-motor-protocol.md) streamed moves lost focus travel, probably because the
 * focus-by-wire test lens turns slow ring movement into less focus, so glides need the camera set to
 * move focus by rotation degree (README, "Focus-by-wire lenses").
 */
#pragma once

#include <Preferences.h>

// The three tabs, and the info page opened from the footer.
enum Page : uint8_t { PAGE_AF, PAGE_CAL, PAGE_CHAR, PAGE_INFO };

// How an autofocus move gets to its goal, chosen on the AF Curve page. The values are saved, so new
// curves go on the end.
enum FocusCurve : uint8_t { CURVE_SMOOTH, CURVE_LINEAR, CURVE_OVERSHOOT, CURVE_CREEP, CURVE_SNAP };
static const uint8_t AF_CURVE_COUNT = 5;
// On-screen order: the curves made of finished moves on top, the glides below.
static const FocusCurve AF_CURVES[AF_CURVE_COUNT] = {CURVE_SNAP, CURVE_OVERSHOOT, CURVE_CREEP, CURVE_SMOOTH, CURVE_LINEAR};

// Name of an autofocus curve, for the screen.
// @param curve Curve to name.
static const char* curveName(FocusCurve curve) {
    switch (curve) {
        case CURVE_SMOOTH: return "Smooth";
        case CURVE_LINEAR: return "Linear";
        case CURVE_OVERSHOOT: return "Overshoot";
        case CURVE_CREEP: return "Creep";
        default: return "Snap";
    }
}

// Whether a curve glides to its goal rather than making finished moves.
// @param curve Curve to check.
static bool curveGlides(FocusCurve curve) {
    return curve == CURVE_SMOOTH || curve == CURVE_LINEAR;
}

static Page page = PAGE_AF;
static uint8_t lens = 0;
static uint8_t calStep = 0;       // the Calib. tab's selected step (CAL_STEP_LABELS)
static bool afEnabled = false;
static bool afFast = false;
static bool cameraEnabled = false;            // look for a BMPCC over Bluetooth to detect the lens
static volatile bool cameraConnected = false;
static int focusTarget = -1;      // position sent to the motor; -1 = none yet
static int focusGoal = -1;        // where the motor should end up, in one or more moves or a glide
static int focusStop = -1;        // Overshoot and Creep: the stop on the way to focusGoal; -1 = none
static int focusChunk = 0;        // longest single move towards focusGoal; 0 = no limit
static uint32_t motorBusyUntilMs = 0;  // when the motor should have finished its current move
static bool gliding = false;      // Smooth and Linear: focusTarget is gliding to focusGoal
static FocusCurve glideCurve = CURVE_SMOOTH;
static uint32_t glideSpeed = 0;   // top speed of the glide, counts per second
static uint32_t glideAccel = 0;   // its acceleration and braking, counts per second squared
static float glideExact = 0;      // focusTarget with sub-count precision
static float glideVelocity = 0;   // counts per second, signed
static uint32_t lastGlideMs = 0;
static int lastSentFocus = -1;
static uint8_t lastSentMotion = 0;
static bool motionOverride = false;  // a test script is setting the direction-and-speed byte (SerialControl.h)
static uint8_t overrideMotion = 0;
static bool positionsHandsOff = false;  // a test script has the bridge play a sequence; send nothing meanwhile
static uint32_t lastSendMs = 0;
static bool afDriving = false;    // the current goal was set by autofocus
static FocusCurve afCurve = CURVE_SNAP;  // curve autofocus moves use

static bool lensLocked = false;   // the arrows can't switch the lens until it is unlocked (LENS_LOCK_HOLD_MS)

// Restores a lens locked before the last restart, locked.
static void loadLensLock() {
    Preferences preferences;
    preferences.begin("magicfocus", false);
    const int8_t locked = preferences.getChar("lensLock", -1);
    preferences.end();
    if (locked < 0 || locked >= MAX_LENSES) return;
    lens = (uint8_t)locked;
    lensLocked = true;
}

// Locks the current lens, or unlocks it, and remembers which lens is locked across restarts.
// @param locked true to lock it.
static void setLensLock(bool locked) {
    lensLocked = locked;
    Preferences preferences;
    preferences.begin("magicfocus", false);
    preferences.putChar("lensLock", locked ? (int8_t)lens : (int8_t)-1);
    preferences.end();
}

// Restores the saved autofocus curve.
static void loadAfCurve() {
    Preferences preferences;
    preferences.begin("magicfocus", false);
    const uint8_t saved = preferences.getUChar("afCurve", CURVE_SNAP);
    preferences.end();
    afCurve = saved <= CURVE_SNAP ? (FocusCurve)saved : CURVE_SNAP;
}

// Selects the autofocus curve and remembers it across restarts.
// @param curve Curve to use for autofocus moves from now on.
static void setAfCurve(FocusCurve curve) {
    afCurve = curve;
    Preferences preferences;
    preferences.begin("magicfocus", false);
    preferences.putUChar("afCurve", curve);
    preferences.end();
}

// The direction-and-speed byte sent with each position. While a glide runs it is the handle's: bit 7
// while the position falls, bits 6-0 the speed in steps of BRIDGE_SPEED_UNIT_COUNTS_PER_S. Finished
// moves are single jumps and send 0.
static uint8_t motionByte() {
    if (motionOverride) return overrideMotion;
    if (!BRIDGE_SEND_SPEED || !gliding || glideVelocity == 0) return 0;
    const long steps = lroundf(fabsf(glideVelocity) / BRIDGE_SPEED_UNIT_COUNTS_PER_S);
    return (uint8_t)((glideVelocity < 0 ? 0x80 : 0) | (steps > 127 ? 127 : steps));
}

// Sends the latest focus target to the bridge. The target is repeated now and then so a lost serial
// byte cannot leave the bridge holding a stale position; repeating the same position doesn't disturb
// a move.
static void pushFocusTarget() {
    if (!bridgeReady || focusTarget < 0 || positionsHandsOff) return;
    const uint32_t now = millis();
    if (now - lastSendMs < FOCUS_SEND_INTERVAL_MS) return;
    const uint8_t motion = motionByte();
    if (focusTarget == lastSentFocus && motion == lastSentMotion && now - lastSendMs < FOCUS_REPEAT_MS) return;
    lastSendMs = now;
    lastSentFocus = focusTarget;
    lastSentMotion = motion;
    sendMotorPosition((uint16_t)focusTarget, motion);
}

// How long the motor needs to finish a move, erring on the long side.
// @param counts Length of the move in motor counts, either direction.
static uint32_t moveDurationMs(int counts) {
    return MOTOR_MOVE_BASE_MS + (uint32_t)abs(counts) * 1000 / MOTOR_SPEED_COUNTS_PER_S;
}

// Gives focusTarget a starting point if it has none, preferring the last position the bridge sent.
// Starting from the fallback sends the motor there from wherever it is, so allow it a full-range move.
// @param fallback Position to start from when the bridge has none.
static void initFocusTarget(int fallback) {
    if (focusTarget >= 0) return;
    if (motorPosition >= 0) {
        focusTarget = motorPosition;
        return;
    }
    focusTarget = constrain(fallback, 0, (int)FOCUS_MAX);
    motorBusyUntilMs = millis() + moveDurationMs(FOCUS_MAX);
}

// Sends the motor to a position and notes when it should be there.
// @param position Motor position for this move.
static void startMove(int position) {
    position = constrain(position, 0, (int)FOCUS_MAX);
    motorBusyUntilMs = millis() + moveDurationMs(position - focusTarget);
    focusTarget = position;
}

// Ends a glide where focusTarget is now, and gives the motor a moment to catch up before any finished
// move.
static void endGlide() {
    if (!gliding) return;
    gliding = false;
    glideVelocity = 0;
    motorBusyUntilMs = millis() + MOTOR_MOVE_BASE_MS;
}

// Lets the motor finish its current move and makes no more.
static void stopFocus() {
    endGlide();
    focusGoal = focusTarget;
    focusStop = -1;
}

// Sets where the motor should end up; updateFocusMotion() gets it there in finished moves.
// @param goal Motor position to end up at.
// @param curve Snap goes in one move; Overshoot and Creep stop on the way (see Config.h).
// @param chunk Longest single move, for slower travel; 0 = no limit.
static void moveFocusTo(int goal, FocusCurve curve = CURVE_SNAP, int chunk = 0) {
    initFocusTarget(goal);
    endGlide();
    focusGoal = constrain(goal, 0, (int)FOCUS_MAX);
    focusChunk = chunk;
    focusStop = -1;
    const int travel = focusGoal - focusTarget;
    if (curve == CURVE_OVERSHOOT && abs(travel) >= AF_OVERSHOOT_MIN_TRAVEL) {
        const int extra = min(abs(travel) * AF_OVERSHOOT_PERCENT / 100, AF_OVERSHOOT_MAX_COUNTS);
        focusStop = constrain(focusGoal + (travel > 0 ? extra : -extra), 0, (int)FOCUS_MAX);
    } else if (curve == CURVE_CREEP && abs(travel) > AF_CREEP_ZONE_COUNTS) {
        focusStop = focusGoal - (travel > 0 ? AF_CREEP_ZONE_COUNTS : -AF_CREEP_ZONE_COUNTS);
    }
}

// Sets where the motor should end up; updateFocusMotion() glides focusTarget there a little at a time,
// and the motor follows. A glide already under way carries on from its current speed.
// @param goal Motor position to end up at.
// @param curve Smooth eases in and out; Linear runs at full speed from start to finish.
// @param speed Top speed in counts per second.
// @param accel Smooth's acceleration and braking in counts per second squared.
static void glideFocusTo(int goal, FocusCurve curve, uint32_t speed, uint32_t accel) {
    initFocusTarget(goal);
    if (!gliding) {
        gliding = true;
        glideExact = focusTarget;
        glideVelocity = 0;
        lastGlideMs = millis();
    }
    glideCurve = curve;
    glideSpeed = speed;
    glideAccel = accel;
    focusGoal = constrain(goal, 0, (int)FOCUS_MAX);
    focusChunk = 0;
    focusStop = -1;
}

// Moves focusTarget along the glide by the time since the last step.
// @param now Current millis().
static void glideStep(uint32_t now) {
    const float dt = min(now - lastGlideMs, (uint32_t)50) / 1000.0f;
    lastGlideMs = now;
    const float remaining = focusGoal - glideExact;
    if (glideCurve == CURVE_LINEAR) {
        glideVelocity = copysignf((float)glideSpeed, remaining);
    } else {
        // Fastest speed from which the glide can still brake to a stop exactly at the goal.
        const float speed = min((float)glideSpeed, sqrtf(2.0f * glideAccel * fabsf(remaining)));
        const float maxChange = glideAccel * dt;
        glideVelocity += constrain(copysignf(speed, remaining) - glideVelocity, -maxChange, maxChange);
    }
    glideExact += glideVelocity * dt;
    if (remaining == 0 || (focusGoal - glideExact) * remaining <= 0) {
        focusTarget = focusGoal;
        endGlide();
        return;
    }
    focusTarget = (int)lroundf(glideExact);
}

// Moves a glide on. Otherwise, once the motor should have finished its move, sends the next one: the
// stop on the way if there is one, else the goal, at most focusChunk counts at a time.
static void updateFocusMotion() {
    if (!bridgeReady || focusGoal < 0 || focusTarget < 0 || positionsHandsOff) return;
    const uint32_t now = millis();
    if ((int32_t)(now - motorBusyUntilMs) < 0) {
        lastGlideMs = now;  // a glide waits for the last finished move, then starts from rest
        return;
    }
    if (gliding) {
        glideStep(now);
        return;
    }
    if (focusStop == focusTarget) focusStop = -1;
    const int stop = focusStop >= 0 ? focusStop : focusGoal;
    if (stop == focusTarget) return;
    int next = stop;
    if (focusChunk > 0 && abs(next - focusTarget) > focusChunk) next = focusTarget + (next > focusTarget ? focusChunk : -focusChunk);
    startMove(next);
}

// Recomputes the autofocus goal from the lidar distance; updateFocusMotion() moves the motor there.
// Autofocus keeps running on the AF Curve page so curves can be tried out live.
static void updateAutofocus() {
    if (page == PAGE_CAL || !afEnabled || !bridgeReady) {
        if (afDriving) {
            afDriving = false;
            stopFocus();
        }
        return;
    }
    if (!lidarValid()) return;
    if (afDriving && lastAutofocusCm != 0 && abs((int)lidarCm - (int)lastAutofocusCm) < AF_DISTANCE_HYSTERESIS_CM) return;
    const int target = focusForDistance(lenses[lens], lidarCm);
    if (target < 0 || (afDriving && abs(target - focusGoal) < FOCUS_DEADBAND)) return;
    afDriving = true;
    lastAutofocusCm = lidarCm;
    if (curveGlides(afCurve)) {
        glideFocusTo(target, afCurve, afFast ? AF_SPEED_FAST_COUNTS_PER_S : AF_SPEED_SLOW_COUNTS_PER_S,
                     afFast ? AF_ACCEL_FAST_COUNTS_PER_S2 : AF_ACCEL_SLOW_COUNTS_PER_S2);
    } else {
        moveFocusTo(target, afCurve, afFast ? 0 : AF_SLOW_CHUNK_COUNTS);
    }
}

// Moves the focus by a small step in either direction.
// @param direction -1 to decrease motor position, +1 to increase it.
// @param step Number of motor-position counts to move.
static void nudgeFocus(int direction, int step) {
    if (!bridgeReady) {
        setStatus("No radio bridge");
        return;
    }
    initFocusTarget(FOCUS_MAX / 2);
    // Repeated taps add up; each goes out once the motor has finished the last move.
    moveFocusTo((focusGoal >= 0 ? focusGoal : focusTarget) + direction * step);
}

// While an arrow is held, keeps the motor moving one way in small finished moves; stopFocus() ends it.
// @param direction -1 to decrease motor position, +1 to increase it.
// @param step Counts per move.
static void jogFocus(int direction, int step) {
    if (!bridgeReady) return;
    initFocusTarget(FOCUS_MAX / 2);
    if (focusGoal >= 0 && focusGoal != focusTarget) return;   // the last step hasn't been sent yet
    if ((int32_t)(millis() - motorBusyUntilMs) < 0) return;   // or hasn't finished
    moveFocusTo(focusTarget + direction * step);
}

// Glides the motor all the way to the current lens's infinity end, eased in and out like Smooth.
static void driveToInfinity() {
    if (!bridgeReady) {
        setStatus("No radio bridge");
        return;
    }
    afEnabled = false;
    afDriving = false;
    glideFocusTo(infinityPosition(lenses[lens]), CURVE_SMOOTH, INFINITY_SPEED_COUNTS_PER_S, INFINITY_ACCEL_COUNTS_PER_S2);
    setStatus("Driving to infinity");
}
