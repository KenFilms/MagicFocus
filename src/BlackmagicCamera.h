/*
 * MagicFocus firmware: lidar autofocus for an external lens focus motor
 *
 * BlackmagicCamera.h: the Bluetooth link to a Blackmagic Pocket Cinema Camera 6K, which reports the
 * lens name so MagicFocus can switch to that lens's slot. Core2 only, as explained below.
 */
#pragma once

#include <sdkconfig.h>

// The camera link talks to the Bluedroid BLE stack directly. Arduino-esp32 3.x ships it for the
// ESP32 (Core2), but builds the ESP32-S3 (CoreS3) with NimBLE instead, where these headers don't exist;
// there the camera link is left out (see the stubs at the end of this file).
#ifdef CONFIG_BT_BLUEDROID_ENABLED

#include <Preferences.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_gatt_common_api.h>

// ============================================================================
// BLACKMAGIC CAMERA LENS DETECTION
//
// A listen-only Bluetooth link to the BMPCC. The camera reports the attached lens name as camera
// control group 12 "Metadata", parameter 9 "Lens type"; the matching calibration slot is selected.
// Focus itself still goes to the Tilta motor.
// ============================================================================

static const char* CAMERA_SERVICE_UUID = "291D567A-6D75-11E6-8B77-86F30CA893D3";
static const char* INCOMING_UUID = "B864E140-76A0-416A-BF30-5876504537D9";      // control packets the camera reports
static const char* DEVICE_NAME_UUID = "FFAC0C52-C9FB-41A0-B063-CC76282EB89C";   // the name this device reports to the camera
static const uint8_t CAMERA_GROUP_METADATA = 12, CAMERA_PARAM_LENS_TYPE = 9, CAMERA_TYPE_STRING = 5;

static esp_gatt_if_t gattInterface = ESP_GATT_IF_NONE;
static uint16_t gattConnectionId = 0;
static uint16_t serviceStartHandle = 0, serviceEndHandle = 0;
static uint16_t incomingHandle = 0, deviceNameHandle = 0;
static esp_gatt_char_prop_t incomingProperties = 0;
static esp_bd_addr_t cameraBda = {};
static esp_bd_addr_t pairingBda = {};
static volatile bool connecting = false;
static volatile bool bleStackReady = false;
static volatile bool scanParametersReady = false;
static volatile bool cameraFound = false;
static volatile bool gattConnectionOpen = false;
static volatile bool securityReady = false;
static volatile bool serviceSearchStarted = false;
static volatile bool passkeyReplyPending = false;
static volatile bool pinRequested = false;
static volatile bool pinSubmitted = false;
static bool pinScreenDrawn = false;

// The BLE callbacks run on the Bluetooth task, so they hand results to the main loop through these.
static const char* volatile pendingCameraStatus = nullptr;
static portMUX_TYPE cameraMux = portMUX_INITIALIZER_UNLOCKED;
static char reportedLens[sizeof(LensCalibration::name)] = {};
static volatile bool lensReported = false;
static char lastCameraLens[sizeof(LensCalibration::name)] = {};
static uint32_t lastCameraAttemptMs = 0;

static void beginPinEntry(esp_ble_gap_cb_param_t* parameter);
static void drawPinScreen();
static void replyWithEnteredPin();

// Queues a footer message from the Bluetooth task for the main loop to show.
// @param message String literal to display.
static void postCameraStatus(const char* message) {
    pendingCameraStatus = message;
}

// Stores a lens name reported by the camera for the main loop to apply.
// @param text Lens name bytes from the packet; not necessarily null-terminated.
// @param length Number of bytes available in text.
static void storeLensName(const char* text, size_t length) {
    length = strnlen(text, length);
    while (length > 0 && text[length - 1] == ' ') --length;
    if (length == 0) return;
    if (length >= sizeof(reportedLens)) length = sizeof(reportedLens) - 1;
    portENTER_CRITICAL(&cameraMux);
    memcpy(reportedLens, text, length);
    reportedLens[length] = '\0';
    lensReported = true;
    portEXIT_CRITICAL(&cameraMux);
}

// True if a control packet header starts at the given offset.
// @param bytes Notification data.
// @param length Number of bytes in the notification.
// @param offset Position to test.
static bool startsControlPacket(const uint8_t* bytes, size_t length, size_t offset) {
    return offset + 4 <= length && bytes[offset] == 0xFF && bytes[offset + 2] == 0;
}

// Picks the lens type reports out of the camera notifications.
// @param bytes Notification data, possibly holding several control packets.
// @param length Number of bytes in the notification.
static void parseControlPackets(const uint8_t* bytes, size_t length) {
    size_t offset = 0;
    while (offset + 4 <= length) {
        const uint8_t* packet = bytes + offset;
        const uint8_t payloadLength = packet[1];
        const size_t rawPacketLength = 4 + payloadLength;
        const size_t paddedPacketLength = 4 + ((payloadLength + 3) & ~((size_t)3));
        const size_t remainingLength = length - offset;
        if (rawPacketLength > remainingLength || packet[2] != 0) break;

        // The camera may or may not pad packets to 4 bytes, so decide by where the next packet starts.
        size_t packetLength = rawPacketLength;
        if (paddedPacketLength > rawPacketLength && paddedPacketLength <= remainingLength) {
            const bool rawNext = startsControlPacket(bytes, length, offset + rawPacketLength);
            const bool paddedNext = startsControlPacket(bytes, length, offset + paddedPacketLength);
            if (paddedNext && !rawNext) packetLength = paddedPacketLength;
            else if (remainingLength == paddedPacketLength) packetLength = paddedPacketLength;
        }
        // TEMPORARY: remove once lens detection is confirmed on the camera.
        if (payloadLength > 4 && packet[4] == CAMERA_GROUP_METADATA) {
            Serial.printf("[CAMERA] metadata param=%u type=%u len=%u\n", packet[5], packet[6], payloadLength);
        }
        if (payloadLength > 4 && packet[4] == CAMERA_GROUP_METADATA && packet[5] == CAMERA_PARAM_LENS_TYPE &&
            packet[6] == CAMERA_TYPE_STRING) {
            storeLensName((const char*)packet + 8, payloadLength - 4);
        }
        offset += packetLength;
    }
}

// Clears cached camera state after a disconnect.
static void markDisconnected() {
    cameraConnected = false;
    connecting = false;
    pinRequested = false;
    pinSubmitted = false;
    passkeyReplyPending = false;
    gattConnectionId = 0;
    serviceStartHandle = serviceEndHandle = 0;
    incomingHandle = deviceNameHandle = 0;
    incomingProperties = 0;
    gattConnectionOpen = securityReady = serviceSearchStarted = false;
}

// ----------------------------------------------------------------------------
// BLE advertisement & UUID helpers
// ----------------------------------------------------------------------------

// Value of one hex digit, or -1 if the character is not hex.
// @param value Character to convert.
static int hexNibble(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

// Converts a UUID string to the reversed byte order Bluedroid uses; len stays 0 if it is malformed.
// @param text UUID in the usual 8-4-4-4-12 hex form.
static esp_bt_uuid_t uuid128FromString(const char* text) {
    esp_bt_uuid_t uuid = {};
    uint8_t bytes[16] = {};
    size_t byteCount = 0;
    for (size_t index = 0; text[index] != '\0' && byteCount < sizeof(bytes);) {
        if (text[index] == '-') { ++index; continue; }
        const int high = hexNibble(text[index]);
        const int low = hexNibble(text[index + 1]);
        if (high < 0 || low < 0) return uuid;
        bytes[byteCount++] = (uint8_t)((high << 4) | low);
        index += 2;
    }
    if (byteCount != sizeof(bytes)) return uuid;
    uuid.len = ESP_UUID_LEN_128;
    for (size_t index = 0; index < sizeof(bytes); ++index) uuid.uuid.uuid128[index] = bytes[15 - index];
    return uuid;
}

// True if a discovered 128-bit UUID matches the UUID named by text.
// @param candidate UUID reported by the camera.
// @param text Expected UUID string.
static bool uuidMatches(const esp_bt_uuid_t& candidate, const char* text) {
    const esp_bt_uuid_t expected = uuid128FromString(text);
    return candidate.len == ESP_UUID_LEN_128 && expected.len == ESP_UUID_LEN_128 &&
           memcmp(candidate.uuid.uuid128, expected.uuid.uuid128, ESP_UUID_LEN_128) == 0;
}

// Copies the advertised device name (complete or shortened) into name; empty if there is none.
// @param advertisement Raw advertisement data.
// @param name Output buffer for the name.
// @param size Capacity of the output buffer in bytes.
static void advertisedName(uint8_t* advertisement, char* name, size_t size) {
    uint8_t nameLength = 0;
    uint8_t* nameData = esp_ble_resolve_adv_data(advertisement, ESP_BLE_AD_TYPE_NAME_CMPL, &nameLength);
    if (nameData == nullptr) nameData = esp_ble_resolve_adv_data(advertisement, ESP_BLE_AD_TYPE_NAME_SHORT, &nameLength);
    memset(name, 0, size);
    if (nameData != nullptr) memcpy(name, nameData, nameLength < size - 1 ? nameLength : size - 1);
}

// A camera advertises its name, the camera service UUID, or both.
// @param advertisement Raw advertisement data.
static bool isCameraAdvertisement(uint8_t* advertisement) {
    char name[64];
    advertisedName(advertisement, name, sizeof(name));
    if (strstr(name, "Blackmagic") != nullptr || strstr(name, "BMPCC") != nullptr) return true;

    uint8_t length = 0;
    uint8_t* uuids = esp_ble_resolve_adv_data(advertisement, ESP_BLE_AD_TYPE_128SRV_CMPL, &length);
    if (uuids == nullptr) return false;
    const esp_bt_uuid_t service = uuid128FromString(CAMERA_SERVICE_UUID);
    for (uint8_t offset = 0; offset + ESP_UUID_LEN_128 <= length; offset += ESP_UUID_LEN_128) {
        if (memcmp(uuids + offset, service.uuid.uuid128, ESP_UUID_LEN_128) == 0) return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
// BLE connection lifecycle — scan, open, pair, discover, subscribe
// ----------------------------------------------------------------------------

// Aborts the in-progress connection attempt, dropping any half-open link so the next scan can find the camera.
// @param message Footer message explaining the failure, or nullptr to stay quiet.
static void failConnection(const char* message) {
    connecting = false;
    if (gattConnectionOpen) esp_ble_gap_disconnect(cameraBda);
    if (message != nullptr) postCameraStatus(message);
}

// Starts the camera scan once the scan parameters have been accepted.
static void startCameraScan() {
    if (!connecting || !scanParametersReady) return;
    if (esp_ble_gap_start_scanning(8) != ESP_OK) failConnection(nullptr);
}

// Looks up a characteristic of the camera service; returns 0 if it is missing.
// @param uuidText UUID of the characteristic.
// @param properties Receives the characteristic's properties; may be nullptr.
static uint16_t findCharacteristicHandle(const char* uuidText, esp_gatt_char_prop_t* properties) {
    esp_gattc_char_elem_t characteristic = {};
    uint16_t count = 1;
    const esp_bt_uuid_t uuid = uuid128FromString(uuidText);
    if (uuid.len != ESP_UUID_LEN_128 ||
        esp_ble_gattc_get_char_by_uuid(gattInterface, gattConnectionId, serviceStartHandle, serviceEndHandle,
                                       uuid, &characteristic, &count) != ESP_GATT_OK || count == 0) return 0;
    if (properties != nullptr) *properties = characteristic.properties;
    return characteristic.char_handle;
}

// Stores the characteristic handles, names this device on the camera, and subscribes to its reports.
static void discoverCameraCharacteristics() {
    incomingHandle = findCharacteristicHandle(INCOMING_UUID, &incomingProperties);
    deviceNameHandle = findCharacteristicHandle(DEVICE_NAME_UUID, nullptr);
    if (deviceNameHandle != 0) {
        uint8_t deviceName[] = "MagicFocusEF";
        esp_ble_gattc_write_char(gattInterface, gattConnectionId, deviceNameHandle, sizeof(deviceName) - 1,
                                 deviceName, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_MITM);
    }
    if (incomingHandle == 0 || (incomingProperties & (ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE)) == 0) {
        failConnection("Camera reports unavailable");
        return;
    }
    cameraConnected = true;
    connecting = false;
    postCameraStatus("Camera connected");
    esp_ble_gattc_register_for_notify(gattInterface, cameraBda, incomingHandle);
}

// Subscribes to the camera's reports by writing the client config descriptor.
// @param characteristicHandle Handle of the incoming camera control characteristic.
static void writeClientConfig(uint16_t characteristicHandle) {
    esp_bt_uuid_t cccdUuid = {};
    cccdUuid.len = ESP_UUID_LEN_16;
    cccdUuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_gattc_descr_elem_t descriptor = {};
    uint16_t count = 1;
    if (esp_ble_gattc_get_descr_by_char_handle(gattInterface, gattConnectionId, characteristicHandle,
                                               cccdUuid, &descriptor, &count) != ESP_GATT_OK || count == 0) return;
    // CCCD value: 1 = notifications, 2 = indications.
    uint8_t config[2] = {(uint8_t)((incomingProperties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) ? 1 : 2), 0};
    esp_ble_gattc_write_char_descr(gattInterface, gattConnectionId, descriptor.handle, sizeof(config),
                                   config, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_MITM);
}

// The service search needs both the open connection and finished pairing.
static void startServiceSearchIfReady() {
    if (!gattConnectionOpen || !securityReady || serviceSearchStarted) return;
    esp_bt_uuid_t service = uuid128FromString(CAMERA_SERVICE_UUID);
    serviceSearchStarted = true;
    if (esp_ble_gattc_search_service(gattInterface, gattConnectionId, &service) != ESP_OK) {
        serviceSearchStarted = false;
        failConnection("Camera service search failed");
    }
}

// A camera advertisement was seen during the scan; stop scanning and open a GATT connection to it.
// @param parameter Scan result holding the camera's address.
static void connectToFoundCamera(esp_ble_gap_cb_param_t* parameter) {
    cameraFound = true;
    memcpy(cameraBda, parameter->scan_rst.bda, sizeof(cameraBda));
    esp_ble_gap_stop_scanning();
    if (esp_ble_gattc_open(gattInterface, cameraBda, parameter->scan_rst.ble_addr_type, true) != ESP_OK) {
        failConnection("Camera connection failed");
    }
}

// Handles one scan result or the end of the scan; ignored once a camera has already been chosen.
// @param parameter Scan event data.
static void handleScanEvent(esp_ble_gap_cb_param_t* parameter) {
    if (!connecting || cameraFound) return;
    if (parameter->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
        if (isCameraAdvertisement(parameter->scan_rst.ble_adv)) connectToFoundCamera(parameter);
    } else if (parameter->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
        failConnection(nullptr);  // retried quietly from updateCameraLink()
    }
}

// Pairing finished; on success look for the service, on failure drop the link.
// @param parameter Authentication result.
static void handleAuthComplete(esp_ble_gap_cb_param_t* parameter) {
    if (parameter->ble_security.auth_cmpl.success) {
        securityReady = true;
        startServiceSearchIfReady();
        return;
    }
    pinRequested = false;
    passkeyReplyPending = false;
    failConnection("Camera pairing failed");
}

// Handles scan results and pairing (security request, PIN request, authentication result).
// @param event GAP event type.
// @param parameter Event data.
static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* parameter) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            scanParametersReady = parameter->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS;
            if (scanParametersReady) startCameraScan();
            else failConnection("Bluetooth scan setup failed");
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            handleScanEvent(parameter);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            esp_ble_gap_security_rsp(parameter->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            beginPinEntry(parameter);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            handleAuthComplete(parameter);
            break;
        default:
            break;
    }
}

// Drives the connection: open, pair, find the service, enable notifications, then receive reports.
// @param event GATT client event type.
// @param interface GATT interface the event belongs to.
// @param parameter Event data.
static void gattClientEventHandler(esp_gattc_cb_event_t event, esp_gatt_if_t interface, esp_ble_gattc_cb_param_t* parameter) {
    if (event == ESP_GATTC_REG_EVT) {
        if (parameter->reg.status != ESP_GATT_OK) {
            postCameraStatus("Bluetooth registration failed");
            return;
        }
        gattInterface = interface;
        static esp_ble_scan_params_t scanParameters = {};
        scanParameters.scan_type = BLE_SCAN_TYPE_ACTIVE;
        scanParameters.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        scanParameters.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
        scanParameters.scan_interval = 0x50;
        scanParameters.scan_window = 0x30;
        scanParameters.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;
        if (esp_ble_gap_set_scan_params(&scanParameters) != ESP_OK) postCameraStatus("Bluetooth scan setup failed");
        return;
    }
    if (interface != gattInterface) return;

    switch (event) {
        case ESP_GATTC_CONNECT_EVT:
            gattConnectionId = parameter->connect.conn_id;
            memcpy(cameraBda, parameter->connect.remote_bda, sizeof(cameraBda));
            if (esp_ble_set_encryption(cameraBda, ESP_BLE_SEC_ENCRYPT_MITM) != ESP_OK) failConnection("Camera pairing failed");
            break;
        case ESP_GATTC_OPEN_EVT:
            if (parameter->open.status != ESP_GATT_OK) {
                failConnection("Camera connection failed");
                break;
            }
            gattConnectionId = parameter->open.conn_id;
            gattConnectionOpen = true;
            if (!cameraEnabled) {
                failConnection(nullptr);  // switched off while the connection was being opened
                break;
            }
            startServiceSearchIfReady();
            break;
        case ESP_GATTC_SEARCH_RES_EVT:
            if (uuidMatches(parameter->search_res.srvc_id.uuid, CAMERA_SERVICE_UUID)) {
                serviceStartHandle = parameter->search_res.start_handle;
                serviceEndHandle = parameter->search_res.end_handle;
            }
            break;
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (parameter->search_cmpl.status != ESP_GATT_OK || serviceStartHandle == 0) {
                failConnection("Camera service unavailable");
                break;
            }
            discoverCameraCharacteristics();
            break;
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            if (parameter->reg_for_notify.status == ESP_GATT_OK) writeClientConfig(parameter->reg_for_notify.handle);
            break;
        case ESP_GATTC_NOTIFY_EVT:
            parseControlPackets(parameter->notify.value, parameter->notify.value_len);
            break;
        case ESP_GATTC_DISCONNECT_EVT: {
            const bool wasConnected = cameraConnected;
            markDisconnected();
            if (wasConnected && cameraEnabled) postCameraStatus("Camera disconnected");
            break;
        }
        default:
            break;
    }
}

// Starts the BLE host and registers the GATT client; keyboard-only I/O makes the camera ask for a PIN.
static bool initializeBluetooth() {
    if (!btStarted() && !btStart()) return false;
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) return false;
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED && esp_bluedroid_init() != ESP_OK) return false;
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED && esp_bluedroid_enable() != ESP_OK) return false;
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) return false;

    esp_ble_auth_req_t authRequest = ESP_LE_AUTH_REQ_BOND_MITM;
    esp_ble_io_cap_t ioCapability = ESP_IO_CAP_IN;
    uint8_t keySize = 16;
    uint8_t keyMask = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    if (esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &authRequest, sizeof(authRequest)) != ESP_OK ||
        esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &ioCapability, sizeof(ioCapability)) != ESP_OK ||
        esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &keySize, sizeof(keySize)) != ESP_OK ||
        esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &keyMask, sizeof(keyMask)) != ESP_OK ||
        esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &keyMask, sizeof(keyMask)) != ESP_OK ||
        esp_ble_gatt_set_local_mtu(200) != ESP_OK ||
        esp_ble_gap_register_callback(gapEventHandler) != ESP_OK ||
        esp_ble_gattc_register_callback(gattClientEventHandler) != ESP_OK ||
        esp_ble_gattc_app_register(0) != ESP_OK) return false;
    bleStackReady = true;
    return true;
}

// Starts scanning for the camera; the rest of the connection runs from the BLE events.
static void beginConnection() {
    connecting = true;
    cameraFound = false;
    gattConnectionOpen = false;
    securityReady = false;
    serviceSearchStarted = false;
    serviceStartHandle = serviceEndHandle = 0;
    startCameraScan();
}

// Turns the camera Bluetooth link on or off and remembers the choice across restarts.
// @param enabled True to look for a BMPCC, false to leave Bluetooth idle.
static void setCameraEnabled(bool enabled) {
    if (enabled && !bleStackReady && !initializeBluetooth()) {
        setStatus("Camera Bluetooth unavailable");
        return;
    }
    cameraEnabled = enabled;
    Preferences preferences;
    preferences.begin("magicfocus", false);
    preferences.putBool("camera", enabled);
    preferences.end();
    if (enabled) {
        lastCameraAttemptMs = millis() - CAMERA_RETRY_MS;  // scan right away
        setStatus("Looking for camera...");
        return;
    }
    if (connecting && !cameraFound) esp_ble_gap_stop_scanning();
    connecting = false;
    if (gattConnectionOpen) esp_ble_gap_disconnect(cameraBda);
    lastCameraLens[0] = '\0';
    setStatus("Camera link off");
}

// Restores the saved Camera setting; Bluetooth is only started if it was on.
static void beginCameraLink() {
    Preferences preferences;
    preferences.begin("magicfocus", false);
    const bool enabled = preferences.getBool("camera", false);
    preferences.end();
    if (enabled) setCameraEnabled(true);
}

// Switches to the calibration slot named after the lens the camera reported, creating one if needed.
static void applyCameraLens() {
    if (!lensReported) return;
    char name[sizeof(reportedLens)];
    portENTER_CRITICAL(&cameraMux);
    memcpy(name, reportedLens, sizeof(name));
    lensReported = false;
    portEXIT_CRITICAL(&cameraMux);
    // Only act on a change, so a manual lens choice isn't overridden by repeated reports.
    if (strcmp(name, lastCameraLens) == 0) return;
    memcpy(lastCameraLens, name, sizeof(lastCameraLens));
    Serial.printf("[CAMERA] lens \"%s\"\n", name);

    // The camera reports the lens actually on it, so it switches even a locked lens (see below).
    int slot = -1, freeSlot = -1;
    for (uint8_t i = 0; i < MAX_LENSES && slot < 0; ++i) {
        if (strcasecmp(lenses[i].name, name) == 0) slot = i;
        else if (freeSlot < 0 && lenses[i].name[0] == '\0') freeSlot = i;
    }
    const char* saveError = nullptr;
    if (slot < 0) {
        if (freeSlot < 0) {
            setStatus("No free lens slot");
            return;
        }
        slot = freeSlot;
        strncpy(lenses[slot].name, name, sizeof(lenses[slot].name) - 1);
        saveError = saveLenses();
    }
    if (slot != lens) {
        activateLens((uint8_t)slot);
        if (lensLocked) setLensLock(true);   // the lock moves to the camera's lens, also across restarts
    } else {
        drawLensSelector();
    }
    char message[sizeof(statusMessage)];
    if (saveError) snprintf(message, sizeof(message), "Lens save failed: %s", saveError);
    else snprintf(message, sizeof(message), "Lens: %s", name);
    setStatus(message);
}

// Runs the camera link from the main loop: PIN screen, lens changes, and reconnecting.
static void updateCameraLink() {
    if (!bleStackReady) return;
    const char* message = pendingCameraStatus;
    if (message != nullptr) {
        pendingCameraStatus = nullptr;
        setStatus(message);
    }
    if (!cameraEnabled) return;
    if (pinRequested) {
        if (!pinScreenDrawn) drawPinScreen();
        if (pinSubmitted && passkeyReplyPending) replyWithEnteredPin();
        return;
    }
    if (pinScreenDrawn) {
        pinScreenDrawn = false;
        touchWasDown = true;  // the OK press is still held and would land on the Infinity button
        screenDirty = true;
    }
    applyCameraLens();
    const uint32_t now = millis();
    if (!connecting && !cameraConnected && !gattConnectionOpen && now - lastCameraAttemptMs >= CAMERA_RETRY_MS) {
        lastCameraAttemptMs = now;
        beginConnection();
    }
}

#else  // !CONFIG_BT_BLUEDROID_ENABLED

static const bool pinScreenDrawn = false;  // no camera, so the PIN keypad never takes over the screen

// The Camera button only explains why it does nothing on this board.
// @param enabled True if the user asked to turn the camera link on.
static void setCameraEnabled(bool enabled) {
    cameraEnabled = false;
    if (enabled) setStatus("Camera link not on this board");
}

static void beginCameraLink() {}
static void updateCameraLink() {}

#endif  // CONFIG_BT_BLUEDROID_ENABLED
