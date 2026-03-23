/*
 * BetaflightMSP - Arduino library for communicating with Betaflight flight controllers
 * 
 * This library implements the MSP (MultiWii Serial Protocol) used by Betaflight.
 * 
 * Author: Arduino MSP Library
 * License: GPL v3
 */

#include "BetaflightMSP.h"

BetaflightMSP::BetaflightMSP() {
    _port = nullptr;
    resetRxState();
}

void BetaflightMSP::begin(Stream &serialPort) {
    _port = &serialPort;
    resetRxState();
}

void BetaflightMSP::resetRxState() {
    _rxState = MSP_IDLE;
    _rxPayloadSize = 0;
    _rxPayloadIndex = 0;
    _rxCommand = 0;
    _rxChecksum = 0;
    _rxChecksum2 = 0;
    _rxDirection = 0;
    _rxError = false;
    _messageReceived = false;
    _responseReceived = false;
}

void BetaflightMSP::sendMSP(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, char direction, bool expectResponse) {
    if (!_port) return;
    
    // MSP V1 frame format: $M[direction][size][cmd][payload][checksum]
    uint8_t checksum = 0;
    
    // Header
    _port->write('$');
    _port->write('M');
    _port->write(direction);
    
    // Size
    _port->write(payloadSize);
    checksum ^= payloadSize;
    
    // Command
    _port->write(cmd & 0xFF);
    checksum ^= (cmd & 0xFF);
    
    // Payload
    for (uint8_t i = 0; i < payloadSize; i++) {
        _port->write(payload[i]);
        checksum ^= payload[i];
    }
    
    // Checksum
    _port->write(checksum);
    
    if (expectResponse) {
        _responseReceived = false;
        _messageReceived = false;
        _requestTimeout = millis() + 500;  // 500ms timeout
    }
}

bool BetaflightMSP::request(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, uint32_t timeout) {
    if (!_port) return false;
    
    resetRxState();
    sendMSP(cmd, payload, payloadSize, MSP_DIRECTION_REQUEST, true);
    
    uint32_t startTime = millis();
    while (millis() - startTime < timeout) {
        update();
        if (_responseReceived && _rxCommand == cmd && _rxDirection != MSP_DIRECTION_REQUEST) {
            return !_rxError;
        }
    }
    
    return false;  // Timeout
}

bool BetaflightMSP::command(uint16_t cmd, uint8_t *payload, uint8_t payloadSize) {
    if (!_port) return false;
    
    sendMSP(cmd, payload, payloadSize, MSP_DIRECTION_REQUEST, false);
    return true;
}

bool BetaflightMSP::reply(uint16_t cmd, uint8_t *payload, uint8_t payloadSize) {
    if (!_port) return false;

    sendMSP(cmd, payload, payloadSize, MSP_DIRECTION_RESPONSE, false);
    return true;
}

void BetaflightMSP::update() {
    if (!_port) return;
    
    while (_port->available()) {
        uint8_t c = _port->read();
        if (processReceivedByte(c)) {
            _messageReceived = true;
            _responseReceived = true;
            return;
        }
    }
}

void BetaflightMSP::clearMessage() {
    _messageReceived = false;
    _responseReceived = false;
}

bool BetaflightMSP::processReceivedByte(uint8_t c) {
    switch (_rxState) {
        case MSP_IDLE:
            if (c == '$') {
                _rxState = MSP_HEADER_START;
            }
            break;
            
        case MSP_HEADER_START:
            if (c == 'M') {
                _rxState = MSP_HEADER_M;
            } else {
                _rxState = MSP_IDLE;
            }
            break;
            
        case MSP_HEADER_M:
            if (c == '<') {
                _rxState = MSP_HEADER_ARROW;
                _rxDirection = MSP_DIRECTION_REQUEST;
                _rxError = false;
                _rxChecksum = 0;
            } else if (c == '>') {
                _rxState = MSP_HEADER_ARROW;
                _rxDirection = MSP_DIRECTION_RESPONSE;
                _rxError = false;
                _rxChecksum = 0;
            } else if (c == '!') {
                _rxState = MSP_HEADER_ARROW;
                _rxDirection = MSP_DIRECTION_ERROR;
                _rxError = true;
                _rxChecksum = 0;
            } else {
                _rxState = MSP_IDLE;
            }
            break;
            
        case MSP_HEADER_ARROW:
            _rxPayloadSize = c;
            _rxChecksum ^= c;
            _rxPayloadIndex = 0;
            _rxState = MSP_HEADER_SIZE;
            break;
            
        case MSP_HEADER_SIZE:
            _rxCommand = c;
            _rxChecksum ^= c;
            if (_rxPayloadSize > 0) {
                _rxState = MSP_PAYLOAD;
            } else {
                _rxState = MSP_CHECKSUM;
            }
            break;
            
        case MSP_PAYLOAD:
            _rxPayload[_rxPayloadIndex++] = c;
            _rxChecksum ^= c;
            if (_rxPayloadIndex >= _rxPayloadSize) {
                _rxState = MSP_CHECKSUM;
            }
            break;
            
        case MSP_CHECKSUM:
            if (_rxChecksum == c) {
                _rxState = MSP_COMMAND_RECEIVED;
                return true;  // Message received successfully
            } else {
                // Checksum error
                _rxError = true;
            }
            resetRxState();
            break;
            
        default:
            resetRxState();
            break;
    }
    
    return false;
}

// Convenience functions
bool BetaflightMSP::getApiVersion(msp_api_version_t &data) {
    if (!request(MSP_API_VERSION, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.protocolVersion = _rxPayload[offset++];
    data.apiVersionMajor = _rxPayload[offset++];
    data.apiVersionMinor = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getFcVariant(msp_fc_variant_t &data) {
    if (!request(MSP_FC_VARIANT, nullptr, 0)) {
        return false;
    }
    
    memcpy(data.identifier, _rxPayload, 4);
    data.identifier[4] = '\0';
    
    return true;
}

bool BetaflightMSP::getFcVersion(msp_fc_version_t &data) {
    if (!request(MSP_FC_VERSION, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.versionMajor = _rxPayload[offset++];
    data.versionMinor = _rxPayload[offset++];
    data.versionPatchLevel = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getBoardInfo(msp_board_info_t &data) {
    if (!request(MSP_BOARD_INFO, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    memcpy(data.boardIdentifier, &_rxPayload[offset], 4);
    data.boardIdentifier[4] = '\0';
    offset += 4;
    
    data.hardwareRevision = readU16(_rxPayload, offset);
    data.boardType = _rxPayload[offset++];
    data.targetCapabilities = _rxPayload[offset++];
    data.targetNameLength = _rxPayload[offset++];
    
    if (data.targetNameLength > 0 && data.targetNameLength < 32) {
        memcpy(data.targetName, &_rxPayload[offset], data.targetNameLength);
        data.targetName[data.targetNameLength] = '\0';
    } else {
        data.targetName[0] = '\0';
    }
    
    return true;
}

bool BetaflightMSP::getName(msp_name_t &data) {
    if (!request(MSP_NAME, nullptr, 0)) {
        return false;
    }
    
    uint8_t len = min(_rxPayloadSize, (uint8_t)15);
    memcpy(data.name, _rxPayload, len);
    data.name[len] = '\0';
    
    return true;
}

bool BetaflightMSP::getStatus(msp_status_t &data) {
    if (!request(MSP_STATUS, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.cycleTime = readU16(_rxPayload, offset);
    data.i2cErrorCounter = readU16(_rxPayload, offset);
    data.sensor = readU16(_rxPayload, offset);
    data.flightModeFlags = readU32(_rxPayload, offset);
    data.configProfileIndex = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getAttitude(msp_attitude_t &data) {
    if (!request(MSP_ATTITUDE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.roll = read16(_rxPayload, offset);
    data.pitch = read16(_rxPayload, offset);
    data.yaw = read16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getAltitude(msp_altitude_t &data) {
    if (!request(MSP_ALTITUDE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.estimatedAltitude = read32(_rxPayload, offset);
    data.vario = read16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getAnalog(msp_analog_t &data) {
    if (!request(MSP_ANALOG, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.vbat = _rxPayload[offset++];
    data.mAhDrawn = readU16(_rxPayload, offset);
    data.rssi = readU16(_rxPayload, offset);
    data.amperage = read16(_rxPayload, offset);
    
    // Voltage field added in API 1.42
    if (_rxPayloadSize >= 9) {
        data.voltage = readU16(_rxPayload, offset);
    } else {
        data.voltage = data.vbat * 10;  // Convert to 0.01V
    }
    
    return true;
}

bool BetaflightMSP::getRawGPS(msp_raw_gps_t &data) {
    if (!request(MSP_RAW_GPS, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.fixType = _rxPayload[offset++];
    data.numSat = _rxPayload[offset++];
    data.lat = read32(_rxPayload, offset);
    data.lon = read32(_rxPayload, offset);
    data.altCm = read16(_rxPayload, offset);
    data.groundSpeed = readU16(_rxPayload, offset);
    data.groundCourse = readU16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getCompGPS(msp_comp_gps_t &data) {
    if (!request(MSP_COMP_GPS, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.distanceToHome = readU16(_rxPayload, offset);
    data.directionToHome = readU16(_rxPayload, offset);
    data.heartbeat = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getBatteryState(msp_battery_state_t &data) {
    if (!request(MSP_BATTERY_STATE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.cellCount = _rxPayload[offset++];
    data.capacity = readU16(_rxPayload, offset);
    data.voltage = _rxPayload[offset++];
    data.mAhDrawn = readU16(_rxPayload, offset);
    data.amperage = readU16(_rxPayload, offset);
    data.batteryState = _rxPayload[offset++];
    data.voltage16 = readU16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getRC(msp_rc_t &data) {
    if (!request(MSP_RC, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    uint8_t channelCount = _rxPayloadSize / 2;
    for (uint8_t i = 0; i < channelCount && i < 18; i++) {
        data.channels[i] = readU16(_rxPayload, offset);
    }
    
    return true;
}

bool BetaflightMSP::getSeekerCamInfo(msp_seeker_cam_info_t &data) {
    if (!request(MSP_SEEKER_CAM_INFO, nullptr, 0)) {
        return false;
    }

    uint8_t offset = 0;
    data.width_px = readU16(_rxPayload, offset);
    data.height_px = readU16(_rxPayload, offset);
    data.intrinsics.fx_px_x1000 = readU32(_rxPayload, offset);
    data.intrinsics.fy_px_x1000 = readU32(_rxPayload, offset);
    data.intrinsics.cx_px_x1000 = readU32(_rxPayload, offset);
    data.intrinsics.cy_px_x1000 = readU32(_rxPayload, offset);
    data.hfov_deg = _rxPayload[offset++];
    data.vfov_deg = _rxPayload[offset++];
    data.tilt_angle_deg = (int8_t)_rxPayload[offset++];
    data.orientation = _rxPayload[offset++];
    data.lock_rate_hz = readU16(_rxPayload, offset);
    data.flags = _rxPayload[offset++];

    return true;
}

bool BetaflightMSP::getCameraRawLock(msp_camera_raw_lock_t &data) {
    if (!request(MSP_CAMERA_GET_LOCK, nullptr, 0)) {
        return false;
    }

    uint8_t offset = 0;
    data.flags = _rxPayload[offset++];
    data.x_px = readU16(_rxPayload, offset);
    data.y_px = readU16(_rxPayload, offset);

    return true;
}

bool BetaflightMSP::getCameraLock(msp_camera_lock_t &data) {
    if (!request(MSP_CAMERA_LOCK, nullptr, 0)) {
        return false;
    }

    uint8_t offset = 0;
    data.flags = _rxPayload[offset++];
    data.x_px = readU16(_rxPayload, offset);
    data.y_px = readU16(_rxPayload, offset);
    data.age_ms = readU16(_rxPayload, offset);

    return true;
}

bool BetaflightMSP::getFpvCamInfo(msp_fpv_cam_info_t &data) {
    if (!request(MSP_FPV_CAM_INFO, nullptr, 0)) {
        return false;
    }

    uint8_t offset = 0;
    data.intrinsics.fx_px_x1000 = readU32(_rxPayload, offset);
    data.intrinsics.fy_px_x1000 = readU32(_rxPayload, offset);
    data.intrinsics.cx_px_x1000 = readU32(_rxPayload, offset);
    data.intrinsics.cy_px_x1000 = readU32(_rxPayload, offset);
    data.tilt_angle_deg = (int8_t)_rxPayload[offset++];
    data.flags = _rxPayload[offset++];

    return true;
}

bool BetaflightMSP::setRawRC(uint16_t *channels, uint8_t channelCount) {
    if (channelCount > 18) channelCount = 18;
    
    uint8_t payload[36];  // Max 18 channels * 2 bytes
    uint8_t offset = 0;
    
    for (uint8_t i = 0; i < channelCount; i++) {
        writeU16(payload, offset, channels[i]);
    }
    
    return command(MSP_SET_RAW_RC, payload, offset);
}

bool BetaflightMSP::setGPSHome(int32_t lat, int32_t lon, uint16_t altitudeM) {
    uint8_t payload[10];
    uint8_t offset = 0;
    
    write32(payload, offset, lat);
    write32(payload, offset, lon);
    writeU16(payload, offset, altitudeM);
    
    return command(MSP_WP, payload, offset);
}

bool BetaflightMSP::setRawGPS(uint8_t fixType, uint8_t numSat, int32_t lat, int32_t lon, int16_t altM, uint16_t groundSpeed) {
    uint8_t payload[16];
    uint8_t offset = 0;
    
    payload[offset++] = fixType;
    payload[offset++] = numSat;
    write32(payload, offset, lat);
    write32(payload, offset, lon);
    writeU16(payload, offset, altM * 100);  // Convert meters to cm
    writeU16(payload, offset, groundSpeed);

    return command(MSP_SET_RAW_GPS, payload, offset);
}

bool BetaflightMSP::setSeekerCamInfo(const msp_seeker_cam_info_t &data) {
    uint8_t payload[27];
    uint8_t offset = 0;

    writeU16(payload, offset, data.width_px);
    writeU16(payload, offset, data.height_px);
    writeU32(payload, offset, data.intrinsics.fx_px_x1000);
    writeU32(payload, offset, data.intrinsics.fy_px_x1000);
    writeU32(payload, offset, data.intrinsics.cx_px_x1000);
    writeU32(payload, offset, data.intrinsics.cy_px_x1000);
    payload[offset++] = data.hfov_deg;
    payload[offset++] = data.vfov_deg;
    payload[offset++] = (uint8_t)data.tilt_angle_deg;
    payload[offset++] = data.orientation;
    writeU16(payload, offset, data.lock_rate_hz);
    payload[offset++] = data.flags;

    return request(MSP_SET_SEEKER_CAM_INFO, payload, offset);
}

bool BetaflightMSP::setFpvCamInfo(const msp_fpv_cam_info_t &data) {
    uint8_t payload[18];
    uint8_t offset = 0;

    writeU32(payload, offset, data.intrinsics.fx_px_x1000);
    writeU32(payload, offset, data.intrinsics.fy_px_x1000);
    writeU32(payload, offset, data.intrinsics.cx_px_x1000);
    writeU32(payload, offset, data.intrinsics.cy_px_x1000);
    payload[offset++] = (uint8_t)data.tilt_angle_deg;
    payload[offset++] = data.flags;

    return request(MSP_SET_FPV_CAM_INFO, payload, offset);
}

bool BetaflightMSP::replyCameraRawLock(const msp_camera_raw_lock_t &data) {
    uint8_t payload[5];
    uint8_t offset = 0;

    payload[offset++] = data.flags;
    writeU16(payload, offset, data.x_px);
    writeU16(payload, offset, data.y_px);

    return reply(MSP_CAMERA_GET_LOCK, payload, offset);
}
