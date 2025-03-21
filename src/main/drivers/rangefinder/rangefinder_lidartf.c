/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_TF

#include "build/debug.h"
#include "build/build_config.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lidartf.h"

#define TF_DEVTYPE_NONE 0
#define TF_DEVTYPE_MINI 1
#define TF_DEVTYPE_02   2
#define TF_DEVTYPE_MINI_S 3

static uint8_t tfDevtype = TF_DEVTYPE_NONE;

#define TF_FRAME_LENGTH    6             // Excluding sync bytes (0x59) x 2 and checksum
#define TF_MINI_S_FRAME_LENGTH 7
#define TF_FRAME_SYNC_BYTE 0x59
#define TF_TIMEOUT_MS      (100 * 2)

//
// Benewake TFmini frame format
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   Integral time
// 8    5   Reserved
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If distance is 12m (1200cm), then OoR.
//
#define TF_MINI_FRAME_INTEGRAL_TIME 4

//
// Benewake TF02 frame format (From SJ-GU-TF02-01 Version: A01)
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   SIG (Reliability in 1~8, less than 7 is unreliable)
// 8    5   TIME (Exposure time, 3 or 6)
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If SIG is less than 7, unreliable
// 2. If distance is 22m (2200cm), then OoR.
//
#define TF_02_FRAME_SIG 4

// Maximum ratings

#define TF_MINI_RANGE_MIN 40
#define TF_MINI_RANGE_MAX 1200

#define TF_MINI_S_RANGE_MIN 30
#define TF_MINI_S_RANGE_MAX 1200

#define TF_02_RANGE_MIN 40
#define TF_02_RANGE_MAX 2200

#define TF_DETECTION_CONE_DECIDEGREES 900

static serialPort_t *tfSerialPort = NULL;

typedef enum {
    TF_FRAME_STATE_WAIT_START1,
    TF_FRAME_STATE_WAIT_START2,
    TF_FRAME_STATE_READING_PAYLOAD,
    TF_FRAME_STATE_WAIT_CKSUM,
} tfFrameState_e;

static tfFrameState_e tfFrameState;
static uint8_t tfFrame[TF_FRAME_LENGTH];
static uint8_t tfReceivePosition;

// TFmini
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
static uint8_t tfCmdTFmini[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

// TF02
// Same as TFmini for now..
static uint8_t tfCmdTF02[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

static int32_t  lidarTFValue   = -1;
static uint16_t lidarTFerrors = 0;

static void lidarTFSendCommand(void) {
    // TFmini-S does NOT need a special command, so we skip it
    switch (tfDevtype) {
        case TF_DEVTYPE_MINI:
            serialWriteBuf(tfSerialPort, tfCmdTFmini, sizeof(tfCmdTFmini));
            break;
        case TF_DEVTYPE_02:
            serialWriteBuf(tfSerialPort, tfCmdTF02, sizeof(tfCmdTF02));
            break;
        case TF_DEVTYPE_MINI_S:
        default:
            // Do nothing for TFmini-S or unknown
            break;
    }
}

void lidarTFInit(rangefinderDev_t *dev)
{
    UNUSED(dev);

    tfFrameState = TF_FRAME_STATE_WAIT_START1;
    tfReceivePosition = 0;

    lidarTFSendCommand();
}

void lidarTFUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);
    static timeMs_t lastFrameReceivedMs = 0;
    const timeMs_t timeNowMs = millis();

    if (tfSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(tfSerialPort)) {
        uint8_t c = serialRead(tfSerialPort);
        switch (tfFrameState) {
        case TF_FRAME_STATE_WAIT_START1:
            if (c == TF_FRAME_SYNC_BYTE) {
                tfFrameState = TF_FRAME_STATE_WAIT_START2;
            }
            break;

        case TF_FRAME_STATE_WAIT_START2:
            if (c == TF_FRAME_SYNC_BYTE) {
                tfReceivePosition = 0;
                tfFrameState = TF_FRAME_STATE_READING_PAYLOAD;
            } else {
                tfFrameState = TF_FRAME_STATE_WAIT_START1;
            }
            break;

        case TF_FRAME_STATE_READING_PAYLOAD:
            tfFrame[tfReceivePosition++] = c;

            {
                // Decide how many bytes we expect for the payload
                const uint8_t expectedFrameLength = (tfDevtype == TF_DEVTYPE_MINI_S)
                    ? TF_MINI_S_FRAME_LENGTH
                    : TF_FRAME_LENGTH;

                // When we've read enough payload bytes, go to WAIT_CKSUM
                if (tfReceivePosition == expectedFrameLength) {
                    tfFrameState = TF_FRAME_STATE_WAIT_CKSUM;
                }
            }
            break;

            case TF_FRAME_STATE_WAIT_CKSUM:
            {
                // Checksum is sum of everything including the two sync bytes
                uint8_t cksum = TF_FRAME_SYNC_BYTE + TF_FRAME_SYNC_BYTE;

                const uint8_t frameLength = (tfDevtype == TF_DEVTYPE_MINI_S)
                    ? TF_MINI_S_FRAME_LENGTH
                    : TF_FRAME_LENGTH;

                for (int i = 0; i < frameLength; i++) {
                    cksum += tfFrame[i];
                }

                // Compare c to cksum
                if (c == cksum) {
                    // Good frame
                    const uint16_t distance = (tfFrame[0] | (tfFrame[1] << 8));
                    const uint16_t strength = (tfFrame[2] | (tfFrame[3] << 8));

                    DEBUG_SET(DEBUG_LIDAR_TF, 0, distance);
                    DEBUG_SET(DEBUG_LIDAR_TF, 1, strength);

                    switch (tfDevtype) {

                    case TF_DEVTYPE_MINI_S: {
                        // TFmini-S has temperature in [4],[5]
                        const int16_t temp_raw = tfFrame[4] | (tfFrame[5] << 8);
                        const float temperature = (temp_raw / 8.0f) - 256;
                        DEBUG_SET(DEBUG_LIDAR_TF, 2, (int16_t)temperature);

                        if (distance >= TF_MINI_S_RANGE_MIN && distance < TF_MINI_S_RANGE_MAX) {
                            lidarTFValue = distance;
                        } else {
                            lidarTFValue = -1;
                        }
                        break;
                    }

                    case TF_DEVTYPE_MINI: {
                        // Original TFmini
                        if (distance >= TF_MINI_RANGE_MIN && distance < TF_MINI_RANGE_MAX) {
                            lidarTFValue = distance;
                            // The integral-time trick
                            if (tfFrame[TF_MINI_FRAME_INTEGRAL_TIME] == 7) {
                                lidarTFValue -= 13;
                            }
                        } else {
                            lidarTFValue = -1;
                        }
                        break;
                    }

                    case TF_DEVTYPE_02: {
                        // TF02
                        // Check reliability
                        if (distance >= TF_02_RANGE_MIN &&
                            distance < TF_02_RANGE_MAX &&
                            tfFrame[TF_02_FRAME_SIG] >= 7) {
                            lidarTFValue = distance;
                        } else {
                            lidarTFValue = -1;
                        }
                        break;
                    }

                    default:
                        lidarTFValue = -1;
                        break;
                    }
                    DEBUG_SET(DEBUG_LIDAR_TF, 4, 1);
                    lastFrameReceivedMs = timeNowMs;
                } else {
                    // Checksum error
                    ++lidarTFerrors;
                    DEBUG_SET(DEBUG_LIDAR_TF, 4, 0); // <--- Add this line: indicate error
                }
            }
            tfFrameState = TF_FRAME_STATE_WAIT_START1;
            tfReceivePosition = 0;
            break;
        }
    }

    // If valid frame hasn't been received for more than a timeout, re-send command
    if (timeNowMs - lastFrameReceivedMs > TF_TIMEOUT_MS) {
        lidarTFSendCommand();
    }
}

// Return most recent device output in cm

int32_t lidarTFGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    return lidarTFValue;
}

static bool lidarTFDetect(rangefinderDev_t *dev, uint8_t devtype)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);
    if (!portConfig) {
        return false;
    }

    tfSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, 115200, MODE_RXTX, 0);
    if (tfSerialPort == NULL) {
        return false;
    }

    tfDevtype = devtype;

    // ~10ms between readings
    dev->delayMs = 10;

    // Use correct maximum range
    if (devtype == TF_DEVTYPE_MINI) {
        dev->maxRangeCm = TF_MINI_RANGE_MAX;
    } else if (devtype == TF_DEVTYPE_MINI_S) {
        dev->maxRangeCm = TF_MINI_S_RANGE_MAX;
    } else {
        // Default for TF02 or unknown
        dev->maxRangeCm = TF_02_RANGE_MAX;
    }

    dev->detectionConeDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;

    dev->init  = &lidarTFInit;
    dev->update = &lidarTFUpdate;
    dev->read  = &lidarTFGetDistance;

    return true;
}

bool lidarTFminiDetect(rangefinderDev_t *dev)
{
    return lidarTFDetect(dev, TF_DEVTYPE_MINI);
}

bool lidarTFminiSDetect(rangefinderDev_t *dev)
{
    return lidarTFDetect(dev, TF_DEVTYPE_MINI_S);
}

bool lidarTF02Detect(rangefinderDev_t *dev)
{
    return lidarTFDetect(dev, TF_DEVTYPE_02);
}
#endif
