/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#ifdef USE_SERIALRX_CRSF

#include "build/build_config.h"
#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"
#include "io/osd.h"

#include "rx/rx.h"
#include "rx/crsf.h"

// printing 
#include "fc/cli.h"


#include "telemetry/crsf.h"
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_POWER_COUNT 9

STATIC_UNIT_TESTED bool crsfFrameDone = false;
STATIC_UNIT_TESTED bool crsfFrameDone_3 = false;
STATIC_UNIT_TESTED crsfFrame_t crsfFrame;
STATIC_UNIT_TESTED crsfFrame_t crsfFrame_3;

STATIC_UNIT_TESTED uint32_t crsfChannelData[CRSF_MAX_CHANNEL];
STATIC_UNIT_TESTED uint32_t crsfChannelData_3[CRSF_MAX_CHANNEL];

static serialPort_t *serialPort;
static serialPort_t *serialPort_3;
static timeUs_t crsfFrameStartAt = 0;
static timeUs_t crsfFrameStartAt_3 = 0;
static uint8_t telemetryBuf[CRSF_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;

uint8_t rx_kind = 0;
uint32_t rx_switch_old = 0;
rxRuntimeConfig_t *rxRuntimeConfigCopy = NULL;
// Global function pointer declaration

typedef uint16_t (*RawFnPtr)(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan); // used by receiver driver to return channel data
typedef uint8_t (*StatusFnPtr)(rxRuntimeConfig_t *rxRuntimeConfig);


static timeUs_t last_print = 0;
static timeUs_t last_rx_switch = 0;
static timeUs_t flyaway_turned_on = 0;

RawFnPtr functionPointer_1C = NULL;
RawFnPtr functionPointer_1E = NULL;

StatusFnPtr functionPointer_2C = NULL;
StatusFnPtr functionPointer_2E = NULL;
// uint16_t (*functionPointer_1C)(const rxRuntimeConfig_t *, uint8_t) = NULL;
// uint16_t (*functionPointer_2C)(const rxRuntimeConfig_t *, uint8_t) = NULL;

// uint16_t (*functionPointer_1E)(const rxRuntimeConfig_t *, uint8_t) = NULL;
// uint16_t (*functionPointer_2E)(const rxRuntimeConfig_t *, uint8_t) = NULL;


const uint16_t crsfTxPowerStatesmW[CRSF_POWER_COUNT] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address> <Frame length> < Type> <Payload> < CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;
struct crsfPayloadRcChannelsPacked_s preparse_for_flyaway;

typedef struct crsfPayloadLinkStatistics_s {
    uint8_t     uplinkRSSIAnt1;
    uint8_t     uplinkRSSIAnt2;
    uint8_t     uplinkLQ;
    int8_t      uplinkSNR;
    uint8_t     activeAntenna;
    uint8_t     rfMode;
    uint8_t     uplinkTXPower;
    uint8_t     downlinkRSSI;
    uint8_t     downlinkLQ;
    int8_t      downlinkSNR;
} __attribute__ ((__packed__)) crsfPayloadLinkStatistics_t;

typedef struct crsfPayloadLinkStatistics_s crsfPayloadLinkStatistics_t;

STATIC_UNIT_TESTED uint8_t crsfFrameCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

STATIC_UNIT_TESTED uint8_t crsfFrameCRC_3(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame_3.frame.type);
    for (int ii = 0; ii < crsfFrame_3.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame_3.frame.payload[ii]);
    }
    return crc;
}

// Receive ISR callback, called back from serial port
STATIC_UNIT_TESTED void crsfDataReceive(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    static uint8_t crsfFramePosition = 0;
    const timeUs_t now = micros();

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = now - crsfFrameStartAt;
#endif

    if (now > crsfFrameStartAt + CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0;
    }

    if (crsfFramePosition == 0) {
        crsfFrameStartAt = now;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

    if (crsfFramePosition < fullFrameLength) {
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t)c;
        crsfFrameDone = crsfFramePosition < fullFrameLength ? false : true;
        if (crsfFrameDone) {
            crsfFramePosition = 0;
            if (crsfFrame.frame.type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
                // scam
                const uint8_t crc = crsfFrameCRC();
                if (crc == crsfFrame.bytes[fullFrameLength - 1]) {
                    switch (crsfFrame.frame.type)
                    {
#if defined(USE_MSP_OVER_TELEMETRY)
                        case CRSF_FRAMETYPE_MSP_REQ:
                        case CRSF_FRAMETYPE_MSP_WRITE: {
                            uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                            if (bufferCrsfMspFrame(frameStart, CRSF_FRAME_RX_MSP_FRAME_SIZE)) {
                                crsfScheduleMspResponse();
                            }
                            break;
                        }
#endif
                        default:
                            break;
                    }
                }
            }
        }
    }
}

// Receive ISR callback, called back from serial port
STATIC_UNIT_TESTED void crsfDataReceive_3(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    static uint8_t crsfFramePosition_3 = 0;
    const timeUs_t now = micros();

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = now - crsfFrameStartAt_3;
#endif

    if (now > crsfFrameStartAt_3 + CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition_3 = 0;
    }

    if (crsfFramePosition_3 == 0) {
        crsfFrameStartAt_3 = now;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = crsfFramePosition_3 < 3 ? 5 : crsfFrame_3.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

    if (crsfFramePosition_3 < fullFrameLength) {
        crsfFrame_3.bytes[crsfFramePosition_3++] = (uint8_t)c;
        crsfFrameDone_3 = crsfFramePosition_3 < fullFrameLength ? false : true;
        if (crsfFrameDone_3) {
            crsfFramePosition_3 = 0;
            if (crsfFrame_3.frame.type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
                // scam
                const uint8_t crc = crsfFrameCRC();
                if (crc == crsfFrame_3.bytes[fullFrameLength - 1]) {
                    switch (crsfFrame_3.frame.type)
                    {
#if defined(USE_MSP_OVER_TELEMETRY)
                        case CRSF_FRAMETYPE_MSP_REQ:
                        case CRSF_FRAMETYPE_MSP_WRITE: {
                            uint8_t *frameStart = (uint8_t *)&crsfFrame_3.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                            if (bufferCrsfMspFrame(frameStart, CRSF_FRAME_RX_MSP_FRAME_SIZE)) {
                                crsfScheduleMspResponse();
                            }
                            break;
                        }
#endif
                        default:
                            break;
                    }
                }
            }
        }
    }
}

STATIC_UNIT_TESTED uint16_t crsfReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    /* conversion from RC value to PWM
     *       RC     PWM
     * min  172 ->  988us
     * mid  992 -> 1500us
     * max 1811 -> 2012us
     * scale factor = (2012-988) / (1811-172) = 0.62477120195241
     * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
     */
    if(rx_kind == 0)
    {
        return (crsfChannelData[chan] * 1024 / 1639) + 881;
    }
    else
    {
        return (crsfChannelData_3[chan] * 1024 / 1639) + 881;
    }
}

STATIC_UNIT_TESTED uint16_t crsfReadRawRC_3(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    /* conversion from RC value to PWM
     *       RC     PWM
     * min  172 ->  988us
     * mid  992 -> 1500us
     * max 1811 -> 2012us
     * scale factor = (2012-988) / (1811-172) = 0.62477120195241
     * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
     */
    return (crsfChannelData_3[chan] * 1024 / 1639) + 881;
}


STATIC_UNIT_TESTED uint8_t crsfFrameStatus_3(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (crsfFrameDone_3) {
        crsfFrameDone_3 = false;
        if (crsfFrame_3.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC_3();
            if (crc != crsfFrame_3.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame_3.frame.frameLength = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            preparse_for_flyaway = *(crsfPayloadRcChannelsPacked_t *)crsfFrame_3.frame.payload;
            // unpack the RC channels

            // flyaway switch
            if(preparse_for_flyaway.chan11 > 1600){
                flyaway_turned_on = micros();
            }

            if(rx_kind == 1 && preparse_for_flyaway.chan11 < 1600 && micros() - flyaway_turned_on > 50000)
            {
                const crsfPayloadRcChannelsPacked_t* rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame_3.frame.payload;
                crsfChannelData_3[0] = rcChannels->chan0;
                crsfChannelData_3[1] = rcChannels->chan1;
                crsfChannelData_3[2] = rcChannels->chan2;
                crsfChannelData_3[3] = rcChannels->chan3;
                crsfChannelData_3[4] = rcChannels->chan4;
                crsfChannelData_3[5] = rcChannels->chan5;
                crsfChannelData_3[6] = rcChannels->chan6;
                crsfChannelData_3[7] = rcChannels->chan7;
                crsfChannelData_3[8] = rcChannels->chan8;
                crsfChannelData_3[9] = rcChannels->chan9;
                crsfChannelData_3[10] = rcChannels->chan10;
                crsfChannelData_3[11] = rcChannels->chan11;
                crsfChannelData_3[12] = rcChannels->chan12;
                crsfChannelData_3[13] = rcChannels->chan13;
                crsfChannelData_3[14] = rcChannels->chan14;
                crsfChannelData_3[15] = rcChannels->chan15;
                if(crsfChannelData_3[10] < 700)
                {
                    rx_kind = 0;
                    // cliPrint("ELRS: rx_kind: 1 -> 0\n");
                }
                // if(micros() - last_print > 1000000)
                // {
                //     last_print = micros();
                //     cliPrint("ELRS: ");
                //     char str[12]; // Buffer big enough for an integer
                //     itoa(crsfChannelData_3[10], str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     if(crsfChannelData_3[10] < 800)
                //     {
                //         cliPrint(" Switching to crsf\n");
                //     }
                //     else
                //     {
                //         cliPrint(" idle\n");
                //     }
                //     // cliPrint("\n");
                // }
            }


            return RX_FRAME_COMPLETE;
        }
        else if (crsfFrame_3.frame.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame_3.frame.payload[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame_3.frame.frameLength = CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            if(rx_kind == 1)
            {
                const crsfPayloadLinkStatistics_t* linkStats = (crsfPayloadLinkStatistics_t*)&crsfFrame_3.frame.payload;
                const uint8_t crsftxpowerindex = (linkStats->uplinkTXPower < CRSF_POWER_COUNT) ? linkStats->uplinkTXPower : 0;
                // if(linkStats->uplinkLQ < 10){
                //     rx_kind = 0;
                //     return RX_FRAME_PENDING;
                //     cliPrint("ELRS: rx_kind: 1 -> 0\n");
                // }
                rxLinkStatistics.uplinkRSSI = -1* (linkStats->activeAntenna ? linkStats->uplinkRSSIAnt2 : linkStats->uplinkRSSIAnt1);
                rxLinkStatistics.uplinkLQ = linkStats->uplinkLQ;
                rxLinkStatistics.uplinkSNR = linkStats->uplinkSNR;
                rxLinkStatistics.rfMode = linkStats->rfMode;
                rxLinkStatistics.uplinkTXPower = crsfTxPowerStatesmW[crsftxpowerindex];
                rxLinkStatistics.activeAntenna = linkStats->activeAntenna;

                // char str[12]; // Buffer big enough for an integer
                // cliPrint("ELRS rf uplinkLQ: ");
                // itoa(rxLinkStatistics.uplinkLQ, str, 10); // 10 is the base for decimal numbers
                // cliPrint(str);
                // cliPrint("\n");

                // if(micros() - last_print > 100000 && micros() > 30000000)
                // {
                //     last_print = micros();
                //     char str[12]; // Buffer big enough for an integer

                //     cliPrint("ELRS rf uplinkRSSI: ");
                //     itoa(rxLinkStatistics.uplinkRSSI, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");

                //     cliPrint("ELRS rf uplinkLQ: ");
                //     itoa(rxLinkStatistics.uplinkLQ, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");

                //     cliPrint("ELRS rf uplinkSNR: ");
                //     itoa(rxLinkStatistics.uplinkSNR, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");

                //     cliPrint("ELRS rf rfMode: ");
                //     itoa(rxLinkStatistics.rfMode, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");
                // }


#ifdef USE_OSD
            if (rxLinkStatistics.uplinkLQ > 0) {
                int16_t uplinkStrength;   // RSSI dBm converted to %
                uplinkStrength = constrain((100 * sq((osdConfig()->rssi_dbm_max - osdConfig()->rssi_dbm_min)) - (100 * sq((osdConfig()->rssi_dbm_max  - rxLinkStatistics.uplinkRSSI)))) / sq((osdConfig()->rssi_dbm_max - osdConfig()->rssi_dbm_min)),0,100);
                if (rxLinkStatistics.uplinkRSSI >= osdConfig()->rssi_dbm_max )
                    uplinkStrength = 99;
                else if (rxLinkStatistics.uplinkRSSI < osdConfig()->rssi_dbm_min)
                    uplinkStrength = 0;
                lqTrackerSet(rxRuntimeConfig->lqTracker, scaleRange(uplinkStrength, 0, 99, 0, RSSI_MAX_VALUE));
            } else {
                lqTrackerSet(rxRuntimeConfig->lqTracker, 0);
            }
#endif
            // This is not RC channels frame, update channel value but don't indicate frame completion
            }
            return RX_FRAME_PENDING;
        }
    }
    return RX_FRAME_PENDING;
}

STATIC_UNIT_TESTED uint8_t crsfFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    // if(rx_kind == 1){
    //     // return crsfFrameStatus_3(&rxRuntimeConfig);
    // }
    // else
    // {

    if (crsfFrameDone) {
        crsfFrameDone = false;
        if (crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame.frame.frameLength = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
            // preparse_for_flyaway = (crsfPayloadRcChannelsPacked_t)crsfFrame.frame.payload;
            preparse_for_flyaway = *(crsfPayloadRcChannelsPacked_t *)crsfFrame.frame.payload;

            // flyaway switch
            if(preparse_for_flyaway.chan11 > 1600){
                flyaway_turned_on = micros();
            }

            if(rx_kind == 0 && preparse_for_flyaway.chan11 < 1600 && micros() - flyaway_turned_on > 50000)
            {
                // unpack the RC channels
                const crsfPayloadRcChannelsPacked_t* rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame.frame.payload;

                crsfChannelData[0] = rcChannels->chan0;
                crsfChannelData[1] = rcChannels->chan1;
                crsfChannelData[2] = rcChannels->chan2;
                crsfChannelData[3] = rcChannels->chan3;
                crsfChannelData[4] = rcChannels->chan4;
                crsfChannelData[5] = rcChannels->chan5;
                crsfChannelData[6] = rcChannels->chan6;
                crsfChannelData[7] = rcChannels->chan7;
                crsfChannelData[8] = rcChannels->chan8;
                crsfChannelData[9] = rcChannels->chan9;
                crsfChannelData[10] = rcChannels->chan10;
                crsfChannelData[11] = rcChannels->chan11;
                crsfChannelData[12] = rcChannels->chan12;
                crsfChannelData[13] = rcChannels->chan13;
                crsfChannelData[14] = rcChannels->chan14;
                crsfChannelData[15] = rcChannels->chan15;

                // if(micros() - last_print > 1000000)
                // {
                //     last_print = micros();
                //     cliPrint("CRSF: ");
                //     char str[12]; // Buffer big enough for an integer
                //     itoa(crsfChannelData[10], str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");
                // }

                if(crsfChannelData[10] > 1600)
                {
                    rx_kind = 1;
                    // cliPrint("CRSF: rx_kind: 0 -> 1\n");
                }
            }
            return RX_FRAME_COMPLETE;
        }
        else if (crsfFrame.frame.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame.frame.payload[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame.frame.frameLength = CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
            if(rx_kind == 0)
            {
                const crsfPayloadLinkStatistics_t* linkStats = (crsfPayloadLinkStatistics_t*)&crsfFrame.frame.payload;
                const uint8_t crsftxpowerindex = (linkStats->uplinkTXPower < CRSF_POWER_COUNT) ? linkStats->uplinkTXPower : 0;
                // if(linkStats->uplinkLQ < 10){
                //     rx_kind = 1;
                //     // cliPrint("CRSF: rx_kind: 0 -> 1\n");
                //     return RX_FRAME_PENDING;
                // }
                rxLinkStatistics.uplinkRSSI = -1* (linkStats->activeAntenna ? linkStats->uplinkRSSIAnt2 : linkStats->uplinkRSSIAnt1);
                rxLinkStatistics.uplinkLQ = linkStats->uplinkLQ;
                rxLinkStatistics.uplinkSNR = linkStats->uplinkSNR;
                rxLinkStatistics.rfMode = linkStats->rfMode;
                rxLinkStatistics.uplinkTXPower = crsfTxPowerStatesmW[crsftxpowerindex];
                rxLinkStatistics.activeAntenna = linkStats->activeAntenna;


                // if(micros() - last_print > 100000 && micros() > 30000000)
                // {
                //     last_print = micros();
                //     char str[12]; // Buffer big enough for an integer

                //     cliPrint("CRSF rf uplinkRSSI: ");
                //     itoa(rxLinkStatistics.uplinkRSSI, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");

                //     cliPrint("CRSF rf uplinkLQ: ");
                //     itoa(rxLinkStatistics.uplinkLQ, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");

                //     cliPrint("CRSF rf uplinkSNR: ");
                //     itoa(rxLinkStatistics.uplinkSNR, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");

                //     cliPrint("CRSF rf rfMode: ");
                //     itoa(rxLinkStatistics.rfMode, str, 10); // 10 is the base for decimal numbers
                //     cliPrint(str);
                //     cliPrint("\n");
                // }

    #ifdef USE_OSD
                if (rxLinkStatistics.uplinkLQ > 0) {
                    int16_t uplinkStrength;   // RSSI dBm converted to %
                    uplinkStrength = constrain((100 * sq((osdConfig()->rssi_dbm_max - osdConfig()->rssi_dbm_min)) - (100 * sq((osdConfig()->rssi_dbm_max  - rxLinkStatistics.uplinkRSSI)))) / sq((osdConfig()->rssi_dbm_max - osdConfig()->rssi_dbm_min)),0,100);
                    if (rxLinkStatistics.uplinkRSSI >= osdConfig()->rssi_dbm_max )
                        uplinkStrength = 99;
                    else if (rxLinkStatistics.uplinkRSSI < osdConfig()->rssi_dbm_min)
                        uplinkStrength = 0;
                    lqTrackerSet(rxRuntimeConfig->lqTracker, scaleRange(uplinkStrength, 0, 99, 0, RSSI_MAX_VALUE));
                } else {
                    lqTrackerSet(rxRuntimeConfig->lqTracker, 0);
                }
    #endif
                // This is not RC channels frame, update channel value but don't indicate frame completion
            }
            return RX_FRAME_PENDING;
        }
    }
    return RX_FRAME_PENDING;
    // }
}


STATIC_UNIT_TESTED uint8_t status_frame_manger(rxRuntimeConfig_t *rxRuntimeConfig){

    if(rx_kind == 1){
        crsfFrameStatus(&rxRuntimeConfig);
        return crsfFrameStatus_3(&rxRuntimeConfig);
    }
    else if(rx_kind == 0)
    {
        crsfFrameStatus_3(&rxRuntimeConfig);
        return crsfFrameStatus(&rxRuntimeConfig);
    }

}

void crsfRxWriteTelemetryData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void crsfRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        // check that we are not in bi dir mode or that we are not currently receiving data (ie in the middle of an RX frame)
        // and that there is time to send the telemetry frame before the next RX frame arrives
        if (CRSF_PORT_OPTIONS & SERIAL_BIDIR) {
            const timeDelta_t timeSinceStartOfFrame = cmpTimeUs(micros(), crsfFrameStartAt);
            if ((timeSinceStartOfFrame < CRSF_TIME_NEEDED_PER_FRAME_US) ||
                (timeSinceStartOfFrame > CRSF_TIME_BETWEEN_FRAMES_US - CRSF_TIME_NEEDED_PER_FRAME_US)) {
                return;
            }
        }
        serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

bool crsfRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    // stealing the pointer :D
    rxRuntimeConfigCopy = rxRuntimeConfig;

    for (int ii = 0; ii < CRSF_MAX_CHANNEL; ++ii) {
        crsfChannelData[ii] = (16 * PWM_RANGE_MIDDLE) / 10 - 1408;
    }

    rxRuntimeConfig->channelCount = CRSF_MAX_CHANNEL;
    rxRuntimeConfig->rcReadRawFn = crsfReadRawRC;
    // rxRuntimeConfig->rcFrameStatusFn = crsfFrameStatus;
    rxRuntimeConfig->rcFrameStatusFn = status_frame_manger;

    // const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    // if (!portConfig) {
    //     return false;
    // }

    // serialPort = openSerialPort(portConfig->identifier,
    serialPort = openSerialPort(1,
        FUNCTION_RX_SERIAL,
        crsfDataReceive,
        NULL,
        CRSF_BAUDRATE,
        CRSF_PORT_MODE,
        CRSF_PORT_OPTIONS | (tristateWithDefaultOffIsActive(rxConfig->halfDuplex) ? SERIAL_BIDIR : 0)
        );

    return serialPort != NULL;
}

bool crsfRxInit_3(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    // stealing the pointer :D
    rxRuntimeConfigCopy = rxRuntimeConfig;

    for (int ii = 0; ii < CRSF_MAX_CHANNEL; ++ii) {
        crsfChannelData_3[ii] = (16 * PWM_RANGE_MIDDLE) / 10 - 1408;
    }

    rxRuntimeConfig->channelCount = CRSF_MAX_CHANNEL;
    rxRuntimeConfig->rcReadRawFn = crsfReadRawRC_3;
    // rxRuntimeConfig->rcFrameStatusFn = crsfFrameStatus_3;
    rxRuntimeConfig->rcFrameStatusFn = status_frame_manger;

    // const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    // if (!portConfig) {
    //     return false;
    // }

    // serialPort = openSerialPort(portConfig->identifier,
    serialPort_3 = openSerialPort(4,
        FUNCTION_RX_SERIAL,
        crsfDataReceive_3,
        NULL,
        CRSF_BAUDRATE,
        CRSF_PORT_MODE,
        CRSF_PORT_OPTIONS | (tristateWithDefaultOffIsActive(rxConfig->halfDuplex) ? SERIAL_BIDIR : 0)
        );
        
    return serialPort_3 != NULL;
}

bool dual_crsf_Init(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{

    // elrs
    crsfRxInit_3(rxConfig, rxRuntimeConfig);
    functionPointer_1E = crsfReadRawRC_3;
    functionPointer_2E = crsfFrameStatus_3;


    // crsf
    crsfRxInit(rxConfig, rxRuntimeConfig);
    functionPointer_1C = crsfReadRawRC;
    functionPointer_2C = crsfFrameStatus;



    
    if(rx_kind == 0)
    {
        rxRuntimeConfig->rcReadRawFn = functionPointer_1C;
        rxRuntimeConfig->rcFrameStatusFn = functionPointer_2C;
    }
    else if(rx_kind == 1)
    {
        rxRuntimeConfig->rcReadRawFn = functionPointer_1E;
        rxRuntimeConfig->rcFrameStatusFn = functionPointer_2E;
    }
    // rxRuntimeConfig->rcReadRawFn = functionPointer_1E;
    // rxRuntimeConfig->rcFrameStatusFn = functionPointer_2E;
    return true;
}

void switchRX(void)
{
    if(micros() - last_rx_switch > 150000){
        last_rx_switch = micros();
        if (rx_kind == 0)
        {
            rx_kind = 1;
            // rxRuntimeConfigCopy->rcReadRawFn = crsfReadRawRC_3;
            // rxRuntimeConfigCopy->rcFrameStatusFn = crsfFrameStatus_3;
        }
        else if (rx_kind == 1)
        {
            rx_kind = 0;
            // rxRuntimeConfigCopy->rcReadRawFn = crsfReadRawRC;
            // rxRuntimeConfigCopy->rcFrameStatusFn = crsfFrameStatus;
        }
    }
}

bool crsfRxIsActive(void)
{
    return true;
}

bool crsfRxIsActive_2(void)
{
    return true;
}
#endif
