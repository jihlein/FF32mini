/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the Naze32Pro Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// EEPROM Defines
///////////////////////////////////////////////////////////////////////////////

#define WRITE_ENABLE                    0x06
#define WRITE_DISABLE                   0x04
#define READ_STATUS_REGISTER            0x05
#define WRITE_STATUS_REGISTER           0x01
#define READ_DATA                       0x03
#define FAST_READ                       0x0B
#define FAST_READ_DUAL_OUTPUT           0x3B
#define FAST_READ_DUAL_IO               0xBB
#define PAGE_PROGRAM_256_BYTES          0x02
#define SECTOR_ERASE_4K                 0x20
#define BLOCK_ERASE_32KB                0x52
#define BLOCK_ERASE_64KB                0xD8
#define CHIP_ERASE                      0xC7
#define POWER_DOWN                      0xB9
#define RELEASE_POWER_DOWN              0xAB
#define MANUFACTURER_DEVICE_ID          0x90
#define MANUFACTURER_DEVICE_ID_DUAL_IO  0x92
#define JEDEC_ID                        0x9F
#define READ_UNIQUE_ID                  0x4B

///////////////////////////////////////

// Sensor data stored in 1st 4KB Sector, 0x000000

#define SENSOR_ADDR_BYTE2  0x00
#define SENSOR_ADDR_BYTE1  0x00
#define SENSOR_ADDR_BYTE0  0x00

// System data stored in 2nd 4KB Sector, 0x001000

#define SYSTEM_ADDR_BYTE2  0x00
#define SYSTEM_ADDR_BYTE1  0x01
#define SYSTEM_ADDR_BYTE0  0x00

///////////////////////////////////////////////////////////////////////////////
// EEPROM Variables
///////////////////////////////////////////////////////////////////////////////

static uint8_t sensorVersion = 1;
static uint8_t systemVersion = 2;

sensorEEPROM_u sensorEEPROM;
systemEEPROM_u systemEEPROM;

///////////////////////////////////////////////////////////////////////////////
// Write Enable
///////////////////////////////////////////////////////////////////////////////

void writeEnable(void)
{
	ENABLE_EEPROM;

    spiTransfer(EEPROM_SPI, WRITE_ENABLE);

    DISABLE_EEPROM;

    delayMicroseconds(2);
}

///////////////////////////////////////////////////////////////////////////////
// Sensor EEPROM Checksun
///////////////////////////////////////////////////////////////////////////////

uint32_t crc32bSensor(sensorEEPROM_u *e, int includeCRCAtEnd)
{
    return crc32B((uint32_t*)e, includeCRCAtEnd ? (uint32_t*)(e + 1) : e->data.sensorCRCAtEnd);
}

///////////////////////////////////////////////////////////////////////////////
// Read Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSensorEEPROM(void)
{
	sensorEEPROM_u *dst = &sensorEEPROM;

	int32_t index;

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, SENSOR_ADDR_BYTE2);
	spiTransfer(EEPROM_SPI, SENSOR_ADDR_BYTE1);
	spiTransfer(EEPROM_SPI, SENSOR_ADDR_BYTE0);

	index = -1;

	while (index++ < numberOfSensorBytes)
		sensorEEPROM.bytes[index] = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32bSensor(dst, true) )
    {
        evrPush(EVR_SensorCRCFail,0);
        dst->data.sensorCRCFlags |= CRC_HistoryBad;
    }
    else if ( dst->data.sensorCRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadSensorHistory,0);
    }

    ///////////////////////////////////

    //accConfidenceDecay = 1.0f / sqrt(sensorEEPROM.data.accelCutoff);
}

///////////////////////////////////////////////////////////////////////////////
// Write Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSensorEEPROM(void)

{
    sensorEEPROM_u *src = &sensorEEPROM;

    if (src->data.sensorCRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadSensorHistory,0);

    src->data.sensorCRCAtEnd[0] = crc32B( (uint32_t*)&src[0], src->data.sensorCRCAtEnd);

    ///////////////////////////////////

    cliPortPrint("Writing Sensor EEPROM\n");

    //writeEnable();
}

///////////////////////////////////////////////////////////////////////////////
// Check Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSensorEEPROM(bool eepromReset)
{
    uint8_t version;

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, SENSOR_ADDR_BYTE2);
	spiTransfer(EEPROM_SPI, SENSOR_ADDR_BYTE1);
	spiTransfer(EEPROM_SPI, SENSOR_ADDR_BYTE0);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != sensorVersion)
    {
		// Default settings
        sensorEEPROM.data.sensorVersion = sensorVersion;

	    ///////////////////////////////

        sensorEEPROM.data.accelBiasMPU[XAXIS] = 0.0f;
        sensorEEPROM.data.accelBiasMPU[YAXIS] = 0.0f;
        sensorEEPROM.data.accelBiasMPU[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorEEPROM.data.accelScaleFactorMPU[XAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        sensorEEPROM.data.accelScaleFactorMPU[YAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        sensorEEPROM.data.accelScaleFactorMPU[ZAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)

	    ///////////////////////////////

        sensorEEPROM.data.accelTCBiasSlope[XAXIS] = 0.0f;
        sensorEEPROM.data.accelTCBiasSlope[YAXIS] = 0.0f;
        sensorEEPROM.data.accelTCBiasSlope[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorEEPROM.data.accelTCBiasIntercept[XAXIS] = 0.0f;
        sensorEEPROM.data.accelTCBiasIntercept[YAXIS] = 0.0f;
        sensorEEPROM.data.accelTCBiasIntercept[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorEEPROM.data.gyroTCBiasSlope[ROLL ] = 0.0f;
        sensorEEPROM.data.gyroTCBiasSlope[PITCH] = 0.0f;
        sensorEEPROM.data.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    sensorEEPROM.data.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    sensorEEPROM.data.gyroTCBiasIntercept[PITCH] = 0.0f;
	    sensorEEPROM.data.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    sensorEEPROM.data.magBias[XAXIS] = 0.0f;
	    sensorEEPROM.data.magBias[YAXIS] = 0.0f;
	    sensorEEPROM.data.magBias[ZAXIS] = 0.0f;

		///////////////////////////////

		sensorEEPROM.data.accelCutoff = 1.0f;

		///////////////////////////////

	    sensorEEPROM.data.KpAcc = 5.0f;    // proportional gain governs rate of convergence to accelerometer
	    sensorEEPROM.data.KiAcc = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
	    sensorEEPROM.data.KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
	    sensorEEPROM.data.KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases

	    ///////////////////////////////

	    sensorEEPROM.data.compFilterA =  2.000f;
	    sensorEEPROM.data.compFilterB =  1.000f;

	    ///////////////////////////////////

	    sensorEEPROM.data.dlpfSetting = BITS_DLPF_CFG_98HZ;

	    ///////////////////////////////////

	    sensorEEPROM.data.sensorCRCFlags = 0;

	    ///////////////////////////////////

	    writeSensorEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
// System EEPROM Checksum
///////////////////////////////////////////////////////////////////////////////

uint32_t crc32bSystem(systemEEPROM_u *e, int includeCRCAtEnd)
{
    return crc32B((uint32_t*)e, includeCRCAtEnd ? (uint32_t*)(e + 1) : e->data.systemCRCAtEnd);
}

///////////////////////////////////////////////////////////////////////////////
// Read System EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSystemEEPROM(void)
{
	systemEEPROM_u *dst = &systemEEPROM;

	uint16_t index;

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, SYSTEM_ADDR_BYTE2);
	spiTransfer(EEPROM_SPI, SYSTEM_ADDR_BYTE1);
	spiTransfer(EEPROM_SPI, SYSTEM_ADDR_BYTE0);

	index = -1;

	while (index++ < numberOfSystemBytes)
		systemEEPROM.bytes[index] = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32bSystem(dst, true) )
    {
        evrPush(EVR_SystemCRCFail,0);
        dst->data.systemCRCFlags |= CRC_HistoryBad;
    }
    else if ( dst->data.systemCRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadSystemHistory,0);
    }

    ///////////////////////////////////

    //if (systemEEPROM.data.yawDirection >= 0)
	//    systemEEPROM.data.yawDirection = 1.0f;
	//else
    //   systemEEPROM.data.yawDirection = -1.0f;
}

///////////////////////////////////////////////////////////////////////////////
// Write System EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSystemEEPROM(void)

{
    systemEEPROM_u *src = &systemEEPROM;

    // there's no reason to write these values to EEPROM, they'll just be noise
    zeroPIDintegralError();
    zeroPIDstates();

    if (src->data.systemCRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadSystemHistory,0);

    src->data.systemCRCAtEnd[0] = crc32B( (uint32_t*)&src[0], src->data.systemCRCAtEnd);

    ///////////////////////////////////

    cliPortPrint("Writing System EEPROM\n");

    //writeEnable();
}

///////////////////////////////////////////////////////////////////////////////
// Check System EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSystemEEPROM(bool eepromReset)
{
    uint8_t version;

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, SYSTEM_ADDR_BYTE2);
	spiTransfer(EEPROM_SPI, SYSTEM_ADDR_BYTE1);
	spiTransfer(EEPROM_SPI, SYSTEM_ADDR_BYTE0);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != systemVersion)
    {
		// Default settings
        systemEEPROM.data.systemVersion = systemVersion;

	    ///////////////////////////////////

	    systemEEPROM.data.rollAndPitchRateScaling = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

        systemEEPROM.data.yawRateScaling          = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

        systemEEPROM.data.attitudeScaling         = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

        systemEEPROM.data.nDotEdotScaling         = 0.009f;                 // Stick to nDot/eDot scaling (9 mps)/(1000 RX PWM Steps) = 0.009

        systemEEPROM.data.hDotScaling             = 0.003f;                 // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

        ///////////////////////////////////

	    systemEEPROM.data.receiverType  = SPEKTRUM;

        systemEEPROM.data.slaveSpektrum = false;

	    parseRcChannels("TAER2134");

	    systemEEPROM.data.escPwmRate   = 450;
        systemEEPROM.data.servoPwmRate = 50;

        ///////////////////////////////////

        systemEEPROM.data.mixerConfiguration = MIXERTYPE_TRI;
        systemEEPROM.data.yawDirection = 1.0f;

        systemEEPROM.data.triYawServoPwmRate             = 50;
        systemEEPROM.data.triYawServoMin                 = 2000.0f;
        systemEEPROM.data.triYawServoMid                 = 3000.0f;
        systemEEPROM.data.triYawServoMax                 = 4000.0f;
        systemEEPROM.data.triCopterYawCmd500HzLowPassTau = 0.05f;

        // Free Mix Defaults to Quad X
		systemEEPROM.data.freeMixMotors        = 4;

		systemEEPROM.data.freeMix[0][ROLL ]    =  1.0f;
		systemEEPROM.data.freeMix[0][PITCH]    = -1.0f;
		systemEEPROM.data.freeMix[0][YAW  ]    = -1.0f;

		systemEEPROM.data.freeMix[1][ROLL ]    = -1.0f;
		systemEEPROM.data.freeMix[1][PITCH]    = -1.0f;
		systemEEPROM.data.freeMix[1][YAW  ]    =  1.0f;

		systemEEPROM.data.freeMix[2][ROLL ]    = -1.0f;
		systemEEPROM.data.freeMix[2][PITCH]    =  1.0f;
		systemEEPROM.data.freeMix[2][YAW  ]    = -1.0f;

		systemEEPROM.data.freeMix[3][ROLL ]    =  1.0f;
		systemEEPROM.data.freeMix[3][PITCH]    =  1.0f;
		systemEEPROM.data.freeMix[3][YAW  ]    =  1.0f;

		systemEEPROM.data.freeMix[4][ROLL ]    =  0.0f;
		systemEEPROM.data.freeMix[4][PITCH]    =  0.0f;
		systemEEPROM.data.freeMix[4][YAW  ]    =  0.0f;

		systemEEPROM.data.freeMix[5][ROLL ]    =  0.0f;
		systemEEPROM.data.freeMix[5][PITCH]    =  0.0f;
        systemEEPROM.data.freeMix[5][YAW  ]    =  0.0f;

        ///////////////////////////////////

        systemEEPROM.data.midCommand   = 3000.0f;
        systemEEPROM.data.minCheck     = (float)(MINCOMMAND + 200);
        systemEEPROM.data.maxCheck     = (float)(MAXCOMMAND - 200);
        systemEEPROM.data.minThrottle  = (float)(MINCOMMAND + 200);
        systemEEPROM.data.maxThrottle  = (float)(MAXCOMMAND);

        ///////////////////////////////////

        systemEEPROM.data.PID[ROLL_RATE_PID].B               =   1.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].P               = 250.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].I               = 100.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].D               =   0.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].iTerm           =   0.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].windupGuard     = 100.0f;  // PWMs
        systemEEPROM.data.PID[ROLL_RATE_PID].lastDcalcValue  =   0.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].lastDterm       =   0.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].lastLastDterm   =   0.0f;
        systemEEPROM.data.PID[ROLL_RATE_PID].dErrorCalc      =   D_ERROR;
        systemEEPROM.data.PID[ROLL_RATE_PID].type            =   OTHER;

        systemEEPROM.data.PID[PITCH_RATE_PID].B              =   1.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].P              = 250.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].I              = 100.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].D              =   0.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].iTerm          =   0.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].windupGuard    = 100.0f;  // PWMs
        systemEEPROM.data.PID[PITCH_RATE_PID].lastDcalcValue =   0.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].lastDterm      =   0.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].lastLastDterm  =   0.0f;
        systemEEPROM.data.PID[PITCH_RATE_PID].dErrorCalc     =   D_ERROR;
        systemEEPROM.data.PID[PITCH_RATE_PID].type           =   OTHER;

        systemEEPROM.data.PID[YAW_RATE_PID].B                =   1.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].P                = 350.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].I                = 100.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].D                =   0.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].iTerm            =   0.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].windupGuard      = 100.0f;  // PWMs
        systemEEPROM.data.PID[YAW_RATE_PID].lastDcalcValue   =   0.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].lastDterm        =   0.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].lastLastDterm    =   0.0f;
        systemEEPROM.data.PID[YAW_RATE_PID].dErrorCalc       =   D_ERROR;
        systemEEPROM.data.PID[YAW_RATE_PID].type             =   OTHER;

        systemEEPROM.data.PID[ROLL_ATT_PID].B                =   1.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].P                =   2.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].I                =   0.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].D                =   0.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].iTerm            =   0.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].windupGuard      =   0.5f;  // radians/sec
        systemEEPROM.data.PID[ROLL_ATT_PID].lastDcalcValue   =   0.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].lastDterm        =   0.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].lastLastDterm    =   0.0f;
        systemEEPROM.data.PID[ROLL_ATT_PID].dErrorCalc       =   D_ERROR;
        systemEEPROM.data.PID[ROLL_ATT_PID].type             =   ANGULAR;

        systemEEPROM.data.PID[PITCH_ATT_PID].B               =   1.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].P               =   2.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].I               =   0.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].D               =   0.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].iTerm           =   0.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].windupGuard     =   0.5f;  // radians/sec
        systemEEPROM.data.PID[PITCH_ATT_PID].lastDcalcValue  =   0.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].lastDterm       =   0.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].lastLastDterm   =   0.0f;
        systemEEPROM.data.PID[PITCH_ATT_PID].dErrorCalc      =   D_ERROR;
        systemEEPROM.data.PID[PITCH_ATT_PID].type            =   ANGULAR;

        systemEEPROM.data.PID[HEADING_PID].B                 =   1.0f;
        systemEEPROM.data.PID[HEADING_PID].P                 =   3.0f;
        systemEEPROM.data.PID[HEADING_PID].I                 =   0.0f;
        systemEEPROM.data.PID[HEADING_PID].D                 =   0.0f;
        systemEEPROM.data.PID[HEADING_PID].iTerm             =   0.0f;
        systemEEPROM.data.PID[HEADING_PID].windupGuard       =   0.5f;  // radians/sec
        systemEEPROM.data.PID[HEADING_PID].lastDcalcValue    =   0.0f;
        systemEEPROM.data.PID[HEADING_PID].lastDterm         =   0.0f;
        systemEEPROM.data.PID[HEADING_PID].lastLastDterm     =   0.0f;
        systemEEPROM.data.PID[HEADING_PID].dErrorCalc        =   D_ERROR;
        systemEEPROM.data.PID[HEADING_PID].type              =   ANGULAR;

        systemEEPROM.data.PID[NDOT_PID].B                    =   1.0f;
        systemEEPROM.data.PID[NDOT_PID].P                    =   3.0f;
        systemEEPROM.data.PID[NDOT_PID].I                    =   0.0f;
        systemEEPROM.data.PID[NDOT_PID].D                    =   0.0f;
        systemEEPROM.data.PID[NDOT_PID].iTerm                =   0.0f;
        systemEEPROM.data.PID[NDOT_PID].windupGuard          =   0.5f;
        systemEEPROM.data.PID[NDOT_PID].lastDcalcValue       =   0.0f;
        systemEEPROM.data.PID[NDOT_PID].lastDterm            =   0.0f;
        systemEEPROM.data.PID[NDOT_PID].lastLastDterm        =   0.0f;
        systemEEPROM.data.PID[NDOT_PID].dErrorCalc           =   D_ERROR;
        systemEEPROM.data.PID[NDOT_PID].type                 =   OTHER;

        systemEEPROM.data.PID[EDOT_PID].B                    =   1.0f;
        systemEEPROM.data.PID[EDOT_PID].P                    =   3.0f;
        systemEEPROM.data.PID[EDOT_PID].I                    =   0.0f;
        systemEEPROM.data.PID[EDOT_PID].D                    =   0.0f;
        systemEEPROM.data.PID[EDOT_PID].iTerm                =   0.0f;
        systemEEPROM.data.PID[EDOT_PID].windupGuard          =   0.5f;
        systemEEPROM.data.PID[EDOT_PID].lastDcalcValue       =   0.0f;
        systemEEPROM.data.PID[EDOT_PID].lastDterm            =   0.0f;
        systemEEPROM.data.PID[EDOT_PID].lastLastDterm        =   0.0f;
        systemEEPROM.data.PID[EDOT_PID].dErrorCalc           =   D_ERROR;
        systemEEPROM.data.PID[EDOT_PID].type                 =   OTHER;

        systemEEPROM.data.PID[HDOT_PID].B                    =   1.0f;
        systemEEPROM.data.PID[HDOT_PID].P                    =   2.0f;
        systemEEPROM.data.PID[HDOT_PID].I                    =   0.0f;
        systemEEPROM.data.PID[HDOT_PID].D                    =   0.0f;
        systemEEPROM.data.PID[HDOT_PID].iTerm                =   0.0f;
        systemEEPROM.data.PID[HDOT_PID].windupGuard          =   5.0f;
        systemEEPROM.data.PID[HDOT_PID].lastDcalcValue       =   0.0f;
        systemEEPROM.data.PID[HDOT_PID].lastDterm            =   0.0f;
        systemEEPROM.data.PID[HDOT_PID].lastLastDterm        =   0.0f;
        systemEEPROM.data.PID[HDOT_PID].dErrorCalc           =   D_ERROR;
        systemEEPROM.data.PID[HDOT_PID].type                 =   OTHER;

        systemEEPROM.data.PID[N_PID].B                       =   1.0f;
        systemEEPROM.data.PID[N_PID].P                       =   3.0f;
        systemEEPROM.data.PID[N_PID].I                       =   0.0f;
        systemEEPROM.data.PID[N_PID].D                       =   0.0f;
        systemEEPROM.data.PID[N_PID].iTerm                   =   0.0f;
        systemEEPROM.data.PID[N_PID].windupGuard             =   0.5f;
        systemEEPROM.data.PID[N_PID].lastDcalcValue          =   0.0f;
        systemEEPROM.data.PID[N_PID].lastDterm               =   0.0f;
        systemEEPROM.data.PID[N_PID].lastLastDterm           =   0.0f;
        systemEEPROM.data.PID[N_PID].dErrorCalc              =   D_ERROR;
        systemEEPROM.data.PID[N_PID].type                    =   OTHER;

        systemEEPROM.data.PID[E_PID].B                       =   1.0f;
        systemEEPROM.data.PID[E_PID].P                       =   3.0f;
        systemEEPROM.data.PID[E_PID].I                       =   0.0f;
        systemEEPROM.data.PID[E_PID].D                       =   0.0f;
        systemEEPROM.data.PID[E_PID].iTerm                   =   0.0f;
        systemEEPROM.data.PID[E_PID].windupGuard             =   0.5f;
        systemEEPROM.data.PID[E_PID].lastDcalcValue          =   0.0f;
        systemEEPROM.data.PID[E_PID].lastDterm               =   0.0f;
        systemEEPROM.data.PID[E_PID].lastLastDterm           =   0.0f;
        systemEEPROM.data.PID[E_PID].dErrorCalc              =   D_ERROR;
        systemEEPROM.data.PID[E_PID].type                    =   OTHER;

        systemEEPROM.data.PID[H_PID].B                       =   1.0f;
        systemEEPROM.data.PID[H_PID].P                       =   2.0f;
        systemEEPROM.data.PID[H_PID].I                       =   0.0f;
        systemEEPROM.data.PID[H_PID].D                       =   0.0f;
        systemEEPROM.data.PID[H_PID].iTerm                   =   0.0f;
        systemEEPROM.data.PID[H_PID].windupGuard             =   5.0f;
        systemEEPROM.data.PID[H_PID].lastDcalcValue          =   0.0f;
        systemEEPROM.data.PID[H_PID].lastDterm               =   0.0f;
        systemEEPROM.data.PID[H_PID].lastLastDterm           =   0.0f;
        systemEEPROM.data.PID[H_PID].dErrorCalc              =   D_ERROR;
        systemEEPROM.data.PID[H_PID].type                    =   OTHER;

        ///////////////////////////////////

        systemEEPROM.data.batteryCells             = 3;
		systemEEPROM.data.voltageMonitorScale      = 11.0f / 1.0f;
		systemEEPROM.data.voltageMonitorBias       = 0.0f;

		systemEEPROM.data.batteryLow               = 3.30f;
        systemEEPROM.data.batteryVeryLow           = 3.20f;
        systemEEPROM.data.batteryMaxLow            = 3.10f;

        ///////////////////////////////////

        systemEEPROM.data.armCount                 =  50;
		systemEEPROM.data.disarmCount              =  0;

		///////////////////////////////////

		systemEEPROM.data.activeTelemetry          =  0;
		systemEEPROM.data.mavlinkEnabled           =  false;

		///////////////////////////////////

		systemEEPROM.data.verticalVelocityHoldOnly = true;

		///////////////////////////////////

		systemEEPROM.data.systemCRCFlags = 0;

		///////////////////////////////////

	    writeSystemEEPROM();

	}
}

///////////////////////////////////////////////////////////////////////////////
