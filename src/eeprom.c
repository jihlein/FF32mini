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
#define PAGE_PROGRAM_256_BYTES          0x02
#define SECTOR_ERASE_64KB               0xD8
#define CHIP_ERASE                      0xC7

///////////////////////////////////////

// Sensor data stored in 1st 64KB Sector, 0x000000

#define SENSOR_EEPROM_ADDR  0x000000

// System data stored in 2nd 64KB Sector, 0x010000

#define SYSTEM_EEPROM_ADDR  0x010000

///////////////////////////////////////////////////////////////////////////////
// EEPROM Variables
///////////////////////////////////////////////////////////////////////////////

static uint8_t sensorVersion = 1;
static uint8_t systemVersion = 1;

sensorEEPROM_u sensorEEPROM;
systemEEPROM_u systemEEPROM;

uint32andUint8_t sensorEEPROMaddr;
uint32andUint8_t systemEEPROMaddr;

///////////////////////////////////////////////////////////////////////////////
// EEPROM Busy
///////////////////////////////////////////////////////////////////////////////

void eepromBusy(void)
{
	uint8_t busy = 0x01;

	ENABLE_EEPROM;

    spiTransfer(EEPROM_SPI, READ_STATUS_REGISTER);

    while (busy == 0x01)
        busy = spiTransfer(EEPROM_SPI, 0x00) & 0x01;

    DISABLE_EEPROM;

    delayMicroseconds(2);
}

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
// Read Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSensorEEPROM(void)
{
	int32_t index;

	///////////////////////////////////

	setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

	sensorEEPROMaddr.value = SENSOR_EEPROM_ADDR;

	///////////////////////////////////

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[0]);

	for (index = 0; index < NUMBER_OF_SENSOR_BYTES; index++)
		sensorEEPROM.bytes[index] = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32B((uint32_t*)(&sensorEEPROM),       // CRC32B[sensorEEPROM CRC32B[sensorEEPROM]]
                               (uint32_t*)(&sensorEEPROM + 1)))
    {
        evrPush(EVR_SensorCRCFail,0);
        sensorEEPROM.value.CRCFlags |= CRC_HistoryBad;
    }
    else if ( sensorEEPROM.value.CRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadSensorHistory,0);
    }

    ///////////////////////////////////

    //accConfidenceDecay = 1.0f / sqrt(sensorEEPROM.value.accelCutoff);
}

///////////////////////////////////////////////////////////////////////////////
// Write Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSensorEEPROM(void)
{
    uint16_t byteCount = 0;
    uint32_t byteIndex = 0;
    uint8_t  pageIndex;

    ///////////////////////////////////

    if (sensorEEPROM.value.CRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadSensorHistory,0);

    sensorEEPROM.value.CRCAtEnd[0] = crc32B((uint32_t*)(&sensorEEPROM),                  // CRC32B[sensorEEPROM]
                                            (uint32_t*)(&sensorEEPROM.value.CRCAtEnd));

    ///////////////////////////////////

    setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

    sensorEEPROMaddr.value = SENSOR_EEPROM_ADDR;

    ///////////////////////////////////

    cliPortPrint("Erasing EEPROM Sector\n");

    // Sector Erase

    writeEnable();

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, SECTOR_ERASE_64KB);

	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[0]);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	eepromBusy();

    ///////////////////////////////////

    cliPortPrint("Writing Sensor EEPROM\n");

	// Program Page(s)

	for (pageIndex = 0; pageIndex < NUMBER_OF_SENSOR_PAGES; pageIndex++)
    {
        cliPortPrintF("  Page: %d\n", pageIndex);

        writeEnable();

        ENABLE_EEPROM;

        spiTransfer(EEPROM_SPI, PAGE_PROGRAM_256_BYTES);

		spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[2]);
		spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[1]);
		spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[0]);

	    for (byteCount = 0; byteCount < 256; byteCount++)
	    {
			spiTransfer(EEPROM_SPI, sensorEEPROM.bytes[byteIndex]);

			byteIndex++;

			if (byteIndex >= NUMBER_OF_SENSOR_BYTES)
			    break;
		}

		DISABLE_EEPROM;

		delayMicroseconds(2);

		eepromBusy();

		sensorEEPROMaddr.value += 0x0100;
    }

	readSensorEEPROM();
}

///////////////////////////////////////////////////////////////////////////////
// Check Sensor EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSensorEEPROM(bool eepromReset)
{
    uint8_t version;

    sensorEEPROMaddr.value = SENSOR_EEPROM_ADDR;

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, sensorEEPROMaddr.bytes[0]);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != sensorVersion)
    {
		// Default settings
        sensorEEPROM.value.version = sensorVersion;

	    ///////////////////////////////

        sensorEEPROM.value.accelBiasMPU[XAXIS] = 0.0f;
        sensorEEPROM.value.accelBiasMPU[YAXIS] = 0.0f;
        sensorEEPROM.value.accelBiasMPU[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorEEPROM.value.accelScaleFactorMPU[XAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        sensorEEPROM.value.accelScaleFactorMPU[YAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)
        sensorEEPROM.value.accelScaleFactorMPU[ZAXIS] = 0.00119708f;  // (1/8192) * 9.8065  (8192 LSB = 1 G)

	    ///////////////////////////////

        sensorEEPROM.value.accelTCBiasSlope[XAXIS] = 0.0f;
        sensorEEPROM.value.accelTCBiasSlope[YAXIS] = 0.0f;
        sensorEEPROM.value.accelTCBiasSlope[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorEEPROM.value.accelTCBiasIntercept[XAXIS] = 0.0f;
        sensorEEPROM.value.accelTCBiasIntercept[YAXIS] = 0.0f;
        sensorEEPROM.value.accelTCBiasIntercept[ZAXIS] = 0.0f;

        ///////////////////////////////

        sensorEEPROM.value.gyroTCBiasSlope[ROLL ] = 0.0f;
        sensorEEPROM.value.gyroTCBiasSlope[PITCH] = 0.0f;
        sensorEEPROM.value.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    sensorEEPROM.value.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    sensorEEPROM.value.gyroTCBiasIntercept[PITCH] = 0.0f;
	    sensorEEPROM.value.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    sensorEEPROM.value.magBias[XAXIS] = 0.0f;
	    sensorEEPROM.value.magBias[YAXIS] = 0.0f;
	    sensorEEPROM.value.magBias[ZAXIS] = 0.0f;

		///////////////////////////////

		sensorEEPROM.value.accelCutoff = 1.0f;

		///////////////////////////////

	    sensorEEPROM.value.KpAcc = 5.0f;    // proportional gain governs rate of convergence to accelerometer
	    sensorEEPROM.value.KiAcc = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
	    sensorEEPROM.value.KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
	    sensorEEPROM.value.KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases

	    ///////////////////////////////

	    sensorEEPROM.value.compFilterA =  2.000f;
	    sensorEEPROM.value.compFilterB =  1.000f;

	    ///////////////////////////////////

	    sensorEEPROM.value.dlpfSetting = BITS_DLPF_CFG_98HZ;

	    ///////////////////////////////////

	    sensorEEPROM.value.CRCFlags = 0;

	    ///////////////////////////////////

	    writeSensorEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
// Read System EEPROM
///////////////////////////////////////////////////////////////////////////////

void readSystemEEPROM(void)
{
	uint32_t index;

	///////////////////////////////////

	setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

	systemEEPROMaddr.value = SYSTEM_EEPROM_ADDR;

	///////////////////////////////////

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[0]);

	for (index = 0; index < NUMBER_OF_SYSTEM_BYTES; index++)
		systemEEPROM.bytes[index] = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32B((uint32_t*)(&systemEEPROM),       // CRC32B[systemEEPROM CRC32B[systemEEPROM]]
                               (uint32_t*)(&systemEEPROM + 1)))
    {
        evrPush(EVR_SystemCRCFail,0);
        systemEEPROM.value.CRCFlags |= CRC_HistoryBad;
    }
    else if ( systemEEPROM.value.CRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadSystemHistory,0);
    }

    ///////////////////////////////////

    if (systemEEPROM.value.yawDirection >= 0)
	    systemEEPROM.value.yawDirection =  1.0f;
	else
        systemEEPROM.value.yawDirection = -1.0f;
}

///////////////////////////////////////////////////////////////////////////////
// Write System EEPROM
///////////////////////////////////////////////////////////////////////////////

void writeSystemEEPROM(void)

{
    uint16_t byteCount = 0;
    uint32_t byteIndex = 0;
    uint8_t  pageIndex;

    systemEEPROM_u *src = &systemEEPROM;

    systemEEPROMaddr.value = SYSTEM_EEPROM_ADDR;

    // there's no reason to write these values to EEPROM, they'll just be noise
    zeroPIDintegralError();
    zeroPIDstates();

    if (src->value.CRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadSystemHistory,0);

    src->value.CRCAtEnd[0] = crc32B( (uint32_t*)&src[0], src->value.CRCAtEnd);

    ///////////////////////////////////

    setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

    ///////////////////////////////////

    cliPortPrint("Erasing EEPROM Sector\n");

    // Sector Erase

    writeEnable();

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, SECTOR_ERASE_64KB);

	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[0]);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	eepromBusy();

    ///////////////////////////////////

    cliPortPrint("Writing System EEPROM\n");

	// Program Page(s)

	for (pageIndex = 0; pageIndex < NUMBER_OF_SYSTEM_PAGES; pageIndex++)
    {
        cliPortPrintF("  Page: %d\n", pageIndex);

        writeEnable();

        ENABLE_EEPROM;

        spiTransfer(EEPROM_SPI, PAGE_PROGRAM_256_BYTES);

		spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[2]);
		spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[1]);
		spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[0]);

	    for (byteCount = 0; byteCount < 256; byteCount++)
	    {
			spiTransfer(EEPROM_SPI, systemEEPROM.bytes[byteIndex]);

			byteIndex++;

			if (byteIndex >= NUMBER_OF_SYSTEM_BYTES)
			    break;
		}

		DISABLE_EEPROM;

		delayMicroseconds(2);

		eepromBusy();

		systemEEPROMaddr.value += 0x0100;
    }

	//readSystemEEPROM();
}

///////////////////////////////////////////////////////////////////////////////
// Check System EEPROM
///////////////////////////////////////////////////////////////////////////////

void checkSystemEEPROM(bool eepromReset)
{
    uint8_t version;

    systemEEPROMaddr.value = SYSTEM_EEPROM_ADDR;

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[2]);
	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[1]);
	spiTransfer(EEPROM_SPI, systemEEPROMaddr.bytes[0]);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != systemVersion)
    {
		// Default settings
        systemEEPROM.value.version = systemVersion;

	    ///////////////////////////////////

	    systemEEPROM.value.rollAndPitchRateScaling = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

        systemEEPROM.value.yawRateScaling          = 100.0 / 180000.0 * PI;  // Stick to rate scaling for 100 DPS

        systemEEPROM.value.attitudeScaling         = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

        systemEEPROM.value.nDotEdotScaling         = 0.009f;                 // Stick to nDot/eDot scaling (9 mps)/(1000 RX PWM Steps) = 0.009

        systemEEPROM.value.hDotScaling             = 0.003f;                 // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

        ///////////////////////////////////

	    systemEEPROM.value.receiverType  = SPEKTRUM;

        systemEEPROM.value.slaveSpektrum = false;

	    parseRcChannels("TAER2134");

	    systemEEPROM.value.escPwmRate   = 450;
        systemEEPROM.value.servoPwmRate = 50;

        ///////////////////////////////////

        systemEEPROM.value.mixerConfiguration = MIXERTYPE_TRI;
        systemEEPROM.value.yawDirection = 1.0f;

        systemEEPROM.value.triYawServoPwmRate             = 50;
        systemEEPROM.value.triYawServoMin                 = 2000.0f;
        systemEEPROM.value.triYawServoMid                 = 3000.0f;
        systemEEPROM.value.triYawServoMax                 = 4000.0f;
        systemEEPROM.value.triCopterYawCmd500HzLowPassTau = 0.05f;

        // Free Mix Defaults to Quad X
		systemEEPROM.value.freeMixMotors        = 4;

		systemEEPROM.value.freeMix[0][ROLL ]    =  1.0f;
		systemEEPROM.value.freeMix[0][PITCH]    = -1.0f;
		systemEEPROM.value.freeMix[0][YAW  ]    = -1.0f;

		systemEEPROM.value.freeMix[1][ROLL ]    = -1.0f;
		systemEEPROM.value.freeMix[1][PITCH]    = -1.0f;
		systemEEPROM.value.freeMix[1][YAW  ]    =  1.0f;

		systemEEPROM.value.freeMix[2][ROLL ]    = -1.0f;
		systemEEPROM.value.freeMix[2][PITCH]    =  1.0f;
		systemEEPROM.value.freeMix[2][YAW  ]    = -1.0f;

		systemEEPROM.value.freeMix[3][ROLL ]    =  1.0f;
		systemEEPROM.value.freeMix[3][PITCH]    =  1.0f;
		systemEEPROM.value.freeMix[3][YAW  ]    =  1.0f;

		systemEEPROM.value.freeMix[4][ROLL ]    =  0.0f;
		systemEEPROM.value.freeMix[4][PITCH]    =  0.0f;
		systemEEPROM.value.freeMix[4][YAW  ]    =  0.0f;

		systemEEPROM.value.freeMix[5][ROLL ]    =  0.0f;
		systemEEPROM.value.freeMix[5][PITCH]    =  0.0f;
        systemEEPROM.value.freeMix[5][YAW  ]    =  0.0f;

        ///////////////////////////////////

        systemEEPROM.value.midCommand   = 3000.0f;
        systemEEPROM.value.minCheck     = (float)(MINCOMMAND + 200);
        systemEEPROM.value.maxCheck     = (float)(MAXCOMMAND - 200);
        systemEEPROM.value.minThrottle  = (float)(MINCOMMAND + 200);
        systemEEPROM.value.maxThrottle  = (float)(MAXCOMMAND);

        ///////////////////////////////////

        systemEEPROM.value.PID[ROLL_RATE_PID].B               =   1.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].P               = 250.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].I               = 100.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].D               =   0.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].iTerm           =   0.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].windupGuard     = 100.0f;  // PWMs
        systemEEPROM.value.PID[ROLL_RATE_PID].lastDcalcValue  =   0.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].lastDterm       =   0.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].lastLastDterm   =   0.0f;
        systemEEPROM.value.PID[ROLL_RATE_PID].dErrorCalc      =   D_ERROR;
        systemEEPROM.value.PID[ROLL_RATE_PID].type            =   OTHER;

        systemEEPROM.value.PID[PITCH_RATE_PID].B              =   1.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].P              = 250.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].I              = 100.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].D              =   0.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].iTerm          =   0.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].windupGuard    = 100.0f;  // PWMs
        systemEEPROM.value.PID[PITCH_RATE_PID].lastDcalcValue =   0.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].lastDterm      =   0.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].lastLastDterm  =   0.0f;
        systemEEPROM.value.PID[PITCH_RATE_PID].dErrorCalc     =   D_ERROR;
        systemEEPROM.value.PID[PITCH_RATE_PID].type           =   OTHER;

        systemEEPROM.value.PID[YAW_RATE_PID].B                =   1.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].P                = 350.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].I                = 100.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].D                =   0.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].iTerm            =   0.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].windupGuard      = 100.0f;  // PWMs
        systemEEPROM.value.PID[YAW_RATE_PID].lastDcalcValue   =   0.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].lastDterm        =   0.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].lastLastDterm    =   0.0f;
        systemEEPROM.value.PID[YAW_RATE_PID].dErrorCalc       =   D_ERROR;
        systemEEPROM.value.PID[YAW_RATE_PID].type             =   OTHER;

        systemEEPROM.value.PID[ROLL_ATT_PID].B                =   1.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].P                =   2.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].I                =   0.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].D                =   0.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].iTerm            =   0.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].windupGuard      =   0.5f;  // radians/sec
        systemEEPROM.value.PID[ROLL_ATT_PID].lastDcalcValue   =   0.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].lastDterm        =   0.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].lastLastDterm    =   0.0f;
        systemEEPROM.value.PID[ROLL_ATT_PID].dErrorCalc       =   D_ERROR;
        systemEEPROM.value.PID[ROLL_ATT_PID].type             =   ANGULAR;

        systemEEPROM.value.PID[PITCH_ATT_PID].B               =   1.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].P               =   2.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].I               =   0.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].D               =   0.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].iTerm           =   0.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].windupGuard     =   0.5f;  // radians/sec
        systemEEPROM.value.PID[PITCH_ATT_PID].lastDcalcValue  =   0.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].lastDterm       =   0.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].lastLastDterm   =   0.0f;
        systemEEPROM.value.PID[PITCH_ATT_PID].dErrorCalc      =   D_ERROR;
        systemEEPROM.value.PID[PITCH_ATT_PID].type            =   ANGULAR;

        systemEEPROM.value.PID[HEADING_PID].B                 =   1.0f;
        systemEEPROM.value.PID[HEADING_PID].P                 =   3.0f;
        systemEEPROM.value.PID[HEADING_PID].I                 =   0.0f;
        systemEEPROM.value.PID[HEADING_PID].D                 =   0.0f;
        systemEEPROM.value.PID[HEADING_PID].iTerm             =   0.0f;
        systemEEPROM.value.PID[HEADING_PID].windupGuard       =   0.5f;  // radians/sec
        systemEEPROM.value.PID[HEADING_PID].lastDcalcValue    =   0.0f;
        systemEEPROM.value.PID[HEADING_PID].lastDterm         =   0.0f;
        systemEEPROM.value.PID[HEADING_PID].lastLastDterm     =   0.0f;
        systemEEPROM.value.PID[HEADING_PID].dErrorCalc        =   D_ERROR;
        systemEEPROM.value.PID[HEADING_PID].type              =   ANGULAR;

        systemEEPROM.value.PID[NDOT_PID].B                    =   1.0f;
        systemEEPROM.value.PID[NDOT_PID].P                    =   3.0f;
        systemEEPROM.value.PID[NDOT_PID].I                    =   0.0f;
        systemEEPROM.value.PID[NDOT_PID].D                    =   0.0f;
        systemEEPROM.value.PID[NDOT_PID].iTerm                =   0.0f;
        systemEEPROM.value.PID[NDOT_PID].windupGuard          =   0.5f;
        systemEEPROM.value.PID[NDOT_PID].lastDcalcValue       =   0.0f;
        systemEEPROM.value.PID[NDOT_PID].lastDterm            =   0.0f;
        systemEEPROM.value.PID[NDOT_PID].lastLastDterm        =   0.0f;
        systemEEPROM.value.PID[NDOT_PID].dErrorCalc           =   D_ERROR;
        systemEEPROM.value.PID[NDOT_PID].type                 =   OTHER;

        systemEEPROM.value.PID[EDOT_PID].B                    =   1.0f;
        systemEEPROM.value.PID[EDOT_PID].P                    =   3.0f;
        systemEEPROM.value.PID[EDOT_PID].I                    =   0.0f;
        systemEEPROM.value.PID[EDOT_PID].D                    =   0.0f;
        systemEEPROM.value.PID[EDOT_PID].iTerm                =   0.0f;
        systemEEPROM.value.PID[EDOT_PID].windupGuard          =   0.5f;
        systemEEPROM.value.PID[EDOT_PID].lastDcalcValue       =   0.0f;
        systemEEPROM.value.PID[EDOT_PID].lastDterm            =   0.0f;
        systemEEPROM.value.PID[EDOT_PID].lastLastDterm        =   0.0f;
        systemEEPROM.value.PID[EDOT_PID].dErrorCalc           =   D_ERROR;
        systemEEPROM.value.PID[EDOT_PID].type                 =   OTHER;

        systemEEPROM.value.PID[HDOT_PID].B                    =   1.0f;
        systemEEPROM.value.PID[HDOT_PID].P                    =   2.0f;
        systemEEPROM.value.PID[HDOT_PID].I                    =   0.0f;
        systemEEPROM.value.PID[HDOT_PID].D                    =   0.0f;
        systemEEPROM.value.PID[HDOT_PID].iTerm                =   0.0f;
        systemEEPROM.value.PID[HDOT_PID].windupGuard          =   5.0f;
        systemEEPROM.value.PID[HDOT_PID].lastDcalcValue       =   0.0f;
        systemEEPROM.value.PID[HDOT_PID].lastDterm            =   0.0f;
        systemEEPROM.value.PID[HDOT_PID].lastLastDterm        =   0.0f;
        systemEEPROM.value.PID[HDOT_PID].dErrorCalc           =   D_ERROR;
        systemEEPROM.value.PID[HDOT_PID].type                 =   OTHER;

        systemEEPROM.value.PID[N_PID].B                       =   1.0f;
        systemEEPROM.value.PID[N_PID].P                       =   3.0f;
        systemEEPROM.value.PID[N_PID].I                       =   0.0f;
        systemEEPROM.value.PID[N_PID].D                       =   0.0f;
        systemEEPROM.value.PID[N_PID].iTerm                   =   0.0f;
        systemEEPROM.value.PID[N_PID].windupGuard             =   0.5f;
        systemEEPROM.value.PID[N_PID].lastDcalcValue          =   0.0f;
        systemEEPROM.value.PID[N_PID].lastDterm               =   0.0f;
        systemEEPROM.value.PID[N_PID].lastLastDterm           =   0.0f;
        systemEEPROM.value.PID[N_PID].dErrorCalc              =   D_ERROR;
        systemEEPROM.value.PID[N_PID].type                    =   OTHER;

        systemEEPROM.value.PID[E_PID].B                       =   1.0f;
        systemEEPROM.value.PID[E_PID].P                       =   3.0f;
        systemEEPROM.value.PID[E_PID].I                       =   0.0f;
        systemEEPROM.value.PID[E_PID].D                       =   0.0f;
        systemEEPROM.value.PID[E_PID].iTerm                   =   0.0f;
        systemEEPROM.value.PID[E_PID].windupGuard             =   0.5f;
        systemEEPROM.value.PID[E_PID].lastDcalcValue          =   0.0f;
        systemEEPROM.value.PID[E_PID].lastDterm               =   0.0f;
        systemEEPROM.value.PID[E_PID].lastLastDterm           =   0.0f;
        systemEEPROM.value.PID[E_PID].dErrorCalc              =   D_ERROR;
        systemEEPROM.value.PID[E_PID].type                    =   OTHER;

        systemEEPROM.value.PID[H_PID].B                       =   1.0f;
        systemEEPROM.value.PID[H_PID].P                       =   2.0f;
        systemEEPROM.value.PID[H_PID].I                       =   0.0f;
        systemEEPROM.value.PID[H_PID].D                       =   0.0f;
        systemEEPROM.value.PID[H_PID].iTerm                   =   0.0f;
        systemEEPROM.value.PID[H_PID].windupGuard             =   5.0f;
        systemEEPROM.value.PID[H_PID].lastDcalcValue          =   0.0f;
        systemEEPROM.value.PID[H_PID].lastDterm               =   0.0f;
        systemEEPROM.value.PID[H_PID].lastLastDterm           =   0.0f;
        systemEEPROM.value.PID[H_PID].dErrorCalc              =   D_ERROR;
        systemEEPROM.value.PID[H_PID].type                    =   OTHER;

        ///////////////////////////////////

        systemEEPROM.value.batteryCells             = 3;
		systemEEPROM.value.voltageMonitorScale      = 11.0f / 1.0f;
		systemEEPROM.value.voltageMonitorBias       = 0.0f;

		systemEEPROM.value.batteryLow               = 3.30f;
        systemEEPROM.value.batteryVeryLow           = 3.20f;
        systemEEPROM.value.batteryMaxLow            = 3.10f;

        ///////////////////////////////////

        systemEEPROM.value.armCount                 =  50;
		systemEEPROM.value.disarmCount              =  0;

		///////////////////////////////////

		systemEEPROM.value.activeTelemetry          =  0;
		systemEEPROM.value.mavlinkEnabled           =  false;

		///////////////////////////////////

		systemEEPROM.value.verticalVelocityHoldOnly = true;

		///////////////////////////////////

		systemEEPROM.value.CRCFlags = 0;

		///////////////////////////////////

	    writeSystemEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
