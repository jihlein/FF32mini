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
// EEPROM CLI
///////////////////////////////////////////////////////////////////////////////

#define LINE_LENGTH 32

#define TIMEOUT 100     // mSec

///////////////////////////////////////

int min(int a, int b)
{
    return a < b ? a : b;
}

///////////////////////////////////////

int8_t parse_hex(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 0x0A;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 0x0A;
    return -1;
}

///////////////////////////////////////

void cliPrintSensorEEPROM(void)
{
    uint32_t old_crc = sensorEEPROM.value.CRCAtEnd[0];

    uint8_t *by = (uint8_t*)&sensorEEPROM;

    int i, j;

    sensorEEPROM.value.CRCAtEnd[0] = crc32B((uint32_t*)(&sensorEEPROM),                  // CRC32B[sensorEEPROM]
                                            (uint32_t*)(&sensorEEPROM.value.CRCAtEnd));

    if (sensorEEPROM.value.CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)NUMBER_OF_SENSOR_BYTES / LINE_LENGTH); i++)
    {
        for (j = 0; j < min(LINE_LENGTH, NUMBER_OF_SENSOR_BYTES - LINE_LENGTH * i); j++)
            cliPortPrintF("%02X", by[i * LINE_LENGTH + j]);

        cliPortPrint("\n");
    }

    sensorEEPROM.value.CRCAtEnd[0] = old_crc;
}

///////////////////////////////////////

void cliPrintSystemEEPROM(void)
{
    uint32_t old_crc = systemEEPROM.value.CRCAtEnd[0];

    uint8_t *by = (uint8_t*)&systemEEPROM;

    int i, j;

     systemEEPROM.value.CRCAtEnd[0] = crc32B((uint32_t*)(&systemEEPROM),                  // CRC32B[systemEEPROM]
                                             (uint32_t*)(&systemEEPROM.value.CRCAtEnd));

    if (systemEEPROM.value.CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)NUMBER_OF_SYSTEM_BYTES / LINE_LENGTH); i++)
    {
        for (j = 0; j < min(LINE_LENGTH, NUMBER_OF_SYSTEM_BYTES - LINE_LENGTH * i); j++)
            cliPortPrintF("%02X", by[i * LINE_LENGTH + j]);

        cliPortPrint("\n");
    }

    systemEEPROM.value.CRCAtEnd[0] = old_crc;
}

///////////////////////////////////////

void eepromCLI()
{
	char c;

	sensorEEPROM_u sensorRam;

	systemEEPROM_u systemRam;

	uint8_t  eepromQuery = 'x';

	uint8_t  *p;

	uint8_t  *end;

	uint8_t  secondNibble;

	uint8_t  validQuery  = false;

	uint16_t i;

	uint32_t c1, c2;

    uint32_t size;

	uint32_t time;

	uint32_t charsEncountered;

	///////////////////////////////////////////////////////////////////////////////

    cliBusy = true;

    cliPortPrint("\nEntering EEPROM CLI....\n\n");

    while(true)
    {
        cliPortPrint("EEPROM CLI -> ");

        while ((cliPortAvailable() == false) && (validQuery == false));

        if (validQuery == false)
            eepromQuery = cliPortRead();

        cliPortPrint("\n");

        switch(eepromQuery)
        {
            ///////////////////////////

            case 'a': // config struct data
                c1 = sensorEEPROM.value.CRCAtEnd[0];

                c2 = crc32B((uint32_t*)(&sensorEEPROM),                  // CRC32B[sensorEEPROM]
                            (uint32_t*)(&sensorEEPROM.value.CRCAtEnd));

                cliPortPrintF("Sensor EEPROM structure information:\n");
                cliPortPrintF("Version          : %d\n", sensorEEPROM.value.version);
                cliPortPrintF("Size             : %d\n", NUMBER_OF_SENSOR_BYTES);
                cliPortPrintF("CRC on last read : %08X\n", c1);
                cliPortPrintF("Current CRC      : %08X\n", c2);

                if ( c1 != c2 )
                    cliPortPrintF("  CRCs differ. Current Sensor Config has not yet been saved.\n");

                cliPortPrintF("CRC Flags :\n");
                cliPortPrintF("  History Bad    : %s\n", sensorEEPROM.value.CRCFlags & CRC_HistoryBad ? "true" : "false" );

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // Write out to Console in Hex.  (RAM -> console)
                // we assume the flyer is not in the air, so that this is ok;

                cliPortPrintF("\n");

                cliPrintSensorEEPROM();

                cliPortPrintF("\n");

                if (crcCheckVal != crc32B((uint32_t*)(&sensorEEPROM),       // CRC32B[sensorEEPROM CRC32B[sensorEEPROM]]
                                          (uint32_t*)(&sensorEEPROM + 1)))
                {
                    cliPortPrint("NOTE: in-memory sensor config CRC invalid; there have probably been\n");
                    cliPortPrint("      changes to sensor config since the last write to flash/eeprom.\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'c': // Read Sensor Config -> RAM
                cliPortPrint("Re-reading Sensor EEPROM.\n");
                readSensorEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'd': // Read Console -> Sensor RAM
                charsEncountered = 0;

                secondNibble = 0;

                size = NUMBER_OF_SENSOR_BYTES;

                time = millis();

                p = (uint8_t*)&sensorRam;

                end = (uint8_t*)(&sensorRam + 1);

                ///////////////////////

                cliPortPrintF("Ready to read in sensor config. Expecting %d (0x%03X) bytes as %d\n", size, size, size * 2);

                cliPortPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\n");

                cliPortPrintF("Times out if no character is received for %dms\n", TIMEOUT);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliPortAvailable() && millis() - time < TIMEOUT) {}
                    time = millis();

                    c = cliPortAvailable() ? cliPortRead() : '\0';

                    int8_t hex = parse_hex(c);

                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        charsEncountered++;

                    if (ignore)
                        continue;

                    if (hex == -1)
                        break;

                    *p |= secondNibble ? hex : hex << 4;

                    p += secondNibble;

                    secondNibble ^= 1;
                }

                if (c == 0)
                {
                    cliPortPrintF("Did not receive enough hex chars! (got %d, expected %d)\n",
                        (p - (uint8_t*)&sensorRam) * 2 + secondNibble, size * 2);
                }
                else if (p < end || secondNibble)
                {
                    cliPortPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        charsEncountered, c, c);
                }
                else if (crcCheckVal != crc32B((uint32_t*)(&sensorEEPROM),       // CRC32B[sensorEEPROM CRC32B[sensorEEPROM]]
                                               (uint32_t*)(&sensorEEPROM + 1)))
                {
                    cliPortPrintF("CRC mismatch! Not writing to in-memory config.\n");
                    cliPortPrintF("Here's what was received:\n\n");
                    cliPrintSensorEEPROM();
                }
                else
                {
                    // check to see if the newly received sytem config
                    // actually differs from what's in-memory

                    for (i = 0; i < size; i++)
                        if (((uint8_t*)&sensorRam)[i] != ((uint8_t*)&sensorEEPROM)[i])
                            break;

                    if (i == size)
                    {
                        cliPortPrintF("NOTE: Uploaded Sensor Config was Identical to RAM Config.\n");
                    }
                    else
                    {
                        sensorEEPROM = sensorRam;
                        cliPortPrintF("Sensor RAM Config updated!\n");
                        cliPortPrintF("NOTE: Sensor Config not written to EEPROM; use 'w' to do so.\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something

                time = millis();

                while (millis() - time < TIMEOUT)
                    if (cliPortAvailable())
                        cliPortRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'h': // Clear Bad Sensor History Flag
                cliPortPrintF("Clearing Bad Sensor History Flag.\n");
                sensorEEPROM.value.CRCFlags &= ~CRC_HistoryBad;
                validQuery = false;
                break;

            ///////////////////////////

            case 'v': // Reset Sensor EEPROM Parameters
                cliPortPrint( "\nSensor EEPROM Parameters Reset....(not rebooting)\n" );
                checkSensorEEPROM(true);
                validQuery = false;
                break;

            ///////////////////////////

            case 'w': // Write to Sensor EEPROM
                cliPortPrint("\nWriting Sensor EEPROM Parameters....\n");
                writeSensorEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'x': // exit EEPROM CLI
                cliPortPrint("\nExiting EEPROM CLI....\n\n");
                cliBusy = false;
                return;
                break;

            ///////////////////////////

            case 'A': // config struct data
                c1 = systemEEPROM.value.CRCAtEnd[0];

                zeroPIDintegralError();
                zeroPIDstates();

                c2 = crc32B((uint32_t*)(&systemEEPROM),                  // CRC32B[systemEEPROM]
                            (uint32_t*)(&systemEEPROM.value.CRCAtEnd));

                cliPortPrintF("System EEPROM structure information:\n");
                cliPortPrintF("Version          : %d\n", systemEEPROM.value.version);
                cliPortPrintF("Size             : %d\n", NUMBER_OF_SYSTEM_BYTES);
                cliPortPrintF("CRC on last read : %08X\n", c1);
                cliPortPrintF("Current CRC      : %08X\n", c2);

                if ( c1 != c2 )
                    cliPortPrintF("  CRCs differ. Current SystemConfig has not yet been saved.\n");

                cliPortPrintF("CRC Flags :\n");
                cliPortPrintF("  History Bad    : %s\n", systemEEPROM.value.CRCFlags & CRC_HistoryBad ? "true" : "false" );

                validQuery = false;
                break;

            ///////////////////////////

            case 'B': // Write out to Console in Hex.  (RAM -> console)
                // we assume the flyer is not in the air, so that this is ok;

                // these change randomly when not in flight and can mistakenly
                // make one think that the in-memory eeprom struct has changed
                zeroPIDintegralError();
                zeroPIDstates();

                cliPortPrintF("\n");

                cliPrintSystemEEPROM();

                cliPortPrintF("\n");

                if (crcCheckVal != crc32B((uint32_t*)(&systemEEPROM),       // CRC32B[systemEEPROM CRC32B[systemEEPROM]]
                                          (uint32_t*)(&systemEEPROM + 1)))
                {
                    cliPortPrint("NOTE: in-memory system config CRC invalid; there have probably been\n");
                    cliPortPrint("      changes to system config since the last write to flash/eeprom.\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'C': // Read System Config -> RAM
                cliPortPrint("Re-reading System EEPROM.\n");
                readSystemEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case 'D': // Read Console -> System RAM
            	charsEncountered = 0;

            	secondNibble = 0;

            	size = NUMBER_OF_SYSTEM_BYTES;

                time = millis();

                p = (uint8_t*)&systemRam;

                end = (uint8_t*)(&systemRam + 1);

                ///////////////////////

                cliPortPrintF("Ready to read in system config. Expecting %d (0x%03X) bytes as %d\n", size, size, size * 2);

                cliPortPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\n");

                cliPortPrintF("Times out if no character is received for %dms\n", TIMEOUT);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliPortAvailable() && millis() - time < TIMEOUT) {}
                    time = millis();

                    c = cliPortAvailable() ? cliPortRead() : '\0';

                    int8_t hex = parse_hex(c);

                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        charsEncountered++;

                    if (ignore)
                        continue;

                    if (hex == -1)
                        break;

                    *p |= secondNibble ? hex : hex << 4;

                    p += secondNibble;

                    secondNibble ^= 1;
                }

                if (c == 0)
                {
                    cliPortPrintF("Did not receive enough hex chars! (got %d, expected %d)\n",
                        (p - (uint8_t*)&systemRam) * 2 + secondNibble, size * 2);
                }
                else if (p < end || secondNibble)
                {
                    cliPortPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        charsEncountered, c, c);
                }
                else if (crcCheckVal != crc32B((uint32_t*)(&systemEEPROM),       // CRC32B[systemEEPROM CRC32B[systemEEPROM]]
                                               (uint32_t*)(&systemEEPROM + 1)))
                {
                    cliPortPrintF("CRC mismatch! Not writing to in-memory config.\n");
                    cliPortPrintF("Here's what was received:\n\n");
                    cliPrintSystemEEPROM();
                }
                else
                {
                    // check to see if the newly received sytem config
                    // actually differs from what's in-memory
                    zeroPIDintegralError();
                    zeroPIDstates();

                    for (i = 0; i < size; i++)
                        if (((uint8_t*)&systemRam)[i] != ((uint8_t*)&systemEEPROM)[i])
                            break;

                    if (i == size)
                    {
                        cliPortPrintF("NOTE: Uploaded System Config was Identical to RAM Config.\n");
                    }
                    else
                    {
                        systemEEPROM = systemRam;
                        cliPortPrintF("System RAM Config updated!\n");
                        cliPortPrintF("NOTE: System Config not written to EEPROM; use 'W' to do so.\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something

                time = millis();

                while (millis() - time < TIMEOUT)
                    if (cliPortAvailable())
                        cliPortRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'H': // Clear Bad System History Flag
                cliPortPrintF("Clearing Bad System History Flag.\n");
                systemEEPROM.value.CRCFlags &= ~CRC_HistoryBad;
                validQuery = false;
                break;

            ///////////////////////////

            case 'V': // Reset System EEPROM Parameters
                cliPortPrint( "\nSystem EEPROM Parameters Reset....(not rebooting)\n" );
                checkSystemEEPROM(true);

                validQuery = false;
                break;

            ///////////////////////////

            case 'W': // Write out to System EEPROM
                cliPortPrint("\nWriting System EEPROM Parameters....\n");
                writeSystemEEPROM();

                validQuery = false;
                break;

            ///////////////////////////

            case '?':
            //                0         1         2         3         4         5         6         7
            //                01234567890123456789012345678901234567890123456789012345678901234567890123456789
                cliPortPrintF("\n");
                cliPortPrintF("'a' Display Sensor Config Information     'A' Display System Config Information\n");
                cliPortPrintF("'b' Write Sensor Config -> Console        'B' Write System Config - > Console\n");
                cliPortPrintF("'c' Read Sensor Config -> RAM             'C' Read System Config -> RAM\n");
                cliPortPrintF("'d' Read Console -> Sensor RAM            'D' Read Console -> System RAM\n");
                cliPortPrintF("'h' Clear System CRC Bad History flag     'H' Clear System CRC Bad History flag\n");
                cliPortPrintF("'v' Reset Sensor Config to Default        'V' Reset System Config to Default\n");
                cliPortPrintF("'w' Write Sensor Config -> EEPROM         'W' Write System Config -> EEPROM\n");
                cliPortPrintF("'x' Exit EEPROM CLI                       '?' Command Summary\n");
                cliPortPrintF("\n");
                break;

            ///////////////////////////
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
