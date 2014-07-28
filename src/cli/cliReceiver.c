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
// Receiver CLI
///////////////////////////////////////////////////////////////////////////////

void receiverCLI()
{
    char     rcOrderString[9];
    float    tempFloat;
    uint8_t  index;
    uint8_t  receiverQuery = 'x';
    uint8_t  validQuery    = false;

    NVIC_InitTypeDef  NVIC_InitStructure;

    cliBusy = true;

    cliPortPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        cliPortPrint("Receiver CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = cliPortRead();

		cliPortPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                cliPortPrint("\nReceiver Type:                  ");
                switch(systemConfig.receiverType)
                {
                    case PPM:
                        cliPortPrint("PPM\n");
                        break;
                    case SPEKTRUM:
                        cliPortPrint("Spektrum\n");
                        break;
		        }

                cliPortPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < 8; index++)
                    rcOrderString[systemConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                cliPortPrint(rcOrderString);  cliPortPrint("\n");

                cliPortPrintF("Secondary Spektrum:             ");

                if ((systemConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on Naze32 Pro
                    cliPortPrintF("Installed\n");
                else
                    cliPortPrintF("Uninstalled\n");

                cliPortPrintF("Mid Command:                    %4ld\n",   (uint16_t)systemConfig.midCommand);
				cliPortPrintF("Min Check:                      %4ld\n",   (uint16_t)systemConfig.minCheck);
				cliPortPrintF("Max Check:                      %4ld\n",   (uint16_t)systemConfig.maxCheck);
				cliPortPrintF("Min Throttle:                   %4ld\n",   (uint16_t)systemConfig.minThrottle);
				cliPortPrintF("Max Thottle:                    %4ld\n\n", (uint16_t)systemConfig.maxThrottle);

				tempFloat = systemConfig.rollAndPitchRateScaling * 180000.0 / PI;
				cliPortPrintF("Max Roll and Pitch Rate Cmd:    %6.2f DPS\n", tempFloat);

				tempFloat = systemConfig.yawRateScaling * 180000.0 / PI;
				cliPortPrintF("Max Yaw Rate Cmd:               %6.2f DPS\n\n", tempFloat);

				cliPortPrintF("Roll Rate Cmd Tau:              %6.2f\n",   systemConfig.rollRateCmdLowPassTau);
				cliPortPrintF("Pitch Rate Cmd Tau:             %6.2f\n\n", systemConfig.pitchRateCmdLowPassTau);

				tempFloat = systemConfig.attitudeScaling * 180000.0 / PI;
                cliPortPrintF("Max Attitude Cmd:               %6.2f Degrees\n\n", tempFloat);

				cliPortPrintF("Roll Attitude Cmd Tau:          %6.2f\n",   systemConfig.rollAttCmdLowPassTau);
				cliPortPrintF("Pitch Attitude Cmd Tau:         %6.2f\n\n", systemConfig.pitchAttCmdLowPassTau);

				cliPortPrintF("Arm Delay Count:                %3d Frames\n",   systemConfig.armCount);
				cliPortPrintF("Disarm Delay Count:             %3d Frames\n\n", systemConfig.disarmCount);

				validQuery = false;
				break;

            ///////////////////////////

			case 'x':
			    cliPortPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Toggle PPM/Spektrum Satellite Receiver
            	if (systemConfig.receiverType == PPM)
                {
                    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
                    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
                    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
                    NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;

                    NVIC_Init(&NVIC_InitStructure);

                	TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
                	systemConfig.receiverType = SPEKTRUM;
                    spektrumInit();
                }
                else
                {
                	NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_TRG_COM_TIM17_IRQn;
                    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
                    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
                    NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;

                    NVIC_Init(&NVIC_InitStructure);

                    TIM_ITConfig(TIM17, TIM_IT_Update, DISABLE);
                  	systemConfig.receiverType = PPM;
                    ppmRxInit();
                }

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Read RC Control Order
                readStringCLI( rcOrderString, 8 );
                parseRcChannels( rcOrderString );

          	    receiverQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Toggle Slave Spektrum State
                // HJI Inhibit Slave Spektrum on Naze32 Pro if (systemConfig.slaveSpektrum == true)
                systemConfig.slaveSpektrum = false;
                // HJI Inhibit Slave Spektrum on Naze32 Pro else
                // HJI Inhibit Slave Spektrum on Naze32 Pro	    systemConfig.slaveSpektrum = true;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read RC Control Points
                systemConfig.midCommand   = readFloatCLI();
    	        systemConfig.minCheck     = readFloatCLI();
    		    systemConfig.maxCheck     = readFloatCLI();
    		    systemConfig.minThrottle  = readFloatCLI();
    		    systemConfig.maxThrottle  = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read Arm/Disarm Counts
                systemConfig.armCount    = (uint8_t)readFloatCLI();
        	    systemConfig.disarmCount = (uint8_t)readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Max Rate Value
                systemConfig.rollAndPitchRateScaling = readFloatCLI() / 180000.0f * PI;
                systemConfig.yawRateScaling          = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read Max Attitude Value
                systemConfig.attitudeScaling = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'H': // Read Rate Cmd Tau Value
                systemConfig.rollRateCmdLowPassTau  = readFloatCLI();
                systemConfig.pitchRateCmdLowPassTau = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'I': // Read Attitude Cmd Tau Value
                systemConfig.rollAttCmdLowPassTau  = readFloatCLI();
                systemConfig.pitchAttCmdLowPassTau = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write System EEPROM Parameters
                cliPortPrint("\nWriting System EEPROM Parameters....\n\n");
                writeSystemEEPROM();

                validQuery = false;
                break;

			///////////////////////////

			case '?':
			   	cliPortPrint("\n");
			   	cliPortPrint("'a' Receiver Configuration Data            'A' Toggle PPM/Spektrum Receiver\n");
   		        cliPortPrint("                                           'B' Set RC Control Order                 BTAER1234\n");
			   	cliPortPrint("                                           'C' Toggle Slave Spektrum State\n");
			   	cliPortPrint("                                           'D' Set RC Control Points                DmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	cliPortPrint("                                           'E' Set Arm/Disarm Counts                EarmCount;disarmCount\n");
			   	cliPortPrint("                                           'F' Set Maximum Rate Commands            FRP;Y RP = Roll/Pitch, Y = Yaw\n");
			   	cliPortPrint("                                           'G' Set Maximum Attitude Command\n");
			   	cliPortPrint("                                           'H' Set Roll/Pitch Rate Command Filters  HROLL;PITCH\n");
			   	cliPortPrint("                                           'I' Set Roll/Pitch Att  Command Filters  IROLL;PITCH\n");
			   	cliPortPrint("                                           'W' Write System EEPROM Parameters\n");
			   	cliPortPrint("'x' Exit Receiver CLI                      '?' Command Summary\n");
			   	cliPortPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
