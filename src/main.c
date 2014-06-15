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

uint8_t        execUpCount = 0;

sensors_t      sensors;

heading_t      heading;

gps_t          gps;

homeData_t     homeData;

uint16_t       timerValue;

uint32_t       (*telemPortAvailable)(void);
void           (*telemPortPrint)(char *str);
void           (*telemPortPrintF)(const char * fmt, ...);
uint8_t        (*telemPortRead)(void);

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
	///////////////////////////////////////////////////////////////////////////

	uint32_t currentTime;

	arm_matrix_instance_f32 a;
	arm_matrix_instance_f32 b;
	arm_matrix_instance_f32 x;

    systemReady = false;

    systemInit();

    systemReady = true;

    evrPush(EVR_StartingMain, 0);

    while (1)
    {
    	evrCheck();

    	///////////////////////////////

        if (frame_50Hz)
        {
        	frame_50Hz = false;

        	currentTime      = micros();
			deltaTime50Hz    = currentTime - previous50HzTime;
			previous50HzTime = currentTime;

			processFlightCommands();

            if (newTemperatureReading && newPressureReading)
            {
                d1Value = d1.value;
                d2Value = d2.value;

                calculateTemperature();
                calculatePressureAltitude();

                newTemperatureReading = false;
                newPressureReading    = false;
            }

            sensors.pressureAlt50Hz = firstOrderFilter(sensors.pressureAlt50Hz, &firstOrderFilters[PRESSURE_ALT_LOWPASS]);

			executionTime50Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_10Hz)
        {
        	frame_10Hz = false;

        	currentTime      = micros();
			deltaTime10Hz    = currentTime - previous10HzTime;
			previous10HzTime = currentTime;

			if (newMagData == true)
			{
			    nonRotatedMagData[XAXIS] = (rawMag[XAXIS].value * magScaleFactor[XAXIS]) - sensorConfig.magBias[XAXIS];
			    nonRotatedMagData[YAXIS] = (rawMag[YAXIS].value * magScaleFactor[YAXIS]) - sensorConfig.magBias[YAXIS];
			    nonRotatedMagData[ZAXIS] = (rawMag[ZAXIS].value * magScaleFactor[ZAXIS]) - sensorConfig.magBias[ZAXIS];

			    arm_mat_init_f32(&a, 3, 3, (float *)hmcOrientationMatrix);

			    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedMagData);

			    arm_mat_init_f32(&x, 3, 1,          sensors.mag10Hz);

			    arm_mat_mult_f32(&a, &b, &x);

				newMagData = false;
			    magDataUpdate = true;
            }

        	decodeUbloxMsg();

        	batMonTick();

        	checkUsbActive();

        	cliCom();

            if (systemConfig.mavlinkEnabled == true)
            {
				mavlinkSendAttitude();
				mavlinkSendVfrHud();
			}

        	executionTime10Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_500Hz)
        {
			frame_500Hz = false;

       	    currentTime       = micros();
       	    deltaTime500Hz    = currentTime - previous500HzTime;
       	    previous500HzTime = currentTime;

       	    TIM_Cmd(TIM6, DISABLE);
       	 	timerValue = TIM_GetCounter(TIM6);
       	 	TIM_SetCounter(TIM6, 0);
       	 	TIM_Cmd(TIM6, ENABLE);

       	 	dt500Hz = (float)timerValue * 0.0000005f;  // For integrations in 500 Hz loop

       	    computeMPU6000TCBias();

       	    nonRotatedAccelData[XAXIS] = ((float)accelSummedSamples500Hz[XAXIS] * 0.5f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[YAXIS] = ((float)accelSummedSamples500Hz[YAXIS] * 0.5f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[ZAXIS] = ((float)accelSummedSamples500Hz[ZAXIS] * 0.5f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedAccelData);

		    arm_mat_init_f32(&x, 3, 1,          sensors.accel500Hz);

		    arm_mat_mult_f32(&a, &b, &x);

            nonRotatedGyroData[ROLL ] = ((float)gyroSummedSamples500Hz[ROLL]  * 0.5f - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
            nonRotatedGyroData[PITCH] = ((float)gyroSummedSamples500Hz[PITCH] * 0.5f - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            nonRotatedGyroData[YAW  ] = ((float)gyroSummedSamples500Hz[YAW]   * 0.5f - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedGyroData);

		    arm_mat_init_f32(&x, 3, 1,          sensors.gyro500Hz);

		    arm_mat_mult_f32(&a, &b, &x);

		    MargAHRSupdate( sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
                            sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
                            sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                            sensorConfig.accelCutoff,
                            magDataUpdate,
                            dt500Hz );

            magDataUpdate = false;

            computeAxisCommands(dt500Hz);
            mixTable();
            writeServos();
            writeMotors();

       	    executionTime500Hz = micros() - currentTime;
		}

        ///////////////////////////////

        if (frame_100Hz)
        {
        	frame_100Hz = false;

        	currentTime       = micros();
			deltaTime100Hz    = currentTime - previous100HzTime;
			previous100HzTime = currentTime;

			TIM_Cmd(TIM7, DISABLE);
			timerValue = TIM_GetCounter(TIM7);
			TIM_SetCounter(TIM7, 0);
			TIM_Cmd(TIM7, ENABLE);

			dt100Hz = (float)timerValue * 0.0000005f;  // For integrations in 100 Hz loop

       	    nonRotatedAccelData[XAXIS] = ((float)accelSummedSamples100Hz[XAXIS] * 0.1f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[YAXIS] = ((float)accelSummedSamples100Hz[YAXIS] * 0.1f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
       	    nonRotatedAccelData[ZAXIS] = ((float)accelSummedSamples100Hz[ZAXIS] * 0.1f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedAccelData);

		    arm_mat_init_f32(&x, 3, 1,          sensors.accel100Hz);

		    arm_mat_mult_f32(&a, &b, &x);

			createRotationMatrix();
            bodyAccelToEarthAccel();
            vertCompFilter(dt100Hz);

            if (armed == true)
            {
				if ( systemConfig.activeTelemetry == 1 )
                {
            	    // 500 Hz Accels
            	    telemPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
            	            			                     sensors.accel500Hz[YAXIS],
            	            			                     sensors.accel500Hz[ZAXIS]);
                }

                if ( systemConfig.activeTelemetry == 2 )
                {
            	    // 500 Hz Gyros
            	    telemPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ],
            	            			                     sensors.gyro500Hz[PITCH],
            	            					             sensors.gyro500Hz[YAW  ]);
                }

                if ( systemConfig.activeTelemetry == 4 )
                {
            	    // 500 Hz Attitudes
            	    telemPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ],
            	            			                     sensors.attitude500Hz[PITCH],
            	            			                     sensors.attitude500Hz[YAW  ]);
                }

                if ( systemConfig.activeTelemetry == 8 )
                {
               	    // Vertical Variables
            	    telemPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %4ld\n", earthAxisAccels[ZAXIS],
            	    		                                              sensors.pressureAlt50Hz,
            	    		                                              hDotEstimate,
            	    		                                              hEstimate,
            	    		                                              ms5611Temperature);
                }

                if ( systemConfig.activeTelemetry == 16 )
                {
               	    // Vertical Variables
            	    telemPortPrintF("%9.4f, %9.4f, %9.4f, %4ld, %1d, %9.4f, %9.4f\n", verticalVelocityCmd,
            	    		                                                          hDotEstimate,
            	    		                                                          hEstimate,
            	    		                                                          ms5611Temperature,
            	    		                                                          verticalModeState,
            	    		                                                          throttleCmd,
            	    		                                                          systemConfig.PID[HDOT_PID].iTerm);
                }

            }

            executionTime100Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
        	frame_5Hz = false;

        	currentTime     = micros();
			deltaTime5Hz    = currentTime - previous5HzTime;
			previous5HzTime = currentTime;

			if (gpsValid() == true)
			{

			}

            //if (systemConfig.mavlinkEnabled == true)
            //{
			//	mavlinkSendGpsRaw();
			//}

			if (batMonVeryLowWarning > 0)
			{
				BEEP_TOGGLE;
				batMonVeryLowWarning--;
			}

			if (execUp == true)
			    LED0_TOGGLE;

			executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
        	frame_1Hz = false;

        	currentTime     = micros();
			deltaTime1Hz    = currentTime - previous1HzTime;
			previous1HzTime = currentTime;

			if (execUp == false)
			    execUpCount++;

			if ((execUpCount == 5) && (execUp == false))
			{
			    execUp = true;

			    pwmEscInit();

			    homeData.magHeading = sensors.attitude500Hz[YAW];
			}

			if (batMonLowWarning > 0)
			{
				BEEP_TOGGLE;
				batMonLowWarning--;
			}

            if (systemConfig.mavlinkEnabled == true)
            {
				mavlinkSendHeartbeat();
				mavlinkSendSysStatus();
			}

			executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }

    ///////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
