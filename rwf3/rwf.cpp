/*
 * Copyright (C) 2014  RoboPeak
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  RoboPeak Lidar System
 *  Simple Data Grabber Demo App
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 *  An ultra simple app to fetech RPLIDAR data continuously....
 *
 */




#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <signal.h>
#include "mh.h"

using namespace std;

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

#define PI 3.14159265
#endif

//using namespace motor_hat;
using namespace rp::standalone::rplidar;
//using namespace std;
void setupKillHandler(void);



bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        //printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int main(int argc, const char * argv[]) {
	//delay(10000);
	setupKillHandler();
	bool too_close = false;
	bool too_far = false;
	bool rotate = false;
	int bad_counts = 0;
	
	double front_dist = 0, right_dist = 10, front_andgle = 0, right_angle = 0;
	
	int thresh = 1;
	motor_hat::motor_hat mh;
	//mh.set_speed(0, 100, 1);
	//delay(500);
	//mh.set_speed(0, 0, 0);
	wiringPiSetup();
	int x = softPwmCreate(26, 0, 100);
	pinMode(26, x);
	softPwmWrite(26, 100);
	double dist = 0;
	double distX = 0;
	double theta_deg = 0;
	double theta_rad = 0;
	double distY = 0;
	double range = 2000;
	double x_collection [360][2] = {};
	double y_collection [360][2] = {};
	int stop = 1000;
	int counter = 0;
	for(int i = 0; i < 360;i++)
	{
		x_collection[i][1] = 0;
		y_collection[i][1] = 0;
	}
	int array_theta = 0;
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }


    // start scan...
    drv->startScan();

	//double dist = 0;
    // fetech result and print it out...
    
    while (1) 
    {

    			//printf(" %s \n", "&&&&&&&&&&&&&&&&&&&&&&&&&");
        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) 
            {     
		dist = (nodes[pos].distance_q2/40.0f);

		//if(dist !=0)
		//{

			theta_deg = ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
			theta_rad = ((theta_deg*PI)/180.0);
			array_theta = (int)theta_deg;
			printf("%s %f %s %f\n", "dist ", dist, "theta ", theta_deg);
			
			
			distX = std::sin(theta_rad)*(dist);
			distY = std::cos(theta_rad)*(dist);
			
			counter++;
		//}
		
		if( theta_deg > 359 && theta_deg < 1)
		{
			if(dist != 0)
				front_dist = dist;
		}
		
		if( theta_deg > 89 && theta_deg < 91)
		{
			if(dist != 0)
				right_dist = dist;
		}
		
		if(right_dist < 30)
		{
			mh.set_speed(2, 130, 1);
			mh.set_speed(0,130, 1);
			mh.set_speed(1,130,1);
			printf("%f %s \n", right_dist, " ----------------------------------");
		}
		else if(right_dist > 30 && right_dist < 60)
		{
			mh.set_speed(2, 100, 1);
			mh.set_speed(0,160, 1);
			mh.set_speed(1,130,1);
			printf("%f %s \n", right_dist, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
		}
		
			//else if(dist > 60 && dist != 0)
			//{
			//	mh.set_speed(2, 70, -1);
			//	mh.set_speed(0,100, 1);
			//	printf("%f %s \n", dist, "************************************");
			//}
		
		if(counter == stop)
			break;

			
            }

        }
        counter = 0;

    }
    	mh.set_speed(2, 0, 0);
	mh.set_speed(0,0, 0);

    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}

void killHandler(int s){
        printf("Caught signal %d\n",s);
        motor_hat::motor_hat mh;
	mh.set_speed(2, 0,0);
	mh.set_speed(0,0,0);
        exit(1); 

}

void setupKillHandler(){
	struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = killHandler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);
   

}