/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <libudev.h>
#include <locale.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;



bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
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

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

    int check_dev(char * opt_channel_param_first){
//	char * opt_is_channel = NULL; 
//	char * opt_channel = NULL;
//	sl_u32         opt_channel_param_second = 0;
//    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_u32         baudrateArray[1] = {115200};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

//	bool useArgcBaudrate = false;

    IChannel* _channel;
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for(size_t i = 0; i < baudRateArraySize; ++i)
    {
        _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
                connectSuccess = true;
                break;
            }
            else{
                delete drv;
                drv = NULL;
        }
    }}


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
//        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
   /* printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);
*/


    // check health...
//    if (!checkSLAMTECLIDARHealth(drv)) {
//        goto on_finished;
//    }

    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
//    while (1) {
//    while (0) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        int lidar_passed = 0;

        if (SL_IS_OK(op_result)) {
            std::ofstream MyFile("filename.txt");
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                MyFile << nodes[pos].angle_z_q14*90.f / 16384.f << " " << nodes[pos].dist_mm_q2/4.0f << "\n";
            // We've seen lidars fail reading all 0s. Check for that.
                if (nodes[pos].dist_mm_q2/4.0f > 0 && ! lidar_passed){
                    lidar_passed = 1;
                }       
            }
            MyFile.close();
        }
//
//        if (ctrl_c_pressed){ 
//            break;
//        }
    
    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    return lidar_passed;
    // done!
    }

int main(int argc, const char * argv[]) {
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	int dev_index = 1;
    const int expected_dev_number = 3;
	char dev_array[expected_dev_number][20];
    // device filepath
    strncpy(dev_array[0], argv[1], sizeof(dev_array[0]));

	
    int all_lidars_passed = 1;
	for (int i = 0; i < dev_index; i++){
		    printf("%s\n", dev_array[i]);
    		check_dev(dev_array[i]);
	}
    if (! all_lidars_passed) {
        printf("Not all lidars returned good data\n\n");
    }
    if (dev_index == 3 && all_lidars_passed){
        printf("Tests passed\n");
    }
    else{
        printf("Not all tests passed\n");
    }

    exit(1);

	 
//    char * opt_channel_param_first = NULL;
//    char * opt_channel_param_second= NULL;
//	opt_channel_param_first = "/dev/ttyUSB0";

	//opt_channel_param_first = "/dev/bus/usb/001/011";
	//opt_channel_param_second= "/dev/ttyUSB1";


    //check_dev(opt_channel_param_second);
    /*
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
    */
}


