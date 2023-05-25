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
int gl (void)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	
	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		exit(1);
	}
	
	/* Create a list of the devices in the 'tty' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "tty");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	/* For each item enumerated, print out its information.
	   udev_list_entry_foreach is a macro which expands to
	   a loop. The loop will be executed for each member in
	   devices, setting dev_list_entry to a list entry
	   which contains the device's path in /sys. */
	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;
		
		/* Get the filename of the /sys entry for the device
		   and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);
		printf("Device Node Path: %s\n", udev_device_get_devnode(dev));

		/* usb_device_get_devnode() returns the path to the device node
		   itself in /dev. */

		/* The device pointed to by dev contains information about
		   the hidraw device. In order to get information about the
		   USB device, get the parent device with the
		   subsystem/devtype pair of "usb"/"usb_device". This will
		   be several levels up the tree, but the function will find
		   it.*/
		dev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
		if (!dev) {
			printf("Unable to find parent usb device.");
		}
	
		/* From here, we can call get_sysattr_value() for each file
		   in the device's /sys entry. The strings passed into these
		   functions (idProduct, idVendor, serial, etc.) correspond
		   directly to the files in the directory which represents
		   the USB device. Note that USB strings are Unicode, UCS2
		   encoded, but the strings returned from
		   udev_device_get_sysattr_value() are UTF-8 encoded. */
		/*printf("  VID/PID: %s %s\n",
		        udev_device_get_sysattr_value(dev,"idVendor"),
		        udev_device_get_sysattr_value(dev, "idProduct"));
			*/
		const char * val = udev_device_get_sysattr_value(dev,"idVendor");
		const char * lidar = "10c4";
		if (val != NULL){
			if (strcmp(val, lidar) == 0){
				printf("LIDAR!\n");		
			}
		}
		/*printf("  %s\n  %s\n",
		        udev_device_get_sysattr_value(dev,"manufacturer"),
		        udev_device_get_sysattr_value(dev,"product"));
		printf("  serial: %s\n",
		         udev_device_get_sysattr_value(dev, "serial"));
			 */
		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	return 0;       
}

char** get_lidars (char ** ls )
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	int index = 0;
	char *lidars [3];
	
	/* Create the udev object */
	
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		exit(1);
	}
	
	/* Create a list of the devices in the 'tty' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "tty");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	/* For each item enumerated, print out its information.
	   udev_list_entry_foreach is a macro which expands to
	   a loop. The loop will be executed for each member in
	   devices, setting dev_list_entry to a list entry
	   which contains the device's path in /sys. */
	udev_list_entry_foreach(dev_list_entry, devices) {
		printf("START");
		const char *path;
		
		/* Get the filename of the /sys entry for the device
		   and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);
		//const char *tty_path = udev_device_get_devnode(dev);
		//printf("Device Node Path: %s\n", udev_device_get_devnode(dev));
		udev_device_unref(dev);
		/* usb_device_get_devnode() returns the path to the device node
		   itself in /dev. */

		/* The device pointed to by dev contains information about
		   the hidraw device. In order to get information about the
		   USB device, get the parent device with the
		   subsystem/devtype pair of "usb"/"usb_device". This will
		   be several levels up the tree, but the function will find
		   it.*/


		/*
		dev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
		if(dev){

		*/
			/* From here, we can call get_sysattr_value() for each file
			   in the device's /sys entry. The strings passed into these
			   functions (idProduct, idVendor, serial, etc.) correspond
			   directly to the files in the directory which represents
			   the USB device. Note that USB strings are Unicode, UCS2
			   encoded, but the strings returned from
			   udev_device_get_sysattr_value() are UTF-8 encoded. */
			/*printf("  VID/PID: %s %s\n",
				udev_device_get_sysattr_value(dev,"idVendor"),
				udev_device_get_sysattr_value(dev, "idProduct"));
				*/

		/*
			const char * val = udev_device_get_sysattr_value(dev,"idVendor");
			const char * lidar = "10c4";
			if (val != NULL){
				if (strcmp(val, lidar) == 0){
					printf("%s\n", tty_path);
					char new_s[100];
					strcpy(new_s, tty_path);	
					lidars[index] = new_s;
					printf("LIDAR!\n");		
					index ++;
				}
			}

			/*printf("  %s\n  %s\n",
				udev_device_get_sysattr_value(dev,"manufacturer"),
				udev_device_get_sysattr_value(dev,"product"));
			printf("  serial: %s\n",
				 udev_device_get_sysattr_value(dev, "serial"));
				 */
		//}
	//		udev_device_unref(dev);
		printf("ASD\n");
			
	}
	printf("FREE");	
	exit(1);
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	printf("asdf");
	//ls = lidars;
	//return ls;       
}

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

    void check_dev(char * opt_channel_param_first){
	char * opt_is_channel = NULL; 
	char * opt_channel = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	bool useArgcBaudrate = false;

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
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



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

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
           /*     //printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    */
            }
        }
//
//        if (ctrl_c_pressed){ 
//            break;
//        }
    
    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
    }

int main(int argc, const char * argv[]) {

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", "SL_LIDAR_SDK_VERSION");
    char ** a_lidars;
    //a_lidars  = get_lidars(a_lidars);
    gl();
    exit(1);
    printf("PRINTING A LIDAR");
    for (int i = 0; i <3; i++){
    	printf("%s", a_lidars[i]);	
    }

	 
    char * opt_channel_param_first = NULL;
    char * opt_channel_param_second= NULL;
	opt_channel_param_first = "/dev/ttyUSB0";

	//opt_channel_param_first = "/dev/bus/usb/001/011";
	//opt_channel_param_second= "/dev/ttyUSB1";


    check_dev(opt_channel_param_first);
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


