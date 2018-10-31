#include <fstream>
#include <iostream>
#include <cassert>
#include <cstring>
#include <queue>
#include <pthread.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <sys/time.h>
#include <assert.h>
#include "spi_if.h"
#include "jy901.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

#define ADIS_INT_PIN 9
#define ICM_INT_PIN  7
#define ICM_INT_EVENT     0x01
#define ADIS_INT_EVENT    0x02
#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define LOG_START_EVENT   0x05
#define LOG_STOP_EVENT    0x06
#define APP_EXIT_EVENT    0x07
#define ODR_NONE	0

#define USE_RAW_ACC 0
#define USE_RAW_GYR 0
#define USE_GRV     0
#define USE_CAL_ACC 1
#define USE_CAL_GYR 1
#define USE_CAL_MAG 1
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_RV      0    /* requires COMPASS*/
#define USE_GEORV   0    /* requires COMPASS*/
#define USE_ORI     0    /* requires COMPASS*/
#define USE_STEPC   0
#define USE_STEPD   0
#define USE_SMD     0
#define USE_BAC     0
#define USE_TILT    0
#define USE_PICKUP  0
#define USE_GRAVITY 0
#define USE_LINACC  0
#define USE_B2S     0

using namespace std;

static const struct {
	uint8_t  type;
	uint32_t period_us;
} sensor_list[] = {
#if USE_RAW_ACC
	{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_RAW_GYR
	{ INV_SENSOR_TYPE_RAW_GYROSCOPE,     50000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
	{ INV_SENSOR_TYPE_ACCELEROMETER, 20000 /* 50 Hz */ },
#endif
#if USE_CAL_GYR
	{ INV_SENSOR_TYPE_GYROSCOPE, 20000 /* 50 Hz */ },
#endif
#if USE_CAL_MAG
	{ INV_SENSOR_TYPE_MAGNETOMETER, 20000 /* 50 Hz */ },
#endif
#if USE_UCAL_GYR
	{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
	{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
	{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
	{ INV_SENSOR_TYPE_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_GEORV
	{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_ORI
	{ INV_SENSOR_TYPE_ORIENTATION, 50000 /* 20 Hz */ },
#endif
#if USE_STEPC
	{ INV_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
	{ INV_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE},
#endif
#if USE_SMD
	{ INV_SENSOR_TYPE_SMD, ODR_NONE},
#endif
#if USE_BAC
	{ INV_SENSOR_TYPE_BAC, ODR_NONE},
#endif
#if USE_TILT
	{ INV_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE},
#endif
#if USE_PICKUP
	{ INV_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE},
#endif
#if USE_GRA
	{ INV_SENSOR_TYPE_GRAVITY, 50000 /* 20 Hz */},
#endif
#if USE_LINACC
	{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */},
#endif
#if USE_B2S
	{ INV_SENSOR_TYPE_B2S, ODR_NONE},
#endif
};

static void sensor_event_cb(const inv_sensor_event_t * event, void * arg);

static const uint8_t dmp3_image[] = {
	#include "Invn/Images/icm20948_img.dmp3a.h"
};

static inv_device_icm20948_t device_icm20948;

static inv_device_t * device;

static const inv_sensor_listener_t sensor_listener = {
	sensor_event_cb,
	0
};

std::queue<char> adis_queue;
std::queue<char> icm_queue;
std::queue<char> jy_queue;
ofstream adis_logfile;
ofstream icm_logfile;
ofstream jy_logfile;
string adis_filename;
string icm_filename;
string jy_filename;

void icm_handler(void)
{
	icm_queue.push(ICM_INT_EVENT);
}

void icm_setup()
{
	int rc = 0;
	uint8_t i = 0;
	uint8_t whoami = 0xff;
	uint64_t available_sensor_mask = 0;

	/*Setup ICM20948*/
	if(wiringPiISR(ICM_INT_PIN,INT_EDGE_RISING,&icm_handler) < 0)
	{
		printf("[ERROR] Unable to setup ICM ISR.\r\n");
	}

	inv_host_serif_open(icm_get_serif_instance_spi());

	inv_device_icm20948_init(&device_icm20948, icm_get_serif_instance_spi(), &sensor_listener, dmp3_image, sizeof(dmp3_image));

	device = inv_device_icm20948_get_base(&device_icm20948);

	inv_device_whoami(device, &whoami);

	printf("ICM WHOAMI = 0x%02X\r\n", whoami);

	cout<<"Setting-up ICM device"<<endl;

	inv_device_setup(device);

	inv_device_load(device,NULL,dmp3_image,sizeof(dmp3_image),true,NULL);

	//check sensor availibility
	for(i = 0; i< sizeof(sensor_list)/sizeof(sensor_list[0]); ++i){
		const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
		printf("Ping %s %s\r\n", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
		if(rc == 0)
		{
			available_sensor_mask |= (1<<sensor_list[i].type);
		}
	}
	
	//start all available sensors from sensor list
	for(i = 0; i< sizeof(sensor_list)/sizeof(sensor_list[0]); ++i){
		//if(available_sensor_mask & (1<<sensor_list[i].type)){
			printf("Starting %s @ %u us\r\n", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
			rc = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
			rc += inv_device_start_sensor(device, sensor_list[i].type);
		//}
	}
}

void *icm_log_thread(void *threadid)
{
	int rc = 0;
	long tid;
	tid = (long)threadid;
	while(1)
	{
		if(!icm_queue.empty())
		{
			switch(icm_queue.front())
			{
			case ICM_INT_EVENT:
			rc = inv_device_poll(device);
			if(rc >= 0)
			{
				icm_queue.pop();
			}
			break;
			case NEW_FILE_EVENT:
			printf("\r\n[INFO] create new ICM log file.\r\n");
			if(icm_logfile.is_open())
				icm_logfile.close();
			icm_logfile.open(icm_filename.c_str());
			assert(!icm_logfile.fail());
			//write headers
			if(icm_logfile.is_open())
			{
				icm_logfile<<"seconds.milliseconds,";
                                icm_logfile<<"gx(dps),gy,gz,";
                                icm_logfile<<"ax(g),ay,az,";
                                icm_logfile<<"mx(uT),my,mz,";
                                icm_logfile<<endl;
			}
			icm_queue.pop();	
			break;
			case FILE_CLOSE_EVENT:
			printf("\r\n[INFO] close ICM log file.\r\n");
                        if(icm_logfile.is_open())
                        	icm_logfile.close();
			icm_queue.pop();	
			break;
			case LOG_START_EVENT:
			printf("\r\n[INFO] ICM log start.\r\n");
			icm_queue.pop();	
			break;
			case LOG_STOP_EVENT:
			 printf("\r\n[INFO] ICM log stop.\r\n");
			icm_queue.pop();	
			break;
			case APP_EXIT_EVENT:
			if(icm_logfile.is_open())
                                        icm_logfile.close();
                        printf("\r\n[INFO] exit ICM thread.\r\n");
			icm_queue.pop();	
			
			pthread_exit(NULL);
			break;
			default:
			icm_queue.pop();	
			break;
			}
		}
	}

}		

void adis_handler(void)
{
	adis_queue.push(ADIS_INT_EVENT);
}

void adis_setup()
{
	int16_t reg, reg_val;

	if(wiringPiISR(ADIS_INT_PIN,INT_EDGE_RISING,&adis_handler) < 0)
	{
		printf("[ERROR] Unable to setup ADIS ISR.\r\n");
	}

	//software reset
        //adis_global_cmd(1<<7);

        //factory calibration restore,clear all calibration register = 0
        //adis_global_cmd(1<<1);

        reg = ADIS16448_PRODUCT_ID;
        reg_val = adis_single_read(reg);
        printf("[INFO] PRODUCT ID: %02x,%02x\r\n",(reg_val>>8)&0xff,reg_val&0xff);

        //range=1000 dps, and low pass filter N = 4
        reg = ADIS16448_SENS_AVG;
        adis_single_write(reg,0x0402);
        reg_val = adis_single_read(reg);
        printf("[INFO] SENS AVG: %02x,%02x\r\n",(reg_val>>8)&0xff,reg_val&0xff);

        //D=3, sampling rate = 102.4 SPS,mag and baro = 51.2 SPS,internal clock
        //D=4, sampling rate = 50 SPS,mag and baro = 50 SPS,external clock
        reg = ADIS16448_SMPL_PRD;
        //adis_single_write(reg,0x0301); //internal clock
        adis_single_write(reg,0x0300); //external clock
        reg_val = adis_single_read(reg);
        printf("[INFO] SMPL PRD: %02x,%02x\r\n",(reg_val>>8)&0xff,reg_val&0xff);
}

void *adis_log_thread(void *threadid)
{
	long tid;
	adis_raw_t adis_raw_data;
	adis_data_t adis_data;
	struct timeval tp;
	tid = (long)threadid;
	while(1)
	{
		if(!adis_queue.empty())
		{
			switch(adis_queue.front())
			{
			case ADIS_INT_EVENT:
				adis_burst_read((uint8_t*) &adis_raw_data);
                                gettimeofday(&tp,NULL);

                                adis_data.ax = adis_raw_data.xaccl_out/1200.0;
                                adis_data.ay = adis_raw_data.yaccl_out/1200.0;
                                adis_data.az = adis_raw_data.zaccl_out/1200.0;
                                adis_data.gx = adis_raw_data.xgyro_out/25.0;
                                adis_data.gy = adis_raw_data.ygyro_out/25.0;
                                adis_data.gz = adis_raw_data.zgyro_out/25.0;
                                adis_data.mx = (adis_raw_data.xmagn_out>>1)/7.0;
                                adis_data.my = (adis_raw_data.ymagn_out>>1)/7.0;
                                adis_data.mz = (adis_raw_data.zmagn_out>>1)/7.0;
                                adis_data.baro = adis_raw_data.baro_out*0.02;
                                adis_data.temp = ((adis_raw_data.temp_out&0xfff)-4096)*0.07386 + 31;
                                //mag and baro not ready
                                if(!(adis_raw_data.diag_stat&0x80))
                                {
                                        adis_data.mx = 0;
                                        adis_data.my = 0;
                                        adis_data.mz = 0;
                                        adis_data.baro = 0;
                                }
                                if(adis_logfile.is_open())
                                {
                                        adis_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
                                        adis_logfile<<adis_data.ax<<","<<adis_data.ay<<","<<adis_data.az<<",";
                                        adis_logfile<<adis_data.gx<<","<<adis_data.gy<<","<<adis_data.gz<<",";
                                        adis_logfile<<adis_data.mx<<","<<adis_data.my<<","<<adis_data.mz<<",";
                                        adis_logfile<<adis_data.baro<<",";
                                        adis_logfile<<adis_data.temp<<endl;
                                }
			break;
			case NEW_FILE_EVENT:
			printf("\r\n[INFO] create new ADIS log file.\r\n");
			if(adis_logfile.is_open())
				adis_logfile.close();
			adis_logfile.open(adis_filename.c_str());
			assert(!adis_logfile.fail());
			//write headers
			if(adis_logfile.is_open())
			{
				adis_logfile<<"seconds.milliseconds,";
                                adis_logfile<<"ax(g),ay,az,";
                                adis_logfile<<"gx(dps),gy,gz,";
                                adis_logfile<<"mx(mgauss),my,mz,";
                                adis_logfile<<"baro(mbar),";
                                adis_logfile<<"temperature(degree)"<<endl;
			}
			break;
			case FILE_CLOSE_EVENT:
			printf("\r\n[INFO] close ADIS log file.\r\n");
                        if(adis_logfile.is_open())
                        	adis_logfile.close();
			break;
			case LOG_START_EVENT:
			printf("\r\n[INFO] ADIS log start.\r\n");
			break;
			case LOG_STOP_EVENT:
			 printf("\r\n[INFO] ADIS log stop.\r\n");
			break;
			case APP_EXIT_EVENT:
			if(adis_logfile.is_open())
                                        adis_logfile.close();
                        printf("\r\n[INFO] exit ADIS thread.\r\n");
			
			pthread_exit(NULL);
			break;
			default:
			break;
			}
			adis_queue.pop();
		}
	}

}		

void jy_set_once()
{
	int fd;
	int i;
	jy_cmd_t cmd;
	uint8_t *buffer = (uint8_t*)&cmd;
	cmd.header = 0xFF;
	cmd.rev = 0xAA; 	

	if((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0)
  	{
    		printf("Unable to open serial device\r\n") ;
    		return;
   	}

	//set receive package
	cmd.addr = JY_RSW;
	cmd.dataL = (1<<4)|(1<<2)|(1<<1);
	cmd.dataH = 0;
	i = 0;
	while(i < sizeof(jy_cmd_t))
	{
		serialPutchar(fd,*(buffer+i));
		i++;
	}
	
	
	//set data rate
	cmd.addr = JY_RATE;
	cmd.dataL = 0x09;
	cmd.dataH = 0;
	i = 0;
	while(i < sizeof(jy_cmd_t))
	{
		serialPutchar(fd,*(buffer+i));
		i++;
	}

	//set baud rate
	cmd.addr = JY_BAUD;
	cmd.dataL = 0x06;
	cmd.dataH = 0;
	i = 0;
	while(i < sizeof(jy_cmd_t))
	{
		serialPutchar(fd,*(buffer+i));
		i++;
	}
	
	serialClose(fd);
	
	if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0)
  	{
    		printf("Unable to open serial device\r\n") ;
    		return;
   	}

	//save config 
	cmd.addr = JY_SAVE;
	cmd.dataL = 0;
	cmd.dataH = 0;
	i = 0;
	while(i < sizeof(jy_cmd_t))
	{
		serialPutchar(fd,*(buffer+i));
		i++;
	}
	serialClose(fd);
}

void *jy_log_thread(void *threadid)
{
	long tid;
	int fd;
	uint8_t data;
	uint8_t sum;
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz,temperature;
	struct timeval tp;
	tid = (long)threadid;
	
	if((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0)
  	{
    		printf("Unable to open serial device\r\n") ;
   	}

	while(1)
	{
		while(serialDataAvail(fd))
		{
			data = serialGetchar(fd);
			//printf("JY data\r\n");
			if(data == 0x55)
			{
				//printf("JY data\r\n");
                		gettimeofday(&tp,NULL);
				//while(serialDataAvail(fd))
				//{
					data = serialGetchar(fd);
					if(data == 0x51) //acc
					{
						ax = serialGetchar(fd);
						data = serialGetchar(fd);
						ax |= (data<<8);
						ay = serialGetchar(fd);
						data = serialGetchar(fd);
						ay |= (data<<8);
						az = serialGetchar(fd);
						data = serialGetchar(fd);
						az |= (data<<8);
						temperature = serialGetchar(fd);
						data = serialGetchar(fd);
						temperature |= (data<<8);
						sum = serialGetchar(fd);
						if(jy_logfile.is_open())
						{
                                        		jy_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
							jy_logfile<<ax<<","<<ay<<","<<az<<",";
						}

					}
					else if(data == 0x52) //gyro
					{
						gx = serialGetchar(fd);
						data = serialGetchar(fd);
						gx |= (data<<8);
						gy = serialGetchar(fd);
						data = serialGetchar(fd);
						gy |= (data<<8);
						gz = serialGetchar(fd);
						data = serialGetchar(fd);
						gz |= (data<<8);
						temperature = serialGetchar(fd);
						data = serialGetchar(fd);
						temperature |= (data<<8);
						sum = serialGetchar(fd);
						if(jy_logfile.is_open())
							jy_logfile<<gx<<","<<gy<<","<<gz<<",";
					
					}
					else if(data == 0x54) //mag
					{
						mx = serialGetchar(fd);
						data = serialGetchar(fd);
						mx |= (data<<8);
						my = serialGetchar(fd);
						data = serialGetchar(fd);
						my |= (data<<8);
						mz = serialGetchar(fd);
						data = serialGetchar(fd);
						mz |= (data<<8);
						temperature = serialGetchar(fd);
						data = serialGetchar(fd);
						temperature |= (data<<8);
						sum = serialGetchar(fd);
						if(jy_logfile.is_open())
							jy_logfile<<mx<<","<<my<<","<<mz<<endl;
					}
					//printf("JY data\r\n");
					if(!jy_queue.empty())
					{
						switch(jy_queue.front())
						{
						case NEW_FILE_EVENT:
						printf("\r\n[INFO] create new JY901 log file.\r\n");
						if(jy_logfile.is_open())
							jy_logfile.close();
						jy_logfile.open(jy_filename.c_str());
						assert(!jy_logfile.fail());
						//write headers
						if(jy_logfile.is_open())
						{
							jy_logfile<<"seconds.milliseconds,";
                                			jy_logfile<<"ax(g),ay,az,";
                                			jy_logfile<<"gx(dps),gy,gz,";
                                			jy_logfile<<"mx(mgauss),my,mz,";
                                			jy_logfile<<"temperature(degree)"<<endl;
						}
						break;
						case FILE_CLOSE_EVENT:
						printf("\r\n[INFO] close JY901 log file.\r\n");
                        			if(jy_logfile.is_open())
                        				jy_logfile.close();
						break;
						case LOG_START_EVENT:
						printf("\r\n[INFO] JY901 log start.\r\n");
						break;
						case LOG_STOP_EVENT:
			 			printf("\r\n[INFO] JY901 log stop.\r\n");
						break;
						case APP_EXIT_EVENT:
						if(jy_logfile.is_open())
                                        		jy_logfile.close();
                        			printf("\r\n[INFO] exit JY901 thread.\r\n");
						serialClose(fd);
						pthread_exit(NULL);
						break;
						default:
						break;
						}
						jy_queue.pop();
					}
				//}	
			}
		}
		
	}
}

int main()
{
	char key = 0;
	pthread_t icm_thread;
	pthread_t adis_thread;
	pthread_t jy_thread;
	
	system("./clock.sh");

	spi_init();

	icm_setup();

	adis_setup();

	jy_set_once();
	
	pthread_create(&adis_thread,NULL,adis_log_thread,(void *)1234);
	pthread_create(&icm_thread,NULL,icm_log_thread,(void *)1234);
	pthread_create(&jy_thread,NULL,jy_log_thread,(void *)1234);

	while(1)
	{
		cout<<endl;
		cout<<"1: create new file for logging data"<<endl;
                cout<<"2: close and save data file"<<endl;
                cout<<"3: pause data logging"<<endl;
                cout<<"4: start data logging"<<endl;
                cout<<"5: save data file and exit the application"<<endl;
                cout<<"Input your selection:";
                key = getchar();
                if(key == '1')
                {	
			string str;
                        cout<<"Input the new filename:";
                        //getline(cin,filename);
                        cin>>str;
			adis_filename = "adis_";
			icm_filename  = "icm_";
			jy_filename = "jy_";
			adis_filename.append(str);
			icm_filename.append(str);
			jy_filename.append(str);
			adis_filename.append(".txt");
			icm_filename.append(".txt");
			jy_filename.append(".txt");
                        adis_queue.push(NEW_FILE_EVENT);
                        icm_queue.push(NEW_FILE_EVENT);
                        jy_queue.push(NEW_FILE_EVENT);
                }
                else if(key == '2')
		{
                        adis_queue.push(FILE_CLOSE_EVENT);
                        icm_queue.push(FILE_CLOSE_EVENT);
                        jy_queue.push(FILE_CLOSE_EVENT);
                }
		else if(key == '3')
		{
                        adis_queue.push(LOG_START_EVENT);
                        icm_queue.push(LOG_START_EVENT);
                        jy_queue.push(LOG_START_EVENT);
                }
		else if(key == '4')
		{
                        adis_queue.push(LOG_STOP_EVENT);
                        icm_queue.push(LOG_STOP_EVENT);
                        jy_queue.push(LOG_STOP_EVENT);
                }
		else if(key == '5')
                {
                        adis_queue.push(APP_EXIT_EVENT);
                        icm_queue.push(APP_EXIT_EVENT);
                        jy_queue.push(APP_EXIT_EVENT);
			pthread_join(icm_thread, NULL);
			pthread_join(adis_thread, NULL);
			pthread_join(jy_thread, NULL);
			exit(0);
                }
	}
 
	return 0;
}



static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	struct timeval tp;
	(void)arg;
	
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

		switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			printf("data event %s (lsb): %llu %d %d %d\r\n", inv_sensor_str(event->sensor),
					event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			printf("data event %s (lsb): %llu %d %d %d\r\n", inv_sensor_str(event->sensor),
					event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
			break;
		
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			//printf("data event %s (mg): %d %d %d\r\n", inv_sensor_str(event->sensor),
			//		(int)(event->data.acc.vect[0]*1000),
			//		(int)(event->data.acc.vect[1]*1000),
			//		(int)(event->data.acc.vect[2]*1000));
			if(icm_logfile.is_open())
			{
				icm_logfile<<event->data.acc.vect[0]<<",";
				icm_logfile<<event->data.acc.vect[1]<<",";
				icm_logfile<<event->data.acc.vect[2]<<",";
			}
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			//printf("data event %s (mdps): %d %d %d\r\n", inv_sensor_str(event->sensor),
			//		(int)(event->data.gyr.vect[0]*1000),
			//		(int)(event->data.gyr.vect[1]*1000),
			//		(int)(event->data.gyr.vect[2]*1000));
			gettimeofday(&tp,NULL);
			if(icm_logfile.is_open())
			{
				icm_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
				icm_logfile<<event->data.gyr.vect[0]<<",";
				icm_logfile<<event->data.gyr.vect[1]<<",";
				icm_logfile<<event->data.gyr.vect[2]<<",";
			}
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			//printf("data event %s (nT): %d %d %d\r\n", inv_sensor_str(event->sensor),
			//		(int)(event->data.mag.vect[0]*1000),
			//		(int)(event->data.mag.vect[1]*1000),
			//		(int)(event->data.mag.vect[2]*1000));
			if(icm_logfile.is_open())
			{
				icm_logfile<<event->data.mag.vect[0]<<",";
				icm_logfile<<event->data.mag.vect[1]<<",";
				icm_logfile<<event->data.mag.vect[2]<<endl;
			}
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			printf("data event %s (mdps): %d %d %d %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			printf("data event %s (nT): %d %d %d %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
			printf("data event %s (e-3): %d %d %d %d \r\n", inv_sensor_str(event->sensor),
					(int)(event->data.quaternion.quat[0]*1000),
					(int)(event->data.quaternion.quat[1]*1000),
					(int)(event->data.quaternion.quat[2]*1000),
					(int)(event->data.quaternion.quat[3]*1000));
			break;
		case INV_SENSOR_TYPE_ORIENTATION:
			printf("data event %s (e-3): %d %d %d %d \r\n", inv_sensor_str(event->sensor),
					(int)(event->data.orientation.x*1000),
					(int)(event->data.orientation.y*1000),
					(int)(event->data.orientation.z*1000));
			break;
		case INV_SENSOR_TYPE_BAC:
			//printf("data event %s : %d %s\r\n", inv_sensor_str(event->sensor),
					//event->data.bac.event, activityName(event->data.bac.event));
			break;
		case INV_SENSOR_TYPE_STEP_COUNTER:
			printf("data event %s : %lu\r\n", inv_sensor_str(event->sensor),
					(unsigned long)event->data.step.count);
			break;
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		
		default:
			printf("data event %s : ...\r\n", inv_sensor_str(event->sensor));
			break;
		}
	}

	return;
}
