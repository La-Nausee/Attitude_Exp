#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "spi_if.h"

#define ADIS_RST_PIN      8
#define SPI_CE0           0
#define SPI_CE1           1
#define SPI_SCLK_ADIS     ADIS16448_SPI_BURST //Hz
#define SPI_SCLK_ICM      ICM20948_SPI_SPEED//Hz

using namespace std;

void spi_init()
{
	int fd_adis,fd_icm;
	int spiMode = 3;
     
	cout<< "Setup WiringPi" <<endl;	
	wiringPiSetup () ;

	fd_adis = wiringPiSPISetup(SPI_CE1, SPI_SCLK_ADIS);
	ioctl(fd_adis,SPI_IOC_WR_MODE,&spiMode);
	
	cout << "ADIS16448 spi setup result: " << fd_adis << endl;
	
	cout << "Reset ADIS16448" << endl ;
	pinMode(ADIS_RST_PIN,OUTPUT);
	digitalWrite (ADIS_RST_PIN, LOW) ;
	sleep(1);
	digitalWrite (ADIS_RST_PIN, HIGH) ;
	
	fd_icm = wiringPiSPISetup(SPI_CE0, SPI_SCLK_ICM);
	ioctl(fd_icm,SPI_IOC_WR_MODE,&spiMode);
	
	cout << "ICM20948 spi setup result: " << fd_icm << endl;
	
}

int16_t adis_single_read(uint8_t reg)
{
 	unsigned char buffer[2];
	
	memset(buffer,0,2);
	buffer[0] = reg;
        buffer[1] = 0;

	wiringPiSPIDataRW(SPI_CE1,buffer,2);
	
	memset(buffer,0,2);
	wiringPiSPIDataRW(SPI_CE1,buffer,2);

	return ((buffer[0]<<8)|buffer[1]);
}

void adis_single_write(uint8_t reg,uint16_t reg_val)
{
 	unsigned char buffer[2];
	
	memset(buffer,0,2);
	buffer[0] = reg|0x80;
        buffer[1] = (reg_val)&0xff;

	wiringPiSPIDataRW(SPI_CE1,buffer,2);
	
	memset(buffer,0,2);
	buffer[0] = reg|0x80|0x01;
        buffer[1] = (reg_val>>8)&0xff;

	wiringPiSPIDataRW(SPI_CE1,buffer,2);
}

void adis_burst_read(uint8_t* p)
{
 	unsigned char buffer[2];
	unsigned char temp;	
	
	memset(buffer,0,2);
	buffer[0] = 0x3E;
        buffer[1] = 0;

	wiringPiSPIDataRW(SPI_CE1,buffer,2);
	
	memset(p,0,24);
	wiringPiSPIDataRW(SPI_CE1,p,24);

	//exhange high byte and low byte
	for(int i=0;i<24;i+=2)
	{
		temp = p[i+1];
		p[i+1] = p[i];
		p[i] = temp;	
	}

}

void adis_global_cmd(uint8_t val)
{

 	unsigned char buffer[2];
	
	memset(buffer,0,2);
	buffer[0] = 0xBE;
        buffer[1] = val;

	wiringPiSPIDataRW(SPI_CE1,buffer,2);
}

static int icm_spi_init(void)
{
	return 0;
}

static int icm_read_reg(uint8_t reg, const uint8_t * rbuffer, uint32_t rlen)
{
	uint8_t *buffer = (uint8_t*) malloc( (size_t)rlen+1);
	
	memset(buffer,0,rlen+1);
	buffer[0] = reg|0x80;

	wiringPiSPIDataRW(SPI_CE0,buffer,rlen+1);

	memcpy(rbuffer,buffer+1,rlen);
	free(buffer);

	return 0;
} 

static int icm_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{

	uint8_t *buffer = (uint8_t*)malloc( (size_t)wlen+1);
	
	memset(buffer,0,wlen+1);
	buffer[0] = reg;
	memcpy(buffer+1,wbuffer,wlen);

	wiringPiSPIDataRW(SPI_CE0,buffer,wlen+1);

	free(buffer);

	return 0;
} 

static const inv_host_serif_t serif_instance_spi = {
	icm_spi_init,
	0,
	icm_read_reg,
	icm_write_reg,
	0,
	1024*32,
	1024*32,
	INV_HOST_SERIF_TYPE_SPI,
};

const inv_host_serif_t* icm_get_serif_instance_spi(void)
{
	return &serif_instance_spi;
}
