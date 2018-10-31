#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef JY901_H
#define JY901_H

#define JY_SAVE		0x00
#define JY_RSW		0x02
#define	JY_RATE		0x03
#define JY_BAUD		0x04

typedef struct{
	uint8_t header;
	uint8_t rev;
	uint8_t addr;
	uint8_t dataL;
	uint8_t dataH;
}jy_cmd_t;
	
#endif
