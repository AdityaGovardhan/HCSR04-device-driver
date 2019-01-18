#include<linux/ioctl.h>

// use 'h' as a magic number
#define HCSR_IOC_MAGIC 	'h'

// set device pins
#define CONFIG_PINS 	_IOW(HCSR_IOC_MAGIC, 0, struct device_pins)

// set m and delta values for the device
#define SET_PARAMETERS 	_IOW(HCSR_IOC_MAGIC, 1, struct m_delta)

// structure to be initialized for CONFIG_PINS ioctl call
struct device_pins{
	int trigger_pin;
	int echo_pin;
};

// structure to be initialized for SET_PARAMETERS ioctl call
struct m_delta{
	int m;
	int delta;
};

// data node for circular buffer
struct node{
	int distance;
	unsigned long long timestamp;
};

// Array for getting gpio pins and muxes from shield pins 
int IO_to_trigger_echo[18][5]={
	{0,11,32,-1,-1},
	{1,12,28,45,-1},
	{2,13,34,77,-1},
	{3,62,-1,76,64},
	{4,6,36,-1,-1},
	{5,0,18,66,-1},
	{6,1,20,68,-1},
	{7,38,-1,-1,-1},
	{8,40,-1,-1,-1},
	{9,4,22,70,-1},
	{10,10,26,74,-1},
	{11,5,24,44,72},
	{12,15,42,-1,-1},
	{13,7,30,46,-1},
	{14,48,-1,-1,-1},
	{15,50,-1,-1,-1},
	{16,52,-1,-1,-1},
	{17,54,-1,-1,-1},
};