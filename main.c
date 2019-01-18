#include<stdio.h>
#include<string.h>
#include<errno.h>
#include<unistd.h>
#include<fcntl.h>
#include<sys/ioctl.h>

#include"hc_ioctl.h"

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

/*
 * export gpio number
 * @gpio - gpio number to be exported
 */
int gpio_export(unsigned int gpio){
	
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
 
	return 0;
}

/*
 * set value for gpio number
 * @gpio - gpio number to be set
 */
int gpio_set_value(unsigned int gpio){
	int fd;
	char buf[MAX_BUF];
 
	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/value");
		return fd;
	}

	write(fd, "0", 1);
 
	close(fd);
	return 0;
}

/*
 * unexport gpio number
 * @gpio - gpio number to be unexported
 */
int gpio_unexport(unsigned int gpio){
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/*
 * setup mux for gpio number
 * @gpio - gpio number for which mux setup carried out
 */
int mux_gpio_set(unsigned int gpio){
	gpio_export(gpio);
	gpio_set_value(gpio);

	return 0;
}

int main(void){

	int IO_Trig0 = 7;
	int IO_Trig1 = 8;
	int IO_Echo0 = 5;
	int IO_Echo1 = 9;
	int m0 = 7;
	int delta0 = 80;
	int m1 = 8;
	int delta1 = 80;

	/*******************************Shield pins to GPIO pins mapping**************************/
	int row;

	int gpio_trigger0 = -1, gpio_trigger1 = -1, gpio_Echo0 = -1, gpio_Echo1 = -1;
	int gpio_t_level_shifter0, gpio_t_level_shifter1;
	int gpio_E_level_shifter0, gpio_E_level_shifter1;
	int gpio_t_mux1_0, gpio_t_mux2_0, gpio_t_mux1_1, gpio_t_mux2_1;
	int gpio_E_mux1_0, gpio_E_mux2_0, gpio_E_mux1_1, gpio_E_mux2_1;

	for(row=0; row<17; row++)
	{
		if(IO_Trig0 == IO_to_trigger_echo[row][0])
		{
			gpio_trigger0=IO_to_trigger_echo[row][1];
			gpio_t_level_shifter0=IO_to_trigger_echo[row][2];
			if(gpio_t_level_shifter0 != -1)
			{
				mux_gpio_set(gpio_t_level_shifter0);
			}
			gpio_t_mux1_0=IO_to_trigger_echo[row][3];
			if(gpio_t_mux1_0 != -1)
			{
				mux_gpio_set(gpio_t_mux1_0);
			}
			gpio_t_mux2_0=IO_to_trigger_echo[row][4];
			if(gpio_t_mux2_0 != -1)
			{
				mux_gpio_set(gpio_t_mux2_0);
			}

		}
		else if(IO_Trig1 == IO_to_trigger_echo[row][0])
		{
			gpio_trigger1=IO_to_trigger_echo[row][1];
			gpio_t_level_shifter1=IO_to_trigger_echo[row][2];
			if(gpio_t_level_shifter1 != -1)
			{
				mux_gpio_set(gpio_t_level_shifter0);
			}
			gpio_t_mux1_1=IO_to_trigger_echo[row][3];
			if(gpio_t_mux1_1 != -1)
			{
				mux_gpio_set(gpio_t_mux1_1);
			}
			gpio_t_mux2_1=IO_to_trigger_echo[row][4];
			if(gpio_t_mux2_1 != -1)
			{
				mux_gpio_set(gpio_t_mux2_1);
			}

		}
		else if(IO_Echo0 == IO_to_trigger_echo[row][0])
		{
			gpio_Echo0=IO_to_trigger_echo[row][1];
			gpio_E_level_shifter0=IO_to_trigger_echo[row][2];
			if(gpio_E_level_shifter0 != -1)
			{
				mux_gpio_set(gpio_E_level_shifter0);
			}
			gpio_E_mux1_0=IO_to_trigger_echo[row][3];
			if(gpio_E_mux1_0 != -1)
			{
				mux_gpio_set(gpio_E_mux1_0);
			}
			gpio_E_mux2_0=IO_to_trigger_echo[row][4];
			if(gpio_E_mux2_0 != -1)
			{
				mux_gpio_set(gpio_E_mux2_0);
			}

		}
		else if(IO_Echo1 == IO_to_trigger_echo[row][0])
		{
			gpio_Echo1=IO_to_trigger_echo[row][1];
			gpio_E_level_shifter1=IO_to_trigger_echo[row][2];
			if(gpio_E_level_shifter1 != -1)
			{
				mux_gpio_set(gpio_E_level_shifter1);
			}
			gpio_E_mux1_1=IO_to_trigger_echo[row][3];
			if(gpio_E_mux1_1 != -1)
			{
				mux_gpio_set(gpio_E_mux1_0);
			}
			gpio_E_mux2_1=IO_to_trigger_echo[row][4];
			if(gpio_E_mux2_1 != -1)
			{
				mux_gpio_set(gpio_E_mux2_1);
			}
		}
	}

	if(gpio_trigger0 == -1 || gpio_Echo0 == -1 || gpio_trigger1 == -1 || gpio_Echo1 == -1){
		printf("Invalid pins, choose between 0 to 17\n");
		return 0;
	}

	/******************initialization****************/

	// HCSR_0
	struct device_pins dpins0 = {gpio_trigger0, gpio_Echo0};
	struct m_delta md0 = {m0, delta0};

	int fd0;
	struct device_pins *pindata0 = &dpins0;
	struct m_delta *mddata0 = &md0;
	int write0 = 0;
	struct node n0;

	//HCSR_1
	struct device_pins dpins1 = {gpio_trigger1, gpio_Echo1};
	struct m_delta md1 = {m1, delta1};

	int fd1;
	struct device_pins *pindata1 = &dpins1;
	struct m_delta *mddata1 = &md1;
	int write1 = 0;
	struct node n1;



	fd0 = open("/dev/HCSR_0", O_RDWR);
	if(fd0 == -1){
		printf("file open error\n");
	}

	fd1 = open("/dev/HCSR_1", O_RDWR);
	if(fd1 == -1){
		printf("file open error\n");
	}
	
	ioctl(fd0, CONFIG_PINS, pindata0);
	ioctl(fd0, SET_PARAMETERS, mddata0);
		
	ioctl(fd1, CONFIG_PINS, pindata1);
	ioctl(fd1, SET_PARAMETERS, mddata1);

	/************Note: all scenarios are carried out on multiple devices simultaneously********/

	/***** scenario 0 - simple write + read ********/
	printf("Scenario 0\n");
	write(fd0, &write0, sizeof(int));

	write(fd1, &write1, sizeof(int));

	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);

	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);

	/***** scenario 1 - write nonzero value (clears buffer + new measurement) + read on empty(triggers new measurement) ********/
	printf("Scenario 1\n");
	write0 = 1;
	write1 = 1;

	write(fd0, &write0, sizeof(int));

	write(fd1, &write1, sizeof(int));

	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);

	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);

	/***** scenario 3 - read on empty twice ********/
	printf("Scenario 3\n");
	write0 = 1;
	write1 = 1;

	write(fd0, &write0, sizeof(int));
	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);

	write(fd1, &write1, sizeof(int));
	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);

	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);

	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);

	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);

	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);

	/***** scenario 4 - write followed by write ********/
	printf("Scenario 4\n");
	write0 = 1;
	write1 = 1;

	write(fd0, &write0, sizeof(int));
	write(fd0, &write0, sizeof(int));

	write(fd1, &write1, sizeof(int));
	write(fd1, &write1, sizeof(int));

	/***** scenario 5 - write + read + read ********/
	printf("Scenario 5\n");
	write0 = 1;
	write1 = 1;

	write(fd0, &write0, sizeof(int));
	write(fd1, &write1, sizeof(int));

	write(fd0, &write0, sizeof(int));
	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);
	read(fd0, &n0, sizeof(int));
	printf("%d %lld\n", n0.distance, n0.timestamp);	

	write(fd1, &write1, sizeof(int));
	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);
	read(fd1, &n1, sizeof(int));
	printf("%d %lld\n", n1.distance, n1.timestamp);


	/************************************* GPIO unexport***************************************/
	if(gpio_t_level_shifter0 != -1)
		{
			gpio_unexport(gpio_t_level_shifter0);
		}
			
	if(gpio_t_mux1_0 != -1)
		{
			gpio_unexport(gpio_t_mux1_0);
		}
			
	if(gpio_t_mux2_0 != -1)
		{
			gpio_unexport(gpio_t_mux2_0);
		}
	if(gpio_t_level_shifter1 != -1)
		{
			gpio_unexport(gpio_t_level_shifter0);
		}
			
	if(gpio_t_mux1_1 != -1)
		{
			gpio_unexport(gpio_t_mux1_1);
		}
			
	if(gpio_t_mux2_1 != -1)
		{
			gpio_unexport(gpio_t_mux2_1);
		}
	if(gpio_E_level_shifter0 != -1)
		{
			gpio_unexport(gpio_E_level_shifter0);
		}
			
	if(gpio_E_mux1_0 != -1)
		{
			gpio_unexport(gpio_E_mux1_0);
		}
			
	if(gpio_E_mux2_0 != -1)
		{
			gpio_unexport(gpio_E_mux2_0);
		}
	if(gpio_E_level_shifter1 != -1)
		{
			gpio_unexport(gpio_E_level_shifter0);
		}
			
	if(gpio_E_mux1_1 != -1)
		{
			gpio_unexport(gpio_E_mux1_1);
		}
			
	if(gpio_E_mux2_1 != -1)
		{
			gpio_unexport(gpio_E_mux2_1);
		}

	close(fd0);	
	close(fd1);

	return 0;
}