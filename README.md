# HCSR04-device-driver

This software is a charachter device driver for HC-SR04 ultrasonic distance sensor devices.
It can handle maximum of 10 devices at a time.
The device generates ultrasonic pulses and the distance is measured according to the pulse width of echo pulse.

The device is triggered on trigger pin using a pulse of about 120us, 5V.
It receives a pulse on echo pin whose width is proportional to the distance of the object from device.

Driver Compilation and Usage:
-----------------------------

Assign the following variables in main.c to configure shield pins to trigger, echo pins and m, delta values
int IO_Trig0 = 7;
int IO_Trig1 = 8;
int IO_Echo0 = 5;
int IO_Echo1 = 9;
int m0 = 7;
int delta0 = 80;
int m1 = 8;
int delta1 = 80;

The driver is installed using command line directed at the driver file location, with following commands
1) make
2) insmod hc_driver.ko

The user level code can access the device files and run the user level code using
3) chmod 777 hcsr_tester
4) ./hcsr_tster

The driver can be uninstalled using
5) rmmod hc_driver.ko
6) make clean



OPEN:
-----
Device files are created in /dev folder with HCSR_* name format. These need to be opened to interact with the devices.

CLOSE:
------
The device files should be closed after use to maintain proper functioning of the devices.

IOCTL:
------
To configure the device, IOCTL commands are passed to the device.
CONFIG_PINS needs an struct argument of {trigger_pin, echo_pin} which sets the trigger pin and echo pin.
SET_PARAMETERS needs an struct argument of {m, delta} where m indicate the number of samples per measurement and delta indicates the sampling period.
trigger pins should be chosen from 0 to 17
echo pins should be chose from 0 to 17
m should be greater than 5
delta should be greater than 60

READ:
-----
Data is read from the device buffer into a variable of struct node which contains {distance, timestamp}.
distance (in cm) represents the distance of object from device
timestamp represents a unique 64bit number when the measurement was taken according to device
If the device buffer is empty, read operation starts a new measurement

WRITE:
------
Write operation starts a new measurement and write to the device buffer
Writing 0 to device starts a new measurement
Writing non-zero value to the device clears the device buffer and starts a new measurement
