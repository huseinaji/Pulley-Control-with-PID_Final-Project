#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>
#include <time.h>

#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include "dynamixel_sdk.h"

using namespace std; 

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_SPEED           38
#define ADDR_MX_PRESENT_LOAD            40
#define ADDR_MX_PRESENT_TEMPERATURE     43
#define ADDR_MX_PRESENT_VOLTAGE         42

#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define DXL_ID_ATAS                     1                   // Dynamixel MX-28 ID: 1 //AX-12A ID:3
#define DXL_ID_BAWAH                    3                   // Dynamixel MX-28 ID: 1 //AX-12A ID:3

#define BAUDRATE                        200000
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    

int file, res;
//////////DYNAMIXEL_SERVO///////////
float load = 0;
float voltage = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//int DXL_MOVING_SPEED = 0;
uint8_t dxl_error = 0;                          // Dynamixel error
uint16_t dxl_present_position = 0;              // Present position
uint16_t dxl_present_load = 0;                  // Present load
uint8_t dxl_present_voltage = 0;               // Present voltage
uint16_t dxl_present_speed = 0;                 //Present speed

void DynamixelInit()
{
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
    }

    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel has been successfully connected \n");
    }
}

void servobawahopen()
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_GOAL_POSITION, 500, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void servobawahclose()
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_GOAL_POSITION, 830, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void bacaservo()
{
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

int write_data(int wreg, int value){
    res = i2c_smbus_write_byte_data(file, wreg, value);
    usleep(10000);
    if(res < 0){
        printf("Write Error"); return -1;
    }
    else{return 0;}
}

int read_data(int rreg){
    res = i2c_smbus_read_byte_data(file, rreg);
    usleep(10000);
    if(res < 0){
        printf("Read Error"); return -1;
    }
    else{return 0;}
}

static int GPIOExport(int pin){
#define BUFFER_MAX 3
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open export for writing!\n");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

static int GPIOUnexport(int pin){
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open unexport for writing!\n");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

static int GPIODirection(int pin, int dir){
    static const char s_directions_str[]  = "in\0out";

#define DIRECTION_MAX 35
    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio direction for writing!\n");
        return(-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        fprintf(stderr, "Failed to set direction!\n");
        return(-1);
    }

    close(fd);
    return(0);
}

static int GPIORead(int pin){
#define VALUE_MAX 30
    char path[VALUE_MAX];
    char value_str[3];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for reading!\n");
        return(-1);
    }

    if (-1 == read(fd, value_str, 3)) {
        fprintf(stderr, "Failed to read value!\n");
        return(-1);
    }

    close(fd);

    return(atoi(value_str));
}

static int GPIOWrite(int pin, int value){
    static const char s_values_str[] = "01";

    char path[VALUE_MAX];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for writing!\n");
        return(-1);
    }

    if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
        fprintf(stderr, "Failed to write value!\n");
        return(-1);
    }

    close(fd);
    return(0);
}
int flag = 0;
int main()
{	
	DynamixelInit();
	
	GPIOExport(7);
	GPIODirection(7, IN);
	
	while(1)
	{
		if((flag==0) && (GPIORead(7)==0)){
			flag = 1;
			servobawahclose();
			while(GPIORead(7)==0){}
		}
		if((flag==1) && (GPIORead(7)==0)){
			flag = 0;
			servobawahopen();
			while(GPIORead(7)==0){}
		}
		
		bacaservo();
		printf("limitswitch: %d posisi: %d\n", GPIORead(7), dxl_present_position);
		usleep(100*1000);
	}
}
