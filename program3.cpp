#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>
#include <time.h>
#include <mosquitto.h>
#include <jansson.h>

#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <thread>
#include "dynamixel_sdk.h"

using namespace std;

#define PI      3.14159265

#define BUKA    0
#define TUTUP   1
#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

#define PULLEY_UP                       1
#define PULLEY_DOWN                     -1
#define SERVO_OFF                       3
#define PULLEY_STOP                     0

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
    
//LIDAR//
int e, Lidar_data;
//I2C///
int file, length, res;
int buff[2] = {0};
//////////TIMER////////////
static uint64_t epochMicro, epochMilli;
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
////////////ROTARY_ENCODER//////////////
int val1, val2;
long encoderPos1 = 0, encoderPos2 = 0;
int encoderLast1 = LOW, encoderLast2 = LOW;
int n1 = LOW, n2 = LOW;
double sudut1 = 0.0, sudut2 = 0.0, jumlahputar1 = 0.0, jumlahputar2 = 0.0, pTali1 = 0, pTali2 = 0;
/////////////////PID_CONTROL//////////////////////
float Reference, c_error = 0, c_integral = 0.0, c_derivative = 0.0, c_lasterror = 0.0, c_PID = 0, KP = 0.5, KI = 0.0, KD = 0;
///////////////
int status_pulley;
int status_servobawah = TUTUP;

void mosq_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
	/* Pring all log messages regardless of level. */
  switch(level){
    //case MOSQ_LOG_DEBUG:
    //case MOSQ_LOG_INFO:
    //case MOSQ_LOG_NOTICE:
    case MOSQ_LOG_WARNING:
    case MOSQ_LOG_ERR: {
      printf("%i:%s\n", level, str);
    }
  }
}

struct mosquitto *mosq = NULL;
char *topic = NULL;

void mqtt_setup(){

	char *host = (char*)"103.24.56.218";
	char *username = (char*)"pensbl";
	char *password = (char*)"1sampai0";
	int port = 1883;
	int keepalive = 60;
	bool clean_session = true;
	topic = (char*)"drone1/djidata";
    //topic = "drone/altitude";
  
  
  mosquitto_lib_init();
  mosq = mosquitto_new(NULL, clean_session, NULL);
  if(!mosq){
		fprintf(stderr, "Error: Out of memory.\n");
	}
  
  mosquitto_log_callback_set(mosq, mosq_log_callback);
  mosquitto_username_pw_set(mosq, username, password);
  if(mosquitto_connect(mosq, host, port, keepalive)){
		fprintf(stderr, "Unable to connect.\n");
	}
  int loop = mosquitto_loop_start(mosq);
  if(loop != MOSQ_ERR_SUCCESS){
    fprintf(stderr, "Unable to start loop: %i\n", loop);
  }
}

int mqtt_send(char *msg){
  return mosquitto_publish(mosq, NULL, topic, strlen(msg), msg, 0, 0);
}

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

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

void pulley_up(int moving_speed)
{    
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_MOVING_SPEED, moving_speed, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley = PULLEY_UP;
}

void pulley_down(int moving_speed)
{
    int i=0;
    moving_speed = moving_speed + 1024; 
       
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_MOVING_SPEED, moving_speed, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley = PULLEY_DOWN;
}

void pulley_stop()
{ 
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_MOVING_SPEED, 0, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley = PULLEY_STOP;
}

void servo_off()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    portHandler->closePort();
    status_pulley = SERVO_OFF;
}

void servobawahclose()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_GOAL_POSITION, 830, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i<10);
    status_servobawah = TUTUP;
}

void servobawahopen()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_GOAL_POSITION, 500, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i<10);
    status_servobawah = BUKA;
}

void I2c_init()
{
    char *bus = (char*)"/dev/i2c-1";
    file = open(bus, O_RDWR);
    
    if(file < 0)
    {
        printf("I2C GAGAL TERSAMBUNG");
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

unsigned int micros(void)
{
    uint64_t now;
#ifdef OLD_WAY
    struct timeval tv;
        gettimeofday(&tv, NULL);
        now = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;
#else
    struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
#endif

    return (uint32_t)(now - epochMicro) ;
}

static void initialiseEpoch (void){
  #ifdef OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
  epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
  #else
  struct timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    + (uint64_t)(ts.tv_nsec / 1000000L) ;
  epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec /    1000L) ;
  #endif
}

int PWMin(int pin){
	
    uint32_t lastMicros = micros();
    
    GPIOExport(pin);
    GPIODirection(pin, IN);
    do{
        if(micros()-lastMicros>20000) return 0;
    }while(GPIORead(pin));
    lastMicros = micros();
    
    do{
        if(micros()-lastMicros>20000) return 0;
    }while(!GPIORead(pin));
    lastMicros = micros();

    do{
        if(micros()-lastMicros>20000) return 0;
    }while(GPIORead(pin));
    
    return (int)micros()-lastMicros;
}

void lidarInit(int addr, int configuration){
    if(ioctl(file, I2C_SLAVE, addr) < 0)
    {
        printf("GAGAL TERSAMBUNG KE SLAVE");
    }
    switch(configuration){
    case 0: // Default mode, balanced performance
        write_data(0x02,0x80); // Default
        write_data(0x04,0x08); // Default
        write_data(0x1c,0x00); // Default
    break;
    
    case 1: // Short range, high speed
        write_data(0x02,0x1d);
        write_data(0x04,0x08); // Default
        write_data(0x1c,0x00); // Default
    break;
    
    case 2: // Default range, higher speed short range
        write_data(0x02,0x80); // Default
        write_data(0x04,0x00);
        write_data(0x1c,0x00); // Default
    break;

    case 3: // Maximum range
        write_data(0x02,0xff);
        write_data(0x04,0x08); // Default
        write_data(0x1c,0x00); // Default
    break;
    
    case 4: // High sensitivity detection, high erroneous measurements
        write_data(0x02,0x80); // Default
        write_data(0x04,0x08); // Default
        write_data(0x1c,0x80);
    break;

    case 5: // Low sensitivity detection, low erroneous measurements
        write_data(0x02,0x80); // Default
        write_data(0x04,0x08); // Default
        write_data(0x1c,0xb0);
    break;
    }
}

int LIDAR_LITE(bool biasCorrection){
    if(biasCorrection) {
        write_data(0x00,0x04);
    }
    else {
        write_data(0x00,0x03);
    }
    read_data(0x8f);            //membaca data dari alamat 0x8f
    buff[0] = res;
    read_data(0x10);             //membaca data dari alamat 0x10
    buff[1] = res;
    
    Lidar_data = (buff[0] << 8) + buff[1];      //data sensor mulai dari alamat 0x8f sampai 0x10 sebanyak masing masing 8bit sehingga perlu digabungkan kedua data tersebut
    //printf("data lidar : %d cm\n", Lidar_data);	
    return Lidar_data;
}

int timex = 0;
int sampling = 0;

void timez()
{
    while(1){
        if(status_pulley == PULLEY_DOWN){
            timex = micros();
            printf("status: %d time = %d\n", status_pulley, timex);
        }
        else{}
    }
}

void RemoteRead(){
    int coba =100;
    while(1){
        if(PWMin(11) >= 1900){
            if(status_pulley == PULLEY_DOWN || status_pulley == PULLEY_UP)
            {
                pulley_stop();
            }
            else{
                if(status_servobawah == TUTUP){
                    servobawahopen();
                    usleep(1000*1000);
                    pulley_down(1000);
                }
                else //if(pTali1 <= Lidar_data){
                {    pulley_down(1000);
                }
            }
            usleep(10*1000);
            while(PWMin(11) >= 1900){}
        }
        if(PWMin(10) >= 1900){
            if(status_pulley == PULLEY_UP || status_pulley == PULLEY_DOWN)
            {
                pulley_stop();
            }
            else if(status_servobawah == TUTUP){}
            else{ 
                pulley_up(1000);
            }
            usleep(10*1000);
            while(PWMin(10) >= 1900){}    
        }
        if(pTali1>=100){
            pulley_stop();
        }
    }
}

void bacaencod()
{
    //encoder1
	GPIOExport(23);
	GPIOExport(24);
	GPIODirection(23, IN);
	GPIODirection(24, IN);
    //encoder2
    GPIOExport(27);
	GPIOExport(22);
	GPIODirection(27, IN);
	GPIODirection(22, IN);
	while(1){
		n1 = GPIORead(23);
        n2 = GPIORead(27);
		if((encoderLast1 == LOW)&&(n1==HIGH)){
			if(GPIORead(24)==LOW){
				encoderPos1--;
			}else{
				encoderPos1++;
			}
			//printf("enc1 = %d\n",encoderPos);
		}
        if((encoderLast2 == LOW)&&(n2==HIGH)){
			if(GPIORead(22)==LOW){
				encoderPos2--;
			}else{
				encoderPos2++;
			}
			//printf("enc2 = %d\n",encoderPos2);
		}
		encoderLast1 = n1;
        sudut1 = (float)encoderPos1/100*360;
        pTali1 = fabs(1.0535*PI * (sudut1/360));
        
        encoderLast2 = n2;
        sudut2 = (float)encoderPos2/100*360;
        pTali2 = fabs(1.0535*PI * (sudut2/360));
        
	}
}

void read_servolidar()
{
    while(1)
    {    
        //read Lidar///
        LIDAR_LITE(0);
        
        //posisi servo bawah
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
          
        //read present position//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    
        //read present speed//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_PRESENT_SPEED, &dxl_present_speed, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    
        //read present load//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        if (dxl_present_load > 1023)
        {
            dxl_present_load = dxl_present_load - 1024;
        }
        load = ((float)dxl_present_load / 1023) * 100;
    
        //read present voltage//
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID_ATAS, ADDR_MX_PRESENT_VOLTAGE, &dxl_present_voltage, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        voltage = (float)dxl_present_voltage / 10;
    
        //printf("[ID:%03d] PresSpeed:%03d  PresLoad:%0.2f%  PresVol:%0.2f  PresPos:%03d\n", DXL_ID, dxl_present_speed, load, voltage, dxl_present_position);
        usleep(500*1000);
  }
}

float Dt = 0.01;
void PID()
{
    while(1){
        Reference = 100;
        c_error = (Reference - pTali1);
        c_integral = c_integral + (c_error * Dt);
        c_derivative = (c_error - c_lasterror) / Dt;
        c_lasterror = c_error;
        c_PID = (KP * c_error);// + (KD * c_derivative);// + (KI * c_integral) + (KD * c_derivative);
        //printf("reference = %g\tPID = %g\t\n", Reference, c_PID);
    }
}

void kontrol()
{
    pulley_down(200);
    do{
        c_error = Reference - pTali1;
    }
    while(c_error==0);
    pulley_stop();
}

float jarak = 0;
float kecepatan;
void baca()
{
    
    while(1){
        jarak = pTali1 - jarak;
        kecepatan = jarak/0.5;
        if(((kecepatan>0) && (kecepatan<0.1)) || ((kecepatan<0) && (kecepatan>-0.1))){kecepatan = 0;}
       
        printf("status: %d,%d,%d LIDAR:%d PresSpeed:%03d PresLoad:%0.2f% PresVol:%0.2f d1 = %0.2f d2 = %0.2f cm jarak : %0.2f kec: %g\n", 
        GPIORead(7),status_pulley, status_servobawah, Lidar_data, dxl_present_speed, load, voltage, pTali1, pTali2, jarak, kecepatan);
        
        jarak = pTali1;
        usleep(500*1000);
        system("clear");
    }
}

void limswitch()
{
 while(1){
    
    if(GPIORead(7) == 0 && status_pulley == PULLEY_UP){
        pulley_stop();
        usleep(1000*1000);
        servobawahclose();
        sleep(1);
        while(GPIORead(7)==0){}
    }
    usleep(1000);
 }
}

void mqt()
{    
    /*time_t now = time(0);
    char* dt = ctime(&now);
  
    char* datafile="raspilogfile1.txt";
    strcat(dt,datafile);
    FILE * pFile;
    //fclose(pFile);
    pFile = fopen (dt,"w");
    fprintf ( pFile,
            "Data_Lidar,"
            "Load,"
            "Voltage,"
            "Tali1, "
            "Tali2, "
            "Status_Pulley, "
            "Status_Snaplock,"
            "Altitude_box\n"
    );*/
    int snd = 0;
    char *buf = (char*)malloc(500);
    mqtt_setup();
    
    json_t *root = json_object();
    while(1)
    {    
        /*fprintf ( pFile, "%d,%0.2f,%0.2f,%0.2f,%0.2f,%d,%0.2f,%0.2f\n"
                   ,Lidar_data
                   ,load
                   ,voltage
                   ,pTali1
                   ,pTali2
                   ,status_pulley
                   ,status_servobawah
                   ,Lidar_data+5.5
        );*/
            
        json_object_set_new( root, "lidar", json_integer( Lidar_data ));
        json_object_set_new( root, "load_dxl", json_real( load ));
        json_object_set_new( root, "voltage", json_real( voltage ));
        json_object_set_new( root, "rotary1", json_real( pTali1 ));
        json_object_set_new( root, "rotary2", json_real( pTali2 ));
        json_object_set_new( root, "status_pulley", json_integer( status_pulley ));
        json_object_set_new( root, "status_snaplock", json_integer( status_servobawah ));
        json_object_set_new( root, "altitude_box", json_real( Lidar_data + 5.5 ));
        
        buf = json_dumps(root, 0);
        topic = (char*)"drone1/katroldata";
        snd = mqtt_send(buf);
        
        if(snd != 0) printf("mqtt_send error=%i\n", snd);
        else printf("data sending oke\n");
        sleep(1);
    }
    //fclose(pFile);
}

int main(int argc, char *argv[])
{    
    DynamixelInit();
    initialiseEpoch();
    I2c_init();
    lidarInit(0x62, 0);
    GPIOExport(7);
    GPIODirection(7, IN);
    Reference = 100;
    sleep(1);
    
    //thread timezz(timez);
    thread readlidarservo(read_servolidar);
    thread rotary(bacaencod);
    thread baca1(baca);
    thread remote(RemoteRead);
    thread limitsw(limswitch);
    thread kirimdata(mqt);
    
    //timezz.join();
    readlidarservo.join();
    rotary.join();
    baca1.join();
    remote.join();
    limitsw.join();
    kirimdata.join();
}
