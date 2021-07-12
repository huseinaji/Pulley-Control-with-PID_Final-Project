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

#define DXL_ID_ATAS1                    1                  
#define DXL_ID_ATAS2					2
#define DXL_ID_BAWAH                    3                  

#define BAUDRATE                        200000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
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
float load_1 = 0;
float voltage_1 = 0;
float load_2 = 0;
float voltage_2 = 0;

float kecM1 = 0, kecM1a = 0, kecM2 = 0;

int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//int DXL_MOVING_SPEED = 0;
uint8_t dxl_error = 0;                          // Dynamixel error
uint16_t dxl_present_position = 0;              
uint16_t dxl_present_load = 0;               
uint8_t dxl_present_voltage = 0;               
uint16_t dxl_present_speed = 0;                
uint16_t dxl_present_position_1 = 0;             
uint16_t dxl_present_load_1 = 0;                  
uint8_t dxl_present_voltage_1 = 0;               
uint16_t dxl_present_speed_1 = 0;                 
uint16_t dxl_present_position_2 = 0;             
uint16_t dxl_present_load_2 = 0;                 
uint8_t dxl_present_voltage_2 = 0;              
uint16_t dxl_present_speed_2 = 0;                 

////////////ROTARY_ENCODER//////////////
int val1, val2;
long encoderPos1 = 0, encoderPos2 = 0;
int encoderLast1 = LOW, encoderLast2 = LOW;
int n1 = LOW, n2 = LOW;
double sudut1 = 0.0, sudut2 = 0.0, jumlahputar1 = 0.0, jumlahputar2 = 0.0, pTali1 = 0, pTali2 = 0;
/////////////////PID_CONTROL///////////////////////////
float Reference, c_error = 0, c_integral = 0.0, c_derivative = 0.0, c_lasterror = 0.0, c_PID = 0, KP = 4, KI = 0.01, KD = 0.5;
int kecMAX, kecMIN;
///////////////////////////////////////////////////////
int status_pulley1;
int status_pulley2;
int status_snaplock = TUTUP;
///////////////////////////////////////////////////
				
///////////////////////////////////////////////////
//////////////////////MQTT/////////////////////////
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

void mqt()
{    
    int snd = 0;
    char *buf = (char*)malloc(500);
    mqtt_setup();
    
    json_t *root = json_object();
    while(1)
    {            
        json_object_set_new( root, "lidar", json_integer( Lidar_data ));
        json_object_set_new( root, "load_dxl", json_real( load ));
        json_object_set_new( root, "voltage", json_real( voltage ));
        json_object_set_new( root, "rotary1", json_real( pTali1 ));
        json_object_set_new( root, "rotary2", json_real( pTali2 ));
        json_object_set_new( root, "status_pulley", json_integer( status_pulley1 ));
        json_object_set_new( root, "status_snaplock", json_integer( status_snaplock ));
        json_object_set_new( root, "altitude_box", json_real( Lidar_data + 5.5 ));
        
        buf = json_dumps(root, 0);
        topic = (char*)"drone1/katroldata";
        snd = mqtt_send(buf);
        
        if(snd != 0) printf("mqtt_send error=%i\n", snd);
        else printf("data sending oke\n");
        sleep(1);
    }
}
//////////////////////////////////////////////////
//////////////////////I2C/////////////////////////
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

///////////////////////////////////////////////////
////////////////////////GPIO///////////////////////
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

///////////////////////////////////////////////////////////////
///////////////////////////Dynamixel///////////////////////////
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

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
    
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

void pulley1_up(float moving_speed)
{    
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_MOVING_SPEED, moving_speed, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley1 = PULLEY_UP;
}

void pulley1_down(float moving_speed)
{
    int i=0;
    moving_speed = moving_speed + 1024; 
       
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_MOVING_SPEED, moving_speed, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley1 = PULLEY_DOWN;
}

void pulley1_stop()
{ 
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_MOVING_SPEED, 0, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley1 = PULLEY_STOP;
}

void servo1_off()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
    status_pulley1 = SERVO_OFF;
}

void pulley2_down(float moving_speed)
{    
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_MOVING_SPEED, moving_speed, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley2 = PULLEY_DOWN;
}

void pulley2_up(float moving_speed)
{
    int i=0;
    moving_speed = moving_speed + 1024; 
       
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_MOVING_SPEED, moving_speed, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley2 = PULLEY_UP;
}

void pulley2_stop()
{ 
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_MOVING_SPEED, 0, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i < 10);
    status_pulley2 = PULLEY_STOP;
}

void servo2_off()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
    status_pulley2 = SERVO_OFF;
}

void snaplockclose()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_GOAL_POSITION, 820, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i<10);
    status_snaplock = TUTUP;
}

void snaplockopen()
{
    int i=0;
    do{
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_GOAL_POSITION, 600, &dxl_error);
        if ((dxl_comm_result != COMM_SUCCESS) && (dxl_error != 0))
        {
            i++;
        }
        else if((dxl_comm_result == COMM_SUCCESS) && (dxl_error == 0))
        {
            i = 10;
        }
    }while(i<10);
    sleep(1);
    status_snaplock = BUKA;
}
/////////////////////////////////////////////////////
//////////////////////micros/////////////////////////
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
        write_data(0x02,0x80); 
        write_data(0x04,0x08); 
        write_data(0x1c,0x00); 
    break;
    
    case 1: // Short range, high speed
        write_data(0x02,0x1d);
        write_data(0x04,0x08); 
        write_data(0x1c,0x00); 
    break;
    
    case 2: // Default range, higher speed short range
        write_data(0x02,0x80);
        write_data(0x04,0x00);
        write_data(0x1c,0x00);
    break;

    case 3: // Maximum range
        write_data(0x02,0xff);
        write_data(0x04,0x08);
        write_data(0x1c,0x00); 
    break;
    
    case 4: // High sensitivity detection, high erroneous measurements
        write_data(0x02,0x80); 
        write_data(0x04,0x08); 
        write_data(0x1c,0x80);
    break;

    case 5: // Low sensitivity detection, low erroneous measurements
        write_data(0x02,0x80); 
        write_data(0x04,0x08); 
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
    
    Lidar_data = (buff[0] << 8) + buff[1];  
    Lidar_data = Lidar_data - 15;    
    return Lidar_data;
}

void bacaencod()
{
    //encoder2
	GPIOExport(23);
	GPIOExport(24);
	GPIODirection(23, IN);
	GPIODirection(24, IN);
    //encoder1
    GPIOExport(27);
	GPIOExport(22);
	GPIODirection(27, IN);
	GPIODirection(22, IN);
	while(1){
		n2 = GPIORead(23);
        n1 = GPIORead(27);
		if((encoderLast2 == LOW)&&(n2==HIGH)){
			if(GPIORead(24)==LOW){
				encoderPos2--;
			}else{
				encoderPos2++;
			}
			//printf("enc1 = %d\n",encoderPos);
		}
        if((encoderLast1 == LOW)&&(n1==HIGH)){
			if(GPIORead(22)==LOW){
				encoderPos1--;
			}else{
				encoderPos1++;
			}
			//printf("enc2 = %d\n",encoderPos2);
		}
		encoderLast1 = n1;
        sudut1 = (float)encoderPos1/100*360;
        //pTali1 = fabs(1.10*PI * (sudut1/360));
        pTali1 = fabs(1.19*PI * (sudut1/360));        
        encoderLast2 = n2;
        sudut2 = (float)encoderPos2/100*360;
        //pTali2 = fabs(2.105*PI * (sudut2/360));
        pTali2 = fabs(1.196*PI * (sudut2/360));
	}
}

void read_servolidar()
{
    while(1)
    {    
        //read Lidar///
        LIDAR_LITE(0);
        
        //posisi servo bawah
        /*dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_BAWAH, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
          
        //read present position//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_PRESENT_POSITION, &dxl_present_position_1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }*/
    
        //read present speed//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_PRESENT_SPEED, &dxl_present_speed_1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        if (dxl_present_speed_1 > 1023)
        {
            dxl_present_speed_1 = dxl_present_speed_1 - 1023;
        }
        
        //read present load//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_PRESENT_LOAD, &dxl_present_load_1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        if (dxl_present_load_1 > 1023)
        {
            dxl_present_load_1 = dxl_present_load_1 - 1023;
        }
        load_1 = ((float)dxl_present_load_1 / 1023) * 100;
    
        //read present voltage//
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID_ATAS1, ADDR_MX_PRESENT_VOLTAGE, &dxl_present_voltage_1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        voltage_1 = (float)dxl_present_voltage_1 / 10;
    
		//read present position//
        /*dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_PRESENT_POSITION, &dxl_present_position_2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }*/
    
        //read present speed//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_PRESENT_SPEED, &dxl_present_speed_2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        if(dxl_present_speed_2 > 1023)
        {
            dxl_present_speed_2 = dxl_present_speed_2 - 1023;
        }
        
        //read present load//
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_PRESENT_LOAD, &dxl_present_load_2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        if (dxl_present_load_2 > 1023)
        {
            dxl_present_load_2 = dxl_present_load_2 - 1023;
        }
        
        load_2 = ((float)dxl_present_load_2 / 1023) * 100;
    
        //read present voltage//
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID_ATAS2, ADDR_MX_PRESENT_VOLTAGE, &dxl_present_voltage_2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        voltage_2 = (float)dxl_present_voltage_2 / 10;
        usleep(20*1000);
  }
}
void PID_stop()
{
    c_error = 0;
    c_integral = 0;
    c_lasterror = 0;
    c_PID = 0;
    
    encoderPos1 = 0;
    encoderPos2 = 0;
}
int statuslim=1;
void limit2()
{
    while(1)
    {
    if(GPIORead(8)==0) 
    {
        usleep(200*1000);
        statuslim++;
        if(statuslim > 3){statuslim = 0;}
        PID_stop();
        if((statuslim==2)){ kecM1a=800;kecM1=750;kecM2=750; kecMAX = 1000; kecMIN = 00;}
        if((statuslim==0)){ kecM1a=800;kecM1=750;kecM2=750; kecMAX = 1000; kecMIN = 00;}
        while(GPIORead(8)==0){} 
    }
    usleep(10*1000);
    }
    
}

void limswitch()
{
    while(1)
    {
        if(GPIORead(7)==0 && statuslim != 2) 
        {
            statuslim = 1;
            usleep(500*1000);
            snaplockclose();
            while(GPIORead(7)==0){}
        }
        usleep(20*1000);
    }
}
float batas_tali = 100;
int SetDecreas = 50;
float DecPoint = 0;

void pPID()
{
	float Dt = 0.05;
	kecMAX = 800;
    kecMIN = 200;
	
	c_error = (pTali2 - pTali1);
                
    //if(c_error<=0.7 && c_error>=-0.7) c_error=0;
        
    c_integral = c_integral + (c_error * Dt);
    c_derivative = (c_error - c_lasterror) / Dt;
    c_lasterror = c_error;   
    c_PID = (KP * c_error) + (KD * c_derivative);// + (KD * c_derivative);
        
    //kecM1 = (kecM1 + (c_PID)) - (kecMAX - DecPoint);
    //kecM2 = (kecM2 - (c_PID)) - (kecMAX - DecPoint);
     
   // kecM1 = (kecM1 + (c_PID)) - (500 - DecPoint) ;
   // kecM2 = (kecM2 - (c_PID)) - (500 - DecPoint);
    
    //kecM1 = (kecM1 + (c_PID)) - (750 - DecPoint) ;
    //kecM2 = (kecM2 - (c_PID)) - (750 - DecPoint);
    if(pTali2 >= (Lidar_data - SetDecreas))
    {
        kecM1 = DecPoint + c_PID;
        kecM2 = DecPoint - c_PID;
    }
    else
    {
        kecM1 = (kecM1 + (c_PID));
        kecM2 = (kecM2 - (c_PID));
    }
    
    if(kecM1>=kecMAX) kecM1 = kecMAX; 
    if(kecM2>=kecMAX) kecM2 = kecMAX; 
    if(kecM1<=kecMIN) kecM1 = kecMIN; 
    if(kecM2<=kecMIN) kecM2 = kecMIN;
           
    if(kecMIN < 0) kecMIN = 0;
}

void PID()
{
    
    while(1){
        pPID();
        usleep(50*1000);
    }
}

void drive()
{

while(1){
    
    if(pTali2 >= (Lidar_data - SetDecreas))
        {
            
            DecPoint = ((750) * (Lidar_data - pTali2)) / SetDecreas;
            //DecPoint = 500;
             if(DecPoint <= 0 && (statuslim!=0)){
                DecPoint = 0;
                pulley1_stop();
                pulley2_stop();
                statuslim = 4;
            }
        }
        else
        {
            DecPoint = 750;
        }
        
        
    /*if((pTali1 >= Lidar_data) && (statuslim!= 0))
    {
        pulley1_stop();
        pulley2_stop();
        statuslim = 4;
    }*/
    
    if((statuslim==0))
    {    
        if(status_snaplock == TUTUP)
        {
            snaplockopen();
            usleep(500*1000);
            //printf("\n\t%d:%d \n",(int)kecM1,(int)kecM2);
            pulley1_up(kecM1);
            pulley2_up(kecM2);
        }
        else
        {   
            //printf("\n\t%d:%d \n",(int)kecM1,(int)kecM2);
            pulley1_up(kecM1);
            pulley2_up(kecM2);
        }
	}
	else if(statuslim == 1)
	{
        pulley1_stop();
		pulley2_stop();
    }
	else if(statuslim == 2)
	{
        if(status_snaplock == TUTUP)
        {
            snaplockopen();
            usleep(500*1000);
            //printf("\n\t%d:%d \n",(int)kecM1,(int)kecM2);
            pulley1_down(kecM1);
            pulley2_down(kecM2);
        }
        else
        {
            //printf("\n\t%d:%d \n",(int)kecM1,(int)kecM2);
            pulley1_down(kecM1);
            pulley2_down(kecM2);
        }
    }
    else if(statuslim == 3)
    {
        pulley1_stop();
        pulley2_stop();
    }
    usleep(20*1000);
 }
}

void baca()
{
    while(1){
        
        printf("PID:%0.2f:%0.4f:KecPoin%0.2f status:%d:%d:%d PresSpeed1:%03d PresSpeed2:%03d err:%0.2f kecM1:%0.2f kecM2:%0.2f tali1:%0.2f tali2:%0.2f load1:%d load2:%d\n", 
        KP,KD,DecPoint,Lidar_data, GPIORead(8), statuslim, dxl_present_speed_1, dxl_present_speed_2, c_error, kecM1, kecM2, pTali1, pTali2, dxl_present_load_1, dxl_present_load_2);
        //printf("%d\n", GPIORead(8));
        
        usleep(10*1000);
        system("clear");
    }
}

void filel()
{
    time_t now = time(0);
    char* dt = ctime(&now);
    char* datafile = "filedata.txt";
    strcat(dt,datafile);
    
    FILE *pFile;
    pFile = fopen (dt,"w");
    fprintf ( pFile,
            "P\t"
            "D\t"
            "Decreasing\t"
            "Tali1\t"
            "Tali2\t"
            "Error\t"
            "RPID\t"
            "kecM1\t"
            "kecM2\t"
            "VinServoCtrl\t"
            "VinServoReal\n"
    );
    
    while(1)
    {
        if((statuslim == 2) || (statuslim == 0))
        {
            fprintf(pFile,"%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%d\t%d\n", KP, KD, DecPoint, pTali1, pTali2, c_error, c_PID, kecM1, kecM2, dxl_present_speed_1, dxl_present_speed_2);
        }
        usleep(10*1000);
    }
        
}

int main(int argc, char *argv[])
{    
    DynamixelInit();
    initialiseEpoch();
    I2c_init();
    lidarInit(0x62, 0);
    GPIOExport(7);
    GPIOExport(8);
    GPIODirection(7, IN);
    GPIODirection(8, IN);

    sleep(1);
    
    thread readlidarservo(read_servolidar);
    thread rotary(bacaencod);
    thread limswasd(drive);
    thread limitsw(limswitch);
    thread lim2(limit2);
    thread PIDD(PID);
    //thread kirimdata(mqt);
    thread print(baca);
    //thread logfile(filel);
    
    readlidarservo.join();
    rotary.join();
    limswasd.join();
    limitsw.join();
    lim2.join();
    PIDD.join();
    //kirimdata.join();
    print.join();
    //logfile.join();
}
