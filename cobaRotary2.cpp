#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <thread>

using namespace std;

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

#define PIN  5
#define POUT 6

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

int val, val2;
long encoderPos = 0, encoderPos2 = 0;
int encoderLast = LOW, encoderLast2 = LOW;
int n = LOW, n2 = LOW;

void bacaEncod1()
{
	GPIOExport(23);
	GPIOExport(24);
	GPIODirection(23, IN);
	GPIODirection(24, IN);
	while(1){
		n = GPIORead(23);
		if((encoderLast == LOW)&&(n==HIGH)){
			if(GPIORead(24)==LOW){
				encoderPos--;
			}else{
				encoderPos++;
			}
			//printf("enc1 = %d\n",encoderPos);
		}
		encoderLast = n;
	}
}

void bacaEncod2()
{
	GPIOExport(27);
	GPIOExport(22);
	GPIODirection(27, IN);
	GPIODirection(22, IN);
	while(1){
		n2 = GPIORead(27);
		if((encoderLast2 == LOW)&&(n2==HIGH)){
			if(GPIORead(22)==LOW){
				encoderPos2--;
			}else{
				encoderPos2++;
			}
			//printf("enc2 = %d\n",encoderPos2);
		}
		encoderLast2 = n2;
	}
}

void print(){
	double sudut = 0.0;
	float jmlRot = 0.0;
	while(1){
		sudut = (float)encoderPos2/100*360;
		jmlRot = sudut/360;
		//printf("enc2 = %d\tenc1 = %d\n",encoderPos2, encoderPos);
		printf("encoder = %d\tsudut = %g\tjmlputar = %g\n", encoderPos2, sudut, jmlRot);
		usleep(10*1000);
	}
}

int main()
{
	thread t1(bacaEncod1);
	thread t2(bacaEncod2);
	thread t3(print);
	t1.join();
	t2.join();
	t3.join();
	return 0;
}
