#include "wiringPi.h"
#include "stdio.h"
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <thread>

using namespace std;

#define CAMERA_UP	0
#define CAMERA_DOWN	1

int status_pulley = 0;
int status_kamera;
int PWMin(int pin){
	
    uint32_t lastMicros = micros();
    
    pinMode (pin, INPUT);
    do{
        if(micros()-lastMicros>20000) return 0;
    }while(digitalRead(pin));
    lastMicros = micros();
    
    do{
        if(micros()-lastMicros>20000) return 0;
    }while(!digitalRead(pin));
    lastMicros = micros();

    do{
        if(micros()-lastMicros>20000) return 0;
    }while(digitalRead(pin));
    
    return (int)micros()-lastMicros;
}

void pwmInit()
{
	pinMode (12, PWM_OUTPUT);

	pwmSetMode (PWM_MODE_MS);
	pwmSetRange (2000);
	pwmSetClock (192);
}

void bacaremote()
{
	while(1){
	if(PWMin(9)>=1900){
		if(status_kamera == CAMERA_UP)
		{
			pwmWrite(12,150);
			status_kamera = CAMERA_DOWN;
			printf("kamera down\n");
		}
		else if(status_kamera == CAMERA_DOWN)
		{
			pwmWrite(12, 50);
			status_kamera = CAMERA_UP;
			printf("kamera up\n");
		}
		while(PWMin(9)>=1900){usleep(100*1000);}
	}
	}
}

int main()
{
	int i;
	wiringPiSetupGpio();
	pwmInit();

	thread kamera(bacaremote);
	
	kamera.join();	
	return 0;
}
