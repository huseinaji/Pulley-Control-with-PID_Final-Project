#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#define PWM_CONTROL 	0
#define	PWM_OUTPUT		2
#define PWM0_RANGE		4
#define PWM0_DATA		5
#define PWM1_RANGE		8
#define PWM1_DATA		9

#define FSEL_ALT0		0b100

#define	PWMCLK_CNTL		40
#define	PWMCLK_DIV		41

#define	PWM0_MS_MODE    0x0080
#define PWM0_ENABLE		0x0001

#define	PWM1_MS_MODE    0x8000
#define	PWM1_ENABLE     0x0100

#define	BCM_PASSWORD	0x5A000000

#define PI_GPIO_MASK	0xFFFFFFC0

static uint64_t epochMicro, epochMilli;

volatile unsigned int *_wiringPiGpio ;
volatile unsigned int *_wiringPiPwm ;
volatile unsigned int *_wiringPiClk ;

static volatile unsigned int *gpio =0;
static volatile unsigned int *pwm =0;
static volatile unsigned int *clk =0;
static volatile unsigned int piGpioBase = 0 ;

static volatile unsigned int GPIO_CLOCK_BASE ;
static volatile unsigned int GPIO_PWM ;
static volatile unsigned int GPIO_BASE ;

#define	BLOCK_SIZE		(4*1024)

static uint8_t gpioToGPFSEL [] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
} ;

static uint8_t gpioToShift [] =
{
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
} ;

int wiringPiDebug       = 0 ;

static void initialiseEpoch (void)
{
#ifdef	OLD_WAY
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

void pwmSetMode (int mode)
{
  if (mode == 0)
     *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE | PWM0_MS_MODE | PWM1_MS_MODE ;
  else
     *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE ;
}

void pwmSetRange (unsigned int range)
{
   *(pwm + PWM0_RANGE) = range ; usleep(10) ;
   *(pwm + PWM1_RANGE) = range ; usleep(10) ;
}

void pwmSetClock (int divisor)
{
  uint32_t pwm_control ;
  divisor &= 4095 ;

    if (wiringPiDebug)
      printf ("Setting to: %d. Current: 0x%08X\n", divisor, *(clk + PWMCLK_DIV)) ;

    pwm_control = *(pwm + PWM_CONTROL) ;		// preserve PWM_CONTROL

    *(pwm + PWM_CONTROL) = 0 ;				// Stop PWM

    *(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x01 ;	// Stop PWM Clock
      usleep (110) ;			// prevents clock going sloooow

    while ((*(clk + PWMCLK_CNTL) & 0x80) != 0)	// Wait for clock to be !BUSY
      usleep (1) ;

    *(clk + PWMCLK_DIV)  = BCM_PASSWORD | (divisor << 12) ;

    *(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x11 ;	// Start PWM clock
    *(pwm + PWM_CONTROL) = pwm_control ;		// restore PWM_CONTROL

    if (wiringPiDebug)
      printf ("Set     to: %d. Now    : 0x%08X\n", divisor, *(clk + PWMCLK_DIV)) ;
  
}

void pinMode (int pin, int mode)
{
	int fSel, shift, alt;
	
	fSel = gpioToGPFSEL [pin] ;
	shift = gpioToShift [pin] ;
	
	if(mode == PWM_OUTPUT)
	{
		if((alt = FSEL_ALT0) == 0) {return ;}
			
		//*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift) ;
		usleep(110);
		
		pwmSetMode (1) ;
		pwmSetRange (1024) ;
		pwmSetClock (32) ;
	}
}

void pwmWrite (int pin, int value)
{
	if((pin & PI_GPIO_MASK) == 0)
	{
		*(pwm + PWM0_DATA) = value ;
	}
	
}

void RegInit()
{
	int fd ;
	
	piGpioBase = 0x3F000000;
	
	GPIO_PWM = piGpioBase + 0x0020C000 ;
	GPIO_CLOCK_BASE = piGpioBase + 0x00101000 ;
	GPIO_BASE	  = piGpioBase + 0x00200000 ;
	
	gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE) ;
	pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PWM) ;	
	clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_CLOCK_BASE) ;
	
	_wiringPiGpio  = gpio ;
	_wiringPiPwm   = pwm ;
	_wiringPiClk   = clk ;
	
	 initialiseEpoch();
}

int main()
{
	RegInit();
	//pwmSetMode(0);
	printf("%d\n",*pwm);
	//pinMode(12, PWM_OUTPUT) ;
	//pwmWrite(12, 200) ;
}
