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

//lidar//
int e, Lidar_data;
//i2c///
int file, length, res;
int buff[2] = {0};

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
    if(biasCorrection)  {
    // Take acquisition & correlation processing with receiver bias correction
        write_data(0x00,0x04);
    }
    else {
    // Take acquisition & correlation processing without receiver bias correction
        write_data(0x00,0x03);
    }
    read_data(0x8f);            //membaca data dari alamat 0x8f
    buff[0] = res;
    read_data(0x10);             //membaca data dari alamat 0x10
    buff[1] = res;
    
    Lidar_data = (buff[0] << 8) + buff[1];      //data sensor mulai dari alamat 0x8f sampai 0x10 sebanyak masing masing 8bit sehingga perlu digabungkan kedua data tersebut
    //printf("data sensor : %d\n", data);	
    return Lidar_data;
}

int main(){
	I2c_init();
    lidarInit(0x62, 0);
    while(1){
        printf("data sensor : %d\n", LIDAR_LITE(2));
    }
	return 0;
}
