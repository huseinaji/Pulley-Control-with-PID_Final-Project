#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>

int file, length, res;
int buff[2] = {0};

void I2c_set(int addr)
{
    char *bus = (char*)"/dev/i2c-1";
    file = open(bus, O_RDWR);
    
    if(file < 0)
    {
        printf("I2C GAGAL TERSAMBUNG");
    }
    
    if(ioctl(file, I2C_SLAVE, addr) < 0)
    {
        printf("GAGAL TERSAMBUNG KE SLAVE");
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

int main()
{
    int e, data;
    I2c_set(0x62);    //alamat I2C pada slave(Lidar-Lite)
    while(1){
        e = write_data(0x00,0x04);     //write data ke alamat 0x00 dengan nilai 0x04
        if(e < 0){return e;}
        e = read_data(0x8f);            //membaca data dari alamat 0x8f
        if(e < 0){return e;}
        else{
            buff[0] = res;
        }
    
        read_data(0x10);             //membaca data dari alamat 0x10
        if(e < 0){return e;}
        else{
            buff[1] = res;
        }
        data = (buff[0] << 8) + buff[1];      //data sensor mulai dari alamat 0x8f sampai 0x10 sebanyak masing masing 8bit sehingga perlu digabungkan kedua data tersebut
        printf("data sensor : %d\n", data);
    }    
}
