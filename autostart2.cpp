#include <stdlib.h>
#include <unistd.h>

int main(){
	sleep(10);
	system("sudo route add default dev ppp0");
}
