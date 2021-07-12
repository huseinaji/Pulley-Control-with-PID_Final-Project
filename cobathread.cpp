#include <iostream>
#include <cstdlib>

#include <thread>
#include <time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

using namespace std;

void call_from_thread(){
	for(int i=0;i<10;i++){
		cout<<"thread1:"<<i<<endl;
		sleep(2);
	}
}
void thread2(){
	for(int z =0;z<10;z++){
		cout<<"thread2:"<<z<<endl;
		sleep(1);
	}
}

int main(){
	thread t1(call_from_thread);
	thread t2(thread2);
	
	t1.join();
	t2.join();
	return 0;
}
