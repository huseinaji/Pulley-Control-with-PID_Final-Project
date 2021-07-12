g++ -DLINUX -D_GNU_SOURCE -Wall -I../DynamixelSDK/c++/include/dynamixel_sdk/ Dynread_write.cpp -o readwrite -ldxl_x86_cpp


g++ -DLINUX -D_GNU_SOURCE -Wall -I../DynamixelSDK/c++/include/dynamixel_sdk/ program3.cpp -o program3 -ldxl_x86_cpp -pthread -lmosquitto -ljansson