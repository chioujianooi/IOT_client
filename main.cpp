#include "socket.h"
#include <iostream>
#include <string>

int main(){
        ClientSocket client;
    client.create();
    std::string ip="192.168.56.1";
    client.customConnect(8080, ip.c_str(), ip.length());
    client.sendData("Hello from client!", 18);
    char buffer[1024];
    client.receiveData(buffer, sizeof(buffer));
    std::cout<< "Received: " << buffer << std::endl;

    

    return 0;
}