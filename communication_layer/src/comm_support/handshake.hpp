#ifndef __HANDSHAKE__
#define __HANDSHAKE__

#include "serial/serial.h"

class handshake
{
private:
    serial::Serial sp;
public:
    handshake(/* args */);
    ~handshake();
    void handshake_start();
    void handshake_end();
};

#endif