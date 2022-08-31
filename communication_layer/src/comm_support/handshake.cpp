#include "handshake.hpp"

handshake::handshake(serial::Serial sp)
{
    this->sp = sp;
}

handshake::~handshake()
{
}

/**
 * @brief handshake start
 */
void handshake::handshake_start()
{
    uint8_t state = 0x00;
    while (1)
    {
        if (state == 0x00)
        {
            sp.write(&handshake_data,1);
            size_t n = sp.available();
            if(n)
            {
                uint8_t buffer;
                n = sp.read(&buffer,1);
                if (buffer == 0xFF)
                {
                    sp.write(&communication_freq,1);
                    return;
                }
                Sleep(50);
            }
        }
    }
}

/**
 * @brief handshake end signal
 * 
 */
void handshake::handshake_end()
{
    uint8_t state = 0x00;
    while (1)
    {
        if (state == 0x00)
        {
            sp.write(&handshake_data,1);
            size_t n = sp.available();
            if(n)
            {
                uint8_t buffer;
                n = sp.read(&buffer,1);
                if (buffer == 0xFF)
                {
                    return;
                }
                Sleep(50);
            }
        }
    }
}