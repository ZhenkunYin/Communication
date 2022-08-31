#include <serial/serial.h>
#include <ros/ros.h>
#include <iostream>

#define TEST 1

int main(int argc, char** argv){
    ros::init(argc,argv,"serial_port");
    ros::NodeHandle nh;

    /*serial port creation*/
    serial::Serial sp;
    serial::Timeout st = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyACM1");
    sp.setBaudrate(115200);
    sp.setTimeout(st);
#if TEST
    try
    {
        sp.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Serial port open failed");
        return -1;
    }

    if(sp.isOpen()){
        ROS_INFO_STREAM("Serial port is opened.");
    }else{
        return -1;
    }
#endif
    ros::Rate loop_rate(10);

    int option;
    while(1){  
        /**
         * read and send back
        size_t n = sp.available();

        if(n){      
            
            uint8_t buffer[1024];
            n = sp.read(buffer,n);

            for (int i = 0; i < n; i++)
            {
                std::cout << buffer[i];
            }
            std::cout << std::endl;
            sp.write(buffer,n);
        }
        */
        std::cout << "Chassis control menu:" << std::endl;
        std::cout << "1.setting speed (vx,vy,vz)" << std::endl;
        std::cout << "2.setting PID parameters for chassis" << std::endl;
        std::cout << "3.setting Gimbal angle" << std::endl;
        std::cout << "4.Exit" << std::endl;

        std::cin >> option;
        uint8_t buf[7]={0};
        switch (option)
        {
        case 1:
            std::cout << "Setting speed" << std::endl;
            std::cout << "In sequence vx, vy, vw"<< std::endl;
            int v[3];
            
            buf[0] = 0xAA;
            for (int i = 0; i < 3; i++)
            {
                std::cin >> v[i];
                while(v[i] > 300 || v[i] < -300)
                {
                    std::cout << "Speed too high, for safety, please reset again:";
                    std::cin >> v[i];
                }
                buf[i*2+1] = (v[i] >> 8) & 0xFF;
                buf[i*2+2] = v[i] & 0xFF;
            }
            sp.write(buf,7);
            break;
        case 2:
            std::cout << "PID Setting" << std::endl;
            std::cout << "In sequence p, i, d"<< std::endl;
            float pid[3];
            
            buf[0] = 0xBB;
            for (int i = 0; i < 3; i++)
            {
                std::cin >> pid[i];
                int integer = static_cast<int>(pid[i]);
                int decimal = static_cast<int>(1.0f/(pid[i]-static_cast<float>(integer)));
                decimal = (decimal > 0xFF)?0xFF:decimal;
                buf[i*2+1] = integer & 0xFF;
                buf[i*2+2] = decimal & 0xFF;
            }
            sp.write(buf,7);
            break;
        case 3:
            std::cout << "Gimbal angle Setting" << std::endl;
            float angle;
            
            buf[0] = 0xDD;
            std::cin >> angle;
            while (angle > 360.0f || angle < 0.0f)
            {
                std::cout << "Angle should be in 0-360 degrees" << std::endl;
                std::cin >> angle;
            }
            int output;
            output = static_cast<int>(8192.0f/360.0f*angle);
            buf[1] = (output >> 8) & 0xFF;
            buf[2] = output & 0xFF;
            sp.write(buf,3);
            break; 
        case 4:
            goto end;
        default:
            std::cout << "wrong number" << std::endl;
            break;
        }
        ROS_INFO("Command send");
        loop_rate.sleep();
    }
end:
    sp.close();

    return 0;
}