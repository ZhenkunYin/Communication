#include "odom_from_stm32.hpp"
#include "comm_support/handshake.hpp"

#define TEST 0

static const uint8_t handshake_data = 0xFF;
static const uint8_t communication_freq = 250;

odom_publisher::odom_publisher() : vx(0),vy(0),vth(0)
{
    ROS_INFO("Odom class init");
    odom2base_data.header.frame_id = "odom";
    odom2base_data.child_frame_id = "base_link";
    odom_data.child_frame_id = "base_link";
    odom_data.header.frame_id = "odom";
    odom_Current = ros::Time::now();
    odom_last = ros::Time::now();
}

odom_publisher::~odom_publisher()
{
}

/**
 * @brief update odom info according to new velocity
 */
void odom_publisher::update_odom_info(uint8_t *buf, int num)
{

}

/**
 * @brief 
 * 
 */
void odom_publisher::update_gimbal_info(uint8_t *buf, int num)
{

}

/**
 * @brief Thread for updating odometry data for chasis
 * 
 * @param op odometry class
 */
void update_odom_info(odom_publisher *op)
{
#if TEST
    /*serial communication for receiving odom data*/
    serial::Timeout st = serial::Timeout::simpleTimeout(100);
    op->sp.setPort("/dev/ttyACM0");
    op->sp.setBaudrate(115200);
    op->sp.setTimeout(st);

    try
    {
        odom->sp.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Serial port open failed");
        return -1;
    }

    if(odom->sp.isOpen()){
        ROS_INFO_STREAM("Serial port is opened.");
    }else{
        return -1;
    }

    handshake *hs = new handshake(op->sp);
    hs->handshake_start();
#endif

    while (op->state)
    {
        size_t n = op->sp.available();
#if TEST
        if(n)
        {
            uint8_t buffer[1024];
            n = op->sp.read(buffer,n);
            switch (buffer[0])
            {
            case 0xCC:          //odom data
                op->update_odom_info(buffer,n);
                break;
            case 0xDD:          //gimbal data
                op->update_gimbal_info(buffer,n);
                break;
            default:
                break;
            }
            op->sp.write(buffer,n);
        }
#endif
        sleep(1);
        std::cout << "thread" << std::endl;
    }
#if TEST
    hs->handshake_end();
#endif
    return;
}

nav_msgs::Odometry odom_publisher::get_odom_data()
{
    odom_data.header.stamp = ros::Time::now();
    odom_data.pose.pose.position.x = x;
    odom_data.pose.pose.position.y = y;
    odom_data.pose.pose.position.z = 0.0;
    odom_data.pose.pose.orientation.w = q.getW();
    odom_data.pose.pose.orientation.x = q.getX();
    odom_data.pose.pose.orientation.y = q.getY();
    odom_data.pose.pose.orientation.z = q.getZ();

    odom_data.twist.twist.linear.x = vx;
    odom_data.twist.twist.linear.y = vy;
    odom_data.twist.twist.angular.z = vth;


    return odom_data;
}

geometry_msgs::TransformStamped odom_publisher::get_odom2base_data()
{
    odom2base_data.header.stamp = ros::Time::now();
    odom2base_data.transform.translation.x = x;
    odom2base_data.transform.translation.y = y;
    odom2base_data.transform.translation.z = 0;
    odom2base_data.transform.rotation.w = q.getW();
    odom2base_data.transform.rotation.x = q.getX();
    odom2base_data.transform.rotation.y = q.getY();
    odom2base_data.transform.rotation.z = q.getZ();

    return odom2base_data;
}

int main(int argc, char **argv)
{
    /*ros init*/
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle n;

    /*odom class*/
    odom_publisher *odom = new odom_publisher();
    
    /*odom topic advertise*/
    ros::Publisher odom_pub;
    odom_pub = n.advertise<nav_msgs::Odometry>("/rtabmap/odom", 10);

    /*odom->base_link*/
    tf2_ros::TransformBroadcaster odom_tf;

    /*Thread for serial data update*/
    odom->state = true;
    std::thread t1(update_odom_info,odom);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std::cout << "main" << std::endl;
        /* code for loop body */
        ros::spinOnce();
        odom_pub.publish(odom->get_odom_data());
        odom_tf.sendTransform(odom->get_odom2base_data());
        loop_rate.sleep();
    }

    odom->state = false;
    t1.join();
    ROS_INFO("odom_drom_stm32 DONE");

    return 0;
}