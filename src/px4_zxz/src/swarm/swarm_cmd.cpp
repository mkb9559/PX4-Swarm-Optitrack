/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>


static geometry_msgs::PoseStamped aim;
static bool isAir;
static int cmdd=0;

void UdpServer(const char* ip,const uint16_t cport,const int UAVID)
{
    ROS_ERROR("UDP %d",UAVID);
    std::string ss;
    geometry_msgs::PoseStamped offset;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }
    struct sockaddr_in addr_client;
    int len;
    memset(&addr_client, 0, sizeof(struct sockaddr_in));
    addr_client.sin_family = AF_INET;
    addr_client.sin_addr.s_addr = inet_addr(ip);
    addr_client.sin_port = htons(cport);
    len = sizeof(addr_client);


    switch(UAVID){
        case 1:{
            offset.pose.position.x = 0.3;
            offset.pose.position.y = 0.3;
            offset.pose.position.z = 0;
            break;
        }
        case 2:{
            offset.pose.position.x = 0.3;
            offset.pose.position.y = -0.3;
            offset.pose.position.z = 0;
            break;
        }
        case 3:{
            offset.pose.position.x = -0.3;
            offset.pose.position.y = -0.3;
            offset.pose.position.z = 0;
            break;
        }
        case 4:{
            offset.pose.position.x = -0.3;
            offset.pose.position.y = 0.3;
            offset.pose.position.z = 0;
            break;
        }
    }


    int sdlen;
    int send_num;
    char send_buf[100];
    char mcmd[10];
    char cmdx[15];
    char cmdy[15];
    char cmdz[15];

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        memset(&send_buf, 0, sizeof(send_buf));
        sdlen=0;

        sprintf(mcmd,"%d", cmdd>=9?9:cmdd);
        sprintf(cmdx,"%.3lf",aim.pose.position.x);
        sprintf(cmdy,"%.3lf",aim.pose.position.y);
        sprintf(cmdz,"%.3lf",aim.pose.position.z);

        sprintf(send_buf+sdlen,"%d",cmdd>=9?9:cmdd);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%.3lf",aim.pose.position.x+offset.pose.position.x);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%.3lf",aim.pose.position.y+offset.pose.position.y);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%.3lf",aim.pose.position.z+offset.pose.position.z);

        send_num = sendto(sock_fd, send_buf, ssize_t(strlen(send_buf)), 0, (struct sockaddr *)&addr_client, len);


        if(send_num < 0)
        {
            ROS_ERROR("Send Fail!, UAV = %d",UAVID);
            //perror("sendto error:");
            //exit(1);
        }
        ROS_INFO("Current Pub: %s",send_buf);
        //std::cout<<ssize_t(strlen(send_buf))<<std::endl;
        rate.sleep();
    }

}


void PrintInfo(){
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< std::endl;
    std::cout << "Input the flag:  "<< std::endl;
    std::cout << " 1 for Offboard"<< std::endl;
    std::cout << " 2 for Arm"<< std::endl;
    std::cout << " 4 for Land"<< std::endl;
    std::cout << " 9 for Static"<< std::endl;
    std::cout << "10 for Low speed in aim +x"<< std::endl;
    std::cout << "20 for Low speed in aim -x"<< std::endl;
    std::cout << "30 for Low speed in aim +y"<< std::endl;
    std::cout << "40 for Low speed in aim -y"<< std::endl;
    std::cout << "50 for Low speed in aim +z"<< std::endl;
    std::cout << "60 for Low speed in aim -z"<< std::endl;
    std::cout << "99 for End"<< std::endl;
    std::cin>> cmdd;
    switch(cmdd){
        case  1:ROS_ERROR("Offboard publish!");break;
        case  2:ROS_ERROR("Arm publish!");break;
        case  4:ROS_ERROR("Land publish!");break;

    }

}
void CmddPuber(){
    ros::Rate rate(20.0);
    while(ros::ok()){
        switch(cmdd){
        /*
            case  1:ROS_INFO("Offboard publish!");break;
            case  2:ROS_INFO("Arm publish!");break;
            case  4:ROS_INFO("Land publish!");break;
            */
            case  9:isAir=true;break;
            case 10:{
                if(isAir==true){
                    aim.pose.position.x = aim.pose.position.x + 0.01;
                }
                else ROS_ERROR("Enable and take off first");
                break;
            }
            case 20:{
                if(isAir==true){
                    aim.pose.position.x = aim.pose.position.x - 0.01;
                }
                else ROS_ERROR("Enable and take off first");
                break;
            }
            case 30:{
                if(isAir==true){
                    aim.pose.position.y = aim.pose.position.y + 0.01;
                }
                else ROS_ERROR("Enable and take off first");
                break;
            }
            case 40:{
                if(isAir==true){
                    aim.pose.position.y = aim.pose.position.y - 0.01;
                }
                else ROS_ERROR("Enable and take off first");
                break;
            }
            case 50:{
                if(isAir==true){
                    aim.pose.position.z = aim.pose.position.z + 0.005;
                }
                else ROS_ERROR("Enable and take off first");
                break;
            }
            case 60:{
                if(isAir==true){
                    aim.pose.position.z = aim.pose.position.z - 0.005;
                }
                else ROS_ERROR("Enable and take off first");
                break;
            }
            case 99: return ;
            default: continue;
        }
        rate.sleep();

    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "swarm_cmd");
    ros::NodeHandle nh;


    aim.pose.position.x = 0.4;
    aim.pose.position.y = 8.9;
    aim.pose.position.z = 0.6;

    aim.pose.orientation.x = 0;

    ros::Rate rate(20.0);
    new std::thread(&CmddPuber);
    //new std::thread(&UdpServer,"127.0.0.1",12001,1);

    new std::thread(&UdpServer,"192.168.1.11",12001,1);
    new std::thread(&UdpServer,"192.168.1.12",12001,2);
    new std::thread(&UdpServer,"192.168.1.13",12001,3);
    new std::thread(&UdpServer,"192.168.1.14",12001,4);

    while(ros::ok()){
        if(cmdd==99)break;
        else PrintInfo();
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}


