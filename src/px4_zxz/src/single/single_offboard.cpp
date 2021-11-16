/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>

static mavros_msgs::State current_state;
static ros::Publisher optitrack_pose_pub;
static int cmdd;
static geometry_msgs::PoseStamped aim;
static geometry_msgs::PoseStamped nmsg;
static double lyaw = 0;
static double lx   = 0;
static double ly   = 0;
static double lz   = 0;



void UdpListen(const uint16_t cport)
{
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_lis;
    int len;
    memset(&addr_lis, 0, sizeof(struct sockaddr_in));
    addr_lis.sin_family = AF_INET;
    addr_lis.sin_port = htons(cport);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
    len = sizeof(addr_lis);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
      perror("bind error:");
      exit(1);
    }


    int recv_num;
    char recv_buf[100];
    const char dot[2] = ",";
    struct sockaddr_in addr_client;

    while(ros::ok()){
        char *p;
        int ent=0;
        int cmd;
        double px,py,pz;

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

        if(recv_num < 0||abs(recv_num-19)>3)
        {
            ROS_ERROR("Recv Fail!");
            continue;
        }
        recv_buf[recv_num] = '\0';
        ROS_INFO("Rec: %s, len=%d",recv_buf,recv_num);

        p = strtok(recv_buf,dot);
        sscanf(p,"%d",&cmd);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&px);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&py);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&pz);
        ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmd,px,py,pz);

        cmdd=cmd;
        aim.pose.position.x=px;
        aim.pose.position.y=py;
        aim.pose.position.z=pz;


    }

}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pose_suber(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double qx,qy,qz,qw,siny,cosy;
  nmsg=*msg;
  optitrack_pose_pub.publish(nmsg);
  lx=nmsg.pose.position.x;
  ly=nmsg.pose.position.y;
  lz=nmsg.pose.position.z;
  qx=msg->pose.orientation.x;
  qy=msg->pose.orientation.y;
  qz=msg->pose.orientation.z;
  qw=msg->pose.orientation.w;

  siny = 2*(qw*qz+qx*qy);
  cosy = 1-2*(qy*qy+qz*qz);
  lyaw = std::atan2(siny,cosy);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;
    std::string vrpnt;
    nh.param<std::string>("single_offboard/vrpn_topic", vrpnt, "vrpn_client_node/UAV1/pose");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (vrpnt, 10, pose_suber);

    optitrack_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose",10);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x  = 0;
    vel.twist.linear.y  = 0;
    vel.twist.linear.z  = 0;
    vel.twist.angular.x = 0;
    vel.twist.angular.y = 0;
    vel.twist.angular.z = 0;
    aim.pose.position.x = 0;
    aim.pose.position.y = 9.5;
    aim.pose.position.z = 0.5;
    aim.pose.orientation.x=0;
    aim.pose.orientation.y=0;
    aim.pose.orientation.z=0;
    aim.pose.orientation.w=1;

    new std::thread(&UdpListen,12001);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd,disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(aim);
        //local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        if(cmdd!=4)local_pos_pub.publish(aim);
        if(cmdd==1){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
        }
        if(cmdd==2){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }else if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(cmdd==9){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                  ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }else if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
/*
            vel.twist.linear.x = 1*(aim.pose.position.x-lx);
            vel.twist.linear.y = 1*(aim.pose.position.y-ly);
            vel.twist.linear.z = 1*(aim.pose.position.z-lz);
            vel.twist.angular.z = -lyaw;
            ROS_INFO("pos_x:%.3lf, pos_y:%.3lf, pos_z:%.3lf, yaw:%.3lf",lx,ly,lz,lyaw);
      ROS_INFO("cmd_x:%.3lf, cmd_y:%.3lf, cmd_z:%.3lf, cmyaw:%.3lf",vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z,vel.twist.angular.z);
*/
            //local_pos_pub.publish(aim);
            //local_vel_pub.publish(vel);
        }
        if(cmdd==4){
          if( current_state.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(5.0))){
              if( set_mode_client.call(offb_set_mode) &&
                  offb_set_mode.response.mode_sent){
                  ROS_ERROR("Offboard enabled");
              }
              last_request = ros::Time::now();
          }else if( current_state.armed && nmsg.pose.position.z<0.4 &&
              (ros::Time::now() - last_request > ros::Duration(5.0))){
              if( arming_client.call(disarm_cmd) &&
                  disarm_cmd.response.success){
                  ROS_ERROR("Vehicle disarmed");
              }
              last_request = ros::Time::now();
          }
          aim.pose.position.z = 0.3;
          local_pos_pub.publish(aim);
          //vel.twist.linear.z  = -0.2;
          //local_vel_pub.publish(vel);

        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

