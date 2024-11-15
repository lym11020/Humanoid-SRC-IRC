#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"


FILE *imu_accel;
FILE *imu_gyro;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Main_node");
    ros::Time::init();
    ros::Rate loop_rate(100);
    ros::NodeHandle nh;

    // Execute the system command
    system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

    
    Dxl dxl;
    Trajectory trajectory;
    IK_Function IK_;

    Pick pick;

    Dxl_Controller dxl_ctrl(&dxl);
    Callback callback(&trajectory, &IK_, &dxl, &pick);
    // callback.Set_Callback();


    
    while (ros::ok())
    {
        callback.Write_All_Theta();
        dxl.SetThetaRef(callback.All_Theta);
        dxl.syncWriteTheta();

        

        ros::spinOnce();
        loop_rate.sleep();

        // About joint msg
        //         sensor_msgs::JointState msg;
        //         msg.header.stamp = ros::Time::now();
        // // , "j3", "j4", "j5", "j6", "j7", "j8", "j9", "j10", "j11", "j12"}
        //         std::vector<std::string> joint_name = {"j1", "j2", "j3"};

        //         for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        //         {
        //             msg.name.push_back(joint_name.at(i));
        //             // dxl.syncReadTheta();
        //             // msg.position.push_back(dxl.th_[i]);
        //         }
        //         joint_state_publisher_.publish(msg);

        // About FSR
        //  dxl.FSR_flag();
        //  dxl.syncWriteTheta();
        //  std::cout << callback.fsr_value << std::endl;
        

        // About IMU
        // dxl.Quaternino2RPY();
        
        // file write
        //  fprintf(imu_accel, "%d %.lf %.lf %.lf\n",t, callback.Accel(0),callback.Accel(1),callback.Accel(2));
        //  fprintf(imu_gyro, "%d %.lf %.lf %.lf\n",t, callback.Gyro(0),callback.Gyro(1),callback.Gyro(2));

    }

    ROS_INFO("Main_node!");

    dxl.~Dxl();
    return 0;
}



