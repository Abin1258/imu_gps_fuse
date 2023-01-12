#include <imu_gps_fuse/imu_gps_fuse.hpp>


int main(int argc, char** argv) {
    ros::init (argc, argv, "imu_gps_fuse");

    ros::NodeHandle nh;

    ImuGpsFuse *imu_gps_fuse = new ImuGpsFuse(nh);

    ros::spin();
    
    
    return 0;
}
