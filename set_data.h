//
// Created by autolab on 24-3-9.
//

#ifndef KITTI_EXTRINSIC_SET_DATA_H
#define KITTI_EXTRINSIC_SET_DATA_H

typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3d V3D;
using  namespace std;

int seq_size = 1; //kitti odometry sequence

// kitti dataset extrinsic
// "Tr" in file "data_odometry_calib", velodyne in the left camera frame
vector<M3D> r_cam_lidar_input(seq_size);
vector<V3D> t_cam_lidar_input(seq_size);

//  R, T in raw_data file "calib_imu_to_velo.txt", imu in lidar frame
vector<M3D> r_lidar_imu_input(seq_size);
vector<V3D> t_lidar_imu_input(seq_size);

//  R, T in raw_data file "calib_velo_to_cam.txt"
vector<M3D> r_cam_lidar_raw_input(seq_size);
vector<V3D> t_cam_lidar_raw_input(seq_size);

vector<M3D> r_imu_cam_output(seq_size);
vector<V3D> t_imu_cam_output(seq_size);


void set_cam_lidar_odom()
{
    Eigen::Matrix<double, 3, 4> Tr;
    Tr << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
            -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
            9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01;
    r_cam_lidar_input[0] = Tr.block<3,3>(0,0);
    t_cam_lidar_input[0] = Tr.block<3,1>(0,3);
}

void set_lidar_imu_raw()
{
    r_lidar_imu_input[0] << 9.999976e-01, 7.553071e-04, -2.035826e-03,
                            -7.854027e-04, 9.998898e-01, -1.482298e-02,
                            2.024406e-03, 1.482454e-02, 9.998881e-01;
    t_lidar_imu_input[0] << -8.086759e-01, 3.195559e-01, -7.997231e-01;
}

void set_cam_lidar_raw()
{
    r_cam_lidar_raw_input[0] << 7.967514e-03, -9.999679e-01, -8.462264e-04,
                                -2.771053e-03, 8.241710e-04, -9.999958e-01,
                                9.999644e-01, 7.969825e-03, -2.764397e-03;
    t_cam_lidar_raw_input[0] << -1.377769e-02, -5.542117e-02, -2.918589e-01;
}

#endif //KITTI_EXTRINSIC_SET_DATA_H
