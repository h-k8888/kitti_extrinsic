#include <iostream>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include "set_data.h"

//// imu <-- camera, camera in imu frame
//void transform_1_imu_cam(const M3D & r_imu_lidar, const V3D& t_imu_lidar, const M3D & r_lidar_cam, const V3D& t_lidar_cam,
//                       M3D & r_imu_cam, V3D & t_imu_cam)
//{
//    r_imu_cam = r_imu_lidar * r_lidar_cam;
//    t_imu_cam = t_imu_lidar + r_imu_lidar * t_lidar_cam;
//}

// imu <-- lidar, camera in imu frame
void transform_imu_lidar(const M3D & r_lidar_imu, const V3D& t_lidar_imu, M3D & r_imu_lidar, V3D & t_imu_lidar)
{
    r_imu_lidar = r_lidar_imu.transpose();
    t_imu_lidar = -r_imu_lidar * t_lidar_imu;
}

// imu <-- lidar <--- camera, camera in imu frame
void transform_1_imu_cam(const M3D & r_lidar_imu, const V3D& t_lidar_imu, const M3D & r_cam_lidar, const V3D& t_cam_lidar,
                         M3D & r_imu_cam, V3D & t_imu_cam)
{
    M3D r_imu_lidar = r_lidar_imu.transpose();
    V3D t_imu_lidar = -r_imu_lidar * t_lidar_imu;

    M3D r_lidar_cam = r_cam_lidar.transpose();
    V3D t_lidar_cam = -r_lidar_cam * t_cam_lidar;

    r_imu_cam = r_imu_lidar * r_lidar_cam;
    t_imu_cam = t_imu_lidar + r_imu_lidar * t_lidar_cam;
}

// inverse of camera <-- imu, ie, imu <-- camera, camera in imu frame
void transform_2_imu_cam(const M3D & r_lidar_imu, const V3D& t_lidar_imu, const M3D & r_cam_lidar, const V3D& t_cam_lidar,
                            M3D & r_imu_cam, V3D & t_imu_cam)
{
    M3D r_cam_imu = r_cam_lidar * r_lidar_imu;
    V3D t_cam_imu = t_cam_lidar + r_cam_lidar * t_lidar_imu;

    r_imu_cam = r_cam_imu.transpose();
    t_imu_cam = -r_imu_cam * t_cam_imu;
}

void test_transform_1_2()
{
    for (int i = 0; i < seq_size; ++i) {
        M3D & r_cam_lid = r_cam_lidar_input[i];
        V3D & t_cam_lid = t_cam_lidar_input[i];

        M3D & r_lid_imu = r_lidar_imu_input[i];
        V3D & t_lid_imu = t_lidar_imu_input[i];

        M3D r_imu_cam_1, r_imu_cam_2, r_imu_lidar;
        V3D t_imu_cam_1, t_imu_cam_2, t_imu_lidar;

        transform_imu_lidar(r_lid_imu, t_lid_imu, r_imu_lidar, t_imu_lidar);
        transform_1_imu_cam(r_lid_imu, t_lid_imu, r_cam_lid, t_cam_lid, r_imu_cam_1, t_imu_cam_1);
        transform_2_imu_cam(r_lid_imu, t_lid_imu, r_cam_lid, t_cam_lid, r_imu_cam_2, t_imu_cam_2);

        cout << "r_imu_cam_1\n" <<r_imu_cam_1 << endl;
        cout << "t_imu_cam_1\n" <<t_imu_cam_1.transpose() << endl;
        cout << "r_imu_cam_2\n" <<r_imu_cam_2 << endl;
        cout << "t_imu_cam_2\n" <<t_imu_cam_2.transpose() << endl;

        r_imu_lidar_output[i] = r_imu_lidar;
        t_imu_lidar_output[i] = t_imu_lidar;
        r_imu_cam_output[i] = r_imu_cam_1;
        t_imu_cam_output[i] = t_imu_cam_1;

        M3D & r_cam_lid_raw = r_cam_lidar_raw_input[i];
        V3D & t_cam_lid_raw = t_cam_lidar_raw_input[i];
        M3D r_imu_cam_raw_1, r_imu_cam_raw_2;
        V3D t_imu_cam_raw_1, t_imu_cam_raw_2;

        transform_1_imu_cam(r_lid_imu, t_lid_imu, r_cam_lid_raw, t_cam_lid_raw, r_imu_cam_raw_1, t_imu_cam_raw_1);
        transform_2_imu_cam(r_lid_imu, t_lid_imu, r_cam_lid_raw, t_cam_lid_raw, r_imu_cam_raw_2, t_imu_cam_raw_2);

        cout << "r_imu_cam_raw_1\n" <<r_imu_cam_raw_1 << endl;
        cout << "t_imu_cam_raw_1\n" <<t_imu_cam_raw_1.transpose() << endl;
        cout << "r_imu_cam_raw_2\n" <<r_imu_cam_raw_2 << endl;
        cout << "t_imu_cam_raw_2\n" <<t_imu_cam_raw_2.transpose() << endl;
    }
}

// lidar in imu frame
void saveIMULidarOutput()
{
    string file = "/home/autolab/kitti_extrinsic/imu_lidar.txt";
    ofstream of_file(file);
    if (of_file.is_open())
    {
        of_file.setf(ios::fixed, ios::floatfield);
        of_file.precision(20);
        for (int i = 0; i < seq_size; ++i) {
            M3D & r = r_imu_lidar_output[i];
            V3D & t = t_imu_lidar_output[i];
            of_file << "#seq " << i << endl;
            of_file << "extrinsic_T: [" <<
                    t(0,0) << ", " << t(1,0) << ", " <<t(2,0)  << "]\n";
            of_file << "extrinsic_R: [" <<
                    r(0,0) << ", " << r(0,1) << ", " << r(0,2) << ",\n" <<
                    r(1,0) << ", " << r(1,1) << ", " << r(1,2) << ",\n" <<
                    r(2,0) << ", " << r(2,1) << ", " << r(2,2) << "]\n";
        }
        of_file.close();
    }
}
// cam in imu frame
void saveIMUCamOutput()
{
    string file = "/home/autolab/kitti_extrinsic/imu_cam.txt";
//    string file = "imu_cam.txt";
    ofstream of_file(file);
    if (of_file.is_open())
    {
        of_file.setf(ios::fixed, ios::floatfield);
        of_file.precision(20);
        for (int i = 0; i < seq_size; ++i) {
            M3D & r = r_imu_cam_output[i];
            V3D & t = t_imu_cam_output[i];
            of_file << "#seq " << i << endl;
            of_file << "extrinsic_T: [" <<
                    t(0,0) << ", " << t(1,0) << ", " <<t(2,0)  << "]\n";
            of_file << "extrinsic_R: [" <<
            r(0,0) << ", " << r(0,1) << ", " << r(0,2) << ",\n" <<
            r(1,0) << ", " << r(1,1) << ", " << r(1,2) << ",\n" <<
            r(2,0) << ", " << r(2,1) << ", " << r(2,2) << "]\n";
        }
        of_file.close();
    }
}

int main() {
    std::cout << "Hello, KITTI!" << std::endl;

    set_cam_lidar_odom();
    set_lidar_imu_raw();
    set_cam_lidar_raw();
    test_transform_1_2();

    saveIMULidarOutput();
    saveIMUCamOutput();
    return 0;
}
