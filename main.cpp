#include <iostream>
#include <vector>

#include <Eigen/Dense>

typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3d V3D;
using  namespace std;

int seq_size = 11; //kitti odometry sequence

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


//// imu <-- camera, camera in imu frame
//void transform_1_imu_cam(const M3D & r_imu_lidar, const V3D& t_imu_lidar, const M3D & r_lidar_cam, const V3D& t_lidar_cam,
//                       M3D & r_imu_cam, V3D & t_imu_cam)
//{
//    r_imu_cam = r_imu_lidar * r_lidar_cam;
//    t_imu_cam = t_imu_lidar + r_imu_lidar * t_lidar_cam;
//}

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

        M3D r_imu_cam_1, r_imu_cam_2;
        V3D t_imu_cam_1, t_imu_cam_2;

        transform_1_imu_cam(r_lid_imu, t_lid_imu, r_cam_lid, t_cam_lid, r_imu_cam_1, t_imu_cam_1);
        transform_2_imu_cam(r_lid_imu, t_lid_imu, r_cam_lid, t_cam_lid, r_imu_cam_2, t_imu_cam_2);

        cout << "r_imu_cam_1\n" <<r_imu_cam_1 << endl;
        cout << "t_imu_cam_1\n" <<t_imu_cam_1.transpose() << endl;
        cout << "r_imu_cam_2\n" <<r_imu_cam_2 << endl;
        cout << "t_imu_cam_2\n" <<t_imu_cam_2.transpose() << endl;

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

int main() {
    std::cout << "Hello, KITTI!" << std::endl;
    test_transform_1_2();



    return 0;
}
