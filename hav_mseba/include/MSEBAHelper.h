#ifndef MSEBA_HELPER_H
#define MSEBA_HELPER_H

#include <omp.h> 

#include "MSEBAPclPointType.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>
#include <list>
#include <mutex>
#include <memory>
#include <thread>
#include <unistd.h>
#include <iomanip>

#include "tsl/robin_map.h"   // 只有头文件，使用容易

using namespace std;

enum class SensorType
{ 
    VELODYNE,                                // 机械式激光雷达的扫描线数 NSCAN = 16, 32 ,64
    RSLIDAR,                                 // 机械式激光雷达的扫描线数 NSCAN = ??
    OUSTER,                                  // 机械式激光雷达的扫描线数 NSCAN = 16, 32 ,64
    DATASET,
    OTHER 
};

typedef pcl::PointXYZINormal PointType;
typedef std::vector<Eigen::Vector3d> PL_VEC;

struct M_POINT
{
  float xyz[3];
  int count = 0;
};

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

// 局部体素地图的哈希值
class Vector3iHash {
public:
	size_t operator()(const Eigen::Vector3i& x) const {
		size_t seed = 0;
		boost::hash_combine(seed, x[0]);
		boost::hash_combine(seed, x[1]);
		boost::hash_combine(seed, x[2]);
		return seed;
	}
};

// 降采样模块	
using GridVoxelSampler = tsl::robin_map<Eigen::Vector3i, PointType, Vector3iHash, std::equal_to<Eigen::Vector3i>, 
          Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, PointType>>>;

// // Hash value
// namespace std
// {
//   template<>
//   struct hash<VOXEL_LOC>
//   {
//     size_t operator() (const VOXEL_LOC &s) const
//     {
//       using std::size_t; using std::hash;
//       return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^ (hash<int64_t>()(s.z) << 1);
//     }
//   };
// }

// /// hash of vector
// template <int N>
// struct hash_vec {
//     inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
// };

// /// vec 2 hash
// /// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
// template <>
// inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
//     return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
// }

// /// vec 3 hash
// template <>
// inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
//     return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
// }

template <typename T>
Eigen::Matrix<T, 3, 3> skew_symmetric(Eigen::Matrix<T, 3, 1> v) {
  Eigen::Matrix<T, 3, 3> S;
  S << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
  return S;
}

// https://blog.csdn.net/qq_39400324/article/details/126955863
template <typename T>
double ComputeAngleBetweenVector(Eigen::Matrix<T, 3, 1> v1, Eigen::Matrix<T, 3, 1> v2)
{
    double cosValNew = v1.dot(v2) /(v1.norm()*v2.norm());
    return cosValNew;
}

// 消息发布函数
template <typename T>
void pub_func(T &pl, ros::Publisher &pub, const ros::Time &current_time)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "camera_init";
  output.header.stamp = current_time;
  pub.publish(output);
}

// a-loam
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

#endif // MSEBA_HELPER_H