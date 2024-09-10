#ifndef MSEBA_PCL_POINT_TYPE_H
#define MSEBA_PCL_POINT_TYPE_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>  // 基于icp实现配准

namespace velodyne_pcl
{
    struct PointTQXYZI 
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY;              // preferred way of adding a XYZ+padding
        double t;
        float  qx;
        float  qy;
        float  qz;
        float  qw;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointTQXYZI,
                                 (float,  x, x) (float,  y, y) (float,  z, z)
                                 (float,  intensity, intensity)
                                 (double, t,  t)
                                 (float,  qx, qx)
                                 (float,  qy, qy)
                                 (float,  qz, qz)
                                 (float,  qw, qw))

// define new type pointcloud with timestamp
// KITTI-raw & KITTI-360
namespace velodyne_pcl
{
    struct PointXYZT
    {
        PCL_ADD_POINT4D;                    // quad-word XYZ
        float    timestamp;                 ///< laser intensity reading
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
    } 
    EIGEN_ALIGN16;  
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, timestamp, timestamp))
                                
// KITTI-carla
namespace velodyne_pcl
{
    struct PointXYZCTIS
    {
        PCL_ADD_POINT4D;                    // quad-word XYZ
        float    cos_angle_lidar_surface;
        float    timestamp;
        std::uint32_t instance;
        std::uint32_t semantic;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
    } 
    EIGEN_ALIGN16; 
}


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZCTIS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, cos_angle_lidar_surface, cos_angle_lidar_surface)
                                  (float, timestamp, timestamp)
                                  (std::uint32_t, instance, instance)
                                  (std::uint32_t, semantic, semantic))

namespace velodyne_pcl
{
    struct PointXYZIRT
    {
        PCL_ADD_POINT4D;                    // quad-word XYZ
        float         intensity;            ///< laser intensity reading
        std::uint16_t ring;                 ///< laser ring number
        float         time;                 ///< laser time reading
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
    }
    EIGEN_ALIGN16;
}  // namespace velodyne_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

namespace ouster_pcl {
    struct PointXYZTRRRI
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
} // namespace ouster_pcl 

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_pcl::PointXYZTRRRI,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                    (std::uint32_t, t, t)
                                    (std::uint16_t, reflectivity, reflectivity)
                                    (std::uint8_t, ring, ring)
                                    (std::uint32_t, range, range))

// typedef pcl::PointXYZI PointType;
typedef velodyne_pcl::PointTQXYZI PointPose;
typedef pcl::PointCloud<PointPose> CloudPose;
typedef pcl::PointCloud<PointPose>::Ptr CloudPosePtr;


#endif // MSEBA_PCL_POINT_TYPE_H