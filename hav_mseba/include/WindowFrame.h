#ifndef WINDOWFRAME_H
#define WINDOWFRAME_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include "MSEBAHelper.h"


using namespace std;

class WindowFrame
{
public:
    // 时间戳 维护id -> mnId=nNextId++;
	WindowFrame(double dTimeStamp, Eigen::Isometry3d WFTwl);

	void SetWFPose(Eigen::Isometry3d WFTwl);
	Eigen::Isometry3d GetWFPose();

    // 存储采样特征点
	void SetPlaneSamplingPoint(pcl::PointCloud<PointType>::Ptr PlaneSamplingPoints);
    pcl::PointCloud<PointType>::Ptr GetPlaneSamplingPoint(); 

public:
	static long unsigned int nNextId; 
	long unsigned int mnId;            // 从1开始 
	double mdTimeStamp;                // 时间戳 

protected:
	Eigen::Isometry3d mTwl;            // 窗口帧对应的世界系位姿
    pcl::PointCloud<PointType>::Ptr mPlaneSamplingPoints;
    
};

#endif // WINDOWFRAME_H