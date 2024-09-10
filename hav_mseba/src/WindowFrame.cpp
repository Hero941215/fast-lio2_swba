#include "WindowFrame.h"

long unsigned int WindowFrame::nNextId = 0;

 // 时间戳 维护id -> mnId=nNextId++;
WindowFrame::WindowFrame(double dTimeStamp, Eigen::Isometry3d WFTwl): mdTimeStamp(dTimeStamp)
{
    mnId = nNextId++;
	mTwl = WFTwl;

    mPlaneSamplingPoints.reset(new pcl::PointCloud<PointType>());
}

void WindowFrame::SetWFPose(Eigen::Isometry3d WFTwl)
{
    mTwl = WFTwl;
}

Eigen::Isometry3d WindowFrame::GetWFPose()
{
    return mTwl;
}

// 存储采样特征点
void WindowFrame::SetPlaneSamplingPoint(pcl::PointCloud<PointType>::Ptr PlaneSamplingPoints)
{
    pcl::copyPointCloud(*PlaneSamplingPoints, *mPlaneSamplingPoints);
}

pcl::PointCloud<PointType>::Ptr WindowFrame::GetPlaneSamplingPoint()
{
    return mPlaneSamplingPoints;
}