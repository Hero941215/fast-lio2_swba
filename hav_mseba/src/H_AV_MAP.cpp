#include "H_AV_Map.h"

H_AV_Map::H_AV_Map(int nAWNumber): mnAWNumber(nAWNumber)
{
    mbBuildFixedPCMap = true;
    mbSaveFixedWF = true;
    mFixedPCMap.reset(new pcl::PointCloud<PointType>());
}

// 返回指定容器中窗口帧的数量
int H_AV_Map::GetFWWFNumber()
{
    return mdpFixedWindowFrames.size(); 
}

int H_AV_Map::GetAWWFNumber()
{
    return mdpActiveWindowFrames.size(); 
}

int H_AV_Map::GetWFNumber()
{
    return mdpWindowFrames.size(); 
}

// 维护所有的窗口帧
// 添加到 mdpWindowFrames
void H_AV_Map::AddWindowFrame(WindowFrame *pWF)
{
    mdpWindowFrames.push_back(pWF);
}  

void H_AV_Map::AddAWWindowFrame(WindowFrame *pWF)
{
    mdpActiveWindowFrames.push_back(pWF);
}

// 不删除指针，从 mdpActiveWindowFrames 移动到 mdpFixedWindowFrames
void H_AV_Map::EraseOldestAWWindowFrame()
{
    WindowFrame* pWF = mdpActiveWindowFrames.front();

    mdpFixedWindowFrames.push_back(pWF);
    mdpActiveWindowFrames.pop_front();
}          

// 获取活跃窗口窗口帧集
std::deque<WindowFrame*> H_AV_Map::GetAWWindowFrames()
{
    return mdpActiveWindowFrames;
}

std::deque<WindowFrame*> H_AV_Map::GetFWWindowFrames()
{
    return mdpFixedWindowFrames;
}

WindowFrame* H_AV_Map::GetLatestAWWindowFrame()
{
    return mdpActiveWindowFrames.back();
}

WindowFrame* H_AV_Map::GetOldestAWWindowFrame()
{
    return mdpActiveWindowFrames.front();
}

// 维护哈希自适应体素地图的相关操作
// 测量插入到地图
void H_AV_Map::CutVoxel(WindowFrame* pMF, int fnum)
{
    Eigen::Isometry3d WFPose = pMF->GetWFPose();

    IMUST pose;
    pose.R = WFPose.linear();
    pose.p = WFPose.translation();

    // 从运动帧获取对应的测量
    pcl::PointCloud<PointType>::Ptr PlaneSamplingPoints = pMF->GetPlaneSamplingPoint(); 

    // 调用 bavoxel 中的 cut_voxel 函数，将平面测量插入地图
    cut_voxel(surf_map, *PlaneSamplingPoints, pose, fnum);
}  


// 自适应分割
void H_AV_Map::RecutHAVMap(int win_count)
{
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
    {
        if(iter->second->is2opt)
        {
            iter->second->recut(win_count);
        }
    }
}

// 需要活跃窗口帧位姿作为输入
void H_AV_Map::MargHAVMap(int margi_size, int win_count)
{
    vector<IMUST> x_buf2;
    for(int i=0; i<mdpActiveWindowFrames.size(); i++) // 此时是滑动窗口中窗口帧最多的时候
    {
        Eigen::Isometry3d WFPose = mdpActiveWindowFrames[i]->GetWFPose();

        IMUST pose;
        pose.R = WFPose.linear();
        pose.p = WFPose.translation();
        x_buf2.push_back(pose);
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
    {
        if(iter->second->is2opt)
        {
            iter->second->marginalize(margi_size, x_buf2, win_count);  // 执行边缘化
        }
    }

    for(int i=0; i<margi_size; i++)
    {
        WindowFrame* pWF = mdpActiveWindowFrames.front();

        mdpFixedWindowFrames.push_back(pWF);
        mdpActiveWindowFrames.pop_front();

        if(mbBuildFixedPCMap)
        {
            // 获取窗口帧位姿，将测量转换到地图系
            Eigen::Isometry3d WFPose = pWF->GetWFPose();
            pcl::PointCloud<PointType>::Ptr SamplingPoints = pWF->GetPlaneSamplingPoint();

            pcl::PointCloud<PointType>::Ptr transPC(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*SamplingPoints, *transPC, WFPose.matrix());

            // 存储到容器
            *mFixedPCMap += *transPC;
        }

        if(mbSaveFixedWF)
		{
			Eigen::Isometry3d WFPose = pWF->GetWFPose();
			Eigen::Quaterniond qml; qml = WFPose.linear();
			Eigen::Vector3d tml = WFPose.translation();
			
			if(1) 
			{
				ofstream f;
				std::string res_dir = string(ROOT_DIR) + "result/be_fixed_pose.txt";
				f.open(res_dir, ios::app);
				f << fixed;
				f << setprecision(6) << pWF->mdTimeStamp << setprecision(7) << " " << tml.x() << " " << tml.y() << " " << tml.z() << " " <<
                              qml.x() << " " << qml.y() << " " << qml.z() << " " << qml.w() << std::endl;
				f.close();
			}
		}
    }

    
}      

// 从地图中找到需要的数据关联
VOX_HESS H_AV_Map::BuildLocalBA(int win_count)
{
    VOX_HESS voxhess;
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)  
        if(iter->second->is2opt) // 一个根节点，里面有窗口测量         
            iter->second->tras_opt(voxhess, win_count);
    
    return voxhess;
}  

void H_AV_Map::GenerateInitPlaneLandmark(VOX_HESS &v_H)
{
    vector<IMUST> xs;
    for(int i=0; i<mdpActiveWindowFrames.size(); i++)
    {
        Eigen::Isometry3d WFPose = mdpActiveWindowFrames[i]->GetWFPose(); // 扰动位姿

        IMUST pose;
        pose.R = WFPose.linear();
        pose.p = WFPose.translation();
        xs.push_back(pose);
    }

    v_H.CreateNoisePlaneLandmark(xs);
}

pcl::PointCloud<PointType>::Ptr H_AV_Map::GetFixedPCMap()
{
    return mFixedPCMap;
}
