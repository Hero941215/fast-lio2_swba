#ifndef H_AV_MAP_H
#define H_AV_MAP_H

#include "MSEBAHelper.h"
#include "WindowFrame.h"

#include "hav/BAs.hpp"
#include "hav/toolss.hpp"

#include <set>
#include <unordered_map>
#include <deque>

class WindowFrame;

class H_AV_Map
{
public:
	H_AV_Map(int nAWNumber);

    // 返回指定容器中窗口帧的数量
    int GetFWWFNumber();
    int GetAWWFNumber();
    int GetWFNumber();

    // 维护所有的窗口帧
	void AddWindowFrame(WindowFrame *pWF);     // 添加到 mdpWindowFrames

    void AddAWWindowFrame(WindowFrame *pWF);
    void EraseOldestAWWindowFrame();           // 不删除指针，从 mdpActiveWindowFrames 移动到 mdpFixedWindowFrames

    // 获取活跃窗口窗口帧集
	std::deque<WindowFrame*> GetAWWindowFrames();
    std::deque<WindowFrame*> GetFWWindowFrames();

    WindowFrame* GetLatestAWWindowFrame(); // 最新的
	WindowFrame* GetOldestAWWindowFrame(); // 最老的

    // 维护哈希自适应体素地图的相关操作
    void CutVoxel(WindowFrame* pMF, int fnum);             // 测量插入到地图
    void RecutHAVMap(int win_count);                       // 自适应分割
    void MargHAVMap(int margi_size, int win_count);        // 需要活跃窗口帧位姿作为输入
    VOX_HESS BuildLocalBA(int win_count);                  // 从地图中找到需要的数据关联
    void GenerateInitPlaneLandmark(VOX_HESS &v_H);         // 基于位姿生成用于BA的初始平面路标

    pcl::PointCloud<PointType>::Ptr GetFixedPCMap();

public:
	int mnAWNumber;                                      // 活跃窗口尺寸
    bool mbBuildFixedPCMap;
    bool mbSaveFixedWF;                                  // 边缘化时，是否存储固定窗口帧

protected:
    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;  // 全局哈希自适应体素地图
    std::deque<WindowFrame*> mdpActiveWindowFrames;      // 活跃窗口中的窗口帧
    std::deque<WindowFrame*> mdpFixedWindowFrames;       // 离开活跃窗口后固定的窗口帧
    std::deque<WindowFrame*> mdpWindowFrames;            // 所有插入后端的窗口帧

    pcl::PointCloud<PointType>::Ptr mFixedPCMap;
};

#endif // H_AV_MAP_H