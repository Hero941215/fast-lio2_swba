#include <omp.h> 

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

#include <limits>
#include <string>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "tic_toc.h"
#include "stdlib.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

#include "MSEBAHelper.h"

#include "hav/BAs.hpp"
#include "hav/toolss.hpp"
#include "ceressolver/ceres_pa.h"

#include "H_AV_Map.h"
#include "WindowFrame.h"

using namespace std;
using namespace cv;

class Be_Hav_Mseba
{
public:
    Be_Hav_Mseba() // : mNh("~")
    {
        // 载入ros参数
        mNh.param<std::string>("hav_mseba/pointCloudTopic", pointCloudTopic, "/velodyne_points");
        mNh.param<std::string>("hav_mseba/odomPoseTopic", odomPoseTopic, "/lidar_odom_pose");
        std::string sensorStr;
        mNh.param<std::string>("hav_mseba/sensorType", sensorStr, "");
        if(sensorStr == "velodyne")
        {
            mSensor = SensorType::VELODYNE;
        }
        else if(sensorStr == "rslidar")
        {
            mSensor = SensorType::RSLIDAR;
        }
        else if(sensorStr == "ouster")
        {
            mSensor = SensorType::OUSTER;
        }
        else if(sensorStr == "dataset")
        {
            mSensor = SensorType::DATASET;
        }
        else if(sensorStr == "other")
        {
            mSensor = SensorType::OTHER;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be 'velodyne' 'rslidar' 'ouster' 'dataset' or 'other'): " << sensorStr);
            ros::shutdown();
        }
        mNh.param<std::string>("hav_mseba/beParamsPath", beParamsPath, "/home/mtcjyb/hav_mseba_ws/src/hav_mseba/config/be/Be.yaml");
        cv::FileStorage fSettings(beParamsPath, cv::FileStorage::READ);

        // 采样器初始化
        ds_size = fSettings["ds_size"]; // 对应稠密点云

        // 地图初始化
        layer_limit = fSettings["layer_limit"];
        win_size = fSettings["window_size"];
        voxel_size = fSettings["root_surf_voxel_size"];
        fix_size = fSettings["fix_size"];
        mpMap = new H_AV_Map(win_size);

        // 优化器参数
        opt_type = fSettings["opt_type"];
        opt_iterations = fSettings["opt_iterations"];

        if(opt_type==1)           // balm2
            ROS_INFO("\033[1;32m---->\033[0m Backend with BALM2.");
        else if(opt_type==1)      // ceres-pa
            ROS_INFO("\033[1;32m---->\033[0m Backend with CeresPA.");
        else
            ROS_INFO("\033[1;31m----> unknown optimizer type.\033[0m");
        
        // 初始化接收者和发布者
        mSubPointCloud = mNh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &Be_Hav_Mseba::laserCloudHandler, this);
        mSubOdom = mNh.subscribe<nav_msgs::Odometry>(odomPoseTopic, 1, &Be_Hav_Mseba::OdomHandler, this);

        mPubActivePCMap = mNh.advertise<sensor_msgs::PointCloud2>("/hav_mseba/active_pc_map", 1);          // 红色
        mPubFixedPCMap = mNh.advertise<sensor_msgs::PointCloud2>("/hav_mseba/fixed_pc_map", 1);            // 白色
        mPubFixedTrajectory = mNh.advertise<sensor_msgs::PointCloud2>("/hav_mseba/fixed_path", 1);         // 彩色
        mPubActiveTrajectory = mNh.advertise<sensor_msgs::PointCloud2>("/hav_mseba/active_path", 1);       // 绿色 
        mPubOdomAftMappedHighFrec = mNh.advertise<nav_msgs::Odometry>("/hav_mseba/aft_mapped_pose", 1);  // 高频矫正位姿

        initializationValue();
    }

    void initializationValue()
    {
        win_count = 0;

        timeNewLaserCloud = 0;
        timeNewOdomPose = 0;
        newLaserCloud = false;
        newOdomPose = false;

        mOusterRawPointCloud.reset(new pcl::PointCloud<ouster_pcl::PointXYZTRRRI>());
	    mVelodyneRawPointCloud.reset(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>());
        mRawPointCloud.reset(new pcl::PointCloud<PointType>());

        Tol.setIdentity();
		Tmo.setIdentity();
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        cloudHeader = laserCloudMsg->header;
        timeNewLaserCloud = laserCloudMsg->header.stamp.toSec();

        if(mSensor == SensorType::VELODYNE)
        {
            mVelodyneRawPointCloud->clear();
            pcl::fromROSMsg(*laserCloudMsg, *mVelodyneRawPointCloud);
        }
        else if(mSensor == SensorType::OUSTER)
        {
            mOusterRawPointCloud->clear();
            pcl::fromROSMsg(*laserCloudMsg, *mOusterRawPointCloud);
        }
        else if(mSensor == SensorType::OTHER)
        {
            mRawPointCloud->clear();
            pcl::fromROSMsg(*laserCloudMsg, *mRawPointCloud);
        }

        newLaserCloud = true;
    }

    void OdomHandler(const nav_msgs::Odometry::ConstPtr &msg)
    {
        timeNewOdomPose = msg->header.stamp.toSec();
        
        Eigen::Vector3d tmo(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
		Eigen::Quaterniond qmo(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

		Tol.setIdentity();
		Tol.rotate(qmo);
		Tol.pretranslate(tmo);

		Tml_predict = Tmo*Tol;
		Eigen::Quaterniond qml_predict;  qml_predict  = Tml_predict.linear();
		Eigen::Vector3d tml_predict = Tml_predict.translation();

		// 发布高频预测位姿 
		predictOdomPose.header.frame_id = "camera_init";
		predictOdomPose.child_frame_id = "laser";
		predictOdomPose.header.stamp = ros::Time().fromSec(timeNewOdomPose);
		predictOdomPose.pose.pose.position.x =  tml_predict.x();
		predictOdomPose.pose.pose.position.y = tml_predict.y();
		predictOdomPose.pose.pose.position.z = tml_predict.z();
		predictOdomPose.pose.pose.orientation.x = qml_predict.x();
		predictOdomPose.pose.pose.orientation.y = qml_predict.y();
		predictOdomPose.pose.pose.orientation.z = qml_predict.z();
		predictOdomPose.pose.pose.orientation.w = qml_predict.w();
		mPubOdomAftMappedHighFrec.publish(predictOdomPose);

        newOdomPose = true;
    }

    // ----------------------------------------- 具体操作 ------------------------------------------

    void DataPreprocess()
    {
        if(mSensor == SensorType::VELODYNE)
        {
            mRawPointCloud->clear();
            DataPreprocessWithVLP();
        }
        else if(mSensor == SensorType::OUSTER)
        {
            mRawPointCloud->clear();
            DataPreprocessWithOUSTER();
        }
        else if(mSensor == SensorType::OTHER)
        {
            return;
        }
    }

    void DataPreprocessWithVLP() 
    {
        PointType point;
        for(int i=0; i<mVelodyneRawPointCloud->points.size(); i++)
        {
            point.x = mVelodyneRawPointCloud->points[i].x;
            point.y = mVelodyneRawPointCloud->points[i].y;
            point.z = mVelodyneRawPointCloud->points[i].z;
            point.intensity = mVelodyneRawPointCloud->points[i].intensity;
            mRawPointCloud->push_back(point);
        }
    } 

    void DataPreprocessWithOUSTER()
    {
        PointType point;
        for(int i=0; i<mOusterRawPointCloud->points.size(); i++)
        {
            point.x = mOusterRawPointCloud->points[i].x;
            point.y = mOusterRawPointCloud->points[i].y;
            point.z = mOusterRawPointCloud->points[i].z;
            point.intensity = mOusterRawPointCloud->points[i].intensity;
            mRawPointCloud->push_back(point);
        }
    } 

    Eigen::Vector3i ComputeVoxelCoord(Eigen::Vector3d pl, double dDSSize)
    {
        double loc_xyz[3];
        for(int j=0; j<3; j++)
        {
            loc_xyz[j] = pl[j] / dDSSize;
            if(loc_xyz[j] < 0)
            {
                loc_xyz[j] -= 1.0;
            }
        }

        Eigen::Vector3i VoxelCoord((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        return VoxelCoord;
    }

    pcl::PointCloud<PointType>::Ptr GridDownSampling(pcl::PointCloud<PointType>::Ptr RawPC)
    {
        pcl::PointCloud<PointType>::Ptr DSPC(new pcl::PointCloud<PointType>());

        GridVoxelSampler sampler;
        for(int i=0; i<RawPC->points.size(); i++)
        {
            Eigen::Vector3d p3d(RawPC->points[i].x, RawPC->points[i].y, RawPC->points[i].z);
            Eigen::Vector3i vc = ComputeVoxelCoord(p3d, ds_size);
            if (sampler.find(vc) == sampler.end()) {
                sampler[vc] = RawPC->points[i];
            }
        }

        for(auto it = sampler.begin(); it != sampler.end(); ++it) 
            DSPC->push_back(it->second);

        return DSPC;
    }

    void UpdateHavMap(pcl::PointCloud<PointType>::Ptr PlanePC)
    {
        WindowFrame* pNewWF = new WindowFrame(timeNewLaserCloud, Tml_predict);
        pNewWF->SetPlaneSamplingPoint(PlanePC);

        // 更新体素地图
        mpMap->CutVoxel(pNewWF, win_count-1);

        // 插入地图中的滑动窗口
        mpMap->AddWindowFrame(pNewWF);
        mpMap->AddAWWindowFrame(pNewWF);
    }

    // 进行ceres-pa优化
    void MapRefineCeresPA()
    {
        VOX_HESS v_H = mpMap->BuildLocalBA(win_count);
		mpMap->GenerateInitPlaneLandmark(v_H);

        PLV(3) rot_params, pos_params, pla_params;  // 参数存储容器

        // vector<IMUST> xs;
		std::deque<WindowFrame*> dAWWFs = mpMap->GetAWWindowFrames();
		for(int i=0; i<dAWWFs.size(); i++)
		{
			Eigen::Isometry3d WFPose = dAWWFs[i]->GetWFPose(); 

			IMUST pose;
			pose.R = WFPose.linear();
			pose.p = WFPose.translation();
			rot_params.push_back(Log(pose.R));
			pos_params.push_back(pose.p);
			// xs.push_back(pose);
		}

        for(int j=0; j<v_H.plvec_voxels.size(); j++)
		{
			// 提取当前路标的噪声参数，并创建节点
			Eigen::Vector4d pi = v_H.plane_lm[j];
			Eigen::Vector3d normal; normal << pi(0), pi(1), pi(2); double di = pi(3); 
			
			if(std::fabs(di)<1e-2)
			{
				pla_params.push_back(Eigen::Vector3d::Zero());
				continue;
			}
		
			Eigen::Vector3d cp = di*normal; 
			pla_params.push_back(cp);
		}

        // 设置ceres参数块（特别是SO(3)）
		ceres::Problem problem;
		for(int i=0; i<dAWWFs.size(); i++)
		{
			ceres::LocalParameterization *parametrization = new pa::ParamSO3();
			problem.AddParameterBlock(rot_params[i].data(), 3, parametrization);
			problem.AddParameterBlock(pos_params[i].data(), 3);
			if(i==0) // 固定第一帧位姿
			{
				problem.SetParameterBlockConstant(rot_params[i].data());
				problem.SetParameterBlockConstant(pos_params[i].data());
			}
		}

        // 添加数据关联，只使用窗口内的
		for(int j=0; j<v_H.plvec_voxels.size(); j++) 
		{
			if(std::fabs(v_H.plane_lm[j](3))<1e-2)
				continue;
			
			const vector<PointCluster> &sig_orig = *v_H.plvec_voxels[j];
			// 添加集成测量边
			for(int k=0; k<win_count; k++)
			{
				if(sig_orig[k].N)
				{
					Eigen::Matrix4d im;
					im.block(0,0,3,3) = sig_orig[k].P;
					im.block(0,3,3,1) = sig_orig[k].v;
					im.block(3,0,1,3) = sig_orig[k].v.transpose();
					im(3,3) = sig_orig[k].N;

					// 进行矩阵分解
					Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> saes(im);
					Eigen::Vector4d evalue = saes.eigenvalues();
					Eigen::Matrix4d mleft = saes.eigenvectors();
					Eigen::Matrix4d mvalue; mvalue.setZero();
					for(int i=0; i<4; i++)
					{
						if(evalue[i] > 0)
							mvalue(i, i) = sqrt(evalue[i]);
					}
					Eigen::Matrix4d mat = (mleft * mvalue).transpose();

					pa::PACeresFactor *f = new pa::PACeresFactor(mat);
					problem.AddResidualBlock(f, NULL, rot_params[k].data(), pos_params[k].data(), pla_params[j].data());

				}
			}
		}

        ceres::Solver::Options options;  // 设置Schur顺序
		ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
		for(int i=0; i<dAWWFs.size(); i++)
		{
			ordering->AddElementToGroup(rot_params[i].data(), 1);
			ordering->AddElementToGroup(pos_params[i].data(), 1);
		}
		for(int i=0; i<pla_params.size(); i++)
		{
			ordering->AddElementToGroup(pla_params[i].data(), 0);
		}
		options.linear_solver_ordering.reset(ordering);

        // options.linear_solver_type = ceres::SPARSE_SCHUR;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.max_solver_time_in_seconds = 0.5;  // 最长求解时间0.2s
		options.max_num_iterations = opt_iterations;
        options.num_threads = 4;
		options.minimizer_progress_to_stdout = true;
		
		// options.function_tolerance = 1e-10;
		// options.parameter_tolerance = 1e-10;
        // options.use_inner_iterations = true;

        ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		// 存储优化后的位姿
		for(int i=0; i<dAWWFs.size(); i++)
		{
			Eigen::Quaterniond Estqwl; Estqwl = pa::Exp(rot_params[i]);
			Eigen::Vector3d Esttwl = pos_params[i];

			Eigen::Isometry3d EstT; EstT.setIdentity();
			EstT.rotate(Estqwl);
			EstT.pretranslate(Esttwl);
			dAWWFs[i]->SetWFPose(EstT);
		}

		// 维护矫正量
		Eigen::Isometry3d Tml_correct = dAWWFs.back()->GetWFPose();
		Tmo = Tml_correct * Tol.inverse();
    }

    // 进行balm2优化
    void MapRefineBALM2()
	{
		VOX_HESS v_H = mpMap->BuildLocalBA(win_count);

        std::cout << "mapvoxel num join ba: " << v_H.plvec_voxels.size() << std::endl;

		// 从地图中拿到当前的活跃窗口位姿
		vector<IMUST> x_buf;
		std::deque<WindowFrame*> dAWWFs = mpMap->GetAWWindowFrames();
		for(int j=0; j<dAWWFs.size(); j++)
		{
			Eigen::Isometry3d WFPose = dAWWFs[j]->GetWFPose();
			
			IMUST pose;
			pose.R = WFPose.linear(); 
			pose.p = WFPose.translation(); 
			x_buf.push_back(pose);
		}

		VOX_OPTIMIZER opt_lsv;
		Eigen::MatrixXd Rcov(6*win_count, 6*win_count); Rcov.setZero();
        int temp_win_size = win_size;
        win_size = win_count;
		opt_lsv.damping_iter(opt_iterations, x_buf, v_H, Rcov, 0);  // 不计算位姿协方差
		win_size = temp_win_size;

		// 存储优化结果	
		for(int j=0; j<dAWWFs.size(); j++)
		{
			Eigen::Quaterniond Estqwl; Estqwl = x_buf[j].R;
			Eigen::Vector3d Esttwl = x_buf[j].p;

			Eigen::Isometry3d EstT; EstT.setIdentity();
			EstT.rotate(Estqwl);
			EstT.pretranslate(Esttwl);
			dAWWFs[j]->SetWFPose(EstT);
		}
		
		// 维护矫正量
		Eigen::Isometry3d Tml_correct = dAWWFs.back()->GetWFPose();
		Tmo = Tml_correct * Tol.inverse();
	}

    // 处理主函数
    void Process()
    {
        // 数据对齐
        // 如果有新数据进来则执行，否则不执行任何操作
        if (newLaserCloud && newOdomPose && std::abs(timeNewLaserCloud - timeNewOdomPose) < 0.05){
            newLaserCloud = false;
            newOdomPose = false;
        }
        else{
            return;
        }

        TicToc t_be;

        // step1: 点云数据统一转换成 mRawPointCloud 
        DataPreprocess();

        // step2: 点云降采样
        TicToc t_extract;
        pcl::PointCloud<PointType>::Ptr DSPC = GridDownSampling(mRawPointCloud);
        std::cout << "extract feature points time: " << t_extract.toc() << " ms" << std::endl;

        // step3: 更新地图 (创建窗口关键帧，存储关键帧位姿、采样点云和时间戳)
        win_count++;
        TicToc t_update;
        UpdateHavMap(DSPC);
        std::cout << "update hav map time: " << t_update.toc() << " ms" << std::endl;

        if(win_count < win_size + fix_size) 
        {
            PublishROS();
            return;
        }  

        // step4: 地图自适应分割
        mpMap->RecutHAVMap(win_count);

        // step5: 地图细化
        if(opt_type==1)           // balm2
            MapRefineBALM2();
        else if(opt_type==2)      // ceres-pa
            MapRefineCeresPA();     
        else
            ROS_INFO("\033[1;31m----> unknown optimizer type.\033[0m");

        // step6: 边缘化
        mpMap->MargHAVMap(fix_size, win_count);

        win_count -= fix_size;

        PublishROS();

        std::cout << "be cost time: " << t_be.toc() << " ms" << std::endl;
    }

    void PublishROS()
    {
        PointType point;

        // 固定轨迹
        std::deque<WindowFrame*> FWWFs = mpMap->GetFWWindowFrames();
        pcl::PointCloud<PointType>::Ptr FixedPath(new pcl::PointCloud<PointType>());
        for(int i=0; i<FWWFs.size(); i++)
        {
            Eigen::Isometry3d Twl = FWWFs[i]->GetWFPose();
		    Eigen::Vector3d t = Twl.translation();
            point.x = t.x();
		    point.y = t.y();
		    point.z = t.z();
		    point.intensity = i;
            FixedPath->push_back(point);
        }

        sensor_msgs::PointCloud2 FixedPathRos;
        pcl::toROSMsg(*FixedPath, FixedPathRos);
        FixedPathRos.header.stamp = cloudHeader.stamp;
        FixedPathRos.header.frame_id = "camera_init";
        mPubFixedTrajectory.publish(FixedPathRos);
	    
        // 发布活跃轨迹
        std::deque<WindowFrame*> AWWFs = mpMap->GetAWWindowFrames();
        pcl::PointCloud<PointType>::Ptr ActivePath(new pcl::PointCloud<PointType>());
        for(int i=0; i<AWWFs.size(); i++)
        {
            Eigen::Isometry3d Twl = AWWFs[i]->GetWFPose();
		    Eigen::Vector3d t = Twl.translation();
            point.x = t.x();
		    point.y = t.y();
		    point.z = t.z();
		    point.intensity = i;
            ActivePath->push_back(point);
        }

        sensor_msgs::PointCloud2 ActivePathRos;
        pcl::toROSMsg(*ActivePath, ActivePathRos);
        ActivePathRos.header.stamp = cloudHeader.stamp;
        ActivePathRos.header.frame_id = "camera_init";
        mPubActiveTrajectory.publish(ActivePathRos);

        if(0)
        {
            // 发布固定点云
            pcl::PointCloud<PointType>::Ptr FixedPCMap = mpMap->GetFixedPCMap();

            sensor_msgs::PointCloud2 FixedPCMapRos;
            pcl::toROSMsg(*FixedPCMap, FixedPCMapRos);
            FixedPCMapRos.header.stamp = cloudHeader.stamp;
            FixedPCMapRos.header.frame_id = "camera_init";
            mPubFixedPCMap.publish(FixedPCMapRos);
        }

        pcl::PointCloud<PointType>::Ptr ActivePCMap(new pcl::PointCloud<PointType>());
        for(int i=0; i<AWWFs.size(); i++)
        {
            // 获取窗口帧位姿，将测量转换到地图系
            Eigen::Isometry3d WFPose = AWWFs[i]->GetWFPose();
            pcl::PointCloud<PointType>::Ptr SamplingPoints = AWWFs[i]->GetPlaneSamplingPoint();

            pcl::PointCloud<PointType>::Ptr transPC(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*SamplingPoints, *transPC, WFPose.matrix());

            *ActivePCMap += *transPC;
        }

        sensor_msgs::PointCloud2 ActivePCMapRos;
        pcl::toROSMsg(*ActivePCMap, ActivePCMapRos);
        ActivePCMapRos.header.stamp = cloudHeader.stamp;
        ActivePCMapRos.header.frame_id = "camera_init";
        mPubActivePCMap.publish(ActivePCMapRos);  // 活跃滑动窗口中的点云，需要先组织
    }


private:
	ros::NodeHandle mNh; 
    ros::Subscriber mSubPointCloud;  // 接收去畸变点云
    ros::Subscriber mSubOdom;        // 接受里程计位姿

    ros::Publisher mPubActivePCMap;
    ros::Publisher mPubFixedPCMap;
    ros::Publisher mPubFixedTrajectory;
    ros::Publisher mPubActiveTrajectory;
    
	ros::Publisher mPubOdomAftMappedHighFrec;  // 添加一个发布者，对最新到来的里程计进行预测并直接发出

    // 节点接收消息
    std_msgs::Header cloudHeader;
    nav_msgs::Odometry mOdomPose;
    pcl::PointCloud<ouster_pcl::PointXYZTRRRI>::Ptr mOusterRawPointCloud;
	pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr mVelodyneRawPointCloud;
    pcl::PointCloud<PointType>::Ptr mRawPointCloud;    // 把话题接收到的消息统一转换成 mRawPointCloud

    // 同步控制
    double timeNewLaserCloud;
    double timeNewOdomPose;
    bool newLaserCloud;
    bool newOdomPose;

    // 后端运行参数
    string pointCloudTopic;
    string odomPoseTopic;
    SensorType mSensor;              // 调用不同的预处理函数
    
    string beParamsPath;             // 后端参数配置文件 

    // hav 地图
    int win_count;
    double ds_size;                  // 点云grid降采样尺寸
    H_AV_Map* mpMap;

    // ba optimizer
    int opt_type;
    int opt_iterations;

    // like loam: 
    Eigen::Isometry3d Tmo;                   // 前端和后端估计的矫正量
	Eigen::Isometry3d Tol;                   // 前端里程计位姿
	Eigen::Isometry3d Tml_predict;           // 预测矫正的位姿用于初始化当前窗口帧
	nav_msgs::Odometry predictOdomPose;      // mOdomPose移除， mPredictOdomPose中的信息和 Tml_predict一样
};

int main(int argc, char **argv) 
{
    Eigen::setNbThreads(4);
    Eigen::initParallel();

    ros::init(argc, argv, "hav_mse");

    ROS_INFO("\033[1;32m---->\033[0m Hav_mseba backend Started.");

    Be_Hav_Mseba BHM;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        BHM.Process();

        rate.sleep();
    }
    
    ros::spin();

    return 0;
}
