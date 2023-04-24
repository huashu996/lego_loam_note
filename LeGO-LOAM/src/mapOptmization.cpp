// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.
//mapOptmization.cpp进行的内容主要是地图优化，将得到的局部地图信息融合到全局地图中去。
#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

class mapOptimization{

private:
	// 因子图优化相关变量
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;
	// ROS相关变量定义
    ros::NodeHandle nh;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubtestcloud;
    ros::Publisher pubLocalmapDS;
    ros::Publisher pubLocalmap;
    

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRegisteredCloud;

    ros::Subscriber subLaserCloudCornerLast;
    ros::Subscriber subLaserCloudSurfLast;
    ros::Subscriber subOutlierCloudLast;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subImu;
    // 为了信息发布而定义的msg
    nav_msgs::Odometry odomAftMapped;
    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 关键帧的角点点云集合
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 关键帧的面点点云集合
    vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames; // 关键帧的离群点云集合

    deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
    int latestFrameID;

    vector<int> surroundingExistingKeyPosesID;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;
    
    PointType previousRobotPosPoint; //上一帧关键帧位姿点
    PointType currentRobotPosPoint; //使用经过 scan-to-model 优化的位姿创建当前位置点

	// 所有关键帧三自由度位置集合 (x,y,z表示位置,i表示位姿点在集合中的索引)（这是经过因子图优化后的位置）
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    //关键帧六自由度位姿集合 (x,y,z表示位置,i表示索引,rpy表示姿态,time记录里程计时间)（这是经过因子图优化后的位置）
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    

    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses; //局部关键帧位置集合——目标点附近的位置点集合
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;  // 上者的降采样结果

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS; // corner feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS; // downsampled corner featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;// 局部角点点云地图  (每轮进行清空)
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMaptest;// 局部角点点云地图  (每轮进行清空)
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;// 局部平面点点云地图  (每轮进行清空)
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;// 局部角点点云的降采样  (每轮进行清空)
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;// 局部角点点云的降采样  (每轮进行清空)

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap; // 局部角点点云地图 的KD树
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;// 局部平面点点云地图 的KD树


    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;//所有关键帧位置点集合的KD树
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    
    pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
    pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

    pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterOutlier;
    pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames; // for histor key frames of loop closure
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
	// 信息的时间戳
    double timeLaserCloudCornerLast; // Less角点的时间
    double timeLaserCloudSurfLast; // Less平面点的时间
    double timeLaserOdometry;  // 里程计时间
    double timeLaserCloudOutlierLast; // 离群点点云 的时间
    double timeLastGloalMapPublish;

    // 信息接收的标志位
    bool newLaserCloudCornerLast; //Less角点的标志位
    bool newLaserCloudSurfLast;//Less平面点的标志位
    bool newLaserOdometry;//里程计信息的标志位
    bool newLaserCloudOutlierLast;//离群点点云的标志位

    float transformLast[6]; // 上一关键帧 经过因子图优化后的位姿，用于添加因子图节点。
    float transformSum[6]; // 里程计的六自由度位姿(p y r x y z)
    float transformIncre[6]; // 两次位姿优化时，里程计的相对位姿变化，
    float transformTobeMapped[6];// 机器人的位姿,全局姿态优化的目标变量：位姿优化的初始值->在地图优化过程中得到更新->经过因子图优化后的位姿
    float transformBefMapped[6];//  未经过scan-to-model优化的位姿，即里程计位姿。
    float transformAftMapped[6];//  由经过scan-to-model优化后的位姿赋值,并在因子图优化后进行修正。



    int imuPointerFront; //与里程计时间对准的IMU数据索引
    int imuPointerLast; //最新的IMU数据索引

    // IMU时间、数据队列
    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];

    std::mutex mtx;

    double timeLastProcessing;  // 上一次进行全局位姿优化的时间

    PointType pointOri, pointSel, pointProj, coeff;

    cv::Mat matA0; //存放距离平面点最近的五个点(5×3矩阵)
    cv::Mat matB0;//5*1 的矩阵
    cv::Mat matX0;//3*1 的矩阵

    cv::Mat matA1;//3×3 的协方差矩阵
    cv::Mat matD1;//1*3 的特征值向量
    cv::Mat matV1;// 3个 特征向量

    bool isDegenerate;
    cv::Mat matP;
	// 一些点云的数量
    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum;
    int laserCloudSurfLastDSNum;
    int laserCloudOutlierLastDSNum;
    int laserCloudSurfTotalLastDSNum;

    bool potentialLoopFlag;
    double timeSaveFirstCurrentScanForLoopClosure;
    int closestHistoryFrameID;
    int latestFrameIDLoopCloure;

    bool aLoopIsClosed;

    float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
    float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

public:

    

    mapOptimization():
        nh("~")
    {
    	//因子图初始化
    	ISAM2Params parameters;
		parameters.relinearizeThreshold = 0.01;
		parameters.relinearizeSkip = 1;
    	isam = new ISAM2(parameters);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);//机器人关键帧在全局坐标系下的位置信息，轨迹
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);//机器人周围激光雷达点云数据
        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);//经过位姿图优化和点云配准后的里程计信息
        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);//机器人历史轨迹的点云数据
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);//经过ICP配准后的机器人激光雷达点云数据
        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);//机器人最近获取的点云数据
        pubRegisteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 2);//经过位姿图优化和点云配准后的机器人点云数据用于建图
        pubtestcloud = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud", 2);
        pubLocalmap= nh.advertise<sensor_msgs::PointCloud2>("/localmap", 2);
        pubLocalmapDS= nh.advertise<sensor_msgs::PointCloud2>("/localmapDS", 2);

		//接收相机坐标系下的点和里程计
        subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);//上一帧角点
        subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);//上一帧面点
        subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2, &mapOptimization::laserCloudOutlierLastHandler, this); //上一帧无效点
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);//里程计位姿
        subImu = nh.subscribe<sensor_msgs::Imu> (imuTopic, 50, &mapOptimization::imuHandler, this); //IMU数据



        downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
        downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

        downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for histor key frames of loop closure
        downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for surrounding key poses of scan-to-map optimization

        downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

        odomAftMapped.header.frame_id = "camera_init";
        odomAftMapped.child_frame_id = "aft_mapped";

        aftMappedTrans.frame_id_ = "camera_init";
        aftMappedTrans.child_frame_id_ = "aft_mapped";

        allocateMemory();
    }

    void allocateMemory(){

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
        surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());        

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
        laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner feature set from odoOptimization
        laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMaptest.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        
        nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
        nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
        globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
        globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

        timeLaserCloudCornerLast = 0;
        timeLaserCloudSurfLast = 0;
        timeLaserOdometry = 0;
        timeLaserCloudOutlierLast = 0;
        timeLastGloalMapPublish = 0;

        timeLastProcessing = -1;

        newLaserCloudCornerLast = false;
        newLaserCloudSurfLast = false;

        newLaserOdometry = false;
        newLaserCloudOutlierLast = false;

        for (int i = 0; i < 6; ++i){
            transformLast[i] = 0;
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformTobeMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }

        imuPointerFront = 0;
        imuPointerLast = -1;

        for (int i = 0; i < imuQueLength; ++i){
            imuTime[i] = 0;
            imuRoll[i] = 0;
            imuPitch[i] = 0;
        }

        gtsam::Vector Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        priorNoise = noiseModel::Diagonal::Variances(Vector6);
        odometryNoise = noiseModel::Diagonal::Variances(Vector6);

        matA0 = cv::Mat (5, 3, CV_32F, cv::Scalar::all(0));
        matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
        matX0 = cv::Mat (3, 1, CV_32F, cv::Scalar::all(0));

        matA1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        matD1 = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        matV1 = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        isDegenerate = false;
        matP = cv::Mat (6, 6, CV_32F, cv::Scalar::all(0));

        laserCloudCornerFromMapDSNum = 0;
        laserCloudSurfFromMapDSNum = 0;
        laserCloudCornerLastDSNum = 0;
        laserCloudSurfLastDSNum = 0;
        laserCloudOutlierLastDSNum = 0;
        laserCloudSurfTotalLastDSNum = 0;

        potentialLoopFlag = false;
        aLoopIsClosed = false;

        latestFrameID = 0;
    }
    
 // 将坐标转移到世界坐标系下,得到可用于建图的Lidar坐标
    void transformAssociateToMap()
    {
        // transformSum 当前帧的map坐标系位置，激光里程计
        // transformBefMapped 前一帧在全局优化后在map坐标系的位置
        // transformTobeMapped 转换到全局优化后的坐标系
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;
		 // 计算平移增量
        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformTobeMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                       crycrx / cos(transformTobeMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                       crzcrx / cos(transformTobeMapped[0]));

        x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
        y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
        z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

        transformTobeMapped[3] = transformAftMapped[3] 
                               - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
        transformTobeMapped[4] = transformAftMapped[4] - y2;
        transformTobeMapped[5] = transformAftMapped[5] 
                               - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
    }

    void transformUpdate()
    {
		if (imuPointerLast >= 0) {
		    float imuRollLast = 0, imuPitchLast = 0;
		    // imuPointerFront不断移动，直到 某个imu数据时间戳 比 激光里程计+1圈扫描 时间戳晚
            // 这是为了后面进行插值
		    while (imuPointerFront != imuPointerLast) {
		        if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
		            break;
		        }
		        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
		    }
			// 如果没有找到合适的imu数据
		    if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
		        imuRollLast = imuRoll[imuPointerFront];
		        imuPitchLast = imuPitch[imuPointerFront];
		    } 
		    // 有合适imu数据
            // 再取前一个IMU，准备插值
		    else {
		        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
		        // 开始插值
		        float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
		                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
		        float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
		                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
				// 插值得到 时间戳为 (当前激光里程计+1圈扫描)时间戳的 IMU数据
		        imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
		        imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
		    }
			// 加权得到位姿估计
		    transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
		    transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
		  }
		// 把当前帧里程计值记录到transformBefMapped[]
        // 把优化后的位姿记录到transformAftMapped[]
		for (int i = 0; i < 6; i++) {
		    transformBefMapped[i] = transformSum[i];
		    transformAftMapped[i] = transformTobeMapped[i];
		}
    }

    void updatePointAssociateToMapSinCos(){
        cRoll = cos(transformTobeMapped[0]);
        sRoll = sin(transformTobeMapped[0]);

        cPitch = cos(transformTobeMapped[1]);
        sPitch = sin(transformTobeMapped[1]);

        cYaw = cos(transformTobeMapped[2]);
        sYaw = sin(transformTobeMapped[2]);

        tX = transformTobeMapped[3];
        tY = transformTobeMapped[4];
        tZ = transformTobeMapped[5];
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        float x1 = cYaw * pi->x - sYaw * pi->y;
        float y1 = sYaw * pi->x + cYaw * pi->y;
        float z1 = pi->z;

        float x2 = x1;
        float y2 = cRoll * y1 - sRoll * z1;
        float z2 = sRoll * y1 + cRoll * z1;

        po->x = cPitch * x2 + sPitch * z2 + tX;
        po->y = y2 + tY;
        po->z = -sPitch * x2 + cPitch * z2 + tZ;
        po->intensity = pi->intensity;
    }

    void updateTransformPointCloudSinCos(PointTypePose *tIn){

        ctRoll = cos(tIn->roll);
        stRoll = sin(tIn->roll);

        ctPitch = cos(tIn->pitch);
        stPitch = sin(tIn->pitch);

        ctYaw = cos(tIn->yaw);
        stYaw = sin(tIn->yaw);

        tInX = tIn->x;
        tInY = tIn->y;
        tInZ = tIn->z;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn){
	// !!! DO NOT use pcl for point cloud transformation, results are not accurate
        // Reason: unkown
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
            float y1 = stYaw * pointFrom->x + ctYaw* pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = ctRoll * y1 - stRoll * z1;
            float z2 = stRoll * y1 + ctRoll* z1;

            pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
            pointTo.y = y2 + tInY;
            pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);
        
        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
            float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
            float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

            pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
            pointTo.y = y2 + transformIn->y;
            pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

	//接收离群点
    void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudOutlierLast = msg->header.stamp.toSec();
        laserCloudOutlierLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
        newLaserCloudOutlierLast = true;
    }
	//接收角点
    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudCornerLast = msg->header.stamp.toSec();
        laserCloudCornerLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudCornerLast);
        newLaserCloudCornerLast = true;
    }
	//接收面点
    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudSurfLast = msg->header.stamp.toSec();
        laserCloudSurfLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudSurfLast);
        newLaserCloudSurfLast = true;
    }
	//接收里程计
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        timeLaserOdometry = laserOdometry->header.stamp.toSec();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        //transformSum[] 记录六自由度位姿,即里程计位姿
        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;
        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;
        newLaserOdometry = true;
    }
	//对IMU进行处理
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
    }

    void publishTF(){

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped[3];
        odomAftMapped.pose.pose.position.y = transformAftMapped[4];
        odomAftMapped.pose.pose.position.z = transformAftMapped[5];
        odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
        odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
        odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
        odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
        odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
        odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
        tfBroadcaster.sendTransform(aftMappedTrans);
    }

    PointTypePose trans2PointTypePose(float transformIn[]){
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    void publishKeyPosesAndFrames(){

        if (pubKeyPoses.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "camera_init";
            pubKeyPoses.publish(cloudMsgTemp);
        }

        if (pubRecentKeyFrames.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "camera_init";
            pubRecentKeyFrames.publish(cloudMsgTemp);
        }

        if (pubRegisteredCloud.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);  //角点
            *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);   //平面点
            
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudOut, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "camera_init";
            pubRegisteredCloud.publish(cloudMsgTemp);
        } 
    }

    void visualizeGlobalMapThread(){
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
            // 1.通过KDTree进行最近邻搜索;
            // 2.通过搜索得到的索引放进队列;
            // 3.通过两次下采样，减小数据量;
            //发布/laser_cloud_surround话题
        }
        // save final point cloud
        pcl::io::savePCDFileASCII(fileDirectory+"finalCloud.pcd", *globalMapKeyFramesDS);

        string cornerMapString = "/tmp/cornerMap.pcd";
        string surfaceMapString = "/tmp/surfaceMap.pcd";
        string trajectoryString = "/tmp/trajectory.pcd";

        pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());
        
        for(int i = 0; i < cornerCloudKeyFrames.size(); i++) {
            *cornerMapCloud  += *transformPointCloud(cornerCloudKeyFrames[i],   &cloudKeyPoses6D->points[i]);
    	    *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i],     &cloudKeyPoses6D->points[i]);
    	    *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        }

        downSizeFilterCorner.setInputCloud(cornerMapCloud);
        downSizeFilterCorner.filter(*cornerMapCloudDS);
        downSizeFilterSurf.setInputCloud(surfaceMapCloud);
        downSizeFilterSurf.filter(*surfaceMapCloudDS);

        pcl::io::savePCDFileASCII(fileDirectory+"cornerMap.pcd", *cornerMapCloudDS);
        pcl::io::savePCDFileASCII(fileDirectory+"surfaceMap.pcd", *surfaceMapCloudDS);
        pcl::io::savePCDFileASCII(fileDirectory+"trajectory.pcd", *cloudKeyPoses3D);
    }


//1.通过KDTree进行最近邻搜索;
//2.通过搜索得到的索引放进队列;
//3.通过两次下采样，减小数据量;
    void publishGlobalMap(){

        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;
	    // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
	    // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        // 通过KDTree进行最近邻搜索
        kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
          globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
	    // 对globalMapKeyPoses进行下采样
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
	    // extract visualized and downsampled key frames
        for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i){
			int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
			*globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],   &cloudKeyPoses6D->points[thisKeyInd]);
			*globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
			*globalMapKeyFrames += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
	    // downsample visualized points
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
 
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = "camera_init";
        pubLaserCloudSurround.publish(cloudMsgTemp);  

        globalMapKeyPoses->clear();
        globalMapKeyPosesDS->clear();
        globalMapKeyFrames->clear();
        // globalMapKeyFramesDS->clear();     
    }

	//主要功能是进行闭环检测和闭环修正。
    void loopClosureThread(){

        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(1);
        while (ros::ok()){
            rate.sleep();
            performLoopClosure(); //启用闭环检测判断
        }
    }

	//判断是否进入回环
    bool detectLoopClosure(){

        latestSurfKeyFrameCloud->clear();
        nearHistorySurfKeyFrameCloud->clear();
        nearHistorySurfKeyFrameCloudDS->clear();

        // 资源分配时初始化
        // 在互斥量被析构前不解锁
        std::lock_guard<std::mutex> lock(mtx);
        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        // 1. 查找当前点7米范围内是否有之前已经采样的点
        kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
        // 进行半径historyKeyframeSearchRadius内的邻域搜索，
        // currentRobotPosPoint：需要查询的点，
        // pointSearchIndLoop：搜索完的邻域点对应的索引
        // pointSearchSqDisLoop：搜索完的每个邻域点与当前点之间的欧式距离
        // 0：返回的邻域个数，为0表示返回全部的邻域点
        kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        closestHistoryFrameID = -1;
        for (int i = 0; i < pointSearchIndLoop.size(); ++i){
            int id = pointSearchIndLoop[i];
            // 两个时间差值大于30秒
            if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry) > 30.0){
                closestHistoryFrameID = id;
                break;
            }
        }
        if (closestHistoryFrameID == -1){
       		// 找到的点和当前时间上没有超过30秒的
            return false;
        }
        // save latest key frames
        latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
        // 点云的xyz坐标进行坐标系变换(分别绕xyz轴旋转)
        *latestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
        *latestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],   &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
        // latestSurfKeyFrameCloud中存储的是下面公式计算后的index(intensity):
        // thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
        // 滤掉latestSurfKeyFrameCloud中index<0的点??? index值会小于0?
        pcl::PointCloud<PointType>::Ptr hahaCloud(new pcl::PointCloud<PointType>());
        int cloudSize = latestSurfKeyFrameCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            // intensity不小于0的点放进hahaCloud队列
            // 初始化时intensity是-1，滤掉那些点
            if ((int)latestSurfKeyFrameCloud->points[i].intensity >= 0){
                hahaCloud->push_back(latestSurfKeyFrameCloud->points[i]);
            }
        }
        latestSurfKeyFrameCloud->clear();
        *latestSurfKeyFrameCloud = *hahaCloud;
	   // save history near key frames
        // historyKeyframeSearchNum在utility.h中定义为25，前后25个点进行变换
        for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j){
            if (closestHistoryFrameID + j < 0 || closestHistoryFrameID + j > latestFrameIDLoopCloure)
                continue;
            //要求closestHistoryFrameID + j在0到cloudKeyPoses3D->points.size()-1之间,不能超过索引
            //将搜索范围内的角点点云与平面点点云均叠加至nearHistorySurfKeyFrameCloud中
            *nearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[closestHistoryFrameID+j], &cloudKeyPoses6D->points[closestHistoryFrameID+j]);
            *nearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[closestHistoryFrameID+j],   &cloudKeyPoses6D->points[closestHistoryFrameID+j]);
        }
		// 下采样滤波减少数据量
        downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
        downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);
        // publish history near key frames
        if (pubHistoryKeyFrames.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "camera_init";
            pubHistoryKeyFrames.publish(cloudMsgTemp);
        }

        return true;
    }

//  1.先进行闭环检测detectLoopClosure()，如果返回true,则进入闭环，否则直接返回，程序结束。
//  2.接着使用icp迭代进行对齐。
//  3.对齐之后判断迭代是否收敛以及噪声是否太大，是则返回并直接结束函数。否则进行迭代后的数据发布处理。
//  4.接下来得到latestSurfKeyFrameCloud和nearHistorySurfKeyFrameCloudDS之间的位置平移和旋转。
//  5.然后进行图优化过程。
    void performLoopClosure(){
		// 如果关键帧为空，返回
        //1.先进行闭环检测detectLoopClosure()，
        if (cloudKeyPoses3D->points.empty() == true)
            return;
        // try to find close key frame if there are any
        // 尝试去寻找回环点
        if (potentialLoopFlag == false){

            if (detectLoopClosure() == true){ //// 进行回环检测
                potentialLoopFlag = true; // find some key frames that is old enough or close enough for loop closure
                timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
            }
            if (potentialLoopFlag == false)
                return;
        }
        // reset the flag first no matter icp successes or not
        potentialLoopFlag = false;
        //2.接着使用icp迭代进行对齐
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);  
        icp.setMaximumIterations(100);  //设置最大迭代次数
        icp.setTransformationEpsilon(1e-6);  //设置前后两次迭代的转换矩阵的最大容差（epsilion），一旦两次迭代小于这个最大容差，则认为已经收敛到最优解，迭代停止。
        icp.setEuclideanFitnessEpsilon(1e-6); //设置前后两次迭代的点对的欧式距离均值的最大容差
        icp.setRANSACIterations(0);  // 设置RANSAC运行次数
        // Align clouds进行icp点云对齐
        // 匹配当前帧点云和之前的历史点云
        icp.setInputSource(latestSurfKeyFrameCloud);
        icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result); // 进行icp点云对齐
		//3.对齐之后判断迭代是否收敛以及噪声是否太大，是则返回并直接结束函数。否则进行迭代后的数据发布处理。
        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;
        //4.接下来得到latestSurfKeyFrameCloud和nearHistorySurfKeyFrameCloudDS之间的位置平移和旋转。
        // publish corrected cloud 以下在点云icp收敛并且噪声量在一定范围内进行
        // 发布校正之后的点云
        if (pubIcpKeyFrames.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud (*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "camera_init";
            pubIcpKeyFrames.publish(cloudMsgTemp);
        }   
        /*
        	get pose constraint
        	*/
        // 获取配准结果: 从漂移世界坐标系到真世界坐标系的变换
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionCameraFrame;
        //最终匹配得到的坐标转换和得到平移和旋转的角度
        correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
        // 得到平移和旋转的角度
        // r: (相机坐标系c系)绕x轴的旋转量 (也就是imu读数的pitch(载体坐标系b系)绕y轴的旋转量)
        // p: (相机坐标系c系)绕y轴的旋转量 (也就是imu读数的yaw(载体坐标系b系)绕z轴的旋转量)
        // y: (相机坐标系c系)绕z轴的旋转量 (也就是imu读数的roll(载体坐标系b系)绕x轴的旋转量)
        // x: (相机坐标系c系)x轴偏移      (载体坐标系b系)y轴偏移
        // y: (相机坐标系c系)y轴偏移      (载体坐标系b系)z轴偏移
        // z: (相机坐标系c系)z轴偏移      (载体坐标系b系)x轴偏移
        
        pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
        Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
        // transform from world origin to corrected pose
        
        // 计算矫正量
        // tWrong: 矫正前的关键帧 到 世界坐标系的变换 ==> 即当前帧坐标系 到 (漂移)世界坐标系的变换
        // correctionLidarFrame: 从漂移世界坐标系到真世界坐标系的变换
        // tCorrect: 当前关键帧在真世界坐标系的位姿
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // 矫正后的当前最新优化关键帧位姿
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
         // 取闭环关键帧位姿
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        /* 
        	add constraints
        	*/
        //5.然后进行图优化过程。
        std::lock_guard<std::mutex> lock(mtx);
        // 因子图加入回环边
        gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise));
        isam->update(gtSAMgraph); //添加回环因子
        isam->update();
        gtSAMgraph.resize(0);
        aLoopIsClosed = true;
    }

    Pose3 pclPointTogtsamPose3(PointTypePose thisPoint){ // camera frame to lidar frame
    	return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                           Point3(double(thisPoint.z),   double(thisPoint.x),    double(thisPoint.y)));
    }

    Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint){ // camera frame to lidar frame
    	return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
    }
    
    
//如果使用了回环检测，则添加过去50个关键帧，如果没有使用回环检测，则添加离当前欧式距离最近的50个关键帧，然后拼接成点云。
    void extractSurroundingKeyFrames(){

        if (cloudKeyPoses3D->points.empty() == true)  //判断有没有关键帧
            return;	
		
    	if (loopClosureEnableFlag == true){   // 如果启用回环检测
    	    // only use recent key poses for graph building
				// recentCornerCloudKeyFrames保存的点云数量太少，则清空后重新塞入新的点云直至数量够
				 // 1. 添加最近的关键位姿到图
                if (recentCornerCloudKeyFrames.size() < surroundingKeyframeSearchNum){ // queue is not full (the beginning of mapping or a loop is just closed)
                    // clear recent key frames queue
                    recentCornerCloudKeyFrames. clear();
                    recentSurfCloudKeyFrames.   clear();
                    recentOutlierCloudKeyFrames.clear();
                    int numPoses = cloudKeyPoses3D->points.size();
                     // 从最新的关键帧开始，取关键帧push
                    for (int i = numPoses-1; i >= 0; --i){
                        int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
                        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                        updateTransformPointCloudSinCos(&thisTransformation);
                        // 依据上面得到的变换thisTransformation，对cornerCloudKeyFrames，surfCloudKeyFrames，surfCloudKeyFrames进行坐标变换
                        // 提取过去50个关键帧
                        recentCornerCloudKeyFrames. push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                        recentSurfCloudKeyFrames.   push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                        recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                        if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
                            break;
                    }
                }
                // recentCornerCloudKeyFrames中点云保存的数量较多
                // pop队列最前端的一个，再push后面一个
                else{  // queue is full, pop the oldest key frame and push the latest key frame
                    if (latestFrameID != cloudKeyPoses3D->points.size() - 1){  // if the robot is not moving, no need to update recent frames

                        recentCornerCloudKeyFrames. pop_front();
                        recentSurfCloudKeyFrames.   pop_front();
                        recentOutlierCloudKeyFrames.pop_front();
                        // 2. 弹出队列中时间最久的帧，添加最新的帧到队列
                        latestFrameID = cloudKeyPoses3D->points.size() - 1;
                        PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
                        updateTransformPointCloudSinCos(&thisTransformation);
                        recentCornerCloudKeyFrames. push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
                        recentSurfCloudKeyFrames.   push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
                        recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
                    }
                }
				// 3. 拼接为点云
                for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i){
                    *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
                    *laserCloudSurfFromMap   += *recentSurfCloudKeyFrames[i];
                    *laserCloudSurfFromMap   += *recentOutlierCloudKeyFrames[i];
                    cout<<i<<endl;
                }
                /*
                cout<<"pub"<<endl;
                *laserCloudCornerFromMaptest = *laserCloudCornerFromMap;
                sensor_msgs::PointCloud2 cloudMsgTemp;
				pcl::toROSMsg(*laserCloudCornerFromMaptest, cloudMsgTemp);
				cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				cloudMsgTemp.header.frame_id = "camera_init";
				pubLocalmap.publish(cloudMsgTemp);  
                laserCloudCornerFromMaptest->clear();
                */
    	}
    	//不启用闭环检测
    	else{
            surroundingKeyPoses->clear();
            surroundingKeyPosesDS->clear();
    	    // extract all the nearby key poses and downsample them
    	      // 1. 查找当前pose 50米内的姿态
    	    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
    	    //1.进行半径surroundingKeyframeSearchRadius内的邻域搜索
    	    // 进行半径surroundingKeyframeSearchRadius内的邻域搜索，
            // currentRobotPosPoint：需要查询的点，
            // pointSearchInd：搜索完的邻域点对应的索引
            // pointSearchSqDis：搜索完的每个领域点点与传讯点之间的欧式距离
            // 0：返回的邻域个数，为0表示返回全部的邻域点
    	    kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);
    	    for (int i = 0; i < pointSearchInd.size(); ++i)
                surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
            //下采样
    	    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    	    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    	    //2.双重循环，不断对比surroundingExistingKeyPosesID和surroundingKeyPosesDS中点的index,如果能够找到一样，说明存在关键帧。然后在队列中去掉找不到的元素，留下可以找到的。
    	    // delete key frames that are not in surrounding region
    	    // 2. 删除不在周围区域的关键帧
            int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
            // 遍历已经存在的keypose
            // 如果最新的最近临列表surroundingKeyPosesDS[]里面没有这些已存在的keypose，则把这些已存在的keypose剔除
            for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i){
                bool existingFlag = false;
                for (int j = 0; j < numSurroundingPosesDS; ++j){
                    if (surroundingExistingKeyPosesID[i] == (int)surroundingKeyPosesDS->points[j].intensity){
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == false){
                    // 如果surroundingExistingKeyPosesID[i]对比了一轮的已经存在的关键位姿的索引后（intensity保存的就是size()）
                    // 没有找到相同的关键点，那么把这个点从当前队列中删除
                    // 否则的话，existingFlag为true，该关键点就将它留在队列中
                    surroundingExistingKeyPosesID.   erase(surroundingExistingKeyPosesID.   begin() + i);
                    surroundingCornerCloudKeyFrames. erase(surroundingCornerCloudKeyFrames. begin() + i);
                    surroundingSurfCloudKeyFrames.   erase(surroundingSurfCloudKeyFrames.   begin() + i);
                    surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin() + i);
                    --i;
                }
            }
			 // 3. 添加新关键帧
            // 上一个两重for循环主要用于删除数据，此处的两重for循环用来添加数据
    	    // add new key frames that are not in calculated existing key frames
            for (int i = 0; i < numSurroundingPosesDS; ++i) {
                bool existingFlag = false;
                for (auto iter = surroundingExistingKeyPosesID.begin(); iter != surroundingExistingKeyPosesID.end(); ++iter){
                    if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity){
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == true){
                    continue;
                }else{
                    int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    surroundingExistingKeyPosesID.   push_back(thisKeyInd);
                    surroundingCornerCloudKeyFrames. push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                    surroundingSurfCloudKeyFrames.   push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                    surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                }
            }
            // 4. 拼接点云
            for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
                *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i]; //局部角点地图
                *laserCloudSurfFromMap   += *surroundingSurfCloudKeyFrames[i];
                *laserCloudSurfFromMap   += *surroundingOutlierCloudKeyFrames[i];  //局部面点和界外点地图
                cout<<i<<endl;
            }
    	}
    	//5. 不管是否进行闭环过程，最后的输出都要进行一次下采样减小数据量的过程。
        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
       
    }
	//函数是将当前各类点云降采样
    void downsampleCurrentScan(){

        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);  //将上一帧的角点下采样保存到laserCloudCornerLastDS
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

        laserCloudOutlierLastDS->clear();
        downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
        downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
        laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

        laserCloudSurfTotalLast->clear();
        laserCloudSurfTotalLastDS->clear();
        *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
        *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
        downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
        downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
        laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
    }

	//角点优化
    void cornerOptimization(int iterCount){

        updatePointAssociateToMapSinCos();
        // 进行坐标变换,转换到全局坐标中去（世界坐标系）
        // pointSel:表示选中的点，point select
        // 输入是pointOri，输出是pointSel
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            pointOri = laserCloudCornerLastDS->points[i];//上一帧降采样的角点
            pointAssociateToMap(&pointOri, &pointSel);
            // 1. 查找最近的5个点
            // pointSel为需要搜索的点，
            // pointSearchInd搜索完的邻域对应的索引
            // pointSearchSqDis 邻域点与查询点之间的距离
            //kdtreeCornerFromMap 是关键帧角点组成的kdtree
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            // 2. 上述点距离小于1m
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                // 3. 5个点的质心
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                // 4. 协方差
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);
					// 5. 计算coeff
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOri->push_back(pointOri);
                        coeffSel->push_back(coeff);
                    }
                }
            }
        }
    }

    void surfOptimization(int iterCount){
        updatePointAssociateToMapSinCos();
        for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
            pointOri = laserCloudSurfTotalLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0, 0);
                float pb = matX0.at<float>(1, 0);
                float pc = matX0.at<float>(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOri->push_back(pointOri);
                        coeffSel->push_back(coeff);
                    }
                }
            }
        }
    }

    bool LMOptimization(int iterCount){
        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        int laserCloudSelNum = laserCloudOri->points.size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        // 1. 遍历laserCloudSel
        for (int i = 0; i < laserCloudSelNum; i++) {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        // 2. 最小二乘法
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
		// 3. 获取transform
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;
        }
        return false;
    }
    //根据现有地图与最新点云数据进行配准从而更新机器人精确位姿与融合建图，它分为角点优化、平面点优化、配准与更新等部分。
    void scan2MapOptimization(){

        if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {
			// 1. 根据周围关键帧点云创建kdtree
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 10; iterCount++) {

                laserCloudOri->clear();
                coeffSel->clear();
				// 2.角和面特征计算距离
                cornerOptimization(iterCount);//角点优化
                surfOptimization(iterCount);//面点优化
				 // 3. LM优化
                if (LMOptimization(iterCount) == true)//判断迭代次数
                    break;              
            }

            transformUpdate();// 迭代结束更新相关的转移矩阵
        }
    }

	//保存关键帧
    void saveKeyFramesAndFactor(){

        currentRobotPosPoint.x = transformAftMapped[3];
        currentRobotPosPoint.y = transformAftMapped[4];
        currentRobotPosPoint.z = transformAftMapped[5];
		 
        bool saveThisKeyFrame = true;
        // 距离大于0.3保存关键帧
        if (sqrt((previousRobotPosPoint.x-currentRobotPosPoint.x)*(previousRobotPosPoint.x-currentRobotPosPoint.x)
                +(previousRobotPosPoint.y-currentRobotPosPoint.y)*(previousRobotPosPoint.y-currentRobotPosPoint.y)
                +(previousRobotPosPoint.z-currentRobotPosPoint.z)*(previousRobotPosPoint.z-currentRobotPosPoint.z)) < 0.3){
            saveThisKeyFrame = false;
        }

        

        if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty())
        	return;

        previousRobotPosPoint = currentRobotPosPoint;
        /**
         * update grsam graph
         */
        //一开始在因子图中添加固定先验因子
        if (cloudKeyPoses3D->points.empty()){
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                                       		 Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                                  Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
            for (int i = 0; i < 6; ++i)
            	transformLast[i] = transformTobeMapped[i];
        }
       // 随后添加位姿间的二元因子
        else{
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                                Point3(transformLast[5], transformLast[3], transformLast[4]));
            gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                                Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size()-1, cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                                                     		   Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
        }
        /**
         * update iSAM
         */
        //更新isam和计算位姿
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        /**
         * 保存关键poses
         */
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;
		//访问最终矫正后的位姿	
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
		//保存cloudKeyPoses3D
        thisPose3D.x = latestEstimate.translation().y();
        thisPose3D.y = latestEstimate.translation().z();
        thisPose3D.z = latestEstimate.translation().x();
        thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D); //cloudKeyPoses3D是一个一个坐标系关键位置点

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().pitch();
        thisPose6D.pitch = latestEstimate.rotation().yaw();
        thisPose6D.yaw   = latestEstimate.rotation().roll(); // in camera frame
        thisPose6D.time = timeLaserOdometry;
        cloudKeyPoses6D->push_back(thisPose6D); //cloudKeyPoses6D是一个一个欧拉位置点
        /**
         * save updated transform
         */
        if (cloudKeyPoses3D->points.size() > 1){
            transformAftMapped[0] = latestEstimate.rotation().pitch();
            transformAftMapped[1] = latestEstimate.rotation().yaw();
            transformAftMapped[2] = latestEstimate.rotation().roll();
            transformAftMapped[3] = latestEstimate.translation().y();
            transformAftMapped[4] = latestEstimate.translation().z();
            transformAftMapped[5] = latestEstimate.translation().x();

            for (int i = 0; i < 6; ++i){
            	transformLast[i] = transformAftMapped[i];
            	transformTobeMapped[i] = transformAftMapped[i];
            }
        }

        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());
		// 保存点云
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);
        pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
        //发布测试
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*thisCornerKeyFrame, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = "camera_init";
        pubtestcloud.publish(cloudMsgTemp);  
    }


	//回环检测如果成功，会设置aLoopIsClosed为True，才会进行这一步。将isam优化后的位姿替换之前的位姿。
    void correctPoses(){
    	if (aLoopIsClosed == true){
            recentCornerCloudKeyFrames. clear();
            recentSurfCloudKeyFrames.   clear();
            recentOutlierCloudKeyFrames.clear();
            // update key poses
            // 1. 将isam优化后的位姿替换之前的位姿
                int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i){
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().z();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().x();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            }

            aLoopIsClosed = false;
        }
    }

    void clearCloud(){
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();  
        laserCloudCornerFromMapDS->clear();
        laserCloudSurfFromMapDS->clear();   
    }

    void run(){
		//判断是否有新的数据到来并且时间差值小于0.005
        if (newLaserCloudCornerLast  && std::abs(timeLaserCloudCornerLast  - timeLaserOdometry) < 0.005 &&
            newLaserCloudSurfLast    && std::abs(timeLaserCloudSurfLast    - timeLaserOdometry) < 0.005 &&
            newLaserCloudOutlierLast && std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 &&
            newLaserOdometry)
        {

            newLaserCloudCornerLast = false; newLaserCloudSurfLast = false; newLaserCloudOutlierLast = false; newLaserOdometry = false;
			 // 创建对象 lock 对mtx进行上锁,当lock被析构时,自动解锁
            std::lock_guard<std::mutex> lock(mtx);
			// 全局姿态优化的时间判断条件：距离上次优化已经过去0.3s
            if (timeLaserOdometry - timeLastProcessing >= mappingProcessInterval) {

                timeLastProcessing = timeLaserOdometry; // 储存里程计数据时间戳
				//1.转换到全局优化后的map坐标系下
                transformAssociateToMap();
				//2.提取上一帧优化得到的位姿附近的临近点合并成局部地图
				//由于帧数的频率大于建图的频率，因此需要提取关键帧进行匹配
				//     2.1 在 关键帧位置集合cloudKeyPoses3D 中
                //     检索 当前位置currentRobotPosPoint 附近的姿态点
                //     获得局部位置点，赋值给 局部位置点集合surroundingKeyPoses
                //     2.2 根据 局部位置点集合surroundingKeyPoses 更新
                //     局部关键帧集合 surroundingExistingKeyPosesID
                //     局部关键帧 角点点云集合surroundingCornerCloudKeyFrames
                //     局部关键帧 平面点点云集合surroundingSurfCloudKeyFrames
                //     局部关键帧 离群点点云集合surroundingOutlierCloudKeyFrames
                //     增加新进入局部的关键帧、并删除离开局部的关键帧。
                //     2.3 为局部点云地图赋值
                //     laserCloudCornerFromMap     所有局部关键帧的角点集合
                //     laserCloudSurfFromMap       所有局部关键帧平面点和离群点的集合
                extractSurroundingKeyFrames();
				//3.降采样匹配以及增加地图点云，回环检测
				// 对来自msg的点云进行降采样
                // laserCloudCornerLast   //角点点云
                // laserCloudSurfLast     // 平面点点云
                // laserCloudOutlierLast  // 离群点点云
                // laserCloudSurfTotalLast // 平面点+离群点

                // 当前扫描进行边缘优化，图优化以及进行LM优化的过程,
                // 根据现有地图与最新点云数据进行配准,从而更新机器人精确位姿与融合建图，
                // 它分为角点优化、平面点优化、配准与更新等部分。
                //使用 scan-to-model 的策略优化了 变换 transformTobeMapped
                downsampleCurrentScan();
				//4.scan到map优化
				// 使用scan-to-model位姿优化，获得当前时间点机器人的位姿 transformTobeMapped
                // 参考IMU的姿态 对 transformTobeMapped 进行中值滤波,获得最终的机器人位姿
                // 为 transformBefMapped 赋值为 里程计位姿，即scan-to-model优化前的位姿
                // 为 transformAftMapped 赋值为 transformTobeMapped 即scan-to-model优化后的位姿
                scan2MapOptimization();
                
                
				//5. 保存关键帧和因子gtsam优化
					// 1选定关键帧
		            // 2根据新的关键帧更新因子图
		            // 3经过因子图优化后，更新当前位姿
		            // transformAftMapped
		            // transformTobeMapped
		            // 并用因子图优化后的位姿 创建关键帧位置点和位姿点,添加到集合
		            // cloudKeyPoses3D  关键帧位置点集合
		            // cloudKeyPoses6D  关键帧位姿点集合
		            // 并为 transformLast 赋值
		            // 更新
		            // cornerCloudKeyFrames 关键帧角点点云集合
		            // surfCloudKeyFrames   关键帧平面点点云集合
		            // outlierCloudKeyFrames关键帧离群点点云集合
		            //correctPoses是回环检测成功后将位姿图的数据依次更新
		            //闭环处理矫正、暂不考虑
                saveKeyFramesAndFactor();
				//6.校正位姿
                correctPoses();
				//7.发布坐标变换
                publishTF();
				//8.发布关键帧和因子
					// 主题 "/key_pose_origin" 发布关键帧位姿点集合
		            // 主题 "/recent_cloud" 发布 局部平面点点云
		            // 主题 "/registered_cloud" 发布 当前帧点云
                publishKeyPosesAndFrames();
				//9.清除点云
                clearCloud();
            }
        }
    }
};

//从主函数开始分析，主函数分为2个主要的功能，一是启动单独的线程进行回环检测，二是在循环中执行主流程。
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");
 	// std::thread 构造函数，将MO作为参数传入构造的线程中使用
    mapOptimization MO;
    // 1.启动单独的闭环检测与闭环的功能
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    // 2.将数据发布到ros中，可视化
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        MO.run(); // 3.执行run函数

        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
