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
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.
/*
此文件主要将点云投影成深度图，随后去除地面点，对点云进行聚类分割筛选有效聚类点，最后发布，一共有7个主要函数如下：
整体输入：
		subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);//接收消息到cloudHandler回调函数
整体输出：
		// 投影成图片的点云
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        // 转换成图片的并带有距离信息的点云
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);
		// 发布提取的地面特征
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        // 发布已经分割的点云
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        // 具有几何信息的分割点云
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        // 具有信息的分割点云
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        // 含有异常信息的点云
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        
        
1. copyPointCloud(laserCloudMsg)  转化成ROS消息
	1.1点云消息转ROS
	1.2去除NAN点
2. findStartEndAngle();   确定起始终止角度
	2.1 找寻开始点和结束点的航向角
	2.2 保证 所有角度 落在 [M_PI , 3M_PI] 上
3. projectPointCloud();   投影成深度图
	3.1 遍历点云
	3.2 通过垂直角度判断点在哪一行
	3.3 通过水平角度判断点在哪一列
	3.4 计算距离过滤较近的点
	3.5 保存到rangeMat二维矩阵
	3.6 保存投影点云和深度信息到fullInfoCloud
4. groundRemoval();   标记地面点
	4.1 先选择可能是地面点的线束遍历
	4.2 根据intensity筛选无效点保存到groundMat
	4.3 计算两个线束之间的夹角
	4.4 判断是否为地面点
	4.5 标记地面点记号-1
	4.6 保存地面点
5. cloudSegmentation();   提取有效分割点
	5.1 对非地面点进行分割，聚类
		5.1.1 BFS的指针和下标赋初值
		5.1.2 遍历整个点云 遍历前后左右四个点,求点之间的角度数值
		5.1.3 聚类点累积
		5.1.4 判断聚类是否有效
		5.1.5 无效聚类标记
	5.2 提取分割点云 用于激光雷达里程计
		5.2.1 去除界外点
		5.2.2 地面点云稀疏化
		5.2.3 保存筛选后分割点云信息到segMsg
	5.3 分割点云保存到segmentedCloudPure
6. publishCloud();   发布点云
	6.1 发布自定义分割点云消息segMsg
	6.2 发布边缘异常点云
	6.3 发布带地面的分割点云
	6.4 发布全部点云无距离信息
	6.5 发布地面点
	6.6 发布没有地面点的分割点云，用于后面的edge特征提取
	6.7 发布所有点云信息
7. resetParameters();   重新初始化参数
	7.1 清空所有点云变量
	7.2 所有矩阵设为0
*/



#include "utility.h"
//ImageProjection类
class ImageProjection{
//定义私有变量
private:
    //1、定义ROS接口
    ros::NodeHandle nh;
	//定义接收者
    ros::Subscriber subLaserCloud; 
    //定义发布者
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;
    //2、定义点云变量
    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration
    //3、其他变量
    //定义三个矩阵分别代表点云投影后的range image、标签矩阵和地面矩阵
    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;
    //一帧点云的起始角和结束角
    float startOrientation;
    float endOrientation;
    //定义rosmsg
    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;
    //定义用于BFS的索引
    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

public:
    ImageProjection(): //构造函数
    	//在代码中，使用 nh("~") 创建节点句柄对象，并将其设置为私有命名空间，这意味着节点可以在其私有命名空间中设置和获取参数，而不会影响其他节点中的同名参数。节点可以使用 nh.getParam() 和 nh.setParam() 方法来获取和设置参数。
        nh("~"){
        //接收消息
        // 订阅原始的激光点云
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);//接收消息到cloudHandler回调函数
		//发布消息
		// 转换成图片的点云
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        // 转换成图片的并带有距离信息的点云
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);
		// 发布提取的地面特征
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        // 发布已经分割的点云
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        // 具有几何信息的分割点云
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        // 含有异常信息的点云
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        //无效点
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();//numeric_limits 类模板提供查询各种算术类型属性的标准化方式（例如 int 类型的最大可能值是 std::numeric_limits::max() ）
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory(); //申请点云资源
        resetParameters(); //清空内存初始化
    }
//————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//构造函数中的子函数
    void allocateMemory(){

        //初始化NEW动态空间
        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());
		//重置投影点云的大小
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);
		//自定义ROS消息的分配空间和初始化
        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);
		//neighborIterator用于求某个点的4邻域点在labelComponents中使用
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);
		//索引重置
        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        //7.1 清空所有点云变量
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();
		//7.2 所有矩阵设为0
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection(){}
//——————————————————————————————————————————————————————————————————————————————————————————————————————————    
    //回调函数中的子函数
    //const 可以用于修饰任何基本类型、对象和指针类型
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
		//1.1点云消息转ROS——————————————————————————
        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn); //原始点云保存到laserCloudIn变量中
        //1.2去除NAN点——————————————————————————
        std::vector<int> indices; //将一个整数value添加到该vector容器中
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices); //cloud_in是输入点云，cloud_out是输出点云，index是一个可选的输出参数，它包含了被移除点的索引。
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud转化
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }

    void findStartEndAngle(){
        //2.1 找寻开始点和结束点的航向角——————————————————————————
        // start and end orientation of this cloud开始点和结束点的航向角
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        // 2.2 保证 所有角度 落在 [M_PI , 3M_PI] 上——————————————————————————
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

	// 点云消息处理成图像方式的阵列 将一帧点云变成 16* 1800 的图片
    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;
		//3.1 遍历点云——————————————————————————
        cloudSize = laserCloudIn->points.size();
		//遍历整个点云
        for (size_t i = 0; i < cloudSize; ++i){  

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            //3.2 通过垂直角度判断点在哪一行——————————————————————————
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;
			//3.3 通过水平角度判断点在哪一列——————————————————————————
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
			// 保证 columnIdn 的大小在 [0 , 1800)
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
			// 3.4 计算距离过滤较近的点——————————————————————————
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            //如果距离小于 1米 则过滤掉 通常是过滤自身（距离传感器比较近的点）
            if (range < sensorMinimumRange)
                continue;
            //信息保存到 rangeMat 中，这样就将一个三维的坐标 转换到一个rangeMat矩阵中了.
            // 3.5 保存到rangeMat二维矩阵——————————————————————————
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            // 3.6 保存投影点云和深度信息到fullInfoCloud——————————————————————————
			 // 将 index 和 横坐标存储在 intensity 中 整数部分是线束数值  小数部分是方向角度
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
			//定义深度图中点的索引值  index = 列号 +  行号 * 1800 
            index = columnIdn  + rowIdn * Horizon_SCAN;
            //保存投影后的点到fullCloud
            fullCloud->points[index] = thisPoint;
             //保存投影后的点+深度信息 到fullInfoCloud
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

	//地面点去除首先判断地面点的标志就是相邻两个激光线束扫描到点的坐标，如果两个坐标之间的差值 或者两个坐标之间的斜率大于一个设定的值，则将该点判断是地面点。
    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        //4.1 先选择可能是地面点的线束进行去除————————————————————————
        for (size_t j = 0; j < Horizon_SCAN; ++j){  //一圈
            for (size_t i = 0; i < groundScanInd; ++i){  //线束

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;
				// 4.2 根据intensity筛选无效点保存到groundMat————————————————————————
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                // 4.3 计算两个线束之间的夹角——————————————————————
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
				// 4.4 判断是否为地面点
				// 如果夹角数值小于 10 度， 则可以判断为平面保存到groundMat
                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        // 4.5 标记地面点
         // 给地面点 标记一个符号 为 -1 
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        // 4.6 保存地面点
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }
	//分割函数
    void cloudSegmentation(){
        //5.1 对非地面点进行分割，聚类——————————————————
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j); //调用函数聚类

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        // 5.2 提取分割点云 用于激光雷达里程计——————————————————
        for (size_t i = 0; i < N_SCAN; ++i) {
            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5; //去除每束激光前5个点
			
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    //5.2.1 去除界外点
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }
                        else{
                            continue;
                        }
                    }
                    //5.2.2 地面点云稀疏化
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    //5.2.3 保存筛选后分割点云信息到segMsg——————————————
                    // mark ground points so they will not be considered as edge features later标记地面点
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later 点云列索引
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info 深度信息
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5; //舍弃每束激光最后五个点：
        }
        //5.3 无地面的分割点云保存到segmentedCloudPure————————————————————
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

	//地面障碍物分割聚类
    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        //5.1.1 BFS的指针和下标赋初值——————————————————
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            //5.1.2 遍历整个点云 遍历前后左右四个点,求点之间的角度数值 ——————————————————
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        //5.1.3 聚类点云累积——————————————————
        // 如果是 allPushedIndSize 累加的数量增加到了30 个 则判断这部分点云属于一个聚类
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        //5.1.4 判断聚类是否有效——————————————————
        // 如果垂直 方向上点的数量大于最低阈值 默认是一个有效的聚类
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }
        // 5.1.5 无效聚类标记
        else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    //发布点云
    void publishCloud(){
        //6.1 发布自定义分割点云消息segMsg————————
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        //6.2 发布边缘异常点云————————————
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        //6.3 发布带地面的分割点云————————————
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        //6.4 发布全部点云————————————
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
         //6.5 发布地面点————————————
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
         //6.6 发布没有地面点的分割点云，用于后面的edge特征提取————————————
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        //6.7 发布所有点云信息————————————
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};
//——————————————————————————————————————————————————————————————————————————————————————————————————————



int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");  //ROS初始化
    
    ImageProjection IP;  //创建一个类，运行构造函数

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin(); //循环
    return 0;
}
