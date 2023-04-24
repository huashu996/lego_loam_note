Header header 
#cloud_info.msg文件主要作用是构建了一个自定义的ROS Message类型。其中的各个变量包含不同的含义，在各函数中发挥不同的作用。
# 激光点的起始索引与终止索引（在所有激光点云中的序号）
int32[] startRingIndex
int32[] endRingIndex

float32 startOrientation
float32 endOrientation
float32 orientationDiff

bool[]    segmentedCloudGroundFlag # true - ground point, false - other points
uint32[]  segmentedCloudColInd # point column index in range image
float32[] segmentedCloudRange # point range 
