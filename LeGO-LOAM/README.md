# 依赖安装
具体安装调试细节见
https://blog.csdn.net/HUASHUDEYANJING/article/details/121107774
## gtsam
```
    cd ~
    git clone https://bitbucket.org/gtborg/gtsam.git
    cd ~/gtsam
    mkdir build
    cd build
    cmake ..
    make check   #可选的，运行单元测试，我没执行这个命令，因为在TX2上编译太慢了，太慢了，太慢了
    make install
```
## 下载lego_loam
```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/haushu996/lego_loam_note.git
    cd ..
    catkin_make -j1
```
当第一次编译代码时，需要在“catkin_make”后面添加“-j1”以生成一些消息类型。将来的编译不需要“-j1”。
## 运行lego_loam程序
```
    roslaunch lego_loam run.launch
    rosbag play 3-1.bag --clock
    #转为pcd文件
    rosbag record -o out /laser_cloud_surround
    rosrun pcl_ros bag_to_pcd input.bag /laser_cloud_surround pcd
    pcl_viewer xxxxxx.pcd
```
