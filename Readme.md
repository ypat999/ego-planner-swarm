# EGO-Planner-Swarm (XTDrone2版本)

这是EGO-Planner-Swarm的增强版本，支持ROS2、TF2坐标转换和改进的点云处理，专为XTDrone2框架优化。

## 主要特性
- **ROS2支持**: 完整迁移到ROS2，性能优化
- **TF2集成**: 高级坐标框架转换，支持点云数据
- **增强点云处理**: 鲁棒的错误处理和坐标系转换
- **无人机检测**: 集成无人机检测功能
- **集群协调**: 复杂环境中的多无人机轨迹规划

## 1. 需要的库
* vtk (安装PCL的依赖库，编译时需要勾选Qt)
* PCL (点云库)
* Eigen3 (线性代数库)
* TF2 (ROS2变换库)

## 2. 前置条件
### 2.1 DDS配置
可能是我一些发布订阅的设置写的不太对，使用ROS2默认的FastDDS会导致程序运行很卡，目前还没找到原因，所以请按照下述方法将DDS修改为cyclonedds

#### 安装cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

#### 修改默认的DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

#### 检查是否修改成功
```
ros2 doctor --report | grep "RMW middleware"
```
输出显示rmw_cyclonedds_cpp则说明修改成功

### 2.2 TF2配置
系统现在使用TF2进行坐标转换。确保您的相机/深度传感器框架在URDF或启动文件中正确配置。

### 2.3 点云处理
增强的点云处理功能：
- 从相机坐标系到世界坐标系的自动转换
- 缺失变换时的鲁棒错误处理
- 大规模点云的内存优化

## 3. 代码运行
### 3.1 启动可视化
```
ros2 launch ego_planner rviz.launch.py 
```

### 3.2 运行规划程序
新开一个终端，输入以下指令：

#### 单机模式
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```

#### 集群模式
```
ros2 launch ego_planner swarm.launch.py 
```

#### 大型集群
```
ros2 launch ego_planner swarm_large.launch.py  
```

### 3.3 参数配置
附加参数，可以选择地图生成模式以及是否考虑动力学：
* **use_mockamap**: 地图生成方式
  * 默认为False
  * False时使用Random Forest
  * True时使用mockamap
* **use_dynamic**: 是否考虑动力学
  * 默认为False
  * False时不考虑
  * True时考虑

```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

### 3.4 新增功能
#### TF2坐标转换
系统现在支持自动坐标系转换:
* 相机坐标系到世界坐标系的自动转换
* 支持多种相机框架配置
* 实时TF2监听和变换

#### 增强点云处理
* 改进的错误处理机制
* 坐标系转换失败时的优雅降级
* 内存使用优化
* 支持大规模点云数据处理

#### 无人机检测
* 集成无人机检测功能
* 实时障碍物识别
* 多传感器数据融合

## 4. 技术细节

### 4.1 坐标系转换
本版本在`grid_map.cpp`中实现了完整的TF2坐标系转换：
```cpp
// 获取从相机坐标系到世界坐标系的变换
geometry_msgs::msg::TransformStamped transform;
transform = tf_buffer_->lookupTransform(mp_.frame_id_, source_frame, cloud_time, timeout);

// 使用PCL的高效转换函数
Eigen::Affine3d affine_transform(transform_matrix);
pcl::transformPointCloud(latest_cloud, world_cloud, affine_transform);
```

### 4.2 错误处理
改进了点云处理中的错误处理机制：
```cpp
try {
    pcl::fromROSMsg(*img, latest_cloud);
} catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "点云数据转换失败: %s", e.what());
    return;
}
```

### 4.3 性能优化
- 预分配内存避免重复分配
- 使用全局TF2缓冲区避免重复创建
- 优化的坐标转换算法

## 5. 贡献
本项目基于[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)进行改进，主要贡献包括：

1. **ROS2完整迁移**: 从ROS1迁移到ROS2，支持最新的ROS生态系统
2. **TF2集成**: 添加了完整的TF2坐标系转换支持
3. **错误处理增强**: 改进了点云处理和数据转换的鲁棒性
4. **XTDrone2集成**: 专门为XTDrone2框架优化

## 6. 许可证
本项目遵循原始项目的许可证条款。详情请参考LICENSE文件。

---

# EGO-Planner-Swarm (XTDrone2 Version)

This is an enhanced version of EGO-Planner-Swarm with ROS2 support, TF2 coordinate transformations, and improved point cloud processing for XTDrone2 framework.

## Key Features
- **ROS2 Support**: Full migration to ROS2 with improved performance
- **TF2 Integration**: Advanced coordinate frame transformations for point cloud data
- **Enhanced Point Cloud Processing**: Robust error handling and coordinate system conversions
- **Drone Detection**: Integrated drone detection capabilities
- **Swarm Coordination**: Multi-drone trajectory planning in cluttered environments

## 1. Required Libraries 
* vtk (A dependency library for PCL installation, need to check Qt during compilation)
* PCL (Point Cloud Library)
* Eigen3 (Linear algebra library)
* TF2 (ROS2 transformation library)

## 2. Prerequisites
### 2.1 DDS Configuration
It might be due to some incorrect settings in my publish/subscribe configurations. Using ROS2's default FastDDS causes significant lag during program execution. The reason hasn't been identified yet. Please follow the steps below to change the DDS to cyclonedds.

#### Install cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

#### Change default DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

#### Verify the change
```
ros2 doctor --report | grep "RMW middleware"
```
If the output shows rmw_cyclonedds_cpp, the modification is successful.

### 2.2 TF2 Configuration
The system now uses TF2 for coordinate transformations. Ensure your camera/depth sensor frames are properly configured in your URDF or launch files.

### 2.3 Point Cloud Processing
Enhanced point cloud processing with:
- Automatic coordinate frame transformation from camera to world frame
- Robust error handling for missing transformations
- Optimized memory allocation for large point clouds

## 3. Running the Code
### 3.1 Launch Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 Run the planning program
Open a new terminal and execute:
* Single drone
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* Additional parameters (optional):
    * use_mockamap:Map generation method. Default: False (uses Random Forest), True uses mockamap.
    * use_dynamic:Whether to consider dynamics. Default: False (disabled), True enables dynamics.
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

### 3.4 New Features
#### TF2 Coordinate Transformation
The system now supports automatic coordinate system conversion:
* Automatic conversion from camera coordinate system to world coordinate system
* Support for multiple camera frame configurations
* Real-time TF2 listening and transformation

#### Enhanced Point Cloud Processing
* Improved error handling mechanism
* Graceful degradation when coordinate system conversion fails
* Memory usage optimization
* Support for large-scale point cloud data processing

#### Drone Detection
* Integrated drone detection function
* Real-time obstacle recognition
* Multi-sensor data fusion

## 4. Technical Details

### 4.1 Coordinate Transformation
This version implements complete TF2 coordinate transformation in `grid_map.cpp`:
```cpp
// Get transformation from camera coordinate system to world coordinate system
geometry_msgs::msg::TransformStamped transform;
transform = tf_buffer_->lookupTransform(mp_.frame_id_, source_frame, cloud_time, timeout);

// Use PCL's efficient transformation function
Eigen::Affine3d affine_transform(transform_matrix);
pcl::transformPointCloud(latest_cloud, world_cloud, affine_transform);
```

### 4.2 Error Handling
Improved error handling mechanism in point cloud processing:
```cpp
try {
    pcl::fromROSMsg(*img, latest_cloud);
} catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Point cloud data conversion failed: %s", e.what());
    return;
}
```

### 4.3 Performance Optimization
- Pre-allocated memory to avoid repeated allocation
- Use global TF2 buffer to avoid repeated creation
- Optimized coordinate transformation algorithm

## 5. Contributing
This project is based on [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) with the following major contributions:

1. **Complete ROS2 Migration**: Migrated from ROS1 to ROS2, supporting the latest ROS ecosystem
2. **TF2 Integration**: Added complete TF2 coordinate transformation support
3. **Enhanced Error Handling**: Improved robustness in point cloud processing and data conversion
4. **XTDrone2 Integration**: Specifically optimized for XTDrone2 framework

## 6. License
This project follows the license terms of the original project. Please refer to the LICENSE file for details.