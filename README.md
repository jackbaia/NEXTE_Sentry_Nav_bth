# MID ROS 项目

这是一个基于ROS Noetic的机器人导航和定位系统，集成了多种传感器和算法。

## 项目概述

本项目包含以下主要功能模块：

- **FAST-LIO**: 激光雷达惯性里程计
- **FAST-LIO Localization**: 基于FAST-LIO的定位系统
- **Livox ROS Driver**: Livox激光雷达驱动
- **Lidar Obstacle Detector**: 激光雷达障碍物检测
- **Navigation Converter**: 导航指令转换器
- **Sentry Navigation**: 哨兵机器人导航系统
- **Velocity Smoother**: 速度平滑器

## 系统要求

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- C++14 或更高版本

## 安装依赖

```bash
# 安装ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop-full

# 安装其他依赖
sudo apt install python3-pip python3-catkin-tools
pip3 install numpy scipy
```

## 编译项目

```bash
# 克隆项目
git clone <your-repository-url>
cd mid_ros

# 编译
catkin_make

# 或者使用catkin tools
catkin build
```

## 使用方法

### 1. 启动激光雷达驱动

```bash
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 2. 启动FAST-LIO建图

```bash
roslaunch fast_lio mapping_mid360.launch
```

### 3. 启动定位系统

```bash
roslaunch fast_lio_localization sentry_localize.launch
```

### 4. 启动导航系统

```bash
roslaunch sentry_nav sentry_movebase_tunnel.launch
```

## 项目结构

```
mid_ros/
├── src/
│   ├── FAST_LIO/                 # FAST-LIO建图
│   ├── FAST_LIO_LOCALIZATION/    # FAST-LIO定位
│   ├── livox_ros_driver2/        # Livox激光雷达驱动
│   ├── lidar_obstacle_detector/  # 障碍物检测
│   ├── nav_converter/            # 导航转换器
│   ├── sentry_nav/               # 哨兵导航
│   ├── sentry_serial/            # 串口通信
│   └── velocity_smoother_ema/    # 速度平滑器
├── build/                        # 编译目录
├── devel/                        # 开发环境
└── README.md                     # 项目说明
```

## 配置参数

各模块的配置文件位于对应的 `config/` 目录下：

- `FAST_LIO/config/`: FAST-LIO参数配置
- `nav_converter/param/`: 导航转换器参数
- `sentry_nav/param/`: 导航参数

## 故障排除

### 常见问题

1. **global_localization节点崩溃**
   - 已修复numpy兼容性问题
   - 确保ros_numpy正确安装

2. **编译错误**
   - 检查ROS环境是否正确设置
   - 确保所有依赖已安装

3. **传感器连接问题**
   - 检查USB连接
   - 确认设备权限

## 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 联系方式

如有问题或建议，请通过以下方式联系：

- 项目Issues: [GitHub Issues](your-repo-url/issues)
- 邮箱: your-email@example.com 