# RealTimePCLViewer

A real-time point cloud visualization software powered by PCL and Qt.  
由PCL和QT驱动的实时点云可视化软件。

## 项目简介

本项目实现了一个实时点云可视化工具，使用UDP协议接收点云数据并通过Qt进行可视化。该工具可以用于展示来自传感器或其他数据源的实时点云数据。

![示例 GIF](images/example.gif)

## 文件结构

```
.
├── pcl_reader
│   ├── pclreader.cpp        // 点云数据处理实现
│   ├── pclreader.h          // 点云数据处理头文件
├── testUDP_py
│   ├── car_road_cloud.py    // 发送车辆道路点云数据的脚本
│   ├── dragon.py            // 发送龙形点云数据的脚本
│   ├── sky.py               // 发送螺旋点云数据的脚本
└── main.qml                 // QML界面文件
```

## 使用说明

### 依赖项

- ****Qt 6.2**** 或更高版本
- ****PCL**** (Point Cloud Library)
- ****Python 3.x****
- ****NumPy****

### 编译和运行

#### CMake构建流程

```cmake
# 点云配置关键

```cmake
# 点云配置关键
find_package(Qt6 6.2 COMPONENTS Quick Quick3D REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
)
```

1. **创建构建目录**  
   在项目根目录下创建一个构建目录：
   ```bash
   mkdir build
   cd build
   ```

2. **运行CMake**  
   使用CMake配置项目：
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release
   ```

3. **编译项目**  
   使用以下命令编译项目：
   ```bash
   cmake --build . --config Release
   ```

4. **启动应用程序**  
   运行编译后的Qt应用程序，应用程序将开始接收UDP数据并显示点云。

#### QMake构建流程

如果使用QMake，确保您的 `.pro` 文件如下或仓库所示：

```propro
# 3D和OpenGL
QT += quick quick3d opengl

# 你的PCL配置
include(D:/PCL/PCL1.13.0/PCL.pri)
INCLUDEPATH += $$(PCL_ROOT)/include/pcl-1.13
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/Boost/include/boost-1_80
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/Eigen/eigen3
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/VTK/include/vtk-9.2

# 添加OpenGL
win32 {
    LIBS += -lopengl32
}
```

### 核心代码讲解

#### 点云处理

在 `pcl_reader/pclreader.cpp` 中，`PointCloudProcessThread` 类负责处理接收到的点云数据。以下是主要功能的简要说明：

- **构造函数**: 初始化线程和运行标志。
- **addPointCloudData**: 将接收到的点云数据添加到处理队列。
- **run**: 线程主循环，持续处理队列中的数据。
- **processData**: 解析接收到的字节数据，提取车辆位置和点云数据。

```cpp
void PointCloudProcessThread::run() {
    while (running_) {
        QByteArray data;
        {
            QMutexLocker locker(&mutex_);  // 加锁访问共享数据
            if (!dataQueue_.isEmpty()) {
                data = dataQueue_.dequeue();  // 取出一个数据包
            }
        }

        if (!data.isEmpty()) {
            try {
                auto cloud = processData(data);        // 处理数据
                emit cloudProcessed(cloud);            // 发送处理完成信号
            } catch (const std::exception& e) {
                emit processError(QString("处理错误: %1").arg(e.what()));  // 发送错误信号
            }
        }

        QThread::msleep(1);  // 短暂休眠，避免CPU占用过高
    }
}
```

#### QML界面

在 `main.qml` 中，使用Qt Quick构建用户界面，提供实时点云的可视化和控制功能。用户可以通过界面控制相机、启动UDP监听、清除点云等。

### UDP数据发送

在 `testUDP_py` 目录下，提供了多个Python脚本用于模拟UDP数据发送。每个脚本生成不同类型的点云数据并通过UDP发送到Qt应用程序。

- **car_road_cloud.py**: 发送车辆在道路上的点云数据。暂时预留了车辆朝向代码，但UDP包里并没发送，因为展示模型是个正方体用不上，后续可以更改，注释较为全面。另，车辆坐标系转化暂不需要，如果能够确保你UDP发送的坐标点能转换，但也预留，仅供参考。
- **dragon.py**: 发送龙形状的点云数据。
- **sky.py**: 发送螺旋形状的点云数据。

## 示例

在运行UDP发送脚本后，您将看到实时更新的点云数据在Qt应用程序中显示。

## 贡献

欢迎任何形式的贡献！如果您有任何建议或问题，请随时提出。

## 许可证

本项目采用MIT许可证，详细信息请查看LICENSE文件。
