# RealTimePCLViewer / 实时点云可视化工具

A real-time point cloud visualization software powered by PCL and Qt.  
由PCL和QT驱动的实时点云可视化软件。

## 项目简介 / Project Introduction

本项目实现了一个实时点云可视化工具，使用UDP协议接收点云数据并通过Qt进行可视化。该工具可以用于展示来自传感器或其他数据源的实时点云数据。  
This project implements a real-time point cloud visualization tool that receives point cloud data via UDP protocol and visualizes it using Qt. This tool can be used to display real-time point cloud data from sensors or other data sources.

![示例 GIF / Example GIF](images/example.gif)

## 文件结构 / File Structure

```
.
├── pcl_reader
│   ├── pclreader.cpp        // 点云数据处理实现 / Point cloud data processing implementation
│   ├── pclreader.h          // 点云数据处理头文件 / Point cloud data processing header file
├── testUDP_py
│   ├── car_road_cloud.py    // 发送车辆道路点云数据的脚本 / Script to send vehicle road point cloud data
│   ├── dragon.py            // 发送龙形点云数据的脚本 / Script to send dragon-shaped point cloud data
│   ├── sky.py               // 发送螺旋点云数据的脚本 / Script to send spiral-shaped point cloud data
└── main.qml                 // QML界面文件 / QML interface file
```

## 使用说明 / Usage Instructions

### 依赖项 / Dependencies

- ****Qt 6.2**** 或更高版本 / ****Qt 6.2**** or higher
- ****PCL**** (Point Cloud Library)
- ****Python 3.x****
- ****NumPy****

### 编译和运行 / Compilation and Execution

#### CMake构建流程 / CMake Build Process

```cmake
# 点云配置关键 / Key point cloud configuration

```cmake
# 点云配置关键 / Key point cloud configuration
find_package(Qt6 6.2 COMPONENTS Quick Quick3D REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
)
```

1. **创建构建目录** / **Create build directory**  
   在项目根目录下创建一个构建目录： / Create a build directory in the project root directory:
   ```bash
   mkdir build
   cd build
   ```

2. **运行CMake** / **Run CMake**  
   使用CMake配置项目： / Configure the project using CMake:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release
   ```

3. **编译项目** / **Compile the project**  
   使用以下命令编译项目： / Compile the project using the following command:
   ```bash
   cmake --build . --config Release
   ```

4. **启动应用程序** / **Start the application**  
   运行编译后的Qt应用程序，应用程序将开始接收UDP数据并显示点云。 / Run the compiled Qt application, the application will start receiving UDP data and displaying the point cloud.

#### QMake构建流程 / QMake Build Process

如果使用QMake，确保您的 `.pro` 文件如下或仓库所示： / If you are using QMake, make sure your `.pro` file is as follows or as shown in the repository:

```propro
# 3D和OpenGL / 3D and OpenGL
QT += quick quick3d opengl

# 你的PCL配置 / Your PCL configuration
include(D:/PCL/PCL1.13.0/PCL.pri)
INCLUDEPATH += $$(PCL_ROOT)/include/pcl-1.13
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/Boost/include/boost-1_80
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/Eigen/eigen3
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/VTK/include/vtk-9.2

# 添加OpenGL / Add OpenGL
win32 {
    LIBS += -lopengl32
}
```

### 核心代码讲解 / Core Code Explanation

#### 点云处理 / Point Cloud Processing

在 `pcl_reader/pclreader.cpp` 中，`PointCloudProcessThread` 类负责处理接收到的点云数据。以下是主要功能的简要说明： / In `pcl_reader/pclreader.cpp`, the `PointCloudProcessThread` class is responsible for processing the received point cloud data. Here is a brief description of the main functions:

- **构造函数**: 初始化线程和运行标志。 / **Constructor**: Initialize the thread and the run flag.
- **addPointCloudData**: 将接收到的点云数据添加到处理队列。 / **addPointCloudData**: Add the received point cloud data to the processing queue.
- **run**: 线程主循环，持续处理队列中的数据。 / **run**: The main loop of the thread, continuously processing the data in the queue.
- **processData**: 解析接收到的字节数据，提取车辆位置和点云数据。 / **processData**: Parse the received byte data, extract the vehicle position and point cloud data.

```cpp
void PointCloudProcessThread::run() {
    while (running_) {
        QByteArray data;
        {
            QMutexLocker locker(&mutex_);  // 加锁访问共享数据 / Lock to access shared data
            if (!dataQueue_.isEmpty()) {
                data = dataQueue_.dequeue();  // 取出一个数据包 / Take out a data packet
            }
        }

        if (!data.isEmpty()) {
            try {
                auto cloud = processData(data);        // 处理数据 / Process data
                emit cloudProcessed(cloud);            // 发送处理完成信号 / Send processing complete signal
            } catch (const std::exception& e) {
                emit processError(QString("处理错误: %1").arg(e.what()));  // 发送错误信号 / Send error signal
            }
        }

        QThread::msleep(1);  // 短暂休眠，避免CPU占用过高 / Brief sleep to avoid high CPU usage
    }
}
```

#### QML界面 / QML Interface

在 `main.qml` 中，使用Qt Quick构建用户界面，提供实时点云的可视化和控制功能。用户可以通过界面控制相机、启动UDP监听、清除点云等。 / In `main.qml`, use Qt Quick to build the user interface, providing real-time point cloud visualization and control functions. Users can control the camera, start UDP listening, clear the point cloud, etc. through the interface.

### UDP数据发送 / UDP Data Transmission

在 `testUDP_py` 目录下，提供了多个Python脚本用于模拟UDP数据发送。每个脚本生成不同类型的点云数据并通过UDP发送到Qt应用程序。 / In the `testUDP_py` directory, there are multiple Python scripts provided to simulate UDP data transmission. Each script generates different types of point cloud data and sends it to the Qt application via UDP.

- **car_road_cloud.py**: 发送车辆在道路上的点云数据。暂时预留了车辆朝向代码，但UDP包里并没发送，因为展示模型是个正方体用不上，后续可以更改，注释较为全面。另，车辆坐标系转化暂不需要，如果能够确保你UDP发送的坐标点能转换，但也预留，仅供参考。 / **car_road_cloud.py**: Send point cloud data of vehicles on the road. The orientation code of the vehicle is temporarily reserved, but not sent in the UDP packet, because the display model is a cube and not used, it can be changed later, and the comments are more comprehensive. Also, the conversion of the vehicle coordinate system is not needed for the time being. If you can ensure that the coordinate points sent by your UDP can be converted, but also reserved, for reference only.
- **dragon.py**: 发送龙形状的点云数据。 / **dragon.py**: Send dragon-shaped point cloud data.
- **sky.py**: 发送螺旋形状的点云数据。 / **sky.py**: Send spiral-shaped point cloud data.

## 示例 / Example

在运行UDP发送脚本后，您将看到实时更新的点云数据在Qt应用程序中显示。 / After running the UDP transmission script, you will see the real-time updated point cloud data displayed in the Qt application.

## 贡献 / Contribution

欢迎任何形式的贡献！如果您有任何建议或问题，请随时提出。 / Welcome any form of contribution! If you have any suggestions or questions, please feel free to ask.

## 许可证 / License

本项目采用MIT许可证，详细信息请查看LICENSE文件。 / This project is licensed under the MIT License, please see the LICENSE file for details.
