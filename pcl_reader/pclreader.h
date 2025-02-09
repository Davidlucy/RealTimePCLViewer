//  接受格式
//     报文头部（12字节）：
//     float carX (4字节)
//     float carY (4字节)
//     float carZ (4字节)
//     点云数据（每个点16字节）：
//     float x (4字节)
//     float y (4字节)
//     float z (4字节)
//     float padding (4字节，填充0) 必要的，保证数据按照4字节16字节等对齐，提高GPU以及图形处理能力

// 数据包结构示意
// struct PacketFormat {
//     // 1. 车辆位置 (12字节)
//     float carX;    // 0-3字节
//     float carY;    // 4-7字节
//     float carZ;    // 8-11字节
//     后续可添加车辆朝向
//     // 2. 点云数据 (每个点16字节)
//     struct PointData {
//         float x;       // 点的X坐标
//         float y;       // 点的Y坐标
//         float z;       // 点的Z坐标
//         float padding; // 填充数据（通常为0）
//     } points[N];  // N个点
// };


// PCLReader.h - 点云数据处理和显示的头文件
#ifndef PCLREADER_H
#define PCLREADER_H

// Windows API 相关头文件（需要在 Qt 头文件之前包含）
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// Qt相关头文件
#include <QtQuick3D/QQuick3DGeometry>  // 用于3D几何体显示
#include <QtGui/QVector3D>             // 3D向量
#include <QtGui/QMatrix4x4>            // 4x4矩阵，用于变换
#include <QObject>                     // Qt基础对象
#include <QMutex>                      // 互斥锁，用于线程同步
#include <QQueue>                      // 队列容器
#include <QUdpSocket>                  // UDP套接字
#include <QThread>                     // 线程类
#include <QDateTime>                   // 日期类

// PCL点云库头文件
#include <pcl/point_cloud.h>           // 点云数据结构
#include <pcl/point_types.h>           // 点类型定义

#include <memory>                      // 智能指针
#include <vector>                      // 标准库向量

// 点云处理线程类，负责在后台处理接收到的点云数据
class PointCloudProcessThread : public QThread {
    Q_OBJECT  // Qt元对象宏，支持信号槽机制
public:
    explicit PointCloudProcessThread(QObject* parent = nullptr);
    void addPointCloudData(const QByteArray& data);  // 添加待处理的点云数据
    void stop();  // 停止线程

signals:
    // 处理完成后发送的信号
    void cloudProcessed(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);  // 点云处理完成信号
    void processError(const QString& error);  // 处理错误信号
    void vehiclePositionReceived(float x, float y, float z);  // 车辆位置信号

protected:
    void run() override;  // 线程主循环函数

private:
    QMutex mutex_;              // 互斥锁，保护数据队列
    QQueue<QByteArray> dataQueue_;  // 待处理数据队列
    bool running_;              // 线程运行标志

    // 处理单个数据包的函数
    pcl::PointCloud<pcl::PointXYZ>::Ptr processData(const QByteArray& data);
};

// 主点云读取器类，继承自QQuick3DGeometry用于3D显示
class PCLReader : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(PCLReader)  // 注册为QML元素

    // Qt属性声明，用于QML交互
    Q_PROPERTY(float carX READ carX NOTIFY carPositionChanged)  // 车辆X坐标
    Q_PROPERTY(float carY READ carY NOTIFY carPositionChanged)  // 车辆Y坐标
    Q_PROPERTY(float carZ READ carZ NOTIFY carPositionChanged)  // 车辆Z坐标
    Q_PROPERTY(float vehicleHeading READ vehicleHeading WRITE setVehicleHeading NOTIFY vehicleHeadingChanged)  // 车辆朝向
    Q_PROPERTY(bool isLoading READ isLoading NOTIFY isLoadingChanged)  // 加载状态
    Q_PROPERTY(float memoryUsage READ memoryUsage NOTIFY memoryUsageChanged)
    Q_PROPERTY(float memoryThreshold READ memoryThreshold WRITE setMemoryThreshold NOTIFY memoryThresholdChanged)

public:
    explicit PCLReader();
    ~PCLReader();

    // 属性访问函数
    float carX() const { return carX_; }
    float carY() const { return carY_; }
    float carZ() const { return carZ_; }
    float vehicleHeading() const { return vehicleHeading_; }
    bool isLoading() const { return isLoading_; }
    float memoryUsage() const { return currentMemoryUsage_; }

    // 获取和设置内存阈值
    float memoryThreshold() const { return memory_usage_threshold_; }
    void setMemoryThreshold(float threshold) {
        if (memory_usage_threshold_ != threshold) {
            memory_usage_threshold_ = threshold;
            emit memoryThresholdChanged();
        }
    }

public slots:
    void startUDPListener(int port);  // 启动UDP监听
    void stopUDPListener();           // 停止UDP监听
    void setVehiclePosition(float x, float y, float z);  // 设置车辆位置
    void setVehicleHeading(float heading);               // 设置车辆朝向
    void handleLoadedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);  // 处理加载的点云
    void handleVehiclePosition(float x, float y, float z);  // 处理车辆位置更新
    void clearPoints();  // 清除点云数据

signals:
    // 信号声明
    void carPositionChanged();        // 车辆位置改变信号
    void errorOccurred(const QString &error);  // 错误发生信号
    void vehicleHeadingChanged();     // 车辆朝向改变信号
    void isLoadingChanged();          // 加载状态改变信号
    void memoryUsageChanged();        // 内存改变信号
    void memoryThresholdChanged();    // 内存阈值改变信号

private slots:
    void processPendingDatagrams();   // 处理待处理的UDP数据包

private:
    // 顶点数据结构
    struct VertexData {
        QVector3D position;  // 点位置
        QVector3D color;     // 点颜色
        float padding;       // 填充对齐
    };

    // 私有成员函数
    void updateGeometry(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_points);  // 更新几何体
    void updateCurrentTransform();  // 更新当前变换矩阵
    bool canGrowBuffer() const;     // 检查是否可以增长缓冲区
    void growBuffer(size_t required_size);  // 增长缓冲区
    float getSystemMemoryUsage() const;     // 获取系统内存使用率
    bool canEmergencyDownsample() const;  // 检查是否可以进行紧急降采样 没用上
    void updateMemoryUsage();          // 更新内存使用率

private:
    // 私有成员变量
    QByteArray vertexBuffer_;         // 顶点缓冲区
    size_t currentBufferPosition_ = 0; // 当前缓冲区位置
    bool firstTime = true;            // 首次更新标志
    bool attributesSet_ = false;      // 属性设置标志
    QMutex geometryMutex_;           // 几何体互斥锁
    QMatrix4x4 currentTransform_;     // 当前变换矩阵

    // 缓冲区管理相关
    const size_t INITIAL_MAX_POINTS = 100000;  // 初始最大点数（10万）
    const size_t MAX_GROWTH_LIMIT = 200000;   // 最大增长限制（20万或者当前根据内存分配的值）超过后对最开始加入的点做平均稀疏以满足点的数量最大在current_max_points_
    size_t current_max_points_;                  // 当前最大点数
    float memory_usage_threshold_;               // 内存使用阈值（百分比）

    QElapsedTimer emergencyDownsampleTimer_;  // 紧急降采样计时器
    const int EMERGENCY_DOWNSAMPLE_COOLDOWN = 20000;  // 冷却时间20秒(毫秒)

    // 车辆状态相关变量
    float carX_ = 0.0f;              // 车辆X坐标
    float carY_ = 0.0f;              // 车辆Y坐标
    float carZ_ = 0.0f;              // 车辆Z坐标
    float vehicleHeading_ = 0.0f;     // 车辆朝向

    // 处理相关对象
    PointCloudProcessThread* processThread_;  // 点云处理线程
    QUdpSocket* udpSocket_;                  // UDP套接字
    bool isLoading_ = false;                 // 加载状态标志
    float currentMemoryUsage_ = 0.0f;       // 当前内存使用率
};

#endif // PCLREADER_H

