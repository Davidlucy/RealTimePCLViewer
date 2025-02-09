#include "pclreader.h"
#include <QDebug>

// PointCloudProcessThread 类的构造函数
PointCloudProcessThread::PointCloudProcessThread(QObject* parent)
    : QThread(parent), running_(true)
{
}

// 添加点云数据到处理队列
void PointCloudProcessThread::addPointCloudData(const QByteArray& data)
{
    QMutexLocker locker(&mutex_);  // 自动加锁和解锁
    dataQueue_.enqueue(data);      // 将数据添加到队列
}

// 停止线程
void PointCloudProcessThread::stop()
{
    QMutexLocker locker(&mutex_);
    running_ = false;  // 设置停止标志
}

// 线程主循环
void PointCloudProcessThread::run()
{
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessThread::processData(const QByteArray& data)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 检查数据大小是否合法
    if (data.size() < (3 + 4) * sizeof(float)) {
        return cloud;
    }

    // 将字节数组转换为float指针
    const float* dataPtr = reinterpret_cast<const float*>(data.constData());

    // 读取车辆位置（前3个float）
    float carX = dataPtr[0];
    float carY = dataPtr[1];
    float carZ = dataPtr[2];

    // 发送车辆位置更新信号
    emit vehiclePositionReceived(carX, carY, carZ);

    // 计算点云数据
    const float* points = dataPtr + 3;  // 跳过车辆位置数据
    int numPoints = (data.size() - 3 * sizeof(float)) / (4 * sizeof(float));  // 计算点的数量

    cloud->points.reserve(numPoints);  // 预分配内存

    // 解析每个点的数据
    for (int i = 0; i < numPoints; i++) {
        pcl::PointXYZ point;
        point.x = points[i * 4];     // X坐标
        point.y = points[i * 4 + 1]; // Y坐标
        point.z = points[i * 4 + 2]; // Z坐标
        // points[i * 4 + 3] 是padding，被跳过
        cloud->points.push_back(point);
    }

    return cloud;
}

// PCLReader类构造函数
PCLReader::PCLReader()
    : currentBufferPosition_(0)
    , firstTime(true)
    , attributesSet_(false)
    , processThread_(nullptr)
    , udpSocket_(nullptr)
    , isLoading_(false)
    , current_max_points_(INITIAL_MAX_POINTS)
    , memory_usage_threshold_(80.0f)  // 设置80%的内存使用阈值
{
    emergencyDownsampleTimer_.start();  // 启动计时器
    vertexBuffer_.resize(current_max_points_ * sizeof(VertexData));
    updateCurrentTransform();
}

// 析构函数
PCLReader::~PCLReader()
{
    stopUDPListener();  // 停止UDP监听
}

// 启动UDP监听器
void PCLReader::startUDPListener(int port)
{
    // 创建UDP套接字
    if (!udpSocket_) {
        udpSocket_ = new QUdpSocket(this);
    }

    // 创建并启动处理线程
    if (!processThread_) {
        processThread_ = new PointCloudProcessThread(this);
        // 连接信号和槽
        connect(processThread_, &PointCloudProcessThread::cloudProcessed,
                this, &PCLReader::handleLoadedCloud);
        connect(processThread_, &PointCloudProcessThread::processError,
                this, &PCLReader::errorOccurred);
        connect(processThread_, &PointCloudProcessThread::vehiclePositionReceived,
                this, &PCLReader::handleVehiclePosition);
        processThread_->start();
    }

    // 绑定UDP端口
    if (!udpSocket_->bind(QHostAddress::Any, port)) {
        emit errorOccurred(QString("无法绑定到端口 %1").arg(port));
        return;
    }

    // 连接数据接收信号
    connect(udpSocket_, &QUdpSocket::readyRead,
            this, &PCLReader::processPendingDatagrams);
}

// 停止UDP监听器
void PCLReader::stopUDPListener()
{
    // 停止并清理处理线程
    if (processThread_) {
        processThread_->stop();
        processThread_->wait();
        delete processThread_;
        processThread_ = nullptr;
    }

    // 关闭并清理UDP套接字
    if (udpSocket_) {
        udpSocket_->close();
        delete udpSocket_;
        udpSocket_ = nullptr;
    }
}

// 处理待处理的UDP数据包
void PCLReader::processPendingDatagrams()
{
    while (udpSocket_->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket_->pendingDatagramSize());
        udpSocket_->readDatagram(datagram.data(), datagram.size());

        if (processThread_) {
            processThread_->addPointCloudData(datagram);  // 添加数据到处理队列
        }
    }
}

// 清除点云数据
void PCLReader::clearPoints(){

    QMutexLocker locker(&geometryMutex_);  // 加锁保护
    currentBufferPosition_ = 0;             // 重置缓冲区位置
    vertexBuffer_.clear();                  // 清空缓冲区

    // 使用初始大小重新分配内存
    current_max_points_ = INITIAL_MAX_POINTS;  // 重置为初始大小
    vertexBuffer_.resize(current_max_points_ * sizeof(VertexData));  // 重新分配内存
        // 强制更新几何体数据
    setVertexData(vertexBuffer_);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    setStride(sizeof(VertexData));
        // 重新设置属性
    if (!attributesSet_) {
        addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                     offsetof(VertexData, position),
                     QQuick3DGeometry::Attribute::F32Type);
        addAttribute(QQuick3DGeometry::Attribute::ColorSemantic,
                     offsetof(VertexData, color),
                     QQuick3DGeometry::Attribute::F32Type);
        attributesSet_ = true;
    }
    // 强制更新边界
    setBounds(QVector3D(-1000, -1000, -1000), QVector3D(1000, 1000, 1000));
    update();

    qDebug() << "Points cleared. Buffer reset to initial size:" << current_max_points_;
}

void PCLReader::updateGeometry(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_points)
{
    QMutexLocker locker(&geometryMutex_);

    size_t newPoints = pcd_points->points.size();
    size_t totalRequired = currentBufferPosition_ + newPoints;
    float memoryUsage = getSystemMemoryUsage();

    // 显示当前状态
    qDebug() << "\n  点云状态更新:"
             << "\n  当前点数:" << currentBufferPosition_
             << "\n  新增点数:" << newPoints
             << "\n  需要总点数:" << totalRequired
             << "\n  当前最大容量:" << current_max_points_
             << "\n  内存使用率:" << memoryUsage << "%";

    // 检查是否需要更多空间
    if (totalRequired > current_max_points_) {
        qDebug() << "\n  需要扩展缓冲区:"
                 << "\n  所需点数:" << totalRequired
                 << "\n  当前容量:" << current_max_points_;

        // 首先尝试增长缓冲区
        if (canGrowBuffer()) {
            growBuffer(totalRequired);
            qDebug() << "  缓冲区增长后容量:" << current_max_points_;
        }

        // 如果增长后仍然空间不足，进行降采样
        if (totalRequired > current_max_points_) {
            int step = 10;  // 每10个点保留9个点

            VertexData* vertexData = reinterpret_cast<VertexData*>(vertexBuffer_.data());
            size_t newPosition = 0;

            for (size_t i = 0; i < currentBufferPosition_; i++) {
                if (i % step != 9) {  // 每10个点中保留9个点
                    vertexData[newPosition] = vertexData[i];
                    newPosition++;
                }
            }

            size_t removedPoints = currentBufferPosition_ - newPosition;
            currentBufferPosition_ = newPosition;

            qDebug() << "\n  降采样完成:"
                     << "\n  原有点数:" << currentBufferPosition_ + removedPoints
                     << "\n  降采样后点数:" << currentBufferPosition_
                     << "\n  移除点数:" << removedPoints
                     << "\n  降采样步长:" << step;
        }
    }

    // 添加新的点云数据
    VertexData* vertexData = reinterpret_cast<VertexData*>(vertexBuffer_.data());
    for (size_t i = 0; i < newPoints; ++i) {
        const auto& point = pcd_points->points[i];
        QVector3D transformedPoint = currentTransform_.map(QVector3D(point.x, point.y, point.z));

        vertexData[currentBufferPosition_].position = transformedPoint;
        vertexData[currentBufferPosition_].color = QVector3D(0, 0.5, 1);
        vertexData[currentBufferPosition_].padding = 0.0f;

        currentBufferPosition_++;
    }

    // 更新几何体属性
    setVertexData(vertexBuffer_);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    setStride(sizeof(VertexData));

    if (!attributesSet_) {
        addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                     offsetof(VertexData, position),
                     QQuick3DGeometry::Attribute::F32Type);

        addAttribute(QQuick3DGeometry::Attribute::ColorSemantic,
                     offsetof(VertexData, color),
                     QQuick3DGeometry::Attribute::F32Type);

        attributesSet_ = true;
    }

    setBounds(QVector3D(-1000, -1000, -1000), QVector3D(1000, 1000, 1000));
    update();
        // 显示最终状态
    qDebug() << "\n更新完成:"
             << "\n  最终点数:" << currentBufferPosition_
             << "\n  缓冲区容量:" << current_max_points_
             << "\n  缓冲区使用率:" << (currentBufferPosition_ * 100.0f / current_max_points_) << "%"
             << "\n  系统内存使用率:" << getSystemMemoryUsage() << "%"
             << "\n----------------------------------------";

    // 在适当的位置更新内存使用率
    updateMemoryUsage();
}

// 添加新的辅助函数
float PCLReader::getSystemMemoryUsage() const {
#ifdef Q_OS_WIN
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    if (GlobalMemoryStatusEx(&memInfo)) {
        return static_cast<float>(memInfo.dwMemoryLoad);
    }
#endif
    return 0.0f;
}

bool PCLReader::canGrowBuffer() const {
    float memoryUsage = getSystemMemoryUsage();
    bool canGrow = current_max_points_ < MAX_GROWTH_LIMIT &&
                   memoryUsage > 0 &&
                   memoryUsage < memory_usage_threshold_;

    if (!canGrow) {
        qDebug() << "Buffer growth check:"
                 << "Current points:" << current_max_points_
                 << "Memory usage:" << memoryUsage << "%"
                 << "Can grow:" << canGrow;
    }

    return canGrow;
}

void PCLReader::growBuffer(size_t required_size) {
    // 固定增加100万点
    size_t new_size = current_max_points_ + INITIAL_MAX_POINTS;
    
    // 确保不超过最大限制
    new_size = std::min(new_size, MAX_GROWTH_LIMIT);

    try {
        QByteArray newBuffer;
        newBuffer.resize(new_size * sizeof(VertexData));

        if (currentBufferPosition_ > 0) {
            memcpy(newBuffer.data(), vertexBuffer_.data(),
                   currentBufferPosition_ * sizeof(VertexData));
        }

        vertexBuffer_ = std::move(newBuffer);
        current_max_points_ = new_size;

        qDebug() << "缓冲区增长:"
                 << "\n  原大小:" << (new_size - INITIAL_MAX_POINTS) << "点"
                 << "\n  增加了:" << INITIAL_MAX_POINTS << "点"
                 << "\n  新大小:" << new_size << "点"
                 << "\n  内存使用:" << getSystemMemoryUsage() << "%";
                 
        // 如果增长后仍然不够，提示可能需要多次增长
        if (new_size < required_size) {
            qDebug() << "注意: 当前空间仍不足，需要继续增长"
                     << "\n  还需要:" << (required_size - new_size) << "点";
        }
    } catch (const std::bad_alloc&) {
        qWarning() << "内存分配失败 - 无法增长缓冲区";
        throw;
    } catch (...) {
        qWarning() << "未知错误 - 无法增长缓冲区";
        throw;
    }
}


// 设置车辆位置
void PCLReader::setVehiclePosition(float x, float y, float z)
{
    if (x != carX_ || y != carY_ || z != carZ_) {
        carX_ = x;
        carY_ = y;
        carZ_ = z;
        //updateCurrentTransform();  // 更新变换矩阵
        emit carPositionChanged(); // 发送位置改变信号
    }
}

// 设置车辆朝向
void PCLReader::setVehicleHeading(float heading)
{
    if (heading != vehicleHeading_) {
        vehicleHeading_ = heading;
        //updateCurrentTransform();      // 更新变换矩阵
        emit vehicleHeadingChanged();  // 发送朝向改变信号
    }
}

// 更新当前变换矩阵
void PCLReader::updateCurrentTransform()
{
    currentTransform_.setToIdentity();  // 重置为单位矩阵
    currentTransform_.translate(carX_, carY_, carZ_);  // 平移
    currentTransform_.rotate(vehicleHeading_ * 180.0f / M_PI, 0, 0, 1);  // 旋转
}

// 处理加载的点云数据
void PCLReader::handleLoadedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (!cloud || cloud->empty()) {
        return;
    }
    updateGeometry(cloud);  // 更新几何体
}

// 处理车辆位置更新
void PCLReader::handleVehiclePosition(float x, float y, float z)
{
    setVehiclePosition(x, y, z);  // 更新车辆位置
}

// 添加检查函数
bool PCLReader::canEmergencyDownsample() const {
    return emergencyDownsampleTimer_.elapsed() >= EMERGENCY_DOWNSAMPLE_COOLDOWN;
}

void PCLReader::updateMemoryUsage() {
    float newUsage = getSystemMemoryUsage();
    if (std::abs(newUsage - currentMemoryUsage_) > 0.1f) {  // 只在变化超过0.1%时更新
        currentMemoryUsage_ = newUsage;
        emit memoryUsageChanged();
    }
}
