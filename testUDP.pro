QT += quick quick3d opengl
CONFIG += c++17

SOURCES += \
        main.cpp \
        pcl_reader/pclreader.cpp

resources.files = main.qml
resources.prefix = /$${TARGET}
RESOURCES += resources

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

include(D:/PCL/PCL1.13.0/PCL.pri)
# 添加PCL相关库
INCLUDEPATH += $$(PCL_ROOT)/include/pcl-1.13
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/Boost/include/boost-1_80
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/Eigen/eigen3
INCLUDEPATH += $$(PCL_ROOT)/3rdParty/VTK/include/vtk-9.2

# 添加 OpenGL 相关库
win32 {
    LIBS += -lopengl32
}

QMAKE_PROJECT_DEPTH=0
LIBS -= -L"path_to_vtk_libraries" -lvtkWrPCLingTools-9.2-gd
LIBS -= -L"path_to_vtk_libraries" -lvtkWrPCLingTools-9.2

HEADERS += \
    pcl_reader/pclreader.h
