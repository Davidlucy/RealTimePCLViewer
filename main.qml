import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import QtQuick.Controls
import QtQuick.Layouts
import PCDReader 1.0

Window {
    width: 1280
    height: 800
    visible: true
    title: qsTr("实时点云显示")

    property bool isSimulating: false
    property real simulationTime: 0
    property real simulationSpeed: 0.05
    property real circleRadius: 50

    property bool cameraLocked: true
    property vector3d cameraRotation: Qt.vector3d(0, 0, 0.1)
    property real defaultDistance: 5
    property real currentDistance: defaultDistance
    property real minZoom: 0.01
    property real maxZoom: 1000.0
    property real zoomSpeed: 1.15

    Timer {
        id: movementTimer
        interval: 50
        running: isSimulating
        repeat: true
        onTriggered: {
            simulationTime += simulationSpeed
            var newX = circleRadius * Math.cos(simulationTime)
            var newY = circleRadius * Math.sin(simulationTime)
            var heading = simulationTime + Math.PI/2
            pcdReader.setVehiclePosition(newX, newY, 0)
            pcdReader.vehicleHeading = heading
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        property point lastPos: Qt.point(0, 0)
        property bool isPanning: false
        property real rotationSpeed: 0.1

        onPressed: (mouse) => {
                       lastPos = Qt.point(mouse.x, mouse.y)
                       if (mouse.button === Qt.MiddleButton ||
                           (mouse.button === Qt.LeftButton && mouse.modifiers & Qt.ShiftModifier)) {
                           isPanning = true
                       }
                   }

        onReleased: (mouse) => {
                        isPanning = false
                    }

        onPositionChanged: (mouse) => {
                               if (true) {
                                   let dx = mouse.x - lastPos.x
                                   let dy = mouse.y - lastPos.y

                                   if (mouse.buttons & Qt.LeftButton && !isPanning) {
                                       cameraRotation.y += dx * rotationSpeed
                                       if (cameraRotation.y > 180) cameraRotation.y -= 360
                                       if (cameraRotation.y < -180) cameraRotation.y += 360
                                       mainCamera.eulerRotation = cameraRotation
                                   }
                                   else if (mouse.buttons & Qt.RightButton) {
                                       if (mouse.modifiers & Qt.ShiftModifier) {
                                           cameraRotation.z += dx * rotationSpeed
                                           if (cameraRotation.z > 180) cameraRotation.z -= 360
                                           if (cameraRotation.z < -180) cameraRotation.z += 360
                                       } else {
                                           let newRotationX = cameraRotation.x - dy * rotationSpeed
                                           cameraRotation.x = Math.max(-90, Math.min(90, newRotationX))
                                       }
                                       mainCamera.eulerRotation = cameraRotation
                                   }
                                   else if (isPanning) {
                                       let panSpeed = currentDistance / 1000.0
                                       panSpeed = Math.max(0.1, panSpeed)

                                       let angle = cameraRotation.y * Math.PI / 180
                                       let cosAngle = Math.cos(angle)
                                       let sinAngle = Math.sin(angle)

                                       let deltaX = (dx * cosAngle + dy * sinAngle) * panSpeed
                                       let deltaY = (-dx * sinAngle + dy * cosAngle) * panSpeed

                                       cameraNode.x -= deltaX
                                       cameraNode.y -= deltaY
                                   }
                                   lastPos = Qt.point(mouse.x, mouse.y)
                               }
                           }

        onWheel: (wheel) => {
            if (!cameraLocked) {
                let currentZoomSpeed = zoomSpeed
                let distanceScale = Math.log(currentDistance) / Math.log(10)
                currentZoomSpeed = 1 + (distanceScale * 0.1)

                if (wheel.angleDelta.y > 0) {
                    if (currentDistance > minZoom) {
                        currentDistance /= currentZoomSpeed
                    }
                } else {
                    if (currentDistance < maxZoom) {
                        currentDistance *= currentZoomSpeed
                    }
                }
            }
        }
    }

    View3D {
        id: view3D
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: "darkgray"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: sceneRoot

            Model {
                id: vehicleCube
                source: "#Cube"
                scale: Qt.vector3d(0.005, 0.005, 0.005)
                position: Qt.vector3d(pcdReader.carX, pcdReader.carY, pcdReader.carZ)
                eulerRotation: Qt.vector3d(0, 0, pcdReader.vehicleHeading * 180 / Math.PI)
                materials: [
                    DefaultMaterial {
                        diffuseColor: "red"
                        opacity: 0.7
                    }
                ]
            }

            Node {
                id: cameraNode
                position: {
                    if (cameraLocked) {
                        return Qt.vector3d(pcdReader.carX, pcdReader.carY, pcdReader.carZ + defaultDistance)
                    } else {
                        return Qt.vector3d(x, y, currentDistance)
                    }
                }

                PerspectiveCamera {
                    id: mainCamera
                    clipNear: 0.01
                    clipFar: 20000.0
                    fieldOfView: 60

                    eulerRotation: {
                        if (cameraLocked) {
                            return Qt.vector3d(0, 0, 1)
                        } else {
                            return cameraRotation
                        }
                    }
                }
            }

            DirectionalLight {
                position: Qt.vector3d(0, 0, 400)
                color: "white"
                ambientColor: Qt.rgba(0.1, 0.1, 0.1, 1.0)
                brightness: 100
            }

            Model {
                id: grid
                scale: Qt.vector3d(50, 50, 1)
                geometry: GridGeometry {
                    horizontalLines: 200
                    verticalLines: 200
                }
                materials: [
                    DefaultMaterial {
                        diffuseColor: "gray"
                        lighting: DefaultMaterial.NoLighting
                    }
                ]
            }

            Model {
                id: pcd_model
                geometry: PCLReader {
                    id: pcdReader
                    onErrorOccurred: function(error) {
                        console.log("Error:", error)
                        errorDialog.text = error
                        errorDialog.open()
                    }
                }
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        pointSize: 5
                        vertexColorsEnabled: true
                        cullMode: DefaultMaterial.NoCulling
                    }
                ]
                scale: Qt.vector3d(1, 1, 1)
            }
        }
    }

    Rectangle {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 10
        width: 250
        height: controlColumn.height + 20
        color: "#80ffffff"
        radius: 5

        ColumnLayout {
            id: controlColumn
            anchors.centerIn: parent
            spacing: 10
            width: parent.width - 20

            GroupBox {
                Layout.fillWidth: true
                title: "点云显示"

                ColumnLayout {
                    anchors.fill: parent

                    // RowLayout {
                    //     Label { text: "点大小:" }
                    //     Slider {
                    //         Layout.fillWidth: true
                    //         from: 1
                    //         to: 10
                    //         value: pcd_model.materials[0].pointSize
                    //         onValueChanged: pcd_model.materials[0].pointSize = value
                    //     }
                    // }

                    CheckBox {
                        text: "显示网格"
                        checked: grid.visible
                        onCheckedChanged: grid.visible = checked
                    }

                    CheckBox {
                        text: "显示车辆"
                        checked: vehicleCube.visible
                        onCheckedChanged: vehicleCube.visible = checked
                    }
                }
            }

            GroupBox {
                Layout.fillWidth: true
                title: "相机控制"

                ColumnLayout {
                    anchors.fill: parent

                    Button {
                        text: cameraLocked ? "解锁相机" : "锁定相机"
                        Layout.fillWidth: true
                        onClicked: {
                            cameraLocked = !cameraLocked
                            if (cameraLocked) {
                                cameraRotation = Qt.vector3d(0, 0, 1)
                                currentDistance = pcdReader.carZ + defaultDistance
                            }
                        }
                    }

                    Button {
                        text: "重置视角"
                        Layout.fillWidth: true
                        onClicked: {
                            cameraRotation = Qt.vector3d(0, 0, 0.1)
                            currentDistance = defaultDistance
                            if (!cameraLocked) {
                                cameraNode.x = pcdReader.carX
                                cameraNode.y = pcdReader.carY
                                cameraNode.z = pcdReader.carZ
                            }
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "距离:" }
                        Label {
                            text: currentDistance.toFixed(1) + " m"
                            font.bold: true
                        }
                    }
                }
            }

            GroupBox {
                Layout.fillWidth: true
                title: "运动模拟"

                ColumnLayout {
                    anchors.fill: parent

                    Button {
                        text: isSimulating ? "停止模拟" : "开始模拟"
                        Layout.fillWidth: true
                        onClicked: {
                            isSimulating = !isSimulating
                        }
                    }

                    RowLayout {
                        Label { text: "速度:" }
                        Slider {
                            Layout.fillWidth: true
                            from: 0.01
                            to: 0.2
                            value: simulationSpeed
                            onValueChanged: simulationSpeed = value
                        }
                    }

                    RowLayout {
                        Label { text: "半径:" }
                        Slider {
                            Layout.fillWidth: true
                            from: 10
                            to: 100
                            value: circleRadius
                            onValueChanged: circleRadius = value
                        }
                    }
                }
            }

            // 在UDP控制GroupBox中添加:
            GroupBox {
                Layout.fillWidth: true
                title: "UDP控制"

                ColumnLayout {
                    anchors.fill: parent

                    SpinBox {
                        id: udpPort
                        Layout.fillWidth: true
                        from: 1024
                        to: 65535
                        value: 8080
                        editable: true
                    }

                    Button {
                        text: "启动UDP监听"
                        Layout.fillWidth: true
                        onClicked: {
                            pcdReader.startUDPListener(udpPort.value)
                        }
                    }

                    Button {
                        text: "停止UDP监听"
                        Layout.fillWidth: true
                        onClicked: {
                            pcdReader.stopUDPListener()
                        }
                    }

                    Button {
                        text: "清除点云"  // 新增按钮
                        Layout.fillWidth: true
                        onClicked: {
                            pcdReader.clearPoints()
                        }
                    }
                }
            }

            GroupBox {
                Layout.fillWidth: true
                title: "车辆位置控制"

                GridLayout {
                    columns: 2
                    anchors.fill: parent

                    Label { text: "当前 X:" }
                    Label { text: pcdReader.carX.toFixed(2) }

                    Label { text: "当前 Y:" }
                    Label { text: pcdReader.carY.toFixed(2) }

                    Label { text: "当前 Z:" }
                    Label { text: pcdReader.carZ.toFixed(2) }

                    Label { text: "新 X:" }
                    SpinBox {
                        id: newX
                        from: -10000
                        to: 10000
                        stepSize: 1
                        value: 0
                        editable: true
                        property real realValue: value / 10.0

                        validator: DoubleValidator {
                            decimals: 1
                            notation: DoubleValidator.StandardNotation
                        }

                        textFromValue: function(value, locale) {
                            return Number(value / 10).toLocaleString(locale, 'f', 1)
                        }

                        valueFromText: function(text, locale) {
                            return Number.fromLocaleString(locale, text) * 10
                        }
                    }

                    Label { text: "新 Y:" }
                    SpinBox {
                        id: newY
                        from: -10000
                        to: 10000
                        stepSize: 1
                        value: 0
                        editable: true
                        property real realValue: value / 10.0

                        validator: DoubleValidator {
                            decimals: 1
                            notation: DoubleValidator.StandardNotation
                        }

                        textFromValue: function(value, locale) {
                            return Number(value / 10).toLocaleString(locale, 'f', 1)
                        }

                        valueFromText: function(text, locale) {
                            return Number.fromLocaleString(locale, text) * 10
                        }
                    }

                    Label { text: "新 Z:" }
                    SpinBox {
                        id: newZ
                        from: -10000
                        to: 10000
                        stepSize: 1
                        value: 0
                        editable: true
                        property real realValue: value / 10.0

                        validator: DoubleValidator {
                            decimals: 1
                            notation: DoubleValidator.StandardNotation
                        }

                        textFromValue: function(value, locale) {
                            return Number(value / 10).toLocaleString(locale, 'f', 1)
                        }

                        valueFromText: function(text, locale) {
                            return Number.fromLocaleString(locale, text) * 10
                        }
                    }

                    Button {
                        text: "更新位置"
                        Layout.columnSpan: 2
                        Layout.fillWidth: true
                        onClicked: {
                            pcdReader.setVehiclePosition(newX.realValue, newY.realValue, newZ.realValue)
                        }
                    }
                }
            }

            GroupBox {
                Layout.fillWidth: true
                title: "车辆朝向控制"

                ColumnLayout {
                    anchors.fill: parent

                    Label {
                        text: "当前朝向: " + (pcdReader.vehicleHeading * 180 / Math.PI).toFixed(2) + "°"
                    }

                    Slider {
                        Layout.fillWidth: true
                        from: -Math.PI
                        to: Math.PI
                        value: pcdReader.vehicleHeading
                        onValueChanged: pcdReader.vehicleHeading = value
                    }
                }
            }
        }
    }

    Dialog {
        id: errorDialog
        title: "错误"
        modal: true
        standardButtons: Dialog.Ok
        anchors.centerIn: parent
        property alias text: errorText.text

        Text {
            id: errorText
        }
    }

    // 添加内存显示组件
    Rectangle {
        id: memoryMonitor
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 10
        width: memoryLayout.width + 20
        height: memoryLayout.height + 10
        color: "#80ffffff"  // 半透明背景
        radius: 5

        ColumnLayout {
            id: memoryLayout
            anchors.centerIn: parent
            spacing: 5

            RowLayout {
                Text {
                    text: "内存: "
                    font.pixelSize: 14
                    font.bold: true
                }
                Text {
                    text: pcdReader.memoryUsage.toFixed(1) + "%"
                    font.pixelSize: 14
                    font.bold: true
                    color: pcdReader.memoryUsage >= pcdReader.memoryThreshold ? "#ff4444" : "#17a81a"
                    
                    Behavior on color {
                        ColorAnimation { duration: 200 }
                    }
                }
            }

            // 添加一个小进度条
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 4
                color: "#e6e6e6"
                radius: 2

                Rectangle {
                    width: parent.width * (pcdReader.memoryUsage / 100)
                    height: parent.height
                    radius: 2
                    color: pcdReader.memoryUsage >= pcdReader.memoryThreshold ? "#ff4444" : "#17a81a"
                    
                    Behavior on color {
                        ColorAnimation { duration: 200 }
                    }
                }
            }
        }

        // 添加鼠标悬停效果
        MouseArea {
            anchors.fill: parent
            hoverEnabled: true
            
            ToolTip {
                visible: parent.containsMouse
                text: "系统内存使用率\n阈值: " + pcdReader.memoryThreshold.toFixed(1) + "%"
                delay: 500
            }
        }
    }
}
