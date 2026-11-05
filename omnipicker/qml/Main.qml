import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D

ApplicationWindow {
    id: win
    width: 1440; height: 900
    minimumWidth: 1100; minimumHeight: 760
    visible: true
    title: "OmniPicker Studio"
    color: "#10161c"
    property var s: studio.state
    property bool writable: s.connected && s.compatible && !s.locked && !s.busy
    property bool feedbackReady: !s.feedbackRequired || s.feedbackValid
    property bool live: writable && feedbackReady && s.enableState === "enabled" && !s.testActive
    property bool expanded: false
    property string leaving: "close"
    property bool allowClose: false
    property var limitNames: ["velocity_limit","traj_vel","traj_acc","traj_dcc","ramp_vel","current_limit"]
    property var limitLabels: ["速度上限","轨迹速度","轨迹加速度","轨迹减速度","斜坡速度","电流上限 / A"]
    onSChanged: rig.updateDisplay(s, studio.target)
    Component.onCompleted: rig.updateDisplay(s, studio.target)

    function requestLeave(action) {
        leaving = action
        if (!s.connected && !s.mayBeEnabled && !s.modified) studio.leave(action,"keep")
        else if (s.connected && s.enableState === "disabled") studio.leave(action,"disable")
        else exitDialog.open()
    }
    onClosing: function(close) { if(!allowClose) { close.accepted = false; requestLeave("close") } }
    Connections {
        target: studio
        function onQuitReady() { win.allowClose=true; win.close() }
        function onTargetChanged() { rig.updateDisplay(s, studio.target) }
    }
    Shortcut { sequence: "Escape"; enabled: !exitDialog.visible && s.connected; onActivated: studio.action("stop") }
    Shortcut { sequence: "Ctrl+Shift+S"; context: Qt.ApplicationShortcut; onActivated: studio.capture() }

    component Caption: Text { color: "#8496a3"; font.pixelSize: 12 }
    component Heading: Text { color: "#edf3f6"; font.pixelSize: 16; font.weight: Font.DemiBold }
    component Divider: Rectangle { Layout.fillWidth: true; height: 1; color: "#2b3740" }

    ColumnLayout {
        anchors.fill: parent; anchors.margins: 24; spacing: 16
        RowLayout {
            Layout.fillWidth: true; Layout.preferredHeight: 58; spacing: 16
            Rectangle {
                width: 42; height: 42; radius: 12; color: "#1c3d45"; border.color: "#38616a"
                Text { anchors.centerIn: parent; text: "O"; color: "#86e0e3"; font.pixelSize: 26; font.weight: Font.Light }
            }
            ColumnLayout {
                spacing: 2
                Text { text: "OmniPicker Studio"; color: "#f0f6f7"; font.pixelSize: 23; font.weight: Font.DemiBold; font.letterSpacing: 0.1 }
                Caption { text: "CONTROL WORKSPACE   /   原生 USB · 三维操作台"; font.pixelSize: 10; font.letterSpacing: 1 }
            }
            Item { Layout.fillWidth: true }
            Rectangle {
                width: 108; height: 30; radius: 15; color: s.connected ? (s.simulated ? "#3b3021" : "#183c35") : "#242d35"
                Text { anchors.centerIn: parent; text: s.connected ? (s.simulated ? "●  模拟设备" : "●  USB 已连接") : "○  尚未连接"; color: s.connected ? (s.simulated ? "#f2ce89" : "#89dfbf") : "#a3b0ba"; font.pixelSize: 12 }
            }
            ColumnLayout {
                Layout.preferredWidth: 104; spacing: 2
                Caption { text: "FIRMWARE"; font.pixelSize: 10; font.letterSpacing: 1 }
                Text { text: s.firmware + (s.connected && !s.compatible ? " · 只读" : ""); color: "#d6e0e6"; font.pixelSize: 14 }
            }
            ColumnLayout {
                Layout.preferredWidth: 78; spacing: 2
                Caption { text: "故障码"; font.pixelSize: 10 }
                Text { text: !s.connected || s.faultCode < 0 ? "—" : String(s.faultCode); color: s.faultCode > 0 ? "#ff918b" : "#9eddc8"; font.pixelSize: 16 }
            }
            ActionButton { text: "停止（下使能）"; danger: true; implicitWidth: 164; implicitHeight: 44; enabled: s.connected && s.compatible; onClicked: studio.action("stop") }
        }

        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 16
            Rectangle {
                id: viewport; Layout.fillWidth: true; Layout.fillHeight: true
                radius: 14; color: "#151f27"; border.color: "#2b3944"; clip: true
                gradient: Gradient { GradientStop { position: 0; color: "#202e39" } GradientStop { position: 1; color: "#131d25" } }
                Canvas {
                    anchors.fill: parent; opacity: 0.15
                    onWidthChanged: requestPaint(); onHeightChanged: requestPaint()
                    onPaint: {
                        var c = getContext("2d"); c.reset(); c.strokeStyle = "#7898ae"; c.lineWidth = 0.5
                        for(var x=0;x<width;x+=48) { c.beginPath();c.moveTo(x,0);c.lineTo(x,height);c.stroke() }
                        for(var y=0;y<height;y+=48) { c.beginPath();c.moveTo(0,y);c.lineTo(width,y);c.stroke() }
                    }
                }
                View3D {
                    id: view; objectName: "gripperView"; anchors.fill: parent
                    visible: rig.modelVisible
                    opacity: rig.displayMode === "stale" ? 0.28 : 1.0
                    environment: SceneEnvironment { backgroundMode: SceneEnvironment.Transparent; antialiasingMode: SceneEnvironment.MSAA; antialiasingQuality: SceneEnvironment.High }
                    camera: camera
                    DirectionalLight { eulerRotation: Qt.vector3d(-35,-30,0); brightness: 1.8; ambientColor: "#56677a" }
                    DirectionalLight { eulerRotation: Qt.vector3d(10,140,20); brightness: 1.15; color: "#afdbeb" }
                    DirectionalLight { eulerRotation: Qt.vector3d(55,30,0); brightness: 0.6 }
                    Node {
                        id: orbit; position: Qt.vector3d(0,73,0); eulerRotation: Qt.vector3d(-12,65,0)
                        PerspectiveCamera { id: camera; z: 290; clipNear: 1; clipFar: 2000; fieldOfView: 40 }
                    }
                    Node {
                        eulerRotation.x: -90
                        Repeater3D {
                            model: rig.parts
                            delegate: Model {
                                required property var modelData
                                geometry: modelData.geometry
                                position: modelData.position; rotation: modelData.rotation
                                materials: PrincipledMaterial {
                                    baseColor: modelData.name === "base_link" ? "#9cabb3" : modelData.name.indexOf("loop") >= 0 ? "#427782" : "#c0cbd0"
                                    metalness: 0.6; roughness: 0.34
                                    cullMode: Material.NoCulling
                                }
                            }
                        }
                    }
                }
                MouseArea {
                    anchors.fill: parent
                    property real previousX; property real previousY
                    onPressed: function(mouse) { previousX=mouse.x;previousY=mouse.y }
                    onPositionChanged: function(mouse) {
                        if (pressed) {
                            orbit.eulerRotation.y += (mouse.x-previousX)*0.4
                            orbit.eulerRotation.x = Math.max(-80, Math.min(80,orbit.eulerRotation.x+(mouse.y-previousY)*0.35))
                            previousX=mouse.x;previousY=mouse.y
                        }
                    }
                    onWheel: function(wheel) { camera.z = Math.max(170,Math.min(600,camera.z-wheel.angleDelta.y*0.12)) }
                    onDoubleClicked: { orbit.eulerRotation=Qt.vector3d(-12,65,0);camera.z=290 }
                }
                Column {
                    anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 24; spacing: 7
                    Caption { text: "OP1  /  DIGITAL VIEW"; color: "#7d9aa9"; font.pixelSize: 11; font.letterSpacing: 1.5 }
                    Text { objectName: "poseSourceLabel"; text: rig.displayTitle; color: rig.displayMode === "stale" ? "#e0b998" : "#dfeaf0"; font.pixelSize: 19 }
                    Caption { text: "名义联动 · 不含受力指节形变"; color: "#8b9eaa" }
                }
                Text {
                    anchors.centerIn: parent; visible: !rig.modelVisible
                    text: "暂无有效位置反馈"; color: "#9db0bc"; font.pixelSize: 18
                }
                Row {
                    anchors.top: parent.top; anchors.right: parent.right; anchors.margins: 20; spacing: 8
                    ActionButton { text: "复位视角"; onClicked: { orbit.eulerRotation=Qt.vector3d(-12,65,0);camera.z=290 } }
                    ActionButton { text: "截图"; implicitWidth: 60; onClicked: studio.capture() }
                }
                Column {
                    anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 24; spacing: 8
                    Text { objectName: "posePositionValue"; text: rig.displayValue; color: "#e8f6f8"; font.pixelSize: 34; font.weight: Font.Light }
                    Caption { text: rig.displayDetail }
                }
                Caption { anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.margins: 24; text: "拖动旋转   ·   滚轮缩放   ·   双击复位"; font.pixelSize: 11 }
            }

            Rectangle {
                Layout.preferredWidth: 354; Layout.fillHeight: true; radius: 14; color: "#19232c"; border.color: "#2c3943"
                ScrollView {
                    anchors.fill: parent; anchors.margins: 20; clip: true
                    contentWidth: availableWidth
                    ColumnLayout {
                        width: parent.width; spacing: 16
                        RowLayout {
                            Heading { text: "设备连接" }
                            Item { Layout.fillWidth: true }
                            Caption { text: s.connected ? "CAN ID  " + s.canId : "USB / Fibre" }
                        }
                        RowLayout {
                            DeviceSelect {
                                id: deviceSelect; Layout.fillWidth: true; implicitHeight: 40
                                model: ["模拟设备","真实设备 · USB"]
                                enabled: !s.connected && !s.busy
                            }
                            ActionButton {
                                text: s.busy ? "连接中" : s.connected ? "断开" : "连接"; primary: !s.connected; enabled: !s.busy
                                onClicked: s.connected ? requestLeave("disconnect") : studio.connectDevice(deviceSelect.currentIndex===0,usbSerial.text)
                            }
                        }
                        Field { id: usbSerial; Layout.fillWidth: true; text: "20773954464D"; visible: deviceSelect.currentIndex===1 && !s.connected; enabled: !s.busy; Accessible.name: "USB序列号" }
                        Caption { text: s.connected ? "SN  " + s.serial : "选择设备并连接，不会自动使能" }
                        Divider {}
                        RowLayout {
                            Heading { text: "目标开度控制" }
                            Item { Layout.fillWidth: true }
                            Text { text: live ? "● 实时下发" : "○ 编辑目标"; color: live ? "#73d9dc" : "#91a0ac"; font.pixelSize: 12 }
                        }
                        RowLayout {
                            Text { text: studio.target.toFixed(1); color: "#eef7f8"; font.pixelSize: 40; font.weight: Font.Light }
                            Caption { text: "%"; font.pixelSize: 18; Layout.alignment: Qt.AlignBottom; Layout.bottomMargin: 8 }
                            Item { Layout.fillWidth: true }
                            Field {
                                id: targetInput; objectName: "targetInput"; Layout.preferredWidth: 94; text: studio.target.toFixed(1); horizontalAlignment: Text.AlignRight; enabled: !s.testActive
                                Accessible.name: "目标开度百分比"
                                validator: DoubleValidator { bottom: 0; top: 100; decimals: 1; notation: DoubleValidator.StandardNotation; locale: "C" }
                                onEditingFinished: { if(acceptableInput) studio.editTarget(Number(text),true); text=studio.target.toFixed(1) }
                                Connections { target: studio; function onTargetChanged() { if(!targetInput.activeFocus) targetInput.text=studio.target.toFixed(1) } }
                            }
                        }
                        Slider {
                            id: opening; objectName: "openingSlider"; Layout.fillWidth: true; from: 0; to: 100; stepSize: 0.1; value: studio.target; enabled: !s.testActive
                            Accessible.name: "开度实时滑块"
                            onMoved: studio.editTarget(value,true)
                            onPressedChanged: if(!pressed) studio.editTarget(value,true)
                            background: Rectangle { x: opening.leftPadding; y: opening.topPadding+opening.availableHeight/2-2; width: opening.availableWidth; height: 4; radius: 2; color: "#374751"; Rectangle { width: opening.visualPosition*parent.width; height: 4; radius: 2; color: "#6bd7de" } }
                            handle: Rectangle { x: opening.leftPadding+opening.visualPosition*(opening.availableWidth-width); y: opening.topPadding+opening.availableHeight/2-height/2; width: 18; height: 18; radius: 9; color: "#c4f3f3"; border.width: 3; border.color: "#559da8" }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 6
                            Repeater { model: [0,15,35,50,100]; ActionButton { required property int modelData; text: modelData+"%"; implicitWidth: 50; Layout.fillWidth: true; enabled: !s.testActive; onClicked: studio.editTarget(modelData,true) } }
                        }
                        Rectangle {
                            Layout.fillWidth: true; implicitHeight: 58; radius: 8; color: "#121d24"
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 12
                                ColumnLayout { spacing: 3; Caption { text: "最近已确认指令"; font.pixelSize: 11 } Text { text: s.ackTarget<0 ? "—" : s.ackTarget.toFixed(1)+" %"; color: "#abdade"; font.pixelSize: 16 } }
                                Item { Layout.fillWidth: true }
                                ColumnLayout { spacing: 3; Caption { text: "通信 RTT"; font.pixelSize: 11 } Text { text: s.connected ? s.rtt.toFixed(1)+" ms" : "—"; color: "#d5e2e8"; font.pixelSize: 14 } }
                            }
                        }
                        RowLayout {
                            ActionButton { text: "使能"; primary: true; Layout.fillWidth: true; enabled: writable && feedbackReady && !s.testActive && s.enableState!=="enabled"; onClicked: studio.action("enable") }
                            ActionButton { text: "下使能"; Layout.fillWidth: true; enabled: s.connected && s.compatible; onClicked: studio.action("stop") }
                        }
                        Caption { text: s.enableState==="enabled" ? "使能请求已确认" : s.enableState==="disabled" ? "下使能请求已确认" : "使能状态未知"; color: s.enableState==="enabled" ? "#81ddd1" : "#bdc7d0" }
                        Caption { text: "目标输入 0.1% · 下发最多 20 Hz\n位置反馈量化 1% · 实时使能状态不可读"; lineHeight: 1.5; font.pixelSize: 11 }
                        Divider {}
                        ActionButton { text: "运动参数   ›"; Layout.fillWidth: true; enabled: s.connected && s.compatible; onClicked: { params.load(); params.open() } }
                        ActionButton { text: "显式清错"; Layout.fillWidth: true; enabled: s.connected && s.compatible && s.enableState==="disabled"; onClicked: studio.action("clear") }
                    }
                }
            }
        }

        FeedbackPanel { Layout.fillWidth: true; deviceState: s }

        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: expanded ? 188 : 46
            color: "#172129"; radius: 12; border.color: "#2b3842"
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 14; spacing: 12
                RowLayout {
                    Layout.fillWidth: true; Layout.preferredHeight: 18
                    Text { text: "测试与记录"; color: "#dbe5ea"; font.pixelSize: 13; font.weight: Font.DemiBold }
                    Item { Layout.fillWidth: true }
                    Caption { text: s.testProgress; color: s.testActive ? "#81ddd1" : "#8fa1ad" }
                    ActionButton { text: expanded ? "收起 −" : "展开 +"; implicitWidth: 70; implicitHeight: 24; onClicked: expanded=!expanded }
                }
                RowLayout {
                    visible: expanded; Layout.fillWidth: true; Layout.fillHeight: true; spacing: 24
                    ColumnLayout {
                        Layout.preferredWidth: 605; spacing: 10
                        RowLayout {
                            spacing: 10
                            Caption { text: "A / %" }
                            Field { id: testA; text: "15"; Layout.preferredWidth: 65; Accessible.name: "测试A开度"; validator: DoubleValidator { bottom: 0; top: 100; decimals: 1; locale: "C" } }
                            Caption { text: "↔  B / %" }
                            Field { id: testB; text: "35"; Layout.preferredWidth: 65; Accessible.name: "测试B开度"; validator: DoubleValidator { bottom: 0; top: 100; decimals: 1; locale: "C" } }
                            Caption { text: "停留 / s" }
                            Field { id: dwell; text: "6"; Layout.preferredWidth: 60; Accessible.name: "每段停留秒数"; validator: DoubleValidator { bottom: 0.5; top: 60; decimals: 1; locale: "C" } }
                            Caption { text: "往返" }
                            Field { id: cycles; text: "1"; Layout.preferredWidth: 55; Accessible.name: "往返次数"; validator: IntValidator { bottom: 1; top: 100 } }
                            Caption { text: "次" }
                        }
                        RowLayout {
                            ActionButton { text: "开始往返"; primary: true; enabled: live && testA.acceptableInput && testB.acceptableInput && dwell.acceptableInput && cycles.acceptableInput; onClicked: studio.startTest(Number(testA.text),Number(testB.text),Number(dwell.text),Number(cycles.text)) }
                            ActionButton { text: "停止测试"; enabled: s.testActive; onClicked: studio.action("stop") }
                            Caption { text: "A → B → A · 完成后保持末次目标\n停留按时间执行，不表示实际到位"; font.pixelSize: 10; lineHeight: 1.5 }
                        }
                    }
                    Rectangle { width: 1; Layout.fillHeight: true; color: "#2b3842" }
                    ListView {
                        id: eventLog; Layout.fillWidth: true; Layout.fillHeight: true; clip: true; model: studio.logs; spacing: 6
                        ScrollBar.vertical: ScrollBar {}
                        delegate: Text {
                            required property var modelData
                            width: eventLog.width-12; text: modelData.time.substring(11,19)+"   "+modelData.message
                            color: modelData.level==="error" ? "#f9a39b" : modelData.level==="warning" ? "#dcb777" : "#91a7b5"
                            font.pixelSize: 11; wrapMode: Text.Wrap
                        }
                    }
                }
            }
        }
        RowLayout {
            Layout.fillWidth: true; Layout.preferredHeight: 20
            Text { text: s.status; color: s.locked ? "#f0a19a" : "#93a9b7"; font.pixelSize: 11; elide: Text.ElideRight; Layout.fillWidth: true }
            Row { visible: s.connected && s.simulated; spacing: 10
                ActionButton { text: "模拟反馈中断"; implicitHeight: 24; onClicked: studio.action("mock_feedback_pause") }
                ActionButton { text: "模拟故障"; implicitHeight: 24; onClicked: studio.action("mock_fault") }
                ActionButton { text: "模拟超时"; implicitHeight: 24; onClicked: studio.action("mock_timeout") }
            }
            Caption { text: studio.recordingError || "本地采集  ·  v0.2.1"; color: studio.recordingError ? "#f9a39b" : "#8496a3"; font.pixelSize: 10 }
        }
    }

    Popup {
        id: params; anchors.centerIn: parent; width: 610; height: 680; modal: true; focus: true; padding: 28
        closePolicy: Popup.CloseOnEscape
        background: Rectangle { color: "#1c2832"; radius: 16; border.color: "#435561" }
        property string validationMessage: ""
        function load() { validationMessage="";for(var i=0;i<6;i++) paramRows.itemAt(i).fieldText=String(Number(Number(s.limits[limitNames[i]]||0).toPrecision(7))) }
        function profile(ratio) { validationMessage="";for(var i=0;i<6;i++) paramRows.itemAt(i).fieldText=String(i===5 ? Math.min(1,s.baseline[limitNames[i]]) : Number((s.baseline[limitNames[i]]*ratio).toPrecision(7))) }
        function apply() { var v={};for(var i=0;i<6;i++) {var row=paramRows.itemAt(i);if(!row.valid){validationMessage="请检查参数：必须大于 0，且不能超过连接时原值";return};v[limitNames[i]]=Number(row.fieldText)};validationMessage="";studio.applyLimits(v) }
        ColumnLayout {
            anchors.fill: parent; spacing: 14
            RowLayout { Heading { text: "运动参数"; font.pixelSize: 22 } Item { Layout.fillWidth: true } ActionButton { text: "截图"; onClicked: studio.capture() } ActionButton { text: "关闭"; onClicked: params.close() } }
            Caption { text: "相对本次连接原值 · 只改 RAM · 不写入 Flash\n速度档调整前五项；电流档位上限取 1 A 与原值中的较小值"; lineHeight: 1.5 }
            RowLayout {
                Repeater { model: [0.1,0.25,0.5,1]; ActionButton { required property real modelData; text: (modelData*100)+"% 速度"; Layout.fillWidth: true; onClicked: params.profile(modelData) } }
            }
            Divider {}
            Repeater {
                id: paramRows; model: 6
                RowLayout {
                    id: paramRow; required property int index
                    property alias fieldText: paramField.text
                    property bool valid: paramField.acceptableInput && Number(paramField.text)>0 && Number(paramField.text)<=Number(s.baseline[limitNames[index]])+0.0000001
                    Layout.fillWidth: true
                    Caption { text: limitLabels[index]; Layout.preferredWidth: 112; color: "#c6d3db" }
                    Field { id: paramField; Layout.preferredWidth: 118; validator: DoubleValidator { bottom: 0.000001; top: 100000; decimals: 8; locale: "C"; notation: DoubleValidator.StandardNotation } Accessible.name: limitNames[index] }
                    Caption { text: limitNames[index]; Layout.fillWidth: true; font.pixelSize: 10 }
                    Caption { text: "原 " + Number(s.baseline[limitNames[index]]||0).toPrecision(4); color: paramRow.valid ? "#8fa6b5" : "#e9aa91" }
                }
            }
            Item { Layout.fillHeight: true }
            Caption { text: params.validationMessage || s.status; Layout.fillWidth: true; wrapMode: Text.Wrap; color: params.validationMessage ? "#edaa91" : "#91cfd2" }
            RowLayout {
                ActionButton { text: "停止 / 下使能"; danger: true; enabled: s.connected; onClicked: studio.action("stop") }
                ActionButton { text: "恢复原参数"; Layout.fillWidth: true; enabled: writable && s.enableState==="disabled"; onClicked: studio.action("restore") }
                ActionButton { text: "应用并核对"; primary: true; Layout.fillWidth: true; enabled: writable && s.enableState==="disabled"; onClicked: params.apply() }
            }
        }
    }

    Popup {
        id: exitDialog; anchors.centerIn: parent; width: 570; height: 300; modal: true; focus: true; padding: 28
        closePolicy: Popup.CloseOnEscape
        background: Rectangle { color: "#1c2832"; radius: 16; border.color: "#435561" }
        ColumnLayout {
            anchors.fill: parent; spacing: 18
            Heading { text: leaving==="close" ? "退出前如何处理夹爪？" : "断开前如何处理夹爪？"; font.pixelSize: 22 }
            Text { text: "设备可能仍处于使能状态。\n下使能后退出：收到确认后恢复本程序临时限幅。\n保持使能退出：保留 RAM 参数，不发下使能请求。"; color: "#b5c5cf"; font.pixelSize: 13; lineHeight: 1.7 }
            Item { Layout.fillHeight: true }
            RowLayout {
                ActionButton { text: "下使能后退出"; primary: true; onClicked: { exitDialog.close();studio.leave(leaving,"disable") } }
                ActionButton { text: "保持使能退出"; onClicked: { exitDialog.close();studio.leave(leaving,"keep") } }
                ActionButton { text: "取消"; Layout.fillWidth: true; onClicked: exitDialog.close() }
            }
        }
    }
}
