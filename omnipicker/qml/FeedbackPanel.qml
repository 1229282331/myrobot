import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: panel
    required property var deviceState
    property bool valid: deviceState.feedbackValid
    property var trail: deviceState.feedbackTrail || []
    implicitHeight: 142
    radius: 12
    color: "#17242c"
    border.color: valid ? "#2b4b52" : "#34414a"
    onTrailChanged: graph.requestPaint()
    onValidChanged: graph.requestPaint()

    component Small: Text { color: "#93a7b3"; font.pixelSize: 11 }
    ColumnLayout {
        anchors.fill: parent; anchors.margins: 16; spacing: 10
        RowLayout {
            Layout.fillWidth: true
            Text { text: deviceState.simulated && deviceState.connected ? "模拟反馈" : "设备反馈"; color: "#dcebef"; font.pixelSize: 14; font.weight: Font.DemiBold }
            Small { text: deviceState.feedbackMessage; color: panel.valid ? (deviceState.simulated ? "#e7c58a" : "#83d9c7") : "#d7b299"; Layout.fillWidth: true; elide: Text.ElideRight }
            Small { text: deviceState.feedbackPort + "  ·  " + (panel.valid ? deviceState.feedbackHz.toFixed(0)+" Hz" : "— Hz") + "  ·  数据年龄 " + (deviceState.feedbackAgeMs<0 ? "—" : Math.min(99999,deviceState.feedbackAgeMs).toFixed(0))+" ms" }
        }
        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 28
            ColumnLayout {
                Layout.preferredWidth: 143; spacing: 2
                Small { text: "回传位置 / 归一化" }
                Text { objectName: "feedbackPositionValue"; text: panel.valid ? deviceState.feedbackPosition.toFixed(0)+" %" : "—"; color: panel.valid ? "#8de2e3" : "#81949f"; font.pixelSize: 30; font.weight: Font.Light }
                Small { text: "文本量化 1 个百分点"; font.pixelSize: 10 }
            }
            ColumnLayout {
                Layout.preferredWidth: 148; spacing: 2
                Small { text: "回传速度 / 归一化" }
                Text { objectName: "feedbackVelocityValue"; text: panel.valid ? deviceState.feedbackVelocity.toFixed(0)+" %/s" : "—"; color: "#d4e2e9"; font.pixelSize: 26; font.weight: Font.Light }
                Small { text: "张开为正 · 闭合为负"; font.pixelSize: 10 }
            }
            Rectangle { width: 1; Layout.fillHeight: true; color: "#30404a" }
            ColumnLayout {
                Layout.fillWidth: true; Layout.fillHeight: true; spacing: 3
                RowLayout {
                    Small { text: "最近 10 秒" }
                    Item { Layout.fillWidth: true }
                    Small { text: "━ 回传位置"; color: "#77d9dc" }
                    Small { text: "┄ 已确认目标"; color: "#d6b378" }
                    Small { text: "0–100%" }
                }
                Canvas {
                    id: graph; objectName: "feedbackGraph"; Layout.fillWidth: true; Layout.fillHeight: true
                    onWidthChanged: requestPaint()
                    onHeightChanged: requestPaint()
                    onPaint: {
                        var c=getContext("2d"); c.reset()
                        c.strokeStyle="#344650"; c.lineWidth=0.5
                        for(var i=0;i<3;i++){var y=2+(height-4)*i/2;c.beginPath();c.moveTo(0,y);c.lineTo(width,y);c.stroke()}
                        var rows=panel.trail
                        if(rows.length<2)return
                        var right=rows[rows.length-1].t, left=right-10
                        c.globalAlpha=panel.valid?1:0.3
                        for(var column=0;column<2;column++){
                            c.strokeStyle=column===0?"#77d9dc":"#d6b378";c.lineWidth=column===0?1.6:1
                            c.setLineDash(column===0?[]:[4,4]);c.beginPath();var began=false
                            for(var j=0;j<rows.length;j++){
                                var row=rows[j], value=column===0?row.position:row.target
                                if(row.t<left||value<0){began=false;continue}
                                var x=(row.t-left)/10*width, py=height-2-value/100*(height-4)
                                if(!began){c.moveTo(x,py);began=true}else c.lineTo(x,py)
                            }
                            c.stroke()
                        }
                    }
                }
                Small { text: "主机接收时间 · 非设备采样时间    /    第1列 " + (panel.valid ? deviceState.feedbackColumn1.toFixed(2) : "—") + "（单位未核实）"; font.pixelSize: 10 }
            }
        }
    }
}
