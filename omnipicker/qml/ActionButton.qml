import QtQuick
import QtQuick.Controls

Button {
    id: control
    property bool primary: false
    property bool danger: false
    implicitHeight: 40
    implicitWidth: Math.max(76, contentItem.implicitWidth + 28)
    hoverEnabled: true
    Accessible.name: text
    font.pixelSize: 13
    contentItem: Text {
        text: control.text
        font: control.font
        color: !control.enabled ? "#53606a" : control.primary ? "#081c20" : control.danger ? "#ffada8" : "#d5dfe4"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
    background: Rectangle {
        radius: 8
        color: !control.enabled ? "#192127" : control.down ? "#2f5660" : control.primary ? "#6bd7de" : control.danger ? "#3a2228" : control.hovered ? "#2a3944" : "#202c35"
        border.width: 1
        border.color: control.activeFocus ? "#7edbe0" : control.danger ? "#6c3e44" : control.primary ? "#6bd7de" : "#35424c"
        Behavior on color { ColorAnimation { duration: 100 } }
    }
}
