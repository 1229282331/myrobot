import QtQuick
import QtQuick.Controls

TextField {
    id: control
    implicitHeight: 40
    color: "#e5edf1"
    selectionColor: "#327482"
    selectedTextColor: "white"
    placeholderTextColor: "#5a6a77"
    font.pixelSize: 13
    leftPadding: 12
    selectByMouse: true
    background: Rectangle {
        color: control.enabled ? "#10191f" : "#192127"
        radius: 7
        border.color: control.activeFocus ? "#6bd7de" : "#34424b"
    }
}
