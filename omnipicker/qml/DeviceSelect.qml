import QtQuick
import QtQuick.Controls

ComboBox {
    id: box
    implicitHeight: 40
    font.pixelSize: 13
    leftPadding: 12; rightPadding: 26
    contentItem: Text { text: box.displayText; color: "#dce7ee"; font: box.font; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
    indicator: Text { x: box.width-width-12; anchors.verticalCenter: parent.verticalCenter; text: "⌄"; color: "#8da8b8"; font.pixelSize: 18 }
    background: Rectangle { radius: 8; color: "#23313c"; border.color: box.activeFocus ? "#68cbd4" : "#35434e" }
    delegate: ItemDelegate {
        id: entry
        required property int index
        required property var modelData
        width: box.width; height: 42
        contentItem: Text { text: entry.modelData; color: "#e1ecf0"; font: box.font; verticalAlignment: Text.AlignVCenter }
        highlighted: box.highlightedIndex===index
        background: Rectangle { color: entry.highlighted ? "#355461" : "#23313c" }
    }
    popup: Popup {
        y: box.height+4; width: box.width; padding: 4
        implicitHeight: contentItem.implicitHeight+8
        background: Rectangle { color: "#23313c"; radius: 8; border.color: "#435663" }
        contentItem: ListView { clip: true; implicitHeight: contentHeight; model: box.popup.visible ? box.delegateModel : null; currentIndex: box.highlightedIndex }
    }
}
