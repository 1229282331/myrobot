import os
from pathlib import Path
import sys
from datetime import datetime

from PySide6.QtCore import QUrl, QLockFile, qInstallMessageHandler
from PySide6.QtGui import QGuiApplication, QFont
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuickControls2 import QQuickStyle
from .device import ROOT
from .bridge import Bridge
from .scene import Rig


def main():
    if Path(sys.prefix).name != "omnipicker" or sys.version_info[:2] != (3,11):
        raise SystemExit("请使用 Conda omnipicker 环境的 Python 3.11 启动。")
    os.environ.setdefault("QSG_RHI_BACKEND","d3d11")
    QQuickStyle.setStyle("Basic")
    app = QGuiApplication(sys.argv)
    app.setApplicationName("OmniPicker Studio")
    app.setOrganizationName("OmniPickerLocal")
    app.setFont(QFont("Microsoft YaHei UI",10))
    (ROOT/"logs").mkdir(exist_ok=True)
    instance_lock = QLockFile(str(ROOT/"logs/studio.lock"))
    if not instance_lock.tryLock(0):
        return 0  # Keep one owner of the physical USB interface.
    def message(kind,context,text):
        with (ROOT/"logs/qt.log").open("a",encoding="utf-8") as f:
            f.write(f"{datetime.now().isoformat()} {text}\n")
        print(text, file=sys.stderr)
    qInstallMessageHandler(message)
    bridge, rig = Bridge(), Rig()
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("studio",bridge)
    engine.rootContext().setContextProperty("rig",rig)
    engine.load(QUrl.fromLocalFile(str(ROOT/"qml/Main.qml")))
    if not engine.rootObjects():
        bridge.shutdown()
        return 1
    window = engine.rootObjects()[0]
    bridge.quitReady.connect(app.quit)
    def capture():
        path = ROOT/"logs"/("capture-"+datetime.now().strftime("%Y%m%d-%H%M%S")+".png")
        window.grabWindow().save(str(path))
        bridge.worker.engine.log("界面截图已保存："+str(path), action="screenshot")
    bridge.captureRequested.connect(capture)
    app.aboutToQuit.connect(bridge.shutdown)
    result = app.exec()
    del engine
    instance_lock.unlock()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
