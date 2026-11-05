from PySide6.QtCore import QObject, QUrl
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuickControls2 import QQuickStyle
from studio.bridge import Bridge
from studio.scene import Rig
from studio.device import ROOT


def test_qml_loads_geometry_and_starts_without_device(qapp,qtbot):
    QQuickStyle.setStyle('Basic')
    bridge,rig=Bridge(),Rig()
    engine=QQmlApplicationEngine();warnings=[]
    engine.warnings.connect(lambda errors:warnings.extend(str(x) for x in errors))
    engine.rootContext().setContextProperty('studio',bridge)
    engine.rootContext().setContextProperty('rig',rig)
    engine.load(QUrl.fromLocalFile(str(ROOT/'qml/Main.qml')))
    try:
        assert len(engine.rootObjects())==1
        win=engine.rootObjects()[0]
        qtbot.wait(150)
        assert not warnings
        assert not bridge.state['connected'] and bridge.worker.engine.device is None
        assert len(rig.parts)==9
        assert all(p.geometry.vertexData().size()>0 for p in rig.parts)
        for width,height in [(1100,760),(1440,900)]:
            win.resize(width,height)
            qtbot.wait(120)
            capture=win.grabWindow()
            assert not capture.isNull()
            capture.save(str(ROOT/'logs'/f'qa-{win.devicePixelRatio():g}x-{width}x{height}.png'))
        assert not warnings
        bridge.leave('close','keep')
        qtbot.waitUntil(lambda:not win.isVisible(),timeout=2000)
        assert win.property('allowClose')
    finally:
        bridge.shutdown()
        engine.deleteLater()


def test_qml_feedback_drives_pose_and_editing_cannot_move_it(qapp, qtbot):
    QQuickStyle.setStyle('Basic')
    bridge, rig = Bridge(), Rig()
    # This view test uses injected observations only; no device is opened.
    bridge.shutdown()
    qtbot.wait(20)
    engine = QQmlApplicationEngine()
    warnings = []
    engine.warnings.connect(lambda errors:warnings.extend(str(x) for x in errors))
    engine.rootContext().setContextProperty('studio', bridge)
    engine.rootContext().setContextProperty('rig', rig)
    engine.load(QUrl.fromLocalFile(str(ROOT/'qml/Main.qml')))
    try:
        win = engine.rootObjects()[0]
        value = win.findChild(QObject, 'posePositionValue')
        view = win.findChild(QObject, 'gripperView')
        bridge.editTarget(35., False)
        assert rig.displayMode == 'preview' and rig.opening == 35
        state = dict(bridge.state, connected=True, connectionId='view-a', simulated=True,
                     feedbackValid=True, feedbackPosition=15., feedbackVelocity=0.,
                     feedbackColumn1=0., feedbackAgeMs=8., feedbackHz=100.,
                     feedbackMessage='离线注入观测 · 非实机', compatible=True)
        bridge.receive('state', state)
        qtbot.wait(40)
        assert rig.displayMode == 'simulated' and rig.opening == 15
        assert value.property('text') == '15 %'
        for width, height in [(1100,760), (1440,900)]:
            win.resize(width, height)
            qtbot.wait(100)
            win.grabWindow().save(str(ROOT/'logs'/f'qa-pose-feedback-{width}x{height}.png'))
        bridge.editTarget(0., False)
        assert rig.opening == 15
        bridge.receive('state', dict(state, feedbackPosition=22.))
        assert rig.opening == 22 and value.property('text') == '22 %'
        bridge.receive('state', dict(state, feedbackValid=False, feedbackPosition=None))
        bridge.editTarget(100., False)
        assert rig.opening == 22 and rig.displayMode == 'stale'
        assert value.property('text') == '—' and view.property('opacity') < .5
        qtbot.wait(100)
        win.grabWindow().save(str(ROOT/'logs'/'qa-pose-stale.png'))
        bridge.receive('state', dict(state, connectionId='view-b', feedbackValid=False))
        assert not view.property('visible') and rig.displayMode == 'waiting'
        assert not warnings
        assert bridge.worker.engine.device is None
    finally:
        win.setProperty('allowClose', True)
        win.close()
        bridge.shutdown()
        engine.deleteLater()
