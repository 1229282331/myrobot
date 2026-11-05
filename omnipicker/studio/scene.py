import math

from PySide6.QtCore import QObject, Property, Signal, Slot, QByteArray
from PySide6.QtGui import QVector3D, QQuaternion, QMatrix3x3
from PySide6.QtQuick3D import QQuick3DGeometry
from .model import Assembly, stl_vertices


class Mesh(QQuick3DGeometry):
    def __init__(self, path, parent=None):
        super().__init__(parent)
        v = stl_vertices(path)
        self.setVertexData(QByteArray(v.tobytes()))
        self.setStride(24)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        attr = QQuick3DGeometry.Attribute
        self.addAttribute(attr.Semantic.PositionSemantic,0,attr.ComponentType.F32Type)
        self.addAttribute(attr.Semantic.NormalSemantic,12,attr.ComponentType.F32Type)
        self.setBounds(QVector3D(*v[:,:3].min(0).tolist()),QVector3D(*v[:,:3].max(0).tolist()))


class Part(QObject):
    changed = Signal()

    def __init__(self, name, path, parent):
        super().__init__(parent)
        self._name = name
        self._geometry = Mesh(path)
        self._position = QVector3D()
        self._rotation = QQuaternion()

    name = Property(str, lambda self:self._name, constant=True)
    geometry = Property(QObject, lambda self:self._geometry, constant=True)
    position = Property(QVector3D, lambda self:self._position, notify=changed)
    rotation = Property(QQuaternion, lambda self:self._rotation, notify=changed)

    def update(self, matrix):
        self._position = QVector3D(*(matrix[:3,3]*1000).tolist())
        self._rotation = QQuaternion.fromRotationMatrix(QMatrix3x3(matrix[:3,:3].ravel().tolist()))
        self.changed.emit()


class Rig(QObject):
    displayChanged = Signal()

    def __init__(self):
        super().__init__()
        self.assembly = Assembly()
        self._parts = [Part(n,p,self) for n,p,v in self.assembly.visuals]
        self._session = None
        self._last_feedback = None
        self._opening = None
        self._mode = "preview"
        self._title = "目标预览 · 未连接"
        self._value = "15.0 %"
        self._detail = "仅编辑目标 · 未连接设备"
        self.setOpening(15)

    parts = Property("QVariantList", lambda self:self._parts, constant=True)
    displayMode = Property(str, lambda self:self._mode, notify=displayChanged)
    opening = Property(float, lambda self:self._opening, notify=displayChanged)
    displayTitle = Property(str, lambda self:self._title, notify=displayChanged)
    displayValue = Property(str, lambda self:self._value, notify=displayChanged)
    displayDetail = Property(str, lambda self:self._detail, notify=displayChanged)
    modelVisible = Property(bool, lambda self:self._mode != "waiting", notify=displayChanged)

    @Slot("QVariantMap", float)
    def updateDisplay(self, state, target):
        """Choose the visualization source without submitting any control intent.

        A connection never falls back to its command target. Cached observations
        are scoped to one connection; missing feedback freezes or hides the rig.
        """
        connected = bool(state.get("connected"))
        session = state.get("connectionId") if connected else None
        if session != self._session or not connected:
            self._session = session
            self._last_feedback = None
        position = state.get("feedbackPosition")
        age = state.get("feedbackAgeMs", -1)
        valid = (connected and state.get("feedbackValid")
                 and isinstance(position, (int, float)) and math.isfinite(position)
                 and -5 <= position <= 105
                 and isinstance(age, (int, float)) and 0 <= age <= 300)
        if not connected:
            self._mode = "preview"
            self._title = "目标预览 · 未连接"
            self._value = f"{target:.1f} %"
            self._detail = "仅编辑目标 · 未连接设备"
            opening = target
        elif valid:
            self._last_feedback = position
            self._mode = "simulated" if state.get("simulated") else "feedback"
            self._title = "模拟反馈姿态" if state.get("simulated") else "反馈驱动姿态"
            self._value = f"{position:.0f} %"
            self._detail = f"回传位置 · 编辑目标 {target:.1f} %"
            opening = position
        elif self._last_feedback is not None:
            self._mode = "stale"
            self._title = "反馈失效 · 姿态冻结"
            self._value = "—"
            self._detail = f"末次回传 {self._last_feedback:.0f} % · 非实时"
            opening = self._last_feedback
        else:
            self._mode = "waiting"
            self._title = "等待位置反馈"
            self._value = "—"
            self._detail = "有效反馈到达后显示模型"
            opening = None
        # Small normalized endpoint overshoots remain visible in the raw value;
        # only the uncalibrated geometric visualization is bounded to its range.
        if opening is not None:
            self.setOpening(max(0., min(100., opening)))
        self.displayChanged.emit()

    @Slot(float)
    def setOpening(self, percent):
        if percent == self._opening:
            return
        matrices = self.assembly.transforms(self.assembly.nominal_angles(percent))
        for part, (name,_,visual) in zip(self._parts,self.assembly.visuals):
            part.update(matrices[name] @ visual)
        self._opening = percent
