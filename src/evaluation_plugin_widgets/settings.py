from __future__ import annotations

from typing import Any

import rospy

from python_qt_binding.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QTimeEdit,
    QVBoxLayout,
)


class SettingsDialog(QDialog):
    def __init__(self, settings: dict[str, Any]):
        super().__init__()

        self.setWindowTitle("Field Robot Event Evaluation - settings")

        layout = QVBoxLayout()
        form = QFormLayout()

        max_time_label = QLabel("Task time (mm:ss):")
        self.max_time_widget = QTimeEdit()
        self.max_time_widget.setDisplayFormat("mm : ss")
        self.max_time_widget.setTime(settings["task_time"])
        form.addRow(max_time_label, self.max_time_widget)

        layout.addLayout(form)

        bbox = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Discard)
        bbox.button(QDialogButtonBox.Save).clicked.connect(self.save)
        bbox.button(QDialogButtonBox.Discard).clicked.connect(self.discard)
        layout.addWidget(bbox)

        self.setLayout(layout)

    def save(self) -> None:
        rospy.loginfo(f"Changing task time to {self.max_time_widget.time().toPyTime()}")
        return super().accept()

    def discard(self) -> None:
        return super().reject()
