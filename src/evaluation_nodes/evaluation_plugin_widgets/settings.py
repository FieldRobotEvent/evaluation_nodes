from __future__ import annotations

from argparse import Namespace
from typing import Any

from python_qt_binding.QtCore import QTime
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QTimeEdit,
    QVBoxLayout,
    QWidget,
)


class SettingsDialog(QDialog):
    def __init__(
        self, parent: QWidget, settings: dict[str, Any], args: Namespace
    ) -> None:
        super().__init__(parent)

        self.setWindowTitle("Field Robot Event Evaluation - settings")

        layout = QVBoxLayout()
        form = QFormLayout()

        current_task_label = QLabel("Current task:")
        self.current_task_widget = QComboBox()
        self.current_task_widget.addItems(["navigation", "mapping"])
        self.current_task_widget.setCurrentText(args.task)
        form.addRow(current_task_label, self.current_task_widget)

        max_time_label = QLabel("Task time (mm:ss):")
        self.max_time_widget = QTimeEdit()
        self.max_time_widget.setDisplayFormat("mm : ss")
        t = settings["task_time_seconds"]
        self.max_time_widget.setTime(QTime(t // 3600, t // 60, t % 60))
        form.addRow(max_time_label, self.max_time_widget)

        additional_time_label = QLabel(
            "Additional time after correct detection in task mapping (mm:ss):"
        )
        self.bonus_time_seconds_widget = QTimeEdit()
        self.bonus_time_seconds_widget.setDisplayFormat("mm : ss")
        t = settings["bonus_time_seconds"]
        self.bonus_time_seconds_widget.setTime(QTime(t // 3600, t // 60, t % 60))
        form.addRow(additional_time_label, self.bonus_time_seconds_widget)

        layout.addLayout(form)

        self.stop_automatically_checkbox = QCheckBox(
            "Stop simulation when time maximum time is elapsed."
        )
        self.stop_automatically_checkbox.setChecked(
            settings["stop_simulation_automatically"]
        )
        layout.addWidget(self.stop_automatically_checkbox)

        bbox = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Discard)
        bbox.button(QDialogButtonBox.Save).clicked.connect(self.save)
        bbox.button(QDialogButtonBox.Discard).clicked.connect(self.discard)
        layout.addWidget(bbox)

        self.setLayout(layout)

    def save(self) -> None:
        return super().accept()

    def discard(self) -> None:
        return super().reject()
