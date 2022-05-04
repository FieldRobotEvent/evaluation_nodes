from __future__ import annotations

from argparse import ArgumentParser
from datetime import time
from typing import TYPE_CHECKING

import rospy
from python_qt_binding.QtCore import Qt, QTime
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QTimeEdit,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin
from urdf_parser_py.urdf import URDF

from evaluation_nodes.msg import Count

if TYPE_CHECKING:
    from typing import Any

    from qt_gui.plugin_context import PluginContext
    from qt_gui.settings import Settings


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


class EvaluationPlugin(Plugin):
    parser = ArgumentParser()
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Use plugin in verbose mode.",
    )
    parser.add_argument(
        "--no_lookup_robot_name",
        action="store_true",
        help="Disable loopup robot name from parameter server.",
    )

    def __init__(self, context: PluginContext) -> None:
        super().__init__(context)
        self.setObjectName("EvaluationPlugin")
        self.settings = {}

        # Parse optional arguments
        args, unknowns = EvaluationPlugin.parser.parse_known_args(context.argv())
        if args.verbose:
            print(f"Arguments: {args}")
            print(f"Unknown arguments: {unknowns}")

        self._widget = QWidget()
        self._widget.setObjectName("EvaluationPluginUI")
        self._widget.setWindowTitle("Field Robot Event Evaluation")

        layout = QVBoxLayout(self._widget)

        # Get robot name
        if not args.no_lookup_robot_name:
            while not rospy.has_param("robot_description") and not rospy.is_shutdown():
                rospy.loginfo_once(
                    "Waiting for the robot description in the param server"
                )
                rospy.sleep(0.5)
            rospy.loginfo("Found the robot description in the param server")

            robot_name_widget = QLabel(f"{URDF.from_parameter_server().name}")
            robot_name_widget.setFont(QFont("Arial", 25))
            robot_name_widget.setAlignment(Qt.AlignCenter)
            layout.addWidget(robot_name_widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        # Register subscribers
        self._count_subscriber = rospy.Subscriber(
            "fre_counter/info", Count, self._count_callback
        )

    def _count_callback(self, msg: Count) -> None:
        pass

    def shutdown_plugin(self) -> None:
        self._count_subscriber.unregister()

    def save_settings(
        self, plugin_settings: Settings, instance_settings: Settings
    ) -> None:
        plugin_settings.set_value("task_time", self.settings["task_time"])

    def restore_settings(
        self, plugin_settings: Settings, instance_settings: Settings
    ) -> None:
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self.settings["task_time"] = plugin_settings.value(
            "task_time", time(minute=3, second=0)
        )

    def trigger_configuration(self) -> None:
        dlg = SettingsDialog(self.settings)
        if dlg.exec():
            rospy.loginfo("Apply and save settings.")
            self.settings["task_time"] = dlg.max_time_widget.time().toPyTime()
