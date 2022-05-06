from __future__ import annotations

from argparse import ArgumentParser
from datetime import time

import rospy
from qt_gui.plugin import Plugin
from qt_gui.plugin_context import PluginContext
from qt_gui.settings import Settings
from rospkg import RosPack
from urdf_parser_py.urdf import URDF

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QFormLayout,
    QLabel,
    QLCDNumber,
    QVBoxLayout,
    QWidget,
)

from evaluation_nodes.msg import Count
from evaluation_plugin_widgets.robot_name import RobotNameWidget
from evaluation_plugin_widgets.rviz import RVIZWidget
from evaluation_plugin_widgets.settings import SettingsDialog

_rospack = RosPack()


class EvaluationPlugin(Plugin):
    parser = ArgumentParser()
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Use plugin in verbose mode.",
    )
    parser.add_argument(
        "--rviz_config",
        type=str,
        help="Path to the RVIZ configuration file",
        default=_rospack.get_path("evaluation_nodes") + "/config/config.rviz",
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
            rospy.loginfo(f"Arguments: {args}")
            rospy.loginfo(f"Unknown arguments: {unknowns}")

        self._widget = QWidget()
        self._widget.setObjectName("EvaluationPluginUI")
        self._widget.setWindowTitle("Field Robot Event Evaluation")

        layout = QVBoxLayout(self._widget)

        # Add robot name widget
        if not args.no_lookup_robot_name:
            robot_name_widget = RobotNameWidget()
            layout.addWidget(robot_name_widget)

        f_layout = QFormLayout()
        # f_layout.setFormAlignment(Qt.AlignLeft)

        lcd = QLCDNumber()
        lcd.setDigitCount(5)
        lcd.setMinimumHeight(50)
        lcd.setStyleSheet("border: 0px;")
        lcd.setSegmentStyle(QLCDNumber.Filled)
        lcd.display("05:00")

        lcd_label = QLabel("Remaining time: ")
        lcd_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        f_layout.addRow(lcd_label, lcd)

        layout.addLayout(f_layout)

        # Add RVIZ
        rviz = RVIZWidget(args.rviz_config)
        layout.addWidget(rviz)

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
