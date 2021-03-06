from __future__ import annotations

from argparse import ArgumentParser
from csv import writer as csv_writer
from json import loads as convert_to_python
from pathlib import Path

import rospy
from qt_gui.plugin import Plugin
from qt_gui.plugin_context import PluginContext
from qt_gui.settings import Settings
from rospkg import RosPack
from rosservice import get_service_list
from std_srvs.srv import Empty

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QGridLayout,
    QLabel,
    QLCDNumber,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from evaluation_nodes.evaluation_plugin_widgets.maps import ShowMapDialog
from evaluation_nodes.evaluation_plugin_widgets.robot_name import RobotNameWidget
from evaluation_nodes.evaluation_plugin_widgets.rviz import RVIZWidget
from evaluation_nodes.evaluation_plugin_widgets.settings import SettingsDialog
from evaluation_nodes.msg import Count

_rospack = RosPack()
_gazebo_timeout = 120  # seconds


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
        default=_rospack.get_path("virtual_maize_field") + "/rviz/config.rviz",
    )
    parser.add_argument(
        "--task",
        type=str,
        help="Current competition task.",
        choices=["navigation", "mapping"],
        default="navigation",
    )
    parser.add_argument(
        "--no_lookup_robot_name",
        action="store_true",
        help="Disable loopup robot name from parameter server.",
    )
    parser.add_argument(
        "--save_stats",
        action="store_true",
        help="Save statistics every second.",
    )

    def __init__(self, context: PluginContext) -> None:
        super().__init__(context)
        self.setObjectName("EvaluationPlugin")
        self.settings = {}
        self.last_time = rospy.Time.now()
        self.paused = True
        self.robot_name = ""

        # Parse optional arguments
        self.args, unknowns = EvaluationPlugin.parser.parse_known_args(context.argv())
        if self.args.verbose:
            rospy.loginfo(f"Arguments: {self.args}")
            rospy.loginfo(f"Unknown arguments: {unknowns}")

        self._pause_client = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self._play_client = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

        self._widget = self.setup_ui()

        self._check_gazebo_status_timer = QTimer(self)
        self._check_gazebo_status_timer.setInterval(500)
        self._check_gazebo_status_timer.timeout.connect(self._check_gazebo_status)
        self._check_gazebo_status_timer.start()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        self._count_subscriber = rospy.Subscriber(
            "fre_counter/info", Count, self._count_callback
        )

        self._stats_file_stream = (
            Path(_rospack.get_path("virtual_maize_field")) / "gt/stats.csv"
        ).open(mode="w")
        self._stats_file_writer = csv_writer(self._stats_file_stream)
        self._stats_file_writer.writerow(
            [
                "seconds_from_start",
                "robot_name",
                "destroyed_plants",
                "number_of_rows",
                "distance",
                "distance_in_row",
            ]
        )
        self._current_stats: Count | None = None
        self._prev_stats_time = rospy.Time.now()

        self._update_clock_timer = QTimer(self)
        self._update_clock_timer.setInterval(100)
        self._update_clock_timer.timeout.connect(self._clock_callback)
        self._update_clock_timer.start()

        self.start_time = rospy.Time.now()
        self.bonus_seconds = 0

    def setup_ui(self) -> QWidget:
        widget = QWidget()
        widget.setObjectName("EvaluationPluginUI")
        widget.setWindowTitle("Field Robot Event Evaluation")

        layout = QVBoxLayout(widget)

        # Add robot name widget
        if not self.args.no_lookup_robot_name:
            robot_name_widget = RobotNameWidget()
            self.robot_name = robot_name_widget.robot_name
            layout.addWidget(robot_name_widget)

        grid = QGridLayout()

        # Add remaining time
        remaining_time_label = QLabel("Remaining time [mm:ss]: ")
        remaining_time_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        grid.addWidget(remaining_time_label, 0, 0)
        self.remaining_time_widget = QLCDNumber()
        self.remaining_time_widget.setDigitCount(5)
        self.remaining_time_widget.setMinimumHeight(50)
        self.remaining_time_widget.setStyleSheet("border: 0px;")
        self.remaining_time_widget.setSegmentStyle(QLCDNumber.Filled)
        self.remaining_time_widget.display("00:00")
        grid.addWidget(self.remaining_time_widget, 0, 1)

        # Add number of hitted plants
        hit_plants_label = QLabel("Destroyed plants: ")
        hit_plants_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        grid.addWidget(hit_plants_label, 1, 0)
        self._hit_plant_widget = QLCDNumber()
        self._hit_plant_widget.setDigitCount(5)
        self._hit_plant_widget.setMinimumHeight(50)
        self._hit_plant_widget.setStyleSheet("border: 0px;")
        self._hit_plant_widget.setSegmentStyle(QLCDNumber.Filled)
        self._hit_plant_widget.display("000.0")
        grid.addWidget(self._hit_plant_widget, 1, 1)

        # Add driven distance
        distance_label = QLabel("Travelled distance [m]: ")
        distance_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        grid.addWidget(distance_label, 0, 2)
        self._distance_widget = QLCDNumber()
        self._distance_widget.setDigitCount(5)
        self._distance_widget.setMinimumHeight(50)
        self._distance_widget.setStyleSheet("border: 0px;")
        self._distance_widget.setSegmentStyle(QLCDNumber.Filled)
        self._distance_widget.display("000.0")
        grid.addWidget(self._distance_widget, 0, 3)

        # Add driven distance in row
        distance_in_row_label = QLabel("Travelled distance in row [m]: ")
        distance_in_row_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        grid.addWidget(distance_in_row_label, 1, 2)
        self._distance_in_row_widget = QLCDNumber()
        self._distance_in_row_widget.setDigitCount(5)
        self._distance_in_row_widget.setMinimumHeight(50)
        self._distance_in_row_widget.setStyleSheet("border: 0px;")
        self._distance_in_row_widget.setSegmentStyle(QLCDNumber.Filled)
        self._distance_in_row_widget.display("000.0")
        grid.addWidget(self._distance_in_row_widget, 1, 3)

        # Pause and play button
        self.play_pause_button = QPushButton("Play")
        self.play_pause_button.clicked.connect(self._toggle_play_pause)
        self.play_pause_button.setEnabled(False)
        grid.addWidget(self.play_pause_button, 0, 4)

        # Add button to give bonus time
        self._add_time_button = QPushButton("Add bonus time")
        self._add_time_button.clicked.connect(self._add_time)
        grid.addWidget(self._add_time_button, 1, 4)

        # Add compare maps buttons
        self._compare_maps_button = QPushButton("Show maps")
        self._compare_maps_button.clicked.connect(self._compare_maps)
        grid.addWidget(self._compare_maps_button, 2, 4)

        layout.addLayout(grid)

        # Add RVIZ
        rviz = RVIZWidget(self.args.rviz_config)
        layout.addWidget(rviz)

        return widget

    def _count_callback(self, msg: Count) -> None:
        if self.args.save_stats:
            self._current_stats = msg

        self._hit_plant_widget.display(f"{msg.plants_destroyed:05}")
        self._distance_widget.display(f"{msg.robot_distance:05.1f}")
        self._distance_in_row_widget.display(f"{msg.robot_distance_in_row:05.1f}")

    def _clock_callback(self) -> None:
        now = rospy.Time.now()

        if now - self._prev_stats_time > rospy.Duration(secs=1):
            self._save_stats(now)
            self._prev_stats_time = now

        if self.last_time == now and not self.paused:
            self.paused = True
            self.play_pause_button.setText("Play")

            rospy.loginfo("Pause simulation")

        elif self.last_time != now and self.paused:
            self.paused = False
            self.play_pause_button.setText("Pause")

            rospy.loginfo("Unpause simulation")

        if not self.paused:
            self.last_time = now

            elapsed_seconds = (rospy.Time.now() - self.start_time).to_sec()
            remaining_time = (
                int(round(self.settings["task_time_seconds"] - elapsed_seconds))
                + self.bonus_seconds
            )
            self.remaining_time_widget.display(
                f"{remaining_time//60:02}:{remaining_time%60:02}"
            )

            if self.settings["stop_simulation_automatically"] and remaining_time <= 0:
                rospy.loginfo("Stopped simulation because time limit is reached!")
                self._pause_client.call()
                self._update_clock_timer.stop()

    def _toggle_play_pause(self) -> None:
        if self.paused:
            self._play_client.call()
        else:
            self._pause_client.call()

    def _compare_maps(self) -> None:
        rospy.loginfo("Opening compare maps dialog.")
        dlg = ShowMapDialog(self._widget)
        dlg.showMaximized()

    def _add_time(self) -> None:
        rospy.loginfo(f"Adding bonus time (total +{self.bonus_seconds}s)")
        self.bonus_seconds += self.settings["bonus_time_seconds"]

    def _hide_unhide_task_mapping_buttons(self) -> None:
        self._compare_maps_button.setVisible(self.args.task == "mapping")
        self._add_time_button.setVisible(self.args.task == "mapping")

    def _save_stats(self, current_time: rospy.Time) -> None:
        if self._current_stats is not None:
            self._stats_file_writer.writerow(
                [
                    round((current_time - self.start_time).to_sec(), 3),
                    self.robot_name,
                    self._current_stats.plants_destroyed,
                    self._current_stats.rows_finished,
                    self._current_stats.robot_distance,
                    self._current_stats.robot_distance_in_row,
                ]
            )
        self._stats_file_stream.flush()

    def _check_gazebo_status(self) -> None:
        srv_list = get_service_list()

        if (
            "/gazebo/pause_physics" in srv_list
            and "/gazebo/unpause_physics" in srv_list
        ):
            self._check_gazebo_status_timer.stop()
            self.play_pause_button.setEnabled(True)
            self.start_time = rospy.Time.now()
            rospy.loginfo("Connected to Gazebo!")
        elif rospy.Time.now() - self.start_time > rospy.Duration(secs=_gazebo_timeout):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Could not connect to Gazebo")
            msg.setInformativeText("Did you launched the Gazebo environment?")
            msg.setWindowTitle("Error")
            msg.exec_()

    def shutdown_plugin(self) -> None:
        self._stats_file_stream.close()
        self._count_subscriber.unregister()
        self._update_clock_timer.stop()
        self._play_client.close()
        self._pause_client.close()

    def save_settings(
        self, plugin_settings: Settings, instance_settings: Settings
    ) -> None:
        for setting_name in (
            "task_time_seconds",
            "stop_simulation_automatically",
            "bonus_time_seconds",
        ):
            plugin_settings.set_value(setting_name, self.settings[setting_name])

    def restore_settings(
        self, plugin_settings: Settings, instance_settings: Settings
    ) -> None:
        for setting_name, default_value in zip(
            (
                "task_time_seconds",
                "stop_simulation_automatically",
                "bonus_time_seconds",
            ),
            ("180", "true", "30"),
        ):
            self.settings[setting_name] = convert_to_python(
                plugin_settings.value(setting_name, default_value)
            )

        self._hide_unhide_task_mapping_buttons()

    def trigger_configuration(self) -> None:
        dlg = SettingsDialog(self._widget, self.settings, self.args)
        if dlg.exec():
            rospy.loginfo("Apply and save settings.")
            t = dlg.max_time_widget.time().toPyTime()
            self.settings["task_time_seconds"] = (
                t.hour * 60 + t.minute
            ) * 60 + t.second
            self.settings[
                "stop_simulation_automatically"
            ] = dlg.stop_automatically_checkbox.isChecked()
            t = dlg.bonus_time_seconds_widget.time().toPyTime()
            self.settings["bonus_time_seconds"] = (
                t.hour * 60 + t.minute
            ) * 60 + t.second

            # Override task
            self.args.task = dlg.current_task_widget.currentText()

        self._hide_unhide_task_mapping_buttons()
