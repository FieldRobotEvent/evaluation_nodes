from __future__ import annotations

import rospy
from urdf_parser_py.urdf import URDF

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QLabel


class RobotNameWidget(QLabel):
    def __init__(self) -> None:
        while not rospy.has_param("robot_description") and not rospy.is_shutdown():
            rospy.loginfo_once("Waiting for the robot description in the param server")
            rospy.sleep(0.5)

        robot_name = URDF.from_parameter_server().name

        super().__init__(robot_name)

        self.setFont(QFont("Arial", 25))
        self.setAlignment(Qt.AlignCenter)
