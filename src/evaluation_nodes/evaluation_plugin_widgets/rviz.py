from __future__ import annotations

from io import StringIO
from numpy import disp

from rospkg import RosPack
from rviz import bindings as rviz
from yaml import safe_dump, safe_load

from python_qt_binding.QtWidgets import QVBoxLayout, QWidget

_rospack = RosPack()


class RVIZWidget(QWidget):
    def __init__(self, config_file: str) -> None:
        super().__init__()

        rviz_config = self.init_rviz_config(config_file)

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath(
            _rospack.get_path("evaluation_nodes") + "/misc/fre_icon.png"
        )
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readString(config, rviz_config)
        self.frame.load(config)

        self.setWindowTitle(config.mapGetChild("Title").getValue())

        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)
        self.frame.hideLeftDock(True)
        self.frame.hideRightDock(True)

        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        layout = QVBoxLayout()
        layout.addWidget(self.frame)
        self.setLayout(layout)

    @staticmethod
    def init_rviz_config(config_file: str, base_link_name: str = "base_link") -> str:
        output_stream = StringIO()

        with open(config_file, "r") as content_stream:
            content = safe_load(content_stream)

            # Clear panels on side
            content["Panels"].clear()

            # Disable save on exit
            content["Preferences"]["PromptSaveOnExit"] = False

            # Remove tool buttons
            content["Toolbars"]["toolButtonStyle"] = 0

            # Remove all tools except the interaction, move and focus tool
            _tools_list = []
            for tool in content["Visualization Manager"]["Tools"]:
                if tool["Class"] in (
                    "rviz/Interact",
                    "rviz/MoveCamera",
                    "rviz/FocusCamera",
                ):
                    _tools_list.append(tool)
            content["Visualization Manager"]["Tools"] = _tools_list
            
            # Make grid larger and fixed to the odom frame
            _display_list = []
            for display in content["Visualization Manager"]["Displays"]:
                if display["Name"] == "Grid":
                    display["Plane Cell Count"] = 1000
                    display["Reference Frame"] = "odom"

                _display_list.append(display)
            content["Visualization Manager"]["Displays"] = _display_list

            # Make camera move with base_link
            content["Visualization Manager"]["Global Options"]["Fixed Frame"] = base_link_name

            safe_dump(content, output_stream)

        output_stream.seek(0)
        return output_stream.read()
