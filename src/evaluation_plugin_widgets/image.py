from __future__ import annotations

from io import BytesIO

from python_qt_binding.QtWidgets import (
    QDialog,
    QLabel,
    QGridLayout,
)
from python_qt_binding.QtGui import QPixmap


class ImageDialog(QDialog):
    def __init__(self, buffer: BytesIO) -> None:
        super().__init__()

        self.img = QPixmap()
        self.img.loadFromData(buffer.getvalue())
        self.label = QLabel()
        self.label.setPixmap(self.img)

        self.grid = QGridLayout()
        self.grid.addWidget(self.label,1,1)
        self.setLayout(self.grid)

        self.setGeometry(50,50,320,200)
        self.setWindowTitle("Compare map results")
        self.show()