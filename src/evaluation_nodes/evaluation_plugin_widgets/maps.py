from __future__ import annotations

from pathlib import Path

import numpy as np
from rospkg import RosPack

from python_qt_binding.QtCore import (
    QObject,
    QPoint,
    QRectF,
    Qt,
    QThread,
    QTimer,
    pyqtSignal,
)
from python_qt_binding.QtGui import (
    QBrush,
    QColor,
    QImage,
    QMouseEvent,
    QPixmap,
    QResizeEvent,
    QWheelEvent,
)
from python_qt_binding.QtWidgets import (
    QDialog,
    QFileDialog,
    QFrame,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
    QGridLayout,
    QProgressBar,
    QWidget,
)

from evaluation_nodes.compare_maps import FieldMap

_rospack = RosPack()


class CompareMapsThread(QObject):
    finished = pyqtSignal(float, float, tuple, tuple)

    def __init__(self, gt: FieldMap, pred: FieldMap) -> None:
        super().__init__()

        self._gt = gt
        self._pred = pred

    def run(self) -> None:
        weed_score, weed_matches = self._pred.compute_score_with(self._gt, "weed")
        litter_score, litter_matches = self._pred.compute_score_with(self._gt, "litter")

        self.finished.emit(weed_score, litter_score, weed_matches, litter_matches)


class PhotoViewer(QGraphicsView):
    # Source: https://stackoverflow.com/questions/35508711/how-to-enable-pan-and-zoom-in-a-qgraphicsview
    photoClicked = pyqtSignal(QPoint)

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self._zoom = 0
        self._empty = True
        self._scene = QGraphicsScene(self)
        self._photo = QGraphicsPixmapItem()
        self._scene.addItem(self._photo)
        self.setScene(self._scene)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QBrush(QColor(30, 30, 30)))
        self.setFrameShape(QFrame.NoFrame)

    def hasPhoto(self) -> None:
        return not self._empty

    def fitInView(self, scale: bool = True) -> None:
        rect = QRectF(self._photo.pixmap().rect())
        if not rect.isNull():
            self.setSceneRect(rect)
            if self.hasPhoto():
                unity = self.transform().mapRect(QRectF(0, 0, 1, 1))
                self.scale(1 / unity.width(), 1 / unity.height())
                viewrect = self.viewport().rect()
                scenerect = self.transform().mapRect(rect)
                factor = min(
                    viewrect.width() / scenerect.width(),
                    viewrect.height() / scenerect.height(),
                )
                self.scale(factor, factor)
            self._zoom = 0

    def setPhoto(self, pixmap: QPixmap | None = None) -> None:
        self._zoom = 0
        if pixmap and not pixmap.isNull():
            self._empty = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self._photo.setPixmap(pixmap)
        else:
            self._empty = True
            self.setDragMode(QGraphicsView.NoDrag)
            self._photo.setPixmap(QPixmap())
        self.fitInView()

    def wheelEvent(self, event: QWheelEvent) -> None:
        if self.hasPhoto():
            if event.angleDelta().y() > 0:
                factor = 1.25
                self._zoom += 1
            else:
                factor = 0.8
                self._zoom -= 1
            if self._zoom > 0:
                self.scale(factor, factor)
            elif self._zoom == 0:
                self.fitInView()
            else:
                self._zoom = 0

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        self.fitInView()

    def toggleDragMode(self) -> None:
        if self.dragMode() == QGraphicsView.ScrollHandDrag:
            self.setDragMode(QGraphicsView.NoDrag)
        elif not self._photo.pixmap().isNull():
            self.setDragMode(QGraphicsView.ScrollHandDrag)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if self._photo.isUnderMouse():
            self.photoClicked.emit(self.mapToScene(event.pos()).toPoint())
        super().mousePressEvent(event)


class ShowMapDialog(QDialog):
    def __init__(self, parent: QWidget) -> None:
        super().__init__(
            parent,
            flags=Qt.Window
            | Qt.WindowTitleHint
            | Qt.WindowMinimizeButtonHint
            | Qt.WindowMaximizeButtonHint
            | Qt.WindowCloseButtonHint,
        )

        self.setup_ui()

        gt_map_file = Path(_rospack.get_path("virtual_maize_field")) / "gt/map.csv"
        pred_map_file = (
            Path(_rospack.get_path("virtual_maize_field")) / "map/pred_map.csv"
        )

        # If file has not the expected name, open a filedialog to select the correct file
        if not pred_map_file.is_file():
            pred_map_file = Path(
                QFileDialog.getOpenFileName(
                    self,
                    "Open file",
                    str(pred_map_file.parent),
                    "Map files (*.csv)",
                )[0]
            )

            if not pred_map_file.is_file():
                self.close()
                return

        self.gt = FieldMap.from_csv(gt_map_file)
        self.pred = FieldMap.from_csv(pred_map_file)

        # Create map in background thread
        self.thread = QThread()
        self.worker = CompareMapsThread(self.gt, self.pred)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.finished.connect(self.show_image)
        self.thread.finished.connect(self.thread.deleteLater)

        # Wait 0.05 seconds to start thread to avoid freezing main window
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(self.thread.start)
        timer.start(50)

    def setup_ui(self) -> None:
        grid = QGridLayout()

        self.image_viewer = PhotoViewer(self)
        self.image_viewer.hide()
        grid.addWidget(self.image_viewer, 1, 1)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0)
        grid.addWidget(self.progress_bar, 2, 1)

        self.setLayout(grid)
        self.setGeometry(50, 50, 700, 700)
        self.setWindowTitle("Show map results")

    def show_image(
        self,
        weed_score: float,
        litter_score: float,
        weed_matches: tuple[np.ndarray, np.ndarray],
        litter_matches: tuple[np.ndarray, np.ndarray],
    ) -> None:
        fig = self.pred.plot_matches_with(
            self.gt,
            "pred",
            "gt",
            weed_matches,
            litter_matches,
            weed_score,
            litter_score,
            figsize=(10, 10),
        )
        output_file = (
            Path(_rospack.get_path("virtual_maize_field")) / "map/mapping_results.png"
        )
        fig.savefig(str(output_file), dpi=600)

        image = QImage(str(output_file))
        pixmap = QPixmap.fromImage(image)
        self.progress_bar.hide()
        self.image_viewer.show()

        self.image_viewer.setPhoto(pixmap)
