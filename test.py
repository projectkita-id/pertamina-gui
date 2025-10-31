from PySide6 import QtWidgets, QtGui, QtCore
import open3d as o3d
import sys

class Open3DEmbedWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QtWidgets.QVBoxLayout(self)

        self.label = QtWidgets.QLabel("Initializing Open3D...")
        self.layout.addWidget(self.label)

        # buat point cloud
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector([[0,0,0], [1,0,0], [0,1,0]])

        # buat Visualizer klasik
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible=True)  # window masih terpisah
        self.vis.add_geometry(self.pcd)

        # tunggu sebentar biar window siap
        QtCore.QTimer.singleShot(200, self.embed_window)

        # update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_vis)
        self.timer.start(30)

    def embed_window(self):
        try:
            # linux: gunakan handle dari Open3D Visualizer langsung
            hwnd = self.vis.get_window_handle()  # ini baru di Linux 0.17+
            if hwnd:
                qwindow = QtGui.QWindow.fromWinId(hwnd)
                container = self.createWindowContainer(qwindow, self)
                self.layout.addWidget(container)
                self.label.hide()
        except Exception as e:
            self.label.setText(f"Failed to embed Open3D: {e}")

    def update_vis(self):
        self.vis.poll_events()
        self.vis.update_renderer()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.widget = Open3DEmbedWidget()
        self.setCentralWidget(self.widget)
        self.setWindowTitle("Open3D Embedded in PySide6")
        self.resize(800, 600)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
