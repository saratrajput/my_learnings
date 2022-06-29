import sys
from PyQt5.QtWidgets import QApplication, QWidget


app = QApplication(sys.argv)
win = QWidget()
win.setWindowTitle("PyQt5 GUI")  # Rename window title.
win.resize(400, 300)  # Set window size: width, height
win.show()


# app.exec()  # exec is a Python keyword.
# app.exec_()  # Qt exec_ ends with an underscore.
sys.exit(app.exec_())  # Cleanly exit on exceptions.