from video_2_decoupling_python_code_from_generated_ui_code import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets


def set_table_items():
    row = 0
    ui.tableWidget.setItem(row, 0, QtWidgets.QTableWidgetItem("item 1"))
    ui.tableWidget.setItem(1, 1, QtWidgets.QTableWidgetItem("item 2"))
    ui.tableWidget.setItem(2, 2, QtWidgets.QTableWidgetItem("item 3"))

def button_clicked():
    ui.pushButton.setText("Button was clicked!")

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()

    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    # Call function
    set_table_items()

    ui.pushButton.clicked.connect(button_clicked)

    MainWindow.show()
    sys.exit(app.exec_())
