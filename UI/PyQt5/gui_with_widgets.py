import sys
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QAction


# class GUI(QWidget):  # Inherit from QWidget.
class GUI(QMainWindow):  # Inherit from QMainWindow.
    def __init__(self):
        super().__init__()  # Initialize super clas, which creates the Window.
        self.initUI()  # Refer to Window as self.

    def initUI(self):
        self.setWindowTitle("PyQt5 GUI")  # Add widgets and change properties.
        self.resize(400, 300)  # Resize window (width, height).
        self.statusBar().showMessage('Text in statusbar')

        # File menu
        menubar = self.menuBar()  # Create a menu bar.
        file_menu = menubar.addMenu('File')  # Add menu to menu bar.
        new_action = QAction('New', self)  # Create an action.
        file_menu.addAction(new_action)
        new_action.setStatusTip('New File')  # Status bar updated.

        file_menu.addSeparator()  # Add a separator line between menu items.

        exit_action = QAction('Exit', self)  # Create exit action.
        exit_action.setStatusTip('Click to exit the application')
        exit_action.triggered.connect(self.close)  # Close application when clicked.
        exit_action.setShortcut('Ctrl+Q')  # Keyboard shortcut to close application. Main window has focus.
        file_menu.addAction(exit_action)

        # Edit menu
        edit_menu = menubar.addMenu('Edit')  # Add a second menu.


if __name__ == "__main__":
    app = QApplication(sys.argv)  # Create application.
    gui = GUI()  # Create instance of class.
    gui.show()  # Show the constructed PyQt window.
    sys.exit(app.exec_())  # Execute the application.