import sys
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QAction
from PyQt5.Qt import QLabel, QPushButton, QHBoxLayout, QVBoxLayout, QGridLayout
from PyQt5.QtCore import Qt


# class GUI(QWidget):  # Inherit from QWidget.
class GUI(QMainWindow):  # Inherit from QMainWindow.
    def __init__(self):
        super().__init__()  # Initialize super clas, which creates the Window.
        self.initUI()  # Refer to Window as self.

    def initUI(self):
        self.setWindowTitle("PyQt5 GUI")  # Add widgets and change properties.
        self.resize(400, 300)  # Resize window (width, height).
        self.add_menus_and_status()
        # self.positional_widget_layout()
        # self.horizontal_vertical_box_layout()
        self.layout_using_grid()

    def layout_using_grid(self):
        label_1 = QLabel("First label")
        label_2 = QLabel("Another label")
        label_span = QLabel("Label spanning columns span span span span")

        button_1 = QPushButton("Click 1")
        button_2 = QPushButton("Click 2")

        grid_layout = QGridLayout()

        grid_layout.addWidget(label_1, 0, 0)  # Row=0, Col=0
        grid_layout.addWidget(button_1, 0, 1)  # Row=0, Col=1
        grid_layout.addWidget(label_2, 1, 0)  # Row=1, Col=0
        grid_layout.addWidget(button_2, 1, 1)  # Row=1, Col=1
        grid_layout.addWidget(label_span, 2, 0, 1, 3)  # Row=2, Col=0, rowspan=1, colspan=3

        # grid_layout.setAlignment(Qt.AlignBottom)  # Align grid to the bottom.
        grid_layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)  # Align grid to top and left.
        grid_layout.setAlignment(label_1, Qt.AlignRight)  # Align label to the right.
        grid_layout.setAlignment(label_2, Qt.AlignRight)  # Align label to the right.

        layout_widget = QWidget()  # Create QWidget object.
        layout_widget.setLayout(grid_layout)  # Set layout.

        self.setCentralWidget(layout_widget)  # Make QWidget the central widget.

    def horizontal_vertical_box_layout(self):
        label_1 = QLabel("First Label.")
        label_2 = QLabel("Another Label.")

        button_1 = QPushButton("Click 1")
        button_2 = QPushButton("Click 2")

        hbox_1 = QHBoxLayout()
        hbox_1.addStretch()  # Push/Stretch to right.
        hbox_2 = QHBoxLayout()
        hbox_2.addStretch()  # Push/Stretch to right.

        hbox_1.addWidget(label_1)
        hbox_1.addWidget(button_1)

        hbox_2.addWidget(label_2)
        hbox_2.addWidget(button_2)

        vbox = QVBoxLayout()
        vbox.addStretch()  # Push/Stretch down.
        vbox.addLayout(hbox_1)
        vbox.addLayout(hbox_2)

        layout_widget = QWidget()  # Create QWidget object.
        layout_widget.setLayout(vbox)  # Set layout.

        self.setCentralWidget(layout_widget)  # Make QWidget the central widget.

    def positional_widget_layout(self):
        # Label w/out text, window is parent.
        # Default positio overlays menubar.
        label_1 = QLabel("Our first label", self)  
        label_1.move(10, 20)  # Position label below menubar.

        print(self.menuBar().size())
        mbar_height = self.menuBar().height()
        print(mbar_height)
        label_1.move(10, mbar_height)  # Position label below menubar.

        label_2 = QLabel("Another label", self)  # Create another label.
        label_2.move(10, mbar_height * 2)

        button_1 = QPushButton("Click 1", self)
        button_2 = QPushButton("Click 2", self)

        button_1.move(label_1.width(), label_1.height())
        button_2.move(label_1.width(), label_1.height() * 2)
        

    def add_menus_and_status(self):
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