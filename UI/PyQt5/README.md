# Python GUI Programming Recipes using PyQt5

### Section 1: 

* Install PyQt5
```
pip install pyqt5
```

* Install Qt Designer
```
pip install pyqt5-tools
```

### Section 2: Designing Python GUIs with Qt Designer
* Installing the PyQt5 designer tools.
* Creating our first GUI using Qt Designer.
* Adding controls to our GUI with Qt Designer.
* Using Qt Designer for widget layout.
* Converting Qt Designer code to Python code.

* Launch Qt Designer
```
designer
```

* Run GUI in Qt-Designer by pressing ```Ctrl+R```.

* Convert ```.ui``` file to ```.py``` file
```
pyuic5 -o ./gui_4_gui_to_python_code.py ./gui_4_gui_to_python_code.ui 
```

* Make it executable
```
pyuic5 -x -o ./gui_4_gui_to_python_code.py ./gui_4_gui_to_python_code.ui 
```

### Section 3: Enhancing the Qt5 GUI Functionality
* Calling dialogs from the main window.
* Decoupling Python code from generated UI code.
* Building a complex GUI with PyQt5.
* Multi-threading keeps our GUI responsive.
* Using Drag and Drop within the PyQt5 GUI.