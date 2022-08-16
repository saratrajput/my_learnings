import win32gui


window_name = "Calculator"
window_handle = win32gui.FindWindow(None, window_name)
print(window_handle)
x0, y0, x1, y1 = win32gui.GetWindowRect(window_handle)
w = x1 - x0
h = y1 - y0
x0 = 0
y0 = 0

win32gui.MoveWindow(window_handle, x0, y0, 100, 100, True)