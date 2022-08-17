import logging
import subprocess
import time

# Done by Frannecklp
import cv2
import numpy as np
import pyautogui
import win32api
import win32con
import win32gui
import win32ui

# Initialize logging
log = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG) # Add filename='example.log' for logging to file

def grab_screen(region=None):
    hwin = win32gui.GetDesktopWindow()

    if region:
        left, top, x2, y2 = region
        width = x2 - left + 1
        height = y2 - top + 1
    else:
        width = win32api.GetSystemMetrics(win32con.SM_CXVIRTUALSCREEN)
        height = win32api.GetSystemMetrics(win32con.SM_CYVIRTUALSCREEN)
        left = win32api.GetSystemMetrics(win32con.SM_XVIRTUALSCREEN)
        top = win32api.GetSystemMetrics(win32con.SM_YVIRTUALSCREEN)

    hwindc = win32gui.GetWindowDC(hwin)
    srcdc = win32ui.CreateDCFromHandle(hwindc)
    memdc = srcdc.CreateCompatibleDC()
    bmp = win32ui.CreateBitmap()
    bmp.CreateCompatibleBitmap(srcdc, width, height)
    memdc.SelectObject(bmp)
    memdc.BitBlt((0, 0), (width, height), srcdc, (left, top), win32con.SRCCOPY)

    signedIntsArray = bmp.GetBitmapBits(True)
    img = np.fromstring(signedIntsArray, dtype="uint8")
    img.shape = (height, width, 4)

    srcdc.DeleteDC()
    memdc.DeleteDC()
    win32gui.ReleaseDC(hwin, hwindc)
    win32gui.DeleteObject(bmp.GetHandle())

    return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)


def find_and_click_template_loc(template, source_img, x0, y0):
    """
    Match the resource template and click on resource to collect it.
    """
    w, h = template.shape[::-1]
    result = cv2.matchTemplate(source_img, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.5
    loc = np.where(result >= threshold)

    loc_pos = [pt for pt in zip(*loc[::-1])]
    log.debug("Loc pos: {}".format(loc_pos))

    resource_loc = (loc_pos[0][0] + x0 + w / 2, loc_pos[0][1] + y0 + h / 2)

    pyautogui.moveTo(resource_loc)

    pyautogui.click(resource_loc)
    time.sleep(1)


# Open Bluestacks
path_to_app = "C:\Program Files\BlueStacks_nxt\HD-Player.exe"
proc = subprocess.Popen(path_to_app)
time.sleep(10)

# Open Clash of Clans
window_name = "BlueStacks App Player"
window_handle = win32gui.FindWindow(None, window_name)
print(window_handle)
x0, y0, x1, y1 = win32gui.GetWindowRect(window_handle)

# Grab screen
open_screen_img = grab_screen((x0, y0, x1, y1))

open_screen_gray_img = cv2.cvtColor(open_screen_img, cv2.COLOR_BGR2GRAY)

coc_template = cv2.imread("images/coc_icon.png", 0)
w, h = coc_template.shape[::-1]

try:
    find_and_click_template_loc(coc_template, open_screen_gray_img, x0, y0)
except:
    log.error("Could not match template.")
    win32gui.CloseWindow(window_handle)

time.sleep(20)
elixir_template = cv2.imread("images/elixir_collector_real.png", 0)
gold_template = cv2.imread("images/gold_collector_real.png", 0)
dark_elixir_template = cv2.imread("images/dark_elixir_collector_real.png", 0)

coc_img = grab_screen((x0, y0, x1, y1))
coc_gray_img = cv2.cvtColor(coc_img, cv2.COLOR_BGR2GRAY)

for template in [elixir_template, gold_template, dark_elixir_template]:
    try:
        find_and_click_template_loc(template, coc_gray_img, x0, y0)
    except:
        log.error("Could not match template.")

win32gui.CloseWindow(window_handle)