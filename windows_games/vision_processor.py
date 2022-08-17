"""
This module handles the vision processing.
"""
import logging
from dataclasses import dataclass

# Done by Frannecklp
import cv2
import numpy as np
import win32api
import win32con
import win32gui
import win32ui

# Get the top-level logger
log = logging.getLogger(__name__)


class VisionProcessor:
    threshold: float = 0.5

    def grab_screen(self, region=None):
        hwin = win32gui.GetDesktopWindow()

        if region:
            self.left, self.top, x2, y2 = region
            width = x2 - self.left + 1
            height = y2 - self.top + 1
        else:
            width = win32api.GetSystemMetrics(win32con.SM_CXVIRTUALSCREEN)
            height = win32api.GetSystemMetrics(win32con.SM_CYVIRTUALSCREEN)
            self.left = win32api.GetSystemMetrics(win32con.SM_XVIRTUALSCREEN)
            self.top = win32api.GetSystemMetrics(win32con.SM_YVIRTUALSCREEN)

        hwindc = win32gui.GetWindowDC(hwin)
        srcdc = win32ui.CreateDCFromHandle(hwindc)
        memdc = srcdc.CreateCompatibleDC()
        bmp = win32ui.CreateBitmap()
        bmp.CreateCompatibleBitmap(srcdc, width, height)
        memdc.SelectObject(bmp)
        memdc.BitBlt(
            (0, 0), (width, height), srcdc, (self.left, self.top), win32con.SRCCOPY
        )

        signedIntsArray = bmp.GetBitmapBits(True)
        img = np.fromstring(signedIntsArray, dtype="uint8")
        img.shape = (height, width, 4)

        srcdc.DeleteDC()
        memdc.DeleteDC()
        win32gui.ReleaseDC(hwin, hwindc)
        win32gui.DeleteObject(bmp.GetHandle())

        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    def find_template_location(self, template, source_img):
        """
        Match the resource template and click on resource to collect it.
        """
        w, h = template.shape[::-1]
        result = cv2.matchTemplate(source_img, template, cv2.TM_CCOEFF_NORMED)
        # threshold = 0.5
        loc = np.where(result >= self.threshold)

        template_locations = [pt for pt in zip(*loc[::-1])]

        if template_locations:
            first_match = template_locations[0]
            # template_center = (template_locations[0][0] + x0 + w / 2, template_locations[0][1] + self.top + h / 2)
            template_center = (
                first_match[0] + self.left + w / 2,
                first_match[1] + self.top + h / 2,
            )
            return template_center
        else:
            log.error("Template not found.")
            return None
