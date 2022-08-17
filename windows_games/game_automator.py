"""
This module provides the methods to automate a game.
author: Suraj Pattar
date: 5 May 2022
"""
import logging
import subprocess
import time
from dataclasses import dataclass

# Done by Frannecklp
import pyautogui
import win32gui

# Get the top-level logger
log = logging.getLogger(__name__)


@dataclass
class GameAutomator:
    path_to_app: str = r"C:\Program Files\BlueStacks_nxt\HD-Player.exe"
    window_name: str = r"BlueStacks App Player"

    def open_application(self):
        """
        Open application.
        """
        proc = subprocess.Popen(self.path_to_app)
        time.sleep(10)
        self.window_handle = win32gui.FindWindow(None, self.window_name)
        log.debug("Window Handle: {}".format(self.window_handle))
        # self.x0, self.y0, self.x1, self.y1 = win32gui.GetWindowRect(window_handle)
        window_region = win32gui.GetWindowRect(self.window_handle)
        return self.window_handle, window_region

    def click_location(self, location):
        """
        Click the given location.
        """
        if location:
            log.info("Clicking on location: {}".format(location))
            pyautogui.moveTo(location)
            pyautogui.click(location)
            time.sleep(1)
            return True
        else:
            log.warning("No location provided.")
            return False

    def close_application(self):
        """
        Close window.
        """
        log.info("Closing application.")
        win32gui.CloseWindow(self.window_handle)
