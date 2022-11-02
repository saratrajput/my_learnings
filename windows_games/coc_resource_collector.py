"""
Module Docstring.

author: Suraj Pattar
date: 17 April 2022
"""


import argparse
import logging
import time
from dataclasses import dataclass

import cv2
import numpy as np
import pyautogui

from grab_screen import grab_screen

# Initialize logging
log = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO
)  # Add filename='example.log' for logging to file


@dataclass
class ClashOfClans:
    top_left_x: int = 265
    top_left_y: int = 106
    bottom_right_x: int = 1718
    bottom_right_y: int = 928
    dual_screen: bool = True
    screen1_res = (0, 0)
    clash_of_clans_loc = (1100, 280)
    okay_loc = (1050, 650)

    def open_coc(self):
        """
        Open clash of clans on bluestacks.
        """
        # pyautogui.moveTo(self.clash_of_clans_loc)
        pyautogui.click(self.clash_of_clans_loc)

    def grab_coc(self, image):
        """
        # Use MPos to get the coordinates: https://sourceforge.net/projects/mpos/
        # Top-left
        # 265, 106
        # Bottom-right
        # 1718, 938
        # Add 1920 to width slices because of dual monitors
        # Crop width and height
        """
        if self.dual_screen:
            self.screen1_res = (1920, 1080)

        width = slice(
            self.top_left_x + self.screen1_res[0],
            self.bottom_right_x + self.screen1_res[0],
        )
        height = slice(self.top_left_x, self.bottom_right_y)

        image = image[height, width]
        self.img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def find_template_loc(self, template_path):
        """
        Match the resource template and click on resource to collect it.
        Template matching:
        https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
        """
        template = cv2.imread(template_path, 0)
        # Width and height of the template
        w, h = template.shape[::-1]

        result = cv2.matchTemplate(self.img_gray, template, cv2.TM_CCOEFF_NORMED)
        threshold = 0.5
        loc = np.where(result >= threshold)

        loc_pos = [pt for pt in zip(*loc[::-1])]

        resource_loc = (
            loc_pos[0][0] + self.top_left_x + w / 2,
            loc_pos[0][1] + self.top_left_y + h / 2,
        )
        return resource_loc

        # # pyautogui.moveTo(resource_loc)
        # pyautogui.click(resource_loc)

        # time.sleep(1)

    def exit_game(self):
        """
        Exit clash of clans on bluestacks.
        """
        # # Click somewhere on the app
        # pyautogui.click(self.clash_of_clans_loc)

        # Use the bluestacks hotkey to quit
        pyautogui.hotkey("ctrl", "shift", "2")

        # Click on Okay
        time.sleep(0.5)
        pyautogui.click(self.okay_loc)


def argument_parser():
    """
    Parse arguments.
    """
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--game_template",
        type=str,
        default="images/coc_template.png",
        help="Path to coc template.",
    )
    parser.add_argument(
        "--okay_template",
        type=str,
        default="images/okay_template.png",
        help="Path to coc template.",
    )
    parser.add_argument(
        "--elixir_template",
        type=str,
        default="images/elixir_collector_real.png",
        help="Path to elixir collector template.",
    )
    parser.add_argument(
        "--gold_template",
        type=str,
        default="images/gold_collector_real.png",
        help="Path to elixir collector template.",
    )
    parser.add_argument(
        "--dark_elixir_collector",
        type=str,
        default="images/dark_elixir_collector_real.png",
        help="Path to dark elixir collector template.",
    )

    return parser.parse_args()


def main(args):
    """
    Implement the main function.
    """
    coc = ClashOfClans()
    image = grab_screen()
    coc.grab_coc(image)

    # Open coc
    coc_loc = coc.find_template_loc(args.game_template)
    pyautogui.click(coc_loc)

    coc.open_coc()
    time.sleep(4)

    # Collect resources
    for template in [
        args.elixir_template,
        args.gold_template,
        args.dark_elixir_template,
    ]:
        pyautogui.click(coc.find_template_loc(template))
        time.sleep(1)

    # Exit game
    coc.exit_game()
    okay_loc = coc.find_template_loc(args.okay_template)
    pyautogui.click(okay_loc)


if __name__ == "__main__":
    args = argument_parser()
    main(args)
