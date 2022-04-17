"""
Module Docstring.

author:name
date:date
"""


import argparse
import logging
import time
from dataclasses import dataclass

# Done by Frannecklp
import cv2
import numpy as np
import pyautogui

from grab_screen import grab_screen

# Initialize logging
log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO) # Add filename='example.log' for logging to file


@dataclass
class ClashOfClans():
    top_left_x: int = 265
    top_left_y: int = 106
    bottom_right_x: int = 1718
    bottom_right_y: int = 928
    dual_screen: bool = True
    screen1_res = (1920, 1080)

    def click_loc(self, loc):
        pass

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
            x1 = self.top_left_x + self.screen1_res[0]
            x2 = self.bottom_right_x + self.screen1_res[0]

        width = slice(x1, x2)
        height = slice(self.top_left_x, self.top_left_y)
        # # image = grab_screen()
        # width = slice(2185, 3638)
        # width = slice(self.x1, 3638)
        # height = slice(106, 938)

        image = image[height, width]
        return image


def grab_coc(full_image):


def open_coc():
    """
    Open clash of clans on bluestacks.
    """
    pyautogui.moveTo(clash_of_clans_loc)
    pyautogui.click(clash_of_clans_loc)
    

def exit_game():
    """
    Exit clash of clans on bluestacks.
    """
    # Click somewhere on the app
    pyautogui.click(clash_of_clans_loc)
    # Use the bluestacks hotkey to quit
    pyautogui.hotkey('ctrl','shift','2')
    # Click on Okay
    time.sleep(0.5)
    pyautogui.click(okay_loc)


# Template matching
# https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
def collect_resource(template):
    """
    Match the resource template and click on resource to collect it.
    """
    w, h = template.shape[::-1]
    result = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.5
    loc = np.where(result >= threshold)

    loc_pos = [pt for pt in zip(*loc[::-1])]

    resource_loc = (loc_pos[0][0] + blue_stack_win_top_left[0] + w/2, loc_pos[0][1] + blue_stack_win_top_left[1] + h/2)

    pyautogui.moveTo(resource_loc)

    pyautogui.click(resource_loc)
    time.sleep(1)


def argument_parser():
    """
    Parse arguments.
    """
    parser = argparse.ArgumentParser()

    parser.add_argument("--opt_str", type=str,
                        default="string_arg",
                        help="Optional string argument")
    parser.add_argument("--opt_bool", type=bool,
                        default=True,
                        help="Optional bool argument")
    parser.add_argument("--opt_int", type=int,
                        default=0,
                        help="Optional int argument")
    return parser.parse_args()


def main(args):
    """
    Implement the main function.
    """
    # log.info("Optional arguments: {}, {}, {}".format(args.opt_str, args.opt_bool,
    #     args.opt_int))
    # log.info("Hello, name!")
    coc = ClashOfClans()
    image = grab_screen()

    # grab_coc(image)
    clash_of_clans_loc = (1100, 280)
    okay_loc = (1050, 650)

    open_coc(clash_of_clans_loc)

    img_rgb = coc.grab_coc(image)
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)


    elixir_template = cv2.imread('images/elixir_collector_real.png', 0)
    gold_template = cv2.imread('images/gold_collector_real.png', 0)
    dark_elixir_template = cv2.imread('images/dark_elixir_collector_real.png', 0)


    blue_stack_win_top_left = (267, 107)

    for template in [elixir_template, gold_template, dark_elixir_template]:
        collect_resource(template)


if __name__=="__main__":
    args = argument_parser()
    main(args)
