import pytest
from .game_automator import GameAutomator


def test_CanInstantiateGameAutomator():
    GameAutomator()


def test_CanOpenGame():
    game_automator = GameAutomator()
    win_handle = game_automator.open_application()
    assert win_handle != 0
