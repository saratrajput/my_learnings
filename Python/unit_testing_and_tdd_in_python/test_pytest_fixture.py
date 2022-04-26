import pytest

# @pytest.fixture() # Method-1
@pytest.fixture(autouse=True)
def setup():
    print("\nSetup")

# def test1(setup): # Method-1
def test1():
    print("Executing test1!")
    assert True

# @pytest.mark.usefixtures("setup") # Method-1
def test2():
    print("Executing test2!")
    assert True