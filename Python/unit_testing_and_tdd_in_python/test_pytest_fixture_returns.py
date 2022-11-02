import pytest


@pytest.fixture(params=[1, 2, 3])
def setup(request):
    retVal = request.param
    print("\nSetup! retVal = {}".format(retVal))
    return retVal


def test1(setup):
    print("\nsetup = {}".format(setup))
    assert True
