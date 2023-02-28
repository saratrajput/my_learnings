import pytest
from bubble_sort import bubble_sort


def test_bubbleSort():
    input_list = [3, 2, 1]
    output_list = bubble_sort(input_list)

    assert output_list == [1, 2, 3]


def test_bubbleSort2():
    input_list = [5, 8, 3, 2, 1, 18]
    output_list = bubble_sort(input_list)

    assert output_list == [1, 2, 3, 5, 8, 18]
