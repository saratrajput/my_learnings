def bubble_sort(unsorted_list):
    for i in range(len(unsorted_list)):
        for j in range(i, len(unsorted_list)):
            if unsorted_list[i] > unsorted_list[j]:
                temp = unsorted_list[i]
                unsorted_list[i] = unsorted_list[j]
                unsorted_list[j] = temp

    return unsorted_list
