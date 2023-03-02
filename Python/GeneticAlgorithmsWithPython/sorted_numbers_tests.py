import genetic
import unittest
import datetime


class SortedNumbersTests(unittest.TestCase):
    def test_sort_10_numbers(self):
        self.sort_numbers(10)

    def sort_numbers(self, total_numbers):
        gene_set = [i for i in range(100)]
        start_time = datetime.datetime.now()

        def fn_display(candidate):
            display(candidate, start_time)

        def fn_get_fitness(genes):
            return get_fitness(genes)

        optimal_fitness = total_numbers

        best = genetic.get_best(
            fn_get_fitness, total_numbers, optimal_fitness, gene_set, fn_display
        )
        self.assertTrue(not optimal_fitness > best.fitness)


def get_fitness(genes):
    fitness = 1

    for i in range(1, len(genes)):
        if genes[i] > genes[i - 1]:
            fitness += 1

    return fitness


def display(candidate, start_time):
    time_diff = datetime.datetime.now() - start_time
    print(
        "{0}\t=> {1}\t{2}".format(
            ", ".join(map(str, candidate.genes)), candidate.fitness, str(time_diff)
        )
    )
