import datetime
import genetic
import unittest


class GuessPasswordTests(unittest.TestCase):
    gene_set = " abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!.,"

    def test_Hello_World(self):
        target = "Hello World!"
        self.guess_password(target)

    def test_For_I_am_fearfully_and_wonderfully_made(self):
        target = "For I am fearfully and wonderfully made."
        self.guess_password(target)

    def guess_password(self, target):
        start_time = datetime.datetime.now()

        def fn_get_fitness(genes):
            return get_fitness(genes, target)

        def fn_display(candidate):
            display(candidate, start_time)

        optimal_fitness = len(target)
        best = genetic.get_best(
            fn_get_fitness, len(target), optimal_fitness, self.geneset, fn_display
        )
        self.assertEqual(best.genes, target)

    def test_benchmark(self):
        genetic.Benchmark.run(self.test_For_I_am_fearfully_and_wonderfully_made)


# def display(genes, target, start_time):
def display(candidate, start_time):
    time_diff = datetime.datetime.now() - start_time
    print("{0}\t{1}\t{2}".format(candidate.genes, candidate.fitness, str(time_diff)))


def get_fitness(genes, target):
    return sum(1 for expected, actual in zip(target, genes) if expected == actual)


if __name__ == "__main__":
    unittest.main()
