import random
import statistics
import time


class Chromosome:
    genes = None
    fitness = None

    def __init__(self, genes, fitness):
        self.genes = genes
        self.fitness = fitness


def _generate_parent(length, gene_set, get_fitness):
    genes = []
    while len(genes) < length:
        sample_size = min(length - len(genes), len(gene_set))
        genes.extend(random.sample(gene_set, sample_size))

    genes = "".join(genes)
    fitness = get_fitness(genes)

    return Chromosome(genes, fitness)


def _mutate(parent, gene_set, get_fitness):
    index = random.randrange(0, len(parent.genes))
    child_genes = list(parent.genes)

    new_gene, alternate = random.sample(gene_set, 2)

    child_genes[index] = alternate if new_gene == child_genes[index] else new_gene

    genes = "".join(child_genes)
    fitness = get_fitness(genes)

    return Chromosome(genes, fitness)


def get_best(get_fitness, target_len, optimal_fitness, gene_set, display):
    random.seed()
    best_parent = _generate_parent(target_len, gene_set, get_fitness)
    display(best_parent)

    if best_parent.fitness >= optimal_fitness:
        return best_parent

    while True:
        child = _mutate(best_parent, gene_set, get_fitness)

        if best_parent.fitness >= child.fitness:
            continue
        display(child)

        if child.fitness >= optimal_fitness:
            return child

        best_parent = child


class Benchmark:
    @staticmethod
    def run(function):
        timings = []
        for i in range(100):
            start_time = time.time()
            function()
            seconds = time.time() - start_time

            timings.append(seconds)
            mean = statistics.mean(timings)
            print(
                "{0} {1:3.2f} {2:3.2f}".format(
                    1 + i, mean, statistics.stdev(timings, mean) if i > 1 else 0
                )
            )

