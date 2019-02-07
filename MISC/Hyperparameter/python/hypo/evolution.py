# -*- coding: utf-8 -*-
"""evolution module
"""
from . import optimizer
##############################################################################


class EvolutionaryAlgorithm(optimizer.Optimizer):
    """進化的アルゴリズム

    Note:
        https://en.wikipedia.org/wiki/Evolutionary_algorithm
    """
    pass


class GeneticAlgorithm(EvolutionaryAlgorithm):
    """遺伝的アルゴリズム
    """
    pass


class GeneticProgramming(EvolutionaryAlgorithm):
    """遺伝的プログラミング
    """
    pass


class EvolutionStrategy(EvolutionaryAlgorithm):
    """進化的戦略
    """
    pass


class EvolutionaryProgramming(EvolutionaryAlgorithm):
    """進化的プログラミング
    """
    pass
