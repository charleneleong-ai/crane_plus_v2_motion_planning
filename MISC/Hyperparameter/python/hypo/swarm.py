# -*- coding: utf-8 -*-
"""swarm module
"""
from . import optimizer
##############################################################################


class SwarmIntelligence(optimizer.Optimizer):
    """群知能

    Note:
        https://en.wikipedia.org/wiki/Swarm_intelligence
    """


class AntColonyOptimization(SwarmIntelligence):
    """蟻コロニー最適化
    """
    pass


class ParticleSwarmOptimization(SwarmIntelligence):
    """粒子群最適化
    """
    pass
