#!/usr/bin/env python3
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Monday, January 21st 2019, 11:56:25 am
# Modified By: Charlene Leong
###

# from session import Session
# from __future__ import scipy
import logging
import numpy as np

from smac.configspace import ConfigurationSpace
from ConfigSpace.hyperparameters import CategoricalHyperparameter, \
    UniformFloatHyperparameter, UniformIntegerHyperparameter

from smac.tae.execute_func import ExecuteTAFuncDict
from smac.scenario.scenario import Scenario
from smac.facade.smac_facade import SMAC

from session import Session

class SMACSession(Session):
    def __init__(self):
        super(SMACSession, self).__init__()

    def _objective(self, cfg, seed):
        """
            Creates a random forest regressor from sklearn and fits the given data on it.
            This is the function-call we try to optimize. Chosen values are stored in
            the configuration (cfg).
            Parameters:
            -----------
            cfg: Configuration
                configuration chosen by smac
            seed: int or RandomState
                used to initialize the rf's random generator
            Returns:
            -----------
            np.mean(rmses): float
                mean of root mean square errors of random-forest test predictions
                per cv-fold
        """
        rfr = RandomForestRegressor(
            n_estimators=cfg["num_trees"],
            criterion=cfg["criterion"],
            min_samples_split=cfg["min_samples_to_split"],
            min_samples_leaf=cfg["min_samples_in_leaf"],
            min_weight_fraction_leaf=cfg["min_weight_frac_leaf"],
            max_features=cfg["max_features"],
            max_leaf_nodes=cfg["max_leaf_nodes"],
            bootstrap=cfg["do_bootstrapping"],
            random_state=seed)

        def run(self):
            pass

