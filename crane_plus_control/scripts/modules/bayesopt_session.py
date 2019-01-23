#!/usr/bin/env python3
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Wednesday, January 23rd 2019, 4:54:09 pm
# Modified By: Charlene Leong
###

# from session import Session
from bayes_opt import BayesianOptimization

# class BayesOptSession(Session):
#     def __init__(self):
#         super(BayesOptSession, self).__init__()
        
#     def run(self):
#         pass

def black_box_function(x, y):
    """Function with unknown internals we wish to maximize.

    This is just serving as an example, for all intents and
    purposes think of the internals of this function, i.e.: the process
    which generates its output values, as unknown.
    """
    return (-x ** 2 - (y - 1) ** 2 + 1)

# Bounded region of parameter space
pbounds = {'x': (2, 4), 'y': (-3, 3)}

optimizer = BayesianOptimization(
    f=black_box_function,
    pbounds=pbounds,
    random_state=1,
)

print(optimizer.max)