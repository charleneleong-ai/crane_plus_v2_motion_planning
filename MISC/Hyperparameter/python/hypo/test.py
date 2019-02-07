# -*- coding: utf-8 -*-
"""test module

最適化の性能評価のための関数

Note:
    https://en.wikipedia.org/wiki/Test_functions_for_optimization
"""

import numpy as np
import hypo.parameter as pm
###############################################################################


def Rastrigin_func(params):
    """Rastrigin function

    Global minimum: f(0, 0, ..., 0) = 0
    """
    a = 10
    n = len(params)
    f = a * n
    for x in params.values():
        f += x**2 - a*np.cos(2*np.pi*x)
    return {'f': f}


Rastringin_2D_space = pm.Block(
        pm.Uniform(name='x1', low=-5.12, high=5.12, value=0.0, step=1e-6),
        pm.Uniform(name='x2', low=-5.12, high=5.12, value=0.0, step=1e-6),
        )


def Sphere_func(params):
    """Sphere function

    Global minimum: f(0, 0, ..., 0) = 0
    """
    f = 0
    for x in params.values():
        f += x**2
    return {'f': f}


Sphere_2D_space = pm.Block(
        pm.Normal(name='x1', mu=0.0, sigma=1.0),
        pm.Normal(name='x2', mu=0.0, sigma=1.0),
        )


def McCormick_func(params):
    """McCormick function

    Global minimum: f(-0.54719,-1.54719) = -1.9133
    """
    x = params['x']
    y = params['y']
    f = np.sin(x + y) + (x - y)**2 - 1.5 * x + 2.5 * y + 1
    return {'f': f}


McCormick_space = pm.Block(
        pm.Uniform(name='x', low=-1.5, high=4.0, step=1e-6),
        pm.Uniform(name='y', low=-3.0, high=4.0, step=1e-6),
        )
###############################################################################


def Binh_and_Korn_func(params):
    x = params['x']
    y = params['y']
    return {
            'f1': 4*x**2 + 4*y**2,
            'f2': (x-5)**2 + (y-5)**2,
            'g1': (x-5)**2 + y**2 <= 25,
            'g2': (x-8)**2 + (y+3)**2 >= 7.7,
            }


Binh_and_Korn_space = pm.Block(
        pm.Uniform(name='x', value=0.0, low=0.0, high=5.0, step=1e-6),
        pm.Uniform(name='y', value=0.0, low=0.0, high=3.0, step=1e-6),
        )
###############################################################################


# XXX Sphere space for test 1
Sphere_boolean_space = pm.Boolean(
        name='CheckBox',
        value=True,
        children=[
                pm.Block(
                        pm.Normal(name='x1', mu=0.0, sigma=0.1),
                        pm.Normal(name='x2', mu=0.0, sigma=0.1),
                        ),
                pm.Block(
                        pm.Normal(name='y1', mu=0.0, sigma=0.1),
                        pm.Normal(name='y2', mu=0.0, sigma=0.1),
                        ),
                ]
        )


# XXX Sphere space for test 2
Sphere_category_space = pm.Category(
        name='DropDown',
        options=[1, 2],
        value=1,
        children=[
                pm.Block(
                        pm.Normal(name='x1', mu=0.0, sigma=0.1),
                        pm.Normal(name='x2', mu=0.0, sigma=0.1),
                        ),
                pm.Block(
                        pm.Normal(name='y1', mu=0.0, sigma=0.1),
                        pm.Normal(name='y2', mu=0.0, sigma=0.1),
                        ),
                ]
        )
