# -*- coding: utf-8 -*-
"""example module
"""

import hypo.parameter as pm
###############################################################################

svm_space = pm.Block(
        pm.LogUniform('C', base=10, low=-1, high=1, value=1.0),
        pm.Category(
                'kernel',
                options=['linear', 'RBF'],
                value='RBF',
                children=[
                        None,
                        pm.Block(pm.LogUniform(
                                'width',
                                base=10,
                                low=-1,
                                high=1,
                                value=1.0,
                                )),
                        ],
                )
        )

dtree_space = pm.Block(
        pm.Category(
                'criterion',
                options=['gini', 'entropy'],
                value='gini',
                ),
        pm.Category(
                'max_depth',
                options=[0, 1, 2, 3],
                value=2,
                ),
        pm.RandInt(
                'min_samples_split',
                low=1,
                high=2,
                value=2,
                step=1,
                ),
        )

sklearn_space = pm.Category(
        'type',
        options=['naive_bayes', 'svm', 'dtree'],
        value='dtree',
        children=[None, svm_space, dtree_space],
        )
###############################################################################

all_space = pm.Category(
        'data_type',
        options=['int', 'float'],
        value='float',
        children=[
                pm.Block(
                        pm.RandInt(name='RandInt'),
                        pm.Poisson(name='Poisson', lam=1),
                        pm.Binomial(name='Binomoal', n=10, p=2),
                        ),
                pm.Block(pm.Boolean(
                        name='log',
                        children=[
                                pm.Block(
                                        pm.Normal(name='Normal'),
                                        pm.Uniform(name='Uniform'),
                                        ),
                                pm.Block(
                                        pm.LogNormal(name='LogNormal'),
                                        pm.LogUniform(name='LogUniform'),
                                        ),
                                ],
                        )),
                ]
        )
