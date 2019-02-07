# -*- coding: utf-8 -*-
"""parameter module

Define the search space of parameters using ipywidgets.

Classes:
    SearchSpace: Define common function of SearchSpace.
    Block(SearchSpace): Merge multiple SearchSpace.
    Choice(SearchSpace): Select SearchSpace.
    Category(Choice): Category = Dropdown
    Boolean(Choice): Boolean = Checkbox
    Empty(SearchSpace): Empty SearchSpace
    Normal(SearchSpace): Normal distribution
    LogNormal(SearchSpace): Log-Normal distribution
    Uniform(SearchSpace): Uniform distribution
    LogUniform(SearchSpace): Log-Uniform distribution
    RandInt(SearchSpace): Random Int distribution
    Poisson(SearchSpace): Poisson distribution
    Binomial(SearchSpace): Binomial distribution

Note:
    For distribution please refer to the following URL.
    https://www.biwako.shiga-u.ac.jp/sensei/mnaka/ut/statdist.html
"""

import ipywidgets as widgets
import numpy as np
import math
import random
###############################################################################


class SearchSpace:
    """探索空間
    Base class for parameter search space

    Attributes:
        widget (ipywidgets): ウィジェット
        name (str): 名称
    """
    def get_name(self):
        return self.name

    def get_widget(self):
        return self.widget

    def get_value(self):
        try:
            return self.widget.value
        except AttributeError:
            return None

    def set_value(self, v):
        try:
            self.widget.value = v
        except AttributeError:
            pass

    def set_random_value(self):
        pass
###############################################################################


class Block(SearchSpace):
    """
    Merge multiple search spaces.
    Display border if this widget has children.

    Attributes:
        widget (widgets.VBox):
            Merged search spaces.
        layout (widget.layout):
            Default layout, which is used for display/hide toggle.
        children (tuple of searchSpace):
            Tuple of target search spaces.
            If Block has no search space, set Empty().
    """
    def __init__(self, *children):
        widget_list = [
                child.get_widget() for child in children
                if child.get_widget() is not None]
        # Hide border if widget_list has no widgets
        if len(widget_list) == 0:
            layout = widgets.Layout()
        else:
            layout = widgets.Layout(border='solid thin')
        self.widget = widgets.VBox(widget_list, layout=layout)
        self.layout = self.widget.layout
        self.children = children
        self.name = ''

    def hide_widget(self):
        self.widget.layout = widgets.Layout(height='0')

    def display_widget(self):
        self.widget.layout = self.layout

    def set_random_value(self):
        for i in range(len(self.children)):
            self.children[i].set_random_value()
###############################################################################


class Choice(SearchSpace):
    """
    Base class for Category/Boolean class.
    """
    def prepare(self):
        # Replace None with empty search space
        for i, child in enumerate(self.children):
            if child is None:
                self.children[i] = Block(Empty())
        # Associate widget with children
        if isinstance(self.widget, widgets.widget_bool._Bool):
            self.associate_boolean()
        elif isinstance(self.widget, widgets.widget_selection._Selection):
            self.associate_selection()
        else:
            raise ValueError
        # Rearange widget
        self.widget = widgets.VBox([self.widget, widgets.VBox([
                child.widget for child in self.children])])

    def display_child(self, i):
        """
        Display i-th child widget and hide all other children.
        """
        for child in self.children:
            child.hide_widget()
        self.children[i].display_widget()

    def associate_boolean(self):
        def on_value_change(change):
            self.display_child(change.new * 1)
        # Set prefix
        self.children[0].prefix = 'False'
        self.children[1].prefix = 'True'
        # Register change event
        self.widget.observe(on_value_change, names='value')
        # Reset widget
        self.display_child(self.widget.value * 1)

    def associate_selection(self):
        def on_index_change(change):
            self.display_child(change.new)
        # Set prefix using options
        for child, option in zip(self.children, self.widget.options):
            child.prefix = str(option)
        # Register change event
        self.widget.observe(on_index_change, names='index')
        # Reset widget
        self.display_child(self.widget.index)

    def get_value(self):
        if self.children is None:
            return self.widget.value
        else:
            return self.widget.children[0].value


class Category(Choice):
    """Category (DropDown)

    Assosiate options with children if children is not None.
    """
    def __init__(self, name, options=None, value='', children=None):
        # Check error
        if None in options:
            raise ValueError('options includes None: {}'.format(options))
        if len(options) != len(set(options)):
            raise ValueError(
                    'Options has Duplicate values: {}'.format(options))
        self.widget = widgets.Dropdown(
                description=name,
                options=options,
                value=value,
                )
        self.name = name
        self.value = value
        self.children = children
        self.options = options
        if children is not None:
            if len(children) != len(options):
                raise ValueError(
                        'The number of children is not equal to'
                        ' the number of options: {} {}'.format(
                                len(children), len(options)))
            self.prepare()

    def set_random_value(self):
        index = np.random.randint(0, len(self.options))
        if self.children is None:
            self.widget.index = index
        else:
            # Set a random value for the selected child
            self.widget.children[0].index = index
            self.children[index].set_random_value()


class Boolean(Choice):
    """Boolean (CheckBox)

    Assosiate value of checkbox with children if children is not None.
    """
    def __init__(self, name, value=False, children=None):
        self.widget = widgets.Checkbox(value=value, description=name)
        self.name = name
        self.value = value
        self.children = children
        if children is not None:
            if len(children) != 2:
                raise ValueError(
                        'The number of children must be 2: {}'
                        .format(children))
            self.prepare()

    def set_random_value(self):
        value = random.choice([False, True])
        if self.children is None:
            self.widget.value = value
        else:
            # Set a random value for the selected child
            self.widget.children[0].value = value
            self.children[value * 1].set_random_value()
###############################################################################


class Empty(SearchSpace):
    """空の探索空間
    Empty search space
    """
    def __init__(self):
        self.widget = None
        self.name = ''
###############################################################################


class Normal(SearchSpace):
    """正規分布
    Normal distribution
    """
    def __init__(self, name, mu=0.0, sigma=1.0):
        self.widget = widgets.FloatText(
                value=mu,
                step=sigma,
                description=name,
                )
        self.name = name
        self.mu = mu
        self.sigma = sigma

    def set_random_value(self):
        value = np.random.normal(loc=self.mu, scale=self.sigma)
        self.widget.value = value


class LogNormal(SearchSpace):
    """対数正規分布
    Log-Normal distribution
    """
    def __init__(self, name, base=10, mu=0.0, sigma=1.0):
        self.widget = widgets.FloatText(
                value=mu,
                step=sigma,
                description=name,
                )
        self.name = name
        self.mu = mu
        self.sigma = sigma
        self.base = base

    def set_random_value(self):
        value = np.random.normal(loc=self.mu, scale=self.sigma)
        self.widget.value = np.log(value, self.base)


class Uniform(SearchSpace):
    """一様分布
    Uniform distribution
    """
    def __init__(self, name, low=0, high=100, value=0, step=0.1):
        self.widget = widgets.FloatSlider(
                value=value,
                min=low,
                max=high,
                step=step,
                description=name,
                )
        self.name = name
        self.low = low
        self.high = high
        self.value = value
        self.step = step

    def set_random_value(self):
        value = np.random.uniform(self.low, self.high)  # XXX stepが働いていない
        self.widget.value = value


class LogUniform(SearchSpace):
    """対数一様分布
    Log-Uniform distribution
    """
    def __init__(self, name, base=10, low=-1.0, high=1.0, value=0.0, step=0.1):
        self.widget = widgets.FloatLogSlider(
                value=value,
                base=base,
                min=low,
                max=high,
                step=step,
                description=name,
                )
        self.name = name
        self.low = low
        self.high = high
        self.value = value
        self.step = step
        self.base = base

    def set_random_value(self):
        value = np.random.uniform(self.low, self.high)  # XXX stepが働いていない
        self.widget.value = math.log(value, self.base)
###############################################################################


class RandInt(SearchSpace):
    """ランダム整数
    Random Int distribution
    """
    def __init__(self, name, low=0, high=100, value=0, step=1):
        self.widget = widgets.IntSlider(
                value=value,
                min=low,
                max=high,
                step=step,
                description=name,
                )
        self.name = name
        self.low = low
        self.high = high
        self.value = value
        self.step = step

    def set_random_value(self):
        value = random.randrange(self.low, self.high + 1, self.step)
        self.widget.value = value


class Poisson(SearchSpace):
    """ポアソン分布
    Poisson distribution
    """
    def __init__(self, name, lam):
        self.widget = widgets.IntText(
                value=int(lam),
                min=0,
                description=name,
                )
        self.name = name
        self.lam = lam
        self.widget.observe(self.change_value, names='value')

    def change_value(self, change):
        if self.widget.value < 0:
            self.widget.value = 0

    def set_random_value(self):
        value = np.random.poisson(self.lam)
        self.widget.value = value


class Binomial(SearchSpace):
    """二項分布
    Binomial distribution
    """
    def __init__(self, name, n, p):
        self.widget = widgets.IntSlider(
                value=int(n * p),
                min=0,
                max=n,
                description=name,
                )
        self.name = name
        self.n = n
        self.p = p

    def set_random_value(self):
        value = np.random.binomial(self.n, self.p)
        self.widget.value = value
###############################################################################
