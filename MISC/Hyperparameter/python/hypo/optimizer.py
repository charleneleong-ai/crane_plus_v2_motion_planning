# -*- coding: utf-8 -*-
"""optimizer module

Note:
    参考記事
    - https://en.wikipedia.org/wiki/Hyperparameter_optimization
        - 一番近いのはハイパーパラメータ最適化
        - 順序の最適化などは対象外っぽい
        - wikiは代表的な手法やOSSがまとめてあり有用
    - https://en.wikipedia.org/wiki/Metaheuristic
        - メタヒューリスティック
        - 最適な解を探索
    - https://en.wikipedia.org/wiki/Hyper-heuristic
        - ハイパーヒューリステック
        - 最適なヒューリスティックを探索
    - https://en.wikipedia.org/wiki/Global_optimization
        - 大域的最適化はメタヒューリスティックを含む
    - https://en.wikipedia.org/wiki/Multi-objective_optimization
        - 多目的最適化
    - https://en.wikipedia.org/wiki/Mathematical_optimization
        - 数理最適化は一番広い括り
    参考OSS
    - http://hyperopt.github.io/hyperopt/
        - パラメータ空間が条件分岐する場合も扱えるようで参考になる
        - 特に以下のチュートリアルが重要
            - https://github.com/hyperopt/hyperopt/wiki/FMin
"""
import numpy as np
import ipywidgets as widgets
from IPython.display import display
from tqdm import tqdm
###############################################################################


class Optimizer:
    """
    Attributes:
        func (function): 目的関数 Objective function
            Input type is dictionary (or list).
            Output type is dictionary.
        input_dict (dict): Input for objective function
        space (SearchSpace): 探索空間 Search space
            Command 'display(self.space.widget)' displays GUI
            of the search space on jupyter notebook.
        result (ipywidgets): 結果
            Command 'display(self.result)' displays GUI
            of the result on jupyter notebook.
    """
    def __init__(self, func, space):
        """
        Args:
            func (function): 目的関数 Objective function
            space (SearchSpace): 探索空間 Search space
        """
        self.func = func
        self.space = space
        self.result = widgets.VBox()

    def minimize(self,
                 objectives=['f'],
                 constraints=[],
                 max_iterations=1000,
                 ):
        """最小化

        Minimize the objective function on the search space.

        Args:
            objectives (list of str): List of objective functions
                目的関数が返却する辞書のキーをリストで指定する．
                Specify keys of the dictionary
                which is returned by the objective function.
                単目的最適化の場合は単一のキーをリストで指定する．
                For single-purpose optimization,
                specify a single key in the list.
                多目的最適化の場合は複数のキーをリストで指定する．
                For multi-objective optimization,
                specify multiple keys in the list.
            constraints (list of str): 制約のリスト List of constraints
                目的関数が返却する辞書のキーをリストで指定する．
                Specify keys of the dictionary
                which is returned by the objective function.
                制約がない場合は空のリストを指定する．
                If there is no constraint, specify an empty list.
            max_iterations (int): 最大反復回数 Maximum number of iterations

        Todo:
            widgets.Play で一時停止・再開を自由にできるGUIにするのもアリ
        """
        self.objectives = objectives
        self.constraints = constraints
        # Display the search space and result widget.
        display(self.space.widget)
        display(self.result)
        # Initialize
        self.initialize()
        # Minimize
        for i in tqdm(range(max_iterations)):
            self.search()
            self.update()
        # Finalize
        self.finalize()

    def get_prefix(self, i, space, prefix=''):
        """
        Args:
            i (int): index of children
            space(SearchSpace): Target search space
            prefix (str): prefix
        """
        if hasattr(space.children[i], 'prefix'):
            prefix_ = prefix + space.children[i].prefix + '_'
        else:
            prefix_ = prefix
        return prefix_

    def get_param(self):
        """パラメータ取得

        探索空間のパラメータを取得する．
        目的関数に入力可能な形式の辞書を取得する．
        Get a parameter of the search space in a dictionary format
        that can be input to the objective function.
        """
        self.param = {}
        self._get_param(self.space)
        return self.param

    def _get_param(self, space, prefix=''):
        """パラメータ取得

        探索空間のパラメータを再帰的に取得する．
        Get a parameter of the search space recursively.
        """
        key = prefix + space.get_name()
        value = space.get_value()
        if value is not None:
            self.param.update({key: value})
        # Check children's parameter.
        if hasattr(space, 'children'):
            if space.children is not None:
                for i, subspace in enumerate(space.children):
                    prefix_ = self.get_prefix(i, space, prefix)
                    self._get_param(subspace, prefix_)

    def set_param(self, param):
        """パラメータ設定

        探索空間のパラメータを設定する．
        目的関数から出力結果を取得する．
        Set a parameter of the search space
        and get result from the objective function.
        """
        self.param = param
        self._set_param(self.space)
        output = self.func(param)
        self.set_result(output)

    def _set_param(self, space, prefix=''):
        """パラメータ設定

        探索空間のパラメータを再帰的に設定する．
        Set a parameters of the search space recursively.
        """
        key = prefix + space.get_name()
        if key in self.param.keys():
            space.set_value(self.param[key])

        # Check children's parameter.
        if hasattr(space, 'children'):
            if space.children is not None:
                for i, subspace in enumerate(space.children):
                    prefix_ = self.get_prefix(i, space, prefix)
                    self._set_param(subspace, prefix_)

    def get_score(self, output):
        """スコア取得

        Todo:
            多目的最適化の場合は未対応
        """
        score = sum([output[objective] for objective in self.objectives])
        return score

    def set_result(self, output):
        """結果格納

        Create widget for displaying results.

        Args:
            output (dict): Output dictionary of the objective function
        """
        children = []
        for key, value in output.items():
            if type(value) == bool:
                widget = widgets.Valid
            elif type(value) == str:
                widget = widgets.Text
            else:
                widget = widgets.FloatText
            children.append(widget(
                    description=key,
                    value=value,
                    ))
        self.result.children = children

    def visualize_pareto_frontier(self):
        """パレート境界可視化

        Todo:
            bokehで対話的な可視化を行う
            目的が2か3の場合のみ
            目的が4以上の場合は一部を固定しないと可視化できない
            各パラメータを固定・非固定できるようにできるといい
        """
        pass
###############################################################################


class GridSearch(Optimizer):
    """グリッド探索
    """
    pass
###############################################################################


class RandomSearch(Optimizer):
    """ランダム探索

    Attributes:
        best_param: Best parameter.
        best_result: Best result of target function.
        best_score: Best score.
        param_history(list): History of parameter.
        output_history(list): History of putput.
        score_history(list): History of score.
    """
    def initialize(self):
        """
        Initialize attributes.
        """
        self.best_param = None
        self.best_result = None
        self.best_score = np.inf
        # self.param_history = []
        # self.output_history = []
        # self.score_history = []

    def search(self):
        """
        Search on the search space.
        """
        self.space.set_random_value()

    def update(self):
        """
        Update the best parameter.
        """
        param = self.get_param()
        output = self.func(param)
        score = self.get_score(output)
        self.set_result(output)
        # self.param_history.append(param)
        # self.output_history.append(output)
        # self.score_history.append(score)
        if score < self.best_score:
            self.best_param = param
            self.best_output = output
            self.best_score = score

    def finalize(self):
        """
        Set the best parameter.
        """
        self.set_param(self.best_param)
###############################################################################
