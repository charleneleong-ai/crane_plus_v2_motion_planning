[moveit_benchmark_statistics.py](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_ros/benchmarks/scripts/moveit_benchmark_statistics.py) from MoveIt! framework

Tuning Method from [Automatic Parameter Tuning of Motion Planning Algorithms (IROS 2018)](http://homepages.inf.ed.ac.uk/jcanore/pub/2018_iros.pdf)

A = Motion planning algo configured 

P = Set of planning params

M  = Set of cost metrics

 - planning time
 - path cost

Find parameter combo P* that allows for the best trajectory optimising M



Therefore how to define M(A, P)??