#!/usr/bin/python

import subprocess

if __name__ == '__main__':
    print "Running the planner repeatedly with different params"
    command_str = "rosrun deformable_astar planner3d"
    pareto = 0.0
    ctrl = 0
    for i in range(101):
        for j in range(1,4):
            ctrl = j
            pareto = float(i) / 100.0
            filepath = "path_" + str(pareto) + "_" + str(ctrl) + "[p_c]"
            full_cmd = command_str + " " + str(pareto) + " " + str(ctrl) + " " + filepath
            print "Running command: " + full_cmd
            subprocess.call(full_cmd, shell=True)
