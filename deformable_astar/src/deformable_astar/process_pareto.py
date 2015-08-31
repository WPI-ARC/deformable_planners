#!/usr/bin/python

import numpy
import os
import matplotlib
import matplotlib.pyplot
import subprocess
import math

def ParsePathFile(filepath):
    [pareto, env_option] = GetParams(filepath)
    path_file = open(filepath)
    path_lines = path_file.readlines()
    path_file.close()
    data = []
    dist = 0.0
    cost = 0.0
    avg_sum = 0.0
    prev_state = None
    for path_line in path_lines:
        [x,y,z] = path_line.rstrip("\n").split(",")
        state = [float(x),float(y),float(z)]
        state_cost = GetCost(state, env_option)
        cost += state_cost
        dist += GetDist(state, prev_state)
        data.append(state)
        prev_state = state
    print len(path_lines)
    return [env_option, pareto, data, dist, cost]

def GetParams(filepath):
    pareto = float(filepath.split("/")[-1].split("_")[1])
    env_option = env_option = filepath.split("[p_c]")[0][-1]
    return [pareto, env_option]

def GetCost(new_state, env_option):
    #print new_state
    if (env_option == "0"):
        return 0.0
    full_cmd = "../bin/statechecker " + env_option + " " + str(new_state[0]) + " " + str(new_state[1]) + " " + str(new_state[2])
    #print full_cmd
    result = subprocess.check_output(full_cmd, shell=True)
    #print "Command finished"
    relevant_line = result.split("\n")[-2]
    #print relevant_line
    chunks = relevant_line.split(" ")
    return float(chunks[3])

def GetDist(new_state, prev_state):
    if (prev_state is None):
        return 0.0
    else:
        xd = new_state[0] - prev_state[0]
        yd = new_state[1] - prev_state[1]
        zd = new_state[2] - prev_state[2]
        return math.sqrt(xd**2 + yd**2 + zd**2)

def DrawLengthFigure(data):
    # Make a new figure
    figure = matplotlib.pyplot.figure()
    plot = figure.add_subplot(111)
    # Draw the path
    [shx,shy] = ExtractXY(data, "1", "length")
    [hsx,hsy] = ExtractXY(data, "2", "length")
    [ssx,ssy] = ExtractXY(data, "3", "length")
    matplotlib.pyplot.plot(ssx,ssy,color='b')
    matplotlib.pyplot.plot(shx,shy,color='g')
    matplotlib.pyplot.plot(hsx,hsy,color='m')
    # Show it
    matplotlib.pyplot.show()

def ExtractXY(data, env, field):
    data_dict = {}
    for element in data:
        if (element[0] == env):
            temp_dict = {}
            temp_dict["length"] = element[3]
            temp_dict["deform"] = element[4]
            data_dict[str(element[1])] = temp_dict
    p = []
    l = []
    d = []
    for p_key in sorted(data_dict.keys()):
        p.append(float(p_key))
        l.append(data_dict[p_key]["length"])
        d.append(data_dict[p_key]["deform"])
    if (field == "deform"):
        return [p,d]
    elif (field == "length"):
        return [p,l]
    else:
        print "WTF?!"

def DrawDeformFigure(data):
    # Make a new figure
    figure = matplotlib.pyplot.figure()
    plot = figure.add_subplot(111)
    # Draw the path
    [shx,shy] = ExtractXY(data, "1", "deform")
    [hsx,hsy] = ExtractXY(data, "2", "deform")
    [ssx,ssy] = ExtractXY(data, "3", "deform")
    matplotlib.pyplot.plot(ssx,ssy,color='b')
    matplotlib.pyplot.plot(shx,shy,color='g')
    matplotlib.pyplot.plot(hsx,hsy,color='m')
    # Show it
    matplotlib.pyplot.show()

if __name__ == '__main__':
    path = subprocess.check_output("rospack find astar_deform", shell=True)
    path = path.strip("\n") + "/data"
    possible_files = os.listdir(path)
    relevant_files = []
    for filename in possible_files:
        if ("[p_c].csv" in filename):
            relevant_files.append(filename)
    directory_data = []
    for filename in relevant_files:
        file_data = ParsePathFile(path + "/" + filename)
        print "Env type: " + file_data[0] + " Deformation: " + str(file_data[4]) + " Length: " + str(file_data[3])
        directory_data.append(file_data)
    hardhard_data = []
    softhard_data = []
    hardsoft_data = []
    softsoft_data = []
    for element in directory_data:
        if (element[0] == "0"):
            hardhard_data.append([element[1], element[3], element[4]])
        elif (element[0] == "1"):
            softhard_data.append([element[1], element[3], element[4]])
        elif (element[0] == "2"):
            hardsoft_data.append([element[1], element[3], element[4]])
        elif (element[0] == "3"):
            softsoft_data.append([element[1], element[3], element[4]])
        else:
            print "WTF?!"
    assert(len(hardhard_data) == len(softhard_data) == len(hardsoft_data) == len(softsoft_data))
    DrawLengthFigure(directory_data)
    DrawDeformFigure(directory_data)
