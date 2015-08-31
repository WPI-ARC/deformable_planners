#!/usr/bin/python

import numpy
import os
import matplotlib
import matplotlib.pyplot
import subprocess
import math

def ParsePathFile(filepath, calibration):
    path_file = open(filepath)
    path_lines = path_file.readlines()
    path_file.close()
    data = []
    avg_sum = 0.0
    for path_line in path_lines:
        chunks = path_line.rstrip("\n").split(",")
        if ("Computed" not in chunks[0]):
            line_dict = {}
            line_dict["computed_deform"] = calibration * float(chunks[0])
            line_dict["computed_deform_sum"] = calibration * float(chunks[1])
            line_dict["measured_forward"] = chunks[2]
            line_dict["measured_forward_sum"] = chunks[4]
            line_dict["measured_backward"] = chunks[3]
            line_dict["measured_backward_sum"] = chunks[5]
            line_dict["average_measured"] = (float(chunks[2]) + float(chunks[3])) / 2.0
            line_dict["average_measured_sum"] = avg_sum + line_dict["average_measured"]
            avg_sum = line_dict["average_measured_sum"]
            data.append(line_dict)
    return data

def CalcRatio(a, b):
    a = float(a)
    b = float(b)
    if (a == 0.0 or b == 0.0):
        return 0.0
    else:
        return a / b

def ComputeRatios(filedata):
    for datapoint in filedata:
        computed = datapoint["computed_deform"]
        measured_f = datapoint["measured_forward"]
        measured_b = datapoint["measured_backward"]
        average = datapoint["average_measured"]
        ratio_f = CalcRatio(measured_f, computed)
        ratio_b = CalcRatio(measured_b, computed)
        ratio_avg = CalcRatio(average, computed)
        datapoint["ratio_forward"] = ratio_f
        datapoint["ratio_backward"] = ratio_b
        datapoint["ratio_average"] = ratio_avg
        print "Computed ratios of " + str(ratio_f) + " [f], " + str(ratio_b) + " [b], " + str(ratio_avg) + " [avg]"
    return filedata

def ExtractXYFromData(data_dict_list, key):
    rx = []
    ry = []
    for i in range(len(data_dict_list)):
        rx.append(float(i))
        ry.append(data_dict_list[i][key])
    return [rx, ry]

def DrawFigure(path_file, calibration):
    print "Loading planner data files for run " + path_file
    path_data = ParsePathFile(path_file, calibration)
    processed_path = ComputeRatios(path_data)
    print "Planner data files loaded - now displaying"
    # Make a new figure
    figure = matplotlib.pyplot.figure()
    plot = figure.add_subplot(111)
    #Draw data
    #[cx,cy] = ExtractXYFromData(processed_path, "computed_deform")
    [csx,csy] = ExtractXYFromData(processed_path, "computed_deform_sum")
    [mfx,mfy] = ExtractXYFromData(processed_path, "measured_forward")
    [mfsx,mfsy] = ExtractXYFromData(processed_path, "measured_forward_sum")
    [mbx,mby] = ExtractXYFromData(processed_path, "measured_backward")
    [mbsx,mbsy] = ExtractXYFromData(processed_path, "measured_backward_sum")
    [rfx,rfy] = ExtractXYFromData(processed_path, "ratio_forward")
    [rbx,rby] = ExtractXYFromData(processed_path, "ratio_backward")
    [rax,ray] = ExtractXYFromData(processed_path, "ratio_average")
    matplotlib.pyplot.plot(csx,csy,color='b')
    matplotlib.pyplot.plot(mfsx,mfsy,color='g')
    matplotlib.pyplot.plot(mbsx,mbsy,color='r')
    matplotlib.pyplot.plot(rfx,rfy,label="forward ratio",color='m')
    matplotlib.pyplot.plot(rbx,rby,label="backward ratio",color='y')
    matplotlib.pyplot.plot(rax,ray,label="average ratio",color='k')
    plot.legend(("computed","forward","backward", "forward ratio", "backward ratio", "average ratio"), "upper center", shadow=False, fancybox=True)
    # Draw the samples
    #matplotlib.pyplot.scatter(sx,sy,color='r')
    # Annotate
    #matplotlib.pyplot.annotate("Start", (px[0],py[0]))
    #matplotlib.pyplot.annotate("Goal", (px[-1],py[-1]))
    # Set axes
    #plot.set_xlim(0,1)
    maxY = max(max(rfy),max(rby),max(ray))
    #plot.set_ylim(0,maxY)
    # Add the title
    plot.set_title("Computed and measured deformation and measured/computed ratios for : " + path_file)
    # Show it
    matplotlib.pyplot.show()
    print "Finished processing planner data files for " + path_file

def ComputeAverageMeasured(file_prefix, num_files, calibration):
    avg_dict = {}
    avg_dict["computed_deform"] = 0.0
    avg_dict["computed_deform_sum"] = 0.0
    avg_dict["measured_forward"] = 0.0
    avg_dict["measured_forward_sum"] = 0.0
    avg_dict["measured_backward"] = 0.0
    avg_dict["measured_backward_sum"] = 0.0
    avg_dict["average_measured"] = 0.0
    avg_dict["average_measured_sum"] = 0.0
    file_data = []
    for i in range(num_files):
        file_data.append(ParsePathFile(file_prefix + str(i + 1) + ".csv", calibration))
    avg_data = []
    for i in range(len(file_data[0])):
        avg_dict = {}
        avg_dict["computed_deform"] = 0.0
        avg_dict["computed_deform_sum"] = 0.0
        avg_dict["measured_forward"] = 0.0
        avg_dict["measured_forward_sum"] = 0.0
        avg_dict["measured_backward"] = 0.0
        avg_dict["measured_backward_sum"] = 0.0
        avg_dict["average_measured"] = 0.0
        avg_dict["average_measured_sum"] = 0.0
        for j in range(num_files):
            data_dict = file_data[j][i]
            avg_dict["computed_deform"] = avg_dict["computed_deform"] + float(data_dict["computed_deform"])
            avg_dict["computed_deform_sum"] = avg_dict["computed_deform_sum"] + float(data_dict["computed_deform_sum"])
            avg_dict["measured_forward"] = avg_dict["measured_forward"] + float(data_dict["measured_forward"])
            avg_dict["measured_forward_sum"] = avg_dict["measured_forward_sum"] + float(data_dict["measured_forward_sum"])
            avg_dict["measured_backward"] = avg_dict["measured_backward"] + float(data_dict["measured_backward"])
            avg_dict["measured_backward_sum"] = avg_dict["measured_backward_sum"] + float(data_dict["measured_backward_sum"])
            avg_dict["average_measured"] = avg_dict["average_measured"] + float(data_dict["average_measured"])
            avg_dict["average_measured_sum"] = avg_dict["average_measured_sum"] + float(data_dict["average_measured_sum"])
        avg_dict["computed_deform"] = abs(avg_dict["computed_deform"] / float(num_files))
        avg_dict["computed_deform_sum"] = abs(avg_dict["computed_deform_sum"] / float(num_files))
        avg_dict["measured_forward"] = abs(avg_dict["measured_forward"] / float(num_files))
        avg_dict["measured_forward_sum"] = abs(avg_dict["measured_forward_sum"] / float(num_files))
        avg_dict["measured_backward"] = abs(avg_dict["measured_backward"] / float(num_files))
        avg_dict["measured_backward_sum"] = abs(avg_dict["measured_backward_sum"] / float(num_files))
        avg_dict["average_measured"] = abs(avg_dict["average_measured"] / float(num_files))
        avg_dict["average_measured_sum"] = abs(avg_dict["average_measured_sum"] / float(num_files))
        avg_data.append(avg_dict)
    return avg_data

def DrawAverageFigure(file_prefix, num_files, calibration, figure_title):
    print "Loading planner data files for run " + file_prefix
    path_data = ComputeAverageMeasured(file_prefix, num_files, calibration)
    processed_path = ComputeRatios(path_data)
    print "Planner data files loaded - now displaying"
    # Make a new figure
    figure = matplotlib.pyplot.figure()
    plot = figure.add_subplot(111)
    #Draw data
    #[cx,cy] = ExtractXYFromData(processed_path, "computed_deform")
    [csx,csy] = ExtractXYFromData(processed_path, "computed_deform_sum")
    [mfx,mfy] = ExtractXYFromData(processed_path, "measured_forward")
    [mfsx,mfsy] = ExtractXYFromData(processed_path, "measured_forward_sum")
    [mbx,mby] = ExtractXYFromData(processed_path, "measured_backward")
    [mbsx,mbsy] = ExtractXYFromData(processed_path, "measured_backward_sum")
    [ax,ay] = ExtractXYFromData(processed_path, "average_measured")
    [asx,asy] = ExtractXYFromData(processed_path, "average_measured_sum")
    [rfx,rfy] = ExtractXYFromData(processed_path, "ratio_forward")
    [rbx,rby] = ExtractXYFromData(processed_path, "ratio_backward")
    [rax,ray] = ExtractXYFromData(processed_path, "ratio_average")
    matplotlib.pyplot.plot(csx,csy,color='b')
    #matplotlib.pyplot.plot(asx,asy,color='g')
    matplotlib.pyplot.plot(mfsx,mfsy,color='g')
    #matplotlib.pyplot.plot(mbsx,mbsy,color='r')
    #matplotlib.pyplot.plot(rfx,rfy,label="forward ratio",color='m')
    #matplotlib.pyplot.plot(rbx,rby,label="backward ratio",color='y')
    #matplotlib.pyplot.plot(rax,ray,label="average ratio",color='k')
    #plot.legend(("Computed","Measured"), "upper left", shadow=False, fancybox=True)
    # Draw the samples
    #matplotlib.pyplot.scatter(sx,sy,color='r')
    # Annotate
    #matplotlib.pyplot.annotate("Start", (px[0],py[0]))
    #matplotlib.pyplot.annotate("Goal", (px[-1],py[-1]))
    # Set axes
    #plot.set_xlim(0,1)
    #maxY = max(max(rfy),max(rby),max(ray))
    #plot.set_ylim(0,maxY)
    # Add the title
    plot.set_xlabel("State # in path")
    plot.set_ylabel("Deformation")
    #plot.set_title(figure_title)
    # Show it
    #matplotlib.pyplot.show()
    print "Finished processing planner data files for " + file_prefix
    return processed_path

def DrawCombinedCostFigure(lowest_path, middle_path, highest_path):
    # Make a new figure
    figure = matplotlib.pyplot.figure()
    plot = figure.add_subplot(111)
    #Draw data
    [lsx,lsy] = ExtractXYFromData(lowest_path, "average_measured_sum")
    [msx,msy] = ExtractXYFromData(middle_path, "average_measured_sum")
    [hsx,hsy] = ExtractXYFromData(highest_path, "average_measured_sum")
    matplotlib.pyplot.plot(lsx,lsy,color='b')
    matplotlib.pyplot.plot(msx,msy,color='g')
    matplotlib.pyplot.plot(hsx,hsy,color='m')
    plot.set_xlabel("State # in path")
    plot.set_ylabel("Deformation")
    plot.legend(("Deformation free","Moderate deformation", "Highest deformation"), "upper right", shadow=False, fancybox=True)
    # Add the title
    plot.set_title("Measured deformation")
    # Show it
    matplotlib.pyplot.show()

def DrawCombinedRatioFigure(lowest_path, middle_path, highest_path):
    # Make a new figure
    figure = matplotlib.pyplot.figure()
    plot = figure.add_subplot(111)
    #Draw data
    [lsx,lsy] = ExtractXYFromData(lowest_path, "ratio_average")
    [msx,msy] = ExtractXYFromData(middle_path, "ratio_average")
    [hsx,hsy] = ExtractXYFromData(highest_path, "ratio_average")
    matplotlib.pyplot.plot(lsx,lsy,color='b')
    matplotlib.pyplot.plot(msx,msy,color='g')
    matplotlib.pyplot.plot(hsx,hsy,color='m')
    plot.legend(("Deformation free","Moderate deformation", "Highest deformation"), "upper right", shadow=False, fancybox=True)
    # Add the title
    plot.set_xlabel("State # in path")
    plot.set_title("Measured/computed deformation ratio")
    # Show it
    matplotlib.pyplot.show()

def ComputeCalibration(path_data, avg_type="mean"):
    non_zeros = []
    for state in path_data:
        if (state["ratio_average"] != 0.0):
            non_zeros.append(state["ratio_average"])
    if (avg_type == "median"):
        non_zeros.sort()
        middle_index = int(math.floor(len(non_zeros) / 2.0))
        if (len(non_zeros) > 0):
            return non_zeros[middle_index]
        else:
            return 1.0
    else:
        summed = 0.0
        for element in non_zeros:
            summed += element
        if (len(non_zeros) > 0):
            return summed / float(len(non_zeros))
        else:
            return 1.0

if __name__ == '__main__':
    #Find all the relevant sets of files in the bin directory
    path = subprocess.check_output("rospack find deformable_astar", shell=True)
    path = path.strip("\n") + "/data"
    #middle_path = path + "/deformation_middle_accurate3.csv"
    #shortest_path = path + "/deformation_worst_accurate3.csv"
    #DrawFigure(middle_path)
    #DrawFigure(shortest_path)
    #First pass
    lowest_path = DrawAverageFigure(path + "/deformation_lowest_accurate", 6, 1, "Deformation free path, uncalibrated")
    middle_path = DrawAverageFigure(path + "/deformation_middle_accurate", 6, 1, "Moderate deformation path, uncalibrated")
    highest_path = DrawAverageFigure(path + "/deformation_worst_accurate", 6, 1, "High deformation path, uncalibrated")
    #Find calibrations
    cal_lowest = ComputeCalibration(lowest_path, "median")
    print "cal_lowest [median] : " + str(cal_lowest)
    cal_middle = ComputeCalibration(middle_path, "median")
    print "cal_middle [median] : " + str(cal_middle)
    cal_highest = ComputeCalibration(highest_path, "median")
    print "cal_highest [median] : " + str(cal_highest)
    mcal_lowest = ComputeCalibration(lowest_path, "mean")
    print "cal_lowest [mean] : " + str(mcal_lowest)
    mcal_middle = ComputeCalibration(middle_path, "mean")
    print "cal_middle [mean] : " + str(mcal_middle)
    mcal_highest = ComputeCalibration(highest_path, "mean")
    print "cal_highest [mean] : " + str(mcal_highest)
    #Second pass
    mcal_val = (mcal_middle + mcal_highest) / 2.0
    print mcal_val
    #cal_lowest_path = DrawAverageFigure(path + "/deformation_lowest_accurate", 6, cal_lowest, "Deformation free path, calibrated")
    cal_middle_path = DrawAverageFigure(path + "/deformation_middle_accurate", 6, mcal_val, "Moderate deformation path, calibrated")
    cal_highest_path = DrawAverageFigure(path + "/deformation_worst_accurate", 6, mcal_val, "High deformation path, calibrated")
    #DrawCombinedCostFigure(cal_lowest_path, cal_middle_path, cal_highest_path)
    #DrawCombinedRatioFigure(cal_lowest_path, cal_middle_path, cal_highest_path)
    #print dir(middle_path)
    #avg_dict = {}
    #avg_dict["computed_deform"] = 0.0
    #avg_dict["computed_deform_sum"] = 0.0
    #avg_dict["measured_forward"] = 0.0
    #avg_dict["measured_forward_sum"] = 0.0
    #avg_dict["measured_backward"] = 0.0
    #avg_dict["measured_backward_sum"] = 0.0
    #avg_dict["average_measured"] = 0.0
    #avg_dict["average_measured_sum"] = 0.0

    mpcr = 0.0
    for path in middle_path:
        mpcr += path["computed_deform_sum"]
    mpcr = mpcr / float(len(middle_path))
    mpcm = 0.0
    for path in middle_path:
        mpcm += path["average_measured_sum"]
    mpcm = mpcm / float(len(middle_path))
    mpcc = mpcr * mcal_val

    hpcr = 0.0
    for path in highest_path:
        hpcr += path["computed_deform_sum"]
    hpcr = hpcr / float(len(highest_path))
    hpcm = 0.0
    for path in highest_path:
        hpcm += path["average_measured_sum"]
    hpcm = hpcm / float(len(highest_path))
    hpcc = hpcr * mcal_val

    print "Totals:"
    print "Middle planned cost raw: " + str(mpcr)
    print "Middle planned cost calibrated: " + str(mpcc)
    print "Middle planned cost measured: " + str(mpcm)
    print "Highest planned cost raw: " + str(hpcr)
    print "Highest planned cost calibrated: " + str(hpcc)
    print "Highest planned cost measured: " + str(hpcm)
