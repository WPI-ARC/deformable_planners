#include <ros/ros.h>
#include <resource_retriever/retriever.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include "deformable_astar/IrosPlanner.h"
#include "deformable_astar/DeformPlanner.h"
#include "deformable_astar/PathState.h"

#define _USE_MATH_DEFINES
#define CIRCLEDIST(x,y) ((fabs(x - y))<=M_PI?(x - y):(M_PI - (x - y)))
#define CLOSE(x,y) ((fabs(x - y))<=0.0001?(1):(0))
#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

//Ros Publisher of the Information
ros::Publisher pub;

//Global model variables
dVoxelArray * g_environment;
dVoxelArray * g_robot;

//Publish an environment to RVIZ
void PublishDVXL(dVoxelArray * input, uint8_t threshold, float scale)
{
    //Create a new Marker
    visualization_msgs::Marker marker;
    //ID and timestamp
    marker.header.frame_id = "/environment_link";
    marker.header.stamp = ros::Time::now();
    //Give it a unique ID
    marker.ns = "DVXL_Visualization";
    marker.id = 0;
    //Give the marker a type, in this case a SPHERE
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    //We're going to add the marker to rviz
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.a = 0.5;
    //Set how long the marker should be around for
    marker.lifetime = ros::Duration(10.0);
    //For each element of the 3D grid
    for (int i = 0; i < input->xDim; i++)
    {
        for (int j = 0; j < input->yDim; j++)
        {
            for (int k = 0; k < input->zDim; k++)
            {
                /**
                  As it turns out, apparently you can set the color of markers in a markerlist independently
                  but you can't set the alpha (transparency) independently. Interesting
                  */
                if (input->dVoxels[i][j][k].A > threshold)
                {
                    geometry_msgs::Point p;
                    std_msgs::ColorRGBA c;
                    p.x = (float)i * scale;
                    p.y = (float)j * scale;
                    p.z = (float)k * scale;
                    c.r = (float)input->dVoxels[i][j][k].R / 255.0;
                    c.g = (float)input->dVoxels[i][j][k].G / 255.0;
                    c.b = (float)input->dVoxels[i][j][k].B / 255.0;
                    c.a = (float)input->dVoxels[i][j][k].A / 255.0;
                    c.a = 0.5;
                    marker.points.push_back(p);
                    marker.colors.push_back(c);
                }
            }
        }
    }
    //Publish the marker and hope for the best!
    pub.publish(marker);
}

float DistanceHeuristicHybrid3D(state6D * stateToEval, state6D * goalState, int debug_level)
{
    float deltaX = goalState->XT - stateToEval->XT;
    float deltaY = goalState->YT - stateToEval->YT;
    float deltatheta = goalState->ZR - stateToEval->ZR;
    float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltatheta, 2));
    debug(debug_level, 2, "Assigned heuristic value: %f\n", distance);
    return distance;
}

StateList * GenerateChildrenHybrid3D(state6D * state, int debug_level)
{
    debug(debug_level, 2, "Generating children...\n");
    if (state == NULL)
    {
        debug(debug_level, 1, "\033[91m *** Passed a null state, unable to generate children! *** \033[0m\n");
        fflush(stdout);
        return AllocateStateList();
    }
    StateList * children = AllocateStateList();
    float Xmax = (float)state->Environment->xDim;
    float Ymax = (float)state->Environment->yDim;
    //float Zmax = (float)state->Environment->zDim;
    float k = state->ZT;
    for (float i = (state->XT - 1.0); i <= (state->XT + 1.0); i+=1.0)
    {
        for (float j = (state->YT - 1.0); j <= (state->YT + 1.0); j+=1.0)
        {
            for (float kr = (state->ZR - M_PI_4); kr <= (state->ZR + M_PI_4); kr+=1.0)
            {
                //Check if we're within the acceptable bounds of the environment
                if (i >= 0.0 && j >= 0.0 && i < Xmax && j < Ymax)
                {
                    //Check to make sure we're not duplicating the parent
                    if (i != state->XT || j != state->YT || kr != state->ZR)
                    {
                        float xdiff = i - state->XT;
                        float ydiff = j - state->YT;
                        float zrdiff = CIRCLEDIST(kr, state->ZR);
                        if(CLOSE(kr, state->ZR) == 1)
                        {
                            zrdiff = 0.0;
                            kr = state->ZR;
                        }
                        float diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zrdiff, 2));
                        state6D * childState = NewState6D(state, state->RefShape, state->Environment, i, j, k, 0.0, 0.0, kr, diff);
                        StateAppend(children, childState);
                    }
                }
            }
        }
    }
    debug(debug_level, 1, "Generated %d possible new children\n", children->Length);
    return children;
}

StateList * GenerateChildren2D(state6D * state, int debug_level)
{
    debug(debug_level, 2, "Generating children...\n");
    if (state == NULL)
    {
        debug(debug_level, 1, "\033[91m *** Passed a null state, unable to generate children! *** \033[0m\n");
        fflush(stdout);
        return AllocateStateList();
    }
    StateList * children = AllocateStateList();
    float Xmax = (float)state->Environment->xDim;
    float Ymax = (float)state->Environment->yDim;
    //float Zmax = (float)state->Environment->zDim;
    float k = state->ZT;
    for (float i = (state->XT - 1.0); i <= (state->XT + 1.0); i+=1.0)
    {
        for (float j = (state->YT - 1.0); j <= (state->YT + 1.0); j+=1.0)
        {
            //Check if we're within the acceptable bounds of the environment
            if (i >= 0.0 && j >= 0.0 && i < Xmax && j < Ymax)
            {
                //Check to make sure we're not duplicating the parent
                if (i != state->XT || j != state->YT)
                {
                    float xdiff = i - state->XT;
                    float ydiff = j - state->YT;
                    float diff = sqrt(pow(xdiff, 2) + pow(ydiff, 2));
                    state6D * childState = NewState6D(state, state->RefShape, state->Environment, i, j, k, 0.0, 0.0, 0.0, diff);
                    StateAppend(children, childState);
                }
            }
        }
    }
    debug(debug_level, 1, "Generated %d possible new children\n", children->Length);
    return children;
}


StateList * RunAstar3D(state6D * startState, state6D * goalState, float pareto_ctrl)
{
    float ParetoWeight = pareto_ctrl;
    int debug_level = 1;
    StateList * path = AstarSearch(startState, goalState, ParetoWeight, DistanceHeuristicHybrid3D, GenerateChildren2D, debug_level);
    return path;
}

int ExportPathToCSV(StateList * path, char * filename)
{
    FILE *path_file = fopen(filename, "w");
    if (path_file == NULL)
    {
        return -1;
    }
    fprintf(path_file, "XT,YT,ZT,XR,YR,ZR,COST,DIST");
    for (int index = 0; index < path->Length; index++)
    {
        state6D * PN = GetIndex(path, index);
        fprintf(path_file, "%f,%f,%f,%f,%f,%f,%f,%f\n", PN->XT, PN->YT, PN->ZT, PN->XR, PN->YR, PN->ZR, PN->COST, PN->DIST);
    }
    int result = fclose(path_file);
    return result;
}

int ExportPathToDVXL(StateList * path, char * filename)
{
    state6D * startState = GetIndex(path, 0);
    state6D * goalState = GetIndex(path, path->Length - 1);
    dVoxelArray * savecopy = CloneDVoxelArray(startState->Environment);
    if (savecopy == NULL)
    {
        return -1;
    }
    //Show the swept volume
    //for (int index = 0; index < path->Length; index+=2)
    //{
    //    state6D * PN = GetIndex(path, index);
    //    TransformMergeDVoxelArrays(savecopy, PN->RefShape, PN->XT, PN->YT, PN->ZT, 0, 0, 0, 100);
    //}
    state6D * SN = GetIndex(path, 0);
    TransformMergeDVoxelArrays(savecopy, SN->RefShape, SN->XT, SN->YT, SN->ZT, 0, 0, 0, 100);
    state6D * GN = GetIndex(path, (path->Length - 1));
    TransformMergeDVoxelArrays(savecopy, Recolor(GN->RefShape, 0, 255, 0, 100, 1), GN->XT, GN->YT, GN->ZT, 0, 0, 0, 100);
    for (int index = 0; index < path->Length; index++)
    {
        state6D * PN = GetIndex(path, index);
        int tx = (int)PN->XT;
        int ty = (int)PN->YT;
        int tz = (int)PN->ZT;
        savecopy->dVoxels[tx][ty][tz].R = 0;
        savecopy->dVoxels[tx][ty][tz].G = 0;
        savecopy->dVoxels[tx][ty][tz].B = 0;
        savecopy->dVoxels[tx][ty][tz].A = 255;
    }
    //Draw in start and goal states
    int sx = startState->XT;
    int sy = startState->YT;
    int sz = startState->ZT;
    int gx = goalState->XT;
    int gy = goalState->YT;
    int gz = goalState->ZT;
    savecopy->dVoxels[sx][sy][sz].R = 255;
    savecopy->dVoxels[sx][sy][sz].G = 0;
    savecopy->dVoxels[sx][sy][sz].B = 255;
    savecopy->dVoxels[sx][sy][sz].A = 255;
    savecopy->dVoxels[gx][gy][gz].R = 0;
    savecopy->dVoxels[gx][gy][gz].G = 255;
    savecopy->dVoxels[gx][gy][gz].B = 0;
    savecopy->dVoxels[gx][gy][gz].A = 255;
    int result = WriteDVoxelArrayToFile(savecopy, filename);
    return result;
}

state6D ** Astar3DSetup(dVoxelArray * environment, dVoxelArray * robot, float * start, float * goal)
{
    //Make start and goal states
    state6D * startState = NewState6D(NULL, robot, environment, start[0], start[1], start[2], start[3], start[4], start[5], 0.0);
    state6D * goalState = NewState6D(NULL, robot, environment, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], 0.0);
    state6D ** states = (state6D**)malloc(sizeof(state6D*) * 2);
    //Check them for feasibility
    states[0] = EvaluateState(startState, goalState, 0.5, IrosDistanceHeuristic3D, 1);
    states[1] = EvaluateState(goalState, goalState, 0.5, IrosDistanceHeuristic3D, 1);
    return states;
}

void DrawStartAndGoal(float * start, float * goal, bool solved)
{
    //Create a new Marker
    visualization_msgs::Marker marker;
    //ID and timestamp
    marker.header.frame_id = "/environment_link";
    marker.header.stamp = ros::Time::now();
    //Give it a unique ID
    marker.ns = "Path_Visualization";
    marker.id = 0;
    //Give the marker a type, in this case a SPHERE
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    //We're going to add the marker to rviz
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.a = 0.5;
    //Set how long the marker should be around for
    marker.lifetime = ros::Duration(100.0);
    //For each element of the 3D grid
    geometry_msgs::Point p1;
    std_msgs::ColorRGBA c1;
    p1.x = start[0] * 0.005;
    p1.y = start[1] * 0.005;
    p1.z = start[2] * 0.005;
    c1.r = 1.0;
    c1.a = 0.5;
    marker.points.push_back(p1);
    marker.colors.push_back(c1);
    geometry_msgs::Point p2;
    std_msgs::ColorRGBA c2;
    p2.x = goal[0] * 0.005;
    p2.y = goal[1] * 0.005;
    p2.z = goal[2] * 0.005;
    c2.r = 1.0;
    c2.g = 1.0;
    if (solved)
    {
        c2.r = 0.0;
    }
    c2.a = 0.5;
    marker.points.push_back(p2);
    marker.colors.push_back(c2);
    //Publish the marker and hope for the best!
    pub.publish(marker);
}

StateList * RunSearch(std::vector<float> start, std::vector<float> goal, int export_ctrl, float pareto_ctrl)
{
    float * startA = start.data();
    float * goalA = goal.data();
    DrawStartAndGoal(startA, goalA, false);
    state6D** base_states = Astar3DSetup(g_environment, g_robot, startA, goalA);
    if (base_states[0] == NULL)
    {
        ROS_ERROR("Invalid start state");
    }
    if (base_states[1] == NULL)
    {
        ROS_ERROR("Invalid goal state");
    }
    if (base_states[0] == NULL || base_states[1] == NULL)
    {
        return NULL;
    }
    StateList * planned_path = RunAstar3D(base_states[0], base_states[1], pareto_ctrl);
    if (planned_path == NULL)
    {
        ROS_ERROR("Planner unable to find a solution");
        return NULL;
    }
    DrawStartAndGoal(startA, goalA, true);
    ROS_INFO("Planner found a solution with %d steps", planned_path->Length);
    if (export_ctrl == 1 || export_ctrl == 3)
    {
        ExportPathToCSV(planned_path, (char*)"/home/calderpg/Dropbox/Research/planner3d/astar_deform/data/planned_path.csv");
        ROS_INFO("Exported path to CSV");
    }
    if (export_ctrl == 2 || export_ctrl == 3)
    {
        ExportPathToDVXL(planned_path, (char*)"/home/calderpg/Dropbox/Research/planner3d/astar_deform/data/planned_path3d.dvxl");
        ROS_INFO("Exported path to DVXL");
    }
    return planned_path;
}

std::vector<deformable_astar::PathState> ExportPlannedPath(StateList * planned_path, float scale)
{
    std::vector<deformable_astar::PathState> path;
    if (planned_path == NULL)
    {
        return path;
    }
    for (int index = 0; index < planned_path->Length; index++)
    {
        state6D * PN = GetIndex(planned_path, index);
        deformable_astar::PathState ps;
        ps.pose.position.x = PN->XT * scale;
        ps.pose.position.x = PN->YT * scale;
        ps.pose.position.x = PN->ZT * scale;
        ps.properties.deformation_cost = PN->COST;
        ps.properties.distance_cost = PN->DIST;
        path.push_back(ps);
    }
    return path;
}

bool Plan(deformable_astar::DeformPlanner::Request  &req, deformable_astar::DeformPlanner::Response &res)
{
  ROS_INFO("Received a planning request...");
  std::vector<float> req_start(3);
  req_start[0] = req.start.pose.position.x;
  req_start[1] = req.start.pose.position.y;
  req_start[2] = req.start.pose.position.z;
  std::vector<float> req_goal(3);
  req_goal[0] = req.goal.pose.position.x;
  req_goal[1] = req.goal.pose.position.y;
  req_goal[2] = req.goal.pose.position.z;
  StateList * planned_path = RunSearch(req_start, req_goal, req.ctrl, req.pareto_ctrl);
  res.path = ExportPlannedPath(planned_path, 0.005);
  ROS_INFO("Planning completed, returning result");
  return true;
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "plannernode");
    ros::NodeHandle nh;
    // Create a ROS publisher that can be used to send out the output
    pub = nh.advertise<visualization_msgs::Marker>("Planning_Markers", 1);
    // Get the environment and robot files
    std::string basepath = "/home/calderpg/Dropbox/Research/planner3d/astar_deform/data/";
    std::string env_filename;
    std::string robot_filename;
    ros::param::get("plannernode/env_filepath", env_filename);
    ros::param::get("plannernode/robot_filepath", robot_filename);
    // Try to load the files
    std::string full_env_filename = basepath + env_filename;
    std::string full_robot_filename = basepath + robot_filename;
    g_environment = LoadDVoxelArrayFromFile((char *)full_env_filename.c_str());
    g_robot = LoadDVoxelArrayFromFile((char *)full_robot_filename.c_str());
    if (g_robot == NULL)
    {
        ROS_FATAL("UNABLE TO LOAD ROBOT");
    }
    if (g_environment == NULL)
    {
        ROS_FATAL("UNABLE TO LOAD ENVIRONMENT");
    }
    if (g_robot == NULL || g_environment == NULL)
    {
        exit(1);
    }
    ROS_INFO("Loaded environment and robot");
    //Register the service callback
    ros::ServiceServer service = nh.advertiseService("astar_deform/PlannerQuery", Plan);
    //Spin and display the environment
    ros::Rate spin_rate(0.1);
    while (ros::ok())
    {
        PublishDVXL(g_environment, 50, 0.005);
        ros::spinOnce();
        spin_rate.sleep();
    }
    return 0;
}

