#include <ros/ros.h>
#include <ros/package.h>
#include <resource_retriever/retriever.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include "deformable_astar/dVoxelLib.h"

//Ros Publisher of the Information
ros::Publisher pub;

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
    marker.color.a = 1.0;
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
//                if (input->dVoxels[i][j][k].R == 255 && input->dVoxels[i][j][k].G == 0 && input->dVoxels[i][j][k].B == 255)
//                {
//                    printf("Found the start state\n");
//                }
//                if (input->dVoxels[i][j][k].R == 0 && input->dVoxels[i][j][k].G == 255 && input->dVoxels[i][j][k].B == 0)
//                {
//                    printf("Found the goal state\n");
//                }
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
                    c.a = 1.0;
                    marker.points.push_back(p);
                    marker.colors.push_back(c);
                }
            }
        }
    }

	//Publish the marker and hope for the best!
	pub.publish(marker);
}



int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "visualizernode");
    ros::NodeHandle nh;
    // Create a ROS publisher that can be used to send out the output 
	pub = nh.advertise<visualization_msgs::Marker>("Marker_Cloud", 1);
    // Get the filename
    std::string basepath = ros::package::getPath("deformable_astar");
    if (basepath.empty())
    {
        ROS_ERROR("Cannot find package!");
        exit(1);
    }
    basepath = basepath + std::string("/data/");
    std::string filename;
    ros::param::get("visualizernode/filepath", filename);
    printf("Node loaded, got filename: %s\n", filename.c_str());
    std::string full_filename = basepath + filename;
    // Read in an environment
    printf("Full filepath: %s\n", full_filename.c_str());
    fflush(stdout);
    dVoxelArray * new_array = LoadDVoxelArrayFromFile((char *)full_filename.c_str());
    printf("File loaded\n");
    fflush(stdout);
    if (new_array == NULL)
    {
        printf("*** Could not load file! ***\n");
        return 1;
    }
    printf("Loaded new DVXL file into memory with dimensions %d|%d|%d\n", new_array->xDim, new_array->yDim, new_array->zDim);
    ros::Rate spin_rate(0.1);
    int iteration = 0;
    while (ros::ok())
    {
        if (iteration % 4 == 0)
        {
            DestroyDVoxelArray(new_array);
            new_array = LoadDVoxelArrayFromFile((char *)full_filename.c_str());
            printf("Reloaded file\n");
        }
        PublishDVXL(new_array, 50, 0.005);
        //TestMarkerCloud();
        ros::spinOnce();
        spin_rate.sleep();
        iteration++;
    }
    printf("Done\n");
    return 0;
}

