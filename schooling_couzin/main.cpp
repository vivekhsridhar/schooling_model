//
//  main.cpp
//  schooling_couzin
//
//  Created by Vivek Hari Sridhar on 03/01/17.
//  Copyright © 2017 Vivek Hari Sridhar. All rights reserved.
//

#include <cmath>
#include <limits.h>
#include <cfloat>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "parameteres.h"

using namespace std;
using namespace rnd;
using namespace cv;

int main()
{
    // Random generator engine from a time-based seed
    unsigned seed = static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    rnd::set_seed(seed);
    
    // Set parameters
    timestep_inc = 0.1;
    arena_size = 300;
    top_left.x = 0.0;
    top_left.y = 0.0;
    bottom_right.x = arena_size;
    bottom_right.y = arena_size;
    
    total_agents = 60;
    angular_error_sd = 0.02;
    max_turning_rate = 114.591559;
    zod = 1.0;
    zop = 36.0;
    speed = 1.0;
    wp = 0.2;
    wo = 0.4;
    wa = 0.4;
    
    agent = new individual[total_agents];
    
    int num_timesteps = 2000000;
    
    // Setup simulation
    Size S(static_cast<int>(arena_size), static_cast<int>(arena_size));
    
    SetupSimulation();
            
    // Time loop
    for(int j = 0; j != num_timesteps; ++j)
    {
        MoveAgents();
                
        if (j % 50 == 0) Graphics();
                
        ++timestep_number;
    }
    
    echo("simulation end");
    return 0;
}

void MoveAgents()
{
    CalculateSocialForces();
    
    for(int i = 0; i != total_agents; ++i)
    {
        // now each fish has a unit vector which is its desired direction of travel in the next timestep
        double dev_angle = 360.0f * normal(0.0, angular_error_sd);
        agent[i].Move(timestep_inc, arena_size, dev_angle);
    }
}

void CalculateSocialForces()
{
    double dist;
    CVec2D temp_vector;
    
    double zop_length = agent[0].zone_of_perception;
    double zod_length = agent[0].zone_of_deflection;
    
    for(int clear = 0; clear != total_agents; ++clear)
    {
        agent[clear].zod_count = 0;
        agent[clear].zop_count = 0;
        agent[clear].total_zod.Clear();
        agent[clear].total_zoo.Clear();
        agent[clear].total_zop.Clear();
    }
    
    for (int i = 0; i != total_agents; ++i)
    {
        for (int j = (i+1); j != total_agents; ++j)
        {
            temp_vector = (agent[j].r_centre - agent[i].r_centre);
            
            if (std::abs(temp_vector.x) > arena_size / 2)
            {
                if (agent[i].r_centre.x < agent[j].r_centre.x) temp_vector.x = agent[j].r_centre.x - (agent[i].r_centre.x + arena_size);
                else temp_vector.x = agent[j].r_centre.x - (agent[i].r_centre.x - arena_size);
            }
            if (std::abs(temp_vector.y) > arena_size / 2)
            {
                if (agent[i].r_centre.y < agent[j].r_centre.y) temp_vector.y = agent[j].r_centre.y - (agent[i].r_centre.y + arena_size);
                else temp_vector.y = agent[j].r_centre.y - (agent[i].r_centre.y - arena_size);
            }
            
            // check to see if it is reasonable that you may be interacting
            if (temp_vector.x * temp_vector.x > zop_length || temp_vector.y * temp_vector.y > zop_length)
            {
                // cannot be close enough to interact with
            }
            else
            {
                dist = temp_vector.length() * temp_vector.length();	// fastest way to check distance
                
                if(dist < zod_length)	// this has highest priority
                {
                    agent[i].zod_count++;
                    agent[j].zod_count++;
                    
                    // there's a problem with the initialisation of the fish - they can all start at 0.0, 0.0 and thus the below function return 0.0, 0.0 which then messes up the normalisation function
                    temp_vector = temp_vector.normalise();
                    
                    agent[i].total_zod+= (-temp_vector);
                    agent[j].total_zod+= (temp_vector);
                }
                
                else if(dist < zop_length)
                {
                    agent[i].zop_count++;
                    agent[j].zop_count++;
                    
                    temp_vector = temp_vector.normalise();
                    
                    agent[i].total_zop += temp_vector;
                    agent[i].total_zoo += agent[j].direction;
                    agent[j].total_zop += (-temp_vector);
                    agent[j].total_zoo += agent[i].direction;
                }
            }
        }
    }
    
    // now have total_zod and total_zop calculated for all individuals
    for(int k = 0; k != total_agents; ++k)
    {
        if(agent[k].zod_count > 0)
        {
            if(fabs(agent[k].total_zod.x) < FLT_EPSILON && fabs(agent[k].total_zod.y) < FLT_EPSILON)	// if it has completely cancelled out
            {
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = agent[k].total_zod.normalise();
            }
        }
        else if(agent[k].zop_count > 0)
        {
            if(fabs(agent[k].total_zoo.x + agent[k].total_zop.x) < FLT_EPSILON && fabs(agent[k].total_zoo.y + agent[k].total_zop.y) < FLT_EPSILON)	// if no zoo fish were found or if they cancelled each other out
            {
                // just do a correlated random walk
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = agent[k].direction * wp + agent[k].total_zoo.normalise() * wo + agent[k].total_zop.normalise() * wa;
            }
        }
        
        agent[k].desired_direction = agent[k].desired_direction.normalise();
    }
}

void SetupSimulation()
{
    timestep_number = 0;	// timestep number
    SetupAgents();
}

void SetupAgents()
{
    CVec2D set_r_centre;
    
    for(int i = 0; i != total_agents; ++i)
    {
        CVec2D set_direction(1.0, 0.0);	// need to be set to unit vectors
        set_direction.rotate(uniform() * 360.0);
        set_r_centre = RandomBoundedPoint();
        
        agent[i].Setup(set_r_centre, set_direction, max_turning_rate, speed, zod, zop, angular_error_sd);
    }
}

CVec2D RandomBoundedPoint()
{
    // Create randomly distributed co-ordinate in the centre of world space
    double range_x = (double) (bottom_right.x - top_left.x);
    double range_y = (double) (bottom_right.y - top_left.y);
    
    double random_x = uniform();
    double random_y = uniform();
    
    // Individuals start in the middle 20th of the tank
    random_x *= (range_x / 100.0f);
    random_y *= (range_y / 100.0f);
    random_x += (double) (range_x / 2.0f - range_x / 200.0f);
    random_y += (double) (range_y / 2.0f - range_y / 200.0f);
    CVec2D random_point(random_x, random_y);
    return random_point;
}

void Graphics()
{
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size, CV_8UC3);
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 1, Scalar(204, 255, 153), -1, CV_AA);
    }
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    imshow("decision_making", visualisation);
    waitKey(1);
}


