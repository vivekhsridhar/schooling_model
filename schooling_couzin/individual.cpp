//
//  individual.cpp
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 15/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include "individual.h"
#include <limits.h>
#include <float.h>
#include <algorithm>

individual::individual(void) : max_turning_rate(0.0), speed(0.0), zone_of_deflection(0.0), zone_of_perception(0.0), angular_error_sd(0.0), zod_count(0.0), zop_count(0.0)
{
}

individual::~individual(void)
{
}

void individual::Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
                       const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_perception,
                       const double& set_angular_error_sd)
{
    r_centre = set_r_centre;	direction = set_direction; 	max_turning_rate = set_max_turning_rate;	speed = set_speed;
    zone_of_deflection = set_zone_of_deflection;    zone_of_perception = set_zone_of_perception;
    angular_error_sd = set_angular_error_sd;
}

void individual::Move(const double& timestep_inc, const double& arena_size, double dev_angle)	// so we have direction (current direction) and a desired direction
{
    if(fabs(desired_direction.x) < FLT_EPSILON && fabs(desired_direction.y) < FLT_EPSILON)
    {
        TurnTowardsVector(direction, timestep_inc, dev_angle);
    }
    else
    {
        TurnTowardsVector(desired_direction, timestep_inc, dev_angle);
    }
    
    MoveMyself(timestep_inc, arena_size);
}

void individual::MoveMyself(const double& timestep_inc, const double& arena_size)
{
    CVec2D velocity = direction;
    velocity *= (speed * timestep_inc);
    r_centre += velocity;
    
    if (r_centre.x >= arena_size) r_centre.x -= arena_size;
    else if (r_centre.y >= arena_size) r_centre.y -= arena_size;
    
    if (r_centre.x < 0.0) r_centre.x += arena_size;
    else if (r_centre.y < 0.0) r_centre.y += arena_size;
}

void individual::TurnTowardsVector(CVec2D& vector, const double& timestep_inc, const double& dev_angle)
{
    // Nice algorithm
    double max_degrees = max_turning_rate * timestep_inc;
    
    vector.rotate(dev_angle);
    
    double check_angle = direction.smallestAngleTo(vector);
    
    if(check_angle <= max_degrees)
    {
        direction = vector;
    }
    else if(check_angle > max_degrees)  // should be able to go from the cross produce to the dot product
    {
        double cross_product = direction.cross(vector);
        
        if(cross_product > 0) direction.rotate(max_degrees);
        else direction.rotate(-max_degrees);
    }
}

void individual::Copy(const individual &other)
{
    r_centre = other.r_centre;	direction = other.direction; 	max_turning_rate = other.max_turning_rate;	speed = other.speed;	zone_of_deflection = other.zone_of_deflection;  zone_of_perception = other.zone_of_perception;  angular_error_sd = other.angular_error_sd;
}

