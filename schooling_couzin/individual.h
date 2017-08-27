//
//  individual.h
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 14/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef individual_h
#define individual_h
#include <iostream>
#include "vector2D.h"

class individual
{
public:
    individual(void);
    ~individual(void);
    
    void Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
               const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_perception,
               const double& set_angular_error_sd);
    
    void Move(const double& timestep_inc, const double& arena_size, double dev_angle);
    
    CVec2D r_centre;            // Rotational centre of the agent (cm)
    CVec2D direction;           // vector representing the current direction of the agent
    CVec2D desired_direction;
    double speed;               // cm per second
    double max_turning_rate;    // maximum turning rate (degrees per second)
    double zone_of_deflection;
    double zone_of_perception;
    double angular_error_sd;
    
    int zop_count;
    int zod_count;
    int equivalence_class;
    CVec2D total_zod;
    CVec2D total_zoo;
    CVec2D total_zop;
    
    void TurnTowardsVector(CVec2D& vector, const double& timestep_inc, const double& dev_angle);
    void MoveMyself(const double& timestep_inc, const double& arena_size);
    void Copy(const individual& other);
};

#endif /* individual_h */
