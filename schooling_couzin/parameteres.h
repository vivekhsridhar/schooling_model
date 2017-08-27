//
//  parameteres.h
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 17/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef parameteres_h
#define parameteres_h

#include "individual.h"
#include <fstream>

int     timestep_number;    // timestep number
double   timestep_inc;       // time increment (between timesteps)
CVec2D  bottom_right;
CVec2D  top_left;
int     arena_size;
int     total_agents;

double	angular_error_sd;
double	max_turning_rate;
double	zod;            // zone of deflection
double	zop;            // zone of perception
double	speed;
double  wp;
double  wo;
double  wa;

individual* agent;

int main();
void SetupSimulation();
void SetupAgents();
void CalculateSocialForces();
void MoveAgents();
CVec2D RandomBoundedPoint();

void Graphics();

#endif /* parameteres_h */
