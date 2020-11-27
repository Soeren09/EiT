//
// Created by soeren on 11/11/20.
//

#ifndef PLUGINUIAPP_GANTRYSIMULATION_H
#define PLUGINUIAPP_GANTRYSIMULATION_H

#include "RobotSimulation.h"

// RobWork includes
#include <rw/rw.hpp>
#include <rwlibs/opengl/RenderImage.hpp>


class GantrySimulation : RobotSimulation{

    GantrySimulation();
    ~GantrySimulation();

    void initializeWorkcell();
    void calculateTrajectory(rw::kinematics::State from, rw::kinematics::State to); //calls uninstantiated calculate_robot_trajectory()
    void executeTrajectory();

};


#endif //PLUGINUIAPP_GANTRYSIMULATION_H
