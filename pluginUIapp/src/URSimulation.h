//
// Created by soeren on 11/11/20.
//

#ifndef PLUGINUIAPP_URSIMULATION_H
#define PLUGINUIAPP_URSIMULATION_H

#include "RobotSimulation.h"

#include <iostream>
// RobWork includes
//#include <rw/rw.hpp>
//#include <rwlibs/opengl/RenderImage.hpp>



class URSimulation : public RobotSimulation {

public:
   // URSimulation() : RobotSimulation() {};
   // ~URSimulation();

    void initializeWorkcell() {
       std::cout << "Here" << std::endl;
   }


//    void calculateTrajectory(rw::kinematics::State from, rw::kinematics::State to); //calls uninstantiated calculate_robot_trajectory()
//    void executeTrajectory();

private:

};


#endif //PLUGINUIAPP_URSIMULATION_H
