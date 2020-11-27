//
// Created by soeren on 11/11/20.
//

#ifndef PLUGINUIAPP_ROBOTSIMULATION_H
#define PLUGINUIAPP_ROBOTSIMULATION_H


#include <iostream>

// RobWork includes
#include <rw/rw.hpp>
#include <rwlibs/opengl/RenderImage.hpp>

// Base class
class RobotSimulation {
public:

      virtual void initializeWorkcell() = 0;

//    RobotSimulation()
//    {
//        std::cout << "Base" << std::endl;
//    }
//    ~RobotSimulation();

 //   virtual void calculateTrajectory(rw::kinematics::State from, rw::kinematics::State to); //calls uninstantiated calculate_robot_trajectory()
//    virtual void executeTrajectory();


//    std::vector<SimulationState> calculateTrajectory(RobotPostition from, RobotPostition to, std::vector<RebarCoordinates> rebarCoordinates) {
//        std::vector<RobotState> robotTrajectory = self.calculateRobotTrajectory(from, to);
//
//        std::vector<Dispenser::DispenserState> dispenserTrajectory = dispenser.calculateTrajectory(robotTrajectory, rebarCoordinates);
//    }
//    virtual std::vector<RobotState> calculateRobotTrajectory(RobotPosition from, RobotPosition to);


//    rw::kinematics::State _state;
//    rw::models::WorkCell _wc;

  //  rw::models::SerialDevice::Ptr _robot;
  //  rw::models::SerialDevice::Ptr _dispenser;



private:
//    Dispenser dispenser;
};

#endif //PLUGINUIAPP_ROBOTSIMULATION_H
