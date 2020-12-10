// RobWork plugin for the practical part of EiT.
// NB: The path to the workcell is hardcorded

// export RW_ROOT=~/Git/RobWork/RobWork/; export RWS_ROOT=~/Git/RobWork/RobWorkStudio/; export RWHW_ROOT=~/Git/RobWork/RobWorkHardware/; export RWSIM_ROOT=~/Git/RobWork/RobWorkSim/


#include "SamplePlugin.hpp"

// #include "GantrySimulation.h"

#include <RobWorkStudio.hpp>
#include <rw/rw.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>

//#include <rwlibs/opengl/RenderImage.hpp>
//#include <rwlibs/simulation/GLFrameGrabber.hpp>
//#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
//#include <rw/trajectory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>


#include <boost/bind.hpp>

#include "URSimulation.h"

#include "AbsolutePaths.h"

using rw::kinematics::State;
using rw::models::WorkCell;
using rws::RobWorkStudioPlugin;


SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_calculateTraj    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_executeTraj    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_getUR    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_getGantry    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
   // connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
    log().info() << "INITALIZE" << "\n";
    std::cout << "INITALIZE" << std::endl;

    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, boost::arg<1>()), this);

    // Auto load workcell
    //rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/soeren/Git/SDU/EiT/Workcell/Scene.wc.xml");
    //rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/soeren/Git/SDU/EiT/Workcell/wsg50/wsg50.xml");
    //if (wc.isNull()) {
    //    std::cout << "WorkCell could not be loaded." << std::endl;
    //}
    //getRobWorkStudio()->setWorkCell(wc);

}

// This function is called every time setWorkCell(wc) is called
void SamplePlugin::open(rw::models::WorkCell *workcell) {


    //log().info() << "OPEN UR by default" << "\n";
    // Save the empty workcell for easy reset
    //_emptyWc = workcell;
    // getUR(workcell); // This send the code to and endless loop
}

void SamplePlugin::close() {
}
void SamplePlugin::runPath(bool setFinalPos) {
        _wc  = getRobWorkStudio()->getWorkCell();
        _state = _wc->getDefaultState();
        _state_default = _state;
        _robot  = _wc->findDevice<rw::models::SerialDevice>("Stompa");

        // Create intermediate poses
        rw::math::Q intermediate1(8, 0, 0, 0, 0, 0, 0, 0, 0);
        rw::math::Q intermediate2(8, 1, 1, 0, 0, 0, 0, 0, 0.1);
        rw::math::Q intermediate3(8, 0.5, 0.5, 0, 3.1, 0, 0, 0.5, 0.3);
        rw::math::Q intermediate4(8, -0.5, 1.5, 0, 1.4, 0.3, 0, -0.5, -0.3);

        std::vector<rw::math::Q> qs;

        // Push intermediate poses to qs vector
        qs.push_back(intermediate1);
        qs.push_back(intermediate2);
        qs.push_back(intermediate3);
        qs.push_back(intermediate4);

        // Calculate the full path
        createPTPPath(qs,0.01);

        // Record the path to a playback file - apply final position based on button pressed
        executePath(0.01, setFinalPos);
}

void SamplePlugin::invCalc() {
        _wc  = getRobWorkStudio()->getWorkCell();
        _state = _wc->getDefaultState();
        _state_default = _state;
        _robot  = _wc->findDevice<rw::models::SerialDevice>("Stompa");
        rw::invkin::JacobianIKSolver test(_robot, _state);

        // Eigen::VectorXd we(6);
        // we[0] = 1.0;
        // we[1] = 1.0;
        // we[2] = 1.0;
        // we[3] = 1.0;
        // we[4] = 0.0;
        // we[5] = 1.0;
        // test.setWeightVector(we);

        rw::math::Q output;
        std::vector<rw::math::Q> result;
        rw::math::Transform3D<> pose(rw::math::Vector3D<>(0,0,0),rw::math::RPY<>(1.57079632679,0,1.57079632679));
        result = test.solve(pose, _state);
        std::cout << "Der er: " << result.size() << " resultater." << std::endl;
        _robot->setQ(result[0],_state);
        getRobWorkStudio()->setState(_state);

}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if (obj == _calculateTraj) {
        log().info() << "Recording Trajectory." << "\n";
        std::cout << "Recording Trajectory." << std::endl;
        runPath(false);
    }
    else if (obj == _executeTraj) {
        log().info() << "Recording Trajectory and apply final pos." << "\n";
        std::cout << "Recording Trajectory and apply final pos." << std::endl;
        //runPath(true);
        invCalc();
        
    }
    else if (obj == _reset){}
    else if (obj == _getUR){
        log().info() << "Getting the UR workcell:" << "\n";
        std::cout << "Get UR" << std::endl;
        getUR();
    }
    else if (obj == _getGantry){
        log().info() << "Getting the Gantry workcell:" << "\n";
        std::cout << "Get Gantry" << std::endl;
        getGantry();
    }

}

void SamplePlugin::getUR(){
        // Load the workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(URFilePath);
    if (wc.isNull()) {
        std::cout << "WorkCell could not be loaded." << std::endl;
    }
    getRobWorkStudio()->setWorkCell(wc);

    _wc  = getRobWorkStudio()->getWorkCell();
    _state = _wc->getDefaultState();
    _state_default = _state;

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        rw::kinematics::Frame *textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        rw::kinematics::Frame *bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage", _bgRender, bgFrame);
        }

        _URReference = _wc->findFrame<rw::kinematics::MovableFrame>("URReference");
        _graspTarget = _wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");

        _robot = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
        if (_robot.isNull()) {
            std::cout << "Robot could not be found." << std::endl;
        } else {
            std::cout << "Robot found." << std::endl;
        }

        _gripper = _wc->findDevice<rw::models::TreeDevice>("WSG50");
        if (_gripper.isNull()) {
            std::cout << "Gripper could not be found." << std::endl;
        } else {
            std::cout << "gripper found." << std::endl;
        }
    }

    _tcpCorrection = rw::kinematics::Kinematics::frameTframe(_wc->findFrame("UR-6-85-5-A.TCP"), _wc->findFrame("WSG50.TCP"), _state);
    _closedFormSolver = (new rw::invkin::ClosedFormIKSolverUR(_robot, _state));
    _detector = rw::common::ownedPtr( new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


    URSimulation URrobot;
    URrobot.initializeWorkcell();
}

void SamplePlugin::getGantry() {
    std::cout << "Get Gantry" << std::endl;
    // Load the workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(GantryFilePath);
    //rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/soeren/Git/SDU/EiT/Workcell/Scene.wc.xml");
    if (wc.isNull()) {
        std::cout << "WorkCell could not be loaded." << std::endl;
    }
    getRobWorkStudio()->setWorkCell(wc);
}



// ##################################### Robitics ###############################


void SamplePlugin::stateChangedListener(const State& state) {

}

bool
SamplePlugin::checkCollisions(rw::models::Device::Ptr device, const State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
    State testState;
    rw::proximity::CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q, testState);
    colFrom = detector.inCollision(testState, &data);
    if (colFrom) {
//        cerr << "Configuration in collision: " << q << endl;
//        cerr << "Colliding frames: " << endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
        }
        return false;
    }
    return true;
}

rw::math::Q SamplePlugin::getQ(rw::math::Transform3D<double> target, rw::math::Q &qclose) {

    std::vector<rw::math::Q> solutions = _closedFormSolver->solve(target, _state);
    std::cout<<"Solution size"<<solutions.size()<<std::endl;

    int bestI = -1;
    double bestDist = std::numeric_limits<double>::max();
    for (unsigned i = 0; i < solutions.size(); i++) {
        _robot->setQ(solutions[i], _state);
        if (checkCollisions(_robot, _state, *_detector, solutions[i])) {//collsiion checking
            if (qclose.size() < 3) {
                std::cout<<"small q as closest q in getQ"<<std::endl;
                return solutions[i];
            }
            double dist = (qclose - solutions[i]).norm2();
            if (dist < bestDist) {
                bestDist = dist;
                bestI = i;
            }
        }
    }
    if (bestI != -1){
        return solutions[bestI];
    }

    std::cout<<"No solutions found"<<std::endl;

    return rw::math::Q::zero(6);
}

rw::math::Q SamplePlugin::getQ(rw::kinematics::Frame *frame, rw::math::Q &qclose){
    return getQ(getPlanningTransform3D(frame),qclose);
}

rw::math::Transform3D<double> SamplePlugin::getPlanningTransform3D(rw::kinematics::Frame *frame) {
    return rw::kinematics::Kinematics::frameTframe(_URReference, frame, _state) * inverse(_tcpCorrection);
}

void SamplePlugin::createPTPPath(std::vector<rw::math::Q> qs, double dt) {
    rw::trajectory::InterpolatorTrajectory<rw::math::Q>::Ptr trajectoryPTP = (new rw::trajectory::InterpolatorTrajectory<rw::math::Q>());
    for (unsigned i = 1; i < qs.size(); i++) {
        std::cout << "Loaded trajectory and planning object with limits: " << _robot->getVelocityLimits()
                  << _robot->getAccelerationLimits() << std::endl;
        rw::trajectory::RampInterpolator<rw::math::Q>::Ptr line =(
                new rw::trajectory::RampInterpolator<rw::math::Q>(qs[i - 1],
                qs[i],
                _robot->getVelocityLimits(),
                _robot->getAccelerationLimits())
                );

        trajectoryPTP->add(line);
    }
    _path.clear();
    std::vector<rw::math::Q> qvec = trajectoryPTP->getPath(dt);
    _path.assign(qvec.begin(), qvec.end());
}

void SamplePlugin::executePath(double dt, bool setFinal) {
    std::cout << "Executing PTP path with " << _path.size() << " steps in " << dt * _path.size() << " seconds"
              << std::endl;
    // Map the configurations to a sequence of states.
    const std::vector<State> states = rw::models::Models::getStatePath(*_robot , _path, _state);

    // Write the sequence of states to a file.
    rw::loaders::PathLoader::storeVelocityTimedStatePath(
        *_wc, states, PlaybackFilePath+"plannedPath.rwplay");

    // Set the final state of the robot.
    if (setFinal){
        _robot->setQ(_path[_path.size()-1],_state);
        getRobWorkStudio()->setState(_state);
    }
    std::cout << "Done " << std::endl;
    
}


//    else if(obj==_btn_getQ){
//        log().info() << "Button 0\n";
//
////        rw::kinematics::MovableFrame* _graspTargetEntry = _wc->findFrame<rw::kinematics::MovableFrame>("GraspTargetEntry");
////        if (_graspTargetEntry == NULL) {
////            std::cout<<"grapTargetEntry Not Found"<<std::endl;
////        }
//
//        rw::math::Q intermediate1(6, -0.325155, -1.745, 1.775, -0.0090059, 2.546, -3.1);// Q[6]{-0.325155, -1.745, 1.775, -0.0090059, 2.546, 0}
//        rw::math::Q intermediate2(6, 2.03575, -1.61949, 1.90066, -0.122522, 2.1033, -3.1);//     Q[6]{2.03575, -1.61949, 1.90066, -0.122522, 2.1033, 0}
//
//        std::vector<rw::math::Q> qs;
//        qs.push_back(intermediate1);
//        qs.push_back(intermediate2);
//
//
//        _robot->setQ(intermediate1,_state);
//        getRobWorkStudio()->setState(_state);
//        rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(_detector, _robot, _state);
//
//        rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_robot),
//                                                          constraint.getQConstraintPtr());
//        rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
////        rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner 	( 	const rw::pathplanning::PlannerConstraint &  	constraint,
////                                                                   rw::core::Ptr< rw::pathplanning::QSampler >  	sampler,
////                                                                   rw::math::QMetric::Ptr  	metric,
////                                                                   double  	extend,
////                                                                   rwlibs::pathplanners::RRTPlanner::RRTBidirectional);
////
//
//
//
//
//
////        auto pathPlanner = (new rw::pathplanning::PathPlanner<rw::math::Q, rw::math::Q>);
////
////        //rw::pathplanning::PathPlanner<rw::math::Q, rw::math::Q> pathPlanner;
////
////        pathPlanner.query(intermediate1, intermediate2, _path);
//
//
//
//
//
//        //rw::trajectory::QPath>(intermediate1,intermediate2, _path);
//
//
////        qs.push_back(getQ(_graspTarget,intermediate1));
//
//
////
////
////        // executing plan
////        createPTPPath(qs,0.01);
////        executePath(0.01);
////        std::cout << "Here" << std::endl;
////        while(_timer->isActive()){
////            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
////        }
//
//    }
