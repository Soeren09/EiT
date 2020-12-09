#include "SamplePlugin.hpp"
//#include "Homography.h"

//#include "ModelObject.hpp"
//#include "PCLHelerFunction.h"
//#include "global_pose_estimater.h"
//#include "local_pose_estimater.h"


#include <random>
#include <ctime>
//OPENCV
//#include <opencv2/core.hpp>
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"

// RobWork
#include <rw/rw.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rw/trajectory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <iterator>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

//using namespace cv;

rw::math::Statistics<double> S;


SamplePlugin::SamplePlugin() :
        RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png")) {
    setupUi(this);

    //_timer = new QTimer(this);
    //connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

//    // now connect stuff from the ui component
//    connect(_btn_img, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_scan, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_pe_m4, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_pe_m2, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_test_m2, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_show_m2, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_test_m4, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_test_m4_2, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_suzanne, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_show_m4, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_rrt, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_ptp, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_rrt_test, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_ptp_test, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_save_q_traj, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_impl_m2, SIGNAL(pressed()), this, SLOT(btnPress()));
//    connect(_btn_impl_m4, SIGNAL(pressed()), this, SLOT(btnPress()));
//
//
//
//    connect(_btn_exec_trajectory, SIGNAL(pressed()), this, SLOT(btnPress()));

//    _framegrabber = NULL;
//    _cameras = {"Camera_Right"};
//    _cameras25D = {"Scanner25D"};
}

void SamplePlugin::initialize() {
//    log().info() << "INITALIZE" << "\n";
//
//    getRobWorkStudio()->stateChangedEvent().add(
//            std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);
//
//    // Auto load workcell
//    //rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../../../workcell/SceneTexture.wc.xml");
//    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/soeren/Git/SDU/EiT/Workcell/Scene.wc.xml");
//
//
//    getRobWorkStudio()->setWorkCell(wc);

}

void SamplePlugin::open(rw::models::WorkCell *workcell) {
//    log().info() << "OPEN" << "\n";
//    _wc = workcell;
//    _state = _wc->getDefaultState();
//    _state_default = _state;
//    log().info() << workcell->getFilename() << "\n";
//
//    if (_wc != NULL) {
//        // Add the texture render to this workcell if there is a frame for texture
//        rw::kinematics::Frame *textureFrame = _wc->findFrame("MarkerTexture");
//        if (textureFrame != NULL) {
//            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage", _textureRender, textureFrame);
//        }
//        // Add the background render to this workcell if there is a frame for texture
//        rw::kinematics::Frame *bgFrame = _wc->findFrame("Background");
//        if (bgFrame != NULL) {
//            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage", _bgRender, bgFrame);
//        }
//
////        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
////        rw::kinematics::Frame *cameraFrame = _wc->findFrame(_cameras[0]);
////        if (cameraFrame != NULL) {
////            if (cameraFrame->getPropertyMap().has("Camera")) {
////                // Read the dimensions and field of view
////                double fovy;
////                int width, height;
////                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
////                std::istringstream iss(camParam, std::istringstream::in);
////                iss >> fovy >> width >> height;
////                // Create a frame grabber
////                _framegrabber = new rwlibs::simulation::GLFrameGrabber(width, height, fovy);
////
////                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
////                _framegrabber->init(gldrawer);
////            }
////        }
//
////        rw::kinematics::Frame *cameraFrame25D = _wc->findFrame(_cameras25D[0]);
////        if (cameraFrame25D != NULL) {
////            if (cameraFrame25D->getPropertyMap().has("Scanner25D")) {
////                // Read the dimensions and field of view
////                double fovy;
////                int width, height;
////                std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
////                std::istringstream iss(camParam, std::istringstream::in);
////                iss >> fovy >> width >> height;
////                // Create a frame grabber
////                _framegrabber25D = new rwlibs::simulation::GLFrameGrabber25D(width, height, fovy);
////                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
////                _framegrabber25D->init(gldrawer);
////            }
////        }
//
//        _URReference = _wc->findFrame<MovableFrame>("URReference");
//        _graspTarget = _wc->findFrame<MovableFrame>("GraspTarget");
//
//
//        _placeTarget = _wc->findFrame<MovableFrame>("PlaceTarget");
//
//
//        _robot = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
//
//        _gripper = _wc->findDevice<rw::models::TreeDevice>("WSG50");
//        _step = -1;
//    }
//
//    _tcpCorrection = Kinematics::frameTframe(_wc->findFrame("UR-6-85-5-A.TCP"), _wc->findFrame("WSG50.TCP"), _state);
//    _closedFormSolver = (new rw::invkin::ClosedFormIKSolverUR(_robot, _state));
//    _detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc,
//                                                                          rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
//
//
//    // -------------- Homography initialization ------------------------------
////    double bestScore = 0;
////    cv::Mat img_scene = _images[0];
////    std::string image_path;
////    double width_mm, height_mm;
////
////    for (auto frame : _wc->getFrames()) {
////        if (frame->getPropertyMap().has("homography")) {
////            std::cout << "Found homography : " << frame->getName() << std::endl;
////            std::string homographyParam = frame->getPropertyMap().get<std::string>("homography");
////            std::istringstream iss(homographyParam, std::istringstream::in);
////            iss >> image_path >> width_mm >> height_mm;
////
////            Homography H;
////            H.frame = frame;
////            H.initialize(image_path, width_mm, height_mm);
////
////            _homography.push_back(H);
////        }
////    }
}

void SamplePlugin::close() {
//    log().info() << "CLOSE" << "\n";
//
//    // Stop the timer
//    _timer->stop();
//    // Remove the texture render
//    rw::kinematics::Frame *textureFrame = _wc->findFrame("MarkerTexture");
//    if (textureFrame != NULL) {
//        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage", textureFrame);
//    }
//    // Remove the background render
//    rw::kinematics::Frame *bgFrame = _wc->findFrame("Background");
//    if (bgFrame != NULL) {
//        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage", bgFrame);
//    }
////    // Delete the old framegrabber
////    if (_framegrabber != NULL) {
////        delete _framegrabber;
////    }
////    _framegrabber = NULL;
//    _wc = NULL;
}

//void SamplePlugin::timer() {
////    if (0 <= _step && _step < _path.size()) {
////        _robot->setQ(_path.at(_step), _state);
////        getRobWorkStudio()->setState(_state);
////        _step++;
////    } else if (_step == _path.size()) {
////        _timer->stop();
////        _step = 0;
////    }
//}

void SamplePlugin::btnPress() {
    QObject *obj = sender();
//    if (obj == _btn_img) {
////        getImage();
////        updateImage(_images[0]);
//    } else if (obj == _btn_scan) {
////        get25DImage();
//    } else if (obj == _btn_pe_m4) {
//        log().info() << "Pose estimating using homography\n";
//        double noise = (double) (_slider1->value());
//        bool makeNoise = noise > 1;
////        poseEstimationM4(makeNoise, 0.0, noise);
//    } else if (obj == _btn_pe_m2) {
//        log().info() << "Pose estimating using dense stereo\n";
////        poseEstimationM2();
//    } else if (obj == _btn_test_m2) {
//        log().info() << "Testing poseestimation using RANSAC\n";
////        poseEstimationM2Test();
//    } else if (obj == _btn_show_m2) {
//        log().info() << "Showing latest homography match\n";
////        PoseEstimationM2LocalTest();
//    } else if (obj == _btn_test_m4) {
//        log().info() << "Testing poseestimation using homography\n";
////        poseEstimationM4Test();
//    } else if (obj == _btn_test_m4_2) {
//        log().info() << "Testing poseestimation using homography\n";
////        poseEstimationM4Test2();
//    } else if (obj == _btn_show_m4) {
//        log().info() << "Showing latest homography match\n";
////        showPoseEstimationM4();
//    } else if (obj == _btn_suzanne) {
//        _modelName = "suzanne";
//    } else if (obj == _btn_rrt) {
//        log().info() << "RRT path planning\n";
//
////        Q qf(getQ(_graspTarget,rw::math::Q::zero(1)));
////        Q qt(getQ(_placeTarget,rw::math::Q::zero(1)));
////        std::cout << "RRT from Q : " << qf << std::endl;
////        std::cout << "RRT from Q : " << qt << std::endl;
////        createPathRRTConnect(qf, qt, 0.1, 30);
////        if (_path.size() < 2) { std::cout << "RRT could find a plan" << std::endl; }
//
//    } else if (obj == _btn_rrt_test) {
//        rrtTest();
//    } else if (obj == _btn_ptp_test) {
//        ptpTest();
//    } else if (obj == _btn_ptp) {
//        QPath qs;
//        qs.push_back(Q(6, 0.358142, -2.14571, 2.72847, -3.7648, -2.03217, 0));
//        qs.push_back(Q(6, -1.1094, -0.930034, 2.27401, -4.48557, -2.03219, -0));
//        qs.push_back(Q(6, -1.10942, -1.51509, 2.13831, -3.7648, -2.03217, -0));
//        qs.push_back(Q(6, 0.358142, -2.14571, 2.72847, -3.7648, -2.03217, 0));
//        qs.push_back(Q(6, 2.29179, -2.14571, 2.72847, -3.7648, -2.03217, 0));
//        qs.push_back(Q(6, 2.149, -0.939, 2.133, -4.339, -2.242, 0));
//        qs.push_back(Q(6, 0.358142, -2.14571, 2.72847, -3.7648, -2.03217, 0));
//
//        /*      std::cout<<"PTP from Q : "<<qpath[0]<<std::endl;
//              std::cout<<"PTP from Q : "<<qpath[-1]<<std::endl;
//              std::vector<Q> qs = rw::trajectory::TrajectoryFactory::makeLinearTrajectory(qpath,*_robot)->getPath(DT,false);
//              _path.clear();
//              _path.assign(qs.begin(),qs.end());
//              _path_type = PTP;*/
//        createPTPPath(qs, DT);
//
//
//    } else if (obj == _btn_exec_trajectory) {
//        executePath(DT);
//    } else if (obj == _save_q_traj) {
//        std::cout << "Saving trajectory" << std::endl;
//        ofstream q_pos;
//        q_pos.open("q_pos.csv");
//        for (int i = 0; i < _path.size(); i++) {
//            std::vector<double> q = _path.at(i).toStdVector();
//            q_pos << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << "\n";
//        }
//        q_pos.close();
//    } else if (obj == _btn_impl_m2) {
//        implementation(M2);
//    } else if (obj == _btn_impl_m4) {
//        implementation(M4);
//
//    }
}


//cv::Mat SamplePlugin::toOpenCVImage(const rw::sensor::Image &img) {
//    Mat res(img.getHeight(), img.getWidth(), CV_8SC3);
//    res.data = (uchar *) img.getImageData();
//    return res;
//}

//void SamplePlugin::getImage() {
//    if (_framegrabber != NULL) {
//        for (int i = 0; i < _cameras.size(); i++) {
//            // Get the image as a RW image
//            rw::kinematics::Frame *cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
//            _framegrabber->grab(cameraFrame, _state);
//
//            const rw::sensor::Image *rw_image = &(_framegrabber->getImage());
//
//            // Convert to OpenCV matrix.
//            cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3,
//                                    (rw::sensor::Image *) rw_image->getImageData());
//
//            // Convert to OpenCV image
//            Mat imflip, imflip_mat;
//            cv::flip(image, imflip, 1);
//            cv::cvtColor(imflip, imflip_mat, COLOR_RGB2BGR);
//
//            _images[i] = imflip_mat.clone();
//
//        }
//    }
//}

//void SamplePlugin::get25DImage() {
//
//    if (_framegrabber25D != NULL) {
//        for (int i = 0; i < _cameras25D.size(); i++) {
//            // Get the image as a RW image
//            rw::kinematics::Frame *cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
//            _framegrabber25D->grab(cameraFrame25D, _state);
//
//            const rw::geometry::PointCloud *img = &(_framegrabber25D->getImage());
//            pclCloud::Ptr temp(new pclCloud(img->getWidth(), img->getHeight()));
//
//            std::cout << "Point cloud ";
//            std::cout << "FIELDS x y z\n";
//            std::cout << "# .PCD v.5 - Point Cloud Data file format\n";
//            std::cout << "SIZE 4 4 4\n";
//            std::cout << "TYPE F F F\n";
//            std::cout << "WIDTH " << img->getWidth() << "\n";
//            std::cout << "HEIGHT " << img->getHeight() << "\n";
//            std::cout << "POINTS " << img->getData().size() << "\n";
//            std::cout << "DATA ascii\n" << std::endl;
//
//
//            for (int i = 0; i < img->getData().size(); i++) {
//                rw::math::Vector3D<float> p = img->getData()[i];
//                temp->points[i].x = p(0);
//                temp->points[i].y = p(1);
//                temp->points[i].z = p(2);
//            }
//
//            // Remove the bakground
//            PassThroughFilter(temp, temp, "z", -1.18, 0);
//            _scan = temp;
//        }
//    }
//}

//void SamplePlugin::updateImage(cv::Mat &imflip_BRG) {
//    cv::Mat imflip;
//    cv::cvtColor(imflip_BRG, imflip, COLOR_BGR2RGB);
//
//    // Show in QLabel
//    QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
//    QPixmap p = QPixmap::fromImage(img);
//
//    _label->setPixmap(p.scaled(480, 640, Qt::KeepAspectRatio));
//}

//void SamplePlugin::stateChangedListener(const rw::kinematics::State &state) {
//    _state = state;
//}

SamplePlugin::~SamplePlugin() {
//    delete _textureRender;
//    delete _bgRender;
}



//void SamplePlugin::implementation(int peType) {
//
//    _state = _state_default;
//
//
//    std::cout<<"Showcase of implementation"<<std::endl;
//    rw::kinematics::MovableFrame* _graspTargetEntry = _wc->findFrame<MovableFrame>("GraspTargetEntry");
//    rw::kinematics::MovableFrame* _graspTargetExit = _wc->findFrame<MovableFrame>("GraspTargetExit");
//    rw::kinematics::MovableFrame* obj;
//
//
//    if (_graspTargetEntry == NULL) {
//        std::cout<<"grapTargetEntry Not Found"<<std::endl;
//    }
//    if (_graspTargetExit == NULL) {
//        std::cout<<"_graspTargetExit Not Found"<<std::endl;
//    }
//
//    Q intermediate1(6, -0.325155, -1.745, 1.775, -0.0090059, 2.546, -3.1);// Q[6]{-0.325155, -1.745, 1.775, -0.0090059, 2.546, 0}
//    Q intermediate2(6, 2.03575, -1.61949, 1.90066, -0.122522, 2.1033, -3.1);//     Q[6]{2.03575, -1.61949, 1.90066, -0.122522, 2.1033, 0}
//
//    vector<Q> qs;
//    std::cout<<"Showcase of implementation"<<std::endl;
//    _robot->setQ(intermediate1,_state);
//    getRobWorkStudio()->setState(_state);
//
//
//    if (peType == M2) {
//        obj = _wc->findFrame<MovableFrame>("suzanne");
//        _modelName = "suzanne";
//        get25DImage();
//        poseEstimationM2();
//    } else {
//        obj = _wc->findFrame<MovableFrame>("milk");
//        poseEstimationM4(true);
//    }
//    Transform3D<double> graspTargetwTf = _graspTarget->wTf(_state);
//    Transform3D<double> newPlaceTarget(Vector3D<double>(0.3,-0.5,graspTargetwTf.P()[2]),RPY<double>(180*Deg2Rad,0,0).toRotation3D()*graspTargetwTf.R());
//    _placeTarget->setTransform(newPlaceTarget,_state);
//    qs.push_back(intermediate1);
//    qs.push_back(getQ(_graspTargetEntry,intermediate1));
//    qs.push_back(getQ(_graspTarget,qs.back()));
//
//    // executing plan
//    createPTPPath(qs,0.01);
//    executePath(0.01);
//    while(_timer->isActive()){
//        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
//    }
//
//    // gripping milk
//    Kinematics::gripFrame(obj,_gripper->getBase(),_state);
//
//
//    Q lastq = qs.back();
//    qs.clear();
//    qs.push_back(getQ(_graspTarget,lastq));
//    qs.push_back(getQ(_graspTargetExit,qs.back()));
//    qs.push_back(intermediate1);
//    qs.push_back(intermediate2);
//    qs.push_back(getQ(_placeTarget,qs.back()));
//
//
//    // executing plan
//    createPTPPath(qs,0.01);
//    executePath(0.01);
//    while(_timer->isActive()){
//        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
//    }
//
//    Kinematics::gripFrame(obj,_wc->getWorldFrame(),_state);
//
//}


//bool
//SamplePlugin::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
//    State testState;
//    CollisionDetector::QueryResult data;
//    bool colFrom;
//
//    testState = state;
//    device->setQ(q, testState);
//    colFrom = detector.inCollision(testState, &data);
//    if (colFrom) {
////        cerr << "Configuration in collision: " << q << endl;
////        cerr << "Colliding frames: " << endl;
//        FramePairSet fps = data.collidingFrames;
//        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
//            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
//        }
//        return false;
//    }
//    return true;
//}

//rw::math::Q SamplePlugin::getQ(rw::math::Transform3D<double> target, rw::math::Q &qclose) {
//
//    std::vector<rw::math::Q> solutions = _closedFormSolver->solve(target, _state);
//    std::cout<<"Solution size"<<solutions.size()<<std::endl;
//
//    int bestI = -1;
//    double bestDist = std::numeric_limits<double>::max();
//    for (int i = 0; i < solutions.size(); i++) {
//        _robot->setQ(solutions[i], _state);
//        if (checkCollisions(_robot, _state, *_detector, solutions[i])) {//collsiion checking
//            if (qclose.size() < 3) {
//                std::cout<<"small q as closest q in getQ"<<std::endl;
//                return solutions[i];
//            }
//            double dist = (qclose - solutions[i]).norm2();
//            if (dist < bestDist) {
//                bestDist = dist;
//                bestI = i;
//            }
//        }
//    }
//    if (bestI != -1){
//        return solutions[bestI];
//    }
//
//    std::cout<<"No solutions found"<<std::endl;
//
//    return Q::zero(6);
//
//}

//rw::math::Q SamplePlugin::getQ(rw::kinematics::Frame *frame, rw::math::Q &qclose){
//    return getQ(getPlanningTransform3D(frame),qclose);
//}

//rw::math::Transform3D<double> SamplePlugin::getPlanningTransform3D(rw::kinematics::Frame *frame) {
//    return rw::kinematics::Kinematics::frameTframe(_URReference, frame, _state) * inverse(_tcpCorrection);
//}

//void SamplePlugin::createPathRRTConnect(Q from, Q to, double extend, double maxTime) {
//    _robot->setQ(from, _state);
//    getRobWorkStudio()->setState(_state);
//    PlannerConstraint constraint = PlannerConstraint::make(_detector, _robot, _state);
//    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_robot),
//                                                      constraint.getQConstraintPtr());
//    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
//    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend,
//                                                           RRTPlanner::RRTBidirectional);
//
//    _path.clear();
//    if (!checkCollisions(_robot, _state, *_detector, from))
//        cout << from << " is in colission!" << endl;
//    if (!checkCollisions(_robot, _state, *_detector, to))
//        cout << to << " is in colission!" << endl;;
//    Timer t;
//    t.resetAndResume();
//    planner->query(from, to, _path, maxTime);
//    t.pause();
//
//
//    if (t.getTime() >= maxTime) {
//        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
//    }
//
//    const int duration = 10;
//
//    if (_path.size() ==
//        2) {  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
//        LinearInterpolator<Q> linInt(from, to, duration);
//        QPath tempQ;
//        for (int i = 0; i < duration + 1; i++) {
//            tempQ.push_back(linInt.x(i));
//        }
//
//        _path = tempQ;
//    }
//
//    _path_type = RRT;
//}

//void SamplePlugin::createPTPPath(std::vector<rw::math::Q> qs, double dt) {
//
//    rw::trajectory::InterpolatorTrajectory<rw::math::Q>::Ptr trajectoryPTP = (new rw::trajectory::InterpolatorTrajectory<rw::math::Q>());
//    for (int i = 1; i < qs.size(); i++) {
//        std::cout << "Loaded trajectory and planning object with limits: " << _robot->getVelocityLimits()
//                  << _robot->getAccelerationLimits() << std::endl;
//        RampInterpolator<Q>::Ptr line = (new RampInterpolator<Q>(qs[i - 1], qs[i], _robot->getVelocityLimits(),
//                                                                 _robot->getAccelerationLimits()));
//        trajectoryPTP->add(line);
//    }
//    _path.clear();
//    std::vector<Q> qvec = trajectoryPTP->getPath(dt);
//    _path.assign(qvec.begin(), qvec.end());
//    _path_type = PTP;
//}

//void SamplePlugin::executePath(double dt) {
//    if (_timer->isActive()) {
//        _timer->stop();
//        std::cout << "Path execution stopped" << std::endl;
//        _state = _wc->getDefaultState();
//    } else if (_path_type == RRT) {
//        std::cout << "Executing RRT path with " << _path.size() << " steps in " << 10000.0 / _path.size()
//                  << " seconds"
//                  << std::endl;
//        _step = 0;
//        _timer->start(10000.0 / _path.size());// The Path will execute in 10 seconds
//    } else if (_path_type == PTP) {
//        std::cout << "Executing PTP path with " << _path.size() << " steps in " << dt * _path.size() << " seconds"
//                  << std::endl;
//        _step = 0;
//        _timer->start(dt * 1000.0);
//    } else {
//        std::cout << "No path type have been set" << std::endl;
//    }
//}

//void SamplePlugin::rrtTest() {
//
//    Q place(6, -0.757106, -1.84117, -2.04441, -2.39761, -0.757106, 0);
//    std::vector<Q> pick_qs;
//    pick_qs.push_back(Q(6, 2.06788, -1.83841, -2.0652, -2.38332, -2.64451, -0.00502655));
//    pick_qs.push_back(Q(6, 2.96702, -1.74742, -2.18665, -2.35937, -0.174568, 0.00834267));
//    pick_qs.push_back(Q(6, 2.67173, -2.38353, -1.15928, -2.75177, 1.10104, 0.0120079));
//
//
//    rw::math::EuclideanMetric<Q>::Ptr euclideanMetric = (new rw::math::EuclideanMetric<Q>());
//    for (int p = 0; p < 3; p++) {
//        std::cout << "rrt_test" << std::endl;
//        ofstream csv_file;
//        csv_file.open("rrt_p" + std::to_string(p) + "_test.csv");
//        rw::pathplanning::PathAnalyzer pathAnalyzer(_robot, _state);
//        for (double eps = 0.2; eps <= 4; eps += 0.2) {
//            std::vector<double> path_size;
//            std::vector<double> path_length;
//            std::vector<double> path_length_tcp;
//            std::vector<double> execution_time;
//
//            for (int s = 0; s < 3; s++) {
//                std::clock_t c_start = std::clock();
//                createPathRRTConnect(pick_qs[0], place, eps, 60.0);
//                std::clock_t c_end = std::clock();
//                double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
//
//                rw::pathplanning::PathAnalyzer::JointSpaceAnalysis jointSpaceAnalysis = pathAnalyzer.analyzeJointSpace(
//                        _path, euclideanMetric);
//                rw::pathplanning::PathAnalyzer::CartesianAnalysis cartesianAnalysis = pathAnalyzer.analyzeCartesian(
//                        _path,
//                        _wc->findFrame(
//                                "WSG50.TCP"));
//                path_size.push_back(jointSpaceAnalysis.nodecount);
//                path_length.push_back(jointSpaceAnalysis.length);
//                path_length_tcp.push_back(cartesianAnalysis.length);
//                execution_time.push_back(time_elapsed_ms / 1000.0);
//            }
//
//            std::pair<double, double> path_sizeMV = S.meanAndVariance(path_size);
//            std::pair<double, double> path_lengthMV = S.meanAndVariance(path_length);
//            std::pair<double, double> path_length_tcpMV = S.meanAndVariance(path_length_tcp);
//            std::pair<double, double> execution_timeMV = S.meanAndVariance(execution_time);
//
//            csv_file << eps << " " << path_sizeMV.first << " " << path_sizeMV.second << " " << path_lengthMV.first
//                     << " "
//                     << path_lengthMV.second << " " << path_length_tcpMV.first << " " << path_length_tcpMV.second
//                     << execution_timeMV.first << " " << execution_timeMV.second << "\n";
//            std::cout << eps << " " << path_sizeMV.first << " " << path_sizeMV.second << " " << path_lengthMV.first
//                      << " "
//                      << path_lengthMV.second << " " << path_length_tcpMV.first << " " << path_length_tcpMV.second
//                      << " "
//                      << execution_timeMV.first << " " << execution_timeMV.second << "\n";
//        }
//
//        csv_file.close();
//    }
//
//}

//void SamplePlugin::ptpTest() {
//    Q place(6, 1.83886, -1.31259, 2.06125, -0.748659, 1.83886,
//            -0);//Q[6]{1.83886, -1.31259, 2.06125, -0.748659, 1.83886, -0}
//    Q place_entry(6, 1.99071, -1.63441, 2.34288, -0.708464, 1.99071,
//                  -0.0015708);//Q[6]{1.99071, -1.63441, 2.34288, -0.708464, 1.99071, -0.0015708}
//
//    std::vector<Q> pick_qs;
//    std::vector<Q> pick_entry_qs;
//
//    pick_qs.push_back(Q(6, 2.06788, -1.83841, -2.0652, -2.38332, -2.64451, -0.00502655));
//    pick_qs.push_back(Q(6, 2.96702, -1.74742, -2.18665, -2.35937, -0.174568, 0.00834267));
//    pick_qs.push_back(Q(6, 2.67173, -2.38353, -1.15928, -2.75177, 1.10104, 0.0120079));
//
//    pick_entry_qs.push_back(Q(6, 0, 0.475, 0.145, -1.5708, 0.00174533,
//                              1.57258));//Q[6]{0, 0.475, 0.145, -1.5708, 0.00174533, 1.57258}
//    pick_entry_qs.push_back(Q(6, -0.25, 0.245, 0.145, 3.14159, 0.00176278,
//                              1.57258));//Q[6]{-0.25, 0.245, 0.145, 3.14159, 0.00176278, 1.57258}
//    pick_entry_qs.push_back(Q(6, 2.72898, -2.57019, -0.614443, -3.10964, 1.15831,
//                              0.0112923));//Q[6]{2.72898, -2.57019, -0.614443, -3.10964, 1.15831, 0.0112923}
//
//    Q intermediate1(6, -0.25, 0.245, 0.42, 3.14159, 0.00176278,
//                    1.57258);//Q[6]{-0.25, 0.245, 0.42, 3.14159, 0.00176278, 1.57258}
//    Q intermediate2(6, 2.276, -1.745, 1.775, -0.009, 2.546, 0);//     Q[6]{2.276, -1.745, 1.775, -0.009, 2.546, 0}
//
//    _robot->setQ(intermediate1, _state);
//    rw::pathplanning::PathAnalyzer pathAnalyzer(_robot, _state);
//    rw::math::EuclideanMetric<Q>::Ptr euclideanMetric = (new rw::math::EuclideanMetric<Q>());
//
//
//    std::cout << "ptp_test" << std::endl;
//    ofstream csv_file;
//    csv_file.open("ptp_test.csv");
//    for (int i = 0; i < pick_qs.size(); i++) {
//        _robot->setQ(intermediate1, _state);
//        std::vector<Q> qpath;
//        qpath.push_back(intermediate1);
//        qpath.push_back(pick_entry_qs[i]);
//        qpath.push_back(pick_qs[i]);
//        qpath.push_back(intermediate1);
//        qpath.push_back(intermediate2);
//        qpath.push_back(place_entry);
//        qpath.push_back(place);
//
//        std::vector<double> path_size;
//        std::vector<double> path_length;
//        std::vector<double> path_length_tcp;
//        std::vector<double> execution_time;
//        for (int j = 0; j < 30; j++) {
//            std::clock_t c_start = std::clock();
//            createPTPPath(qpath, 0.025);
//            std::clock_t c_end = std::clock();
//            double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
//
//            rw::pathplanning::PathAnalyzer::JointSpaceAnalysis jointSpaceAnalysis = pathAnalyzer.analyzeJointSpace(
//                    _path, euclideanMetric);
//            rw::pathplanning::PathAnalyzer::CartesianAnalysis cartesianAnalysis = pathAnalyzer.analyzeCartesian(
//                    _path,
//                    _wc->findFrame(
//                            "WSG50.TCP"));
//            path_size.push_back(jointSpaceAnalysis.nodecount);
//            path_length.push_back(jointSpaceAnalysis.length);
//            path_length_tcp.push_back(cartesianAnalysis.length);
//            execution_time.push_back(time_elapsed_ms / 1000.0);
//
//        }
//        std::pair<double, double> path_sizeMV = S.meanAndVariance(path_size);
//        std::pair<double, double> path_lengthMV = S.meanAndVariance(path_length);
//        std::pair<double, double> path_length_tcpMV = S.meanAndVariance(path_length_tcp);
//        std::pair<double, double> execution_timeMV = S.meanAndVariance(execution_time);
//        csv_file << i << " " << path_sizeMV.first << " " << path_sizeMV.second << " " << path_lengthMV.first
//                 << " "
//                 << path_lengthMV.second << " " << path_length_tcpMV.first << " " << path_length_tcpMV.second << " "
//                 << execution_timeMV.first << execution_timeMV.second << "\n";
//        std::cout << i << " " << path_sizeMV.first << " " << path_sizeMV.second << " " << path_lengthMV.first
//                  << " "
//                  << path_lengthMV.second << " " << path_length_tcpMV.first << " " << path_length_tcpMV.second
//                  << " "
//                  << execution_timeMV.first << " " << execution_timeMV.second << "\n";
//    }
//    csv_file.close();
//
//}


//Transform3D<double> SamplePlugin::poseEstimationM4(bool addNoise, double mean, double std) {
//    getImage();
//
//    if (addNoise) {
//        AddGaussianNoise_Opencv(_images[0], _images[0], mean, std);
//        updateImage(_images[0]);
//    }
//
//    double bestScore = 0;
//
//
//    for (int i = 0; i < _homography.size(); i++) {
//        _homography[i].estimateHomography(_images[0]);
//        if (_homography[i].matchScore > bestScore) {
//            bestScore = _homography[i].matchScore;
//            _bestHidx = i;
//        }
//    }
//
//    // -------------------------------   Extrinsic estimation    -------------------------------
//
//    Eigen::Matrix<double, 3, 3> H = _homography[_bestHidx].homography_eigen;
//    Eigen::Matrix<double, 3, 4> KA = printProjectionMatrix(_cameras[0]);
//    Eigen::Matrix<double, 3, 3> KA_bar = KA.block<3, 3>(0, 0);
//
//    Eigen::Matrix<double, 3, 3> M = KA_bar.inverse() * H;
//
//    // Using svd to get proper transformation
//    Eigen::Matrix<double, 3, 2> M_bar = M.block<3, 2>(0, 0);
//    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> svd(M_bar, Eigen::ComputeThinU |
//                                                                                       Eigen::ComputeThinV);
//    Eigen::Matrix<double, 3, 2> R_bar = svd.matrixU() * (svd.matrixV().transpose());
//
//    double lambda = (R_bar.transpose() * M_bar).trace() / (M_bar.transpose() * M_bar).trace();
//
//    Eigen::Vector3d r3 = R_bar.col(0).cross(R_bar.col(1));
//
//    Eigen::Matrix<double, 4, 4> extrinsic;
//    Eigen::Matrix<double, 3, 3> R;
//
//    R.block<3, 2>(0, 0) = R_bar;
//    R.block<3, 1>(0, 2) = r3;
//
//    extrinsic.block<3, 3>(0, 0) = R.transpose();
//    extrinsic.block<3, 1>(0, 3) = M.col(2) * lambda * _homography[_bestHidx].scale;
//    extrinsic.block<1, 4>(3, 0) = Eigen::Vector4d(0, 0, 0, 1);
//
//
//    //----- moving the pose estimate frame appropriately------
//    MovableFrame *pose_estimation_frame = _wc->findFrame<MovableFrame>("pose_estimate");
//
//    //transformation from milk side/homography-image to the object frame
//    Transform3D<double> hToObj = rw::kinematics::Kinematics::frameTframe(_homography[_bestHidx].frame,
//                                                                         _wc->findFrame("milk"), _state);
//    Transform3D<double> pe(eig4x4ToTransform3D(extrinsic) * hToObj);//
//
//    pose_estimation_frame->setTransform(pe, _state);
//    getRobWorkStudio()->setState(_state);
//
//
//
//    // -- set grasp target to right side of milkcan
//    Transform3D<double> homogToGrasp(Vector3D<double>(0.035478, 0.05, 0.035478),RPY<double>(0,180*Deg2Rad,0).toRotation3D());
//
//    _graspTarget->attachTo(pose_estimation_frame, _state);
//    _graspTarget->setTransform(inverse(hToObj) * homogToGrasp, _state);
//
//    return rw::kinematics::Kinematics::frameTframe(_URReference, pose_estimation_frame, _state);
//
//}


//void SamplePlugin::showPoseEstimationM4() {
//    if (_homography[_bestHidx].homography_found) {
//        cv::imshow("homography matches", _homography[_bestHidx].match_image);
//        cv::waitKey();
//    }
//}

//void SamplePlugin::poseEstimationM4Test() {
//    //reset scene
//    _state = _wc->getDefaultState();
//    getRobWorkStudio()->setState(_state);
//
//    // optain pose estimation and milk frame
//    MovableFrame *pose_estimation_frame = _wc->findFrame<MovableFrame>("pose_estimate");
//    MovableFrame *milk_frame = _wc->findFrame<MovableFrame>("milk");
//    MovableFrame *camera_cv_frame = _wc->findFrame<MovableFrame>("Camera_Right_CV");
//    Transform3D<> milk_init_T = milk_frame->getTransform(_state);
//    std::cout << "milk initial pose : \n" << milk_init_T << std::endl;
//
//
//    //Rotate milk and do pose estimation
//    for (int s = 0; s < 10; s++) {
//        std::cout << "Sample " << s << std::endl;
//        ofstream csv_file;
//        csv_file.open("sample_" + std::to_string(s) + ".csv");
//        for (int i = 0; i < 360; i += 1) {
//            Transform3D<double> T =
//                    milk_init_T * Transform3D<double>(RPY<double>(i * Deg2Rad, 0, 0).toRotation3D());
//            milk_frame->setTransform(T, _state);
//            getRobWorkStudio()->setState(_state);
//            Transform3D<double> pose_estimate = poseEstimationM4();
//            if (_homography[_bestHidx].homography_found) {
//                Transform3D<double> ground_truth = rw::kinematics::Kinematics::frameTframe(_URReference, milk_frame,
//                                                                                           _state);
//                double angularDiff = EAA<double>(pose_estimate.R().inverse() * ground_truth.R()).angle();
//
//                double HCameraDiff = EAA<double>(
//                        Kinematics::frameTframe(camera_cv_frame, _homography[_bestHidx].frame, _state).R()).angle();
//                Vector3D<double> pDiff = (pose_estimate.P() - ground_truth.P());
//
//
//                csv_file << i << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " " << angularDiff << " "
//                         << M_PI - HCameraDiff << "\n";
//                //std::cout << i << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " " << angularDiff << " "
//                //      << M_PI - HCameraDiff << "\n";
//            }
//        }
//        csv_file.close();
//    }
//    milk_frame->setTransform(milk_init_T, _state);
//}

//void SamplePlugin::poseEstimationM4Test2() {
//    //reset scene
//    _state = _wc->getDefaultState();
//    getRobWorkStudio()->setState(_state);
//
//    // optain pose estimation and milk frame
//    MovableFrame *milk_frame = _wc->findFrame<MovableFrame>("milk");
//    MovableFrame *camera_cv_frame = _wc->findFrame<MovableFrame>("Camera_Right_CV");
//    Transform3D<> milk_init_T = milk_frame->getTransform(_state);
//    std::cout << "milk initial pose : \n" << milk_init_T << std::endl;
//
//    // optimize homography computation
//    for (auto h : _homography) {
//        h.draw_matches = false;
//    }
//    //Rotate milk and do pose estimation
//
//    std::cout << "gausian_noise" << std::endl;
//    ofstream csv_file;
//    csv_file.open("gausian_noise_0_100.csv");
//    for (int gstd = 0; gstd <= 200; gstd += 5) {
//        std::vector<double> computeTime;
//        std::vector<double> tDiff;
//        std::vector<double> aDiff;
//        for (int i = -10; i <= 10; i += 5) {
//            for (int s = 0; s < 5; s++) {
//                Transform3D<double> T =
//                        milk_init_T * Transform3D<double>(RPY<double>(i * Deg2Rad, 0, 0).toRotation3D());
//                milk_frame->setTransform(T, _state);
//                getRobWorkStudio()->setState(_state);
//                std::clock_t c_start = std::clock();
//                Transform3D<double> pose_estimate = poseEstimationM4(true, 0, gstd);
//                std::clock_t c_end = std::clock();
//                double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
//                std::cout << "CPU time used: " << time_elapsed_ms / 1000.0 << " s\n";
//
//                if (_homography[_bestHidx].homography_found) {
//                    Transform3D<double> ground_truth = rw::kinematics::Kinematics::frameTframe(_URReference,
//                                                                                               milk_frame,
//                                                                                               _state);
//                    double angularDiff = EAA<double>(pose_estimate.R().inverse() * ground_truth.R()).angle();
//
//                    double HCameraDiff = EAA<double>(
//                            Kinematics::frameTframe(camera_cv_frame, _homography[_bestHidx].frame,
//                                                    _state).R()).angle();
//                    Vector3D<double> pDiff = (pose_estimate.P() - ground_truth.P());
//
//                    computeTime.push_back(time_elapsed_ms / 1000.0);
//                    tDiff.push_back(pDiff.norm1());
//                    aDiff.push_back(angularDiff);
//
//                }
//
//            }
//        }
//        imwrite("gstd" + std::to_string(gstd) + ".jpg", _images[0]);
//        std::pair<double, double> computeTimeMV = S.meanAndVariance(computeTime);
//        std::pair<double, double> tDiffMV = S.meanAndVariance(tDiff);
//        std::pair<double, double> aDiffMV = S.meanAndVariance(aDiff);
//        csv_file << gstd << " " << tDiffMV.first << " " << tDiffMV.second << " " << aDiffMV.first << " "
//                 << aDiffMV.second << " " << computeTimeMV.first << " " << computeTimeMV.second << "\n";
//        std::cout << gstd << " " << tDiffMV.first << " " << tDiffMV.second << " " << aDiffMV.first << " "
//                  << aDiffMV.second << " " << computeTimeMV.first << " " << computeTimeMV.second << "\n";
//    }
//    csv_file.close();
//    milk_frame->setTransform(milk_init_T, _state);
//}


//void SamplePlugin::poseEstimationM2() {
//
//    double leaf = 0.003f;   // Discuss
//    double thressqRANSAC = leaf * leaf / 4;
//    double downsampleFactor = 1.75;
//
//    // Find the table surface
//    pclCloud::Ptr cloud(new pclCloud);
//    pclCloud::Ptr objectAligned(new pclCloud);
//    copyPointCloud(*_scan, *cloud);
//    pcl::PointIndices::Ptr indices_table(new pcl::PointIndices);
//
//
//    // Make a voxelgrid
//    VoxelGrid(cloud, cloud, leaf);
//
//    // Find indices for the table point
//    FindTable(cloud, indices_table);
//    // Remove the the table indices from the pointcloud
//    RemoveIndices(cloud, indices_table, cloud, true);
//
//    // Find indices for front plate point
//    FindTable(cloud, indices_table);
//    // Remove the the table indices from the pointcloud
//    RemoveIndices(cloud, indices_table, cloud, true);
//
//    //PointCloudVisualization(cloud);
//
//    // Here we would remove points below the table, but qhull does not work with the current version of pcl.
//    // We therefore use the romove table twice .
//
//    // Get all the clusters in the point cloud  Ignore idx = 0;
//    std::vector<pclCloud::Ptr> cloudClusters;
//    EuclideanClustering(cloud, cloudClusters);        // idx: 0 = Coffeecan; 1 = Milk; 2 = suzanne
//
//    //PointCloudsVisualization(cloudClusters);
//
//    // Get the model we cant to find in the scene
//    ModelObject objectModel;
//    {
//        objectModel.name = _modelName;
//        getModel(leaf, objectModel, 0);
//        objectModel.downsampleCloud(downsampleFactor);
//        std::cout << "Model " << objectModel.name << " with size: " << objectModel.cloud->points.size()
//                  << std::endl;
//    }
//
//
//    // Do the global pose estiamtion
//    Eigen::Matrix4d T;
//    std::vector<ModelObject> objectModels;
//    //double rmse = global_pose_estimater(objectModel, cloud, T, thressqRANSAC);
//    double rmse = global_pose_estimater(objectModel, cloudClusters[1], T, thressqRANSAC);
//
//    transformPointCloud(*objectModel.cloud, *objectAligned, T);
//    //Eigen::Matrix4d T2 = local_pose_estimater(objectAligned, cloud, 200);
//    //transformPointCloud(*objectAligned, *objectAligned, T2); // Local pose estimation doesnt work
//
//    _state = _wc->getDefaultState();
//
//    // ###### We now want to place the grasp targe frame at the correct location os the robt can pick it up #####
//
//
//
//    // Find the pose estimation
//
//    Transform3D<double> urTcamera = rw::kinematics::Kinematics::frameTframe(_URReference,
//                                                                            _wc->findFrame(_cameras25D[0]), _state);
//    Transform3D<double> cameraTmodel((Vector3D<double>) (T).block<3, 1>(0, 3),
//                                     (Rotation3D<double>) (T).block<3, 3>(0, 0));
//    //Transform3D<double> cameraTmodel( (Vector3D<double>)(T*T2).block<3,1>(0,3), (Rotation3D<double>)(T*T2).block<3,3>(0,0) );
//
//
//    // Transform to offset the grasp target in the right direction
//    Transform3D<double> graspOffset(Vector3D<double>(0, -0.05, 0.02) , RPY<double>(0, 0, -2.53072742).toRotation3D());
//
//    _wc->findFrame<MovableFrame>("Scanner_CV")->setTransform(cameraTmodel*graspOffset, _state);// transform frame to graps point and applies offset
//    _graspTarget->attachTo(_wc->findFrame<MovableFrame>("Scanner_CV"),_state);
//    _graspTarget->setTransform(Transform3D<double>::identity(),_state);
//    getRobWorkStudio()->setState(_state);
//
//    // ####### Calculate the error #####
//
//
//    // Get correct object pose
//    rw::kinematics::Frame *FrameObject = _wc->findFrame("suzanne");
//    Transform3D<double> ground_truth = rw::kinematics::Kinematics::frameTframe(_URReference, FrameObject, _state);
//
//    // Get the transformation in reation to the UR reference frame
//    Transform3D<double> pose_estimate = urTcamera * cameraTmodel;
//
//    double angularDiff = EAA<double>(pose_estimate.R().inverse() * ground_truth.R()).angle();
//    Vector3D<double> pDiff = (pose_estimate.P() - ground_truth.P());
//    std::cout << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " " << angularDiff << "\n";
//
//
//}

//void SamplePlugin::poseEstimationM2Test() {
//
//    // ############################# Test ##########################################
//
//    // Leaf
//    std::string testParameterName = "Downsample";
//    std::vector<double> testParameter = {0.005, 0.004, 0.003, 0.002};
//
//    // Downsampling
//
//    std::string testParameterName2 = "Downsample";
//    std::vector<double> testParameter2 = {4 / 4.0, 5 / 4.0, 6 / 4.0, 7 / 4.0, 8 / 4.0, 9 / 4.0, 10 / 4.0};
//
//    // z_noise
//    std::string testParameterName3 = "z-noise";
//    std::vector<double> testParameter3 = {0, 0.002, 0.004, 0.006, 0.008, 0.010};
//
//
//    for (int p = 0; p < testParameter3.size(); p++) {
//        // Parameter
//        double leaf = 0.003f;   // Unit meter
//        //double leaf = testParameter[p];
//
//        double thressqRANSAC = leaf * leaf / 4;
//        double downsampleFactor = 1.75;
//        //double downsampleFactor = testParameter[p];
//        //double z_noise_std = 0; // Unit meter
//        double z_noise_std = testParameter3[p]; // Unit meter
//
//        ofstream csv_file;
//        csv_file.open("M2_" + testParameterName3 + std::to_string(testParameter3[p]) + ".csv");
//        std::cout << "M2test: " + testParameterName3 + " = " + std::to_string(testParameter3[p]) << std::endl;
//
//
//        int testsize = 20;
//        for (int t = 0; t < testsize; t++) {
//            std::cout << "Run " << t << "/" << testsize - 1 << std::endl;
//            // Get the image scan
//            pclCloud::Ptr cloud(new pclCloud);
//            copyPointCloud(*_scan, *cloud);
//
//            pcl::PointIndices::Ptr indices_table(new pcl::PointIndices);
//
//            // Add some noice
//            {
//                double noise;
//                double len;
//                pcl::common::NormalGenerator<double> cameraNoise(0, z_noise_std);
//
//                for (int i = 0; i < cloud->points.size(); i++) {
//                    noise = cameraNoise.run();
//                    len = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y +
//                               cloud->points[i].z * cloud->points[i].z);
//                    cloud->points[i].x += (cloud->points[i].x / len) * noise;
//                    cloud->points[i].y += (cloud->points[i].y / len) * noise;
//                    cloud->points[i].z += (cloud->points[i].z / len) * noise;
//                }
//            }
//
//            // START TIMING
//            std::clock_t c_start = std::clock();
//
//            // Make a voxelgrid
//            VoxelGrid(cloud, cloud, leaf);
//
//            // Find indices for the table point
//            FindTable(cloud, indices_table);
//            // Remove the the table indices from the pointcloud
//            RemoveIndices(cloud, indices_table, cloud, true);
//
//
//            // Find indices for the front panel
//            FindTable(cloud, indices_table);
//            // Remove the front panel indices from the pointcloud
//            RemoveIndices(cloud, indices_table, cloud, true);
//
//            // Here we would remove points below the table, but qhull does not work with the current version of pcl.
//            // We therefore ignore idx = 0 in  cloudCluster.
//
//            // Get all the clusters in the point cloud  Ignore idx = 0;
//            std::vector<pclCloud::Ptr> cloudClusters;
//            EuclideanClustering(cloud,
//                                cloudClusters);        // idx: 0 = Front panel; 1 = Coffecan; 2 = Milk; 3 = Suzanne
//
//            // Get the model we cant to find in the scene
//            ModelObject objectModel;
//            {
//                objectModel.name = _modelName;
//                getModel(leaf, objectModel, 0);
//                objectModel.downsampleCloud(downsampleFactor);
//            }
//
//            // Get pose estimation
//            Eigen::Matrix4d T;
//            //double rmse = global_pose_estimater(objectModel, cloudClusters[1], T, thressqRANSAC);
//            double rmse = global_pose_estimater(objectModel, cloud, T, thressqRANSAC);
//
//            // STOP TIME
//            std::clock_t c_end = std::clock();
//            double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
//            std::cout << "CPU time used: " << time_elapsed_ms / 1000.0 << " s\n";
//
//
//            if (rmse != -1) // If a pose is found
//            {
//                // Get correct object pose
//                rw::kinematics::Frame *FrameObject = _wc->findFrame("suzanne");
//                Transform3D<double> ground_truth = rw::kinematics::Kinematics::frameTframe(_URReference,
//                                                                                           FrameObject,
//                                                                                           _state);
//
//                // Find the pose estimation
//                Transform3D<double> urTcamera = rw::kinematics::Kinematics::frameTframe(_URReference,
//                                                                                        _wc->findFrame(
//                                                                                                _cameras25D[0]),
//                                                                                        _state);
//                Transform3D<double> cameraTmodel((Vector3D<double>) T.block<3, 1>(0, 3),
//                                                 (Rotation3D<double>) T.block<3, 3>(0, 0));
//
//                Transform3D<double> pose_estimate = urTcamera * cameraTmodel;
//
//                // Find the error
//                double angularDiff = EAA<double>(pose_estimate.R().inverse() * ground_truth.R()).angle();
//                Vector3D<double> pDiff = (pose_estimate.P() - ground_truth.P());
//                csv_file << t << " " << rmse << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " "
//                         << angularDiff << " " << time_elapsed_ms / 1000.0 << "\n";
//
//            } else {
//                csv_file << t << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " "
//                         << time_elapsed_ms / 1000.0 << "\n";
//            }
//            std::cout << std::endl;
//        }
//        csv_file.close();
//    }
//}


//void SamplePlugin::PoseEstimationM2LocalTest() {
//
//    double leaf = 0.003f;   // Discuss
//    double thressqRANSAC = leaf * leaf / 4;
//    double downsampleFactor = 1.75;
//
//    // Find the table surface
//    pclCloud::Ptr cloud(new pclCloud);
//    pclCloud::Ptr objectAligned(new pclCloud);
//    copyPointCloud(*_scan, *cloud);
//    pcl::PointIndices::Ptr indices_table(new pcl::PointIndices);
//
//    // Make a voxelgrid
//    VoxelGrid(cloud, cloud, leaf);
//
//    // Find indices for the table point
//    FindTable(cloud, indices_table);
//
//    // Remove the the table indices from the pointcloud
//    RemoveIndices(cloud, indices_table, cloud, true);
//
//
//    // Here we would remove points below the table, but qhull does not work with the current version of pcl.
//    // We therefore ignore idx = 0 in  cloudCluster.
//
//    // Find indices for the fron panel
//    FindTable(cloud, indices_table);
//
//    // Remove the the front panel indices from the pointcloud
//    RemoveIndices(cloud, indices_table, cloud, true);
//
//    // Get all the clusters in the point cloud  Ignore idx = 0;
//    std::vector<pclCloud::Ptr> cloudClusters;
//    EuclideanClustering(cloud, cloudClusters);        // idx: 0 = Front panel; 1 = Coffeecan; 2 = Milk; 3 = suzanne
//
//    //PointCloudsVisualization(cloudClusters);
//
//    // Get the model we cant to find in the scene
//    ModelObject objectModel;
//    {
//        objectModel.name = _modelName;
//        getModel(leaf, objectModel, 0);
//        objectModel.downsampleCloud(downsampleFactor);
//        std::cout << "Model " << objectModel.name << " with size: " << objectModel.cloud->points.size()
//                  << std::endl;
//    }
//
//
//    _state = _wc->getDefaultState();
//
//    // Get correct object pose
//    rw::kinematics::Frame *FrameObject = _wc->findFrame("suzanne");
//    Transform3D<double> ground_truth = rw::kinematics::Kinematics::frameTframe(_wc->findFrame(_cameras25D[0]),
//                                                                               FrameObject, _state);
//
//    // Find the pose estimation
//    Transform3D<double> cameraTobject = rw::kinematics::Kinematics::frameTframe(_wc->findFrame(_cameras25D[0]),
//                                                                                FrameObject, _state);
//
//    transformPointCloud(*objectModel.cloud, *objectAligned, cameraTobject.e());
//
//    // Find the local transformation
//    //Eigen::Matrix4d T_ = local_pose_estimater(objectAligned, cloud, 200);
//    Eigen::Matrix4d T_ = local_pose_estimater(objectAligned, cloudClusters[1], 200);
//
//    transformPointCloud(*objectAligned, *objectAligned, T_);
//
//    // Find the error
//    Transform3D<double> T((Vector3D<double>) T_.block<3, 1>(0, 3),
//                          (Rotation3D<double>) T_.block<3, 3>(0, 0));
//    Transform3D<double> poseEstimation = cameraTobject * T;
//
//    // Ideal -0.00340465 -0.00395989 -0.00151847 0.00358766
//    double angularDiff = EAA<double>(poseEstimation.R().inverse() * ground_truth.R()).angle();
//    Vector3D<double> pDiff = (poseEstimation.P() - ground_truth.P());
//    std::cout << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " " << angularDiff << "\n";
//
//
//    if (true) {       //  Display the original and the filtered cloud
//        pcl::visualization::PCLVisualizer viewer;
//        viewer.addPointCloud<pcl::PointXYZ>(objectAligned,
//                                            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//                                                    objectAligned, 0, 255, 0), "cloud_RGB");
//        viewer.addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//                cloud, 255, 0, 0), "cloud_RGB1");
//        viewer.spin();
//
//        while (!viewer.wasStopped()) {
//            viewer.spinOnce(100);
//            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//        }
//
//        viewer.close();
//    }
//
//
//
//    // ############ The model is now place exactly in the scene, and we now want to add errors // Ideal [0.00398491 -0.00770718 -0.00198201 0.00742114]
//
//    std::vector<double> rot_std = {0.09, 0.17, 0.26, 0.35}; // unit rad // 1 deg = 0.01745329252 rad
//    std::vector<double> pos_std = {0.005, 0.01, 0.015, 0.02, 0.025, 0.03}; // Unit m
//
//    for (int par_pos = 0; par_pos < pos_std.size(); par_pos++) {
//        for (int par_rot = 0; par_rot < rot_std.size(); par_rot++) {
//            ofstream csv_file;
//            csv_file.open("M2Local_rot" + std::to_string(rot_std[par_rot])  + "pos" + std::to_string(pos_std[par_pos]) + ".csv");
//            std::cout << "M2test:  rot = " << std::to_string(rot_std[par_rot]) + " pos = " <<  std::to_string(pos_std[par_pos]) << std::endl;
//
//
//            // Generate Unit vectors
//            const int testSize = 2;
//            std::vector<Eigen::Vector3d> unitVectorsr;
//            std::vector<Eigen::Vector3d> unitVectorsp;
//            {
//                unitVectorsr.resize(testSize);
//                unitVectorsp.resize(testSize);
//
//                typedef boost::random::mt19937 gen_type;
//
//                // You can seed this your way as well, but this is my quick and dirty way.
//                gen_type rand_gen;
//                rand_gen.seed(static_cast<unsigned int>(std::time(0)));
//
//                // Create the distribution object.
//                boost::uniform_on_sphere<float> unif_sphere(3);
//
//                // This is what will actually supply drawn values.
//                boost::variate_generator<gen_type&, boost::uniform_on_sphere<float> >    random_on_sphere(rand_gen, unif_sphere);
//
//                for (int i = 0; i < testSize; i++) {
//
//                    // Now you can draw a vector of drawn coordinates as such:
//                    std::vector<float> random_sphere_pointp = random_on_sphere();
//                    std::vector<float> random_sphere_pointr = random_on_sphere();
//
//                    Eigen::Vector3d pp(random_sphere_pointp[0], random_sphere_pointp[1], random_sphere_pointp[2]);
//                    Eigen::Vector3d pr(random_sphere_pointr[0], random_sphere_pointr[1], random_sphere_pointr[2]);
//                    unitVectorsp[i] = pp;
//                    unitVectorsr[i] = pr;
//                }
//            }
//
//            // construct a trivial random generator engine from a time-based seed:
//            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//            std::default_random_engine gen(seed);
//
//            boost::normal_distribution<double> dr{0.0, rot_std[par_rot]};   // unit rad // 1 deg = 0.01745329252 rad
//            boost::normal_distribution<double> dp{0.0, pos_std[par_pos]};   // unit m
//
//            // Cointainer for the noise point cloud
//            pclCloud::Ptr objectAligned_noise(new pclCloud);
//
//            for (int t = 0; t < testSize; t++) {
//                // Create the noise
//                Eigen::Matrix3d rot_noise = rw::math::EAA<double>((Vector3D<double>) unitVectorsr[t],
//                                                                  dr(gen)).toRotation3D().e();
//                Eigen::Vector3d pos_noise = unitVectorsp[t] * dp(gen);
//
//                Eigen::Matrix4d T_noise_ = Eigen::Matrix4d::Zero();
//                T_noise_.block<3, 3>(0, 0) = rot_noise;
//                T_noise_.block<3, 1>(0, 3) = pos_noise;
//                T_noise_(3, 3) = 1;
//
//
//                // Add the noise to the point cloud
//                transformPointCloud(*objectModel.cloud_downsampled, *objectAligned_noise, T_noise_);
//                transformPointCloud(*objectAligned_noise, *objectAligned_noise, cameraTobject.e());
//
//
//                //transformPointCloud(*objectAligned, *objectAligned_noise, T_noise_);
//
//                if (false) {       //  Display the original and the filtered cloud
//                    pcl::visualization::PCLVisualizer viewer;
//                    viewer.addPointCloud<pcl::PointXYZ>(objectAligned_noise,
//                                                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//                                                                objectAligned_noise, 0, 255, 0), "cloud_RGB");
//                    viewer.addPointCloud<pcl::PointXYZ>(cloudClusters[1],
//                                                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
//                                                                cloudClusters[1], 255, 0, 0), "cloud_RGB1");
//                    viewer.spin();
//
//                    while (!viewer.wasStopped()) {
//                        viewer.spinOnce(100);
//                        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//                    }
//
//                    viewer.close();
//                }
//
//
//                // START TIMING
//                std::clock_t c_start = std::clock();
//
//                // Find the transformation
//                Eigen::Matrix4d T_ = local_pose_estimater(objectAligned_noise, cloudClusters[1], 200);
//
//
//                // STOP TIME
//                std::clock_t c_end = std::clock();
//                double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
//                std::cout << "CPU time used: " << time_elapsed_ms / 1000.0 << " s\n";
//
//                if (T_ == Eigen::Matrix4d::Identity()) {
//
//                    csv_file << t << " " << -1 << " " << -1 << " " << -1 << " " << -1 << "\n";
//                    std::cout << "No matches" << rot_std[par_rot] << " " << pos_std[par_pos] << std::endl;
//
//                } else {
//
//                    // Find the error
//                    Transform3D<double> T((Vector3D<double>) T_.block<3, 1>(0, 3),
//                                          (Rotation3D<double>) T_.block<3, 3>(0, 0));
//                    Transform3D<double> T_noise((Vector3D<double>) T_noise_.block<3, 1>(0, 3),
//                                                (Rotation3D<double>) T_noise_.block<3, 3>(0, 0));
//                    //Transform3D<double> poseEstimation = cameraTobject*T_noise*T;
//
//
//                    double angularDiff = EAA<double>(T.R().inverse() * T_noise.R()).angle();
//                    Vector3D<double> pDiff = (T.P() - T_noise.P());
//                    std::cout << t << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " " << angularDiff << " " << time_elapsed_ms / 1000.0  << "\n";
//                    csv_file << t << " " << pDiff[0] << " " << pDiff[1] << " " << pDiff[2] << " " << angularDiff << " " << time_elapsed_ms / 1000.0 << "\n";
//                }
//            }
//
//            std::cout << std::endl;
//            csv_file.close();
//        }
//
//    }
//    std::cout << "Test Done" << std::endl;
//}

//Transform3D<double> SamplePlugin::eig4x4ToTransform3D(Eigen::Matrix<double, 4, 4> eigMat4x4) {
//    return Transform3D<double>(
//            Vector3D<double>(eigMat4x4.col(3).coeff(0), eigMat4x4.col(3).coeff(1), eigMat4x4.col(3).coeff(2)),
//            Rotation3D<double>(
//                    Vector3D<double>(eigMat4x4.row(0).coeff(0), eigMat4x4.row(0).coeff(1),
//                                     eigMat4x4.row(0).coeff(2)),
//                    Vector3D<double>(eigMat4x4.row(1).coeff(0), eigMat4x4.row(1).coeff(1),
//                                     eigMat4x4.row(1).coeff(2)),
//                    Vector3D<double>(eigMat4x4.row(2).coeff(0), eigMat4x4.row(2).coeff(1),
//                                     eigMat4x4.row(2).coeff(2))));
//}


// ########################################################### Helper functions  #######################################


//Eigen::Matrix<double, 3, 4> SamplePlugin::printProjectionMatrix(std::string frameName) {
//    rw::kinematics::Frame *cameraFrame = _wc->findFrame(frameName);
//    if (cameraFrame != NULL) {
//        if (cameraFrame->getPropertyMap().has("Camera")) {
//            // Read the dimensions and field of view
//            double fovy;
//            int width, height;
//            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
//            std::istringstream iss(camParam, std::istringstream::in);
//            iss >> fovy >> width >> height;
//
//            double f = height / 2 / tan(fovy * (2 * M_PI) / 360.0 / 2.0);//focal length in pixels
//
//            Eigen::Matrix<double, 3, 4> KA;
//            KA << f, 0, width / 2.0, 0,
//                    0, f, height / 2.0, 0,
//                    0, 0, 1, 0;
//
//            return KA;
//        }
//    } else {
//        std::cout << frameName + " not a camera !!!" << std::endl;
//    }
//}


//// https://answers.opencv.org/question/68589/adding-noise-to-image-opencv/
//bool
//SamplePlugin::AddGaussianNoise_Opencv(const cv::Mat mSrc, cv::Mat &mDst, double Mean = 0.0, double StdDev = 10.0) {
//    if (mSrc.empty()) {
//        cout << "[Error]! Input Image Empty!";
//        return 0;
//    }
//    Mat mSrc_16SC;
//    Mat mGaussian_noise = Mat(mSrc.size(), CV_16SC3);
//    randn(mGaussian_noise, Scalar::all(Mean), Scalar::all(StdDev));
//
//    mSrc.convertTo(mSrc_16SC, CV_16SC3);
//    addWeighted(mSrc_16SC, 1.0, mGaussian_noise, 1.0, 0.0, mSrc_16SC);
//    mSrc_16SC.convertTo(mDst, mSrc.type());
//
//    return true;
//}
//
