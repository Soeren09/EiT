#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP
// Qt
#include <QTimer>
#include <QPushButton>

#include "ui_SamplePlugin.h"

// RobWork includes#
#include <rw/rw.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/opengl/RenderImage.hpp>

#define RRT 1
#define PTP 2
#define DT 0.1

#define M2 22
#define M4 44
// RobWorkStudio includes
#include <rws/RobWorkStudio.hpp>
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

//// OpenCV 2 (4.1.1)
//#include "opencv2/core/mat.hpp"
//
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>


#include <functional>
//#include "Homography.h"
//#include "ModelObject.hpp"


class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();

    virtual ~SamplePlugin();
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();


private slots:
    void btnPress();
    void timer();

private:
//    void getImage();
//    void get25DImage();
//    void updateImage(cv::Mat &imflip);
//    void stateChangedListener(const rw::kinematics::State& state);
//    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
//    rw::math::Transform3D<double> eig4x4ToTransform3D(Eigen::Matrix<double,4,4> eigMat4x4);

//    // ----- vision ------
//    rw::math::Transform3D<double> poseEstimationM4(bool addNoise = false , double mean = 0.0, double std = 10.0);
//    void poseEstimationM4Test();
//    void poseEstimationM2Test();
//    void PoseEstimationM2LocalTest();
//    void poseEstimationM4Test2();
//    void showPoseEstimationM4();
//    void poseEstimationM2();
//    bool AddGaussianNoise_Opencv(const cv::Mat mSrc, cv::Mat &mDst,double Mean, double StdDev);
//    Eigen::Matrix<double, 3, 4>  printProjectionMatrix(std::string frameName);

    //------ robotics -----
//    bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
//    void createPathRRTConnect(rw::math::Q from, rw::math::Q to,  double extend, double maxTime);
//    void createPTPPath(std::vector<rw::math::Q> qs, double dt);
//    void executePath(double dt);
//    void rrtTest();
//    void ptpTest();
//    void generateEntryAndSafePoints();
//
//    void implementation(int peType);
//
//    rw::math::Q getQ(rw::math::Transform3D<double> target,rw::math::Q &qclose);
//    rw::math::Q getQ(rw::kinematics::Frame* frame,rw::math::Q &qclose );
//    rw::math::Transform3D<double> getPlanningTransform3D(rw::kinematics::Frame* frame);


//    // Utillity
//
//    // Qt
//    QTimer* _timer;
//
//    // Scene and device
//    rw::models::WorkCell::Ptr _wc;
//    rw::models::SerialDevice::Ptr _robot;
//    rw::models::TreeDevice::Ptr _gripper;
//
//    // frames and transforms
//    rw::kinematics::State _state;
//    rw::kinematics::State _state_default;
//    rw::kinematics::MovableFrame* _URReference;
//    rw::math::Transform3D<double> _tcpCorrection;
//    rw::kinematics::MovableFrame* _graspTarget;
//    rw::kinematics::MovableFrame* _placeTarget;
//
//
//    // robotics / planning
//    rw::trajectory::QPath _path;
//    rw::invkin::ClosedFormIKSolverUR::Ptr _closedFormSolver;
//    rw::proximity::CollisionDetector::Ptr _detector;
//
//    int _path_type = -1;
//    int _step ;
//
//    // Cameras and renderers
//    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
////    rwlibs::simulation::GLFrameGrabber* _framegrabber;
////    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;
////    std::vector<std::string> _cameras;
////    std::vector<std::string> _cameras25D;
////    std::array<cv::Mat,2> _images;
////    pcl::PointCloud<pcl::PointXYZ>::Ptr _scan;
//    std::string _modelName;
//
////    // Homography related
////    std::vector<Homography> _homography;
////    unsigned int _bestHidx;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/