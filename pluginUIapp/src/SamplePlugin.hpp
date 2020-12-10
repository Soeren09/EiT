#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// Qt
#include <QTimer>
#include <QPushButton>

// RobWork includes
#include <rw/rw.hpp>
#include <rwlibs/opengl/RenderImage.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_SamplePlugin.h"

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
    void btnPressed();
    void stateChangedListener(const rw::kinematics::State& state);


private:

    // Initialization
    void getUR();
    void getGantry();


    // Robotics
    bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);

    rw::math::Q getQ(rw::math::Transform3D<double> target,rw::math::Q &qclose);
    rw::math::Q getQ(rw::kinematics::Frame* frame,rw::math::Q &qclose );
    rw::math::Transform3D<double> getPlanningTransform3D(rw::kinematics::Frame* frame);

    // Robotics / planning
    void runPath(bool setFinalPos);
    void invCalc();
    void createPTPPath(std::vector<rw::math::Q> qs, double dt);
    void executePath(double dt, bool setFinal);

    rw::invkin::IKMetaSolver::Ptr _IKSolver;

    rw::trajectory::QPath _path;
    rw::invkin::ClosedFormIKSolverUR::Ptr _closedFormSolver;
    rw::proximity::CollisionDetector::Ptr _detector;


    // Scene and device
    rw::models::WorkCell::Ptr _wc;
    rw::models::SerialDevice::Ptr _robot;
    rw::models::Device::Ptr _gantry;
    rw::models::TreeDevice::Ptr _gripper;

    // frames and transforms
    rw::kinematics::State _state;
    rw::kinematics::State _state_default;
    rw::kinematics::MovableFrame* _URReference;
    rw::math::Transform3D<double> _tcpCorrection;
    rw::kinematics::MovableFrame* _graspTarget;
//    rw::kinematics::MovableFrame* _placeTarget;


    // Cameras and renderers
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;

    // Qt
    QTimer* _timer;


};

#endif /*SAMPLEPLUGIN_HPP*/
