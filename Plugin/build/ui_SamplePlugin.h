/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *_btn_scan;
    QPushButton *_btn_img;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *_btn_impl_m2;
    QPushButton *_btn_impl_m4;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *_btn_suzanne;
    QPushButton *_btn_pe_m2;
    QPushButton *_btn_test_m2;
    QPushButton *_btn_show_m2;
    QVBoxLayout *verticalLayout_5;
    QPushButton *_btn_pe_m4;
    QPushButton *_btn_test_m4;
    QPushButton *_btn_test_m4_2;
    QPushButton *_btn_show_m4;
    QSlider *_slider1;
    QHBoxLayout *horizontalLayout_4;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_51;
    QPushButton *_btn_ptp;
    QPushButton *_btn_ptp_test;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *_btn_rrt;
    QPushButton *_btn_rrt_test;
    QPushButton *_btn_exec_trajectory;
    QPushButton *_save_q_traj;
    QLabel *_label;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(598, 653);
        SamplePlugin->setFloating(false);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        _btn_scan = new QPushButton(dockWidgetContents);
        _btn_scan->setObjectName(QString::fromUtf8("_btn_scan"));

        horizontalLayout_3->addWidget(_btn_scan);

        _btn_img = new QPushButton(dockWidgetContents);
        _btn_img->setObjectName(QString::fromUtf8("_btn_img"));

        horizontalLayout_3->addWidget(_btn_img);


        verticalLayout_6->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _btn_impl_m2 = new QPushButton(dockWidgetContents);
        _btn_impl_m2->setObjectName(QString::fromUtf8("_btn_impl_m2"));

        horizontalLayout_2->addWidget(_btn_impl_m2);

        _btn_impl_m4 = new QPushButton(dockWidgetContents);
        _btn_impl_m4->setObjectName(QString::fromUtf8("_btn_impl_m4"));

        horizontalLayout_2->addWidget(_btn_impl_m4);


        verticalLayout_6->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setSizeConstraint(QLayout::SetMinimumSize);
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        _btn_suzanne = new QPushButton(dockWidgetContents);
        _btn_suzanne->setObjectName(QString::fromUtf8("_btn_suzanne"));

        horizontalLayout_5->addWidget(_btn_suzanne);


        verticalLayout_3->addLayout(horizontalLayout_5);

        _btn_pe_m2 = new QPushButton(dockWidgetContents);
        _btn_pe_m2->setObjectName(QString::fromUtf8("_btn_pe_m2"));

        verticalLayout_3->addWidget(_btn_pe_m2);

        _btn_test_m2 = new QPushButton(dockWidgetContents);
        _btn_test_m2->setObjectName(QString::fromUtf8("_btn_test_m2"));

        verticalLayout_3->addWidget(_btn_test_m2);

        _btn_show_m2 = new QPushButton(dockWidgetContents);
        _btn_show_m2->setObjectName(QString::fromUtf8("_btn_show_m2"));

        verticalLayout_3->addWidget(_btn_show_m2);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setSizeConstraint(QLayout::SetMinimumSize);
        _btn_pe_m4 = new QPushButton(dockWidgetContents);
        _btn_pe_m4->setObjectName(QString::fromUtf8("_btn_pe_m4"));

        verticalLayout_5->addWidget(_btn_pe_m4);

        _btn_test_m4 = new QPushButton(dockWidgetContents);
        _btn_test_m4->setObjectName(QString::fromUtf8("_btn_test_m4"));

        verticalLayout_5->addWidget(_btn_test_m4);

        _btn_test_m4_2 = new QPushButton(dockWidgetContents);
        _btn_test_m4_2->setObjectName(QString::fromUtf8("_btn_test_m4_2"));

        verticalLayout_5->addWidget(_btn_test_m4_2);

        _btn_show_m4 = new QPushButton(dockWidgetContents);
        _btn_show_m4->setObjectName(QString::fromUtf8("_btn_show_m4"));

        verticalLayout_5->addWidget(_btn_show_m4);


        horizontalLayout->addLayout(verticalLayout_5);

        _slider1 = new QSlider(dockWidgetContents);
        _slider1->setObjectName(QString::fromUtf8("_slider1"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(_slider1->sizePolicy().hasHeightForWidth());
        _slider1->setSizePolicy(sizePolicy);
        _slider1->setMinimum(0);
        _slider1->setMaximum(1000);
        _slider1->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(_slider1);


        verticalLayout_6->addLayout(horizontalLayout);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalLayout_51 = new QHBoxLayout();
        horizontalLayout_51->setObjectName(QString::fromUtf8("horizontalLayout_51"));
        _btn_ptp = new QPushButton(dockWidgetContents);
        _btn_ptp->setObjectName(QString::fromUtf8("_btn_ptp"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(_btn_ptp->sizePolicy().hasHeightForWidth());
        _btn_ptp->setSizePolicy(sizePolicy1);

        horizontalLayout_51->addWidget(_btn_ptp);

        _btn_ptp_test = new QPushButton(dockWidgetContents);
        _btn_ptp_test->setObjectName(QString::fromUtf8("_btn_ptp_test"));

        horizontalLayout_51->addWidget(_btn_ptp_test);


        gridLayout->addLayout(horizontalLayout_51, 0, 0, 1, 1);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        _btn_rrt = new QPushButton(dockWidgetContents);
        _btn_rrt->setObjectName(QString::fromUtf8("_btn_rrt"));
        sizePolicy1.setHeightForWidth(_btn_rrt->sizePolicy().hasHeightForWidth());
        _btn_rrt->setSizePolicy(sizePolicy1);

        horizontalLayout_6->addWidget(_btn_rrt);

        _btn_rrt_test = new QPushButton(dockWidgetContents);
        _btn_rrt_test->setObjectName(QString::fromUtf8("_btn_rrt_test"));

        horizontalLayout_6->addWidget(_btn_rrt_test);


        gridLayout->addLayout(horizontalLayout_6, 0, 2, 1, 1);

        _btn_exec_trajectory = new QPushButton(dockWidgetContents);
        _btn_exec_trajectory->setObjectName(QString::fromUtf8("_btn_exec_trajectory"));

        gridLayout->addWidget(_btn_exec_trajectory, 2, 0, 1, 3);

        _save_q_traj = new QPushButton(dockWidgetContents);
        _save_q_traj->setObjectName(QString::fromUtf8("_save_q_traj"));

        gridLayout->addWidget(_save_q_traj, 0, 1, 1, 1);


        horizontalLayout_4->addLayout(gridLayout);


        verticalLayout_6->addLayout(horizontalLayout_4);


        verticalLayout->addLayout(verticalLayout_6);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));

        verticalLayout->addWidget(_label);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", nullptr));
        _btn_scan->setText(QApplication::translate("SamplePlugin", "Get scan", nullptr));
        _btn_img->setText(QApplication::translate("SamplePlugin", "Get image", nullptr));
        _btn_impl_m2->setText(QApplication::translate("SamplePlugin", "Implementation M2", nullptr));
        _btn_impl_m4->setText(QApplication::translate("SamplePlugin", "Implementation M4", nullptr));
        _btn_suzanne->setText(QApplication::translate("SamplePlugin", "Suzanne", nullptr));
        _btn_pe_m2->setText(QApplication::translate("SamplePlugin", "Pose estimate M2", nullptr));
        _btn_test_m2->setText(QApplication::translate("SamplePlugin", "M2 RANSAC Test", nullptr));
        _btn_show_m2->setText(QApplication::translate("SamplePlugin", "M2 ICP Test", nullptr));
        _btn_pe_m4->setText(QApplication::translate("SamplePlugin", "Pose estimate M4", nullptr));
        _btn_test_m4->setText(QApplication::translate("SamplePlugin", "M4 Rotation Test", nullptr));
        _btn_test_m4_2->setText(QApplication::translate("SamplePlugin", "M4 Noise Test", nullptr));
        _btn_show_m4->setText(QApplication::translate("SamplePlugin", "M4 Show", nullptr));
        _btn_ptp->setText(QApplication::translate("SamplePlugin", "PTP", nullptr));
        _btn_ptp_test->setText(QApplication::translate("SamplePlugin", "T", nullptr));
        _btn_rrt->setText(QApplication::translate("SamplePlugin", "RRT", nullptr));
        _btn_rrt_test->setText(QApplication::translate("SamplePlugin", "T", nullptr));
        _btn_exec_trajectory->setText(QApplication::translate("SamplePlugin", "Execute Trajectory", nullptr));
        _save_q_traj->setText(QApplication::translate("SamplePlugin", "Save Q trajectory", nullptr));
        _label->setText(QApplication::translate("SamplePlugin", "Label", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
