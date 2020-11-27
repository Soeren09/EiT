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
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *_getGantry;
    QPushButton *_getUR;
    QPushButton *_calculateTraj;
    QPushButton *_executeTraj;
    QSpacerItem *verticalSpacer;
    QPushButton *_reset;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(476, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        _getGantry = new QPushButton(dockWidgetContents);
        _getGantry->setObjectName(QString::fromUtf8("_getGantry"));

        horizontalLayout->addWidget(_getGantry);

        _getUR = new QPushButton(dockWidgetContents);
        _getUR->setObjectName(QString::fromUtf8("_getUR"));

        horizontalLayout->addWidget(_getUR);


        verticalLayout_2->addLayout(horizontalLayout);

        _calculateTraj = new QPushButton(dockWidgetContents);
        _calculateTraj->setObjectName(QString::fromUtf8("_calculateTraj"));

        verticalLayout_2->addWidget(_calculateTraj);

        _executeTraj = new QPushButton(dockWidgetContents);
        _executeTraj->setObjectName(QString::fromUtf8("_executeTraj"));

        verticalLayout_2->addWidget(_executeTraj);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        _reset = new QPushButton(dockWidgetContents);
        _reset->setObjectName(QString::fromUtf8("_reset"));

        verticalLayout_2->addWidget(_reset);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", nullptr));
        _getGantry->setText(QApplication::translate("SamplePlugin", "Get Gantry", nullptr));
        _getUR->setText(QApplication::translate("SamplePlugin", "Get UR", nullptr));
        _calculateTraj->setText(QApplication::translate("SamplePlugin", "Calculate Trajectory", nullptr));
        _executeTraj->setText(QApplication::translate("SamplePlugin", "Execute Trajectory", nullptr));
        _reset->setText(QApplication::translate("SamplePlugin", "Reset", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
