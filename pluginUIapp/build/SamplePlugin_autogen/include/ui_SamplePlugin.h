/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 5.9.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
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
            SamplePlugin->setObjectName(QStringLiteral("SamplePlugin"));
        SamplePlugin->resize(476, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        _getGantry = new QPushButton(dockWidgetContents);
        _getGantry->setObjectName(QStringLiteral("_getGantry"));

        horizontalLayout->addWidget(_getGantry);

        _getUR = new QPushButton(dockWidgetContents);
        _getUR->setObjectName(QStringLiteral("_getUR"));

        horizontalLayout->addWidget(_getUR);


        verticalLayout_2->addLayout(horizontalLayout);

        _calculateTraj = new QPushButton(dockWidgetContents);
        _calculateTraj->setObjectName(QStringLiteral("_calculateTraj"));

        verticalLayout_2->addWidget(_calculateTraj);

        _executeTraj = new QPushButton(dockWidgetContents);
        _executeTraj->setObjectName(QStringLiteral("_executeTraj"));

        verticalLayout_2->addWidget(_executeTraj);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        _reset = new QPushButton(dockWidgetContents);
        _reset->setObjectName(QStringLiteral("_reset"));

        verticalLayout_2->addWidget(_reset);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", Q_NULLPTR));
        _getGantry->setText(QApplication::translate("SamplePlugin", "Get Gantry", Q_NULLPTR));
        _getUR->setText(QApplication::translate("SamplePlugin", "Get UR", Q_NULLPTR));
        _calculateTraj->setText(QApplication::translate("SamplePlugin", "Record Trajectory", Q_NULLPTR));
        _executeTraj->setText(QApplication::translate("SamplePlugin", "Record and Apply Trajectory", Q_NULLPTR));
        _reset->setText(QApplication::translate("SamplePlugin", "Reset", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
