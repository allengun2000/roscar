/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"
#include "qwt_dial.h"
#include "qwt_slider.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *info;
    QLabel *wheel_n_2;
    QwtDial *speed;
    QLabel *wheel_num;
    QwtDial *oil;
    QLabel *picture;
    QPushButton *start;
    QLabel *wheel_num_2;
    QLabel *joystick;
    QwtDial *wheel;
    QCheckBox *learn;
    QLabel *picture_2;
    QwtDial *wheel_2;
    QLabel *speed_n;
    QLabel *oil_num;
    QLabel *oil_n;
    QLabel *wheel_n;
    QCheckBox *key_board;
    QLabel *speed_num;
    QPushButton *stop;
    QCustomPlot *way_info;
    QWidget *parm;
    QwtSlider *hmax;
    QwtSlider *h_min;
    QwtSlider *s_max;
    QwtSlider *s_min;
    QwtSlider *v_max;
    QwtSlider *v_min;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(811, 1014);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 801, 901));
        info = new QWidget();
        info->setObjectName(QStringLiteral("info"));
        wheel_n_2 = new QLabel(info);
        wheel_n_2->setObjectName(QStringLiteral("wheel_n_2"));
        wheel_n_2->setGeometry(QRect(620, 720, 81, 41));
        wheel_n_2->setLineWidth(2);
        wheel_n_2->setScaledContents(true);
        wheel_n_2->setWordWrap(false);
        speed = new QwtDial(info);
        speed->setObjectName(QStringLiteral("speed"));
        speed->setGeometry(QRect(570, 360, 201, 191));
        speed->setUpperBound(99);
        speed->setScaleMaxMajor(10);
        speed->setScaleMaxMinor(5);
        speed->setTotalSteps(100u);
        speed->setReadOnly(true);
        speed->setLineWidth(4);
        wheel_num = new QLabel(info);
        wheel_num->setObjectName(QStringLiteral("wheel_num"));
        wheel_num->setGeometry(QRect(470, 730, 31, 41));
        oil = new QwtDial(info);
        oil->setObjectName(QStringLiteral("oil"));
        oil->setGeometry(QRect(360, 360, 201, 191));
        oil->setLowerBound(0);
        oil->setUpperBound(5);
        oil->setScaleMaxMajor(10);
        oil->setScaleMaxMinor(5);
        oil->setTotalSteps(100u);
        oil->setReadOnly(true);
        oil->setLineWidth(4);
        picture = new QLabel(info);
        picture->setObjectName(QStringLiteral("picture"));
        picture->setGeometry(QRect(0, 60, 371, 271));
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush);
        QBrush brush1(QColor(242, 221, 202, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        picture->setPalette(palette);
        picture->setContextMenuPolicy(Qt::DefaultContextMenu);
        picture->setAcceptDrops(false);
        picture->setTextInteractionFlags(Qt::LinksAccessibleByMouse);
        start = new QPushButton(info);
        start->setObjectName(QStringLiteral("start"));
        start->setGeometry(QRect(510, 570, 99, 27));
        wheel_num_2 = new QLabel(info);
        wheel_num_2->setObjectName(QStringLiteral("wheel_num_2"));
        wheel_num_2->setGeometry(QRect(700, 720, 41, 41));
        joystick = new QLabel(info);
        joystick->setObjectName(QStringLiteral("joystick"));
        joystick->setGeometry(QRect(50, 610, 250, 250));
        joystick->setAutoFillBackground(false);
        wheel = new QwtDial(info);
        wheel->setObjectName(QStringLiteral("wheel"));
        wheel->setGeometry(QRect(350, 620, 221, 201));
        QFont font;
        font.setPointSize(6);
        font.setItalic(true);
        wheel->setFont(font);
        wheel->setLowerBound(-721);
        wheel->setUpperBound(721);
        wheel->setScaleMaxMajor(10);
        wheel->setScaleMaxMinor(5);
        wheel->setTotalSteps(100u);
        wheel->setReadOnly(true);
        wheel->setLineWidth(4);
        learn = new QCheckBox(info);
        learn->setObjectName(QStringLiteral("learn"));
        learn->setGeometry(QRect(620, 560, 97, 22));
        picture_2 = new QLabel(info);
        picture_2->setObjectName(QStringLiteral("picture_2"));
        picture_2->setGeometry(QRect(0, 340, 371, 271));
        wheel_2 = new QwtDial(info);
        wheel_2->setObjectName(QStringLiteral("wheel_2"));
        wheel_2->setGeometry(QRect(560, 610, 241, 211));
        wheel_2->setFont(font);
        wheel_2->setLowerBound(-35);
        wheel_2->setUpperBound(35);
        wheel_2->setScaleMaxMajor(10);
        wheel_2->setScaleMaxMinor(5);
        wheel_2->setTotalSteps(100u);
        wheel_2->setReadOnly(true);
        wheel_2->setLineWidth(4);
        speed_n = new QLabel(info);
        speed_n->setObjectName(QStringLiteral("speed_n"));
        speed_n->setGeometry(QRect(620, 470, 51, 21));
        speed_n->setLineWidth(2);
        speed_n->setScaledContents(true);
        speed_n->setWordWrap(false);
        oil_num = new QLabel(info);
        oil_num->setObjectName(QStringLiteral("oil_num"));
        oil_num->setGeometry(QRect(470, 470, 31, 31));
        oil_n = new QLabel(info);
        oil_n->setObjectName(QStringLiteral("oil_n"));
        oil_n->setGeometry(QRect(430, 470, 21, 31));
        oil_n->setLineWidth(2);
        oil_n->setScaledContents(true);
        oil_n->setWordWrap(false);
        wheel_n = new QLabel(info);
        wheel_n->setObjectName(QStringLiteral("wheel_n"));
        wheel_n->setGeometry(QRect(410, 730, 41, 41));
        wheel_n->setLineWidth(2);
        wheel_n->setScaledContents(true);
        wheel_n->setWordWrap(false);
        key_board = new QCheckBox(info);
        key_board->setObjectName(QStringLiteral("key_board"));
        key_board->setGeometry(QRect(510, 550, 97, 22));
        speed_num = new QLabel(info);
        speed_num->setObjectName(QStringLiteral("speed_num"));
        speed_num->setGeometry(QRect(680, 470, 51, 21));
        speed_num->setLineWidth(2);
        speed_num->setScaledContents(true);
        speed_num->setWordWrap(false);
        stop = new QPushButton(info);
        stop->setObjectName(QStringLiteral("stop"));
        stop->setGeometry(QRect(510, 600, 99, 27));
        way_info = new QCustomPlot(info);
        way_info->setObjectName(QStringLiteral("way_info"));
        way_info->setGeometry(QRect(400, 60, 361, 271));
        tabWidget->addTab(info, QString());
        wheel_2->raise();
        wheel->raise();
        wheel_n_2->raise();
        speed->raise();
        wheel_num->raise();
        oil->raise();
        picture->raise();
        start->raise();
        wheel_num_2->raise();
        joystick->raise();
        learn->raise();
        picture_2->raise();
        speed_n->raise();
        oil_num->raise();
        oil_n->raise();
        wheel_n->raise();
        key_board->raise();
        speed_num->raise();
        stop->raise();
        way_info->raise();
        parm = new QWidget();
        parm->setObjectName(QStringLiteral("parm"));
        hmax = new QwtSlider(parm);
        hmax->setObjectName(QStringLiteral("hmax"));
        hmax->setGeometry(QRect(90, 70, 261, 51));
        hmax->setUpperBound(360);
        hmax->setValue(360);
        hmax->setTotalSteps(360u);
        hmax->setOrientation(Qt::Horizontal);
        h_min = new QwtSlider(parm);
        h_min->setObjectName(QStringLiteral("h_min"));
        h_min->setGeometry(QRect(90, 140, 261, 51));
        h_min->setUpperBound(360);
        h_min->setTotalSteps(360u);
        h_min->setOrientation(Qt::Horizontal);
        s_max = new QwtSlider(parm);
        s_max->setObjectName(QStringLiteral("s_max"));
        s_max->setGeometry(QRect(90, 200, 261, 51));
        s_max->setUpperBound(255);
        s_max->setValue(255);
        s_max->setTotalSteps(255u);
        s_max->setOrientation(Qt::Horizontal);
        s_min = new QwtSlider(parm);
        s_min->setObjectName(QStringLiteral("s_min"));
        s_min->setGeometry(QRect(90, 260, 261, 51));
        s_min->setUpperBound(255);
        s_min->setTotalSteps(255u);
        s_min->setOrientation(Qt::Horizontal);
        v_max = new QwtSlider(parm);
        v_max->setObjectName(QStringLiteral("v_max"));
        v_max->setGeometry(QRect(90, 340, 261, 51));
        v_max->setUpperBound(255);
        v_max->setValue(255);
        v_max->setTotalSteps(255u);
        v_max->setOrientation(Qt::Horizontal);
        v_min = new QwtSlider(parm);
        v_min->setObjectName(QStringLiteral("v_min"));
        v_min->setGeometry(QRect(90, 400, 261, 51));
        v_min->setUpperBound(255);
        v_min->setTotalSteps(255u);
        v_min->setOrientation(Qt::Horizontal);
        tabWidget->addTab(parm, QString());
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 811, 31));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        wheel_n_2->setText(QApplication::translate("MainWindow", "wheel echo", 0));
        wheel_num->setText(QApplication::translate("MainWindow", "num", 0));
        picture->setText(QApplication::translate("MainWindow", "image", 0));
        start->setText(QApplication::translate("MainWindow", "start", 0));
        wheel_num_2->setText(QApplication::translate("MainWindow", "num", 0));
        joystick->setText(QApplication::translate("MainWindow", "joystick", 0));
        learn->setText(QApplication::translate("MainWindow", "learn", 0));
        picture_2->setText(QApplication::translate("MainWindow", "image", 0));
        speed_n->setText(QApplication::translate("MainWindow", "speed", 0));
        oil_num->setText(QApplication::translate("MainWindow", "num", 0));
        oil_n->setText(QApplication::translate("MainWindow", "oil", 0));
        wheel_n->setText(QApplication::translate("MainWindow", "wheel", 0));
        key_board->setText(QApplication::translate("MainWindow", "key board", 0));
        speed_num->setText(QApplication::translate("MainWindow", "num", 0));
        stop->setText(QApplication::translate("MainWindow", "stop", 0));
        tabWidget->setTabText(tabWidget->indexOf(info), QApplication::translate("MainWindow", "info", 0));
        tabWidget->setTabText(tabWidget->indexOf(parm), QApplication::translate("MainWindow", "parm", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
