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
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"
#include "qwt_dial.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *speed_n;
    QPushButton *start;
    QCheckBox *key_board;
    QLabel *picture;
    QwtDial *oil;
    QCustomPlot *way_info;
    QLabel *joystick;
    QwtDial *speed;
    QwtDial *wheel;
    QLabel *oil_n;
    QLabel *oil_num;
    QLabel *wheel_n;
    QLabel *wheel_num;
    QLabel *speed_num;
    QwtDial *wheel_2;
    QLabel *wheel_n_2;
    QLabel *wheel_num_2;
    QPushButton *stop;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1409, 868);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        speed_n = new QLabel(centralWidget);
        speed_n->setObjectName(QStringLiteral("speed_n"));
        speed_n->setGeometry(QRect(1230, 120, 51, 21));
        speed_n->setLineWidth(2);
        speed_n->setScaledContents(true);
        speed_n->setWordWrap(false);
        start = new QPushButton(centralWidget);
        start->setObjectName(QStringLiteral("start"));
        start->setGeometry(QRect(360, 470, 99, 27));
        key_board = new QCheckBox(centralWidget);
        key_board->setObjectName(QStringLiteral("key_board"));
        key_board->setGeometry(QRect(360, 420, 97, 22));
        picture = new QLabel(centralWidget);
        picture->setObjectName(QStringLiteral("picture"));
        picture->setGeometry(QRect(10, 10, 371, 271));
        oil = new QwtDial(centralWidget);
        oil->setObjectName(QStringLiteral("oil"));
        oil->setGeometry(QRect(910, 10, 201, 191));
        oil->setLowerBound(0);
        oil->setUpperBound(5);
        oil->setScaleMaxMajor(10);
        oil->setScaleMaxMinor(5);
        oil->setTotalSteps(100u);
        oil->setReadOnly(true);
        oil->setLineWidth(4);
        way_info = new QCustomPlot(centralWidget);
        way_info->setObjectName(QStringLiteral("way_info"));
        way_info->setGeometry(QRect(410, 10, 361, 271));
        joystick = new QLabel(centralWidget);
        joystick->setObjectName(QStringLiteral("joystick"));
        joystick->setGeometry(QRect(60, 450, 250, 250));
        joystick->setAutoFillBackground(false);
        speed = new QwtDial(centralWidget);
        speed->setObjectName(QStringLiteral("speed"));
        speed->setGeometry(QRect(1180, 10, 201, 191));
        speed->setUpperBound(99);
        speed->setScaleMaxMajor(10);
        speed->setScaleMaxMinor(5);
        speed->setTotalSteps(100u);
        speed->setReadOnly(true);
        speed->setLineWidth(4);
        wheel = new QwtDial(centralWidget);
        wheel->setObjectName(QStringLiteral("wheel"));
        wheel->setGeometry(QRect(900, 270, 221, 201));
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
        oil_n = new QLabel(centralWidget);
        oil_n->setObjectName(QStringLiteral("oil_n"));
        oil_n->setGeometry(QRect(980, 120, 21, 31));
        oil_n->setLineWidth(2);
        oil_n->setScaledContents(true);
        oil_n->setWordWrap(false);
        oil_num = new QLabel(centralWidget);
        oil_num->setObjectName(QStringLiteral("oil_num"));
        oil_num->setGeometry(QRect(1020, 120, 31, 31));
        wheel_n = new QLabel(centralWidget);
        wheel_n->setObjectName(QStringLiteral("wheel_n"));
        wheel_n->setGeometry(QRect(960, 380, 41, 41));
        wheel_n->setLineWidth(2);
        wheel_n->setScaledContents(true);
        wheel_n->setWordWrap(false);
        wheel_num = new QLabel(centralWidget);
        wheel_num->setObjectName(QStringLiteral("wheel_num"));
        wheel_num->setGeometry(QRect(1020, 380, 31, 41));
        speed_num = new QLabel(centralWidget);
        speed_num->setObjectName(QStringLiteral("speed_num"));
        speed_num->setGeometry(QRect(1290, 120, 51, 21));
        speed_num->setLineWidth(2);
        speed_num->setScaledContents(true);
        speed_num->setWordWrap(false);
        wheel_2 = new QwtDial(centralWidget);
        wheel_2->setObjectName(QStringLiteral("wheel_2"));
        wheel_2->setGeometry(QRect(1170, 260, 241, 211));
        wheel_2->setFont(font);
        wheel_2->setLowerBound(-35);
        wheel_2->setUpperBound(35);
        wheel_2->setScaleMaxMajor(10);
        wheel_2->setScaleMaxMinor(5);
        wheel_2->setTotalSteps(100u);
        wheel_2->setReadOnly(true);
        wheel_2->setLineWidth(4);
        wheel_n_2 = new QLabel(centralWidget);
        wheel_n_2->setObjectName(QStringLiteral("wheel_n_2"));
        wheel_n_2->setGeometry(QRect(1230, 370, 81, 41));
        wheel_n_2->setLineWidth(2);
        wheel_n_2->setScaledContents(true);
        wheel_n_2->setWordWrap(false);
        wheel_num_2 = new QLabel(centralWidget);
        wheel_num_2->setObjectName(QStringLiteral("wheel_num_2"));
        wheel_num_2->setGeometry(QRect(1310, 370, 41, 41));
        stop = new QPushButton(centralWidget);
        stop->setObjectName(QStringLiteral("stop"));
        stop->setGeometry(QRect(480, 470, 99, 27));
        MainWindow->setCentralWidget(centralWidget);
        start->raise();
        key_board->raise();
        picture->raise();
        oil->raise();
        way_info->raise();
        joystick->raise();
        speed->raise();
        wheel->raise();
        oil_n->raise();
        oil_num->raise();
        wheel_n->raise();
        wheel_num->raise();
        speed_n->raise();
        speed_num->raise();
        wheel_2->raise();
        wheel_n_2->raise();
        wheel_num_2->raise();
        stop->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1409, 31));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        speed_n->setText(QApplication::translate("MainWindow", "speed", 0));
        start->setText(QApplication::translate("MainWindow", "start", 0));
        key_board->setText(QApplication::translate("MainWindow", "key board", 0));
        picture->setText(QApplication::translate("MainWindow", "image", 0));
        joystick->setText(QApplication::translate("MainWindow", "joystick", 0));
        oil_n->setText(QApplication::translate("MainWindow", "oil", 0));
        oil_num->setText(QApplication::translate("MainWindow", "num", 0));
        wheel_n->setText(QApplication::translate("MainWindow", "wheel", 0));
        wheel_num->setText(QApplication::translate("MainWindow", "num", 0));
        speed_num->setText(QApplication::translate("MainWindow", "num", 0));
        wheel_n_2->setText(QApplication::translate("MainWindow", "wheel echo", 0));
        wheel_num_2->setText(QApplication::translate("MainWindow", "num", 0));
        stop->setText(QApplication::translate("MainWindow", "stop", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
