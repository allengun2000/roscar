/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../interface/src/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[17];
    char stringdata0[246];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 16), // "on_start_clicked"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 8), // "ShowData"
QT_MOC_LITERAL(4, 38, 8), // "callback"
QT_MOC_LITERAL(5, 47, 26), // "std_msgs::String::ConstPtr"
QT_MOC_LITERAL(6, 74, 3), // "msg"
QT_MOC_LITERAL(7, 78, 9), // "callback1"
QT_MOC_LITERAL(8, 88, 35), // "geometry_msgs::Quaternion::Co..."
QT_MOC_LITERAL(9, 124, 9), // "callback2"
QT_MOC_LITERAL(10, 134, 27), // "std_msgs::Float32::ConstPtr"
QT_MOC_LITERAL(11, 162, 13), // "imageCallback"
QT_MOC_LITERAL(12, 176, 26), // "sensor_msgs::ImageConstPtr"
QT_MOC_LITERAL(13, 203, 14), // "cvMatToQPixmap"
QT_MOC_LITERAL(14, 218, 7), // "cv::Mat"
QT_MOC_LITERAL(15, 226, 5), // "inMat"
QT_MOC_LITERAL(16, 232, 13) // "cvMatToQImage"

    },
    "MainWindow\0on_start_clicked\0\0ShowData\0"
    "callback\0std_msgs::String::ConstPtr\0"
    "msg\0callback1\0geometry_msgs::Quaternion::ConstPtr\0"
    "callback2\0std_msgs::Float32::ConstPtr\0"
    "imageCallback\0sensor_msgs::ImageConstPtr\0"
    "cvMatToQPixmap\0cv::Mat\0inMat\0cvMatToQImage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    1,   56,    2, 0x08 /* Private */,
       7,    1,   59,    2, 0x08 /* Private */,
       9,    1,   62,    2, 0x08 /* Private */,
      11,    1,   65,    2, 0x08 /* Private */,
      13,    1,   68,    2, 0x08 /* Private */,
      16,    1,   71,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 8,    6,
    QMetaType::Void, 0x80000000 | 10,    6,
    QMetaType::Void, 0x80000000 | 12,    6,
    QMetaType::QPixmap, 0x80000000 | 14,   15,
    QMetaType::QImage, 0x80000000 | 14,   15,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_start_clicked(); break;
        case 1: _t->ShowData(); break;
        case 2: _t->callback((*reinterpret_cast< const std_msgs::String::ConstPtr(*)>(_a[1]))); break;
        case 3: _t->callback1((*reinterpret_cast< const geometry_msgs::Quaternion::ConstPtr(*)>(_a[1]))); break;
        case 4: _t->callback2((*reinterpret_cast< const std_msgs::Float32::ConstPtr(*)>(_a[1]))); break;
        case 5: _t->imageCallback((*reinterpret_cast< const sensor_msgs::ImageConstPtr(*)>(_a[1]))); break;
        case 6: { QPixmap _r = _t->cvMatToQPixmap((*reinterpret_cast< const cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QPixmap*>(_a[0]) = _r; }  break;
        case 7: { QImage _r = _t->cvMatToQImage((*reinterpret_cast< const cv::Mat(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QImage*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
