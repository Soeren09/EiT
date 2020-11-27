/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SamplePlugin_t {
    QByteArrayData data[6];
    char stringdata0[74];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 0, 12), // "SamplePlugin"
QT_MOC_LITERAL(1, 13, 10), // "btnPressed"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 20), // "stateChangedListener"
QT_MOC_LITERAL(4, 46, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(5, 68, 5) // "state"

    },
    "SamplePlugin\0btnPressed\0\0stateChangedListener\0"
    "rw::kinematics::State\0state"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x08 /* Private */,
       3,    1,   25,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    5,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SamplePlugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SamplePlugin::staticMetaObject = { {
    &rws::RobWorkStudioPlugin::staticMetaObject,
    qt_meta_stringdata_SamplePlugin.data,
    qt_meta_data_SamplePlugin,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION
static constexpr unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', '!',
    // metadata version, Qt version, architectural requirements
    0, QT_VERSION_MAJOR, QT_VERSION_MINOR, qPluginArchRequirements(),
    0xbf, 
    // "IID"
    0x02,  0x78,  0x2a,  'd',  'k',  '.',  's',  'd', 
    'u',  '.',  'm',  'i',  'p',  '.',  'R',  'o', 
    'b',  'w',  'o',  'r',  'k',  '.',  'R',  'o', 
    'b',  'W',  'o',  'r',  'k',  'S',  't',  'u', 
    'd',  'i',  'o',  'P',  'l',  'u',  'g',  'i', 
    'n',  '/',  '0',  '.',  '1', 
    // "className"
    0x03,  0x6c,  'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n', 
    // "MetaData"
    0x04,  0xa3,  0x6c,  'd',  'e',  'p',  'e',  'n', 
    'd',  'e',  'n',  'c',  'i',  'e',  's',  0x80, 
    0x64,  'n',  'a',  'm',  'e',  0x6b,  'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x67,  'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x65,  '1',  '.',  '0',  '.',  '0', 
    0xff, 
};
QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
