/****************************************************************************
** Meta object code from reading C++ file 'cbrobot.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cbrobot.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cbrobot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cbRobot_t {
    QByteArrayData data[8];
    char stringdata0[120];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cbRobot_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cbRobot_t qt_meta_stringdata_cbRobot = {
    {
QT_MOC_LITERAL(0, 0, 7), // "cbRobot"
QT_MOC_LITERAL(1, 8, 20), // "robCollisionsChanged"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 20), // "robReturnTimeChanged"
QT_MOC_LITERAL(4, 51, 21), // "robArrivalTimeChanged"
QT_MOC_LITERAL(5, 73, 15), // "robScoreChanged"
QT_MOC_LITERAL(6, 89, 15), // "robStateChanged"
QT_MOC_LITERAL(7, 105, 14) // "cbRobot::State"

    },
    "cbRobot\0robCollisionsChanged\0\0"
    "robReturnTimeChanged\0robArrivalTimeChanged\0"
    "robScoreChanged\0robStateChanged\0"
    "cbRobot::State"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cbRobot[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       3,    1,   47,    2, 0x06 /* Public */,
       4,    1,   50,    2, 0x06 /* Public */,
       5,    1,   53,    2, 0x06 /* Public */,
       6,    1,   56,    2, 0x06 /* Public */,
       6,    1,   59,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 7,    2,
    QMetaType::Void, QMetaType::QString,    2,

       0        // eod
};

void cbRobot::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<cbRobot *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->robCollisionsChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->robReturnTimeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->robArrivalTimeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->robScoreChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->robStateChanged((*reinterpret_cast< cbRobot::State(*)>(_a[1]))); break;
        case 5: _t->robStateChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (cbRobot::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&cbRobot::robCollisionsChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (cbRobot::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&cbRobot::robReturnTimeChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (cbRobot::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&cbRobot::robArrivalTimeChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (cbRobot::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&cbRobot::robScoreChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (cbRobot::*)(cbRobot::State );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&cbRobot::robStateChanged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (cbRobot::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&cbRobot::robStateChanged)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject cbRobot::staticMetaObject = { {
    &cbClient::staticMetaObject,
    qt_meta_stringdata_cbRobot.data,
    qt_meta_data_cbRobot,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *cbRobot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cbRobot::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cbRobot.stringdata0))
        return static_cast<void*>(this);
    return cbClient::qt_metacast(_clname);
}

int cbRobot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = cbClient::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void cbRobot::robCollisionsChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void cbRobot::robReturnTimeChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void cbRobot::robArrivalTimeChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void cbRobot::robScoreChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void cbRobot::robStateChanged(cbRobot::State _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void cbRobot::robStateChanged(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
