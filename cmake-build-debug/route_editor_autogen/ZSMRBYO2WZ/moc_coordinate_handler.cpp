/****************************************************************************
** Meta object code from reading C++ file 'coordinate_handler.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../includes/utils/coordinate_handler.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'coordinate_handler.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CoordinateHandler_t {
    QByteArrayData data[13];
    char stringdata0[160];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CoordinateHandler_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CoordinateHandler_t qt_meta_stringdata_CoordinateHandler = {
    {
QT_MOC_LITERAL(0, 0, 17), // "CoordinateHandler"
QT_MOC_LITERAL(1, 18, 12), // "onClickEvent"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 3), // "lat"
QT_MOC_LITERAL(4, 36, 3), // "lng"
QT_MOC_LITERAL(5, 40, 16), // "onNodeClickEvent"
QT_MOC_LITERAL(6, 57, 6), // "nodeId"
QT_MOC_LITERAL(7, 64, 17), // "onRightClickEvent"
QT_MOC_LITERAL(8, 82, 14), // "onDragendEvent"
QT_MOC_LITERAL(9, 97, 18), // "onMouseScrollEnded"
QT_MOC_LITERAL(10, 116, 11), // "locationStr"
QT_MOC_LITERAL(11, 128, 18), // "onWheelScrollEnded"
QT_MOC_LITERAL(12, 147, 12) // "zoomLevelStr"

    },
    "CoordinateHandler\0onClickEvent\0\0lat\0"
    "lng\0onNodeClickEvent\0nodeId\0"
    "onRightClickEvent\0onDragendEvent\0"
    "onMouseScrollEnded\0locationStr\0"
    "onWheelScrollEnded\0zoomLevelStr"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CoordinateHandler[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   44,    2, 0x0a /* Public */,
       5,    1,   49,    2, 0x0a /* Public */,
       7,    1,   52,    2, 0x0a /* Public */,
       8,    3,   55,    2, 0x0a /* Public */,
       9,    1,   62,    2, 0x0a /* Public */,
      11,    1,   65,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,    6,    3,    4,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void, QMetaType::QString,   12,

       0        // eod
};

void CoordinateHandler::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CoordinateHandler *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onClickEvent((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->onNodeClickEvent((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->onRightClickEvent((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->onDragendEvent((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 4: _t->onMouseScrollEnded((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->onWheelScrollEnded((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CoordinateHandler::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CoordinateHandler.data,
    qt_meta_data_CoordinateHandler,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CoordinateHandler::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CoordinateHandler::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CoordinateHandler.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int CoordinateHandler::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
