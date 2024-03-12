/****************************************************************************
** Meta object code from reading C++ file 'map_node_model.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../includes/model/map_node_model.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'map_node_model.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MapNodeModel_t {
    QByteArrayData data[10];
    char stringdata0[132];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MapNodeModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MapNodeModel_t qt_meta_stringdata_MapNodeModel = {
    {
QT_MOC_LITERAL(0, 0, 12), // "MapNodeModel"
QT_MOC_LITERAL(1, 13, 15), // "mapNodesChanged"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 27), // "QMap<std::string,GraphNode>"
QT_MOC_LITERAL(4, 58, 7), // "nodeMap"
QT_MOC_LITERAL(5, 66, 22), // "selectedMapNodeChanged"
QT_MOC_LITERAL(6, 89, 9), // "GraphNode"
QT_MOC_LITERAL(7, 99, 7), // "mapNode"
QT_MOC_LITERAL(8, 107, 17), // "useAddModeChanged"
QT_MOC_LITERAL(9, 125, 6) // "usable"

    },
    "MapNodeModel\0mapNodesChanged\0\0"
    "QMap<std::string,GraphNode>\0nodeMap\0"
    "selectedMapNodeChanged\0GraphNode\0"
    "mapNode\0useAddModeChanged\0usable"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MapNodeModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x06 /* Public */,
       5,    1,   32,    2, 0x06 /* Public */,
       8,    1,   35,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::Bool,    9,

       0        // eod
};

void MapNodeModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MapNodeModel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->mapNodesChanged((*reinterpret_cast< const QMap<std::string,GraphNode>(*)>(_a[1]))); break;
        case 1: _t->selectedMapNodeChanged((*reinterpret_cast< const GraphNode(*)>(_a[1]))); break;
        case 2: _t->useAddModeChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MapNodeModel::*)(const QMap<std::string,GraphNode> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MapNodeModel::mapNodesChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MapNodeModel::*)(const GraphNode & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MapNodeModel::selectedMapNodeChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MapNodeModel::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MapNodeModel::useAddModeChanged)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MapNodeModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_MapNodeModel.data,
    qt_meta_data_MapNodeModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MapNodeModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MapNodeModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MapNodeModel.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int MapNodeModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void MapNodeModel::mapNodesChanged(const QMap<std::string,GraphNode> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MapNodeModel::selectedMapNodeChanged(const GraphNode & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MapNodeModel::useAddModeChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
