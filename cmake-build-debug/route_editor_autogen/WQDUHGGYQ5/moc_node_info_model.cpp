/****************************************************************************
** Meta object code from reading C++ file 'node_info_model.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../includes/model/node_info_model.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'node_info_model.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_NodeInfoModel_t {
    QByteArrayData data[11];
    char stringdata0[144];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NodeInfoModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NodeInfoModel_t qt_meta_stringdata_NodeInfoModel = {
    {
QT_MOC_LITERAL(0, 0, 13), // "NodeInfoModel"
QT_MOC_LITERAL(1, 14, 15), // "nodesMapChanged"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 30), // "QMap<std::string,QList<Node> >"
QT_MOC_LITERAL(4, 62, 8), // "nodesMap"
QT_MOC_LITERAL(5, 71, 22), // "currentNodeListChanged"
QT_MOC_LITERAL(6, 94, 11), // "QList<Node>"
QT_MOC_LITERAL(7, 106, 8), // "nodeList"
QT_MOC_LITERAL(8, 115, 18), // "currentNodeChanged"
QT_MOC_LITERAL(9, 134, 4), // "Node"
QT_MOC_LITERAL(10, 139, 4) // "node"

    },
    "NodeInfoModel\0nodesMapChanged\0\0"
    "QMap<std::string,QList<Node> >\0nodesMap\0"
    "currentNodeListChanged\0QList<Node>\0"
    "nodeList\0currentNodeChanged\0Node\0node"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NodeInfoModel[] = {

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
    QMetaType::Void, 0x80000000 | 9,   10,

       0        // eod
};

void NodeInfoModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<NodeInfoModel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->nodesMapChanged((*reinterpret_cast< const QMap<std::string,QList<Node> >(*)>(_a[1]))); break;
        case 1: _t->currentNodeListChanged((*reinterpret_cast< const QList<Node>(*)>(_a[1]))); break;
        case 2: _t->currentNodeChanged((*reinterpret_cast< const Node(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (NodeInfoModel::*)(const QMap<std::string,QList<Node>> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NodeInfoModel::nodesMapChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (NodeInfoModel::*)(const QList<Node> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NodeInfoModel::currentNodeListChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (NodeInfoModel::*)(const Node & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NodeInfoModel::currentNodeChanged)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject NodeInfoModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_NodeInfoModel.data,
    qt_meta_data_NodeInfoModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *NodeInfoModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NodeInfoModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NodeInfoModel.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int NodeInfoModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void NodeInfoModel::nodesMapChanged(const QMap<std::string,QList<Node>> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void NodeInfoModel::currentNodeListChanged(const QList<Node> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void NodeInfoModel::currentNodeChanged(const Node & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
