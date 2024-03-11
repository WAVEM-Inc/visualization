/****************************************************************************
** Meta object code from reading C++ file 'route_editor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../includes/ui/editor/route_editor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'route_editor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RouteEditor_t {
    QByteArrayData data[12];
    char stringdata0[198];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RouteEditor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RouteEditor_t qt_meta_stringdata_RouteEditor = {
    {
QT_MOC_LITERAL(0, 0, 11), // "RouteEditor"
QT_MOC_LITERAL(1, 12, 23), // "onAddRouteButtonClicked"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 22), // "onAddNodeButtonClicked"
QT_MOC_LITERAL(4, 60, 20), // "onFileSavableChanged"
QT_MOC_LITERAL(5, 81, 7), // "savable"
QT_MOC_LITERAL(6, 89, 20), // "onPathInfoMapChanged"
QT_MOC_LITERAL(7, 110, 29), // "QMap<std::string,std::string>"
QT_MOC_LITERAL(8, 140, 11), // "pathInfoMap"
QT_MOC_LITERAL(9, 152, 24), // "onCurrentNodeListChanged"
QT_MOC_LITERAL(10, 177, 11), // "QList<Node>"
QT_MOC_LITERAL(11, 189, 8) // "nodeList"

    },
    "RouteEditor\0onAddRouteButtonClicked\0"
    "\0onAddNodeButtonClicked\0onFileSavableChanged\0"
    "savable\0onPathInfoMapChanged\0"
    "QMap<std::string,std::string>\0pathInfoMap\0"
    "onCurrentNodeListChanged\0QList<Node>\0"
    "nodeList"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RouteEditor[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x08 /* Private */,
       3,    0,   40,    2, 0x08 /* Private */,
       4,    1,   41,    2, 0x08 /* Private */,
       6,    1,   44,    2, 0x08 /* Private */,
       9,    1,   47,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void RouteEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RouteEditor *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onAddRouteButtonClicked(); break;
        case 1: _t->onAddNodeButtonClicked(); break;
        case 2: _t->onFileSavableChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->onPathInfoMapChanged((*reinterpret_cast< const QMap<std::string,std::string>(*)>(_a[1]))); break;
        case 4: _t->onCurrentNodeListChanged((*reinterpret_cast< const QList<Node>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject RouteEditor::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_RouteEditor.data,
    qt_meta_data_RouteEditor,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *RouteEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RouteEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RouteEditor.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int RouteEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
