/****************************************************************************
** Meta object code from reading C++ file 'file_info_model.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../includes/model/file_info_model.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'file_info_model.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_FileInfoModel_t {
    QByteArrayData data[10];
    char stringdata0[111];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FileInfoModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FileInfoModel_t qt_meta_stringdata_FileInfoModel = {
    {
QT_MOC_LITERAL(0, 0, 13), // "FileInfoModel"
QT_MOC_LITERAL(1, 14, 15), // "fileInfoChanged"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 8), // "FileInfo"
QT_MOC_LITERAL(4, 40, 4), // "info"
QT_MOC_LITERAL(5, 45, 18), // "fileSavableChanged"
QT_MOC_LITERAL(6, 64, 7), // "savable"
QT_MOC_LITERAL(7, 72, 21), // "latestFilePathChanged"
QT_MOC_LITERAL(8, 94, 11), // "std::string"
QT_MOC_LITERAL(9, 106, 4) // "path"

    },
    "FileInfoModel\0fileInfoChanged\0\0FileInfo\0"
    "info\0fileSavableChanged\0savable\0"
    "latestFilePathChanged\0std::string\0"
    "path"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FileInfoModel[] = {

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
       7,    1,   35,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, 0x80000000 | 8,    9,

       0        // eod
};

void FileInfoModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<FileInfoModel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->fileInfoChanged((*reinterpret_cast< const FileInfo(*)>(_a[1]))); break;
        case 1: _t->fileSavableChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->latestFilePathChanged((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (FileInfoModel::*)(const FileInfo & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&FileInfoModel::fileInfoChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (FileInfoModel::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&FileInfoModel::fileSavableChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (FileInfoModel::*)(const std::string & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&FileInfoModel::latestFilePathChanged)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject FileInfoModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_FileInfoModel.data,
    qt_meta_data_FileInfoModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *FileInfoModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FileInfoModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_FileInfoModel.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int FileInfoModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void FileInfoModel::fileInfoChanged(const FileInfo & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void FileInfoModel::fileSavableChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void FileInfoModel::latestFilePathChanged(const std::string & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
