/****************************************************************************
** Meta object code from reading C++ file 'autofit.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../include/autofit.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'autofit.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Autofit_t {
    QByteArrayData data[3];
    char stringdata0[20];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Autofit_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Autofit_t qt_meta_stringdata_Autofit = {
    {
QT_MOC_LITERAL(0, 0, 7), // "Autofit"
QT_MOC_LITERAL(1, 8, 10), // "performFit"
QT_MOC_LITERAL(2, 19, 0) // ""

    },
    "Autofit\0performFit\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Autofit[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void Autofit::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Autofit *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->performFit(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject Autofit::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Autofit.data,
    qt_meta_data_Autofit,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Autofit::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Autofit::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Autofit.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "ccStdPluginInterface"))
        return static_cast< ccStdPluginInterface*>(this);
    if (!strcmp(_clname, "cccorp.cloudcompare.ccPluginInterface/3.2"))
        return static_cast< ccPluginInterface*>(this);
    if (!strcmp(_clname, "cccorp.cloudcompare.ccStdPluginInterface/1.5"))
        return static_cast< ccStdPluginInterface*>(this);
    return QObject::qt_metacast(_clname);
}

int Autofit::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
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
    0x02,  0x74,  'c',  'c',  '.',  'p',  'l',  'u', 
    'g',  'i',  'n',  's',  '.',  's',  't',  'd', 
    'p',  'l',  'u',  'g',  'i',  'n', 
    // "className"
    0x03,  0x67,  'A',  'u',  't',  'o',  'f',  'i', 
    't', 
    // "MetaData"
    0x04,  0xa8,  0x67,  'a',  'u',  't',  'h',  'o', 
    'r',  's',  0x81,  0xa2,  0x65,  'e',  'm',  'a', 
    'i',  'l',  0x75,  'r',  'f',  '.',  'r',  'o', 
    'm',  'b',  'o',  'u',  't',  's',  '@',  'g', 
    'm',  'a',  'i',  'l',  '.',  'c',  'o',  'm', 
    0x64,  'n',  'a',  'm',  'e',  0x70,  'R',  'i', 
    'c',  'h',  'a',  'r',  'd',  ' ',  'R',  'o', 
    'm',  'b',  'o',  'u',  't',  's',  0x64,  'c', 
    'o',  'r',  'e',  0xf4,  0x6b,  'd',  'e',  's', 
    'c',  'r',  'i',  'p',  't',  'i',  'o',  'n', 
    0x76,  'V',  'i',  'n',  'd',  ' ',  'b',  'u', 
    'i',  'z',  'e',  'n',  ' ',  'e',  'n',  ' ', 
    'b',  'o',  'c',  'h',  't',  'e',  'n',  0x64, 
    'i',  'c',  'o',  'n',  0x78,  0x37,  '/',  'w', 
    'o',  'r',  'k',  'd',  'i',  'r',  '/',  'p', 
    'r',  'o',  'j',  'e',  'c',  't',  's',  '/', 
    'C',  '+',  '+',  '/',  'P',  'l',  'u',  'g', 
    'i',  'n',  'A',  'u',  't',  'o',  'F',  'i', 
    't',  '/',  'i',  'c',  'o',  'n',  's',  '/', 
    'b',  'o',  'c',  'h',  't',  'j',  'e',  '5', 
    '0',  '.',  'p',  'n',  'g',  0x6b,  'm',  'a', 
    'i',  'n',  't',  'a',  'i',  'n',  'e',  'r', 
    's',  0x81,  0xa2,  0x65,  'e',  'm',  'a',  'i', 
    'l',  0x75,  'r',  'f',  '.',  'r',  'o',  'm', 
    'b',  'o',  'u',  't',  's',  '@',  'g',  'm', 
    'a',  'i',  'l',  '.',  'c',  'o',  'm',  0x64, 
    'n',  'a',  'm',  'e',  0x70,  'R',  'i',  'c', 
    'h',  'a',  'r',  'd',  ' ',  'R',  'o',  'm', 
    'b',  'o',  'u',  't',  's',  0x64,  'n',  'a', 
    'm',  'e',  0x67,  'A',  'u',  't',  'o',  'f', 
    'i',  't',  0x6a,  'r',  'e',  'f',  'e',  'r', 
    'e',  'n',  'c',  'e',  's',  0x80,  0x64,  't', 
    'y',  'p',  'e',  0x68,  'S',  't',  'a',  'n', 
    'd',  'a',  'r',  'd', 
    0xff, 
};
QT_MOC_EXPORT_PLUGIN(Autofit, Autofit)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
