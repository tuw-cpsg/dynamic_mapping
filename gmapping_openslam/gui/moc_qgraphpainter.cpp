/****************************************************************************
** QGraphPainter meta object code from reading C++ file 'qgraphpainter.h'
**
** Created: Tue Jul 30 11:22:49 2013
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *QGraphPainter::className() const
{
    return "QGraphPainter";
}

QMetaObject *QGraphPainter::metaObj = 0;
static QMetaObjectCleanUp cleanUp_QGraphPainter( "QGraphPainter", &QGraphPainter::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString QGraphPainter::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "QGraphPainter", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString QGraphPainter::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "QGraphPainter", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* QGraphPainter::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUMethod slot_0 = {"clear", 0, 0 };
    static const QUParameter param_slot_1[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod slot_1 = {"valueAdded", 1, param_slot_1 };
    static const QUParameter param_slot_2[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In },
	{ 0, &static_QUType_double, 0, QUParameter::In },
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod slot_2 = {"valueAdded", 3, param_slot_2 };
    static const QUParameter param_slot_3[] = {
	{ "y", &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod slot_3 = {"setYReference", 1, param_slot_3 };
    static const QUMethod slot_4 = {"disableYReference", 0, 0 };
    static const QUParameter param_slot_5[] = {
	{ "min", &static_QUType_double, 0, QUParameter::In },
	{ "max", &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod slot_5 = {"setRange", 2, param_slot_5 };
    static const QUParameter param_slot_6[] = {
	{ "period", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_6 = {"start", 1, param_slot_6 };
    static const QUParameter param_slot_7[] = {
	{ "title", &static_QUType_charstar, 0, QUParameter::In }
    };
    static const QUMethod slot_7 = {"setTitle", 1, param_slot_7 };
    static const QUParameter param_slot_8[] = {
	{ "a", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_8 = {"setAutoscale", 1, param_slot_8 };
    static const QUParameter param_slot_9[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::Out }
    };
    static const QUMethod slot_9 = {"getAutoscale", 1, param_slot_9 };
    static const QMetaData slot_tbl[] = {
	{ "clear()", &slot_0, QMetaData::Public },
	{ "valueAdded(double)", &slot_1, QMetaData::Public },
	{ "valueAdded(double,double,double)", &slot_2, QMetaData::Public },
	{ "setYReference(double)", &slot_3, QMetaData::Public },
	{ "disableYReference()", &slot_4, QMetaData::Public },
	{ "setRange(double,double)", &slot_5, QMetaData::Public },
	{ "start(int)", &slot_6, QMetaData::Public },
	{ "setTitle(const char*)", &slot_7, QMetaData::Public },
	{ "setAutoscale(bool)", &slot_8, QMetaData::Public },
	{ "getAutoscale()", &slot_9, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"QGraphPainter", parentObject,
	slot_tbl, 10,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_QGraphPainter.setMetaObject( metaObj );
    return metaObj;
}

void* QGraphPainter::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "QGraphPainter" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool QGraphPainter::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: clear(); break;
    case 1: valueAdded((double)static_QUType_double.get(_o+1)); break;
    case 2: valueAdded((double)static_QUType_double.get(_o+1),(double)static_QUType_double.get(_o+2),(double)static_QUType_double.get(_o+3)); break;
    case 3: setYReference((double)static_QUType_double.get(_o+1)); break;
    case 4: disableYReference(); break;
    case 5: setRange((double)static_QUType_double.get(_o+1),(double)static_QUType_double.get(_o+2)); break;
    case 6: start((int)static_QUType_int.get(_o+1)); break;
    case 7: setTitle((const char*)static_QUType_charstar.get(_o+1)); break;
    case 8: setAutoscale((bool)static_QUType_bool.get(_o+1)); break;
    case 9: static_QUType_bool.set(_o,getAutoscale()); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool QGraphPainter::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool QGraphPainter::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool QGraphPainter::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
