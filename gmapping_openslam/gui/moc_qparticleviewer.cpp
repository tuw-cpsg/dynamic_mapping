/****************************************************************************
** GMapping::QParticleViewer meta object code from reading C++ file 'qparticleviewer.h'
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

const char *GMapping::QParticleViewer::className() const
{
    return "GMapping::QParticleViewer";
}

QMetaObject *GMapping::QParticleViewer::metaObj = 0;
static QMetaObjectCleanUp cleanUp_GMapping__QParticleViewer( "GMapping::QParticleViewer", &GMapping::QParticleViewer::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString GMapping::QParticleViewer::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "GMapping::QParticleViewer", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString GMapping::QParticleViewer::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "GMapping::QParticleViewer", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* GMapping::QParticleViewer::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "mp", &static_QUType_ptr, "MatchingParameters", QUParameter::In }
    };
    static const QUMethod slot_0 = {"setMatchingParameters", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ "mp", &static_QUType_ptr, "StartParameters", QUParameter::In }
    };
    static const QUMethod slot_1 = {"setStartParameters", 1, param_slot_1 };
    static const QUMethod slot_2 = {"start", 0, 0 };
    static const QUMethod slot_3 = {"stop", 0, 0 };
    static const QUParameter param_slot_4[] = {
	{ 0, &static_QUType_charstar, 0, QUParameter::In }
    };
    static const QUMethod slot_4 = {"loadFile", 1, param_slot_4 };
    static const QMetaData slot_tbl[] = {
	{ "setMatchingParameters(const MatchingParameters&)", &slot_0, QMetaData::Public },
	{ "setStartParameters(const StartParameters&)", &slot_1, QMetaData::Public },
	{ "start()", &slot_2, QMetaData::Public },
	{ "stop()", &slot_3, QMetaData::Public },
	{ "loadFile(const char*)", &slot_4, QMetaData::Public }
    };
    static const QUParameter param_signal_0[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod signal_0 = {"neffChanged", 1, param_signal_0 };
    static const QUParameter param_signal_1[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In },
	{ 0, &static_QUType_double, 0, QUParameter::In },
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod signal_1 = {"poseEntropyChanged", 3, param_signal_1 };
    static const QUParameter param_signal_2[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In },
	{ 0, &static_QUType_double, 0, QUParameter::In },
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod signal_2 = {"trajectoryEntropyChanged", 3, param_signal_2 };
    static const QUParameter param_signal_3[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod signal_3 = {"mapsEntropyChanged", 1, param_signal_3 };
    static const QUParameter param_signal_4[] = {
	{ 0, &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod signal_4 = {"mapsIGainChanged", 1, param_signal_4 };
    static const QMetaData signal_tbl[] = {
	{ "neffChanged(double)", &signal_0, QMetaData::Public },
	{ "poseEntropyChanged(double,double,double)", &signal_1, QMetaData::Public },
	{ "trajectoryEntropyChanged(double,double,double)", &signal_2, QMetaData::Public },
	{ "mapsEntropyChanged(double)", &signal_3, QMetaData::Public },
	{ "mapsIGainChanged(double)", &signal_4, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"GMapping::QParticleViewer", parentObject,
	slot_tbl, 5,
	signal_tbl, 5,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_GMapping__QParticleViewer.setMetaObject( metaObj );
    return metaObj;
}

void* GMapping::QParticleViewer::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "GMapping::QParticleViewer" ) )
	return this;
    return QWidget::qt_cast( clname );
}

// SIGNAL neffChanged
void GMapping::QParticleViewer::neffChanged( double t0 )
{
    activate_signal( staticMetaObject()->signalOffset() + 0, t0 );
}

#include <qobjectdefs.h>
#include <qsignalslotimp.h>

// SIGNAL poseEntropyChanged
void GMapping::QParticleViewer::poseEntropyChanged( double t0, double t1, double t2 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 1 );
    if ( !clist )
	return;
    QUObject o[4];
    static_QUType_double.set(o+1,t0);
    static_QUType_double.set(o+2,t1);
    static_QUType_double.set(o+3,t2);
    activate_signal( clist, o );
}

// SIGNAL trajectoryEntropyChanged
void GMapping::QParticleViewer::trajectoryEntropyChanged( double t0, double t1, double t2 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 2 );
    if ( !clist )
	return;
    QUObject o[4];
    static_QUType_double.set(o+1,t0);
    static_QUType_double.set(o+2,t1);
    static_QUType_double.set(o+3,t2);
    activate_signal( clist, o );
}

// SIGNAL mapsEntropyChanged
void GMapping::QParticleViewer::mapsEntropyChanged( double t0 )
{
    activate_signal( staticMetaObject()->signalOffset() + 3, t0 );
}

// SIGNAL mapsIGainChanged
void GMapping::QParticleViewer::mapsIGainChanged( double t0 )
{
    activate_signal( staticMetaObject()->signalOffset() + 4, t0 );
}

bool GMapping::QParticleViewer::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: setMatchingParameters((const MatchingParameters&)*((const MatchingParameters*)static_QUType_ptr.get(_o+1))); break;
    case 1: setStartParameters((const StartParameters&)*((const StartParameters*)static_QUType_ptr.get(_o+1))); break;
    case 2: start(); break;
    case 3: stop(); break;
    case 4: loadFile((const char*)static_QUType_charstar.get(_o+1)); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool GMapping::QParticleViewer::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: neffChanged((double)static_QUType_double.get(_o+1)); break;
    case 1: poseEntropyChanged((double)static_QUType_double.get(_o+1),(double)static_QUType_double.get(_o+2),(double)static_QUType_double.get(_o+3)); break;
    case 2: trajectoryEntropyChanged((double)static_QUType_double.get(_o+1),(double)static_QUType_double.get(_o+2),(double)static_QUType_double.get(_o+3)); break;
    case 3: mapsEntropyChanged((double)static_QUType_double.get(_o+1)); break;
    case 4: mapsIGainChanged((double)static_QUType_double.get(_o+1)); break;
    default:
	return QWidget::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool GMapping::QParticleViewer::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool GMapping::QParticleViewer::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
