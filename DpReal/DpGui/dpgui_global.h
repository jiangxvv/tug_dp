#ifndef DPGUI_GLOBAL_H
#define DPGUI_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(DPGUI_LIBRARY)
#  define DPGUISHARED_EXPORT Q_DECL_EXPORT
#else
#  define DPGUISHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // DPGUI_GLOBAL_H
