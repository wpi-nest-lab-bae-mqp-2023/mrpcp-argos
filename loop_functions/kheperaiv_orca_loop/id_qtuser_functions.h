#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <controllers/kheperaiv_orca/kheperaiv_orca.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include "kheperaiv_orca_loop.h"

using namespace argos;

class CIDQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CIDQTUserFunctions();

   virtual ~CIDQTUserFunctions() {}

   void DrawID(CKheperaIVEntity& c_entity);

    virtual void DrawInWorld();

    CKheperaIVORCALoop& m_cKheperaIVORCALoop;

};

#endif
