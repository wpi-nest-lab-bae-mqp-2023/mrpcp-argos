#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <controllers/kheperaiv_orca/kheperaiv_orca.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CIDQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CIDQTUserFunctions();

   virtual ~CIDQTUserFunctions() {}

   void Draw(CKheperaIVEntity& c_entity);

};

#endif
