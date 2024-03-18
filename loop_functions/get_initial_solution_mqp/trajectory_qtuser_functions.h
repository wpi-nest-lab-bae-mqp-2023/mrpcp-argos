#ifndef TRAJECTORY_QTUSER_FUNCTIONS_H
#define TRAJECTORY_QTUSER_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTrajectoryQTUserFunctions();

   virtual ~CTrajectoryQTUserFunctions() {}

    virtual void DrawInWorld();

   void drawPath(std::vector<std::vector<std::vector<std::vector<float>>>> path_arr);

private:

};

#endif
