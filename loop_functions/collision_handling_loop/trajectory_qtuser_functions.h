#ifndef TRAJECTORY_QTUSER_FUNCTIONS_H
#define TRAJECTORY_QTUSER_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "get_initial_solution_mqp.h"

using namespace argos;

class GetInitialSolutionMQP;


class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTrajectoryQTUserFunctions();

   virtual ~CTrajectoryQTUserFunctions() {}

    virtual void DrawInWorld();

   void drawPath(std::vector<std::vector<std::vector<std::vector<float>>>> path_arr);

    GetInitialSolutionMQP& m_cTrajLF;

private:

};

#endif
