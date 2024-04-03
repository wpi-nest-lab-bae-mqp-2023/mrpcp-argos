#include "trajectory_qtuser_functions.h"


CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() :
        m_cTrajLF(dynamic_cast<GetInitialSolutionMQP&>(CSimulator::GetInstance().GetLoopFunctions())) {
}


void CTrajectoryQTUserFunctions::DrawInWorld() {
    /* Go through all the robot paths and draw them in the hook */
    drawPath(m_cTrajLF.GetPath());
}

void CTrajectoryQTUserFunctions::drawPath(std::vector<std::vector<std::vector<std::vector<float>>>> path_arr) {
    for (int ki = 0; ki < path_arr.size(); ++ki) {
        CColor color = colors[ki % 64];
        for (auto & subtouri : path_arr[ki]) {
            for (int pointi = 0; pointi < subtouri.size(); ++pointi) {
                int pointj = pointi + 1;
                if (pointj == subtouri.size()) { pointj = 0; }
                DrawCircle(CVector3(subtouri[pointi][0], subtouri[pointi][1], 0.01), CQuaternion(), 0.05);
                DrawRay(CRay3(CVector3(subtouri[pointi][0], subtouri[pointi][1], 0.01),
                              CVector3(subtouri[pointj][0], subtouri[pointj][1], 0.01)),
                        color, ki);
            }
        }
    }

}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "trajectory_qtuser_functions")
