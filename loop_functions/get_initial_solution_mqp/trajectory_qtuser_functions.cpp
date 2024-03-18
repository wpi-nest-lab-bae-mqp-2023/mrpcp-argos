#include "trajectory_qtuser_functions.h"

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawInWorld() {
    /* Go through all the robot waypoints and draw them */
    std::vector<std::vector<std::vector<std::vector<float>>>> path_arr;
//    std::cout << "Setting up in DrawInWorld" << std::endl;
    drawPath(path_arr);
}

void CTrajectoryQTUserFunctions::drawPath(std::vector<std::vector<std::vector<std::vector<float>>>> path_arr) {
    DrawRay(CRay3(CVector3(-1., 1., 0.01), CVector3(1., -1., 0.01)), CColor::RED, 5);

}

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() = default;


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "trajectory_qtuser_functions")
