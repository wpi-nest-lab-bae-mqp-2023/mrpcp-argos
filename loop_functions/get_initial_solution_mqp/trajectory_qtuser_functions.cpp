#include "trajectory_qtuser_functions.h"

/****************************************/
/****************************************/
int alpha = 250;
// Unique colors from https://stackoverflow.com/questions/1168260/algorithm-for-generating-unique-colors
CColor colors[] = {
        CColor(0x00, 0x00, 0x00, alpha),
        CColor(0x00, 0xFF, 0x00, alpha),
        CColor(0x00, 0x00, 0xFF, alpha),
        CColor(0xFF, 0x00, 0x00, alpha),
        CColor(0x01, 0xFF, 0xFE, alpha),
        CColor(0xFF, 0xA6, 0xFE, alpha),
        CColor(0xFF, 0xDB, 0x66, alpha),
        CColor(0x00, 0x64, 0x01, alpha),
        CColor(0x01, 0x00, 0x67, alpha),
        CColor(0x95, 0x00, 0x3A, alpha),
        CColor(0x00, 0x7D, 0xB5, alpha),
        CColor(0xFF, 0x00, 0xF6, alpha),
        CColor(0xFF, 0xEE, 0xE8, alpha),
        CColor(0x77, 0x4D, 0x00, alpha),
        CColor(0x90, 0xFB, 0x92, alpha),
        CColor(0x00, 0x76, 0xFF, alpha),
        CColor(0xD5, 0xFF, 0x00, alpha),
        CColor(0xFF, 0x93, 0x7E, alpha),
        CColor(0x6A, 0x82, 0x6C, alpha),
        CColor(0xFF, 0x02, 0x9D, alpha),
        CColor(0xFE, 0x89, 0x00, alpha),
        CColor(0x7A, 0x47, 0x82, alpha),
        CColor(0x7E, 0x2D, 0xD2, alpha),
        CColor(0x85, 0xA9, 0x00, alpha),
        CColor(0xFF, 0x00, 0x56, alpha),
        CColor(0xA4, 0x24, 0x00, alpha),
        CColor(0x00, 0xAE, 0x7E, alpha),
        CColor(0x68, 0x3D, 0x3B, alpha),
        CColor(0xBD, 0xC6, 0xFF, alpha),
        CColor(0x26, 0x34, 0x00, alpha),
        CColor(0xBD, 0xD3, 0x93, alpha),
        CColor(0x00, 0xB9, 0x17, alpha),
        CColor(0x9E, 0x00, 0x8E, alpha),
        CColor(0x00, 0x15, 0x44, alpha),
        CColor(0xC2, 0x8C, 0x9F, alpha),
        CColor(0xFF, 0x74, 0xA3, alpha),
        CColor(0x01, 0xD0, 0xFF, alpha),
        CColor(0x00, 0x47, 0x54, alpha),
        CColor(0xE5, 0x6F, 0xFE, alpha),
        CColor(0x78, 0x82, 0x31, alpha),
        CColor(0x0E, 0x4C, 0xA1, alpha),
        CColor(0x91, 0xD0, 0xCB, alpha),
        CColor(0xBE, 0x99, 0x70, alpha),
        CColor(0x96, 0x8A, 0xE8, alpha),
        CColor(0xBB, 0x88, 0x00, alpha),
        CColor(0x43, 0x00, 0x2C, alpha),
        CColor(0xDE, 0xFF, 0x74, alpha),
        CColor(0x00, 0xFF, 0xC6, alpha),
        CColor(0xFF, 0xE5, 0x02, alpha),
        CColor(0x62, 0x0E, 0x00, alpha),
        CColor(0x00, 0x8F, 0x9C, alpha),
        CColor(0x98, 0xFF, 0x52, alpha),
        CColor(0x75, 0x44, 0xB1, alpha),
        CColor(0xB5, 0x00, 0xFF, alpha),
        CColor(0x00, 0xFF, 0x78, alpha),
        CColor(0xFF, 0x6E, 0x41, alpha),
        CColor(0x00, 0x5F, 0x39, alpha),
        CColor(0x6B, 0x68, 0x82, alpha),
        CColor(0x5F, 0xAD, 0x4E, alpha),
        CColor(0xA7, 0x57, 0x40, alpha),
        CColor(0xA5, 0xFF, 0xD2, alpha),
        CColor(0xFF, 0xB1, 0x67, alpha),
        CColor(0x00, 0x9B, 0xFF, alpha),
        CColor(0xE8, 0x5E, 0xBE, alpha)
};

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
