#include "qtuser_functions.h"
#include <loop_functions/master_loop_functions/master_loop_functions.h>
/****************************************/
/****************************************/

CQTUserFunctions::CQTUserFunctions() :
        m_cKheperaIVORCALoop(dynamic_cast<CKheperaIVORCAMQPLoop&>((dynamic_cast<CMasterLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())).GetLoopFunction("kheperaiv_orca_failure_mqp_loop")))
{ // CKheperaIVORCAMQPLoop
   RegisterUserFunction<CQTUserFunctions,CKheperaIVEntity>(&CQTUserFunctions::DrawID);
}

/****************************************/
/****************************************/

void CQTUserFunctions::DrawID(CKheperaIVEntity& c_entity) {
    auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(c_entity.GetControllableEntity().GetController());
//    auto curr_robot_pos = CVector3(cController.curr_pos.GetX(), cController.curr_pos.GetY(), 0.);
    if (cController.since_failed_counter) {
        DrawCircle(CVector3(0., 0., 0.03), CQuaternion(), KHEPERAIV_BASE_RADIUS * 1.5, colors[3]);
    } else if (cController.since_deadlock_counter) {
        DrawCircle(CVector3(0., 0., 0.03), CQuaternion(), KHEPERAIV_BASE_RADIUS * 1.5, colors[20]);
    } else {
        DrawCircle(CVector3(0., 0., 0.03), CQuaternion(), KHEPERAIV_BASE_RADIUS * 1.5, colors[60]);
    }

    auto curr_orca_vec = CVector3(cController.orcaVec.GetX(), cController.orcaVec.GetY(), 0.);
    DrawRay(CRay3(CVector3(0., 0., 0.04),
                  curr_orca_vec + CVector3(0., 0., 0.04)),
            CColor(0xFF, 0xFA, 0xFE, 250), 3);

    if (cController.path_arr.size()) { DrawText(CVector3(0.0, 0.0, 0.5), "kp" + std::to_string(cController.id)); }
}

void CQTUserFunctions::DrawInWorld() {
    /* Go through all the robot paths and draw them in the hook */
    for (int i = 0; i < m_cKheperaIVORCALoop.cKheperaIVs.size(); ++i) {
        auto cKheperaIV = m_cKheperaIVORCALoop.cKheperaIVs[i];
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
        auto curr_robot_pos = CVector3(cController.curr_pos.GetX(), cController.curr_pos.GetY(), 0.);
        auto goal_robot_pos = CVector3(cController.goal_pos.GetX(), cController.goal_pos.GetY(), 0.);
//        DrawCircle(curr_robot_pos + CVector3(0., 0., 0.01), CQuaternion(), 0.1, CColor(0xFF, 0xDB, 0x66, 250));
//        DrawCircle(goal_robot_pos + CVector3(0., 0., 0.01), CQuaternion(), 0.1, CColor(0xFF, 0x02, 0x9D, 250));
        DrawRay(CRay3(curr_robot_pos + CVector3(0., 0., 0.02),
                      goal_robot_pos + CVector3(0., 0., 0.02)),
                CColor(0xFF, 0xA6, 0xFE, 250), 3);
    }
    drawPath(m_cKheperaIVORCALoop.most_recent_path_arr);
}

void CQTUserFunctions::drawPath(std::vector<std::vector<std::vector<std::vector<double>>>> path_arr) {
    for (int ki = 0; ki < path_arr.size(); ++ki) {
        CColor color = colors[ki % 64];
        for (auto & subtouri : path_arr[ki]) {
            for (int pointi = 0; pointi < subtouri.size(); ++pointi) {
                int pointj = pointi + 1;
                if (pointj == subtouri.size()) { pointj = 0; }
                DrawCircle(CVector3(subtouri[pointi][0], subtouri[pointi][1], 0.01), CQuaternion(), 0.05, colors[9]);
                DrawRay(CRay3(CVector3(subtouri[pointi][0], subtouri[pointi][1], 0.01),
                              CVector3(subtouri[pointj][0], subtouri[pointj][1], 0.01)),
                        color, ki);
            }
        }
    }

}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CQTUserFunctions, "qtuser_functions")
