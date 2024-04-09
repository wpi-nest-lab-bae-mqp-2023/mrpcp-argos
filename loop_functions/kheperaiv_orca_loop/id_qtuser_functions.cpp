#include "id_qtuser_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() :
        m_cKheperaIVORCALoop(dynamic_cast<CKheperaIVORCALoop&>(CSimulator::GetInstance().GetLoopFunctions())) {
   RegisterUserFunction<CIDQTUserFunctions,CKheperaIVEntity>(&CIDQTUserFunctions::DrawID);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::DrawID(CKheperaIVEntity& c_entity) {
   DrawText(CVector3(0.0, 0.0, 0.2), c_entity.GetId());
}

void CIDQTUserFunctions::DrawInWorld() {
    /* Go through all the robot paths and draw them in the hook */
    for (int i = 0; i < m_cKheperaIVORCALoop.cKheperaIVs.size(); ++i) {
        auto cKheperaIV = m_cKheperaIVORCALoop.cKheperaIVs[i];
        auto &cController = dynamic_cast<CKheperaIVORCA &>(cKheperaIV->GetControllableEntity().GetController());
        auto curr_robot_pos = CVector3(cController.curr_pos.GetX(), cController.curr_pos.GetY(), 0.);
        auto goal_robot_pos = CVector3(cController.goal_pos.GetX(), cController.goal_pos.GetY(), 0.);
        DrawCircle(curr_robot_pos + CVector3(0., 0., 0.01), CQuaternion(), 0.1, CColor(0xFF, 0xDB, 0x66, 250));
        DrawCircle(goal_robot_pos + CVector3(0., 0., 0.01), CQuaternion(), 0.1, CColor(0xFF, 0x02, 0x9D, 250));
        DrawRay(CRay3(curr_robot_pos + CVector3(0., 0., 0.02),
                      goal_robot_pos + CVector3(0., 0., 0.02)),
                CColor(0xFF, 0xA6, 0xFE, 250), 3);
    }
}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
