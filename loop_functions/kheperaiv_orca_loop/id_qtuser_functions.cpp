#include "id_qtuser_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
   RegisterUserFunction<CIDQTUserFunctions,CKheperaIVEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CKheperaIVEntity& c_entity) {
   DrawText(CVector3(0.0, 0.0, 0.2), c_entity.GetId());
    auto &cController = dynamic_cast<CKheperaIVORCA &>(c_entity.GetControllableEntity().GetController());

    if (cController.id != 0) { return; }
    CVector2 vel_vec = cController.curr_vel / 5.;
    DrawRay(CRay3(CVector3(0., 0., 0.1),
              CVector3(vel_vec.GetX(), vel_vec.GetY(), 0.1)),
        CColor(0xFF, 0xA6, 0xFE, 250), 3);

    for (int i = 0; i < cController.n_vel_vecs.size(); ++i) {
//        std::cout << "i:" << i << std::endl;
        DrawRay(CRay3(CVector3(cController.n_vel_vecs[i].pos.GetX(), cController.n_vel_vecs[i].pos.GetY(), 0.1),
                      CVector3(cController.n_vel_vecs[i].pos.GetX() + cController.n_vel_vecs[i].vel.GetX() / 5., cController.n_vel_vecs[i].pos.GetY() + cController.n_vel_vecs[i].vel.GetY() / 5., 0.1)),
                CColor(0xFF, 0xA6, 0xFE, 250), 3);
    }

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
