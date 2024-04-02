//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca.h"

CKheperaIVORCA::CKheperaIVORCA() {

}

void CKheperaIVORCA::Init(TConfigurationNode& t_node) {

}

void CKheperaIVORCA::ControlStep() {
    std::cout << "New Point:" << curr_pos.GetX() << "," << curr_pos.GetY() << std::endl;
}


void CKheperaIVORCA::Reset(){

}


REGISTER_CONTROLLER(CKheperaIVORCA, "kheperaiv_orca_controller")

