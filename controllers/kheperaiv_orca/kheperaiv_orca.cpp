//
// Created by Yaşar İdikut on 4/1/24.
//

#include <argos3/core/utility/math/rng.h>
#include "kheperaiv_orca.h"

CKheperaIVORCA::CKheperaIVORCA() {

}

void CKheperaIVORCA::Init(TConfigurationNode& t_node) {
    m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcWheelsS = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
    m_pcWheelsA = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
}

void CKheperaIVORCA::ControlStep() {
//        auto m_pcRNG = CRandom::CreateRNG("argos");
    CCI_PositioningSensor::SReading sReading = m_pcPosSens->GetReading();
    sReading.Position.ProjectOntoXY(curr_pos);
    sReading.Orientation.ToEulerAngles(yaw, temp1, temp2);

    m_pcWheelsA->SetLinearVelocity(1., 1.);

    UpdateVelocityVector(m_pcWheelsS->GetReading());
    BroadcastVelocityVector();
//    std::cout << "curr_vel x: " << curr_vel.GetX() << " curr_vel y: " << curr_vel.GetY() <<  std::endl;

    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    n_vel_vecs.clear();
    for(size_t i = 0; i < tPackets.size(); ++i) {
        CKheperaIVORCA::NVelVec nVelVec = GetVelocityVector(tPackets[i]);
        if (!nVelVec.hasFailed) {
//            if (id == 0) { std::cout << "id:" << id << " received: " << nVelVec.vel << " from: " << nVelVec.pos << std::endl; }
            n_vel_vecs.push_back(nVelVec);
        }
    }

}


void CKheperaIVORCA::Reset(){
    // Clear data from the actuator
    m_pcRABA->ClearData();
    n_vel_vecs.clear();
//    std::vector<uint8_t> rabData(m_pcRABA->GetSize(), 0);
//    CByteArray c_data = CByteArray(rabData.data(), m_pcRABA->GetSize());

}

void CKheperaIVORCA::UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading) {

//    From FK:
//    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
//    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double left_wheel_vel = pcWheelsSReading.VelocityLeftWheel;
    double right_wheel_vel = pcWheelsSReading.VelocityRightWheel;
    double vel_magnitude = (left_wheel_vel + right_wheel_vel) / 2.;
    double vel_omega = (vel_magnitude - left_wheel_vel) / KHEPERAIV_HALF_WHEEL_DISTANCE;
    curr_vel = CVector2(vel_magnitude, CRadians(vel_omega));
    curr_vel_vec.hasFailed = false;
    curr_vel_vec.pos = curr_pos;
    curr_vel_vec.vel = curr_vel;
}


void CKheperaIVORCA::BroadcastVelocityVector() {
    CVector2 rotated_vel = CVector2(curr_vel_vec.vel.GetX(), curr_vel_vec.vel.GetY()).Rotate(yaw);
    struct NVelVecBytes nVelVecBytes = {
            false,
//            curr_pos.GetX(),
//            curr_pos.GetY(),
            rotated_vel.GetX(),
            rotated_vel.GetY(),
    };
    char* byteArray = reinterpret_cast<char*>(&nVelVecBytes);
    for(int i=0;i<sizeof nVelVecBytes;i++) {m_pcRABA->SetData(i,byteArray[i]);}
}

CKheperaIVORCA::NVelVec CKheperaIVORCA::GetVelocityVector(CCI_RangeAndBearingSensor::SPacket sPacket) {
    char byteArray[sizeof(struct NVelVecBytes)];
    for (int i = 0; i < sizeof byteArray; ++i) {
        byteArray[i] = sPacket.Data[i];
    }
    struct NVelVecBytes nVelVecBytes = *reinterpret_cast<NVelVecBytes*>(&byteArray);
    return {
        nVelVecBytes.hasFailed,
        CVector2(sPacket.Range, sPacket.HorizontalBearing) / 100.,
        CVector2(nVelVecBytes.vel_x, nVelVecBytes.vel_y).Rotate(CRadians(-yaw))
    };
}


REGISTER_CONTROLLER(CKheperaIVORCA, "kheperaiv_orca_controller")

