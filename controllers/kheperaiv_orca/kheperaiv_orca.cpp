//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca.h"

CKheperaIVORCA::CKheperaIVORCA() :
        maxRobotVelocity(2.5),
        maxRobotOmega(10.),
        vel_kp(1.),
        vel_ki(0.),
        vel_kd(0.),
        theta_kp(20.),
        theta_ki(0.),
        theta_kd(0.1),
        vel_ctrl(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity),
        theta_ctrl(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega)
{

}

void CKheperaIVORCA::Init(TConfigurationNode& t_node) {
    m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcWheelsS = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
    m_pcWheelsA = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");

    Reset();
}

void CKheperaIVORCA::Reset(){
    // Clear data from the actuator
    m_pcRABA->ClearData();
    ResetSim();
    m_pcRNG = CRandom::CreateRNG("argos");
//    std::vector<uint8_t> rabData(m_pcRABA->GetSize(), 0);
//    CByteArray c_data = CByteArray(rabData.data(), m_pcRABA->GetSize());

}

void CKheperaIVORCA::ResetSim() {
    simulator = new RVO::RVOSimulator();
    /* Specify the global time step of the simulation. */
    simulator->setTimeStep(0.25F);

    /* Specify the default parameters for agents that are subsequently added. */
    simulator->setAgentDefaults((float)KHEPERAIV_BASE_RADIUS * 10., 10U, 5.0F, 5.0F, (float)KHEPERAIV_BASE_RADIUS * 4, (float)maxRobotVelocity);
}

void CKheperaIVORCA::ControlStep() {
    CCI_PositioningSensor::SReading sReading = m_pcPosSens->GetReading();
    sReading.Position.ProjectOntoXY(curr_pos);
    sReading.Orientation.ToEulerAngles(yaw, temp1, temp2);

//    m_pcWheelsA->SetLinearVelocity(1., 1.);

//    UpdateVelocityVector(m_pcWheelsS->GetReading());
    BroadcastORCA();
//    std::cout << "curr_vel x: " << curr_vel.GetX() << " curr_vel y: " << curr_vel.GetY() <<  std::endl;
    /* Store the goals of the agents. */
    ResetSim();
    std::vector<bool> has_faileds;
    std::vector<RVO::Vector2> goal_vels;
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    for(size_t i = 0; i < tPackets.size(); ++i) {
        CKheperaIVORCA::NORCAData nORCAData = GetORCAData(tPackets[i]);
        has_faileds.push_back(nORCAData.hasFailed);
        simulator->addAgent(nORCAData.GetCurrPos());
        RVO::Vector2 goalVel = RVO::normalize(nORCAData.GetGoalPos() - nORCAData.GetCurrPos()) * maxRobotVelocity; // constant speed of 0.2 m/s
        goal_vels.push_back(goalVel);
    }
    simulator->addAgent(NORCAData::ARGOStoRVOVec(curr_pos));
    has_faileds.push_back(false);
    goal_vels.push_back(NORCAData::ARGOStoRVOVec(goal_pos - curr_pos));

    // Now simulate
    for(size_t i = 0; i < goal_vels.size(); ++i) {
        RVO::Vector2 goalVel = has_faileds[i] ? RVO::Vector2(0., 0.) : goal_vels[i];
        simulator->setAgentPrefVelocity(i, goalVel);

        /* Perturb a little to avoid deadlocks due to perfect symmetry. */
        float angle = static_cast<float>(m_pcRNG->Uniform(CRange(0., M_PI * 2.)));
        float dist = static_cast<float>(m_pcRNG->Uniform(CRange(0., 0.0001)));

        simulator->setAgentPrefVelocity(
                i, simulator->getAgentPrefVelocity(i) +
                   dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
    }

    orcaVec = NORCAData::RVOtoARGOSVec(simulator->getAgentVelocity(goal_vels.size()-1)).Normalize().Rotate(-yaw);
    std::cout << "curr_pos:" << curr_pos.GetX() << ", " << curr_pos.GetY() << std::endl;
    std::cout << "orcaVec:" << orcaVec.GetX() << ", " << orcaVec.GetY() << std::endl;
    simulator->doStep();
    orcaVec = NORCAData::RVOtoARGOSVec(simulator->getAgentVelocity(goal_vels.size()-1)).Normalize().Rotate(-yaw);
    std::cout << "orcaVec:" << orcaVec.GetX() << ", " << orcaVec.GetY() << std::endl;

    auto angle_err = orcaVec.Angle().GetValue();
    auto dist_err = (goal_pos - curr_pos).Length();
    if(dist_err < 0.05) {
        ApplyTwist(0., 0.);
    } else if (angle_err > 0.02) {
        double omega_eff = theta_ctrl.computeEffort(angle_err);
        ApplyTwist(0., omega_eff);
    } else {
        double omega_eff = theta_ctrl.computeEffort(angle_err);
        double vel_eff = vel_ctrl.computeEffort(maxRobotVelocity * 0.8);
        ApplyTwist(vel_eff, omega_eff);
    }
}

void CKheperaIVORCA::BroadcastORCA() {
    NORCADataBytes nORCADataBytes = NORCAData(false, curr_pos, yaw, goal_pos).GetBytes();
    char* byteArray = reinterpret_cast<char*>(&nORCADataBytes);
    for(int i=0;i<sizeof nORCADataBytes;i++) {m_pcRABA->SetData(i,byteArray[i]);}
}

CKheperaIVORCA::NORCAData CKheperaIVORCA::GetORCAData(CCI_RangeAndBearingSensor::SPacket sPacket) {
    UInt8 byteArray[sizeof(struct NORCADataBytes)];
    for (int i = 0; i < sizeof byteArray; ++i) {
        byteArray[i] = sPacket.Data[i];
    }
    struct NORCADataBytes nVelVecBytes = *reinterpret_cast<NORCADataBytes*>(&byteArray);
    return NORCAData(nVelVecBytes);
}

void CKheperaIVORCA::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheelsA->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
}



REGISTER_CONTROLLER(CKheperaIVORCA, "kheperaiv_orca_controller")

