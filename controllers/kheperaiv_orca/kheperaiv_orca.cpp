//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca.h"

CKheperaIVORCA::CKheperaIVORCA() :
        maxRobotVelocity(2.),
        maxRobotOmega(10.),
        vel_kp(1.),
        vel_ki(0.),
        vel_kd(0.),
        theta_kp(20.),
        theta_ki(0.),
        theta_kd(0.1),
        vel_ctrl(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity),
        theta_ctrl(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega),
        curr_vel_x_filter(AveragingFilter(5, maxRobotVelocity)),
        curr_vel_y_filter(AveragingFilter(5, maxRobotVelocity))
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
    simulator->setAgentDefaults((float)KHEPERAIV_BASE_RADIUS * 10., 10U, 10.0F, 10.0F, (float)KHEPERAIV_BASE_RADIUS * 2, (float)maxRobotVelocity);
}

void CKheperaIVORCA::ControlStep() {
    // Read positioning sensors
    CCI_PositioningSensor::SReading sReading = m_pcPosSens->GetReading();
    sReading.Position.ProjectOntoXY(curr_pos);
    sReading.Orientation.ToEulerAngles(yaw, temp1, temp2);

    // Calculate previous average velocity and broadcast to others
    UpdateVelocityVector(m_pcWheelsS->GetReading());
    BroadcastORCA();

    /* Store the goals of the agents. */
    ResetSim();
    std::vector<bool> has_faileds;
    std::vector<RVO::Vector2> goal_vels;
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    for(size_t i = 0; i < tPackets.size(); ++i) {
        CKheperaIVORCA::NORCAData nORCAData = GetORCAData(tPackets[i]);
        has_faileds.push_back(nORCAData.hasFailed);
        simulator->addAgent(nORCAData.GetCurrPos());
        RVO::Vector2 goalVel = nORCAData.GetCurrVel(); // constant speed of 0.2 m/s
        goal_vels.push_back(goalVel);
    }
    simulator->addAgent(NORCAData::ARGOStoRVOVec(curr_pos));
    has_faileds.push_back(false);
    auto prefVelVec = goal_pos - curr_pos;
    if (prefVelVec.Length() > maxRobotVelocity) { prefVelVec = prefVelVec.Normalize() * maxRobotVelocity; }
    goal_vels.push_back(NORCAData::ARGOStoRVOVec(prefVelVec));
//    goal_vels.push_back(NORCAData::ARGOStoRVOVec(CVector2(prefEff.GetX(), CRadians(prefEff.GetY()))));

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
    simulator->doStep();
    orcaVec = NORCAData::RVOtoARGOSVec(simulator->getAgentVelocity(goal_vels.size()-1) * simulator->getAgentTimeHorizon(goal_vels.size()-1)).Rotate(-yaw);
    auto actEff = CalculateEffort(orcaVec);
    ApplyTwist(actEff.GetX(), actEff.GetY());
}

void CKheperaIVORCA::UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading) {

//    From FK:
//    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
//    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double left_wheel_vel = pcWheelsSReading.VelocityLeftWheel;
    double right_wheel_vel = pcWheelsSReading.VelocityRightWheel;
    double vel_magnitude = (left_wheel_vel + right_wheel_vel) / 2.;
    double vel_omega = (vel_magnitude - left_wheel_vel) / KHEPERAIV_HALF_WHEEL_DISTANCE;
    auto tmp = CVector2(vel_magnitude, CRadians(vel_omega));
    auto curr_vel_x = curr_vel_x_filter.addAndReturnAverage(tmp.GetX());
    auto curr_vel_y = curr_vel_y_filter.addAndReturnAverage(tmp.GetY());
    curr_vel = CVector2(curr_vel_x, curr_vel_y);
}

void CKheperaIVORCA::BroadcastORCA() {
    NORCADataBytes nORCADataBytes = NORCAData(false, curr_pos, curr_vel).GetBytes();
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

CVector2 CKheperaIVORCA::CalculateEffort(CVector2 VelVec) {
    auto dist_err = VelVec.Length();
    auto angle_err = VelVec.Angle().GetValue();
    double omega_eff = 0.; double vel_eff = 0.;
    if (abs(angle_err) > 0.05 && abs(angle_err) < M_PI_2) {
        omega_eff = theta_ctrl.computeEffort(angle_err);
    } else {
        omega_eff = theta_ctrl.computeEffort(angle_err);
        vel_eff = vel_ctrl.computeEffort(dist_err);
        if (abs(angle_err) > M_PI_2) { omega_eff *= -1; vel_eff *= -1; }
    }
    return {vel_eff, omega_eff};
}

void CKheperaIVORCA::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheelsA->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
}



REGISTER_CONTROLLER(CKheperaIVORCA, "kheperaiv_orca_controller")

