//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca.h"

CKheperaIVORCA::CKheperaIVORCA() :
        maxRobotVelocity(2.),
        maxRobotOmega(10.),
        vel_kp(100.),
        vel_ki(0.),
        vel_kd(0.),
        theta_kp(20.),
        theta_ki(0.),
        theta_kd(0.1),
        vel_ctrl(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity),
        theta_ctrl(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega),
        curr_vel_x_filter(AveragingFilter(5, 0.)),
        curr_vel_y_filter(AveragingFilter(5, 0.))
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
    m_pcRNG = CRandom::CreateRNG("argos");
    m_pcRNG->SetSeed(id);
    ResetSim();
}

void CKheperaIVORCA::ResetSim() {
    /* Create a new simulator instance. */
    simulator = new RVO::RVOSimulator();
    /* Specify the global time step of the simulation. */
    simulator->setTimeStep(0.25F);
    /* Specify the default parameters for agents that are subsequently added. */
    simulator->setAgentDefaults((float)(rab_range), 10U, 5.0F, 5.0F, (float)(KHEPERAIV_BASE_RADIUS * 2.), (float)maxRobotVelocity);

    /* Add polygonal obstacles */
    for (const auto& obstacle : obstacles) {
        simulator->addObstacle(obstacle);
//        std::cout << "Adding obstacle..." << std::endl;
    }
    /* Process the obstacles so that they are accounted for in the simulation. */
    simulator->processObstacles();
}

void CKheperaIVORCA::ControlStep() {
    // Read positioning sensors
    CCI_PositioningSensor::SReading sReading = m_pcPosSens->GetReading();
    sReading.Position.ProjectOntoXY(curr_pos);
    sReading.Orientation.ToEulerAngles(yaw, temp1, temp2);

    // Calculate previous average velocity and broadcast to others
    UpdateVelocityVector(m_pcWheelsS->GetReading());
    BroadcastORCA();

    /* Reset simulation and calculate ORCA preferred velocity. */
    ResetSim();
    auto prefVelVec = goal_pos - curr_pos;
    if (prefVelVec.Length() > maxRobotVelocity) { prefVelVec = prefVelVec.Normalize() * maxRobotVelocity; }
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    for(size_t i = 0; i < tPackets.size() + 1; ++i) {
        CKheperaIVORCA::NORCAData nORCAData = i == tPackets.size() ? NORCAData(curr_pos, prefVelVec) : GetORCAData(tPackets[i]);
        simulator->addAgent(nORCAData.GetCurrPos());

        /* Perturb a little to avoid deadlocks due to perfect symmetry. */
        float angle = (float)m_pcRNG->Uniform(CRange(-M_PI, M_PI));
        float dist = (float)m_pcRNG->Uniform(CRange(0., maxRobotVelocity / 100.));
        simulator->setAgentPrefVelocity(
                i, nORCAData.GetCurrVel() +
                   dist * RVO::Vector2(std::cos(angle), std::sin(angle)));

    }
    simulator->doStep();
    orcaVec = NORCAData::RVOtoARGOSVec(simulator->getAgentVelocity(simulator->getNumAgents()-1)).Rotate(-yaw);
    ApplyORCA(orcaVec);
    std::cout << "Number of Neighbors: " << tPackets.size() << std::endl;
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
    auto curr_vel_x = curr_vel_x_filter.isFilledOnce ? curr_vel_x_filter.addAndReturnAverage(tmp.GetX()) : tmp.GetX();
    auto curr_vel_y = curr_vel_x_filter.isFilledOnce ? curr_vel_y_filter.addAndReturnAverage(tmp.GetY()) : tmp.GetY();
    curr_vel = CVector2(curr_vel_x, curr_vel_y);
}

void CKheperaIVORCA::BroadcastORCA() {
    NORCADataBytes nORCADataBytes = NORCAData(curr_pos, curr_vel).GetBytes();
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

void CKheperaIVORCA::ApplyORCA(CVector2 VelVec) {
    auto dist_err = VelVec.Length();
    auto angle_err = VelVec.Angle().GetValue();
    if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
    if(angle_err < -M_PI) { angle_err += 2 * M_PI; }
    double vel_eff = 0.; double omega_eff = 0.;
    if (angle_err > M_PI_4 && angle_err < M_PI * 3./4. || angle_err < -M_PI_4 && angle_err > -M_PI * 3./4.) {
        if (abs(angle_err) > M_PI_2) { angle_err *= -1.; }
        omega_eff = theta_ctrl.computeEffort(angle_err);
    } else {
        omega_eff = theta_ctrl.computeEffort(angle_err);
        vel_eff = vel_ctrl.computeEffort(dist_err);
        if (abs(angle_err) > M_PI_2) { omega_eff *= -1.; vel_eff *= -1.; }
    }
    ApplyTwist(vel_eff, omega_eff);
}

void CKheperaIVORCA::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheelsA->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
}


REGISTER_CONTROLLER(CKheperaIVORCA, "kheperaiv_orca_controller")

