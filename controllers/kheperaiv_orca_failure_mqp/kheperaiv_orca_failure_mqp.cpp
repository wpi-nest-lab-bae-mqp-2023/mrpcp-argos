//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca_failure_mqp.h"

CKheperaIVORCAFailureMQP::CKheperaIVORCAFailureMQP() :
        maxRobotVelocity(0.2),
        maxRobotOmega(M_PI / 2.),
        vel_kp(1.),
        vel_ki(0.),
        vel_kd(0.),
        theta_kp(2.),
        theta_ki(0.1),
        theta_kd(0.1),
        vel_ctrl(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity, maxRobotVelocity/4.),
        theta_ctrl(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega, 0.),
        curr_vel_x_filter(AveragingFilter(10, 0.)),
        curr_vel_y_filter(AveragingFilter(10, 0.))
{ }

void CKheperaIVORCAFailureMQP::SetPath(std::vector<std::vector<std::vector<double>>> path_arrki) {
    path_arr = std::move(path_arrki);
    Reset();
}

void CKheperaIVORCAFailureMQP::Init(TConfigurationNode& t_node) {
    m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcWheelsS = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
    m_pcWheelsA = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");

    Reset();
}

void CKheperaIVORCAFailureMQP::Reset(){
    // Clear data from the actuator
    m_pcRABA->ClearData();
    m_pcRNG = CRandom::CreateRNG("argos");
    m_pcRNG->SetSeed(id);
    ResetSim();

    if(path_arr.empty()){ return; }
    m_eState = STATE_DRIVE;
    subtour_idx = 0;
    node_idx = 0;
    goal_pos.SetX(path_arr[subtour_idx][node_idx][0]);
    goal_pos.SetY(path_arr[subtour_idx][node_idx][1]);
    depot_pos.SetX(path_arr[0][0][0]);
    depot_pos.SetY(path_arr[0][0][1]);

    vel_ctrl.reset();
    theta_ctrl.reset();

    since_failed_counter = 0;
    is_turn_to_startup_depot = false;
    did_leave_from_startup_depot = false;
}

void CKheperaIVORCAFailureMQP::ResetSim() {
    if (simulator != NULL) { delete simulator; }
    /* Create a new simulator instance. */
    simulator = new RVO::RVOSimulator();
    /* Specify the global time step of the simulation. */
    simulator->setTimeStep(0.1F);
    /* Specify the default parameters for agents that are subsequently added. */
    simulator->setAgentDefaults((float)(rab_range), 10U, 1.0F, 1.0F, (float)(KHEPERAIV_BASE_RADIUS * 1.5), (float)(maxRobotVelocity));

    /* Add polygonal obstacles */
    for (const auto& obstacle : obstacles) {
        simulator->addObstacle(obstacle);
//        std::cout << "Adding obstacle..." << std::endl;
    }
    /* Process the obstacles so that they are accounted for in the simulation. */
    simulator->processObstacles();
}

void CKheperaIVORCAFailureMQP::ControlStep() {
    // Step 0: Before you control anything, read sensors, update robot state variables, and broadcast velocity for ORCA
    // Read positioning sensors
    CCI_PositioningSensor::SReading sReading = m_pcPosSens->GetReading();
    sReading.Position.ProjectOntoXY(curr_pos);
    sReading.Orientation.ToEulerAngles(yaw, temp1, temp2);

    // Calculate previous average velocity and broadcast to others
    UpdateVelocityVector(m_pcWheelsS->GetReading());
    BroadcastORCA();

    double x_err = goal_pos.GetX() - curr_pos.GetX();
    double y_err = goal_pos.GetY() - curr_pos.GetY();
    dist_err = sqrt(pow(x_err, 2) + pow(y_err, 2));

    angle_err = atan2(y_err, x_err) - yaw.GetValue();
    if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
    if(angle_err < -M_PI) { angle_err += 2 * M_PI; }

    if(path_arr.empty()){ return; }
    // Step 1: Execute on the state machine only if there is a path for robots to follow.
    switch(m_eState) {
        case STATE_NEW_POINT: {
            node_idx += 1;
            if(node_idx==path_arr[subtour_idx].size()){
                node_idx = 0;
                subtour_idx += 1;
                if(subtour_idx==path_arr.size()){
                    subtour_idx = 0;
                }
            }

            goal_pos.SetX(path_arr[subtour_idx][node_idx][0]);
            goal_pos.SetY(path_arr[subtour_idx][node_idx][1]);
            m_eState = STATE_DRIVE;
            break;
        }
        case STATE_DRIVE: {
            if (!is_turn_to_startup_depot) { return; }
            DriveORCA();

            // Fail with a set probability rate (make sure the robot isn't in the depot)
            if (did_leave_from_startup_depot && m_pcRNG->Uniform(CRange(0., 1.)) < fr) {
                std::cout << "kp" << id << " failed!" << std::endl;
                since_failed_counter = 0;
                m_eState = STATE_FAILURE;
            }
            break;
        }
        case STATE_FAILURE: {
            ApplyTwist(0., 0.);
            orcaVec.SetX(0.); orcaVec.SetY(0.);
            goal_pos = curr_pos;
            since_failed_counter += 1;
            break;
        }
        default: {
            LOGERR << "We can't be here, there's a bug!" << std::endl;
        }
    }
}

void CKheperaIVORCAFailureMQP::UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading) {

//    From FK:
//    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
//    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double left_wheel_vel = pcWheelsSReading.VelocityLeftWheel;
    double right_wheel_vel = pcWheelsSReading.VelocityRightWheel;
    double vel_magnitude = (left_wheel_vel + right_wheel_vel) / 2.;
    double vel_omega = (vel_magnitude - left_wheel_vel) / KHEPERAIV_HALF_WHEEL_DISTANCE;
    auto tmp = CVector2(vel_magnitude, CRadians(vel_omega));
    auto curr_vel_x = curr_vel_x_filter.isFilledOnce ? curr_vel_x_filter.addAndReturnAverage(tmp.GetX()) : tmp.GetX();
    auto curr_vel_y = curr_vel_y_filter.isFilledOnce ? curr_vel_y_filter.addAndReturnAverage(tmp.GetY()) : tmp.GetY();
    curr_vel = CVector2(curr_vel_x, curr_vel_y);
}

void CKheperaIVORCAFailureMQP::BroadcastORCA() {
    if (!is_turn_to_startup_depot) { return; }
    NORCADataBytes nORCADataBytes = NORCAData(curr_pos, curr_vel).GetBytes();
    char* byteArray = reinterpret_cast<char*>(&nORCADataBytes);
    for(int i=0;i<sizeof nORCADataBytes;i++) {m_pcRABA->SetData(i,byteArray[i]);}
}

CKheperaIVORCAFailureMQP::NORCAData CKheperaIVORCAFailureMQP::GetORCAData(CCI_RangeAndBearingSensor::SPacket sPacket) {
    UInt8 byteArray[sizeof(struct NORCADataBytes)];
    for (int i = 0; i < sizeof byteArray; ++i) {
        byteArray[i] = sPacket.Data[i];
    }
    struct NORCADataBytes nVelVecBytes = *reinterpret_cast<NORCADataBytes*>(&byteArray);
    return NORCAData(nVelVecBytes);
}

void CKheperaIVORCAFailureMQP::DriveORCA() {
    /* Reset simulation and calculate ORCA preferred velocity. */
    double nodeVisitationTolerance = KHEPERAIV_BASE_RADIUS / 2.;
    ResetSim();
    auto prefVelVec = goal_pos - curr_pos;
    if (prefVelVec.Length() > maxRobotVelocity) { prefVelVec = prefVelVec.Normalize() * maxRobotVelocity; }
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    for(size_t i = 0; i < tPackets.size() + 1; ++i) {
        CKheperaIVORCAFailureMQP::NORCAData nORCAData = i == tPackets.size() ? NORCAData(curr_pos, prefVelVec) : GetORCAData(tPackets[i]);
        simulator->addAgent(nORCAData.GetCurrPos());

        /* Perturb a little to avoid deadlocks due to perfect symmetry. */
        float angle = (float)m_pcRNG->Uniform(CRange(-M_PI, M_PI));
        float dist = (float)m_pcRNG->Uniform(CRange(0., maxRobotVelocity * 0.05));
        simulator->setAgentPrefVelocity(
                i, nORCAData.GetCurrVel() +
                   dist * RVO::Vector2(std::cos(angle), std::sin(angle)));

        // Increase tolerance if necessary
        auto other_robot_to_goal_dist = (goal_pos-nORCAData.curr_pos).Length();
        if (other_robot_to_goal_dist < 0.5) {
            nodeVisitationTolerance += (0.5 - other_robot_to_goal_dist) / 5.;
        }

    }
    simulator->doStep();
    orcaVec = NORCAData::RVOtoARGOSVec(simulator->getAgentVelocity(simulator->getNumAgents()-1)).Rotate(-yaw);
    // If close to goal point, get new point (tolerance is increased near depot to reduce traffic near depot)
    if (dist_err < nodeVisitationTolerance) { m_eState = STATE_NEW_POINT; return; }
    did_leave_from_startup_depot = (curr_pos.GetX() > depot_pos.GetX() && curr_pos.GetY() > depot_pos.GetY()) || (curr_pos.GetX() > depot_pos.GetX() - 0.3 && curr_pos.GetY() > depot_pos.GetY() + 0.3) || (curr_pos.GetX() > depot_pos.GetX() + 0.3 && curr_pos.GetY() > depot_pos.GetY() - 0.3) ;
    // If not, apply ORCA
    ApplyORCA(orcaVec);
//    std::cout << "Number of Neighbors: " << tPackets.size() << std::endl;
}

void CKheperaIVORCAFailureMQP::ApplyORCA(CVector2 VelVec) {
    auto orca_dist_err = VelVec.Length();
    auto orca_angle_err = VelVec.Angle().GetValue();
    if(orca_angle_err >= M_PI) { orca_angle_err -= 2 * M_PI; }
    if(orca_angle_err < -M_PI) { orca_angle_err += 2 * M_PI; }
//    std::cout << "orca_angle_err: " << orca_angle_err << std::endl;

    double vel_eff = 0.; double omega_eff = 0.;
    double tolerance = wasRotating ? rotatingRotationTolerance : drivingRotationTolerance;
    if (((orca_angle_err > tolerance) && (orca_angle_err < M_PI - tolerance)) || ((orca_angle_err < -tolerance) && (orca_angle_err > -M_PI + tolerance))) {
        if (abs(orca_angle_err) > M_PI_2) { orca_angle_err *= -1.; }
//        std::cout << "in place rotation: " << std::endl;

        omega_eff = theta_ctrl.computeEffort(orca_angle_err);
        wasRotating = true;
    } else {
        omega_eff = theta_ctrl.computeEffort(orca_angle_err);
        vel_eff = vel_ctrl.computeEffort(orca_dist_err);
        if (abs(orca_angle_err) > M_PI_2) { omega_eff *= -1.; vel_eff *= -1.; }
        wasRotating = false;
    }
    ApplyTwist(vel_eff, omega_eff);
}

void CKheperaIVORCAFailureMQP::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheelsA->SetLinearVelocity(left_wheel_vel * 100., right_wheel_vel * 100.);
}


REGISTER_CONTROLLER(CKheperaIVORCAFailureMQP, "kheperaiv_orca_failure_mqp_controller")

