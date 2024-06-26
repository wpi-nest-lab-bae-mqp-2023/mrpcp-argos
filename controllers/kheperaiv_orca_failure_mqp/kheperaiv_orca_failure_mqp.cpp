//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca_failure_mqp.h"

CKheperaIVORCAFailureMQP::CKheperaIVORCAFailureMQP() :
        maxRobotVelocity(0.2),
        maxRobotOmega(M_PI / 2.),
        vel_kp(0.4),
        vel_ki(0.),
        vel_kd(0.),
        theta_kp(1.),
        theta_ki(0.),
        theta_kd(0.),
        vel_ctrl(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity, maxRobotVelocity/4.),
        theta_ctrl(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega, 0.),
        curr_vel_x_filter(AveragingFilter(10, 0.)),
        curr_vel_y_filter(AveragingFilter(10, 0.)),
        deadlock_detection_filter(AveragingFilter(100, 0.))
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
    m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");

    Reset();
}

void CKheperaIVORCAFailureMQP::Reset(){
    // Clear data from the actuator
    m_pcRABA->ClearData();
    m_pcRNG = CRandom::CreateRNG("argos");
    m_pcRNG->SetSeed(id);
    ResetSim();

    m_eState = STATE_DRIVE;
    m_eState_prev = m_eState;

    vel_ctrl.reset();
    theta_ctrl.reset();

    since_failed_counter = 0;
    is_turn_to_startup_depot = false;
    did_leave_from_startup_depot = false;

    curr_vel_x_filter.reset();
    curr_vel_y_filter.reset();
    deadlock_detection_filter.reset();
    since_deadlock_counter = 0;
    since_non_deadlock_counter = 0;

    did_relocate = true;

    if(path_arr.empty()){ return; }
    subtour_idx = 0;
    node_idx = 0;
    goal_pos.SetX(path_arr[subtour_idx][node_idx][0]);
    goal_pos.SetY(path_arr[subtour_idx][node_idx][1]);
    depot_pos.SetX(path_arr[0][0][0]);
    depot_pos.SetY(path_arr[0][0][1]);
}

void CKheperaIVORCAFailureMQP::ResetSim() {
    if (simulator != NULL) { delete simulator; }
    /* Create a new simulator instance. */
    simulator = new RVO::RVOSimulator();
    /* Specify the global time step of the simulation. */
    simulator->setTimeStep(0.1F);
    /* Specify the default parameters for agents that are subsequently added. */
    simulator->setAgentDefaults((float)(rab_range), 10U, (float)orcaTimeHorizon, (float)orcaTimeHorizon, (float)(KHEPERAIV_BASE_RADIUS * 1.5), (float)(maxRobotVelocity));

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

    if (since_failed_counter != 0 && ((int)(since_failed_counter / 10) % 2) == 0) { m_pcLEDs->SetAllColors(CColor::RED); }
    else { m_pcLEDs->SetAllColors(CColor::BLACK); }

    if(path_arr.empty()){ ApplyTwist(0., 0.); return; }

    // Failure checking
    if (m_eState == STATE_DRIVE || m_eState == STATE_DEADLOCK) {
        // Fail with a set probability rate (make sure the robot isn't in the depot)
        if (did_leave_from_startup_depot && m_pcRNG->Uniform(CRange(0., 1.)) < fr) {
            std::cout << "kp" << id << " failed!" << std::endl;
            since_failed_counter = 0;
            m_eState = STATE_FAILURE;
            did_relocate = false;
        }
    }

    // Deadlock detection
    if (m_eState != STATE_FAILURE && since_non_deadlock_counter > 5. * orcaTimeHorizon * 10. && did_leave_from_startup_depot && (m_eState == STATE_DEADLOCK || (deadlock_detection_filter.isFilledOnce && deadlock_detection_filter.getStdDev() < 0.01))) {
        if (m_eState != STATE_DEADLOCK) {
            m_eState_prev = m_eState;
            m_eState = STATE_DEADLOCK;
            std::cout << "kp" << id << " entered deadlock mode!" << std::endl;
        } else {
            if (since_deadlock_counter > 5. * orcaTimeHorizon * 10.) {
                m_eState = m_eState_prev;
                since_non_deadlock_counter = 0;
                std::cout << "kp" << id << " is back in drive mode!" << std::endl;
            }
        }
        since_deadlock_counter += 1;
    } else {
        since_deadlock_counter = 0;
        since_non_deadlock_counter += 1;
    }

    // Execute on the state machine only if there is a path for robots to follow.
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
            if (!is_turn_to_startup_depot) { ApplyTwist(0., 0.); break; }
            DriveORCA(goal_pos);
            break;
        }
        case STATE_DEADLOCK: {
            DriveORCA(curr_pos + (orcaNeighborsCentroid - curr_pos).Rotate(-yaw).Normalize() * maxRobotVelocity * orcaTimeHorizon * 5.);
            break;
        }
        case STATE_FAILURE: {
//            std::cout << "kp" << id << " since_failed_counter:" << since_failed_counter << std::endl;
            ApplyTwist(0., 0.);
            orcaVec.Set(0., 0.);
            goal_pos = curr_pos;
            since_failed_counter += 1;
            break;
        }
        default: {
            LOGERR << "We can't be here, there's a bug!" << std::endl;
        }
    }
    prev_pos = curr_pos;
}

void CKheperaIVORCAFailureMQP::UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading) {

//    From FK:
//    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
//    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double left_wheel_vel = pcWheelsSReading.VelocityLeftWheel * 100.;
    double right_wheel_vel = pcWheelsSReading.VelocityRightWheel * 100.;
    double vel_magnitude = (left_wheel_vel + right_wheel_vel) / 2.;
    double vel_omega = (vel_magnitude - left_wheel_vel) / KHEPERAIV_HALF_WHEEL_DISTANCE;

    auto tmp = (curr_pos - prev_pos) / 10.;
    auto curr_vel_x = curr_vel_x_filter.isFilledOnce ? curr_vel_x_filter.addAndReturnAverage(tmp.GetX()) : tmp.GetX();
    auto curr_vel_y = curr_vel_y_filter.isFilledOnce ? curr_vel_y_filter.addAndReturnAverage(tmp.GetY()) : tmp.GetY();
    curr_vel.Set(curr_vel_x, curr_vel_y);
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

void CKheperaIVORCAFailureMQP::DriveORCA(CVector2 orca_goal_pos) {
    /* Reset simulation and calculate ORCA preferred velocity. */
    double nodeVisitationTolerance = KHEPERAIV_BASE_RADIUS / 2.;
    ResetSim();
    auto prefVelVec = orca_goal_pos - curr_pos;
    if (prefVelVec.Length() > maxRobotVelocity) { prefVelVec = prefVelVec.Normalize() * maxRobotVelocity; }

    orcaNeighborsCentroid.Set(0., 0.);
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
        auto other_robot_to_goal_dist = (orca_goal_pos-nORCAData.curr_pos).Length();
        if (other_robot_to_goal_dist < 0.5) {
            nodeVisitationTolerance += (0.5 - other_robot_to_goal_dist) / 5.;
        }

        // Calculate neighbor centroid
        if (i != tPackets.size()) { orcaNeighborsCentroid += nORCAData.curr_pos / tPackets.size(); }
    }
    simulator->doStep();
    orcaVec = NORCAData::RVOtoARGOSVec(simulator->getAgentVelocity(simulator->getNumAgents()-1)).Rotate(-yaw);
    // If close to goal point, get new point (tolerance is increased near depot to reduce traffic near depot)
    if (dist_err < nodeVisitationTolerance) { m_eState = STATE_NEW_POINT; return; }
    if (!did_leave_from_startup_depot) { did_leave_from_startup_depot = (curr_pos.GetX() > depot_pos.GetX() && curr_pos.GetY() > depot_pos.GetY()) || (curr_pos.GetX() > depot_pos.GetX() - 0.3 && curr_pos.GetY() > depot_pos.GetY() + 0.3) || (curr_pos.GetX() > depot_pos.GetX() + 0.3 && curr_pos.GetY() > depot_pos.GetY() - 0.3); }

    // If not, apply ORCA
    ApplyORCA(orcaVec);
//    std::cout << "Number of Neighbors: " << tPackets.size() << std::endl;
    // For deadlock detection
    deadlock_detection_filter.addDatum((goal_pos - curr_pos).Length());

}

void CKheperaIVORCAFailureMQP::ApplyORCA(CVector2 VelVec) {
    bool inverse_flag = false;
    auto orca_dist_err = VelVec.Length();
    auto orca_angle_err = VelVec.Angle().GetValue();
    if(orca_angle_err >= M_PI) { orca_angle_err -= 2 * M_PI; }
    if(orca_angle_err < -M_PI) { orca_angle_err += 2 * M_PI; }
    if (abs(orca_angle_err) > M_PI_2) { orca_angle_err = VelVec.Rotate(CRadians(M_PI)).Angle().GetValue(); inverse_flag = true;}
    if(orca_angle_err >= M_PI) { orca_angle_err -= 2 * M_PI; }
    if(orca_angle_err < -M_PI) { orca_angle_err += 2 * M_PI; }
//    if (id == 0) { std::cout << "kp" << id << " angle err" << orca_angle_err << std::endl; }

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
        if (inverse_flag) { vel_eff *= -1.; }
//        vel_eff *= -1.;
        wasRotating = false;
    }
//    std::cout << "v_eff: " << v_eff << " omega_eff:" << omega_eff << std::endl;
    ApplyTwist(vel_eff, omega_eff);
}

void CKheperaIVORCAFailureMQP::ApplyTwist(double v_eff, double omega_eff) {
//    std::cout << "v_eff: " << v_eff << " omega_eff:" << omega_eff << std::endl;
    double left_wheel_vel = 100. * (v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff);
    double right_wheel_vel = 100. * (v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff);
    m_pcWheelsA->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
//    std::cout << "l: " << left_wheel_vel << " r:" << right_wheel_vel << std::endl;
}


REGISTER_CONTROLLER(CKheperaIVORCAFailureMQP, "kheperaiv_orca_failure_mqp_controller")
