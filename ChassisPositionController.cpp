#include <cmath>
#include <thread>
#include <boost/math/special_functions/sign.hpp>
#include "log.h"
#include "ChassisPositionController.h"


#define DIFF_LINE_UPPER_BOUND 0.07
#define DIFF_LINE_LOWER_BOUND 0.015
#define MINIMUM_DIFF_TURN 0.7
#define ANGLE_TOLERANCE 0.17453292519943295 // 10 degree


ChassisPositionController::ERROR
ChassisPositionController::Execute_ClearAlarmAction(void)
{
    std::shared_ptr<MotorCommand> motor_command;
    switch (clear_alarm_action->state)
    {
        case CLEAR_ALARM_ACTION_STATE::START:
            if (!clear_alarm_action->entered)
            {
                KUBOT_LOG(info) << controller_name << " ClearAlarmAction: START!";
                clear_alarm_action->entered = true;
            }
            else
            {
                clear_alarm_action->state = CLEAR_ALARM_ACTION_STATE::PREPARE;
                clear_alarm_action->entered = false;
            }
            break;

        case CLEAR_ALARM_ACTION_STATE::PREPARE:
            if (!clear_alarm_action->entered)
            {
                KUBOT_LOG(info) << controller_name << " ClearAlarmAction: PREPARE!";
                clear_alarm_action->entered = true;
            }
            else
            {
                /* disable servo at first */
                motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                {
                    return ERROR::MOTOR_COMMAND_ERROR;
                }
                motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                {
                    return ERROR::MOTOR_COMMAND_ERROR;
                }

                clear_alarm_action->state = CLEAR_ALARM_ACTION_STATE::CHECK;
                clear_alarm_action->entered = false;
            }
            break;

        case CLEAR_ALARM_ACTION_STATE::CHECK:
            if (!clear_alarm_action->entered)
            {
                KUBOT_LOG(info) << controller_name << " ClearAlarmAction: CHECK!";
                clear_alarm_action->entered = true;
            }
            else
            /* check if alarm state is normal */
            if (l_cmd_get_alarm_code->alarm_code == 0
                &&
                r_cmd_get_alarm_code->alarm_code == 0)
            {
                /* repeat disable servo at last */
                motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                {
                    return ERROR::MOTOR_COMMAND_ERROR;
                }
                motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                {
                    return ERROR::MOTOR_COMMAND_ERROR;
                }

                clear_alarm_action->state = CLEAR_ALARM_ACTION_STATE::END;
                clear_alarm_action->entered = false;
            }
            else
            /* check if alarm can be cleared */
            if (l_cmd_get_alarm_code->alarm_type == GOMOTOR_ALARM_TYPE::FAULT
                ||
                r_cmd_get_alarm_code->alarm_type == GOMOTOR_ALARM_TYPE::FAULT)
            {
                return ERROR::MOTOR_REPORT_ALARM;
            }
            else
                /* check if reach the max try limit */
            if (++clear_alarm_action->try_count > 2)
            {
                return ERROR::MOTOR_REPORT_ALARM;
            }
            else
            {
                clear_alarm_action->state = CLEAR_ALARM_ACTION_STATE::CLEAR;
                clear_alarm_action->entered = false;
            }
            break;

        case CLEAR_ALARM_ACTION_STATE::CLEAR:
            if (!clear_alarm_action->entered)
            {
                KUBOT_LOG(info) << controller_name << " ClearAlarmAction: CLEAR!";
                clear_alarm_action->entered = true;
            }
            else
            {
                /* reset alarm state */
                if (l_cmd_get_alarm_code->alarm_code != 0)
                {
                    motor_command = m_l_motor_driver->Execute<MotorCommand_ResetAlarmState>(FT_ARGUMENTS(2, 100));
                    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                    {
                        return ERROR::MOTOR_COMMAND_ERROR;
                    }
                }
                if (r_cmd_get_alarm_code->alarm_code != 0)
                {
                    motor_command = m_r_motor_driver->Execute<MotorCommand_ResetAlarmState>(FT_ARGUMENTS(2, 100));
                    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                    {
                        return ERROR::MOTOR_COMMAND_ERROR;
                    }
                }
                clear_alarm_action->delay_start_time = std::chrono::steady_clock::now();

                clear_alarm_action->state = CLEAR_ALARM_ACTION_STATE::CONFIRM;
                clear_alarm_action->entered = false;
            }
            break;

        case CLEAR_ALARM_ACTION_STATE::CONFIRM:
            if (!clear_alarm_action->entered)
            {
                KUBOT_LOG(info) << controller_name << " ClearAlarmAction: CONFIRM!";
                clear_alarm_action->entered = true;
            }
            else
            if (std::chrono::steady_clock::now() >= clear_alarm_action->delay_start_time + std::chrono::milliseconds(1000))
            {
                clear_alarm_action->state = CLEAR_ALARM_ACTION_STATE::CHECK;
                clear_alarm_action->entered = false;
            }
            else
            {
                return ERROR::NEED_RETRY_LATER;
            }
            break;

        case CLEAR_ALARM_ACTION_STATE::END:
            if (!clear_alarm_action->entered)
            {
                KUBOT_LOG(info) << controller_name << " ClearAlarmAction: END!";
                clear_alarm_action->entered = true;
            }
            else
            {
                return ERROR::OK;
            }
            break;
    }

    return ERROR::NEED_RETRY_NOW;
}


ChassisPositionController::ERROR
ChassisPositionController::Execute_GetMotorStatus(void)
{
    if (m_error.count(ERROR::MOTOR_COMMAND_ERROR) != 0)
    {
        return ERROR::NEED_RETRY_LATER;
    }

    m_l_motor_driver->Execute(FT_ARGUMENTS(2, 100), std::list<std::shared_ptr<MotorCommand>>({
                                                                                                   (l_cmd_get_alarm_code         = std::make_shared<MotorCommand_GetAlarmCode>()),
                                                                                                   //(l_cmd_get_power_voltage      = std::make_shared<MotorCommand_GetPowerVoltage>()),
                                                                                                   (l_cmd_get_loading_rates      = std::make_shared<MotorCommand_GetLoadingRates>()),
                                                                                                   (l_cmd_get_driver_utilization = std::make_shared<MotorCommand_GetDriverUtilization>()),
                                                                                                   (l_cmd_get_current_speed      = std::make_shared<MotorCommand_GetCurrentSpeed>())
                                                                                           }));
    m_r_motor_driver->Execute(FT_ARGUMENTS(2, 100), std::list<std::shared_ptr<MotorCommand>>({
                                                                                                   (r_cmd_get_alarm_code         = std::make_shared<MotorCommand_GetAlarmCode>()),
                                                                                                   //(r_cmd_get_power_voltage      = std::make_shared<MotorCommand_GetPowerVoltage>()),
                                                                                                   (r_cmd_get_loading_rates      = std::make_shared<MotorCommand_GetLoadingRates>()),
                                                                                                   (r_cmd_get_driver_utilization = std::make_shared<MotorCommand_GetDriverUtilization>()),
                                                                                                   (r_cmd_get_current_speed      = std::make_shared<MotorCommand_GetCurrentSpeed>())
                                                                                           }));
    if (l_cmd_get_alarm_code->error_stat         != MOTOR_CMD_STATUS::OK ||
        //l_cmd_get_power_voltage->error_stat      != MOTOR_CMD_STATUS::OK ||
        l_cmd_get_loading_rates->error_stat      != MOTOR_CMD_STATUS::OK ||
        l_cmd_get_driver_utilization->error_stat != MOTOR_CMD_STATUS::OK ||
        l_cmd_get_current_speed->error_stat      != MOTOR_CMD_STATUS::OK ||
        r_cmd_get_alarm_code->error_stat         != MOTOR_CMD_STATUS::OK ||
        //r_cmd_get_power_voltage->error_stat      != MOTOR_CMD_STATUS::OK ||
        r_cmd_get_loading_rates->error_stat      != MOTOR_CMD_STATUS::OK ||
        r_cmd_get_driver_utilization->error_stat != MOTOR_CMD_STATUS::OK ||
        r_cmd_get_current_speed->error_stat      != MOTOR_CMD_STATUS::OK)
    {
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    KUBOT_LOG(trace) << "CHASSIS MOTOR:" /*<< " voltage=(" << l_cmd_get_power_voltage->voltage << "," << r_cmd_get_power_voltage->voltage << ")"*/
                                         << " loading=(" << l_cmd_get_loading_rates->rates << "," << r_cmd_get_loading_rates->rates << ")"
                                         << " utilization=(" << l_cmd_get_driver_utilization->rates << "," << r_cmd_get_driver_utilization->rates << ")";

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_GetLocalizatorStatus(void)
{
    if (m_error.count(ERROR::LOCALIZATOR_REPORT_ERROR) != 0)
    {
        return ERROR::NEED_RETRY_LATER;
    }

    localizator_status      = m_localizator->GetStatus();
    if (localizator_status == nullptr)
    {
        return ERROR::NEED_RETRY_LATER;
    }
    else
    if (localizator_status->error_stat != EKF_Estimator::ERROR::OK)
    {
        return ERROR::LOCALIZATOR_REPORT_ERROR;
    }

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_GetObstacleStatus(void)
{
    if (m_error.count(ERROR::SENSOR_COMMAND_ERROR) != 0)
    {
        return ERROR::NEED_RETRY_LATER;
    }

    m_obstacle_mtx.lock();
    f_obstacle_state = m_f_obstacle_state;
    b_obstacle_state = m_b_obstacle_state;
    m_obstacle_mtx.unlock();
    if (m_f_obstacle_state == nullptr
        ||
        m_b_obstacle_state == nullptr)
    {
        return ERROR::NEED_RETRY_LATER;
    }
    else
    if (m_f_obstacle_state->error_stat != OBSTACLEAVOIDANCESENSOR_ERROR::OK
        ||
        m_b_obstacle_state->error_stat != OBSTACLEAVOIDANCESENSOR_ERROR::OK)
    {
        return ERROR::SENSOR_COMMAND_ERROR;
    }

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_StartHeartbeat(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_error.count(ERROR::MOTOR_COMMAND_ERROR) != 0)
    {
        return ERROR::NEED_RETRY_LATER;
    }

    if (m_heartbeat)
    {
        motor_command = m_l_motor_driver->Execute<MotorCommand_SetHeartbeatEnable>(FT_ARGUMENTS(2, 100), true);
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            return ERROR::MOTOR_COMMAND_ERROR;
        }
        motor_command = m_r_motor_driver->Execute<MotorCommand_SetHeartbeatEnable>(FT_ARGUMENTS(2, 100), true);
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            return ERROR::MOTOR_COMMAND_ERROR;
        }

        motor_command = m_l_motor_driver->Execute<MotorCommand_KeepHeartbeatAlive>(FT_ARGUMENTS(2, 100));
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            return ERROR::MOTOR_COMMAND_ERROR;
        }
        motor_command = m_r_motor_driver->Execute<MotorCommand_KeepHeartbeatAlive>(FT_ARGUMENTS(2, 100));
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            return ERROR::MOTOR_COMMAND_ERROR;
        }

        last_heartbeat_time = std::chrono::steady_clock::now();
    }

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_KeepHeartbeat(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_error.count(ERROR::MOTOR_COMMAND_ERROR) != 0)
    {
        return ERROR::NEED_RETRY_LATER;
    }

    if (m_heartbeat && std::chrono::steady_clock::now() >= last_heartbeat_time + std::chrono::milliseconds(90))
    {
        motor_command = m_l_motor_driver->Execute<MotorCommand_KeepHeartbeatAlive>(FT_ARGUMENTS(2, 100));
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            return ERROR::MOTOR_COMMAND_ERROR;
        }
        motor_command = m_r_motor_driver->Execute<MotorCommand_KeepHeartbeatAlive>(FT_ARGUMENTS(2, 100));
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            return ERROR::MOTOR_COMMAND_ERROR;
        }

        last_heartbeat_time = std::chrono::steady_clock::now();
    }

    return ERROR::OK;
}


ChassisPositionController::ERROR
ChassisPositionController::Execute_ResetAction(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    /* check if alarm state is abnormal */
    if (l_cmd_get_alarm_code->alarm_code != 0 || r_cmd_get_alarm_code->alarm_code != 0)
    {
        return ERROR::MOTOR_REPORT_ALARM;
    }

    switch (reset_action->state)
    {
    case RESET_ACTION_STATE::START:
        if (!reset_action->entered)
        {
            KUBOT_LOG(info) << controller_name << " ResetAction: START!";
            reset_action->entered = true;
        }
        else
        {
            reset_action->state = RESET_ACTION_STATE::WAIT_INITIAL_POSITION;
            reset_action->entered = false;
        }
        break;
    case RESET_ACTION_STATE::WAIT_INITIAL_POSITION:
        if (!reset_action->entered)
        {
            KUBOT_LOG(info) << controller_name << " ResetAction: WAIT_INITIAL_POSITION!";
            reset_action->entered = true;
        }
        else
        {
            /* check if initial position available */
            if (localizator_status == nullptr)
            {
                return ERROR::NEED_RETRY_LATER;
            }
            if (f_obstacle_state == nullptr
                ||
                b_obstacle_state == nullptr)
            {
                return ERROR::NEED_RETRY_LATER;
            }

            /* enable servo */
            motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), true);
            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
            {
                return ERROR::MOTOR_COMMAND_ERROR;
            }

            motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), true);
            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
            {
                return ERROR::MOTOR_COMMAND_ERROR;
            }

            reset_action->state = RESET_ACTION_STATE::END;
            reset_action->entered = false;
        }
        break;
    case RESET_ACTION_STATE::END:
        if (!reset_action->entered)
        {
            KUBOT_LOG(info) << controller_name << " ResetAction: END!";
            reset_action->entered = true;
        }
        else
        {
            return ERROR::OK;
        }
        break;
    }

    return ERROR::NEED_RETRY_NOW;
}


void ChassisPositionController::setNormalAcc(int boost_v, int boost_w)
{
    max_v = Config::Chassis.MAX_LINEAR_VEL;
    max_acc_v = Config::Chassis.MAX_LINEAR_ACC;
    max_dec_v = Config::Chassis.MAX_LINEAR_DEACC;

    max_w = Config::Chassis.MAX_ANGULAR_VEL;
    max_acc_w = Config::Chassis.MAX_TURN_ACC;
    max_dec_w = Config::Chassis.MAX_TURN_DEACC;

    /* linear acc setting */
    if (fabsf(std::get<0>(cur_twist)) < 0.01)
        max_acc_v *= boost_v;
    else
    {
        max_acc_v = m_acc_init - m_acc_increment * (fabsf(std::get<0>(cur_twist)) / max_v);

#if 0
        if (l_cmd_get_driver_utilization->rates * 0.001 > 0.60)
        {
            max_acc_v *= 1 - ((l_cmd_get_driver_utilization->rates * 0.001) - 0.60)/0.15;
        }
#endif
    }
    max_dec_v = static_cast<float>(m_decc_init + m_decc_increment * pow(2.0 * (max_v / (max_v + fabsf(std::get<0>(cur_twist)))), 1.5));

    /* angular acc setting */
    if (fabsf(std::get<1>(cur_twist)) < 0.08)
        max_acc_w *= boost_w;
    else
        max_acc_w = m_ang_acc_init - m_ang_acc_increment * (fabsf(std::get<1>(cur_twist))/max_w);
    max_dec_w = m_ang_decc_init;
}

void ChassisPositionController::setObstacleAvoidanceAcc(OBSTACLE_STATE obstacle, double front_dist)
{
    const double obstacle_avoidance_acc = 1.11111;

    switch (obstacle)
    {
    case OBSTACLE_STATE::noDetecion:
        break;
    case OBSTACLE_STATE::farDetection:
        if (fabs(front_dist) > 3.5)
        {
            max_v = 0;
            max_acc_v = obstacle_avoidance_acc * 0.5;
            max_dec_v = obstacle_avoidance_acc * 0.5;
        }
        break;
    case OBSTACLE_STATE::midDetection:
        if (fabs(front_dist) > 2.3)
        {
            max_v = 0;
            max_acc_v = obstacle_avoidance_acc * 0.72;
            max_dec_v = obstacle_avoidance_acc * 0.72;
        }
        break;
    case OBSTACLE_STATE::shortDetection:
        if (fabs(front_dist) > 0.8)
        {
            max_v = 0;
            max_acc_v = obstacle_avoidance_acc * 1.08;
            max_dec_v = obstacle_avoidance_acc * 1.08;
        }
        break;
    case OBSTACLE_STATE::error:
        {
            max_v = 0;
            max_acc_v = obstacle_avoidance_acc * 1.08;
            max_dec_v = obstacle_avoidance_acc * 1.08;
        }
        break;
    }
}

void ChassisPositionController::limitLinearTrapezoid(void)
{
    double& cur_v = std::get<0>(cur_twist);
    double& tar_v = std::get<0>(tar_twist);

    if(tar_v >= 0)
    {
        /* acceleration case when v > 0 , use min to limit acceleration */
        if (tar_v >= cur_v)
        {
            tar_v = std::min((cur_v + max_acc_v * m_sampling_time), tar_v);
        }
        /* deceleration case when v > 0 , use max to limit acceleration */
        else
        {
            tar_v = std::max((cur_v - max_dec_v * m_sampling_time), tar_v);
        }
    }
    else
    {
        /* acceleration case when v < 0 , use max to limit acceleration */
        if (tar_v <= cur_v)
        {
            tar_v = std::max((cur_v - max_acc_v * m_sampling_time), tar_v);
        }
        /* deceleration case when v < 0 , use min to limit acceleration */
        else
        {
            tar_v = std::min((cur_v + max_dec_v * m_sampling_time), tar_v);
        }
    }
}

void ChassisPositionController::limitAngularTrapezoid(void)
{
    double& cur_w = std::get<1>(cur_twist);
    double& tar_w = std::get<1>(tar_twist);

    if(tar_w >= 0)
    {
        /* acceleration case when w > 0 , use min to limit acceleration */
        if (tar_w >= cur_w)
            tar_w = std::min((cur_w + max_acc_w * m_sampling_time), tar_w);
        /* deacceleration case when w > 0 , use max to limit acceleration */
        else
            tar_w = std::max((cur_w - max_dec_w * m_sampling_time), tar_w);
    }
    else
    {
        /* acceleration case when w < 0 , use max to limit acceleration */
        if (tar_w <= cur_w)
            tar_w = std::max((cur_w - max_acc_w * m_sampling_time), tar_w);
        /* deacceleration case when w < 0 , use min to limit acceleration */
        else
            tar_w = std::min((cur_w + max_dec_w * m_sampling_time), tar_w);
    }
}

void ChassisPositionController::setMaximumTwist(void)
{
    double& v = std::get<0>(tar_twist);
    double& w = std::get<1>(tar_twist);

    v = (fabs(v) > Config::Chassis.MAX_LINEAR_VEL) ? (boost::math::sign(v) * Config::Chassis.MAX_LINEAR_VEL) : v;
    w = (fabs(w) > Config::Chassis.MAX_ANGULAR_VEL) ? (boost::math::sign(w) * Config::Chassis.MAX_ANGULAR_VEL) : w;
}

int ChassisPositionController::setTargetTwist(void)
{
    double r_speed = std::get<0>(tar_twist) + std::get<1>(tar_twist) * Config::Chassis.BASELINE_LENGTH / 2;
    double l_speed = std::get<0>(tar_twist) - std::get<1>(tar_twist) * Config::Chassis.BASELINE_LENGTH / 2;

    auto l_cmd_set_target_speed = m_l_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(l_speed, m_l_tick_ratio));
    if (l_cmd_set_target_speed->error_stat != MOTOR_CMD_STATUS::OK)
    {
        return -1;
    }

    auto r_cmd_set_target_speed = m_r_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(r_speed, m_r_tick_ratio));
    if (r_cmd_set_target_speed->error_stat != MOTOR_CMD_STATUS::OK)
    {
        return -1;
    }

    if (m_state == STATE::MOVE || m_state == STATE::TUNE)
    {
        Position2D dif_pos = {cur_pos->x() - tar_pos->x(), cur_pos->y() - tar_pos->y(), cur_pos->yaw() - tar_pos->yaw()};
#if 0
        KUBOT_LOG(info) << "CHASSIS_DRIVE: dif_pos=(" << dif_pos.ToString() << ")"
                        << " loading=(" << l_cmd_get_loading_rates->rates << "," << r_cmd_get_loading_rates->rates << ")"
                        << " drv_utliz=(" << l_cmd_get_driver_utilization->rates << "," << r_cmd_get_driver_utilization->rates << ")"
                        /*<< " voltage=(" << l_cmd_get_power_voltage->voltage << "," << r_cmd_get_power_voltage->voltage << ")"*/
                        << " obstacle=(" << (int)f_obstacle_state->obstacle << "," << (int)b_obstacle_state->obstacle << ")"
                        << " cur_twist=(" << std::get<0>(cur_twist) << "," << std::get<1>(cur_twist) << ")"
                        << " tar_twist=(" << std::get<0>(tar_twist) << "," << std::get<1>(tar_twist) << ")";
#endif
    }

    return 0;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_MoveAction(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    /* check if alarm state is abnormal */
    if (l_cmd_get_alarm_code->alarm_code != 0 || r_cmd_get_alarm_code->alarm_code != 0)
    {
        return ERROR::MOTOR_REPORT_ALARM;
    }

    switch (move_action->state)
    {
    case MOVE_ACTION_STATE::START:
        if (!move_action->entered)
        {
            KUBOT_LOG(info) << controller_name << " MoveAction: START!";
            move_action->entered = true;
        }
        else
        {
            move_action->state = MOVE_ACTION_STATE::APPROACH_TARGET;
            move_action->entered = false;
        }
        break;
    case MOVE_ACTION_STATE::APPROACH_TARGET:
        if (!move_action->entered)
        {
            KUBOT_LOG(info) << controller_name << " MoveAction: APPROACH_TARGET!";
            move_action->entered = true;

            /* update moving target */
            tar_pos = std::make_shared<Position2D>(move_action->position);
        }
        else
        if (m_state == STATE::TUNE)
        {
            double& v = std::get<0>(tar_twist);
            double& w = std::get<1>(tar_twist);

            double cur_v = std::get<0>(cur_twist);
            if(fabsf(cur_v) < 0.01)
            {
                max_acc_v = 1.2;
            }
            else
            {
                max_acc_v = 0.8;
            }

            /* NOTICE: TUNE will never end, even arrived the target!!! */
            double delta_x = tar_pos->x() - cur_pos->x();
            v = MATH::sign(delta_x) * pow(std::fabs(delta_x)*0.3,0.8) * 1.00/*1.25*/;
            if (fabs(v) > 0.03/*0.05*/)
            {
                v = MATH::sign(static_cast<float>(v)) * 0.03/*0.05*/;
            }
            if(v >= 0)
            {
                /* acceleration case when v > 0 , use min to limit acceleration */
                if (v >= cur_v)
                {
                    v = std::min((cur_v + max_acc_v * m_sampling_time), v);
                }
                /* deacceleration case when v > 0 , use max to limit acceleration */
                else
                {
                    v = std::max((cur_v - 1 * m_sampling_time), v);
                }
            }
            else
            {
                /* acceleration case when v < 0 , use max to limit acceleration */
                if (v <= cur_v)
                {
                    v = std::max((cur_v - max_acc_v * m_sampling_time), v);
                }
                /* deacceleration case when v < 0 , use min to limit acceleration */
                else
                {
                    v = std::min((cur_v + 1 * m_sampling_time), v);
                }
            }
            w = 0;

            limitAngularTrapezoid();

            /* recheck normally control limit */
            setMaximumTwist();

            if (setTargetTwist() != 0)
            {
                return ERROR::MOTOR_COMMAND_ERROR;
            }
            else
            {
                return ERROR::NEED_RETRY_LATER;
            }
        }
        else
        if (move_action->direction == DIRECTION::X_LINE || move_action->direction == DIRECTION::Y_LINE)
        {
            DIRECTION direction;
            HEADING   heading;
            if(fabsf(sinf(cur_pos->yaw())) < ANGLE_TOLERANCE)
            {
                direction = DIRECTION::X_LINE;
                if (cosf(cur_pos->yaw()) * (tar_pos->x() - cur_pos->x()) > 0)
                {
                    heading = HEADING::FORWARD;
                }
                else
                {
                    heading = HEADING::BACKWARD;
                }
            }
            else
            if(fabsf(cosf(cur_pos->yaw())) < ANGLE_TOLERANCE)
            {
                direction = DIRECTION::Y_LINE;
                if (sinf(cur_pos->yaw()) * (tar_pos->y() - cur_pos->y()) > 0)
                {
                    heading = HEADING::FORWARD;
                }
                else
                {
                    heading = HEADING::BACKWARD;
                }
            }
            else
            {
                KUBOT_LOG(error) << "side angle lager than " << ANGLE_TOLERANCE << "while moving in line!";
                return ERROR::DEVIATE_PATH_TOO_MUCH;
            }

            double front_dist;
            double side_dist;
            double delta_x;
            double delta_y;
            if (direction == DIRECTION::X_LINE)
            {
                front_dist = tar_pos->x() - cur_pos->x();
                side_dist  = tar_pos->y() - cur_pos->y();
                delta_x    = (fabs(front_dist) > 2.0) ? (boost::math::sign(front_dist) * 2.0) : front_dist;
                delta_y    = side_dist;
            }
            else
            if (direction == DIRECTION::Y_LINE)
            {
                front_dist = tar_pos->y() - cur_pos->y();
                side_dist  = tar_pos->x() - cur_pos->x();
                delta_x    = side_dist;
                delta_y    = (fabs(front_dist) > 2.0) ? (boost::math::sign(front_dist) * 2.0) : front_dist;
            }

            double theta = atan2(delta_y, delta_x);
            double cur_yaw;
            double tar_yaw;
            switch (heading)
            {
                case HEADING::FORWARD:
                    cur_yaw = cur_pos->yaw();
                    tar_yaw = tar_pos->yaw();
                    break;
                case HEADING::BACKWARD:
                    cur_yaw = cur_pos->yaw() + M_PI;
                    tar_yaw = tar_pos->yaw() + M_PI;
                    break;
            }
            double alpha = MATH::angleBetween((float) cur_yaw, (float) theta);
            double beta  = MATH::angleBetween((float) tar_yaw, (float) theta);
            double dist  = sqrt(delta_x*delta_x + delta_y*delta_y);

            /* check if target arrived */
            if (fabsf(std::get<0>(cur_twist)) < 0.01 && fabsf(std::get<1>(cur_twist)) < 0.01
                &&
                fabsf(MATH::angleBetween(tar_pos->yaw(), cur_pos->yaw())) < move_action->precision_yaw
                &&
                fabsf(front_dist) < move_action->precision_front)
            {
                move_action->state = MOVE_ACTION_STATE::END;
                move_action->entered = false;
            }
            else
            {
                OBSTACLE_STATE obstacle;
                switch (heading)
                {
                    case HEADING::FORWARD:
                        obstacle = f_obstacle_state->obstacle;
                        break;
                    case HEADING::BACKWARD:
                        obstacle = b_obstacle_state->obstacle;
                        break;
                }

                /* set normally control limit */
                setNormalAcc(1, 1);

                /* set obstacle avoidance limit */
                setObstacleAvoidanceAcc(obstacle, front_dist);

                /* distance-speed curve fitting */
                double& v = std::get<0>(tar_twist);
                double& w = std::get<1>(tar_twist);

                double v_kappa;

                k = -1 / dist * (k2 * (alpha - atan2(-k1 * beta, 1)) + sin(alpha) * (1 + k1 / (1 + (k1 * beta) * (k1 * beta))));
                v_kappa = max_v / (1.0 + gamma * k * k);

                if (fabs(dist) > firstRange)
                {
                    v = v_kappa;
                }
                else
                {
                    v = std::min(dist / 1.6 * max_v, v_kappa);
                }

                v = (heading == HEADING::FORWARD) ? v : -v;
                limitLinearTrapezoid();
                w = v * k;
                w = (heading == HEADING::FORWARD) ? w : -w;

                /*
                if (fabs(front_dist) > firstRange)
                {
                    k = -1 / dist * (k2 * (alpha - atan2(-k1 * beta, 1)) + sin(alpha) * (1 + k1 / (1 + (k1 * beta) * (k1 * beta))));
                    v = std::min(fabs(front_dist)*m_start_stop_dist_ratio,max_v) / (1.0 + gamma * k * k);
                    v = (heading == HEADING::FORWARD) ? v : -v;

                    limitLinearTrapezoid();
                    w = v * k;
                    w = (heading == HEADING::FORWARD) ? w : -w;

                }
                else
                if (fabs(front_dist) > secondRange)
                {
                    double exp_factor;
                    if(dist > 1)
                        exp_factor = m_exp_factor_far;
                    else
                        exp_factor = m_exp_factor_near;
                    double temp_dist = pow(dist, exp_factor);
                    v = max_v * 0.73 * k_dist * temp_dist;
                    w = -k_alpha * alpha + k_beta * beta;
                    v = (heading == HEADING::FORWARD) ? v : -v ;

                    limitLinearTrapezoid();
                    limitAngularTrapezoid();
                }
                else
                {
                    double temp_dist = pow(fabs(front_dist), 0.7);
                    double temp_angle = MATH::angleBetween(tar_pos->yaw(), cur_pos->yaw());
                    v = k_v_short *  max_v * temp_dist;
                    if(heading == HEADING::BACKWARD)
                    {
                        v *= -1;
                        w = k_w_short * (temp_angle + std::min(0.5, 5.0 * fabs(front_dist) * side_dist));
                    }
                    else
                    {
                        w = k_w_short * (temp_angle - std::min(0.5, 5.0 * fabs(front_dist) * side_dist));
                    }

                    limitLinearTrapezoid();
                    limitAngularTrapezoid();
                }*/

                /* recheck normally control limit */
                setMaximumTwist();

                if (setTargetTwist() != 0)
                {
                    return ERROR::MOTOR_COMMAND_ERROR;
                }
                else
                {
                    return ERROR::NEED_RETRY_LATER;
                }
            }
        }
        else
        if (move_action->direction == DIRECTION::CIRCLE)
        {
            double delta_x = tar_pos->x() - cur_pos->x();
            double delta_y = tar_pos->y() - cur_pos->y();
            double dist    = sqrt(delta_x * delta_x + delta_y * delta_y);
            double theta   = atan2(delta_y, delta_x);

            double angleToTag = MATH::mod2pi((float)theta);
            double linearMoveAngle = MATH::angleBetween((float) angleToTag, cur_pos->yaw());
            double target_angle = MATH::angleBetween(tar_pos->yaw(), cur_pos->yaw());
#if 0
            double exp_factor = fabs(target_angle) > 1.0     ? 1.2 :
                                fabs(target_angle) < 0.08726 ? 0.8 :
                                                               0.95;
#else
            double exp_factor = fabs(target_angle) > 1 ? 1.2 : 0.9;
#endif
            double temp_angle   = m_turn_boost*boost::math::sign(target_angle) * pow(fabs(target_angle), exp_factor);

            /* check if arrived */
            if (fabsf(std::get<0>(cur_twist)) < 0.01 && fabsf(std::get<1>(cur_twist)) < 0.01
                &&
                fabsf(MATH::angleBetween(tar_pos->yaw(), cur_pos->yaw())) < 0.005)
            {
                move_action->state = MOVE_ACTION_STATE::END;
                move_action->entered = false;
            }
            else
            {
                /* set normally control limit */
                setNormalAcc(1, 1);

                /* distance-speed curve fitting */
                double& v = std::get<0>(tar_twist);
                double& w = std::get<1>(tar_twist);
                v = k_turn_v * cos(linearMoveAngle) * dist;
                w = k_turn_w * temp_angle;

                limitLinearTrapezoid();
                limitAngularTrapezoid();

                KUBOT_LOG(trace) << "current yaw  : " << cur_pos->yaw();
                KUBOT_LOG(trace) << "current twist: " << std::get<0>(cur_twist) << "," << std::get<1>(cur_twist);
                KUBOT_LOG(trace) << "target  twist: " << v << "," << w;

                /* recheck normally control limit */
                setMaximumTwist();

                if (setTargetTwist() != 0)
                {
                    return ERROR::MOTOR_COMMAND_ERROR;
                }
                else
                {
                    return ERROR::NEED_RETRY_LATER;
                }
            }
        }
        break;
    case MOVE_ACTION_STATE::END:
        if (!move_action->entered)
        {
            KUBOT_LOG(info) << controller_name << " MoveAction: END!";
            move_action->entered = true;
        }
        else
        {
            return ERROR::OK;
        }
        break;
    }

    return ERROR::NEED_RETRY_NOW;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_InitAction(void)
{
    /* check if alarm state is abnormal */
    if (l_cmd_get_alarm_code->alarm_code != 0 || r_cmd_get_alarm_code->alarm_code != 0)
    {
        return ERROR::MOTOR_REPORT_ALARM;
    }

    return ERROR::NEED_RETRY_LATER;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_IdleAction(void)
{
    /* check if alarm state is abnormal */
    if (l_cmd_get_alarm_code->alarm_code != 0 || r_cmd_get_alarm_code->alarm_code != 0)
    {
        return ERROR::MOTOR_REPORT_ALARM;
    }

    /* set target twist to zero */
    tar_twist = std::make_tuple<double,double>(0, 0);

    if (setTargetTwist() != 0)
    {
        return ERROR::MOTOR_COMMAND_ERROR;
    }
    else
    {
        return ERROR::NEED_RETRY_LATER;
    }
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_StopAction(void)
{
    /* check if alarm state is abnormal */
    if (l_cmd_get_alarm_code->alarm_code != 0 || r_cmd_get_alarm_code->alarm_code != 0)
    {
        return ERROR::MOTOR_REPORT_ALARM;
    }

    return ERROR::NEED_RETRY_LATER;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_SleepAction(void)
{
    /* check if alarm state is abnormal */
    if (l_cmd_get_alarm_code->alarm_code != 0 || r_cmd_get_alarm_code->alarm_code != 0)
    {
        return ERROR::MOTOR_REPORT_ALARM;
    }

    return ERROR::NEED_RETRY_LATER;
}

ChassisPositionController::ERROR
ChassisPositionController::Execute_ErrorAction(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_error.count(ERROR::MOTOR_COMMAND_ERROR) == 0)
    {
        if (l_cmd_get_alarm_code->alarm_code == 0)
        {
            motor_command = m_l_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), 0);
            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
            {
                return ERROR::MOTOR_COMMAND_ERROR;
            }
        }

        if (r_cmd_get_alarm_code->alarm_code == 0)
        {
            motor_command = m_r_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), 0);
            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
            {
                return ERROR::MOTOR_COMMAND_ERROR;
            }
        }
    }

    return ERROR::NEED_RETRY_LATER;
}


ChassisPositionController::ERROR
ChassisPositionController::Init(void)
{
    ERROR                                       error_retcode;
    std::shared_ptr<MotorCommand>               motor_command;

    /* set motor to a safe state  */
    motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }
    motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    motor_command = m_l_motor_driver->Execute<MotorCommand_SetControlMode>(FT_ARGUMENTS(2, 100), MOTOR_CONTROL_MODE::SPEED);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }
    motor_command = m_r_motor_driver->Execute<MotorCommand_SetControlMode>(FT_ARGUMENTS(2, 100), MOTOR_CONTROL_MODE::SPEED);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    motor_command = m_l_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(0, m_l_tick_ratio));
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }
    motor_command = m_r_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(0, m_r_tick_ratio));
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    /* start heartbeat */
    error_retcode = Execute_StartHeartbeat();
    if (error_retcode != ERROR::OK && error_retcode != ERROR::NEED_RETRY_LATER)
    {
        StateTransition(STATE::ERROR, error_retcode, true);
        return error_retcode;
    }

    /* obstacle state update thread */
    std::thread([this](){
        std::chrono::steady_clock::time_point       last_process_time;

        decltype(m_f_obstacle_state) f_obstacle_state;
        decltype(m_b_obstacle_state) b_obstacle_state;
        while (true)
        {
            last_process_time = std::chrono::steady_clock::now();

            if (([&]()
            {
                m_obstacle_mtx.lock();
                auto error_stat = ((m_f_obstacle_state != nullptr && m_f_obstacle_state->error_stat != OBSTACLEAVOIDANCESENSOR_ERROR::OK)
                                   ||
                                   (m_b_obstacle_state != nullptr && m_b_obstacle_state->error_stat != OBSTACLEAVOIDANCESENSOR_ERROR::OK));
                m_obstacle_mtx.unlock();
                return error_stat;
            })())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            else
            {
                f_obstacle_state = m_f_obstacle_driver->GetObstacleState(FT_ARGUMENTS(2, 100));
                b_obstacle_state = m_b_obstacle_driver->GetObstacleState(FT_ARGUMENTS(2, 100));

                if (f_obstacle_state->error_stat != OBSTACLEAVOIDANCESENSOR_ERROR::OK)
                {
                    KUBOT_LOG(error) << "get front OBSTACLE state failed!";
                }
                if (b_obstacle_state->error_stat != OBSTACLEAVOIDANCESENSOR_ERROR::OK)
                {
                    KUBOT_LOG(error) << "get back OBSTACLE state failed!";
                }

                m_obstacle_mtx.lock();
                m_f_obstacle_state = std::move(f_obstacle_state);
                m_b_obstacle_state = std::move(b_obstacle_state);
                m_obstacle_mtx.unlock();
            }

            /* control loop interval */
            while (true)
            {
                auto compare_time = std::chrono::steady_clock::now() - std::chrono::microseconds(static_cast<int>(m_sampling_time * 1000000));
                if (compare_time >= last_process_time)
                    break;
                else
                    std::this_thread::sleep_for(std::min(std::chrono::microseconds(10000), std::chrono::duration_cast<std::chrono::microseconds>(last_process_time - compare_time)));
            }
        }
    }).detach();

    /* start monitoring thread */
    std::thread([this](){
        std::chrono::steady_clock::time_point       last_process_time;

        ERROR                                       error_retcode;
        std::shared_ptr<MotorCommand>               motor_command;

        while (true)
        {
            last_process_time = std::chrono::steady_clock::now();

            if (m_process_mtx != nullptr)
            {
                pthread_rwlock_rdlock(const_cast<pthread_rwlock_t*>(m_process_mtx));
            }
            {
                /* get motor status */
                error_retcode = Execute_GetMotorStatus();
                if (error_retcode != ERROR::OK && error_retcode != ERROR::NEED_RETRY_LATER)
                {
                    StateTransition(STATE::ERROR, error_retcode, true);
                }
                /* update twist speed */
                if (error_retcode == ERROR::OK)
                {
                    double r_speed = RPM2SPEED(r_cmd_get_current_speed->speed, m_r_tick_ratio);
                    double l_speed = RPM2SPEED(l_cmd_get_current_speed->speed, m_l_tick_ratio);

                    cur_twist = std::make_tuple((r_speed + l_speed) / 2, (r_speed - l_speed) / Config::Chassis.BASELINE_LENGTH);
                    m_localizator->UpdateTwist(std::make_tuple(true, static_cast<float>(std::get<0>(cur_twist)), static_cast<float>(std::get<1>(cur_twist))));
                }
                else
                if (error_retcode != ERROR::NEED_RETRY_LATER)
                {
                    m_localizator->UpdateTwist(std::make_tuple(false, 0, 0));
                }

                error_retcode = Execute_GetLocalizatorStatus();
                if (error_retcode != ERROR::OK && error_retcode != ERROR::NEED_RETRY_LATER)
                {
                    StateTransition(STATE::ERROR, error_retcode, true);
                }
                if (error_retcode == ERROR::OK)
                {
                    cur_pos = std::make_shared<Position2D>(localizator_status->position);
                }

                error_retcode = Execute_GetObstacleStatus();
                if (error_retcode != ERROR::OK && error_retcode != ERROR::NEED_RETRY_LATER)
                {
                    StateTransition(STATE::ERROR, error_retcode, true);
                }

                /* process state action */
                while (true)
                {
                    switch (m_state)
                    {
                    case STATE::RESET:
                        error_retcode = Execute_ResetAction();
                        if (error_retcode == ERROR::OK)
                        {
                            StateTransition(STATE::IDLE, ERROR::OK, true);
                        }
                        break;
                    case STATE::MOVE:
                        error_retcode = Execute_MoveAction();
                        if (error_retcode == ERROR::OK)
                        {
                            StateTransition(STATE::IDLE, ERROR::OK, true);
                        }
                        break;
                    case STATE::TUNE:
                        error_retcode = Execute_MoveAction();
                        if (error_retcode == ERROR::OK)
                        {
                            StateTransition(STATE::TUNE, ERROR::OK, true);
                        }
                        break;
                    case STATE::WAIT:
                        error_retcode = ERROR::NEED_RETRY_LATER;
                        break;
                    case STATE::INIT:
                        error_retcode = Execute_InitAction();
                        break;
                    case STATE::IDLE:
                        error_retcode = Execute_IdleAction();
                        break;
                    case STATE::STOP:
                        error_retcode = Execute_StopAction();
                        break;
                    case STATE::SLEEP:
                        error_retcode = Execute_SleepAction();
                        break;
                    case STATE::ERROR:
                        error_retcode = Execute_ErrorAction();
                        break;
                    case STATE::RECOVERY:
                        if (clear_alarm_action != nullptr)
                        {
                            error_retcode = Execute_ClearAlarmAction();
                            if (error_retcode == ERROR::OK)
                            {
                                clear_alarm_action = nullptr;
                            }
                        }
                        else
                        if (m_calibrated)
                        {
                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetControlMode>(FT_ARGUMENTS(2, 100), MOTOR_CONTROL_MODE::SPEED);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetControlMode>(FT_ARGUMENTS(2, 100), MOTOR_CONTROL_MODE::SPEED);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(0, m_l_tick_ratio));
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(0, m_r_tick_ratio));
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), true);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), true);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            error_retcode = ERROR::OK;
                            StateTransition(STATE::IDLE, ERROR::OK, true);
                        }
                        else
                        {
                            /* set motor to a safe state  */
                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetControlMode>(FT_ARGUMENTS(2, 100), MOTOR_CONTROL_MODE::SPEED);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetControlMode>(FT_ARGUMENTS(2, 100), MOTOR_CONTROL_MODE::SPEED);
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            motor_command = m_l_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(0, m_l_tick_ratio));
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }
                            motor_command = m_r_motor_driver->Execute<MotorCommand_SetTargetSpeed>(FT_ARGUMENTS(2, 100), SPEED2RPM(0, m_r_tick_ratio));
                            if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
                            {
                                error_retcode = ERROR::MOTOR_COMMAND_ERROR;
                                break;
                            }

                            error_retcode = ERROR::OK;
                            StateTransition(STATE::INIT, ERROR::OK, true);
                        }
                        break;
                    }

                    if (error_retcode == ERROR::OK)
                        continue;
                    else
                    if (error_retcode == ERROR::NEED_RETRY_NOW)
                        continue;
                    else
                    if (error_retcode == ERROR::NEED_RETRY_LATER)
                        break;
                    else
                    {
                        StateTransition(STATE::ERROR, error_retcode, true);
                        continue;
                    }
                }

                /* keep heartbeat alive */
                error_retcode = Execute_KeepHeartbeat();
                if (error_retcode != ERROR::OK && error_retcode != ERROR::NEED_RETRY_LATER)
                {
                    StateTransition(STATE::ERROR, error_retcode, true);
                }
            }
            if (m_process_mtx != nullptr)
            {
                pthread_rwlock_unlock(const_cast<pthread_rwlock_t*>(m_process_mtx));
            }

            /* change process frequency according to action state */
            while (true)
            {
                std::chrono::microseconds delay_duration;
                switch (m_state)
                {
                    case STATE::RESET:
                        delay_duration = std::chrono::milliseconds(1);
                        break;

                    case STATE::MOVE:
                    case STATE::TUNE:
                        delay_duration = std::chrono::microseconds(static_cast<int>(m_sampling_time * 1000000));
                        break;

                    default:
                        delay_duration = std::chrono::milliseconds(100);
                        break;
                }

                auto compare_time = std::chrono::steady_clock::now() - delay_duration;
                if (compare_time >= last_process_time)
                    break;
                else
                    std::this_thread::sleep_for(std::min(std::chrono::microseconds(10000), std::chrono::duration_cast<std::chrono::microseconds>(last_process_time - compare_time)));
            }
        }
    }).detach();

    /* init success, switch to INIT mode */
    StateTransition(STATE::INIT);

    return ERROR::OK;
}


ChassisPositionController::ERROR
ChassisPositionController::FindZeroPosition(void)
{
    if (m_state != STATE::INIT)
    {
        KUBOT_LOG(error) << "the specified action FindZeroPosition is allowed only in INIT state!";
        return ERROR::CONDITION_ERROR;
    }

    /* create reset action, switch to RESET mode */
    reset_action = std::make_shared<ResetAction>();
    StateTransition(STATE::RESET);

    return ERROR::OK;
}


ChassisPositionController::ERROR
ChassisPositionController::SetTargetPosition(Position2D position, double speed, int precision_lvl)
{
    switch (m_state)
    {
    case STATE::IDLE:
        {
            float x_diff   = position.x() - cur_pos->x();
            float y_diff   = position.y() - cur_pos->y();
            float yaw_diff = MATH::angleBetween(position.yaw(), cur_pos->yaw());

            if(fabs(x_diff)   < DIFF_LINE_LOWER_BOUND &&
               fabs(y_diff)   < DIFF_LINE_LOWER_BOUND &&
               fabs(yaw_diff) < ANGLE_TOLERANCE)
            {
                KUBOT_LOG(warning)<< "the current position(" << cur_pos->ToString() << ") is too close to the target(" << position.ToString() << ")";
                return ERROR::OK;
            }
            else
            if(fabs(x_diff)   < DIFF_LINE_UPPER_BOUND &&
               fabs(y_diff)   < DIFF_LINE_UPPER_BOUND &&
               fabs(yaw_diff) > ANGLE_TOLERANCE)
            {
                move_action = std::make_shared<MoveAction>(position, DIRECTION::CIRCLE, 0.005, -1);
                StateTransition(STATE::MOVE);
            }
            else
            if (fabs(x_diff) > DIFF_LINE_LOWER_BOUND &&
                fabs(y_diff) < DIFF_LINE_UPPER_BOUND &&
                fabs(sinf(cur_pos->yaw())) < ANGLE_TOLERANCE && fabs(yaw_diff) < ANGLE_TOLERANCE)
            {
                move_action = std::make_shared<MoveAction>(position, DIRECTION::X_LINE, (precision_lvl == 0 ? 0.015 : 0.005), (precision_lvl == 0 ? 0.010 : 0.005));
                StateTransition(STATE::MOVE);
            }
            else
            if (fabs(x_diff) < DIFF_LINE_UPPER_BOUND &&
                fabs(y_diff) > DIFF_LINE_LOWER_BOUND &&
                fabs(cosf(cur_pos->yaw())) < ANGLE_TOLERANCE && fabs(yaw_diff) < ANGLE_TOLERANCE)
            {
                move_action = std::make_shared<MoveAction>(position, DIRECTION::Y_LINE, (precision_lvl == 0 ? 0.015 : 0.005), (precision_lvl == 0 ? 0.010 : 0.005));
                StateTransition(STATE::MOVE);
            }
            else
            {
                KUBOT_LOG(error) << "there is no straightforward way from (" << cur_pos->ToString() << ") to (" << position.ToString() << ")";
                return ERROR::CONDITION_ERROR;
            }
        }
        break;
    case STATE::MOVE:
        if (move_action->direction == DIRECTION::X_LINE || move_action->direction == DIRECTION::Y_LINE)
        {
            float x_diff   = position.x() - move_action->position.x();
            float y_diff   = position.y() - move_action->position.y();
            float yaw_diff = MATH::angleBetween(position.yaw(), move_action->position.yaw());

            if (move_action->direction == DIRECTION::X_LINE)
            {
                if (fabs(y_diff) >= DIFF_LINE_UPPER_BOUND)
                {
                    KUBOT_LOG(error) << "cannot change y-position while moving in x-line";
                    return ERROR::CONDITION_ERROR;
                }
                else
                if (fabs(yaw_diff) >= ANGLE_TOLERANCE)
                {
                    KUBOT_LOG(error) << "cannot turn spot while moving in x-line";
                    return ERROR::CONDITION_ERROR;
                }
                else
                {
                    move_action = std::make_shared<MoveAction>(position, DIRECTION::X_LINE, (precision_lvl == 0 ? 0.015 : 0.005), (precision_lvl == 0 ? 0.010 : 0.005));
                }
            }
            else
            if (move_action->direction == DIRECTION::Y_LINE)
            {
                if (fabs(x_diff) >= DIFF_LINE_UPPER_BOUND)
                {
                    KUBOT_LOG(error) << "cannot change x-position while moving in y-line";
                    return ERROR::CONDITION_ERROR;
                }
                else
                if (fabs(yaw_diff) >= ANGLE_TOLERANCE)
                {
                    KUBOT_LOG(error) << "cannot turn spot while moving in y-line";
                    return ERROR::CONDITION_ERROR;
                }
                else
                {
                    move_action = std::make_shared<MoveAction>(position, DIRECTION::Y_LINE, (precision_lvl == 0 ? 0.015 : 0.005), (precision_lvl == 0 ? 0.010 : 0.005));
                }
            }
            else
            if (move_action->direction == DIRECTION::CIRCLE)
            {
                if (fabs(x_diff) >= DIFF_LINE_UPPER_BOUND || fabs(y_diff) >= DIFF_LINE_UPPER_BOUND)
                {
                    KUBOT_LOG(error) << "cannot change x-position/y-position while turning spot";
                    return ERROR::CONDITION_ERROR;
                }
                else
                {
                    move_action = std::make_shared<MoveAction>(position, DIRECTION::CIRCLE, 0.005, -1);
                }
            }
        }
        break;
    case STATE::TUNE:
        {
            float x_diff   = position.x() - cur_pos->x();

            if (fabs(x_diff) < 0.001)
            {
                KUBOT_LOG(warning)<< "the current tune position(" << cur_pos->x() << ") is too close to the target(" << position.x() << ")";
                return ERROR::OK;
            }
            else
            {
                move_action = std::make_shared<MoveAction>(Position2D(position.x(), cur_pos->y(), cur_pos->yaw()), DIRECTION::X_LINE, 0.001, -1);
            }
        }
        break;
    default:
        {
            KUBOT_LOG(error) << "the specified action SetTargetPosition is allowed only in IDLE/MOVE/TUNE state!";
            return ERROR::CONDITION_ERROR;
        }
    }

    return ERROR::OK;
}


ChassisPositionController::ERROR
ChassisPositionController::EnterTuneMode(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_state != STATE::IDLE && m_state != STATE::MOVE)
    {
        KUBOT_LOG(error) << "the specified action EnterTuneMode is allowed only in IDLE/MOVE state!";
        return ERROR::CONDITION_ERROR;
    }

    /* create move action, switch to TUNE state */
    move_action = std::make_shared<MoveAction>(*cur_pos, DIRECTION::X_LINE, 0.001, -1);
    StateTransition(STATE::TUNE);

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::LeaveTuneMode(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_state != STATE::TUNE)
    {
        KUBOT_LOG(error) << "the specified action LeaveTuneMode is allowed only in TUNE state!";
        return ERROR::CONDITION_ERROR;
    }

    /* switch to IDLE state */
    StateTransition(STATE::IDLE);

    return ERROR::OK;
}


ChassisPositionController::ERROR
ChassisPositionController::EnterSleepMode(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_state != STATE::IDLE)
    {
        KUBOT_LOG(error) << "the specified action EnterSleepMode is allowed only in IDLE state!";
        return ERROR::CONDITION_ERROR;
    }

    motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }
    motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), false);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    /* switch to SLEEP state */
    StateTransition(STATE::SLEEP);

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::LeaveSleepMode(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_state != STATE::SLEEP)
    {
        KUBOT_LOG(error) << "the specified action LeaveSleepMode is allowed only in SLEEP state!";
        return ERROR::CONDITION_ERROR;
    }

    motor_command = m_l_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), true);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }
    motor_command = m_r_motor_driver->Execute<MotorCommand_SetServoEnable>(FT_ARGUMENTS(2, 100), true);
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    /* switch to SLEEP state */
    StateTransition(STATE::IDLE);

    return ERROR::OK;
}


ChassisPositionController::ERROR
ChassisPositionController::Stop(void)
{
    std::shared_ptr<MotorCommand> motor_command;

    if (m_state != STATE::MOVE
        &&
        m_state != STATE::TUNE)
    {
        KUBOT_LOG(error) << "the specified action Stop is allowed only in MOVE/TUNE state!";
        return ERROR::CONDITION_ERROR;
    }

    motor_command = m_l_motor_driver->Execute<MotorCommand_QuickStop>(FT_ARGUMENTS(2, 100));
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    motor_command = m_r_motor_driver->Execute<MotorCommand_QuickStop>(FT_ARGUMENTS(2, 100));
    if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
    {
        StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
        return ERROR::MOTOR_COMMAND_ERROR;
    }

    /* switch to STOP state */
    StateTransition(STATE::STOP);

    return ERROR::OK;
}

ChassisPositionController::ERROR
ChassisPositionController::Recovery(void)
{
    ERROR                                       error_retcode;
    std::shared_ptr<MotorCommand>               motor_command;

    if (m_error.empty())
    {
        KUBOT_LOG(error) << "the specified action Recovery make no sense without error!";
        return ERROR::CONDITION_ERROR;
    }

    /* clear MOTOR_COMMAND_ERROR */
    if (m_error.count(ERROR::MOTOR_COMMAND_ERROR) != 0)
    {
        KUBOT_LOG(info) << "clear MOTOR_COMMAND_ERROR flag!";
        m_error.erase(ERROR::MOTOR_COMMAND_ERROR);

        motor_command = m_l_motor_driver->Execute<MotorCommand_Init>(FT_ARGUMENTS(2, 100));
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
            return ERROR::MOTOR_COMMAND_ERROR;
        }

        motor_command = m_r_motor_driver->Execute<MotorCommand_Init>(FT_ARGUMENTS(2, 100));
        if (motor_command->error_stat != MOTOR_CMD_STATUS::OK)
        {
            StateTransition(STATE::ERROR, ERROR::MOTOR_COMMAND_ERROR);
            return ERROR::MOTOR_COMMAND_ERROR;
        }

        /* restart heartbeat */
        error_retcode = Execute_StartHeartbeat();
        if (error_retcode != ERROR::OK)
        {
            StateTransition(STATE::ERROR, error_retcode, true);
            return error_retcode;
        }
    }

    /* clear SENSOR_COMMAND_ERROR */
    if (m_error.count(ERROR::SENSOR_COMMAND_ERROR) != 0)
    {
        KUBOT_LOG(info) << "clear SENSOR_COMMAND_ERROR flag!";
        m_error.erase(ERROR::SENSOR_COMMAND_ERROR);

        m_obstacle_mtx.lock();
        m_f_obstacle_state = nullptr;
        m_b_obstacle_state = nullptr;
        m_obstacle_mtx.unlock();
    }

    /* clear LOCALIZATOR_REPORT_ERROR */
    if (m_error.count(ERROR::LOCALIZATOR_REPORT_ERROR) != 0)
    {
        KUBOT_LOG(info) << "clear LOCALIZATOR_REPORT_ERROR flag!";
        m_error.erase(ERROR::LOCALIZATOR_REPORT_ERROR);

        m_localizator->Recovery();
    }

    /* clear DEVIATE_PATH_TOO_MUCH */
    if (m_error.count(ERROR::DEVIATE_PATH_TOO_MUCH) != 0)
    {
        KUBOT_LOG(info) << "clear DEVIATE_PATH_TOO_MUCH flag!";
        m_error.erase(ERROR::DEVIATE_PATH_TOO_MUCH);

        m_localizator->Recovery();
    }

    /* create clear alarm action if report alarm */
    if (m_error.count(ERROR::MOTOR_REPORT_ALARM) != 0)
    {
        KUBOT_LOG(info) << "clear MOTOR_REPORT_ALARM flag!";
        m_error.erase(ERROR::MOTOR_REPORT_ALARM);

        clear_alarm_action = std::make_shared<ClearAlarmAction>();
    }
    else
    {
        clear_alarm_action = nullptr;
    }

    /* switch to RECOVERY state */
    StateTransition(STATE::RECOVERY);

    return ERROR::OK;
}


ChassisPositionController::STATUS
ChassisPositionController::GetStatus(void)
{
    std::shared_ptr<Position2D>                                 current_position;
    if (localizator_status != nullptr && localizator_status->error_stat == EKF_Estimator::ERROR::OK)
    {
        current_position = std::make_shared<Position2D>(localizator_status->position);
    }

    std::shared_ptr<std::tuple<double,double>>                  current_twist;
    if (l_cmd_get_current_speed != nullptr && l_cmd_get_current_speed->error_stat == MOTOR_CMD_STATUS::OK
        &&
        r_cmd_get_current_speed != nullptr && r_cmd_get_current_speed->error_stat == MOTOR_CMD_STATUS::OK)
    {
        current_twist = std::make_shared<std::tuple<double,double>>(std::make_tuple<double,double>(RPM2SPEED(l_cmd_get_current_speed->speed, m_l_tick_ratio), RPM2SPEED(r_cmd_get_current_speed->speed, m_r_tick_ratio)));
    }

    std::shared_ptr<std::tuple<OBSTACLE_STATE,OBSTACLE_STATE>>  obstacle_state;
    if (m_f_obstacle_state != nullptr && m_f_obstacle_state->error_stat == OBSTACLEAVOIDANCESENSOR_ERROR::OK
        &&
        m_b_obstacle_state != nullptr && m_b_obstacle_state->error_stat == OBSTACLEAVOIDANCESENSOR_ERROR::OK)
    {
        obstacle_state = std::make_shared<std::tuple<OBSTACLE_STATE,OBSTACLE_STATE>>(std::make_tuple(m_f_obstacle_state->obstacle, m_b_obstacle_state->obstacle));
    }

    return STATUS({m_state, m_error, current_position, current_twist, obstacle_state});
}

void ChassisPositionController::StateTransition(STATE target_state, ERROR error_code, bool raise_event)
{
    bool transitioned = false;
    if (target_state != m_state)
    {
        if (m_state == STATE::RESET && target_state == STATE::IDLE)
        {
            m_calibrated = true;
        }

        m_state = target_state;
        transitioned = true;
    }
    if (error_code != ERROR::OK && m_error.count(error_code) == 0)
    {
        if (error_code == ERROR::LOCALIZATOR_REPORT_ERROR || error_code == ERROR::DEVIATE_PATH_TOO_MUCH)
        {
            m_calibrated = false;
        }

        m_error.insert(error_code);
        transitioned = true;
    }

    auto event_status = GetStatus();
    if (transitioned)
    {
        KUBOT_LOG(info) << controller_name << " StateTransition(" << event_status.ToString() << ")";
    }
    if (!transitioned || !raise_event)
    {
        return;
    }
    for (const auto& event_callback : event_callbacks)
    {
        event_callback(event_status);
    }
}

void ChassisPositionController::RegisterStatusEvent(std::function<void(const STATUS&)> event_callback)
{
    this->event_callbacks.push_back(event_callback);
}
