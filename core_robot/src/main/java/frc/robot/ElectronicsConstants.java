package frc.robot;

import frc.robot.common.robotprovider.PneumaticsModuleType;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // We expect the following to be true.  Change INVERT_*_AXIS to true if any of the following are not met:
    // 1. forwards/up on a joystick is positive, backwards/down is negative.
    // 2. right on a joystick is positive, left on a joystick is negative.
    // 3. pressed on a trigger is positive, released is negative/zero.
    public static final boolean INVERT_XBONE_LEFT_X_AXIS = false;
    public static final boolean INVERT_XBONE_RIGHT_X_AXIS = false;
    public static final boolean INVERT_XBONE_LEFT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_LEFT_TRIGGER = false;
    public static final boolean INVERT_XBONE_RIGHT_TRIGGER = false;

    public static final boolean INVERT_PS4_LEFT_X_AXIS = false;
    public static final boolean INVERT_PS4_RIGHT_X_AXIS = false;
    public static final boolean INVERT_PS4_LEFT_Y_AXIS = true;
    public static final boolean INVERT_PS4_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_PS4_LEFT_TRIGGER = false;
    public static final boolean INVERT_PS4_RIGHT_TRIGGER = false;

    public static final boolean INVERT_THROTTLE_AXIS = true;
    public static final boolean INVERT_TRIGGER_AXIS = false;

    public static final int PNEUMATICS_MODULE_A = 0; // Module A
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_A = PneumaticsModuleType.PneumaticsHub; // Module A
    public static final int PNEUMATICS_MODULE_B = 1; // Module B
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_B = PneumaticsModuleType.PneumaticsHub; // Module B

    public static final boolean PNEUMATICS_USE_ANALOG = true;
    public static final double PNEUMATICS_MIN_PSI = 110.0;
    public static final double PNEUMATICS_MAX_PSI = 120.0;

    //================================================== IMU ==============================================================

    public static final int PIGEON_IMU_CAN_ID = 42;

    //================================================== Indicator Lights ==============================================================

    public static final int INDICATOR_LIGHT_CANDLE_CAN_ID = 55;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_DRIVE_MOTOR_1_CAN_ID = 1;
    public static final int DRIVETRAIN_STEER_MOTOR_1_CAN_ID = 2;
    public static final int DRIVETRAIN_DRIVE_MOTOR_2_CAN_ID = 3;
    public static final int DRIVETRAIN_STEER_MOTOR_2_CAN_ID = 4;
    public static final int DRIVETRAIN_DRIVE_MOTOR_3_CAN_ID = 5;
    public static final int DRIVETRAIN_STEER_MOTOR_3_CAN_ID = 6;
    public static final int DRIVETRAIN_DRIVE_MOTOR_4_CAN_ID = 7;
    public static final int DRIVETRAIN_STEER_MOTOR_4_CAN_ID = 8;

    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_1_CAN_ID = 1;
    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_2_CAN_ID = 2;
    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_3_CAN_ID = 3;
    public static final int DRIVETRAIN_ABSOLUTE_ENCODER_4_CAN_ID = 4;

    //================================================== Cargo Mechanism ==============================================================

    public static final int CARGO_FLYWHEEL_MOTOR_CAN_ID = 9;
    public static final int CARGO_FLYWHEEL_FOLLOWER_MOTOR_CAN_ID = 10;
    public static final int CARGO_INTAKE_MOTOR_CAN_ID = 11;
    public static final int CARGO_CONVEYOR_MOTOR_CAN_ID = 12;
    public static final int CARGO_FEEDER_MOTOR_CAN_ID = 13;

    public static final int CARGO_INTAKE_PISTON_FORWARD = 6;
    public static final int CARGO_INTAKE_PISTON_REVERSE = 7;
    public static final int CARGO_HOOD_FORWARD = 8;
    public static final int CARGO_HOOD_REVERSE = 9;

    public static final int CARGO_FEEDER_THROUGHBEAM_ANALOG_INPUT = 0;
    public static final int CARGO_CONVEYOR_THROUGHBEAM_ANALOG_INPUT = 1;

    //================================================== Climber Mechanism ==============================================================

    public static final int CLIMBER_WINCH_MOTOR_MASTER_CAN_ID = 14;
    public static final int CLIMBER_WINCH_MOTOR_FOLLOWER_CAN_ID = 15;

    public static final int CLIMBER_WINCH_LOCK_FORWARD = 0;
    public static final int CLIMBER_WINCH_LOCK_BACKWARD = 1;
    public static final int CLIMBER_ACTIVE_HOOK_FORWARD = 2;
    public static final int CLIMBER_ACTIVE_HOOK_REVERSE = 3;
    public static final int CLIMBER_ACTIVE_ARM_FORWARD = 4;
    public static final int CLIMBER_ACTIVE_ARM_REVERSE = 5;

    public static final int CLIMBER_WINCH_ARM_RETRACTED_LIMIT_SWITCH_DIGITAL_INPUT = 2;
}
