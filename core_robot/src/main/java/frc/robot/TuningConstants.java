package frc.robot;

import frc.robot.driver.DigitalOperation;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double PERRY_THE_PLATYPUS = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2022;
    public static final boolean LOG_TO_FILE = TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = true;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    //================================================= Power ======================================================

    public static final boolean POWER_TRACK_CURRENT = true;
    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.SAMPLES_PER_LOOP;
    public static final double SAMPLE_DURATION = TuningConstants.LOOP_DURATION / TuningConstants.SAMPLES_PER_LOOP;
    public static final int POWER_OVERCURRENT_SAMPLES = (int)(TuningConstants.POWER_OVERCURRENT_TRACKING_DURATION / TuningConstants.SAMPLE_DURATION); // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_THRESHOLD = 120.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 160.0;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // How long the robot system must remain centered on the target when using time
    public static final double VISION_CENTERING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.025;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.01;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.4;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.012;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.015;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static final double VISION_FAST_ADVANCING_PID_KP = 0.15;
    public static final double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static final double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static final double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT; //+ TuningConstants.LED_STRIP_LED_COUNT; // * 2;

    public static final int CANDLE_ANIMATION_SLOT_1 = 0;
    public static final int CANDLE_ANIMATION_SLOT_2 = 1;

    public static final int INDICATOR_PURPLE_RED = 101;
    public static final int INDICATOR_PURPLE_GREEN = 34;
    public static final int INDICATOR_PURPLE_BLUE = 129;
    public static final int INDICATOR_PURPLE_WHITE = 0;

    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    // Has (at least) a single cargo light
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Has a second cargo light
    public static final int INDICATOR_YELLOW_COLOR_RED = 255;
    public static final int INDICATOR_YELLOW_COLOR_GREEN = 255;
    public static final int INDICATOR_YELLOW_COLOR_BLUE = 0;
    public static final int INDICATOR_YELLOW_COLOR_WHITE = 0;

    // Shooter spin-up lights
    public static final int INDICATOR_GREEN_COLOR_RED = 0;
    public static final int INDICATOR_GREEN_COLOR_GREEN = 255;
    public static final int INDICATOR_GREEN_COLOR_BLUE = 0;
    public static final int INDICATOR_GREEN_COLOR_WHITE = 0;

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;

    public static final boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean DRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 79.365;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = 52.119;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = -128.935;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -125.419;

    // Position PID (angle) per-module
    public static final double DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KF = 0.34; // 1023 over max speed (3000 ticks per 100ms)
    public static final int DRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = 48000;
    public static final int DRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = 48000;

    // Velocity PID (drive) per-module
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 16000.0; // 20000 was highest speed at full throttle FF on blocks. this is #ticks / 100ms

    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = 0.1;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = 0.05115; // .05115 ==> ~ 1023 / 20000 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double DRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KP = 0.0; // 1.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 0.0; // 1.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean DRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double DRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double DRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final boolean DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 35.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 35.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 0.25;

    public static final boolean DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 20.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 30.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 0.1;

    public static final int DRIVETRAIN_SENSOR_FRAME_PERIOD_MS = 10;
    public static final int DRIVETRAIN_PID_FRAME_PERIOD_MS = 100;

    public static final boolean DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.25;

    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY = 0.15;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.15;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double DRIVETRAIN_TURN_SCALE = 4.0; // radians per second
    public static final double DRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double DRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.80 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second

    //================================================== Cargo Mechanism ==============================================================

    public static final double CARGO_FLYWHEEL_MOTOR_PID_KP = 0.055;
    public static final double CARGO_FLYWHEEL_MOTOR_PID_KI = 0.0;
    public static final double CARGO_FLYWHEEL_MOTOR_PID_KD = 0.01;
    public static final double CARGO_FLYWHEEL_MOTOR_PID_KF = 0.0487143; // 1023 / 21000 
    public static final double CARGO_FLYWHEEL_MOTOR_PID_KS = 21000.0; // max speed we will use, in ticks per 100ms yes

    public static final boolean CARGO_FLYWHEEL_MOTOR_MASTER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double CARGO_FLYWHEEL_MOTOR_MASTER_VOLTAGE_COMPENSATION_MAXVOLTAGE = 12.0;
    public static final int CARGO_FLYWHEEL_SENSOR_FRAME_PERIOD_MS = 20;
    public static final int CARGO_FLYWHEEL_FOLLOWER_SENSOR_FRAME_PERIOD_MS = 255;
    public static final int CARGO_FLYWHEEL_FOLLOWER_GENERAL_FRAME_PERIOD_MS = 255;

    public static final double CARGO_INTAKE_POWER = 0.6;
    public static final double CARGO_INTAKE_OUT_POWER = -0.6;
    public static final double CARGO_INTAKE_EJECT_POWER = -0.8;
    public static final double CARGO_CONVEYOR_ADVANCE_ONE_POWER = 0.4;
    public static final double CARGO_CONVEYOR_ADVANCE_TWO_POWER = 0.5;
    public static final double CARGO_CONVEYOR_INTAKE_ONE_POWER = 0.4;
    public static final double CARGO_CONVEYOR_INTAKE_TWO_POWER = 0.5;
    public static final double CARGO_CONVEYOR_REVERSE_POWER = -0.4;
    public static final double CARGO_FEEDER_POWER = 0.4;

    public static final double CARGO_CONVEYOR_RUNTIME_AFTER_INTAKE = 2.0;
    public static final double CARGO_CONVEYOR_RUNTIME_FOR_ADVANCE = 1.5;
    public static final double CARGO_INTAKE_EXTENSION_TIMEOUT = 0.5;

    public static final int CARGO_FLYWHEEL_VELOCITY_PERIOD = 10;
    public static final int CARGO_FLYWHEEL_VELOCITY_WINDOWSIZE = 8;

    // Real-world distances: 25.5", 51", 66", 83", 100", 118", 127"
    public static final double[] CARGO_KNOWN_SHOOTING_DISTANCES =
        new double[]
        {
            50.0,
            60.0,
            70.0,
            80.0,
            90.0,
            100.0,
            //110.0,
            // 120.0,
            // 130.0,
            // 140.0
        };
    public static final DigitalOperation[] CARGO_KNOWN_SHOOTING_HOOD_UP =
        new DigitalOperation[]
        {
            DigitalOperation.CargoHoodMedium,
            DigitalOperation.CargoHoodLong,
            DigitalOperation.CargoHoodLong,
            DigitalOperation.CargoHoodMedium,
            DigitalOperation.CargoHoodLong,
            DigitalOperation.CargoHoodLong,
            //DigitalOperation.CargoHoodLong,
            // DigitalOperation.CargoHoodLong,
            // DigitalOperation.CargoHoodLong,
            // DigitalOperation.CargoHoodLong
        };
    public static final double[] CARGO_KNOWN_SHOOTING_FLYWHEEL_SPIN_SPEED =
        new double[]
        {
            0.583,
            0.67,
            0.58,
            0.72,
            0.60,
            0.615,
            //0.63,
            // 0.825,
            // 0.83,
            // 0.92
        };
    
    public static final double CARGO_FLYWHEEL_CUSTOM_SPEED = 0.2;
    
    // MY FAV CONSTANTS
    public static final double Y_DISTANCE_SHOOTER_TO_HUB = 80;
    public static final double X_DISTANCE_FROM_EDGE_OF_HUB_TO_SHOOTING_SPOT = 36;
    public static final double X_DISTANCE_BUMBER_TO_FLYWHEEL = 20;
    public static final double G_INCHES = 386.1;


    public static final double CARGO_DISTANCE_TO_VELOCITY_MULTIPLIER = 0.00615;

    public static final double CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED = 0.56;
    public static final double CARGO_FLYWHEEL_POINT_BLANK_LOW_SPINUP_SPEED = 0.3;
    public static final double CARGO_FLYWHEEL_AUTO_FOUR_BALL_SHOOT_SPEED = 0.67;
    public static final double CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED = 0.65;

    public static final double CARGO_CONVEYOR_THROUGHBEAM_CUTOFF = 2.7;
    public static final double CARGO_FEEDER_THROUGHBEAM_CUTOFF = 2.7;

    public static final double CARGO_FLYWHEEL_ALLOWABLE_ERROR_RANGE = 500;

    public static final double CARGO_SHOOT_CHECKBALL_MIN_WAIT_TIME = 1.0;
    public static final double CARGO_SHOOT_CHECKBALL_WAIT_TIMEOUT = TuningConstants.CARGO_CONVEYOR_RUNTIME_FOR_ADVANCE;
    public static final double CARGO_SHOOT_SPINUP_MIN_WAIT_TIME = 1.00;///0.75;
    public static final double CARGO_SHOOT_SPINUP_WAIT_TIMEOUT = 2.0;
    public static final double CARGO_SHOOT_MIN_WAIT_TIME = 0.5;
    public static final double CARGO_SHOOT_WAIT_TIMEOUT = 1.75;
    public static final double CARGO_SHOOT_AFTER_SHOOTING_CHECKBALL_MIN_WAIT_TIME = TuningConstants.CARGO_SHOOT_CHECKBALL_MIN_WAIT_TIME;
    public static final double CARGO_SHOOT_AFTER_SHOOTING_CHECKBALL_WAIT_TIMEOUT = TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIMEOUT;

    public static final boolean CARGO_USE_SHOOT_2 = true;
    public static final double CARGO_SHOOT2_CHECKBALL_MIN_WAIT_TIME = 0.5;
    public static final double CARGO_SHOOT2_CHECKBALL_WAIT_TIMEOUT = TuningConstants.CARGO_CONVEYOR_RUNTIME_FOR_ADVANCE;
    public static final double CARGO_SHOOT2_SPINUP_MIN_WAIT_TIME = 0.25;
    public static final double CARGO_SHOOT2_SPINUP_WAIT_TIMEOUT = 1.5;
    public static final double CARGO_SHOOT2_MIN_WAIT_TIME = 0.2;
    public static final double CARGO_SHOOT2_WAIT_TIMEOUT = 2.0;
    public static final double CARGO_SHOOT2_AFTER_SHOOTING_CHECKBALL_MIN_WAIT_TIME = 1.0;
    public static final double CARGO_SHOOT2_AFTER_SHOOTING_CHECKBALL_WAIT_TIMEOUT = 2.0;
    public static final double CARGO_SHOOT2_BETWEEN_SHOTS_MIN_WAIT_TIME = 0.2;
    public static final double CARGO_SHOOT2_BETWEEN_SHOTS_WAIT_TIMEOUT = 1.5;

    public static final double UPPER_HUB_HEIGHT = 104; // inches
    public static final double LOWER_HUB_HEIGHT = 41; // inches
    public static final double RELATIVE_UPPER_HUB_HEIGHT = TuningConstants.UPPER_HUB_HEIGHT - HardwareConstants.CARGO_SHOOTER_HEIGHT; // inches
    public static final double RELATIVE_LOWER_HUB_HEIGHT = TuningConstants.LOWER_HUB_HEIGHT - HardwareConstants.CARGO_SHOOTER_HEIGHT; // inches

    //================================================== Climber Mechanism ==============================================================

    public static final boolean CLIMBER_USE_PID = false;
    public static final boolean CLIMBER_USE_MOTION_MAGIC = false;

    // climber normal Positional PID settings:
    public static final double CLIMBER_WINCH_MOTOR_U_PID_KP = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PID_KI = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PID_KD = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PID_KF = 0.0;
    
    public static final double CLIMBER_WINCH_MOTOR_W_PID_KP = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PID_KI = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PID_KD = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PID_KF = 0.0;

    // climber MotionMagic Positional PID settings:
    public static final double CLIMBER_WINCH_MOTOR_U_PIDVA_KP = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PIDVA_KI = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PIDVA_KD = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PIDVA_KF = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_U_PIDVA_KV = 0.0; // in ticks / 100ms
    public static final double CLIMBER_WINCH_MOTOR_U_PIDVA_KA = 0.0; // in (ticks / 100ms) / s

    public static final double CLIMBER_WINCH_MOTOR_W_PIDVA_KP = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PIDVA_KI = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PIDVA_KD = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PIDVA_KF = 0.0;
    public static final double CLIMBER_WINCH_MOTOR_W_PIDVA_KV = 0.0; // in ticks / 100ms
    public static final double CLIMBER_WINCH_MOTOR_W_PIDVA_KA = 0.0; // in (ticks / 100ms) / s

    public static final boolean CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_MAXVOLTAGE = 12.0;
    public static final int CLIMBER_WINCH_SENSOR_FRAME_PERIOD_MS = 50;
    public static final int CLIMBER_WINCH_FOLLOWER_SENSOR_FRAME_PERIOD_MS = 255;
    public static final int CLIMBER_WINCH_FOLLOWER_GENERAL_FRAME_PERIOD_MS = 255;

    public static final double CLIMBER_FULL_RETRACT_LENGTH = 0.0;
    public static final double CLIMBER_SHORT_EXTEND_LENGTH = 0.2;
    public static final double CLIMBER_MOSTLY_EXTEND_LENGTH = 0.8;
    public static final double CLIMBER_FULL_EXTEND_LENGTH = 1.0;

    public static final double CLIMBER_WINCH_POSITION_EXTEND_ACCEPTABLE_DELTA = 0.05;

    //================================================== Climber Mechanism ==============================================================
    public static final double COMPRESSOR_ENOUGH_PRESSURE = 90.0;
    public static final double COMPRESSOR_FILL_RATE = 1.0; //PSI PER SECOND
}   
