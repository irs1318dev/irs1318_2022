package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Controlls climber and stuff
 * 
 * @author Will, kwen perper, FWJK, harru poyter
 */
@Singleton
public class ClimberMechanism implements IMechanism
{
    private static final int UnweightedSlotId = 0;
    private static final int WeightedSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;

    private final ITalonFX winchMotor;
    // private final IDoubleSolenoid activeHookPiston;
    // private final IDoubleSolenoid activeArmPiston;
    // private final IDoubleSolenoid winchArmLock;
    // private final IDigitalInput winchArmRetractedLimitSwitch;

    private double winchMotorPosition;
    private double winchMotorError;
    private boolean winchRetracted;

    private boolean activeHookGrasped;
    private boolean activeArmOut;
    private boolean winchArmLocked;

    private double desiredWinchPosition;
    private int currentSlot;

    @Inject
    public ClimberMechanism(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        // housekeeping
        this.driver = driver;
        this.logger = logger;

        // winch
        this.winchMotor = provider.getTalonFX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_MASTER_CAN_ID);
        this.winchMotor.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.winchMotor.setInvert(HardwareConstants.CLIMBER_WINCH_MOTOR_MASTER_INVERT);
        this.winchMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.winchMotor.setVoltageCompensation(
            TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_MAXVOLTAGE);
        if (TuningConstants.CLIMBER_USE_MOTION_MAGIC)
        {
            this.winchMotor.setControlMode(TalonXControlMode.MotionMagicPosition);
            this.winchMotor.setMotionMagicPIDF(
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PIDVA_KP,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PIDVA_KI,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PIDVA_KD,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PIDVA_KF,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PIDVA_KV,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PIDVA_KA,
                ClimberMechanism.UnweightedSlotId);
            this.winchMotor.setMotionMagicPIDF(
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PIDVA_KP,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PIDVA_KI,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PIDVA_KD,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PIDVA_KF,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PIDVA_KV,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PIDVA_KA,
                ClimberMechanism.WeightedSlotId);
            this.winchMotor.setSelectedSlot(ClimberMechanism.UnweightedSlotId);
        }
        else if (TuningConstants.CLIMBER_USE_PID)
        {
            this.winchMotor.setControlMode(TalonXControlMode.Position);
            this.winchMotor.setPIDF(
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PID_KP,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PID_KI,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PID_KD,
                TuningConstants.CLIMBER_WINCH_MOTOR_U_PID_KF,
                ClimberMechanism.UnweightedSlotId);
            this.winchMotor.setPIDF(
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PID_KP,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PID_KI,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PID_KD,
                TuningConstants.CLIMBER_WINCH_MOTOR_W_PID_KF,
                ClimberMechanism.WeightedSlotId);
            this.winchMotor.setSelectedSlot(ClimberMechanism.UnweightedSlotId);
        }
        else
        {
            this.winchMotor.setControlMode(TalonXControlMode.PercentOutput);
        }

        ITalonFX winchFollowerMotor = provider.getTalonFX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_CAN_ID);
        winchFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        winchFollowerMotor.setVoltageCompensation(
            TuningConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_POSITION_VOLTAGE_COMPENSATION_MAXVOLTAGE);
        winchFollowerMotor.setInvert(HardwareConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_INVERT);
        winchFollowerMotor.follow(this.winchMotor);

        // this.activeHookPiston =
        //     provider.getDoubleSolenoid(
        //         ElectronicsConstants.PNEUMATICS_MODULE_A,
        //         ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //         ElectronicsConstants.CLIMBER_ACTIVE_HOOK_FORWARD,
        //         ElectronicsConstants.CLIMBER_ACTIVE_HOOK_REVERSE);
        // this.activeArmPiston =
        //     provider.getDoubleSolenoid(
        //         ElectronicsConstants.PNEUMATICS_MODULE_A,
        //         ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //         ElectronicsConstants.CLIMBER_ACTIVE_ARM_FORWARD,
        //         ElectronicsConstants.CLIMBER_ACTIVE_ARM_REVERSE);
        // this.winchArmLock =
        //     provider.getDoubleSolenoid(
        //         ElectronicsConstants.PNEUMATICS_MODULE_A,
        //         ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //         ElectronicsConstants.CLIMBER_WINCH_LOCK_FORWARD,
        //         ElectronicsConstants.CLIMBER_WINCH_LOCK_BACKWARD);

        // this.winchArmRetractedLimitSwitch = provider.getDigitalInput(ElectronicsConstants.CLIMBER_WINCH_ARM_RETRACTED_LIMIT_SWITCH_DIGITAL_INPUT);

        this.activeHookGrasped = false;
        this.activeArmOut = false;
        this.winchRetracted = false;

        this.currentSlot = ClimberMechanism.UnweightedSlotId;
        this.winchMotorError = 0.0;
    }

    @Override
    public void readSensors()
    {
        // this.winchRetracted = this.winchArmRetractedLimitSwitch.get();
        if (this.winchRetracted)
        {
            this.winchMotor.reset();
        }

        this.logger.logBoolean(LoggingKey.ClimberWinchRetracted, this.winchRetracted);

        this.winchMotorPosition = this.winchMotor.getPosition();
        this.winchMotorError = this.winchMotor.getError();

        this.logger.logNumber(LoggingKey.ClimberWinchPosition, this.winchMotorPosition);
        this.logger.logNumber(LoggingKey.ClimberWinchError, this.winchMotorError);
    }

    @Override
    public void update()
    {
        // set pid for when the climber is hanging
        int prevSlot = this.currentSlot;
        if (this.driver.getDigital(DigitalOperation.ClimberEnableWeightedMode))
        {
            this.currentSlot = ClimberMechanism.WeightedSlotId;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberEnableUnweightedMode))
        {
            this.currentSlot = ClimberMechanism.UnweightedSlotId;
        }

        // set class variable for if active hook is enabled
        if (this.driver.getDigital(DigitalOperation.ClimberHookGrasp))
        {
            this.activeHookGrasped = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberHookRelease))
        {
            this.activeHookGrasped = false;
        }

        // set class variable for if arm piston is extended or retracted
        if (this.driver.getDigital(DigitalOperation.ClimberArmOut))
        {
            this.activeArmOut = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberArmUp))
        {
            this.activeArmOut = false;
        }
        
        // set position of lock solenoid
        if (this.driver.getDigital(DigitalOperation.ClimberWinchLock))
        {
            this.winchArmLocked = true;
        }
        else if(this.driver.getDigital(DigitalOperation.ClimberWinchUnlock))
        {
            this.winchArmLocked = false;
        }

        if (this.driver.getDigital(DigitalOperation.ClimberResetWinchPosition))
        {
            this.winchMotor.reset();
        }

        // manually set winch power for debug mode
        double winchMotorPower = this.driver.getAnalog(AnalogOperation.ClimberWinchMotorPower);
        if (winchMotorPower != TuningConstants.PERRY_THE_PLATYPUS)
        {
            this.winchMotor.setControlMode(TalonXControlMode.PercentOutput);
            this.winchMotor.set(winchMotorPower);
            this.desiredWinchPosition = this.winchMotorPosition;

            this.logger.logNumber(LoggingKey.ClimberWinchDesiredPosition, this.desiredWinchPosition);
        }
        else if (!this.winchArmLocked)
        {
            // otherwise, set winch position to desired position
            this.desiredWinchPosition = this.driver.getAnalog(AnalogOperation.ClimberWinchDesiredPosition);
            this.desiredWinchPosition *= HardwareConstants.CLIMBER_WINCH_MAX_POSITION;
            if (prevSlot != this.currentSlot)
            {
                this.winchMotor.setSelectedSlot(this.currentSlot);
            }

            this.winchMotor.setControlMode(TalonXControlMode.Position);
            this.winchMotor.set(this.desiredWinchPosition);

            this.logger.logNumber(LoggingKey.ClimberWinchDesiredPosition, this.desiredWinchPosition);
        }
        else
        {
            // if the winch arm is locked, don't attempt to control the winch
            this.winchMotor.stop();

            this.logger.logNumber(LoggingKey.ClimberWinchDesiredPosition, TuningConstants.MAGIC_NULL_VALUE);
        }

        // this.activeHookPiston.set(this.activeHookGrasped ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);
        // this.activeArmPiston.set(this.activeArmOut ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);
        // this.winchArmLock.set(this.winchArmLocked ? DoubleSolenoidValue.Forward : DoubleSolenoidValue.Reverse);

        this.logger.logBoolean(LoggingKey.ClimberWinchLockIn, this.winchArmLocked);
        this.logger.logBoolean(LoggingKey.ClimberArmOut, this.activeArmOut);
        this.logger.logBoolean(LoggingKey.ClimberHookGrasped, this.activeHookGrasped);
    }

    @Override
    public void stop()
    {
        // Intentionally skipped (don't drop the Robot!): this.activeHookPiston.set(DoubleSolenoidValue.Off);
        // this.activeArmPiston.set(DoubleSolenoidValue.Off);
        // this.winchArmLock.set(DoubleSolenoidValue.Off);
        this.winchMotor.stop();
    }

    // accessor method for winch position
    public double getCurrentPos()
    {
        return this.winchMotorPosition / HardwareConstants.CLIMBER_WINCH_MAX_POSITION;
    }
}
