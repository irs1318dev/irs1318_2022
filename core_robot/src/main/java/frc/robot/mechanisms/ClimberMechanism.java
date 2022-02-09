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
    private final ITimer timer;

    private final ITalonFX winchMotor;
    private final IDoubleSolenoid activeHookPiston;
    private final IDoubleSolenoid activeArmPiston;

    private double winchMotorPosition;
    private double winchMotorError;
    private boolean activeHookGrasped;
    private boolean activeArmOut;

    private double desiredWinchPosition;
    private int currentSlot;

    @Inject
    public ClimberMechanism(IDriver driver, LoggingManager logger, ITimer timer, IRobotProvider provider)
    {
        // housekeeping
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        // winch
        this.winchMotor = provider.getTalonFX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_MASTER_CAN_ID);
        this.winchMotor.setControlMode(TalonXControlMode.Position);
        this.winchMotor.setInvertOutput(HardwareConstants.CLIMBER_WINCH_MOTOR_MASTER_INVERT_OUTPUT);
        this.winchMotor.setNeutralMode(MotorNeutralMode.Brake);
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
        this.winchMotor.setVoltageCompensation(
            TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_MAXVOLTAGE);
        this.winchMotor.setSelectedSlot(ClimberMechanism.UnweightedSlotId);

        ITalonFX winchFollowerMotor = provider.getTalonFX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_CAN_ID);
        winchFollowerMotor.follow(this.winchMotor);
        winchFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        winchFollowerMotor.setVoltageCompensation(
            TuningConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_POSITION_VOLTAGE_COMPENSATION_MAXVOLTAGE);
        winchFollowerMotor.setInvertOutput(HardwareConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_INVERT_OUTPUT);

        this.activeHookPiston = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_MODULE_A,
            PneumaticsModuleType.PneumaticsHub,
            ElectronicsConstants.CLIMBER_ACTIVE_HOOK_FORWARD,
            ElectronicsConstants.CLIMBER_ACTIVE_HOOK_REVERSE);
        this.activeArmPiston = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_MODULE_A,
            PneumaticsModuleType.PneumaticsHub,
            ElectronicsConstants.CLIMBER_ACTIVE_ARM_FORWARD,
            ElectronicsConstants.CLIMBER_ACTIVE_ARM_REVERSE);

        this.activeHookGrasped = false;
        this.activeArmOut = false;

        this.currentSlot = ClimberMechanism.UnweightedSlotId;
        this.winchMotorError = 0.0;
    }

    @Override
    public void readSensors()
    {
        this.winchMotorPosition = this.winchMotor.getPosition();
        this.winchMotorError = this.winchMotor.getError();
        this.logger.logNumber(LoggingKey.ClimberWinchPosition, this.winchMotorPosition);
        this.logger.logNumber(LoggingKey.ClimberWinchError, this.winchMotorError);
    }

    @Override
    public void update()
    {
        double winchMotorPower = this.driver.getAnalog(AnalogOperation.ClimberWinchMotorPower);
        if (winchMotorPower != TuningConstants.PERRY_THE_PLATYPUS)
        {
            this.winchMotor.setControlMode(TalonXControlMode.PercentOutput);
            this.winchMotor.set(winchMotorPower);
            this.desiredWinchPosition = this.winchMotorPosition;
            this.logger.logNumber(LoggingKey.ClimberWinchDesiredPosition, this.desiredWinchPosition);
            this.logger.logNumber(LoggingKey.ClimberWinchPower, winchMotorPower);
        }
        else
        {
            this.desiredWinchPosition = this.driver.getAnalog(AnalogOperation.ClimberWinchDesiredPosition);
            this.winchMotor.setControlMode(TalonXControlMode.Position);
            this.winchMotor.setSelectedSlot(this.currentSlot);
            this.winchMotor.set(this.desiredWinchPosition);
            this.logger.logNumber(LoggingKey.ClimberWinchDesiredPosition, this.desiredWinchPosition);
            this.logger.logNumber(LoggingKey.ClimberWinchPower, -1318.0);
        }

        if (this.driver.getDigital(DigitalOperation.ClimberEnableWeightedMode))
        {
            this.currentSlot = ClimberMechanism.WeightedSlotId;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberEnableUnweightedMode))
        {
            this.currentSlot = ClimberMechanism.UnweightedSlotId;
        }

        if (this.driver.getDigital(DigitalOperation.ClimberHookGrasp))
        {
            this.activeHookGrasped = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberHookRelease))
        {
            this.activeHookGrasped = false;
        }

        if (this.driver.getDigital(DigitalOperation.ClimberArmOut))
        {
            this.activeArmOut = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberArmUp))
        {
            this.activeArmOut = false;
        }

        this.activeHookPiston.set(this.activeHookGrasped ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);
        this.activeArmPiston.set(this.activeArmOut ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);

        this.logger.logBoolean(LoggingKey.ClimberArmOut, this.activeArmOut);
        this.logger.logBoolean(LoggingKey.ClimberHookGrasped, this.activeHookGrasped);
    }

    @Override
    public void stop()
    {
        // this.activeHookPiston.set(DoubleSolenoidValue.Off);
        this.activeArmPiston.set(DoubleSolenoidValue.Off);
        this.winchMotor.stop();
    }

    public double getCurrentPos()
    {
        return this.winchMotor.getPosition();
    }
}
