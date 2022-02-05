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
 * @author Will, kwen perper, FWJK,　ポッター・ハリ, harru poyter, 那噗难
 */
@Singleton
public class ClimberMechanism implements IMechanism 
{
    private static final int WeightedSlotId = 0;
    private static final int UnweightedSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonFX winchMotor;
    private final IDoubleSolenoid activeHookPiston;
    private final IDoubleSolenoid activeArmPiston;

    private double winchMotorPosition;
    private double winchMotorError;
    private boolean activeHookPistonUp;
    private boolean activeArmPistonUp;

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
        this.winchMotor.setVoltageCompensation(TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VOLTAGE_COMPENSATION_MAXVOLTAGE);

        ITalonFX winchFollowerMotor = provider.getTalonFX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_CAN_ID);
        winchFollowerMotor.follow(this.winchMotor);
        winchFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        winchFollowerMotor.setVoltageCompensation(TuningConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_POSITION_VOLTAGE_COMPENSATION_MAXVOLTAGE);
        winchFollowerMotor.setInvertOutput(HardwareConstants.CLIMBER_WINCH_MOTOR_FOLLOWER_INVERT_OUTPUT);

        this.activeHookPiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CLIMBER_ACTIVE_HOOK_FORWARD, ElectronicsConstants.CLIMBER_ACTIVE_HOOK_REVERSE);
        this.activeArmPiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CLIMBER_ACTIVE_ARM_FORWARD, ElectronicsConstants.CLIMBER_ACTIVE_ARM_REVERSE);

        this.activeHookPistonUp = false;
        this.activeArmPistonUp = false;

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
        double winchMotorPower = this.driver.getAnalog(AnalogOperation.winchMotorPower);
        if (winchMotorPower != 0.0)
        {
            this.winchMotor.setControlMode(TalonXControlMode.PercentOutput);
            this.winchMotor.set(winchMotorPower);
            this.logger.logNumber(LoggingKey.ClimberWinchPower, winchMotorPower);
        }
        else
        {
            double winchMotorDesiredPosition = this.driver.getAnalog(AnalogOperation.winchMotorPosition);
            this.winchMotor.setControlMode(TalonXControlMode.Position);
            this.winchMotor.setSelectedSlot(this.currentSlot);
            this.winchMotor.set(winchMotorDesiredPosition);
            this.logger.logNumber(LoggingKey.ClimberWinchDesiredPosition, winchMotorDesiredPosition);
        }

        if (this.driver.getDigital(DigitalOperation.ClimberEnableWeightedMode))
        {
            this.currentSlot = ClimberMechanism.WeightedSlotId;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberEnableUnWeightedMode))
        {
            this.currentSlot = ClimberMechanism.UnweightedSlotId;
        }

        if (this.driver.getDigital(DigitalOperation.ClimberHookUp))
        {
            this.activeHookPistonUp = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberHookDown))
        {
            this.activeHookPistonUp = false;
        }

        if (this.driver.getDigital(DigitalOperation.ClimberArmUp))
        {
            this.activeArmPistonUp = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberArmDown))
        {
            this.activeArmPistonUp = false;
        }

        this.activeHookPiston.set(this.activeHookPistonUp ? DoubleSolenoidValue.Forward : DoubleSolenoidValue.Reverse);
        this.activeArmPiston.set(this.activeArmPistonUp ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);

        this.logger.logBoolean(LoggingKey.ClimberArmOut, this.activeArmPistonUp);
        this.logger.logBoolean(LoggingKey.ClimberHookOut, this.activeHookPistonUp);
    }

    @Override
    public void stop()
    {
        this.activeHookPiston.set(DoubleSolenoidValue.Off);
        this.activeArmPiston.set(DoubleSolenoidValue.Off);
        this.winchMotor.stop();
    }
}
