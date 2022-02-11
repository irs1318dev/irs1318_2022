package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Controlls intake, conveyor, and shooter
 * 
 * @author Will, kwen perper, perry the platypus, FWJK, harru poyter
 */
@Singleton
public class CargoMechanism implements IMechanism
{
    private static final int DefaultSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonSRX intakeMotor;
    private final ITalonSRX conveyorMotor;
    private final ITalonSRX feederMotor;
    private final ITalonFX flywheelMotor;

    private final IDoubleSolenoid innerHoodExtender;
    private final IDoubleSolenoid outerHoodExtender;
    private final IDoubleSolenoid intakeExtender;

    private final IAnalogInput feederThroughBeamSensor;
    private final IAnalogInput conveyorThroughBeamSensor;

    private double flywheelPosition;
    private double flywheelVelocity;
    private double flywheelError;

    private double feederSensorValue;
    private double conveyorSensorValue;
    private boolean feederBeamBroken;
    private boolean conveyorBeamBroken;

    private double lastIntakeTimeout;

    private double flywheelSetpoint;

    private enum ConveyorState
    {
        On,
        Off,
        Reverse,
        Advance
    };

    private ConveyorState currentConveyorState;

    @Inject
    public CargoMechanism(IDriver driver, LoggingManager logger, ITimer timer, IRobotProvider provider)
    {
        // housekeeping
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        // intake
        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.intakeMotor.setInvertOutput(HardwareConstants.CARGO_INTAKE_MOTOR_INVERT_OUTPUT);
        this.intakeMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.intakeExtender = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_MODULE_A,
            PneumaticsModuleType.PneumaticsHub,
            ElectronicsConstants.CARGO_INTAKE_PISTON_FORWARD,
            ElectronicsConstants.CARGO_INTAKE_PISTON_REVERSE);

        // shooter
        this.innerHoodExtender = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_MODULE_A,
            PneumaticsModuleType.PneumaticsHub,
            ElectronicsConstants.CARGO_INNER_HOOD_FORWARD,
            ElectronicsConstants.CARGO_INNER_HOOD_REVERSE);
        this.outerHoodExtender = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_MODULE_A,
            PneumaticsModuleType.PneumaticsHub,
            ElectronicsConstants.CARGO_OUTER_HOOD_FORWARD,
            ElectronicsConstants.CARGO_OUTER_HOOD_REVERSE);

        this.flywheelMotor = provider.getTalonFX(ElectronicsConstants.CARGO_FLYWHEEL_MOTOR_CAN_ID);
        this.flywheelMotor.setControlMode(TalonXControlMode.Velocity);
        this.flywheelMotor.setInvertOutput(HardwareConstants.CARGO_FLYWHEEL_MOTOR_INVERT_OUTPUT);
        this.flywheelMotor.setNeutralMode(MotorNeutralMode.Coast);
        this.flywheelMotor.setPIDF(
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KP,
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KI,
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KD,
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KF,
            CargoMechanism.DefaultSlotId);
        this.flywheelMotor.configureVelocityMeasurements(
            TuningConstants.CARGO_FLYWHEEL_VELOCITY_PERIOD,
            TuningConstants.CARGO_FLYWHEEL_VELOCITY_WINDOWSIZE);
        this.flywheelMotor.setVoltageCompensation(
            TuningConstants.CARGO_FLYWHEEL_MOTOR_MASTER_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.CARGO_FLYWHEEL_MOTOR_MASTER_VOLTAGE_COMPENSATION_MAXVOLTAGE);

        // serializer
        this.feederThroughBeamSensor = provider.getAnalogInput(ElectronicsConstants.CARGO_FEEDER_THROUGHBEAM_ANALOG_INPUT);
        this.conveyorThroughBeamSensor = provider.getAnalogInput(ElectronicsConstants.CARGO_CONVEYOR_THROUGHBEAM_ANALOG_INPUT);

        this.feederMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_FEEDER_MOTOR_CAN_ID);
        this.feederMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.feederMotor.setInvertOutput(HardwareConstants.CARGO_FEEDER_MOTOR_INVERT_OUTPUT);
        this.feederMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.conveyorMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_CONVEYOR_MOTOR_CAN_ID);
        this.conveyorMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.conveyorMotor.setInvertOutput(HardwareConstants.CARGO_CONVEYOR_MOTOR_INVERT_OUTPUT);
        this.conveyorMotor.setNeutralMode(MotorNeutralMode.Brake);
    }

    @Override
    public void readSensors()
    {
        this.flywheelPosition = this.flywheelMotor.getPosition();
        this.flywheelVelocity = this.flywheelMotor.getVelocity();
        this.flywheelError = this.flywheelMotor.getError();

        this.logger.logNumber(LoggingKey.CargoFlywheelPosition, this.flywheelPosition);
        this.logger.logNumber(LoggingKey.CargoFlywheelVelocity, this.flywheelVelocity);
        this.logger.logNumber(LoggingKey.CargoFlywheelError, this.flywheelError);

        this.feederSensorValue = this.feederThroughBeamSensor.getVoltage();
        this.conveyorSensorValue = this.conveyorThroughBeamSensor.getVoltage();

        this.logger.logNumber(LoggingKey.CargoFeederSensor, this.feederSensorValue);
        this.logger.logNumber(LoggingKey.CargoConveyerSensor, this.conveyorSensorValue);

        this.feederBeamBroken = this.feederSensorValue < TuningConstants.CARGO_FEEDER_THROUGHBEAM_CUTOFF;
        this.conveyorBeamBroken = this.conveyorSensorValue < TuningConstants.CARGO_CONVEYOR_THROUGHBEAM_CUTOFF;

        this.logger.logBoolean(LoggingKey.CargoFeederBeamBroken, this.feederBeamBroken);
        this.logger.logBoolean(LoggingKey.CargoConveyerBeamBroken, this.conveyorBeamBroken);
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();

        // extend and retract intake
        if (this.driver.getDigital(DigitalOperation.CargoIntakeExtend))
        {
            this.intakeExtender.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoIntakeRetract))
        {
            this.intakeExtender.set(DoubleSolenoidValue.Reverse);
        }

        // hood positions
        if (this.driver.getDigital(DigitalOperation.CargoHoodPointBlank))
        {
            this.innerHoodExtender.set(DoubleSolenoidValue.Reverse);
            this.outerHoodExtender.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoHoodShort))
        {
            this.innerHoodExtender.set(DoubleSolenoidValue.Forward);
            this.outerHoodExtender.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoHoodMedium))
        {
            this.innerHoodExtender.set(DoubleSolenoidValue.Reverse);
            this.outerHoodExtender.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoHoodLong))
        {
            this.innerHoodExtender.set(DoubleSolenoidValue.Forward);
            this.outerHoodExtender.set(DoubleSolenoidValue.Forward);
        }

        // feeder power
        if (this.driver.getDigital(DigitalOperation.CargoFeed))
        {
            this.feederMotor.set(TuningConstants.CARGO_FEEDER_POWER);
            this.currentConveyorState = ConveyorState.On;
        }
        else
        {
            this.feederMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
        }

        // control intake rollers
        if (this.driver.getDigital(DigitalOperation.CargoEject))
        {
            this.intakeMotor.set(TuningConstants.CARGO_INTAKE_EJECT_POWER);
            if (this.currentConveyorState == ConveyorState.Off || this.currentConveyorState == ConveyorState.Advance)
            {
                // if eject pushed then reverse conveyor
                this.currentConveyorState = ConveyorState.Reverse;
            }
        }
        else
        {
            if (this.currentConveyorState == ConveyorState.Reverse)
            {
                // eject must have been released
                this.currentConveyorState = ConveyorState.Off;
            }

            if (this.driver.getDigital(DigitalOperation.CargoIntakeIn))
            {
                this.intakeMotor.set(TuningConstants.CARGO_INTAKE_POWER);
                this.lastIntakeTimeout = currTime + TuningConstants.CONVEYOR_RUNTIME_AFTER_INTAKE;
                this.currentConveyorState = ConveyorState.Advance;
            }
            else if (this.driver.getDigital(DigitalOperation.CargoIntakeOut))
            {
                this.intakeMotor.set(TuningConstants.CARGO_INTAKE_OUT_POWER);
            }
            else
            {
                this.intakeMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
            }
        }

        if (conveyorBeamBroken && feederBeamBroken && this.currentConveyorState == ConveyorState.Advance)
        {
            this.currentConveyorState = ConveyorState.Off;
        }

        // after intake time
        if (this.currentConveyorState == ConveyorState.Advance &&
            currTime > this.lastIntakeTimeout)
        {
            this.currentConveyorState = ConveyorState.Off;
        }

        // stop reversing
        if (!this.driver.getDigital(DigitalOperation.CargoEject) &&
            this.currentConveyorState == ConveyorState.Reverse)
        {
            this.currentConveyorState = ConveyorState.Off;
        }

        // send next ball forward if it needs to be
        if (conveyorBeamBroken && !feederBeamBroken && this.currentConveyorState == ConveyorState.Off)
        {
            this.currentConveyorState = ConveyorState.Advance;
        }

        switch (this.currentConveyorState)
        {
            case On:
                this.conveyorMotor.set(TuningConstants.CARGO_CONVEYOR_FORWARD_POWER);
                break;

            case Advance:
                this.conveyorMotor.set(TuningConstants.CARGO_CONVEYOR_ADVANCE_POWER);
                break;

            case Reverse:
                this.conveyorMotor.set(TuningConstants.CARGO_CONVEYOR_REVERSE_POWER);
                break;

            case Off:
            default:
                this.conveyorMotor.stop();
                break;
        }

        // flywheel logic
        this.flywheelSetpoint = TuningConstants.PERRY_THE_PLATYPUS;
        if (this.driver.getAnalog(AnalogOperation.CargoFlywheelVelocityGoal) != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.flywheelSetpoint = Math.abs(this.driver.getAnalog(AnalogOperation.CargoFlywheelVelocityGoal) * TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KS);
        }

        if (this.flywheelSetpoint == TuningConstants.PERRY_THE_PLATYPUS)
        {
            this.flywheelMotor.stop();
        }
        else
        {
            this.flywheelMotor.set(this.flywheelSetpoint);
        }
    }

    @Override
    public void stop()
    {
        this.conveyorMotor.stop();
        this.flywheelMotor.stop();
        this.feederMotor.stop();
        this.intakeMotor.stop();
        this.intakeExtender.set(DoubleSolenoidValue.Off);
        this.innerHoodExtender.set(DoubleSolenoidValue.Off);
        this.outerHoodExtender.set(DoubleSolenoidValue.Off);
    }

    public double getFlywheelSetpoint()
    {
        return this.flywheelSetpoint;
    }

    public boolean isFlywheelSpunUp()
    {
        return this.flywheelSetpoint > 0.0 && Math.abs(this.flywheelError) <= TuningConstants.CARGO_FLYWHEEL_ALLOWABLE_ERROR_RANGE;
    }
}