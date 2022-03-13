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

    // private final IDoubleSolenoid intakeExtender;
    // private final IDoubleSolenoid hoodExtender;

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

        // this.intakeExtender =
        //     provider.getDoubleSolenoid(
        //         ElectronicsConstants.PNEUMATICS_MODULE_A,
        //         ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //         ElectronicsConstants.CARGO_INTAKE_PISTON_FORWARD,
        //         ElectronicsConstants.CARGO_INTAKE_PISTON_REVERSE);

        // // shooter
        // this.hoodExtender =
        //     provider.getDoubleSolenoid(
        //         ElectronicsConstants.PNEUMATICS_MODULE_A,
        //         ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
        //         ElectronicsConstants.CARGO_HOOD_FORWARD,
        //         ElectronicsConstants.CARGO_HOOD_REVERSE);

        this.flywheelMotor = provider.getTalonFX(ElectronicsConstants.CARGO_FLYWHEEL_MOTOR_CAN_ID);
        this.flywheelMotor.setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        this.flywheelMotor.setControlMode(TalonXControlMode.Velocity);
        this.flywheelMotor.setInvert(HardwareConstants.CARGO_FLYWHEEL_MOTOR_INVERT);
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

        ITalonFX flywheelFollower = provider.getTalonFX(ElectronicsConstants.CARGO_FLYWHEEL_FOLLOWER_MOTOR_CAN_ID);
        flywheelFollower.setInvert(HardwareConstants.CARGO_FLYWHEEL_FOLLOWER_MOTOR_INVERT);
        flywheelFollower.setNeutralMode(MotorNeutralMode.Coast);
        flywheelFollower.follow(this.flywheelMotor);

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

        this.currentConveyorState = ConveyorState.Off;
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

        this.feederBeamBroken = false; // this.feederSensorValue < TuningConstants.CARGO_FEEDER_THROUGHBEAM_CUTOFF;
        this.conveyorBeamBroken = this.conveyorSensorValue < TuningConstants.CARGO_CONVEYOR_THROUGHBEAM_CUTOFF;

        this.logger.logBoolean(LoggingKey.CargoFeederBeamBroken, this.feederBeamBroken);
        this.logger.logBoolean(LoggingKey.CargoConveyerBeamBroken, this.conveyorBeamBroken);
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();

        // // extend and retract intake
        // if (this.driver.getDigital(DigitalOperation.CargoIntakeExtend))
        // {
        //     this.intakeExtender.set(DoubleSolenoidValue.Forward);
        // }
        // else if (this.driver.getDigital(DigitalOperation.CargoIntakeRetract))
        // {
        //     this.intakeExtender.set(DoubleSolenoidValue.Reverse);
        // }

        // // hood positions
        // if (this.driver.getDigital(DigitalOperation.CargoHoodPointBlank))
        // {
        //     this.hoodExtender.set(DoubleSolenoidValue.Reverse);
        // }
        // else if (this.driver.getDigital(DigitalOperation.CargoHoodLong))
        // {
        //     this.hoodExtender.set(DoubleSolenoidValue.Forward);
        // }

        // feeder power
        if (this.driver.getDigital(DigitalOperation.CargoFeed))
        {
            this.feederMotor.set(TuningConstants.CARGO_FEEDER_POWER);
        }
        else
        {
            this.feederMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
        }

        // control intake rollers
        double intakePower = TuningConstants.PERRY_THE_PLATYPUS;
        if (this.driver.getDigital(DigitalOperation.CargoEject))
        {
            intakePower = TuningConstants.CARGO_INTAKE_EJECT_POWER;
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
                intakePower = TuningConstants.CARGO_INTAKE_POWER;
                this.lastIntakeTimeout = currTime + TuningConstants.CONVEYOR_RUNTIME_AFTER_INTAKE;
                this.currentConveyorState = ConveyorState.Advance;
            }
            else if (this.driver.getDigital(DigitalOperation.CargoIntakeOut))
            {
                intakePower = TuningConstants.CARGO_INTAKE_OUT_POWER;
            }
            else
            {
                intakePower = TuningConstants.PERRY_THE_PLATYPUS;
            }
        }

        this.intakeMotor.set(intakePower);
        this.logger.logNumber(LoggingKey.CargoIntakePower, intakePower);

        if (this.conveyorBeamBroken &&
            this.feederBeamBroken &&
            this.currentConveyorState == ConveyorState.Advance)
        {
            this.currentConveyorState = ConveyorState.Off;
        }

        // after intake time
        if (this.currentConveyorState == ConveyorState.Advance &&
            currTime > this.lastIntakeTimeout)
        {
            this.currentConveyorState = ConveyorState.Off;
        }

        // send next ball forward if it needs to be
        if (this.conveyorBeamBroken &&
            !this.feederBeamBroken &&
            this.currentConveyorState == ConveyorState.Off)
        {
            this.currentConveyorState = ConveyorState.Advance;
        }

        switch (this.currentConveyorState)
        {
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

        this.logger.logString(LoggingKey.CargoConveyorState, this.currentConveyorState.toString());

        // flywheel logic
        double flywheelMotorPower = this.driver.getAnalog(AnalogOperation.CargoFlywheelMotorPower);
        double flywheelVelocityGoal = this.driver.getAnalog(AnalogOperation.CargoFlywheelVelocityGoal);
        if (flywheelMotorPower != TuningConstants.PERRY_THE_PLATYPUS)
        {
            this.flywheelSetpoint = this.flywheelVelocity;
            this.flywheelMotor.setControlMode(TalonXControlMode.PercentOutput);
            this.flywheelMotor.set(flywheelMotorPower);
            this.logger.logNumber(LoggingKey.CargoFlywheelPower, flywheelMotorPower);
        }
        else if (flywheelVelocityGoal != TuningConstants.PERRY_THE_PLATYPUS)
        {
            this.flywheelSetpoint = flywheelVelocityGoal * TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KS;
            this.flywheelMotor.setControlMode(TalonXControlMode.Velocity);
            this.flywheelMotor.set(this.flywheelSetpoint);
            this.logger.logNumber(LoggingKey.CargoFlywheelPower, -1318.0);
        }
        else
        {
            this.flywheelSetpoint = TuningConstants.PERRY_THE_PLATYPUS;
            this.flywheelMotor.stop();
            this.logger.logNumber(LoggingKey.CargoFlywheelPower, TuningConstants.PERRY_THE_PLATYPUS);
        }

        this.logger.logNumber(LoggingKey.CargoFlywheelDesiredVelocity, this.flywheelSetpoint);
    }

    @Override
    public void stop()
    {
        this.conveyorMotor.stop();
        this.flywheelMotor.stop();
        this.feederMotor.stop();
        this.intakeMotor.stop();
        // this.intakeExtender.set(DoubleSolenoidValue.Off);
        // this.hoodExtender.set(DoubleSolenoidValue.Off);
    }

    public double getFlywheelSetpoint()
    {
        return this.flywheelSetpoint;
    }

    public boolean isFlywheelSpunUp()
    {
        return this.flywheelSetpoint > 0.0 && Math.abs(this.flywheelError) <= TuningConstants.CARGO_FLYWHEEL_ALLOWABLE_ERROR_RANGE;
    }

    public boolean isConveyorSensorBlocked()
    {
        return this.conveyorBeamBroken;
    }

    public boolean isFeederSensorBlocked()
    {
        return this.feederBeamBroken;
    }
}
