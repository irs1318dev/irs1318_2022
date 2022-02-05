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
    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonSRX intakeMotor;
    private final ITalonSRX flywheelMotor;
    private final ITalonSRX conveyorMotor;
    private final ITalonSRX feederMotor;
    
    private final IDoubleSolenoid innerHoodPiston;
    private final IDoubleSolenoid outerHoodPiston;
    private final IDoubleSolenoid intakePiston;

    private final IAnalogInput feederSensor;
    private final IAnalogInput conveyorSensor;

    private double flywheelPosition;
    private double flywheelVelocity;
    private double flywheelError;

    private double feederSensorValue;
    private double conveyorSensorValue;

    private double intakeTimer;

    private int slotId = 0;

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

        this.intakePiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CARGO_INTAKE_PISTON_FORWARD, ElectronicsConstants.CARGO_INTAKE_PISTON_REVERSE);

        // shooter
        this.innerHoodPiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CARGO_INNER_HOOD_FORWARD, ElectronicsConstants.CARGO_INNER_HOOD_REVERSE);
        this.outerHoodPiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CARGO_OUTER_HOOD_FORWARD, ElectronicsConstants.CARGO_OUTER_HOOD_REVERSE);

        this.flywheelMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_FLYWHEEL_MOTOR_CAN_ID);
        this.flywheelMotor.setControlMode(TalonXControlMode.Velocity);
        this.flywheelMotor.setInvertOutput(HardwareConstants.CARGO_FLYWHEEL_MOTOR_INVERT_OUTPUT);
        this.flywheelMotor.setNeutralMode(MotorNeutralMode.Coast);
        this.flywheelMotor.setPIDF(
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KP, 
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KI, 
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KD, 
            TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KF, slotId);
        this.flywheelMotor.configureVelocityMeasurements(TuningConstants.CARGO_FLYWHEEL_VELOCITY_PERIOD, TuningConstants.CARGO_FLYWHEEL_VELOCITY_WINDOWSIZE);
        this.flywheelMotor.setVoltageCompensation(TuningConstants.CARGO_FLYWHEEL_MOTOR_MASTER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.CARGO_FLYWHEEL_MOTOR_MASTER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE);

        // serializer
        this.feederSensor= provider.getAnalogInput(ElectronicsConstants.CARGO_FEEDER_THROUGHBEAM_ANALOG_INPUT);
        this.conveyorSensor = provider.getAnalogInput(ElectronicsConstants.CARGO_CONVEYOR_THROUGHBEAM_ANALOG_INPUT);

        this.feederMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_FEEDER_MOTOR_CAN_ID);
        this.feederMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.feederMotor.setInvertOutput(HardwareConstants.CARGO_FEEDER_MOTOR_INVERT_OUTPUT);
        this.feederMotor.setNeutralMode(MotorNeutralMode.Coast);  // brake?

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

        this.feederSensorValue = this.feederSensor.getVoltage();
        this.conveyorSensorValue = this.conveyorSensor.getVoltage();
    }

    @Override
    public void update() 
    {
        // extend and retract intake 
        if (this.driver.getDigital(DigitalOperation.CargoIntakeExtend))
        {
            this.intakePiston.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoIntakeRetract))
        {
            this.intakePiston.set(DoubleSolenoidValue.Reverse);
        }

        // hood positions
        if (this.driver.getDigital(DigitalOperation.CargoHoodPointBlank))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Reverse);
            this.outerHoodPiston.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoHoodShort))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Forward);
            this.outerHoodPiston.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoHoodMedium))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Reverse);
            this.outerHoodPiston.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.CargoHoodLong))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Forward);
            this.outerHoodPiston.set(DoubleSolenoidValue.Forward);
        }

        // feeder power
        if (this.driver.getDigital(DigitalOperation.CargoFeed) && this.flywheelSetpoint != TuningConstants.PERRY_THE_PLATYPUS) 
        {
            this.feederMotor.set(TuningConstants.CARGO_FEEDER_POWER);
            this.currentConveyorState = ConveyorState.On;
        }
        else
        {
            this.feederMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
        }

        // control intake rollers
        if (this.driver.getDigital(DigitalOperation.CargoIntakeIn))
        {
            this.intakeMotor.set(TuningConstants.CARGO_INTAKE_OUTTAKE_POWER);
            this.intakeTimer = this.timer.get() + TuningConstants.CONVEYOR_RUNTIME_AFTER_INTAKE;
            this.currentConveyorState = ConveyorState.Advance;
        }
        else if (this.driver.getDigital(DigitalOperation.CargoIntakeOut) || this.driver.getDigital(DigitalOperation.CargoEject))
        {
            this.intakeMotor.set(-TuningConstants.CARGO_INTAKE_OUTTAKE_POWER);
        }
        else
        {
            this.intakeMotor.set(TuningConstants.PERRY_THE_PLATYPUS);
        }

        boolean conveyorBeamBroken = this.conveyorSensorValue < TuningConstants.CARGO_THROUGHBEAM_CUTOFF;
        boolean feederBeamBroken = this.feederSensorValue < TuningConstants.CARGO_THROUGHBEAM_CUTOFF;
        
        if (conveyorBeamBroken && feederBeamBroken && this.currentConveyorState == ConveyorState.Advance) 
        {
            this.currentConveyorState = ConveyorState.Off;
        }
        //after intake time
        if (this.timer.get() > this.intakeTimer && this.currentConveyorState == ConveyorState.Advance) 
        {
            this.currentConveyorState = ConveyorState.Off;
        }
        // if eject pushed then reverse conveyor
        if (this.driver.getDigital(DigitalOperation.CargoEject) && 
        (this.currentConveyorState == ConveyorState.Off || this.currentConveyorState == ConveyorState.Advance)) {
            this.currentConveyorState = ConveyorState.Reverse;
        }
        //stop reversing
        if (!this.driver.getDigital(DigitalOperation.CargoEject) && this.currentConveyorState == ConveyorState.Reverse) {
            this.currentConveyorState = ConveyorState.Off;
        }
        //send next ball forward if it needs to be
        if (conveyorBeamBroken && !feederBeamBroken && this.currentConveyorState == ConveyorState.Off) {
            this.currentConveyorState = ConveyorState.Advance;
        }

        if (this.currentConveyorState == ConveyorState.Advance || this.currentConveyorState == ConveyorState.On)
        {
            this.conveyorMotor.set(TuningConstants.CARGO_CONVEYOR_POWER);
        }
        else if (this.currentConveyorState == ConveyorState.Reverse)
        {
            this.conveyorMotor.set(-TuningConstants.CARGO_CONVEYOR_POWER);
        }
        else
        {
            this.conveyorMotor.stop();
        }

        // flywheel logic
        this.flywheelSetpoint = TuningConstants.PERRY_THE_PLATYPUS;
        //spin up flywheel
        if (this.driver.getAnalog(AnalogOperation.CargoFlywheelVelocity) == TuningConstants.MAGIC_NULL_VALUE &&
            this.driver.getDigital(DigitalOperation.CargoFlywheelSpinup)) 
        {
            this.flywheelSetpoint = TuningConstants.CARGO_FLYWHEEL_SPINUP_SPEED;
        }
        else if (this.driver.getAnalog(AnalogOperation.CargoFlywheelVelocity) != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.flywheelSetpoint = Math.abs(this.driver.getAnalog(AnalogOperation.CargoFlywheelVelocity) * TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KS);
        }
        else 
        {
            this.flywheelSetpoint = TuningConstants.PERRY_THE_PLATYPUS;
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
        this.intakePiston.set(DoubleSolenoidValue.Off);
        this.innerHoodPiston.set(DoubleSolenoidValue.Off);
        this.outerHoodPiston.set(DoubleSolenoidValue.Off);
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
