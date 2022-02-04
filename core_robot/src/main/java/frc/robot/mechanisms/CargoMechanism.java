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
 * @author Will, kwen perper, perry the platypus, FWJK
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

    private int slotId = 0;
    
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
    public void readSensors() {
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
    public void update() {
        if (this.driver.getDigital(DigitalOperation.CargoIntakeExtend))
        {
            this.intakePiston.set(DoubleSolenoidValue.Forward);
        }
        if (this.driver.getDigital(DigitalOperation.CargoIntakeRetract))
        {
            this.intakePiston.set(DoubleSolenoidValue.Reverse);
        }
        if (this.driver.getDigital(DigitalOperation.CargoIntakeIn))
        {
            this.intakeMotor.set(TuningConstants.CARGO_INTAKE_OUTTAKE_POWER);
        }
        if (this.driver.getDigital(DigitalOperation.CargoIntakeOut))
        {
            this.intakeMotor.set(-TuningConstants.CARGO_INTAKE_OUTTAKE_POWER);
        }

        // hood positions
        if (this.driver.getDigital(DigitalOperation.CargoHoodPointBlank))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Reverse);
            this.outerHoodPiston.set(DoubleSolenoidValue.Reverse);
        }
        if (this.driver.getDigital(DigitalOperation.CargoHoodShort))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Forward);
            this.outerHoodPiston.set(DoubleSolenoidValue.Reverse);
        }
        if (this.driver.getDigital(DigitalOperation.CargoHoodMedium))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Reverse);
            this.outerHoodPiston.set(DoubleSolenoidValue.Forward);
        }
        if (this.driver.getDigital(DigitalOperation.CargoHoodLong))
        {
            this.innerHoodPiston.set(DoubleSolenoidValue.Forward);
            this.outerHoodPiston.set(DoubleSolenoidValue.Forward);
        }

        // feeder power
        if(this.driver.getDigital(DigitalOperation.CargoFeed)) 
        {
            this.feederMotor.set(TuningConstants.CARGO_FEEDER_POWER);
        }

        //this.intakeMotor.set(Tuningoocnstants.cargointakemotorpower)



        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
    
}
