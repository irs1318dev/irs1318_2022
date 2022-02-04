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
 * @author Will, kwen perper
 * @author FWJK
 */
@Singleton
public class ClimberMechanism implements IMechanism 
{
    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonFX winchMotor;
    
    private final IDoubleSolenoid passiveHookPiston;
    private final IDoubleSolenoid activeHookPiston;

    private int slotId;
    
    @Inject
    public ClimberMechanism(IDriver driver, LoggingManager logger, ITimer timer, IRobotProvider provider)
    {
        // housekeeping
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        // shooter
        this.winchMotor = provider.getTalonFX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_MASTER);
        this.winchMotor.setControlMode(TalonXControlMode.Velocity);
        this.winchMotor.setInvertOutput(HardwareConstants.CARGO_FLYWHEEL_MOTOR_INVERT_OUTPUT);
        this.winchMotor.setNeutralMode(MotorNeutralMode.Coast);
        this.winchMotor.setPIDF(
            TuningConstants.CLIMBER_WINCH_MOTOR_PID_KP, 
            TuningConstants.CLIMBER_WINCH_MOTOR_PID_KI, 
            TuningConstants.CLIMBER_WINCH_MOTOR_PID_KD, 
            TuningConstants.CLIMBER_WINCH_MOTOR_PID_KF, slotId);
        this.winchMotor.setVoltageCompensation(TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.CLIMBER_WINCH_MOTOR_MASTER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE);

        passiveHookPiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CLIMBER_PASSIVE_HOOK_FORWARD, ElectronicsConstants.CLIMBER_PASSIVE_HOOK_REVERSE);
        
        activeHookPiston = provider.getDoubleSolenoid(ElectronicsConstants.PCM_MODULE_A, PneumaticsModuleType.PneumaticsHub, ElectronicsConstants.CLIMBER_ACTIVE_HOOK_FORWARD, ElectronicsConstants.CLIMBER_ACTIVE_HOOK_REVERSE);
    }
    
    @Override
    public void readSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
    
}
