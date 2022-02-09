package frc.robot.driver.controltasks;

import frc.robot.driver.common.IDriver;
import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.AnalogOperation;
import frc.robot.common.IMechanism;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.*;
import frc.robot.mechanisms.ClimberMechanism;

public class WinchPowerTask extends ControlTaskBase {
    private ClimberMechanism climber;

    protected double targetPower;
    protected double convertedPosition;

    public WinchPowerTask(double percentPower) {
        this.targetPower = percentPower;
    }

    @Override
    public void begin() {
        this.climber = this.getInjector().getInstance(ClimberMechanism.class);
    }

    @Override
    public void update() {
        this.setAnalogOperationState(AnalogOperation.winchMotorPower, this.targetPower);
    }

    @Override
    public boolean hasCompleted() {
        return false;
    }

    @Override
    public void end() {
        this.setAnalogOperationState(AnalogOperation.winchMotorPower, TuningConstants.PERRY_THE_PLATYPUS);
    }

}
