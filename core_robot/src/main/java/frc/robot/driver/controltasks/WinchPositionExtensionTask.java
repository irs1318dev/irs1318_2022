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

public class WinchPositionExtensionTask extends ControlTaskBase {
    private ClimberMechanism climber;

    protected double targetLength;
    protected double convertedPosition;

    public WinchPositionExtensionTask(double percentExtend) {
        this.targetLength = percentExtend;
    }

    @Override
    public void begin() {
        this.climber = this.getInjector().getInstance(ClimberMechanism.class);
    }

    @Override
    public void update() {
        this.setAnalogOperationState(AnalogOperation.winchMotorPosition, this.targetLength);
    }

    @Override
    public boolean hasCompleted() {
        double currentPos = this.targetLength - climber.getCurrentPos();
        if ((Math.abs(currentPos) < 0.5)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end() {
    }

}
