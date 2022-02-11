package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;

public class ClimberWinchPowerTask extends ControlTaskBase
{
    private double targetPower;

    public ClimberWinchPowerTask(double percentPower)
    {
        this.targetPower = percentPower;
    }

    @Override
    public void begin()
    {
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.ClimberWinchMotorPower, this.targetPower);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ClimberWinchMotorPower, TuningConstants.PERRY_THE_PLATYPUS);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
