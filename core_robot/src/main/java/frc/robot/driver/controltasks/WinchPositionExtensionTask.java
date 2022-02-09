package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.ClimberMechanism;

public class WinchPositionExtensionTask extends ControlTaskBase
{
    private final double extendPercent;

    private ClimberMechanism climber;

    public WinchPositionExtensionTask(double extendPercent)
    {
        this.extendPercent = extendPercent;
    }

    @Override
    public void begin()
    {
        this.climber = this.getInjector().getInstance(ClimberMechanism.class);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.ClimberWinchDesiredPosition, this.extendPercent);
    }

    @Override
    public void end()
    {
    }

    @Override
    public boolean hasCompleted()
    {
        double currentPos = this.extendPercent - climber.getCurrentPos();
        return Math.abs(currentPos) < TuningConstants.CLIMBER_WINCH_POSITION_EXTEND_ACCEPTABLE_DELTA;
    }
}
