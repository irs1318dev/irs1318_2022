package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberArmTask extends CompositeOperationTask
{
    static DigitalOperation[] armPositionOperations =
    {
        DigitalOperation.ClimberArmOut,
        DigitalOperation.ClimberArmUp
    };

    public ClimberArmTask(boolean armUp)
    {
        super(
            0.1,
            armUp ? DigitalOperation.ClimberArmOut : DigitalOperation.ClimberArmUp,
            ClimberArmTask.armPositionOperations);
    }
}
