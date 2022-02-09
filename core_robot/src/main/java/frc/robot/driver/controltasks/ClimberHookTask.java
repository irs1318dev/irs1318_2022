package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberHookTask extends CompositeOperationTask
{
    private static DigitalOperation[] hookPositionOperations =
    {
        DigitalOperation.ClimberHookGrasp,
        DigitalOperation.ClimberHookRelease,
    };

    public ClimberHookTask(boolean grasp)
    {
        super(
            0.1,
            grasp ? DigitalOperation.ClimberHookGrasp : DigitalOperation.ClimberHookRelease,
            ClimberHookTask.hookPositionOperations);
    }
}
