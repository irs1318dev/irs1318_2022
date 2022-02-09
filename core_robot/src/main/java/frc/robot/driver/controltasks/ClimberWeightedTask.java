package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberWeightedTask extends CompositeOperationTask
{
    private static DigitalOperation[] weightOperations =
    {
        DigitalOperation.ClimberEnableUnweightedMode,
        DigitalOperation.ClimberEnableWeightedMode,
    };

    public ClimberWeightedTask(boolean weighted)
    {
        super(
            0.1,
            weighted ? DigitalOperation.ClimberEnableWeightedMode : DigitalOperation.ClimberEnableUnweightedMode,
            ClimberWeightedTask.weightOperations);
    }

}
