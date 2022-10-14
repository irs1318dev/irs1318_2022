package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class RumbleTask extends CompositeOperationTask
{
    private static DigitalOperation[] rumbleOperations =
    {
        DigitalOperation.ForceLightDriverRumble,
    };

    public RumbleTask()
    {
        super(
            0.1,
            DigitalOperation.ForceLightDriverRumble,
            RumbleTask.rumbleOperations);
    }
}
