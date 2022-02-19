package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberArmUnlockTask extends CompositeOperationTask
{
    private static DigitalOperation[] lockedOperations =
    {
        DigitalOperation.ClimberWinchLock,
        DigitalOperation.ClimberWinchUnlock,
    };

    public ClimberArmUnlockTask(boolean unlock)
    {
        super(
            0.1,
            unlock ? DigitalOperation.ClimberWinchUnlock : DigitalOperation.ClimberWinchLock,
            ClimberArmUnlockTask.lockedOperations);
    }

}
