package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberWinchLockTask extends CompositeOperationTask
{
    static DigitalOperation[] armPositionOperations =
    {
        DigitalOperation.ClimberWinchLock,
        DigitalOperation.ClimberWinchUnlock
    };

    public ClimberWinchLockTask(boolean lock)
    {
        super(
            0.1,
            lock ? DigitalOperation.ClimberWinchLock : DigitalOperation.ClimberWinchUnlock,
            ClimberArmTask.armPositionOperations);
    }
}
