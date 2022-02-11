package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class CargoExtendIntakeTask extends CompositeOperationTask
{
    private static DigitalOperation[] armPositionOperations =
    {
        
    };
    public CargoAutonExtendIntakeTask(boolean armUp)
    {
        super(
            0.1,
            armUp ? DigitalOperation.ClimberArmOut : DigitalOperation.ClimberArmUp,
            ClimberArmTask.armPositionOperations);
    }
}
