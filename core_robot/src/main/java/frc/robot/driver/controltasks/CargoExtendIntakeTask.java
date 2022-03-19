package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class CargoExtendIntakeTask extends CompositeOperationTask
{
    private static DigitalOperation[] intakePositionOperations =
    {
        DigitalOperation.CargoIntakeForceExtend,
        DigitalOperation.CargoIntakeForceRetract
    };

    public CargoExtendIntakeTask(boolean intakeDown)
    {
        super(
            0.1,
            intakeDown ? DigitalOperation.CargoIntakeForceExtend : DigitalOperation.CargoIntakeForceRetract,
            CargoExtendIntakeTask.intakePositionOperations);
    }
}
