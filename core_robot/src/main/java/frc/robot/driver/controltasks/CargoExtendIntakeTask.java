package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class CargoExtendIntakeTask extends CompositeOperationTask
{
    private static DigitalOperation[] intakePositionOperations =
    {
        DigitalOperation.CargoIntakeExtend,
        DigitalOperation.CargoIntakeRetract
    };

    public CargoExtendIntakeTask(boolean intakeDown)
    {
        super(
            0.1,
            intakeDown ? DigitalOperation.CargoIntakeExtend : DigitalOperation.CargoIntakeRetract,
            CargoExtendIntakeTask.intakePositionOperations);
    }
}
