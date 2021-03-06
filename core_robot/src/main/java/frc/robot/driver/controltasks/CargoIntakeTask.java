package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class CargoIntakeTask extends CompositeOperationTask
{
    private static DigitalOperation[] intakeDirectionOperations =
    {
        DigitalOperation.CargoIntakeIn,
        DigitalOperation.CargoIntakeOut
    };

    public CargoIntakeTask(double time, boolean inDirection)
    {
        super(
            time,
            inDirection ? DigitalOperation.CargoIntakeIn : DigitalOperation.CargoIntakeOut,
            intakeDirectionOperations);
    }
}
