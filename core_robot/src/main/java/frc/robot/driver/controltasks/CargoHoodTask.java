package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class CargoHoodTask extends CompositeOperationTask
{
    private static DigitalOperation[] hoodOperations =
    {
        DigitalOperation.CargoHoodPointBlank,
        DigitalOperation.CargoHoodShort,
        DigitalOperation.CargoHoodMedium,
        DigitalOperation.CargoHoodLong
    };

    public CargoHoodTask(DigitalOperation hoodOperation)
    {
        super(
            0.1,
            hoodOperation,
            CargoHoodTask.hoodOperations);
    }
}
