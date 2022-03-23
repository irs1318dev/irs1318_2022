package frc.robot.driver.controltasks;

import frc.robot.driver.AutonomousRoutineSelector;
import frc.robot.driver.DigitalOperation;

public class CargoHoodChooseTask extends CompositeOperationTask
{
    private static DigitalOperation[] hoodOperations =
    {
        DigitalOperation.CargoHoodPointBlank,
        DigitalOperation.CargoHoodLong
    };

    public CargoHoodChooseTask()
    {
        super(
            0.1,
            DigitalOperation.CargoHoodPointBlank,
            CargoHoodChooseTask.hoodOperations);
    }

    @Override
    public void begin()
    {
        switch (this.getInjector().getInstance(AutonomousRoutineSelector.class).getHoodPosition())
        {
            case PointBlank:
                this.toPerform = DigitalOperation.CargoHoodPointBlank;
                break;

            default:
            case Long:
                this.toPerform = DigitalOperation.CargoHoodLong;
                break;
        }

        super.begin();
    }
}
