package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.driver.SmartDashboardSelectionManager;

public class CargoHoodChooseTask extends CompositeOperationTask
{
    private static DigitalOperation[] hoodOperations =
    {
        DigitalOperation.CargoHoodPointBlank,
        DigitalOperation.CargoHoodShort,
        DigitalOperation.CargoHoodMedium,
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
        SmartDashboardSelectionManager selectionManager = this.getInjector().getInstance(SmartDashboardSelectionManager.class);

        DigitalOperation toPerform;
        switch (selectionManager.getSelectedHoodPosition())
        {
            case PointBlank:
                toPerform = DigitalOperation.CargoHoodPointBlank;
                break;

            case Short:
                toPerform = DigitalOperation.CargoHoodShort;
                break;

            case Medium:
                toPerform = DigitalOperation.CargoHoodMedium;
                break;

            default:
            case Long:
                toPerform = DigitalOperation.CargoHoodLong;
                break;
        }

        this.setToPerform(toPerform);

        super.begin();
    }
}
