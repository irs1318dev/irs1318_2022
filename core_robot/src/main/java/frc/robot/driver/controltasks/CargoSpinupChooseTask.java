package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;
import frc.robot.driver.SmartDashboardSelectionManager;

public class CargoSpinupChooseTask extends TimedTask
{
    private double desiredSpinSpeed;

    public CargoSpinupChooseTask()
    {
        super(10.0);
    }

    @Override
    public void begin()
    {
        super.begin();

        SmartDashboardSelectionManager selectionManager = this.getInjector().getInstance(SmartDashboardSelectionManager.class);
        this.desiredSpinSpeed = selectionManager.getSelectedShooterSpeed();

        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.desiredSpinSpeed);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.desiredSpinSpeed);
    }
}