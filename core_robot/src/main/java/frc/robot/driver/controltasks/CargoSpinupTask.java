package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;

public class CargoSpinupTask extends TimedTask
{
    private double desiredSpinSpeed;
    
    public CargoSpinupTask(double desiredSpinSpeed)
    {
        super(10.0);

        this.desiredSpinSpeed = desiredSpinSpeed;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.desiredSpinSpeed);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.desiredSpinSpeed);
    }
}
