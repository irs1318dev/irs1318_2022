package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;
import frc.robot.driver.AutonomousRoutineSelector;

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

        this.desiredSpinSpeed = this.getInjector().getInstance(AutonomousRoutineSelector.class).getShooterSpeed();

        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.desiredSpinSpeed);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.desiredSpinSpeed);
    }
}
