package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.CargoMechanism;

public class CargoShootTask extends ControlTaskBase
{
    private final boolean shootAll;
    private CargoMechanism cargo;
    private int shotCount;

    private enum ShootingState
    {
        CheckBall,
        SpinningUp,
        Shooting,
        Completed
    };

    private ShootingState currentState;

    private ITimer timer;
    private double endTime;

    public CargoShootTask()
    {
        this(true);
    }

    public CargoShootTask(boolean shootAll)
    {
        this.shootAll = shootAll;
    }

    @Override
    public void begin()
    {
        this.cargo = this.getInjector().getInstance(CargoMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.endTime = this.timer.get() + TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIME;

        this.currentState = ShootingState.CheckBall;
        this.shotCount = 0;
    }

    @Override
    public void update()
    {
        if (this.currentState == ShootingState.CheckBall)
        {
            if (this.cargo.hasBallReadyToShoot())
            {
                this.currentState = ShootingState.SpinningUp;
                this.endTime = timer.get() + TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME;
            }
            else if (this.timer.get() > this.endTime)
            {
                this.currentState = ShootingState.Completed;
            }
        }

        if (this.currentState == ShootingState.SpinningUp)
        {
            if (this.cargo.isFlywheelSpunUp())
            {
                this.currentState = ShootingState.Shooting;
            }
            else if (this.timer.get() > this.endTime)
            {
                this.currentState = ShootingState.Completed;
            }
        }

        if (this.currentState == ShootingState.Shooting && !this.cargo.hasBallReadyToShoot()) 
        {
            this.shotCount++;
            if (this.shootAll && this.cargo.hasBackupBallToShoot() && this.shotCount < 2) 
            {
                this.currentState = ShootingState.CheckBall;
                this.endTime = this.timer.get() + TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIME;
            }
            else 
            {
                this.currentState = ShootingState.Completed;
            }
        }

        // set operations based on states
        switch (this.currentState)
        {
            case CheckBall:
            case SpinningUp:
            case Completed:
                this.setDigitalOperationState(DigitalOperation.CargoFeed, false);
                break;

            case Shooting:
                this.setDigitalOperationState(DigitalOperation.CargoFeed, true);
                break;
        }
    }

    @Override
    public void end()
    {
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentState == ShootingState.Completed;
    }
}
