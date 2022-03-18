package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.CargoMechanism;

public class CargoShootTask extends ControlTaskBase
{
    private final boolean shootAll;

    private CargoMechanism cargo;
    private ITimer timer;

    private enum ShootingState
    {
        CheckBall,
        SpinningUp,
        Shooting,
        Completed
    };

    private ShootingState currentState;
    private double endTime;
    private int shotsRemaining;

    private boolean useShootAnywayMode;

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

        this.useShootAnywayMode = this.cargo.useShootAnywayMode();
        if (this.useShootAnywayMode)
        {
            this.currentState = ShootingState.CheckBall;
            this.endTime = this.timer.get() + TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIME;
            this.shotsRemaining = this.shootAll ? 2 : 1;
        }
        else
        {
            this.currentState = ShootingState.Shooting;
            this.endTime = this.timer.get() + TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME;
            this.shotsRemaining = 1;
        }
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();
        if (this.currentState == ShootingState.CheckBall)
        {
            if (this.cargo.hasBallReadyToShoot())
            {
                this.currentState = ShootingState.SpinningUp;
                this.endTime = currTime + TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME;
            }
            else if (currTime >= this.endTime)
            {
                this.currentState = ShootingState.Completed;
            }
        }

        if (this.currentState == ShootingState.SpinningUp)
        {
            if (this.cargo.isFlywheelSpunUp() ||
                (this.useShootAnywayMode && currTime >= this.endTime))
            {
                this.currentState = ShootingState.Shooting;
                if (this.useShootAnywayMode)
                {
                    this.endTime = currTime + TuningConstants.CARGO_SHOOT_WAIT_TIME;
                }
            }
            else if (!this.useShootAnywayMode && currTime > this.endTime)
            {
                this.currentState = ShootingState.Completed;
            }
        }

        if (this.currentState == ShootingState.Shooting &&
            (!this.cargo.hasBallReadyToShoot() ||
                (this.useShootAnywayMode && currTime >= this.endTime)))
        {
            this.shotsRemaining--;
            if (this.shotsRemaining > 0 && this.cargo.hasBackupBallToShoot()) 
            {
                this.currentState = ShootingState.CheckBall;
                this.endTime = currTime + TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIME;
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
        this.setDigitalOperationState(DigitalOperation.CargoFeed, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentState == ShootingState.Completed;
    }
}
