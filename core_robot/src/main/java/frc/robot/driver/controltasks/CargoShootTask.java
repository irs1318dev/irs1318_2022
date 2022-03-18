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
    private double notBeforeTime;
    private double timeoutTime;
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

        double currTime = this.timer.get();
        this.useShootAnywayMode = this.cargo.useShootAnywayMode();
        if (this.useShootAnywayMode)
        {
            this.currentState = ShootingState.CheckBall;
            this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_CHECKBALL_MIN_WAIT_TIME;
            this.timeoutTime = currTime + TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIMEOUT;
            this.shotsRemaining = this.shootAll ? 2 : 1;
        }
        else
        {
            this.currentState = ShootingState.Shooting;
            this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_SPINUP_MIN_WAIT_TIME;
            this.timeoutTime = currTime + TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIMEOUT;
        }
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();

        // don't change states before our not-before time has passed
        if (currTime >= this.notBeforeTime)
        {
            if (this.useShootAnywayMode)
            {
                if (this.currentState == ShootingState.CheckBall)
                {
                    this.currentState = ShootingState.SpinningUp;
                    this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIMEOUT;
                }

                if (this.currentState == ShootingState.SpinningUp)
                {
                    this.currentState = ShootingState.Shooting;
                    this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_WAIT_TIMEOUT;
                }

                if (this.currentState == ShootingState.Shooting)
                {
                    this.currentState = ShootingState.Completed;
                }
            }
            else
            {
                if (this.currentState == ShootingState.CheckBall)
                {
                    if (this.cargo.hasBallReadyToShoot())
                    {
                        this.currentState = ShootingState.SpinningUp;
                        this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_SPINUP_MIN_WAIT_TIME;
                        this.timeoutTime = currTime + TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIMEOUT;
                    }
                    else if (currTime >= this.timeoutTime)
                    {
                        this.currentState = ShootingState.Completed;
                    }
                }

                if (this.currentState == ShootingState.SpinningUp)
                {
                    if (this.cargo.isFlywheelSpunUp())
                    {
                        this.currentState = ShootingState.Shooting;
                        this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_MIN_WAIT_TIME;
                        this.timeoutTime = currTime + TuningConstants.CARGO_SHOOT_WAIT_TIMEOUT;
                    }
                    else if (currTime > this.timeoutTime)
                    {
                        this.currentState = ShootingState.Completed;
                    }
                }

                if (this.currentState == ShootingState.Shooting && !this.cargo.hasBallReadyToShoot())
                {
                    this.shotsRemaining--;
                    if (this.shotsRemaining > 0 && this.cargo.hasBackupBallToShoot()) 
                    {
                        this.currentState = ShootingState.CheckBall;
                        this.notBeforeTime = currTime + TuningConstants.CARGO_SHOOT_CHECKBALL_MIN_WAIT_TIME;
                        this.timeoutTime = currTime + TuningConstants.CARGO_SHOOT_CHECKBALL_WAIT_TIMEOUT;
                    }
                    else 
                    {
                        this.currentState = ShootingState.Completed;
                    }
                }
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
