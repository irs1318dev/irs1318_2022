package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 * ASSUMES HOOD IS IN THE 80º POSITION
 */
public class VisionSpinUpTask extends TimedTask
{
    private static final int NO_CENTER_THRESHOLD = 40;

    private OffboardVisionManager visionManager;

    private int noCenterCount;

    private double relativeGoalHeight = TuningConstants.GOAL_HEIGHT - TuningConstants.SHOOTER_HEIGHT;
    private double shootAngle = 80 * Helpers.DEGREES_TO_RADIANS; // TODO hard coded value
    private double quikmafs;


    /**
     * Initializes a new VisionSpinUpTask
     */
    public VisionSpinUpTask()
    {
        super(10);

        this.noCenterCount = 0;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        Double d = this.getDistance();
        if (d != null)
        {
            // trajectory equations: https://www.desmos.com/calculator/kuuwtrpau7 
            // ft/s CHANGE
            this.quikmafs = Math.sqrt((Math.pow(d, 2) * Helpers.GRAVITY)/(d * Math.sin(2 * this.shootAngle) - 2 * this.relativeGoalHeight * Math.pow(Math.cos(this.shootAngle), 2)));
            this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.quikmafs);
        }
    }

    /**
     * Checks whether this task should be stopped, or whether it should continue being processed.
     * @return true if we should cancel this task (and stop performing any subsequent tasks), otherwise false (to keep processing this task)
     */
    @Override
    public boolean shouldCancel()
    {
        if (this.getDistance() == null)
        {
            this.noCenterCount++;
        }
        else
        {
            this.noCenterCount = 0;
        }

        return this.noCenterCount >= VisionSpinUpTask.NO_CENTER_THRESHOLD;
    }

    protected Double getDistance()
    {
        return this.visionManager.getVisionTargetDistance();
    }
}