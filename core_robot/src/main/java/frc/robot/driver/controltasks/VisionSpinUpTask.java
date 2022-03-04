package frc.robot.driver.controltasks;

import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 * ASSUMES HOOD IS IN THE 80º POSITION
 */
public class VisionSpinUpTask extends TimedTask
{
    private static final int NO_CENTER_THRESHOLD = 40;
    private static final double SHOOTER_FLYWHEEL_INCHES_PER_SECOND_TO_PERCENTAGE = HardwareConstants.CARGO_FLYWHEEL_INCHES_PER_SECOND_TO_MOTOR_VELOCITY / TuningConstants.CARGO_FLYWHEEL_MOTOR_PID_KS;

    private final double shootAngleRadians = HardwareConstants.CARGO_SHOOTER_HIGH_ANGLE * Helpers.DEGREES_TO_RADIANS; // TODO hard coded value
    private final double relativeGoalHeight;

    private OffboardVisionManager visionManager;

    private int noCenterCount;

    /**
     * Initializes a new VisionSpinUpTask
     * @param upperHub whether or not we're aiming for the upper hub
     */
    public VisionSpinUpTask(boolean upperHub)
    {
        super(10.0);

        this.noCenterCount = 0;
        this.relativeGoalHeight = upperHub ? TuningConstants.RELATIVE_UPPER_HUB_HEIGHT : TuningConstants.RELATIVE_LOWER_HUB_HEIGHT;
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
            double quikmafs = 
                Math.sqrt(
                    (Math.pow(d, 2.0) * Helpers.GRAVITY_INCH_PER_SQ_SECOND) /
                        (d * Math.sin(2.0 * this.shootAngleRadians) - 2.0 * this.relativeGoalHeight * Math.pow(Math.cos(this.shootAngleRadians), 2.0)));
            quikmafs *= VisionSpinUpTask.SHOOTER_FLYWHEEL_INCHES_PER_SECOND_TO_PERCENTAGE;
            this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, quikmafs);
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