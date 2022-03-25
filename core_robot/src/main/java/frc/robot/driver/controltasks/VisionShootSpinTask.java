package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionShootSpinTask extends TimedTask
{
    private static final int NO_TARGET_THRESHOLD = 40;

    private final boolean bestEffort;

    private OffboardVisionManager visionManager;

    private int noTargetCount;

    private boolean hasDeterminedSettings;
    private double spinUpSpeed;
    private boolean hoodUp;

    public VisionShootSpinTask(double duration, boolean bestEffort)
    {
        super(duration);

        this.bestEffort = bestEffort;

        this.noTargetCount = 0;
        this.hasDeterminedSettings = false;
        this.spinUpSpeed = 0.0;
        this.hoodUp = false;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        
        this.setDigitalOperationState(DigitalOperation.VisionDisableStream, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, true);
        this.setDigitalOperationState(DigitalOperation.VisionEnableGamePieceProcessing, false);
    }

    @Override
    public void update()
    {
        if (!this.hasDeterminedSettings)
        {
            Double currentDistance = this.visionManager.getVisionTargetDistance();
            if (currentDistance != null)
            {
                int desiredIndex = 0;
                double smallestDistanceOffset = currentDistance - TuningConstants.CARGO_KNOWN_SHOOTING_DISTANCES[0];
                for (int i = 1; i < TuningConstants.CARGO_KNOWN_SHOOTING_DISTANCES.length; i++)
                {
                    double newDistanceOffset = currentDistance - TuningConstants.CARGO_KNOWN_SHOOTING_DISTANCES[i];
                    if (Math.abs(newDistanceOffset) < Math.abs(smallestDistanceOffset))
                    {
                        smallestDistanceOffset = newDistanceOffset;
                        desiredIndex = i;
                    }
                }

                this.hasDeterminedSettings = true;
                this.spinUpSpeed = TuningConstants.CARGO_KNOWN_SHOOTING_FLYWHEEL_SPIN_SPEED[desiredIndex];
                this.hoodUp = TuningConstants.CARGO_KNOWN_SHOOTING_HOOD_UP[desiredIndex];
            }
        }

        if (this.hasDeterminedSettings)
        {
            this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, this.spinUpSpeed);
            this.setDigitalOperationState(DigitalOperation.CargoHoodPointBlank, !this.hoodUp);
            this.setDigitalOperationState(DigitalOperation.CargoHoodLong, this.hoodUp);
        }
    }

    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.CargoFlywheelVelocityGoal, 0.0);
        this.setDigitalOperationState(DigitalOperation.CargoHoodPointBlank, false);
        this.setDigitalOperationState(DigitalOperation.CargoHoodLong, false);
    }

    @Override
    public boolean shouldCancel()
    {
        if (this.bestEffort)
        {
            // note: in best-effort mode, this is done in hasCompleted() instead.
            return super.shouldCancel();
        }

        if (this.visionManager.getVisionTargetDistance() == null)
        {
            this.noTargetCount++;
        }
        else
        {
            this.noTargetCount = 0;
        }

        return this.noTargetCount >= VisionShootSpinTask.NO_TARGET_THRESHOLD || super.shouldCancel();
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.bestEffort)
        {
            if (this.visionManager.getVisionTargetDistance() == null)
            {
                this.noTargetCount++;

                return this.noTargetCount >= VisionShootSpinTask.NO_TARGET_THRESHOLD || super.hasCompleted();
            }

            this.noTargetCount = 0;
        }

        return super.hasCompleted();
    }
}
