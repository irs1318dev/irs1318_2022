package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;

public class VisionShootPositionTask extends VisionAdvanceAndCenterTaskBase
{
    private Double desiredPosition;

    public VisionShootPositionTask()
    {
        this(true);
    }

    public VisionShootPositionTask(boolean bestEffort)
    {
        super(false, true, bestEffort);
    }

    @Override
    protected double getDesiredDistance()
    {
        if (this.desiredPosition == null)
        {
            double currentPosition = this.getDistance();
            int wantedIndex = 0;
            double lowestDistance = currentPosition - TuningConstants.CARGO_FLYWHEEL_KNOWN_DISTANCES[0];
            for (int i = 1; i < TuningConstants.CARGO_FLYWHEEL_KNOWN_DISTANCES.length; i++) {
                double newDistance = currentPosition - TuningConstants.CARGO_FLYWHEEL_KNOWN_DISTANCES[i];
                if (Math.abs(newDistance) < Math.abs(lowestDistance))
                {
                    lowestDistance = newDistance;
                    wantedIndex = i;
                }
            }

            this.desiredPosition = TuningConstants.CARGO_FLYWHEEL_KNOWN_DISTANCES[wantedIndex];
        }

        return this.desiredPosition;
    }
}
