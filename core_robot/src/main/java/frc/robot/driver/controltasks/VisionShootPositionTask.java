package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;

public class VisionShootPositionTask extends VisionAdvanceAndCenterTaskBase
{
    private Double desiredDistance;

    public VisionShootPositionTask()
    {
        this(false);
    }

    public VisionShootPositionTask(boolean bestEffort)
    {
        super(false, false, bestEffort, false);
    }

    @Override
    protected double getDesiredDistance(double currentDistance)
    {
        if (this.desiredDistance == null)
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

            this.desiredDistance = TuningConstants.CARGO_KNOWN_SHOOTING_DISTANCES[desiredIndex];
        }

        return this.desiredDistance;
    }
}
