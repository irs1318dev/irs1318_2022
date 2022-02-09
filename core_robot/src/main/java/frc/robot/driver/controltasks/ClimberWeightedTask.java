package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberWeightedTask extends CompositeOperationTask {
    private static DigitalOperation[] weightOperations = {
            DigitalOperation.ClimberEnableUnWeightedMode,
            DigitalOperation.ClimberEnableWeightedMode,
    };

    public ClimberWeightedTask(DigitalOperation toPerform) {
        super(0.1, toPerform, weightOperations);
    }

}
