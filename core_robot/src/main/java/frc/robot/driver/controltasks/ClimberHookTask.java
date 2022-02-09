package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberHookTask extends CompositeOperationTask {
    private static DigitalOperation[] hookPositionOperations = {
            DigitalOperation.ClimberHookUp,
            DigitalOperation.ClimberHookDown,
    };

    public ClimberHookTask(DigitalOperation toPerform) {
        super(0.1, toPerform, hookPositionOperations);
    }
}
