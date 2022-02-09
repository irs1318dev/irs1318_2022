package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberArmTask extends CompositeOperationTask {
    private static DigitalOperation[] armPositionOperations = {
            DigitalOperation.ClimberArmUp,
            DigitalOperation.ClimberArmDown
    };

    public ClimberArmTask(DigitalOperation toPerform) {
        super(0.1, toPerform, armPositionOperations);
    }

}
