package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Controlls climber and stuff
 * 
 * @author Will, kwen perper, FWJK, harru poyter
 */
@Singleton
public class ClimberSammamishMechanism implements IMechanism
{
    private final IDriver driver;

    private final IDoubleSolenoid hookPiston;

    @Inject
    public ClimberSammamishMechanism(IDriver driver, IRobotProvider provider)
    {
        // housekeeping
        this.driver = driver;

        this.hookPiston = provider.getDoubleSolenoid(
            ElectronicsConstants.PNEUMATICS_MODULE_A,
            ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A,
            ElectronicsConstants.SCLIMBER_ACTIVE_HOOK_FORWARD,
            ElectronicsConstants.SCLIMBER_ACTIVE_HOOK_BACKWARD);
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.SClimberForward))
        {
            this.hookPiston.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.SClimberReverse))
        {
            this.hookPiston.set(DoubleSolenoidValue.Reverse);
        }
    }

    @Override
    public void stop()
    {
        // Intentionally skipped (don't drop the Robot!): this.hookPiston.set(DoubleSolenoidValue.Off);
    }
}
