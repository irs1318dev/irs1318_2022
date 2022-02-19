package frc.robot.driver;

import org.junit.jupiter.api.Test;

import frc.robot.driver.common.ButtonMapVerifier;

public class ButtonMapTests
{
    @Test
    public void verifyButtonMap()
    {
        //MAKE TRUE LATE PLXZWSC
        ButtonMapVerifier.Verify(new ButtonMap(), false);
    }
}