package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.common.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    public enum AutoRoutine
    {
        None,
        AutoShootDriveBack,
        ShootDriveBack,
        ShootLowDriveBack,
        WillThreeBallAuto,
        WillTwoBallAuto,
        PravinThreeBallAuto,
        PravinFourBallAuto,
        WinavTwoBallAuto,
        PraTwoBallAll,
        PraTwoBallMid,
        PNWThreeBall,
        tester
    }

    public enum HoodPosition
    {
        PointBlank,
        Short,
        Medium,
        Long
    }

    private final ISendableChooser<AutoRoutine> routineChooser;
    private final ISendableChooser<HoodPosition> hoodChooser;
    private final IDoubleSubscriber shooterSpeedSlider;

    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("1 Ball Autoshoot", AutoRoutine.ShootDriveBack);
        this.routineChooser.addObject("1 Ball Point-Blank", AutoRoutine.ShootDriveBack);
        this.routineChooser.addObject("1 Low Ball Point-Blank", AutoRoutine.ShootLowDriveBack);
        this.routineChooser.addObject("Will's 3-Ball Auto", AutoRoutine.WillThreeBallAuto);
        this.routineChooser.addObject("Will's 2-Ball Auto", AutoRoutine.WillTwoBallAuto);
        this.routineChooser.addObject("Pravin's 3-Ball Auto", AutoRoutine.PravinThreeBallAuto);
        this.routineChooser.addObject("Pravin's 4-Ball Auto", AutoRoutine.PravinFourBallAuto);
        this.routineChooser.addObject("Winav's 2-Ball Auto", AutoRoutine.WinavTwoBallAuto);
        this.routineChooser.addObject("Pra's 2-Ball Auto All", AutoRoutine.PraTwoBallAll);
        this.routineChooser.addObject("Pra's 2-Ball Auto Mid", AutoRoutine.PraTwoBallMid);
        this.routineChooser.addObject("PNW Pravin's 3-Ball Auto", AutoRoutine.PNWThreeBall);
        this.routineChooser.addObject("Tester", AutoRoutine.tester);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        this.hoodChooser = networkTableProvider.getSendableChooser();
        this.hoodChooser.addDefault("PointBlank", HoodPosition.PointBlank);
        this.hoodChooser.addObject("Short", HoodPosition.Short);
        this.hoodChooser.addObject("Medium", HoodPosition.Medium);
        this.hoodChooser.addObject("Long", HoodPosition.Long);
        networkTableProvider.addChooser("Hood position", this.hoodChooser);

        this.shooterSpeedSlider = networkTableProvider.getNumberSlider("Shooter Speed", 0.0);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    public HoodPosition getSelectedHoodPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.hoodChooser, HoodPosition.PointBlank);
    }

    public double getSelectedShooterSpeed()
    {
        return this.shooterSpeedSlider.get(0.0);
    }

    private static <T> T GetSelectedOrDefault(ISendableChooser<T> chooser, T defaultValue)
    {
        T selected = chooser.getSelected();
        if (selected == null)
        {
            selected = defaultValue;
        }

        return selected;
    }
}

