package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Auton testing *harru poyter says pog*
    FollowPathTest1,
    FollowPathTest2,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,

    // Vision operations
    VisionCenterHub,
    VisionCenterCargo,
    VisionMoveToHub,
    VisionIntakeCargo,

    // Climber operations
    ClimberWinchForward,
    ClimberWinchBackward,

    ClimbSetUpWall,
    ClimbSetUpNotWall,
    ClimbSetUpManual,
    ClimberRiseToMidRung,
    ClimberExtendToNextRung,
    ClimberSwitchToNextRung,

    // shooting macros
    SpinUpFlywheel,
    ShootPointBlankHigh,
    ShootPointBlankLow,
    ShootTarmacHigh,
    AutoPositionAndShoot,
    AutoShootOnly,
    ChooseShoot,

    // Cargo macros
    CargoIntakeForceExtendMacro,
    CargoIntakeForceRetractMacro,
}
