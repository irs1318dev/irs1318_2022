package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Auton testing *harru poyter says pog*
    FollowPathTest1,
    FollowPathTest2,
    AutoDriveBackIntakeShoot,
    AutoShootDriveBack,
    ThreeBallAuto,
    FiveBallAuto,
    AutoDriveBackIntakeDriveForwardShoot,

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
    ShootPointBlankHigh,
    ShootPointBlankLow,
    ShootTarmacHigh,
    AutoPositionAndShoot,
    AutoShootOnly,

    // Cargo macros
    CargoIntakeForceExtendMacro,
    CargoIntakeForceRetractMacro,
}
