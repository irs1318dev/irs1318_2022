package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Auton testing *harru poyter says pog*
    AutoDriveBackIntakeShoot,
    AutoShootDriveBack,
    ThreeBallAuto,
    FiveBallAuto,
    AutoDriveBackIntakeDriveForwardShoot,

    // DriveTrain operations:
    PIDBrake,

    // Testing operations
    VisionCenterHub,
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
    AutoShootHigh,

    // Cargo macros
    CargoIntakeForceExtendMacro,
    CargoIntakeForceRetractMacro,
}
