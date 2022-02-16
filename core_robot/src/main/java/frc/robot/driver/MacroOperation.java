package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDBrake,

    // Testing operations
    VisionCenter,
    VisionCenterAndAdvance,

    // Climber operations
    MoveBar,
    WinchForward,
    WinchBackward,

    // Auton *harru poyter says pog*
    AutoDriveBackIntakeShoot,
    AutoShootDriveBack,
}
