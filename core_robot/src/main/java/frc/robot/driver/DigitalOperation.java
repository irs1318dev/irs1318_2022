package frc.robot.driver;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,

    // Vision operations:
    VisionForceDisable,
    VisionDisableStream,
    VisionEnableGamePieceProcessing,
    VisionEnableRetroreflectiveProcessing,

    // Compressor operations:
    CompressorForceDisable,

    // DriveTrain operations:
    DriveTrainPathMode,
    DriveTrainSteerMode,
    DriveTrainMaintainPositionMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,

    // cargo operations
    CargoIntakeForceExtend,
    CargoIntakeForceRetract,
    CargoIntakeIn,
    CargoIntakeOut,
    CargoEject,
    CargoForceIntakeOnly,
    CargoFeed,
    CargoHoodPointBlank,
    CargoHoodShort,
    CargoHoodMedium,
    CargoHoodLong,
    CargoEnableShootAnywayMode,
    CargoDisableShootAnywayMode,

    // Climber operations 
    ClimberHookGrasp,
    ClimberHookRelease,
    ClimberArmOut,
    ClimberArmUp,
    ClimberEnableWeightedMode,
    ClimberEnableUnweightedMode,
    ClimberWinchLock,
    ClimberWinchUnlock,
    ClimberResetWinchPosition,

    //Samamamamish Climber
    SClimberUp,
    SClimberDown,
}
