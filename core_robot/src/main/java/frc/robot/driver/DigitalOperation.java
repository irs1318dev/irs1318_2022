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
    DriveTrainPositionMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,

    // cargo operations
    CargoIntakeExtend,
    CargoIntakeRetract,
    CargoIntakeIn,
    CargoIntakeOut,
    CargoEject,
    CargoFeed,
    CargoHoodPointBlank,
    CargoHoodShort,
    CargoHoodMedium,
    CargoHoodLong,

    // Climber operations 
    ClimberHookGrasp,
    ClimberHookRelease,
    ClimberArmOut,
    ClimberArmUp,
    ClimberEnableWeightedMode,
    ClimberEnableUnweightedMode,
    ClimberWinchLock,
    ClimberWinchUnlock,
}
