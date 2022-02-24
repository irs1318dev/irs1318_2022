package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.common.Helpers;
import frc.robot.driver.common.*;
import frc.robot.driver.common.buttons.*;
import frc.robot.driver.common.descriptions.*;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_16),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test2Debug,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_MAX_VELOCITY),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_MAX_VELOCITY),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnAngleGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            AnalogAxis.XBONE_RSY,
            Shift.None, // Shift.DriverDebug,
            Shift.None,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            1.0,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        // new AnalogOperationDescription(
        //     AnalogOperation.DriveTrainTurnSpeed,
        //     UserInputDevice.Driver,
        //     AnalogAxis.XBONE_RSX,
        //     Shift.DriverDebug,
        //     Shift.DriverDebug,
        //     !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive, as counter-clockwise is positive
        //     -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
        //     TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinLeft,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // make left positive, as counter-clockwise is positive
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),

        // new AnalogOperationDescription(
        //     AnalogOperation.DriveTrainRotationA,
        //     UserInputDevice.Driver,
        //     AnalogAxis.XBONE_LT,
        //     ElectronicsConstants.INVERT_TRIGGER_AXIS,
        //     -TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
        //     TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
        //     TuningConstants.DRIVETRAIN_ROTATION_A_MULTIPLIER),
        // new AnalogOperationDescription(
        //     AnalogOperation.DriveTrainRotationB,
        //     UserInputDevice.Driver,
        //     AnalogAxis.XBONE_RT,
        //     ElectronicsConstants.INVERT_TRIGGER_AXIS,
        //     -TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
        //     TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
        //     TuningConstants.DRIVETRAIN_ROTATION_B_MULTIPLIER),

        // cargo mechanism testing operations
        new AnalogOperationDescription(
            AnalogOperation.CargoFlywheelVelocityGoal,
            UserInputDevice.Test1,
            AnalogAxis.XBONE_LSX,
            Shift.Test1Debug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -0.1,
            0.1),
        new AnalogOperationDescription(
            AnalogOperation.CargoFlywheelMotorPower,
            UserInputDevice.Test1,
            AnalogAxis.XBONE_LSX,
            Shift.Test1Debug,
            Shift.Test1Debug,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -0.1,
            0.1),

        // climber mechanism testing operations
        new AnalogOperationDescription(
            AnalogOperation.ClimberWinchDesiredPosition,
            UserInputDevice.Test1,
            AnalogAxis.XBONE_RSX,
            Shift.Test1Debug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            -1.0,
            0.1),
        new AnalogOperationDescription(
            AnalogOperation.ClimberWinchMotorPower,
            UserInputDevice.Test1,
            AnalogAxis.XBONE_RSX,
            Shift.Test1Debug,
            Shift.Test1Debug,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            -0.1,
            0.1),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),

        // vision operations
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
            Shift.None,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableGamePieceProcessing,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
            Shift.None,
            Shift.None,
            ButtonType.Simple),

        // cargo opertaions:
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeExtend,
            UserInputDevice.Driver,
            0,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeRetract,
            UserInputDevice.Driver,
            180,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeIn,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoEject,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple),

        // climber operations
        new DigitalOperationDescription(
            DigitalOperation.ClimberHookRelease,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ClimberHookGrasp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ClimberArmUp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ClimberArmOut,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),

        // cargo testing operations
        new DigitalOperationDescription(
            DigitalOperation.CargoFeed,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),

        // indicator light testing operations
        new DigitalOperationDescription(
            DigitalOperation.IndicatorLightA,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            ButtonType.Toggle),
        new DigitalOperationDescription(
            DigitalOperation.IndicatorLightB,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            ButtonType.Toggle),
        new DigitalOperationDescription(
            DigitalOperation.IndicatorLightC,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            ButtonType.Toggle),    
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
            UserInputDevice.Driver,
            180, // DPAD-down
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple,
            () -> new PIDBrakeTask(),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenter,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                    new VisionCenteringTask(false),
                    new DriveTrainFieldOrientationModeTask(true)),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterAndAdvance,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                    new VisionAdvanceAndCenterTask(false, true),
                    new DriveTrainFieldOrientationModeTask(true)),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),

        // climber control macros for codriver
        new MacroOperationDescription(
            MacroOperation.ClimberWinchForward,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Simple,
            () -> new ClimberWinchPowerTask(0.5),
            new IOperation[]
            {
                AnalogOperation.ClimberWinchMotorPower,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberWinchBackward,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Simple,
            () -> new ClimberWinchPowerTask(-0.5),
            new IOperation[]
            {
                AnalogOperation.ClimberWinchMotorPower
            }),

        // Climbing macros:
        new MacroOperationDescription(
            MacroOperation.ClimbSetUp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("linUpUnder1stClimberBar"),
                new ClimberArmUnlockTask(true),
                ConcurrentTask.AllTasks(
                    new ClimberArmTask(true),
                    new ClimberWeightedTask(false)
                ),
                new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH),
                new FollowPathTask("goForward5in")
            ),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.ClimberHookGrasp,
                DigitalOperation.ClimberHookRelease,
                DigitalOperation.ClimberArmOut,
                DigitalOperation.ClimberArmUp,
                DigitalOperation.ClimberEnableWeightedMode,
                DigitalOperation.ClimberEnableUnweightedMode,
                DigitalOperation.ClimberWinchLock,
                DigitalOperation.ClimberWinchUnlock
            }
        ),

        new MacroOperationDescription(
            MacroOperation.ClimberRiseToMidRung,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ClimberHookTask(false),
                    new ClimberWeightedTask(true)
                ),
                new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_RETRACT_LENGTH),
                new ClimberHookTask(true)
            ),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.ClimberHookGrasp,
                DigitalOperation.ClimberHookRelease,
                DigitalOperation.ClimberArmOut,
                DigitalOperation.ClimberArmUp,
                DigitalOperation.ClimberEnableWeightedMode,
                DigitalOperation.ClimberEnableUnweightedMode,
                DigitalOperation.ClimberWinchLock,
                DigitalOperation.ClimberWinchUnlock
            }
        ),

        new MacroOperationDescription(
            MacroOperation.ClimberExtendToNextRung,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ClimberWeightedTask(false),
                new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_SHORT_EXTEND_LENGTH),
                new ClimberArmTask(false),
                new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH),
                new ClimberArmTask(true),
                new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_MOSTLY_EXTEND_LENGTH)
            ),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.ClimberHookGrasp,
                DigitalOperation.ClimberHookRelease,
                DigitalOperation.ClimberArmOut,
                DigitalOperation.ClimberArmUp,
                DigitalOperation.ClimberEnableWeightedMode,
                DigitalOperation.ClimberEnableUnweightedMode,
                DigitalOperation.ClimberWinchLock,
                DigitalOperation.ClimberWinchUnlock
            }
        ),

        new MacroOperationDescription(
            MacroOperation.ClimberSwitchToNextRung,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ClimberWeightedTask(true),
                ConcurrentTask.AllTasks(
                    new ClimberHookTask(false),
                    new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_RETRACT_LENGTH)
                ),
                new ClimberHookTask(true)
            ),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.ClimberHookGrasp,
                DigitalOperation.ClimberHookRelease,
                DigitalOperation.ClimberArmOut,
                DigitalOperation.ClimberArmUp,
                DigitalOperation.ClimberEnableWeightedMode,
                DigitalOperation.ClimberEnableUnweightedMode,
                DigitalOperation.ClimberWinchLock,
                DigitalOperation.ClimberWinchUnlock
            }
        ),

        // autonomous testing operations
        new MacroOperationDescription(
            MacroOperation.AutoDriveBackIntakeShoot,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new FollowPathTask("goForward4ft"),
                    new CargoExtendIntakeTask(true),
                    new CargoIntakeTask(true)
                ),
                new FollowPathTask("goBack7ftRotate"),
                // vision centering
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(3000.0),
                    SequentialTask.Sequence(
                        new WaitTask(1.0), // how long it take to spin
                        new CargoShootTask(),
                        new WaitTask(1.0),
                        new CargoShootTask()
                    )
                )
            ),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.CargoFlywheelVelocityGoal,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.CargoIntakeExtend,
                DigitalOperation.CargoIntakeRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
            }),
        new MacroOperationDescription(
            MacroOperation.AutoShootDriveBack,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.Test1Debug,
            Shift.Test1Debug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                // vision centering
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(3000.0),
                    SequentialTask.Sequence(
                        new WaitTask(1.0), // how long it take to spin
                        new CargoShootTask()
                    )
                ),
                new FollowPathTask("goBack4ft")
            ),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                AnalogOperation.CargoFlywheelVelocityGoal,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.CargoIntakeExtend,
                DigitalOperation.CargoIntakeRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
            }),
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
