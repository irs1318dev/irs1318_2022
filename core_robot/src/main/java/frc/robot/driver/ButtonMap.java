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
        // new ShiftDescription(
        //     Shift.Test2Debug,
        //     UserInputDevice.Test2,
        //     UserInputDeviceButton.XBONE_LEFT_BUTTON),
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
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
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
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableGamePieceProcessing,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),

        // cargo opertaions:
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeForceExtend,
            UserInputDevice.Driver,
            0,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeForceRetract,
            UserInputDevice.Driver,
            180,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeIn,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoEject,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            Shift.None,
            Shift.None,
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
        new DigitalOperationDescription(
            DigitalOperation.ClimberResetWinchPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),

        // cargo testing operations
        new DigitalOperationDescription(
            DigitalOperation.CargoFeed,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.CargoHoodPointBlank,
            UserInputDevice.Test1,
            0,
            Shift.None,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CargoHoodLong,
            UserInputDevice.Test1,
            180,
            Shift.None,
            Shift.None,
            ButtonType.Click),
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
            MacroOperation.ClimbSetUpWall,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("lineUpUnder1stClimberBarWall"),
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
                AnalogOperation.DriveTrainTurnAngleReference,
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
            MacroOperation.ClimbSetUpNotWall,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("lineUpUnder1stClimberBarNotWall"),
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
                AnalogOperation.DriveTrainTurnAngleReference,
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

        // extends arm and stuff, driver has to activate this then drive towards the bar and hook it on
        new MacroOperationDescription(
            MacroOperation.ClimbSetUpManual,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ClimberArmUnlockTask(true),
                ConcurrentTask.AllTasks(
                    new ClimberArmTask(true),
                    new ClimberWeightedTask(false)
                ),
                new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH)
            ),
            new IOperation[]
            {
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
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
            MacroOperation.CargoIntakeExtendMacro,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new CargoExtendIntakeTask(true)
            ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract
            }
        ),
        
        new MacroOperationDescription(
            MacroOperation.CargoIntakeRetractMacro,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new CargoExtendIntakeTask(false)
            ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract
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
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
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
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
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
                AnalogOperation.ClimberWinchDesiredPosition,
                AnalogOperation.ClimberWinchMotorPower,
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

        // shooting macros
        new MacroOperationDescription(
            MacroOperation.ShootPointBlankHigh,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                    new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(0.25),
                        new CargoShootTask()
                    )
                ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.ShootPointBlankLow,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                    new CargoHoodTask(DigitalOperation.CargoHoodLong),
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_LOW_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(0.25),
                        new CargoShootTask()
                    )
                ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.ShootTarmacHigh,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                    new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(0.25),
                        new CargoShootTask()
                    )
                ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.AutoShootHigh,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                    new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
                    new VisionSpinUpTask(true),
                    SequentialTask.Sequence(
                        new WaitTask(0.25),
                        new CargoShootTask()
                    )
                ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),


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
                    new CargoIntakeTask(1, true)
                ),
                new FollowPathTask("goBack7ftRotate"),
                new VisionCenteringTask(false),
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME), // how long it take to spin
                        new CargoShootTask(),
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME), // TODO maybe change
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
                AnalogOperation.DriveTrainTurnAngleReference,
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
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
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
                new VisionCenteringTask(false),
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME), // how long it take to spin
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
                AnalogOperation.DriveTrainTurnAngleReference,
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
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
            }),

        // 5-ball auto
        new MacroOperationDescription(
            MacroOperation.FiveBallAuto,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.Test1Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                //1 move to shooting position
                ConcurrentTask.AllTasks(
                    new FollowPathTask("goForward5Feet"),
                    new CargoExtendIntakeTask(true)
                ),
                //2 center with goal
                new VisionCenteringTask(false),
                //3 shoot pre-loaded ball
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME), // how long it take to spin
                        new CargoShootTask(false)
                    )
                ),
                //4 get first ball
                ConcurrentTask.AllTasks(
                    new CargoIntakeTask(10.0, true),
                    new FollowPathTask("goBack5ftLeft3ftTurn180GoBack3ft")
                ),
                //5 get second ball
                ConcurrentTask.AllTasks(
                    new CargoIntakeTask(10.0, true),
                    new FollowPathTask("goBack3ftRight5ftTurn122GoBack2ftRight3ft")
                ),
                //6 shoot the 2 balls
                new FollowPathTask("goBack6ftRight5ftTurn122"),
                ConcurrentTask.AllTasks(
                    new VisionCenteringTask(false),
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME),
                        new CargoShootTask()
                    )
                ),
                //7 move to terminal and start intake
                ConcurrentTask.AllTasks(
                    new FollowPathTask("goBack6ftLeft16ftTurn154GoBack3ftLeft1ft"), //split into 2 tasks
                    new CargoIntakeTask(10.0, true),
                    SequentialTask.Sequence(
                        new WaitTask(2.0),
                        new VisionCenteringTask(true),
                        new WaitTask(1.0),
                        new VisionCenteringTask(true)
                    )   
                ),
                //8 move to shoot those balls but not during the month of november
                new FollowPathTask("goBack18ftLeft12ftTurn154"),
                new VisionCenteringTask(false),
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME),
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
                AnalogOperation.DriveTrainTurnAngleReference,
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
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
            }
        ),

        new MacroOperationDescription(
            MacroOperation.ThreeBallAuto,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.Test1Debug,
            Shift.Test1Debug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                //1 move to shooting position
                ConcurrentTask.AllTasks(
                    new FollowPathTask("goForward5Feet"),
                    new CargoExtendIntakeTask(true)
                ),
                //2 center with goal
                new VisionCenteringTask(false),
                //3 shoot pre-loaded ball
                ConcurrentTask.AllTasks(
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME), // how long it take to spin
                        new CargoShootTask(false)
                    )
                ),
                //4 get first ball
                ConcurrentTask.AllTasks(
                    new CargoIntakeTask(10.0, true),
                    new FollowPathTask("goBack5ftLeft3ftTurn180GoBack3ft")
                ),
                //5 get second ball
                ConcurrentTask.AllTasks(
                    new CargoIntakeTask(10.0, true),
                    new FollowPathTask("goBack3ftRight5ftTurn122GoBack2ftRight3ft")
                ),
                //6 shoot the 2 balls
                new FollowPathTask("goBack6ftRight5ftTurn122"),
                ConcurrentTask.AllTasks(
                    new VisionCenteringTask(false),
                    new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                    SequentialTask.Sequence(
                        new WaitTask(TuningConstants.CARGO_SHOOT_SPINUP_WAIT_TIME),
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
                AnalogOperation.DriveTrainTurnAngleReference,
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
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodLong,
            }
        ),
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
