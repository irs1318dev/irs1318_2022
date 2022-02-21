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
            UserInputDeviceButton.BUTTON_PAD_BUTTON_1),
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
            Shift.DriverDebug,
            Shift.None,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            1.0,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnSpeed,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            Shift.DriverDebug,
            Shift.DriverDebug,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive, as counter-clockwise is positive
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),

        /*
         * new AnalogOperationDescription(
         * AnalogOperation.DriveTrainRotationA,
         * UserInputDevice.Driver,
         * AnalogAxis.XBONE_LT,
         * ElectronicsConstants.INVERT_TRIGGER_AXIS,
         * -TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
         * TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
         * TuningConstants.DRIVETRAIN_ROTATION_A_MULTIPLIER),
         * new AnalogOperationDescription(
         * AnalogOperation.DriveTrainRotationB,
         * UserInputDevice.Driver,
         * AnalogAxis.XBONE_RT,
         * ElectronicsConstants.INVERT_TRIGGER_AXIS,
         * -TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
         * TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
         * TuningConstants.DRIVETRAIN_ROTATION_B_MULTIPLIER),
         */
        
        // cargo mechanism
        
        new AnalogOperationDescription(
            AnalogOperation.CargoFlywheelVelocityGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            Shift.DriverDebug,
            Shift.None,
            ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER,
            TuningConstants.FLYWHEEL_DEAD_ZONE_MIN,
            TuningConstants.FLYWHEEL_DEAD_ZONE_MAX),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
            Shift.None,
            Shift.None,
            ButtonType.Simple),

        // intake extend and retract
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

        // activate rollers
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

        // shoot cargo
        new DigitalOperationDescription(
            DigitalOperation.CargoFeed,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),

        // hood positions

        // Climber Hook Positions
        new DigitalOperationDescription(
            DigitalOperation.ClimberHookRelease,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_9, // DPAD-up
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ClimberHookGrasp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_10, // DPAD-down
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),

        // // Climber Arm Positions
        new DigitalOperationDescription(
            DigitalOperation.ClimberArmUp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_11, // DPAD-right
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ClimberArmOut,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_12, // DPAD-left
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
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
                    new VisionCenteringTask(),
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
            MacroOperation.WinchForward,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
            Shift.None,
            Shift.None,
            ButtonType.Simple,
            () -> new ClimberWinchPowerTask(0.5),
            new IOperation[]
            {
                AnalogOperation.ClimberWinchMotorPower,
            }),
        new MacroOperationDescription(
            MacroOperation.WinchBackward,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
            Shift.None,
            Shift.None,
            ButtonType.Simple,
            () -> new ClimberWinchPowerTask(-0.5),
            new IOperation[]
            {
                AnalogOperation.ClimberWinchMotorPower
            }),
        new MacroOperationDescription(
            MacroOperation.AutoDriveBackIntakeShoot,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
            Shift.None,
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
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_16,
            Shift.None,
            Shift.None,
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
        new MacroOperationDescription(
            MacroOperation.SetUpClimb,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_18,
            Shift.None,
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
            MacroOperation.ClimbToMidRung,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_19,
            Shift.None,
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
            MacroOperation.ExtendToNextRung,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_19,
            Shift.None,
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
            MacroOperation.SwitchToNextRung,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_20,
            Shift.None,
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
