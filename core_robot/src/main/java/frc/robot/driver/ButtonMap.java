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
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
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
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY),
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
        //     AnalogOperation.DriveTrainSpinLeft,
        //     UserInputDevice.Driver,
        //     AnalogAxis.XBONE_LT,
        //     Shift.DriverDebug,
        //     Shift.DriverDebug,
        //     ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
        //     -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
        //     TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),
        // new AnalogOperationDescription(
        //     AnalogOperation.DriveTrainSpinRight,
        //     UserInputDevice.Driver,
        //     AnalogAxis.XBONE_RT,
        //     Shift.DriverDebug,
        //     Shift.DriverDebug,
        //     !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // make left positive, as counter-clockwise is positive
        //     -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
        //     TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),

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

        // // climber mechanism testing operations
        // new AnalogOperationDescription(
        //     AnalogOperation.ClimberWinchDesiredPosition,
        //     UserInputDevice.Test1,
        //     AnalogAxis.XBONE_RSX,
        //     Shift.Test1Debug,
        //     Shift.None,
        //     ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
        //     -1.0,
        //     0.1),
        // new AnalogOperationDescription(
        //     AnalogOperation.ClimberWinchMotorPower,
        //     UserInputDevice.Test1,
        //     AnalogAxis.XBONE_RSX,
        //     Shift.Test1Debug,
        //     Shift.Test1Debug,
        //     ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
        //     -0.1,
        //     0.1),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableMaintainDirectionMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),

        // cargo opertaions:
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeForceExtend,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoIntakeForceExtend,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Simple),
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

        // new DigitalOperationDescription(
        //     DigitalOperation.CargoEnableShootAnywayMode,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.CargoDisableShootAnywayMode,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Click),

        // shoot operations
        // new DigitalOperationDescription(
        //     DigitalOperation.CargoHoodPointBlank,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.CargoHoodShort,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.CargoHoodMedium,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.CargoHoodLong,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Click),

        // Climber operations
        new DigitalOperationDescription(
            DigitalOperation.SClimberUp,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.SClimberDown,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.CodriverDebug,
            Shift.CodriverDebug,
            ButtonType.Click),

        // climber operations
        // new DigitalOperationDescription(
        //     DigitalOperation.ClimberHookRelease,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.ClimberHookGrasp,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.ClimberArmUp,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.ClimberArmOut,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.ClimberResetWinchPosition,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Click),

        // testing operations
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

        new DigitalOperationDescription(
            DigitalOperation.CargoFeed,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            Shift.None,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoForceIntakeOnly,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CargoForceIntakeAndConveyorOnly,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Simple),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        new MacroOperationDescription(
            MacroOperation.PIDLightBrake,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple,
            () -> new PIDBrakeTask(false),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.PIDHeavyBrake,
            UserInputDevice.Driver,
            180, // DPAD-down
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple,
            () -> new PIDBrakeTask(true),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterHub,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new VisionCenteringTask(false, true),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterCargo,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new VisionCenteringTask(true, true),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionIntakeCargo,
            UserInputDevice.Driver,
            180, // DPAD-down
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                    new CargoIntakeTask(0.5, true),
                    ConcurrentTask.AnyTasks(
                        new CargoIntakeTask(5.0, true),
                        new VisionAdvanceAndCenterTask(false, true, true))),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionMoveToHub,
            UserInputDevice.Driver,
            270, // DPAD-left
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new VisionAdvanceAndCenterTask(false, false, true),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),

        // shooting macros
        new MacroOperationDescription(
            MacroOperation.ShootPointBlankHigh,
            UserInputDevice.Driver,
            //// UserInputDeviceButton.XBONE_B_BUTTON,
            AnalogAxis.XBONE_LT,
            0.5,
            1.0,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                    new RumbleTask(),
                    SequentialTask.Sequence(
                        new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
                        ConcurrentTask.AnyTasks(
                            new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                            new CargoShootTask()))),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.ShootPointBlankLow,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                    new RumbleTask(),
                    SequentialTask.Sequence(
                        new CargoHoodTask(DigitalOperation.CargoHoodLong),
                        ConcurrentTask.AnyTasks(
                            new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_LOW_SPINUP_SPEED),
                            new CargoShootTask()))),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.ShootTarmacHigh,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                    new RumbleTask(),
                    SequentialTask.Sequence(
                        new CargoHoodTask(DigitalOperation.CargoHoodLong),
                        ConcurrentTask.AnyTasks(
                            new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
                            new CargoShootTask()))),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.AutoPositionAndShoot,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            0.5,
            1.0,
            ////UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                    new RumbleTask(),
                    SequentialTask.Sequence(
                        new VisionCenteringTask(false),
                        new VisionShootPositionTask(),
                        ConcurrentTask.AnyTasks(
                            new VisionShootSpinTask(10.0, true),
                            new CargoShootTask()))),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.CargoFlywheelVelocityGoal,
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
            }),

        new MacroOperationDescription(
            MacroOperation.SpinUpFlywheel,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
            new IOperation[]
            {
                AnalogOperation.CargoFlywheelVelocityGoal,
            }),

        new MacroOperationDescription(
            MacroOperation.ChooseShoot,
            UserInputDevice.Driver,
            270, // DPAD-left
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                    new RumbleTask(),
                    SequentialTask.Sequence(
                        new CargoHoodChooseTask(),
                        ConcurrentTask.AnyTasks(
                            new CargoSpinupChooseTask(),
                            new CargoShootTask()))),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.ForceLightDriverRumble,
                AnalogOperation.CargoFlywheelVelocityGoal,
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
            }),
        /*new MacroOperationDescription(
            MacroOperation.AutoShootOnly,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            0.5,
            1.0,
            ////UserInputDeviceButton.XBONE_Y_BUTTON,
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                    new VisionShootSpinTask(10.0, true),
                    new CargoShootTask()
                ),
            new IOperation[]
            {
                DigitalOperation.CargoIntakeForceExtend,
                DigitalOperation.CargoIntakeForceRetract,
                DigitalOperation.CargoIntakeIn,
                DigitalOperation.CargoIntakeOut,
                DigitalOperation.CargoEject,
                DigitalOperation.CargoForceIntakeOnly,
                DigitalOperation.CargoForceIntakeAndConveyorOnly,
                DigitalOperation.CargoFeed,
                DigitalOperation.CargoHoodPointBlank,
                DigitalOperation.CargoHoodShort,
                DigitalOperation.CargoHoodMedium,
                DigitalOperation.CargoHoodLong,
                AnalogOperation.CargoFlywheelVelocityGoal,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),*/

        // // climber control macros for codriver
        // new MacroOperationDescription(
        //     MacroOperation.ClimberWinchForward,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Simple,
        //     () -> new ClimberWinchPowerTask(0.5),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberWinchMotorPower,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.ClimberWinchBackward,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Simple,
        //     () -> new ClimberWinchPowerTask(-0.5),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberWinchMotorPower
        //     }),

        // // Climbing macros:
        // new MacroOperationDescription(
        //     MacroOperation.ClimbSetUpWall,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new FollowPathTask("lineUpUnder1stClimberBarWall"),
        //         new ClimberArmUnlockTask(true),
        //         ConcurrentTask.AllTasks(
        //             new ClimberArmTask(true),
        //             new ClimberWeightedTask(false)
        //         ),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH),
        //         new FollowPathTask("goForward5in")
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainTurnSpeed,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         AnalogOperation.DriveTrainTurnAngleReference,
        //         AnalogOperation.ClimberWinchDesiredPosition,
        //         AnalogOperation.ClimberWinchMotorPower,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.ClimberHookGrasp,
        //         DigitalOperation.ClimberHookRelease,
        //         DigitalOperation.ClimberArmOut,
        //         DigitalOperation.ClimberArmUp,
        //         DigitalOperation.ClimberEnableWeightedMode,
        //         DigitalOperation.ClimberEnableUnweightedMode,
        //         DigitalOperation.ClimberWinchLock,
        //         DigitalOperation.ClimberWinchUnlock
        //     }
        // ),

        // new MacroOperationDescription(
        //     MacroOperation.ClimbSetUpNotWall,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
        //     Shift.CodriverDebug,
        //     Shift.CodriverDebug,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new FollowPathTask("lineUpUnder1stClimberBarNotWall"),
        //         new ClimberArmUnlockTask(true),
        //         ConcurrentTask.AllTasks(
        //             new ClimberArmTask(true),
        //             new ClimberWeightedTask(false)
        //         ),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH),
        //         new FollowPathTask("goForward5in")
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainTurnSpeed,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         AnalogOperation.DriveTrainTurnAngleReference,
        //         AnalogOperation.ClimberWinchDesiredPosition,
        //         AnalogOperation.ClimberWinchMotorPower,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.ClimberHookGrasp,
        //         DigitalOperation.ClimberHookRelease,
        //         DigitalOperation.ClimberArmOut,
        //         DigitalOperation.ClimberArmUp,
        //         DigitalOperation.ClimberEnableWeightedMode,
        //         DigitalOperation.ClimberEnableUnweightedMode,
        //         DigitalOperation.ClimberWinchLock,
        //         DigitalOperation.ClimberWinchUnlock
        //     }
        // ),

        // // extends arm and stuff, driver has to activate this then drive towards the bar and hook it on
        // new MacroOperationDescription(
        //     MacroOperation.ClimbSetUpManual,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new ClimberArmUnlockTask(true),
        //         ConcurrentTask.AllTasks(
        //             new ClimberArmTask(true),
        //             new ClimberWeightedTask(false)
        //         ),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH)
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberWinchDesiredPosition,
        //         AnalogOperation.ClimberWinchMotorPower,
        //         DigitalOperation.ClimberHookGrasp,
        //         DigitalOperation.ClimberHookRelease,
        //         DigitalOperation.ClimberArmOut,
        //         DigitalOperation.ClimberArmUp,
        //         DigitalOperation.ClimberEnableWeightedMode,
        //         DigitalOperation.ClimberEnableUnweightedMode,
        //         DigitalOperation.ClimberWinchLock,
        //         DigitalOperation.ClimberWinchUnlock
        //     }
        // ),

        // new MacroOperationDescription(
        //     MacroOperation.ClimberRiseToMidRung,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         ConcurrentTask.AllTasks(
        //             new ClimberHookTask(false),
        //             new ClimberWeightedTask(true)
        //         ),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_RETRACT_LENGTH),
        //         new ClimberHookTask(true)
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberWinchDesiredPosition,
        //         AnalogOperation.ClimberWinchMotorPower,
        //         DigitalOperation.ClimberHookGrasp,
        //         DigitalOperation.ClimberHookRelease,
        //         DigitalOperation.ClimberArmOut,
        //         DigitalOperation.ClimberArmUp,
        //         DigitalOperation.ClimberEnableWeightedMode,
        //         DigitalOperation.ClimberEnableUnweightedMode,
        //         DigitalOperation.ClimberWinchLock,
        //         DigitalOperation.ClimberWinchUnlock
        //     }
        // ),

        // new MacroOperationDescription(
        //     MacroOperation.ClimberExtendToNextRung,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new ClimberWeightedTask(false),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_SHORT_EXTEND_LENGTH),
        //         new ClimberArmTask(false),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_EXTEND_LENGTH),
        //         new ClimberArmTask(true),
        //         new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_MOSTLY_EXTEND_LENGTH)
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberWinchDesiredPosition,
        //         AnalogOperation.ClimberWinchMotorPower,
        //         DigitalOperation.ClimberHookGrasp,
        //         DigitalOperation.ClimberHookRelease,
        //         DigitalOperation.ClimberArmOut,
        //         DigitalOperation.ClimberArmUp,
        //         DigitalOperation.ClimberEnableWeightedMode,
        //         DigitalOperation.ClimberEnableUnweightedMode,
        //         DigitalOperation.ClimberWinchLock,
        //         DigitalOperation.ClimberWinchUnlock
        //     }
        // ),
        

        // new MacroOperationDescription(
        //     MacroOperation.ClimberSwitchToNextRung,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
        //     Shift.CodriverDebug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new ClimberWeightedTask(true),
        //         ConcurrentTask.AllTasks(
        //             new ClimberHookTask(false),
        //             new ClimberWinchPositionExtensionTask(TuningConstants.CLIMBER_FULL_RETRACT_LENGTH)
        //         ),
        //         new ClimberHookTask(true)
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberWinchDesiredPosition,
        //         AnalogOperation.ClimberWinchMotorPower,
        //         DigitalOperation.ClimberHookGrasp,
        //         DigitalOperation.ClimberHookRelease,
        //         DigitalOperation.ClimberArmOut,
        //         DigitalOperation.ClimberArmUp,
        //         DigitalOperation.ClimberEnableWeightedMode,
        //         DigitalOperation.ClimberEnableUnweightedMode,
        //         DigitalOperation.ClimberWinchLock,
        //         DigitalOperation.ClimberWinchUnlock
        //     }
        // ),

        // autonomous testing operations
        // new MacroOperationDescription(
        //     MacroOperation.AutoDriveBackIntakeShoot,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_A_BUTTON,
        //     Shift.Test1Debug,
        //     Shift.None,
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         ConcurrentTask.AllTasks(
        //             new FollowPathTask("goForward4ft"),
        //             new CargoExtendIntakeTask(true),
        //             new CargoIntakeTask(1.0, true)
        //         ),
        //         new FollowPathTask("goBack7ftRotate"),
        //         new VisionCenteringTask(false),
        //         ConcurrentTask.AnyTasks(
        //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
        //             new CargoShootTask()
        //         )
        //     ),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainTurnSpeed,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         AnalogOperation.DriveTrainTurnAngleReference,
        //         AnalogOperation.CargoFlywheelVelocityGoal,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.VisionDisableStream,
        //         DigitalOperation.VisionEnableGamePieceProcessing,
        //         DigitalOperation.VisionEnableRetroreflectiveProcessing,
        //         DigitalOperation.VisionForceDisable,
        //         DigitalOperation.CargoIntakeForceExtend,
        //         DigitalOperation.CargoIntakeForceRetract,
        //         DigitalOperation.CargoIntakeIn,
        //         DigitalOperation.CargoIntakeOut,
        //         DigitalOperation.CargoEject,
        //         DigitalOperation.CargoForceIntakeOnly,
        //         DigitalOperation.CargoForceIntakeAndConveyorOnly,
        //         DigitalOperation.CargoFeed,
        //         DigitalOperation.CargoHoodPointBlank,
        //         DigitalOperation.CargoHoodShort,
        //         DigitalOperation.CargoHoodMedium,
        //         DigitalOperation.CargoHoodLong,
        //     }),

        // ----------------------- SAMMAMISH AUTO -------------------------
        new MacroOperationDescription(
            MacroOperation.FollowPathTest1,
            UserInputDevice.Test1,
            0,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> new FollowPathTask("goForward4ft"),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.FollowPathTest2,
            UserInputDevice.Test1,
            180,
            Shift.None,
            Shift.None,
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft4ft"),
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
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
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
