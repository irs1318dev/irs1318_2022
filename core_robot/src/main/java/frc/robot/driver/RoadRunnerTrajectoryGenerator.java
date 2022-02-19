package frc.robot.driver;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.path.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import de.siegmar.fastcsv.writer.CsvWriter;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.common.robotprovider.ITrajectory;
import frc.robot.common.robotprovider.TrajectoryState;
import frc.robot.driver.common.PathManager;
import frc.robot.driver.common.TrajectoryWrapper;

public class RoadRunnerTrajectoryGenerator
{
    private static final TrajectoryVelocityConstraint velocityConstraint =
        new MinVelocityConstraint(
            Arrays.asList(
                new SwerveVelocityConstraint(
                    TuningConstants.DRIVETRAIN_MAX_MODULE_PATH_VELOCITY,
                    HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE,
                    HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE),
                new AngularVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TURN_VELOCITY * Helpers.DEGREES_TO_RADIANS),
                new TranslationalVelocityConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY)));

    private static final TrajectoryAccelerationConstraint accelerationConstraint =
            new ProfileAccelerationConstraint(TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION);

    public static void main(String[] args)
    {
        Path turnArcLeft = new PathBuilder(new Pose2d(0.0, 0.0, 0.0))
            .splineToSplineHeading(new Pose2d(72.0, 40.0, 90.0 * Helpers.DEGREES_TO_RADIANS), 90.0 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(100.0, 90.0, 0.0 * Helpers.DEGREES_TO_RADIANS), 0.0 * Helpers.DEGREES_TO_RADIANS)
            .build();

        ITrajectory trajectory = new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(turnArcLeft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint));

        try (CsvWriter csvWriter = CsvWriter.builder().build(java.nio.file.Path.of("test.csv"), StandardCharsets.UTF_8))
        {
            csvWriter.writeRow("x", "y", "theta", "vx", "vy", "omega");

            for (double t = 0.0; t < trajectory.getDuration() + 0.01; t += 0.02)
            {
                TrajectoryState state = trajectory.get(t);
                csvWriter.writeRow(
                    Double.toString(state.xPosition),
                    Double.toString(state.yPosition),
                    Double.toString(state.angle),
                    Double.toString(state.xVelocity),
                    Double.toString(state.yVelocity),
                    Double.toString(state.angleVelocity));
            }

            csvWriter.close();
        }
        catch (IOException e)
        {
        }
    }

    public static void generateTrajectories(PathManager pathManager)
    {
// ----------------------------------------- 2022 auto paths ----------------------------------------- //

        Path goForward4ft = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(48, 0), 0)
            .build();
        pathManager.addPath(
            "goForward4ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(goForward4ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        
        Path goForward7ft = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(84, 0), 0)
            .build();
        pathManager.addPath(
            "goForward7ft",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(goForward7ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        
        
        Path turn180Path = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(-1, 0), 180 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "turn180Path",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(turn180Path, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));

        Path goBack7ftRotate = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(-84, 0), 180 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack7ftRotate", 
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(goBack7ftRotate, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        
        Path goBack4ft = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(-48, 0), 0)
            .build();
        pathManager.addPath(
            "goBack4ft", 
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(goBack4ft, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        
        Path lineUpUnder1stClimberBar = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(-10,15,0))
            .build();
        pathManager.addPath(
            "linUpUnder1stClimberBar",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANTCE.generateTrajectory(lineUpUnder1stClimberBar, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
        
        Path goForward5in = new PathBuilder(new Pose2d(0, 0, 0))
            .splineTo(new Vector2d(5, 0), 0)
            .build();
        pathManager.addPath(
            "goForward5in",
            new TrajectoryWrapper(TrajectoryGenerator.INSTANCE.generateTrajectory(goForward5in, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint)));
    }
}