package frc.robot.driver;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
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
        Trajectory turnArcLeft = startTrajectory()
            .splineToSplineHeading(new Pose2d(72.0, 40.0, 90.0 * Helpers.DEGREES_TO_RADIANS), 90.0 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(100.0, 90.0, 0.0 * Helpers.DEGREES_TO_RADIANS), 0.0 * Helpers.DEGREES_TO_RADIANS)
            .build();

        ITrajectory trajectory = new TrajectoryWrapper(turnArcLeft);

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
        // ----------------------------------------- 2022 paths ----------------------------------------- //
        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(48, 0), 0),
            "goForward4ft");

        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(84, 0), 0),
            "goForward7ft");

        addPath(
            pathManager,
            startTrajectory()
                .splineToSplineHeading(new Pose2d(-1, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS),
            "turn180Path");

        addPath(
            pathManager,
            startTrajectory()
                .splineToSplineHeading(new Pose2d(-84, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS),
            "goBack7ftRotate");

        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(-72, 0), 0),
            "goBack6ft");

        addPath(
            pathManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-10,15), 0),
            "lineUpUnder1stClimberBarWall");

        addPath(
            pathManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-10,-15), 0),
            "lineUpUnder1stClimberBarNotWall");

        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(5, 0), 0),
            "goForward5in");

        addPath(
            pathManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-60, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-84, 0), 180 * Helpers.DEGREES_TO_RADIANS),
            "goBack5ftTurn180GoBack2ft");

        // 5-BALL AUTO PATHS

        addPath(
            pathManager,
            startTrajectory()
                .splineTo(new Vector2d(60, 0), 0),
            "goForward5ft");

        addPath(
            pathManager,
            startTrajectory(135.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-60, 54.82, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-100.4, 54.82), 180 * Helpers.DEGREES_TO_RADIANS),
            "goBack5ftLeft3ftTurn180GoBack3ft");

        addPath(
            pathManager,
            startTrajectory(-90 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-40, -62, -122.25 * Helpers.DEGREES_TO_RADIANS), -122.25 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-60, -96), -122.25 * Helpers.DEGREES_TO_RADIANS),
            "goBack3ftRight5ftTurn122GoBack2ftRight3ft");
        
        addPath(
            pathManager,
            startTrajectory(-60 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-74,-63, -79.2 * Helpers.DEGREES_TO_RADIANS), -139.3 * Helpers.DEGREES_TO_RADIANS),
            "goBack6ftRight5ftTurn122");

        addPath(
            pathManager,
            startTrajectory(110 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-76,193, 154.7 * Helpers.DEGREES_TO_RADIANS), 154.7 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-110, 209), 154.7 * Helpers.DEGREES_TO_RADIANS),
            "goBack6ftLeft16ftTurn154GoBack3ftLeft1ft");

        addPath(
            pathManager,
            startTrajectory(90 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-215,142, -154.7 * Helpers.DEGREES_TO_RADIANS), -154.7 * Helpers.DEGREES_TO_RADIANS),
            "goBack18ftLeft12ftTurn154");

        // 2 BALL AUTO SAMMAMISH PATHS

        addPath(
            pathManager, 
            startTrajectory(0)
                .splineToSplineHeading(new Pose2d(41.87, -2.97, 4.1 * Helpers.DEGREES_TO_RADIANS), 4.1 * Helpers.DEGREES_TO_RADIANS),
            "goBack3ftRight1Turn4");

        addPath(
            pathManager,
            startTrajectory(-90 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-99.47, 15.56, -171 * Helpers.DEGREES_TO_RADIANS), -171 * Helpers.DEGREES_TO_RADIANS),
            "goLeft1ftBack8ftTurn171");//-171
    }

    private static TrajectoryBuilder startTrajectory()
    {
        return startTrajectory(0.0);
    }

    private static TrajectoryBuilder startTrajectory(double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(0, 0, 0), RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
    }

    private static void addPath(PathManager pathManager, TrajectoryBuilder trajectoryBuilder, String name)
    {
        try
        {
            pathManager.addPath(name, new TrajectoryWrapper(trajectoryBuilder.build()));
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}