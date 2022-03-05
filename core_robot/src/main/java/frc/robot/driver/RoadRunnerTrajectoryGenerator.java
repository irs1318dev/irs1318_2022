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
        Trajectory turnArcLeft = buildTrajectory()
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
        // ----------------------------------------- 2022 auto paths ----------------------------------------- //
        Trajectory goForward4ft = buildTrajectory()
            .splineTo(new Vector2d(48, 0), 0)
            .build();
        pathManager.addPath(
            "goForward4ft",
            new TrajectoryWrapper(goForward4ft));

        Trajectory goForward7ft = buildTrajectory()
            .splineTo(new Vector2d(84, 0), 0)
            .build();
        pathManager.addPath(
            "goForward7ft",
            new TrajectoryWrapper(goForward7ft));

        Trajectory turn180Trajectory = buildTrajectory()
            .splineToSplineHeading(new Pose2d(-1, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "turn180Path",
            new TrajectoryWrapper(turn180Trajectory));
        
        Trajectory goBack7ftRotate = buildTrajectory()   
            .splineToSplineHeading(new Pose2d(-84, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack7ftRotate",
            new TrajectoryWrapper(goBack7ftRotate));

        Trajectory goBack4ft = buildTrajectory()
            .splineTo(new Vector2d(-48, 0), 0)
            .build();
        pathManager.addPath(
            "goBack4ft", 
            new TrajectoryWrapper(goBack4ft));

        Trajectory lineUpUnder1stClimberBarWall = buildTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
            .splineTo(new Vector2d(-10,15), 0)
            .build();
        pathManager.addPath(
            "lineUpUnder1stClimberBarWall", 
            new TrajectoryWrapper(lineUpUnder1stClimberBarWall));

        Trajectory lineUpUnder1stClimberBarNotWall = buildTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
            .splineTo(new Vector2d(-10,-15), 0)
            .build();
        pathManager.addPath(
            "lineUpUnder1stClimberBarNotWall", 
            new TrajectoryWrapper(lineUpUnder1stClimberBarNotWall));

        Trajectory goForward5in = buildTrajectory()
            .splineTo(new Vector2d(5, 0), 0)
            .build();
        pathManager.addPath(
            "goForward5in",
            new TrajectoryWrapper(goForward5in));

        Trajectory goBack5ftTurn180GoBack2ft = buildTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(-60, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
            .splineToConstantHeading(new Vector2d(-84, 0), 180 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack5ftTurn180GoBack2ft",
            new TrajectoryWrapper(goBack5ftTurn180GoBack2ft));
        

        // 5 BALL AUTON PATHS

        Trajectory goForward5ft = buildTrajectory()
            .splineTo(new Vector2d(60, 0), 0)
            .build();
        pathManager.addPath(
            "goForward5ft", 
            new TrajectoryWrapper(goForward5ft));
        
        
        Trajectory goBack5ftLeft3ftTurn180GoBack3ft = buildTrajectory(135.0 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(-60, 54.82, 180.0 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
            .splineToConstantHeading(new Vector2d(-100.4, 54.82), 180 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack5ftLeft3ftTurn180GoBack3ft",
            new TrajectoryWrapper(goBack5ftLeft3ftTurn180GoBack3ft));
        
        
        Trajectory goBack3ftRight5ftTurn122GoBack2ftRight3ft = buildTrajectory(-90 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(-40, -62, -122.25 * Helpers.DEGREES_TO_RADIANS), -122.25 * Helpers.DEGREES_TO_RADIANS)
            .splineToConstantHeading(new Vector2d(-60, -96), -122.25 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack3ftRight5ftTurn122GoBack2ftRight3ft",
            new TrajectoryWrapper(goBack3ftRight5ftTurn122GoBack2ftRight3ft));
        
        Trajectory goBack6ftRight5ftTurn80 = buildTrajectory(-60 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(-74,-63, -79.2 * Helpers.DEGREES_TO_RADIANS), -139.3 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack6ftRight5ftTurn122",
            new TrajectoryWrapper(goBack6ftRight5ftTurn80));
        
        Trajectory goBack6ftLeft16ftTurn154GoBack3ftLeft1ft = buildTrajectory(110 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(-76,193, 154.7 * Helpers.DEGREES_TO_RADIANS), 154.7 * Helpers.DEGREES_TO_RADIANS)
            .splineToConstantHeading(new Vector2d(-110, 209), 154.7 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack6ftLeft16ftTurn154GoBack3ftLeft1ft",
            new TrajectoryWrapper(goBack6ftLeft16ftTurn154GoBack3ftLeft1ft));
        
        Trajectory goBack18ftLeft12ftTurn154 = buildTrajectory(90 * Helpers.DEGREES_TO_RADIANS)
            .splineToSplineHeading(new Pose2d(-215,142, -154.7 * Helpers.DEGREES_TO_RADIANS), -154.7 * Helpers.DEGREES_TO_RADIANS)
            .build();
        pathManager.addPath(
            "goBack18ftLeft12ftTurn154", 
            new TrajectoryWrapper(goBack18ftLeft12ftTurn154));       
            
    }

    private static TrajectoryBuilder buildTrajectory()
    {
        return buildTrajectory(0.0);
    }

    private static TrajectoryBuilder buildTrajectory(double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(0, 0, 0), RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
    }
}