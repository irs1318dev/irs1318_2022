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
import frc.robot.driver.common.TrajectoryManager;
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
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(trajectoryManager);
        ITrajectory trajectory = trajectoryManager.getTrajectory("w2ba-goToPickUpBall2");

        try (CsvWriter csvWriter = CsvWriter.builder().build(java.nio.file.Path.of("test.csv"), StandardCharsets.UTF_8))
        {
            csvWriter.writeRow("t", "x", "y", "theta", "vx", "vy", "omega");

            for (double t = 0.0; t < trajectory.getDuration() + 0.01; t += 0.02)
            {
                TrajectoryState state = trajectory.get(t);
                csvWriter.writeRow(
                    Double.toString(t),
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

    public static void generateTrajectories(TrajectoryManager trajectoryManager)
    {
        // ----------------------------------------- 2022 paths ----------------------------------------- //
        addPath(
            trajectoryManager,
            startTrajectory()
                .splineTo(new Vector2d(48, 0), 0),
            "goForward4ft");

        addPath(
            trajectoryManager,
            startTrajectory(90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(0, 48), 90.0 * Helpers.DEGREES_TO_RADIANS),
            "goLeft4ft");

        addPath(
            trajectoryManager,
            startTrajectory()
                .splineTo(new Vector2d(84, 0), 0),
            "goForward7ft");
        
        addPath(
            trajectoryManager,
            startTrajectory(-90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(0, -62.0), -90.0 * Helpers.DEGREES_TO_RADIANS),
             "goForwardBrodieMiddle");
        
        addPath(
            trajectoryManager,
            startTrajectory(-90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(0, -42.0), -90.0 * Helpers.DEGREES_TO_RADIANS),
             "goForwardBrodieAll");
        
        addPath(
            trajectoryManager,
            startTrajectory(-90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(0, 17, 90.0 * Helpers.DEGREES_TO_RADIANS), 90.0 * Helpers.DEGREES_TO_RADIANS),
             "goBackBrodie");

        addPath(
            trajectoryManager,
            startTrajectory(0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d( -46, -6, 157 * Helpers.DEGREES_TO_RADIANS), 157 * Helpers.DEGREES_TO_RADIANS),
                "ThreeBallStep1");
        
        addPath(
            trajectoryManager,
            startTrajectory(0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(42, 0), 0 * Helpers.DEGREES_TO_RADIANS),
                "ThreeBallStep2");
        
        addPath(
            trajectoryManager,
            startTrajectory(0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-42, 0), 0 * Helpers.DEGREES_TO_RADIANS),
                "ThreeBallStep3");
        
        addPath(
            trajectoryManager,
            startTrajectory(0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-58, -58, 45 * Helpers.DEGREES_TO_RADIANS), 45 * Helpers.DEGREES_TO_RADIANS),
                "ThreeBallStep4");
        addPath(
            trajectoryManager, 
            startTrajectory(0 * Helpers.DEGREES_TO_RADIANS) 
                .splineTo(new Vector2d(42, 0), 0 * Helpers.DEGREES_TO_RADIANS),
                "ThreeBallStep5");
        
        addPath(
            trajectoryManager, 
            startTrajectory(0 * Helpers.DEGREES_TO_RADIANS) 
                .splineToSplineHeading(new Pose2d(-42, 50, 150 * Helpers.DEGREES_TO_RADIANS), 150 * Helpers.DEGREES_TO_RADIANS),
                "ThreeBallStep6");

        addPath(
            trajectoryManager,
            startTrajectory()
                .splineToSplineHeading(new Pose2d(-1, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "turn180Path");

        addPath(
            trajectoryManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-84, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "goBack7ftRotate");

        addPath(
            trajectoryManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-72, 0), 180.0 * Helpers.DEGREES_TO_RADIANS),
            "goBack6ft");

        addPath(
            trajectoryManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-10,15), 0),
            "lineUpUnder1stClimberBarWall");

        addPath(
            trajectoryManager,
            startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineTo(new Vector2d(-10,-15), 0),
            "lineUpUnder1stClimberBarNotWall");

        // WILL's 3-ball auto:
        addPath(
            trajectoryManager,
            startTrajectory(0.0, 0.0, 69.0 * Helpers.DEGREES_TO_RADIANS, -69.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-6.8, -17.7), -90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-7.8, -70.3, -90.0 * Helpers.DEGREES_TO_RADIANS), -90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-7.8, -82.1, -90.0 * Helpers.DEGREES_TO_RADIANS), -90.0 * Helpers.DEGREES_TO_RADIANS),
            "w3ba-goToPickUpBall2");

        addPath(
            trajectoryManager,
            startTrajectory(-7.8, -82.1, -90.0 * Helpers.DEGREES_TO_RADIANS, -220.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-76.1, -48.6, -180.0 * Helpers.DEGREES_TO_RADIANS), -180.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-98.1, -39.6, -180.0 * Helpers.DEGREES_TO_RADIANS), -180.0 * Helpers.DEGREES_TO_RADIANS),
            "w3ba-goToPickUpBall3");

        addPath(
            trajectoryManager,
            startTrajectory(-98.1, -39.6, -180.0 * Helpers.DEGREES_TO_RADIANS, 45.9 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-60.5, -54.9, 47.9 * Helpers.DEGREES_TO_RADIANS), 45.9 * Helpers.DEGREES_TO_RADIANS),
            "w3ba-turnToShoot");// -50.5, -44.9, 50.9
        
        // WINAV's 2-ball auto:

        addPath(
            trajectoryManager,
            startTrajectory(0.0, 0.0, 69.0 * Helpers.DEGREES_TO_RADIANS, -69.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-6.8, -17.7), -90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-7.8, -82.3, -90.0 * Helpers.DEGREES_TO_RADIANS), -90.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-7.8, -88.1, -90.0 * Helpers.DEGREES_TO_RADIANS), -90.0 * Helpers.DEGREES_TO_RADIANS),
            "winavGoToBall2Ball");
        
        addPath(
            trajectoryManager,
            startTrajectory(-7.8, -82.1, -90.0 * Helpers.DEGREES_TO_RADIANS, 90 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-6.8, -17.7, 69.0 * Helpers.DEGREES_TO_RADIANS), 69.0 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(0.0, 0.0, 69.0 * Helpers.DEGREES_TO_RADIANS), 69.0 * Helpers.DEGREES_TO_RADIANS),
            "winavGoToShoot2Ball");
        
        
            
        // WILL's 2-ball auto:
        addPath(
            trajectoryManager,
            startTrajectory(0.0, 0.0, -21.0 * Helpers.DEGREES_TO_RADIANS, 142.1 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(9.5, 7.4), 142.1 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-55.6, 43.4, 142.1 * Helpers.DEGREES_TO_RADIANS), 142.1 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-64.9, 51.4), 142.1 * Helpers.DEGREES_TO_RADIANS),
            "w2ba-goToPickUpBall2");

        addPath(
            trajectoryManager,
            startTrajectory(-64.9, 51.4, 142.1 * Helpers.DEGREES_TO_RADIANS, -37.9 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-55.6, 43.4, -37.9 * Helpers.DEGREES_TO_RADIANS), -37.9 * Helpers.DEGREES_TO_RADIANS),
            "w2ba-turnToShoot");

        // PRAVIN's 3-ball
        addPath(
            trajectoryManager,
            startTrajectory(0.0, 0.0, -20.90 * Helpers.DEGREES_TO_RADIANS, 159.1 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-25, 6.2), 180 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-65, 6.2, 180 * Helpers.DEGREES_TO_RADIANS), 180 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-77.25, 6.2), 180 * Helpers.DEGREES_TO_RADIANS),
            "pravinGetFirstBall");

        addPath(
            trajectoryManager,
            startTrajectory(-77.25, 6.2, 180 * Helpers.DEGREES_TO_RADIANS, 90 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-56.2, 59.9, 67.8 * Helpers.DEGREES_TO_RADIANS), 67.8 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-45.2, 87.9), 67.8 * Helpers.DEGREES_TO_RADIANS),
            "pravinGetSecondBall");

        addPath(
            trajectoryManager,
            startTrajectory(-45.2, 87.9, 67.8 * Helpers.DEGREES_TO_RADIANS, 0)
                .splineToSplineHeading(new Pose2d( -45.2, 0.0, -20.90 * Helpers.DEGREES_TO_RADIANS), -20.90 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(0.0, 0.0), -20.90 * Helpers.DEGREES_TO_RADIANS),
            "pravinMoveToShoot");

        // PRAVIN'S 4 Ball

        addPath(
            trajectoryManager, 
            startTrajectory(0, 0, -178.6 * Helpers.DEGREES_TO_RADIANS, -178.6 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-40,0), -178.6 * Helpers.DEGREES_TO_RADIANS),
            "pravinGetFirstBall4");
        
        addPath(
            trajectoryManager,
            startTrajectory(-40, 0, -178.6 * Helpers.DEGREES_TO_RADIANS, 45 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-16.2, 15.2, -20.6 * Helpers.DEGREES_TO_RADIANS), -20.6 * Helpers.DEGREES_TO_RADIANS), 
            "pravinMoveToShootFirstSetBalls");
        
        addPath(
            trajectoryManager,
            startTrajectory(-16.2, 15.2, -20.6 * Helpers.DEGREES_TO_RADIANS, 75 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(1.6, -75, 90 * Helpers.DEGREES_TO_RADIANS), 90 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(1.6, -100), 90 * Helpers.DEGREES_TO_RADIANS), 
            "pravinGetSecondBall4");
        
        addPath(
            trajectoryManager,
            startTrajectory(1.6, -100, 90 * Helpers.DEGREES_TO_RADIANS, 110 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(9.2, 219.5, 133.9 * Helpers.DEGREES_TO_RADIANS), 133.9 * Helpers.DEGREES_TO_RADIANS)
                .splineToConstantHeading(new Vector2d(-11.6, 241.1), 133.9 * Helpers.DEGREES_TO_RADIANS),
            "pravinGetThirdBall4");
        
        addPath(
            trajectoryManager,
            startTrajectory(-11.6, 241.1, 133.9 * Helpers.DEGREES_TO_RADIANS, -45 * Helpers.DEGREES_TO_RADIANS)
                .splineToSplineHeading(new Pose2d(-16.2, 15.2, -20.6 * Helpers.DEGREES_TO_RADIANS), -20.6 * Helpers.DEGREES_TO_RADIANS),
            "pravinMoveToShootSecondSetBalls");

    }

    private static TrajectoryBuilder startTrajectory()
    {
        return startTrajectory(0.0, 0.0, 0.0, 0.0);
    }

    private static TrajectoryBuilder startTrajectory(double startTangent)
    {
        return startTrajectory(0.0, 0.0, 0.0, startTangent);
    }

    private static TrajectoryBuilder startTrajectory(double startXPos, double startYPos, double startHeading, double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(startXPos, startYPos, startHeading), startTangent, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
    }

    private static void addPath(TrajectoryManager trajectoryManager, TrajectoryBuilder trajectoryBuilder, String name)
    {
        try
        {
            trajectoryManager.addTrajectory(name, trajectoryBuilder);
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