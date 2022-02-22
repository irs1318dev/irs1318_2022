package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private final IDriver driver;
    private final IDriverStation driverStation;
    private final INetworkTableProvider networkTable;
    private final ILogger logger;

    private final IDigitalOutput reflectiveRingLight;
    private final IDigitalOutput cargoRingLight;

    private double centerX;
    private double centerY;
    private double width;
    private double height;
    private double angle;

    private int missedHeartbeats;
    private double prevHeartbeat;

    private Double distance;
    private Double horizontalAngle;

    /**
     * Initializes a new OffboardVisionManager
     * @param driver for obtaining operations
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.driverStation = provider.getDriverStation();

        this.networkTable = provider.getNetworkTableProvider();
        this.reflectiveRingLight = provider.getDigitalOutput(ElectronicsConstants.VISION_RING_LIGHT_REFLECTIVE_DIO);
        this.cargoRingLight = provider.getDigitalOutput(ElectronicsConstants.VISION_RING_LIGHT_GAMEPIECE_DIO);

        this.centerX = 0.0;
        this.centerY = 0.0;

        this.width = 0.0;
        this.height = 0.0;
        this.angle = 0.0;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0.0;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.centerX = this.networkTable.getSmartDashboardNumber("v.pointX");
        this.centerY = this.networkTable.getSmartDashboardNumber("v.pointY");
        this.width = this.networkTable.getSmartDashboardNumber("v.width");
        this.height = this.networkTable.getSmartDashboardNumber("v.height");
        this.angle = this.networkTable.getSmartDashboardNumber("v.angle");

        this.logger.logNumber(LoggingKey.OffboardVisionX, this.centerX);
        this.logger.logNumber(LoggingKey.OffboardVisionY, this.centerY);
        this.logger.logNumber(LoggingKey.OffboardVisionWidth, this.width);
        this.logger.logNumber(LoggingKey.OffboardVisionHeight, this.height);
        this.logger.logNumber(LoggingKey.OffboardVisionAngle, this.angle);

        double newHeartbeat = this.networkTable.getSmartDashboardNumber("v.heartbeat");
        if (this.prevHeartbeat != newHeartbeat)
        {
            this.missedHeartbeats = 0;
        }
        else
        {
            this.missedHeartbeats++;
        }

        this.logger.logNumber(LoggingKey.OffboardVisionMissedHeartbeats, this.missedHeartbeats);

        // return if we couldn't find a vision target
        if (this.centerX < 0.0 || this.centerY < 0.0 || this.missedHeartbeats > TuningConstants.VISION_MISSED_HEARTBEAT_THRESHOLD)
        {
            this.distance = null;
            this.horizontalAngle = null;

            return;
        }

        double yOffset = VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH - this.centerY;
        double verticalAngle = Helpers.atand(yOffset / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_Y);

        this.distance = (HardwareConstants.CAMERA_TO_TARGET_Z_OFFSET / Helpers.tand(verticalAngle + HardwareConstants.CAMERA_PITCH)) - HardwareConstants.CAMERA_X_OFFSET;

        double xOffset = this.centerX - VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH;
        this.horizontalAngle = Helpers.atand(xOffset / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_X) + HardwareConstants.CAMERA_YAW;

        this.logger.logNumber(LoggingKey.OffboardVisionDistance, this.distance);
        this.logger.logNumber(LoggingKey.OffboardVisionHorizontalAngle, this.horizontalAngle);
    }

    @Override
    public void update()
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = !this.driver.getDigital(DigitalOperation.VisionDisableStream);
        boolean enableGamePieceProcessing = this.driver.getDigital(DigitalOperation.VisionEnableGamePieceProcessing);
        boolean enableRetroreflectiveProcessing = this.driver.getDigital(DigitalOperation.VisionEnableRetroreflectiveProcessing);

        double visionProcessingMode = 0.0;
        if (enableVision)
        {
            if (enableRetroreflectiveProcessing)
            {
                visionProcessingMode = 1.0;
            }
            else if (enableGamePieceProcessing)
            {
                if (this.isRedTeam())
                {
                    visionProcessingMode = 2.0; // 2.0 means red team
                }
                else {
                    visionProcessingMode = 3.0; // 3.0 means blue team
                }
            }
        }

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, enableVision);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logNumber(LoggingKey.OffboardVisionEnableProcessing, visionProcessingMode);

        this.reflectiveRingLight.set(enableVision && enableRetroreflectiveProcessing);
        this.cargoRingLight.set(enableVision && enableGamePieceProcessing);
    }

    @Override
    public void stop()
    {
        this.reflectiveRingLight.set(false);
        this.cargoRingLight.set(false);

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, false);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logNumber(LoggingKey.OffboardVisionEnableProcessing, 0.0);
    }

    public Double getHorizontalAngle()
    {
        return this.horizontalAngle;
    }

    public Double getDistance()
    {
        return this.distance;
    }

    public double getPowercellX() 
    {
        return this.centerX;
    }

    public double getPowercellY() 
    {
        return this.centerY;
    }

    private boolean isRedTeam()
    {
        Alliance currentAlliance = this.driverStation.getAlliance();
        switch (currentAlliance)
        {
            case Red:
                return true;

            case Blue:
                return false;

            case Invalid:
            default:
                break;
        }

        // fallback: use the game-specific message.  if "red", use red, otherwise use blue
        return this.driverStation.getGameSpecificMessage().equalsIgnoreCase("red");
    }
}
