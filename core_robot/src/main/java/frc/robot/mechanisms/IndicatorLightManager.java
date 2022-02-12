package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Indicator Light manager
 * 
 * This class manages indicator lights on the robot.
 * 
 */
@Singleton
public class IndicatorLightManager implements IMechanism
{
    private final CargoMechanism cargo;
    private final ICANdle candle;

    private enum LightTransition
    {
        NoChange,
        TurnOn,
        TurnOff;
    }

    private boolean hasFeederCargoLit;
    private boolean hasConveyorCargoLit;
    private boolean shooterSpunUpLit;

    @Inject
    public IndicatorLightManager(
        IRobotProvider provider,
        CargoMechanism cargo)
    {
        this.cargo = cargo;
        this.candle = provider.getCANdle(ElectronicsConstants.INDICATOR_LIGHT_CANDLE_CAN_ID);
        this.candle.configLEDType(CANdleLEDStripType.GRB);
        this.candle.configVBatOutput(CANdleVBatOutputMode.Off);

        this.hasFeederCargoLit = false;
        this.hasConveyorCargoLit = false;
        this.shooterSpunUpLit = false;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        boolean shouldFeederCargoLightBeOn = this.cargo.isFeederSensorBlocked();
        boolean shouldConveyorCargoLightBeOn = this.cargo.isConveyorSensorBlocked();
        boolean shouldSpinUpLightBeOn = this.cargo.isFlywheelSpunUp();

        // note: we only update the light strip sections when they should be changed (as opposed to every update loop)
        LightTransition updateFeederCargoLight = this.checkTransitionRequired(this.hasFeederCargoLit, shouldFeederCargoLightBeOn);
        LightTransition updateConveyorCargoLight = this.checkTransitionRequired(this.hasConveyorCargoLit, shouldConveyorCargoLightBeOn);
        LightTransition updateShooterSpunUpLight = this.checkTransitionRequired(this.shooterSpunUpLit, shouldSpinUpLightBeOn);

        if (updateFeederCargoLight != LightTransition.NoChange)
        {
            this.hasFeederCargoLit =
                this.updateLightRanges(
                    updateFeederCargoLight,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO_COLOR_RED,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO_COLOR_GREEN,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO_COLOR_BLUE,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO_COLOR_WHITE,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO1_START,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO1_COUNT,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO2_START,
                    TuningConstants.INDICATOR_SECTION_FEEDER_CARGO2_COUNT);
        }

        if (updateConveyorCargoLight != LightTransition.NoChange)
        {
            this.hasConveyorCargoLit =
                this.updateLightRanges(
                    updateConveyorCargoLight,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO_COLOR_RED,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO_COLOR_GREEN,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO_COLOR_BLUE,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO_COLOR_WHITE,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO1_START,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO1_COUNT,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO2_START,
                    TuningConstants.INDICATOR_SECTION_CONVEYOR_CARGO2_COUNT);
        }

        if (updateShooterSpunUpLight != LightTransition.NoChange)
        {
            this.shooterSpunUpLit =
                this.updateLightRanges(
                    updateShooterSpunUpLight,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP_COLOR_RED,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP_COLOR_GREEN,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP_COLOR_BLUE,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP_COLOR_WHITE,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP1_START,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP1_COUNT,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP2_START,
                    TuningConstants.INDICATOR_SECTION_SPIN_UP2_COUNT);
        }
    }

    @Override
    public void stop()
    {
        this.candle.startTwinkleAnimation(
            TuningConstants.TEAM_PURPLE_RED,
            TuningConstants.TEAM_PURPLE_GREEN,
            TuningConstants.TEAM_PURPLE_BLUE,
            TuningConstants.TEAM_PURPLE_WHITE,
            0.25,
            TuningConstants.CANDLE_TOTAL_NUMBER_LEDS,
            CANdleTwinklePercent.Percent42);
    }

    /**
     * Check whether there is a transition required given the current state and the new state
     * @param currentState the current state of the lights (on or off)
     * @param newState the new desired state of the lights (on or off)
     * @return NoChange if no state change is required, TurnOn if we need to turn the lights on, TurnOff if we need to turn the lights off
     */
    private LightTransition checkTransitionRequired(boolean currentState, boolean newState)
    {
        if (currentState == newState)
        {
            return LightTransition.NoChange;
        }

        if (newState)
        {
            return LightTransition.TurnOn;
        }

        return LightTransition.TurnOff;
    }

    /**
     * Update a pair of light ranges to a certiain color (if TurnOn) or to the off (if TurnOff)
     * @param updateType whether to turn the lights to the on color, or the off color (NoChange not supported!!)
     * @param onColorRed the on color's red content [0, 255]
     * @param onColorGreen the on color's green content [0, 255]
     * @param onColorBlue the on color's blue content [0, 255]
     * @param onColorWhite the on color's white content [0, 255]
     * @param range1Start the beginning of the first range to modify
     * @param range1Count the size of the first range to modify
     * @param range2Start the beginning of the second range to modify
     * @param range2Count the size of the second range to modify
     * @return true if we turned on the light ranges, false if we turned off the light ranges
     */
    private boolean updateLightRanges(
        LightTransition updateType,
        int onColorRed,
        int onColorGreen,
        int onColorBlue,
        int onColorWhite,
        int range1Start,
        int range1Count,
        int range2Start,
        int range2Count)
    {
        boolean result;

        int r;
        int g;
        int b;
        int w;
        if (updateType == LightTransition.TurnOn)
        {
            result = true;
            r = onColorRed;
            g = onColorGreen;
            b = onColorBlue;
            w = onColorWhite;
        }
        else // if (updateType == LightTransition.TurnOff)
        {
            result = false;
            r = TuningConstants.INDICATOR_OFF_COLOR_RED;
            g = TuningConstants.INDICATOR_OFF_COLOR_GREEN;
            b = TuningConstants.INDICATOR_OFF_COLOR_BLUE;
            w = TuningConstants.INDICATOR_OFF_COLOR_WHITE;
        }

        this.candle.setLEDs(r, g, b, w, range1Start, range1Count);
        this.candle.setLEDs(r, g, b, w, range2Start, range2Count);

        return result;
    }
}
