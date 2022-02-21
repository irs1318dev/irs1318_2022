package frc.robot.common.robotprovider;

public class FauxbotCANdle implements ICANdle
{
    public FauxbotCANdle(int deviceNumber)
    {
    }

    public double getBusVoltage()
    {
        return 0.0;
    }

    public double get5VRailVoltage()
    {
        return 0.0;
    }

    public double getCurrent()
    {
        return 0.0;
    }

    public double getTemperature()
    {
        return 0.0;
    }

    public void configBrightnessScalar(double brightness)
    {
    }

    public void configLEDType(CANdleLEDStripType type)
    {
    }

    public void configLOSBehavior(boolean disableWhenLOS)
    {
    }

    public void configStatusLedState(boolean disableWhenRunning)
    {
    }

    public void configVBatOutput(CANdleVBatOutputMode mode)
    {
    }

    public void modulateVBatOutput(double dutyCyclePercent)
    {
    }

    public void setLEDs(int r, int g, int b)
    {
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count)
    {
    }

    public void startTwinkleAnimation(int r, int g, int b, int w, double speed, int numLed, CANdleTwinklePercent divider, int ledOffset)
    {
    }

    public void startTwinkleOffAnimation(int r, int g, int b, int w, double speed, int numLed, CANdleTwinklePercent divider, int ledOffset)
    {
    }

    public void startStrobeAnimation(int r, int g, int b, int w, double speed, int numLed, int ledOffset)
    {
    }

    public void startSingleFadeAnimation(int r, int g, int b, int w, double speed, int numLed, int ledOffset)
    {
    }

    public void startRgbFadeAnimation(double brightness, double speed, int numLed, int ledOffset)
    {
    }

    public void startRainbowAnimation(double brightness, double speed, int numLed, boolean reverseDirection, int ledOffset)
    {
    }

    public void startLarsonAnimation(int r, int g, int b, int w, double speed, int numLed, CANdleLarsonBounceMode mode, int size, int ledOffset)
    {
    }

    public void startFireAnimation(double brightness, double speed, int numLed, double sparking, double cooling, boolean reverseDirection, int ledOffset)
    {
    }

    public void startColorFlowAnimation(int r, int g, int b, int w, double speed, int numLed, boolean forward, int ledOffset)
    {
    }
}