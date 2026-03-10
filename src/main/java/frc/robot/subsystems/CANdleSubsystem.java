package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class CANdleSubsystem extends SubsystemBase {

    private final CANdle m_candle = new CANdle(DeviceIds.CANdle.CANdleId, "rio");
    private final int LedCount = 80;
    public String color;


    private Animation m_toAnimate = null;

    public CANdleSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
         m_candle.animate(m_toAnimate);
    }

    public void setAnimate(String color){  // green is red
        this.color = color;
        switch(color) {
            case "Purple":
                m_toAnimate = new SingleFadeAnimation(128, 0, 128, 0, 0.7, LedCount);
                break;
            case "Yellow":
                m_toAnimate = new SingleFadeAnimation(255, 255, 0, 0, 0.7, LedCount);
                break;
            case "Aqua":
                m_toAnimate = new SingleFadeAnimation(0, 200, 40, 0, 0.7, LedCount);
                break;
            case "Red":
                m_toAnimate = new SingleFadeAnimation(255, 0, 0, 0, 0.7, LedCount);
                break;
            case "Rainbow":
                m_toAnimate = new RainbowAnimation(0.8, 0.7, LedCount);
                break;
            case "ColorFlow":
                m_toAnimate = new ColorFlowAnimation(128, 128, 0, 0, .9, LedCount, Direction.Forward, 7);
                break;
            case "Strobe Aqua":
                m_toAnimate = new StrobeAnimation(0, 200, 40, 0, 0.4, LedCount);
                break;
            case "Strobe Purple":
                m_toAnimate = new StrobeAnimation(128, 0, 128, 0, 0.4, LedCount);
                break;
            case "Strobe Yellow":
                m_toAnimate = new StrobeAnimation(255, 255, 0, 0, 0.4, LedCount);
                break;
            case "Strobe Red":
                m_toAnimate = new StrobeAnimation(255, 0, 0, 0, 0.4, LedCount);
                break;
            }  
    }
}