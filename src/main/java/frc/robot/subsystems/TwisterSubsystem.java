package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SerialPort.Port;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.*;
import frc.robot.sciSensorsActuators.SciTalon;
import frc.robot.sciSensorsActuators.SciAtlasColor.SciAtlasColorSD;
import frc.robot.colors.*;
import frc.robot.helpers.Colors;
import frc.robot.robotState.RobotState.SD;

public class TwisterSubsystem extends Subsystem {
  private static final double WHEEL_SPEED = 0.2;
  private static final double HUE_THRESHOLD = 30.0 / 360.0; // to be changed after testing
  private static final HSV PANEL_RED = Colors.toHSV(new CMYK(0, 1, 1, 0)); // hue 0
  private static final HSV PANEL_YELLOW = Colors.toHSV(new CMYK(0, 0, 1, 0)); // hue 60
  private static final HSV PANEL_BLUE = Colors.toHSV(new CMYK(1, 0, 0, 0)); // hue 180
  private static final HSV PANEL_GREEN = Colors.toHSV(new CMYK(1, 0, 1, 0)); // hue 120

  public static enum Color {
    Blue, Green, Red, Yellow, Other
  }

  // TODO: Better naming
  public static enum PistonState {
    Extended, Retracted, Off
  }

  private SciAtlasColor colorSensor;
  private SciTalon talon;
  private SciSolenoid<PistonState> solenoid;

  @Override
  public void initDefaultCommand() {
  }

  public TwisterSubsystem() {
    this.colorSensor = new SciAtlasColor(Port.kMXP);
    this.talon = new SciTalon(PortMap.TWISTER_TALON);
    this.solenoid = new SciSolenoid<>(0, PortMap.TWISTER_SOLENOID, PistonState.Extended, PistonState.Retracted, PistonState.Off);
    talon.setNeutralMode(NeutralMode.Brake);
    this.colorSensor.assignSD(SciAtlasColorSD.H, SD.ColorSensorH);
    this.colorSensor.assignSD(SciAtlasColorSD.S, SD.ColorSensorS);
    this.colorSensor.assignSD(SciAtlasColorSD.V, SD.ColorSensorV);
  }

  public void startWheel() {
    talon.set(WHEEL_SPEED);
  }

  public void stopWheel() {
    talon.set(0);
  }

  public Color getColor() {
    if (Robot.get(SD.ColorSensorH) >= PANEL_RED.getH() - HUE_THRESHOLD
    && Robot.get(SD.ColorSensorH) <= PANEL_RED.getH() + HUE_THRESHOLD)
      return Color.Red;
    else if (Robot.get(SD.ColorSensorH) >= PANEL_YELLOW.getH() - HUE_THRESHOLD
          && Robot.get(SD.ColorSensorH) <= PANEL_YELLOW.getH() + HUE_THRESHOLD)
      return Color.Yellow;
    else if (Robot.get(SD.ColorSensorH) >= PANEL_BLUE.getH() - HUE_THRESHOLD
          && Robot.get(SD.ColorSensorH) <= PANEL_BLUE.getH() + HUE_THRESHOLD)
      return Color.Blue;
    else if (Robot.get(SD.ColorSensorH) >= PANEL_GREEN.getH() - HUE_THRESHOLD
          && Robot.get(SD.ColorSensorH) <= PANEL_GREEN.getH() + HUE_THRESHOLD)
      return Color.Green;
    else
      return Color.Other;
  }

  public boolean didColorChange(int statesAgo) {
    for (int i = statesAgo; i > 0; i--) {
      if (Robot.get(SD.ColorSensorH) - Robot.stateHistory.statesAgo(i).get(SD.ColorSensorH) <= HUE_THRESHOLD)
        return false;
    }
    return true;
  }

  public void toggleDoubleSolenoid() {
    solenoid.toggle();
  }

  public void find() {
    colorSensor.find();
  }
}
