package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.logging.LogUpdater;
import frc.robot.logging.Logger.DefaultValue;

public class PneumaticsSubsystem extends Subsystem implements RobotStateUpdater, LogUpdater {
  
  private AnalogInput pressureSensor;
  private final double NORMALIZED_SUPPLY_VOLTAGE = 5.0;
  private Compressor compressor;
  private final String FILENAME = "PneumaticsSubsystem.java";
  public static final SD VOLTAGE_SD = SD.PressureSensorVoltage;
  
  @Override
  public void initDefaultCommand() {}

  public PneumaticsSubsystem() {
    this.pressureSensor = new AnalogInput(PortMap.PRESSURE_SENSOR);
    //Robot.set(SD.PressureSensorVoltage, 0.0);
    this.compressor = new Compressor();
    //Robot.addSDToLog(SD.PressureSensorVoltage);
    Robot.addRobotStateUpdater(this);
    Robot.addLogUpdater(this);
  }
    
	public void periodicLog(){
    Robot.logger.addData(FILENAME, "pressure", getPressure(), DefaultValue.Previous);
  }
  @Override
  public void updateRobotState(){
    Robot.set(SD.PressureSensorVoltage, pressureSensor.getVoltage());
  }

  public double getPressure() {
    // QUESTION: What's up with 250 and 15? Magic fucking numbers, shmdfh
    return 250.0 * getRawVoltage() / NORMALIZED_SUPPLY_VOLTAGE - 15.0;
  }

  public void startCompressor(){this.compressor.start();}
  public void stopCompressor() {this.compressor.stop();}
  public double getRawVoltage(){return Robot.get(SD.PressureSensorVoltage);}
}
