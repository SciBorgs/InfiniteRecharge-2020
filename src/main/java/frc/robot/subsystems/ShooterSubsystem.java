package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
  private TalonSRX topTalon, bottomTalon;
  public double topOutput, bottomOutput;

  public ShooterSubsystem() {
    this.topTalon    = new TalonSRX(4);
    this.bottomTalon = new TalonSRX(8);
    this.topOutput = bottomOutput = 0; 
  }

  public void setSpeed() {
    this.topTalon.set(ControlMode.PercentOutput, topOutput);
    this.bottomTalon.set(ControlMode.PercentOutput, bottomOutput);
  }

  @Override
  protected void initDefaultCommand() {
		//IGNORE THIS METHOD
  }
}