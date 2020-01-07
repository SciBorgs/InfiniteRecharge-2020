package frc.robot.subsystems;

import java.util.Hashtable;

import frc.robot.Robot;
import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DualOutputGearboxSubsystem extends Subsystem { // Set ratio, motor, change shaft
    private final String FILENAME = "DualOutputGearboxSubsystem.java";
    
    public final DoubleSolenoid.Value OPEN_VALUE = Value.kForward;
    public final DoubleSolenoid.Value CLOSE_VALUE = Utils.oppositeDoubleSolenoidValue(OPEN_VALUE);

    private DoubleSolenoid GEAR_SHIFT_SOLENOID;
    private DoubleSolenoid OUTPUT_SHIFT_SOLENOID;

    public SciSpark l, l1, l2, r, r1, r2;
    public Hashtable<SciSpark, SD> sparkToWheelAngleSD;
    public Hashtable<SciSpark, SD> sparkToValueSD;
    public Hashtable<SciSpark, SD> sparkToVoltageSD;
    public Hashtable<SciSpark, SD> sparkToCurrentSD;

    enum Gearbox {
        Climber(2),
        Drive(0);
        
        private double gearRatio;

        Gearbox(double gearRatio) {
            this.gearRatio = gearRatio;
        }

        public double getGearRatio() {
            return gearRatio;
        }
    }

    public DualOutputGearboxSubsystem() {
        GEAR_SHIFT_SOLENOID   = Utils.newDoubleSolenoid(PortMap.GEAR_RATIO_SOLENOID_PDP, PortMap.GEAR_RATIO_SOLENOID);
        OUTPUT_SHIFT_SOLENOID = Utils.newDoubleSolenoid(PortMap.OUTPUT_SOLENOID_PDP, PortMap.OUTPUT_SOLENOID);
        
        this.sparkToWheelAngleSD = new Hashtable<>();
        this.sparkToValueSD = new Hashtable<>();
        this.sparkToVoltageSD = new Hashtable<>();
        this.sparkToCurrentSD = new Hashtable<>();
        
        this.l  = new SciSpark(PortMap.LEFT_FRONT_SPARK);
		this.l1 = new SciSpark(PortMap.LEFT_MIDDLE_SPARK);
        this.l2 = new SciSpark(PortMap.LEFT_BACK_SPARK);
        
		this.r  = new SciSpark(PortMap.RIGHT_FRONT_SPARK);
		this.r1 = new SciSpark(PortMap.RIGHT_MIDDLE_SPARK);
        this.r2 = new SciSpark(PortMap.RIGHT_BACK_SPARK);

        this.l .setInverted(true);
        this.l1.setInverted(true);
        this.l2.setInverted(true);

        this.l1.follow(this.l);
        this.l2.follow(this.l);

        this.r1.follow(this.r);
        this.r2.follow(this.r);

        // Mappings for logging
        setSDMappings(this.l, SD.LeftWheelAngle,  SD.LeftSparkVal,  SD.LeftSparkVoltage,  SD.LeftSparkCurrent);
        setSDMappings(this.r, SD.RightWheelAngle, SD.RightSparkVal, SD.RightSparkVoltage, SD.RightSparkCurrent);
        
        setSDMappings(this.l1, SD.L1WheelAngle, SD.L1SparkVal, SD.L1SparkVoltage, SD.L1SparkCurrent);
        setSDMappings(this.r1, SD.R1WheelAngle, SD.R1SparkVal, SD.R1SparkVoltage, SD.R1SparkCurrent);
        setSDMappings(this.l2, SD.L2WheelAngle, SD.L2SparkVal, SD.L2SparkVoltage, SD.L2SparkCurrent);
        setSDMappings(this.r2, SD.R2WheelAngle, SD.R2SparkVal, SD.R2SparkVoltage, SD.R2SparkCurrent);
    }
    
    public void shiftGearsPush()     { GEAR_SHIFT_SOLENOID.set(OPEN_VALUE); }

    public void shiftGearRetract()   { GEAR_SHIFT_SOLENOID.set(CLOSE_VALUE); }

    public void shiftOutputPush()    { OUTPUT_SHIFT_SOLENOID.set(OPEN_VALUE); }
    
    public void shiftOutputRetract() { OUTPUT_SHIFT_SOLENOID.set(CLOSE_VALUE); }
    
    public void setSDMappings(SciSpark spark, SD wheelAngleSD, SD valueSD, SD volatageSD, SD currentSd){
        this.sparkToWheelAngleSD.put(spark, wheelAngleSD);
        this.sparkToValueSD     .put(spark, valueSD);
        this.sparkToVoltageSD   .put(spark, volatageSD);
        this.sparkToCurrentSD   .put(spark, currentSd);
    }
    
    public SciSpark[] getSparks() {
        return new SciSpark[]{this.l, this.l1, this.l2, this.r, this.r1, this.r2};
    }
    
    public void updateRobotState(){
        for(SciSpark spark : getSparks()){updateSparkState(spark);}
    }
    
    public void updateSparkState(SciSpark spark){
        Robot.set(this.sparkToWheelAngleSD.get(spark), spark.getWheelAngle());
        Robot.set(this.sparkToValueSD.get(spark),   spark.get());
        Robot.set(this.sparkToVoltageSD.get(spark), spark.getBusVoltage());
        Robot.set(this.sparkToCurrentSD.get(spark), spark.getOutputCurrent());
    }
    
    public void initDefaultCommand() {
        // Please ignore.
    }
}