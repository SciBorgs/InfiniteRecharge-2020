package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.stateEstimation.*;
import frc.robot.logging.*;
import frc.robot.shapes.*;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.controllers.*;
import frc.robot.robotState.*;

public class Robot extends TimedRobot {
    private Timer timer = new Timer();
    private final String FILENAME = "Robot.java";

    public static Logger logger = new Logger();

    public static RobotStateHistory stateHistory = new RobotStateHistory();
    public static DriveSubsystem      driveSubsystem      = new DriveSubsystem();

    public static PigeonSubsystem     pigeonSubsystem     = new PigeonSubsystem();
    public static GearShiftSubsystem  gearShiftSubsystem  = new GearShiftSubsystem();
    public static LimelightSubsystem  limelightSubsystem  = new LimelightSubsystem();
    public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    
    public static Following following = new Following();
    public static Model positionModel = new EncoderLocalization();
    public static CircleController circleController = new CircleController();
    public static OI oi = new OI();

    public static RobotState getState(){ return stateHistory.currentState(); }
    public static RobotState statesAgo(int numTicks){return stateHistory.statesAgo(numTicks);}
    private List<Pair<SD, DefaultValue>> dataToLog = new ArrayList<>();

    public static double get(SD sd)            {return getState().get(sd);}
    public static void   set(SD sd, double val){       getState().set(sd, val);}
    public static Value getSolenoidValue(SD sd){return getState().getSolenoidValue(sd);}

    public static double getDifference(SD sd, int ticksBack1, int ticksBack2) {
        return StateInfo.getDifference(Robot.stateHistory, sd, ticksBack1, ticksBack2);
    }
    public static double getDifference(SD sd, int ticksBack) {
        return StateInfo.getDifference(Robot.stateHistory, sd, ticksBack);
    }

    public static double getAngularVelocity() {return StateInfo.getAngularVelocity(Robot.stateHistory);}
    public static double getXVelocity()       {return StateInfo.getXVelocity(      Robot.stateHistory);}
    public static double getYVelocity()       {return StateInfo.getYVelocity(      Robot.stateHistory);}
    public static double getSpeedSquared()    {return StateInfo.getSpeedSquared(   Robot.stateHistory);}
    public static double getSpeed()           {return StateInfo.getSpeed(          Robot.stateHistory);}

    public static double getAvgWheelInput() {return StateInfo.getAvgWheelInput(getState());}

    public static double getWheelSpeed(CANSparkMax wheel){return StateInfo.getWheelSpeed(Robot.stateHistory, wheel);}

    // testing
    public static Point  getPos() {return new Point(get(SD.X),get(SD.Y));}
    public static double getHeading() {return get(SD.PigeonAngle);}
    public static final Point TEST_POINT = new Point (3, 4);
    public static final double TEST_HEADING = Math.PI * .1;
    public static final Point ORIGINAL_POINT = new Point(0,0);
    public static final double ORIGINAL_ANGLE = Geo.HORIZONTAL_ANGLE;
    

    private int attemptsSinceLastLog;
    public static final int LOG_PERIOD = 5;

    private void allPeriodicLogs() {
        driveSubsystem.periodicLog();
        gearShiftSubsystem.periodicLog();
        limelightSubsystem.periodicLog();
        pneumaticsSubsystem.periodicLog();
        following.periodicLog();
        logState();
    }
    
    private void allUpdateRobotStates() {
        driveSubsystem.updateRobotState();
        gearShiftSubsystem.updateRobotState();
        pneumaticsSubsystem.updateRobotState();
        pigeonSubsystem.updateRobotState();
        positionModel.updateRobotState();
    }

    public void addSDToLog(SD sd, DefaultValue val) { this.dataToLog.add(new Pair<>(sd, val)); }
    public void addSDToLog(SD sd)                   { addSDToLog(sd, DefaultValue.Empty); }

    public void logState() {
        for (Pair<SD, DefaultValue> pair : this.dataToLog) {
            SD sd = pair.first;
            Robot.logger.addData(FILENAME, sd.name(), get(sd), pair.second);
        }
    }

    public void robotInit() {
        timer.start();
        // attemptsSinceLastLog = 0;
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.PigeonAngle, ORIGINAL_ANGLE);
        allUpdateRobotStates();
        positionModel.updateRobotState();
        pneumaticsSubsystem.stopCompressor();
        //logger.incrementPrevious("robot.java", "deploy", DefaultValue.Previous);
        //logger.logData();
    }

    public void logDataPeriodic() {
        // if (LOG_PERIOD == attemptsSinceLastLog) {
        //     attemptsSinceLastLog = 0;
        //     allPeriodicLogs();
        //     logger.logData();
        // } else {
        //     attemptsSinceLastLog++;
        // }
    }
 
    public void robotPeriodic() {
        allUpdateRobotStates();
        Scheduler.getInstance().run();
        stateHistory.addState(getState().copy());
        positionModel.updateRobotState();
        DelayedPrinter.print("X: " + get(SD.X) +"\tY: " + get(SD.Y) +" Theta: " + get(SD.PigeonAngle));
    }
        
    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
        pneumaticsSubsystem.startCompressor();
        enabledPeriodic();
    }
    
    @Override
    public void teleopInit() {
    }

    public void teleopPeriodic() {
        new TankDriveCommand().start();
        pneumaticsSubsystem.startCompressor();
        enabledPeriodic();
    }

    public void testPeriodic() {
        enabledPeriodic();
    }

    public void enabledPeriodic() {logDataPeriodic();}

    public void disabledInit() {
        // allPeriodicLogs();
        // logger.logData();
        // logger.writeLoggedData();
    }
}
