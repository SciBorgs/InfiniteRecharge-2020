package frc.robot;

import com.revrobotics.CANSparkMax;
import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.dataTypes.*;
import frc.robot.stateEstimation.*;
import frc.robot.logging.*;
import frc.robot.shapes.*;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.controllers.*;

public class Robot extends TimedRobot {
    private Timer timer = new Timer();
    private final String FILENAME = "Robot.java";

    public static Logger logger = new Logger();
    private static List<Pair<SD, DefaultValue>> dataToLog = new ArrayList<>();

    public static RobotStateHistory stateHistory = new RobotStateHistory();
    static {
        stateHistory.addState(new RobotState());
    }
    public static DriveSubsystem      driveSubsystem      = new DriveSubsystem();

    public static PigeonSubsystem     pigeonSubsystem     = new PigeonSubsystem();
    public static LimelightSubsystem  limelightSubsystem  = new LimelightSubsystem();
    public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    
    public static Following following = new Following();
    public static Model positionModel = new EncoderLocalization();
    public static CircleController circleController = new CircleController();
    public static OI oi = new OI();

    public static RobotState getState(){ return stateHistory.currentState(); }
    public static RobotState statesAgo(int numTicks){return stateHistory.statesAgo(numTicks);}

    public static double get(SD sd)            {return getState().get(sd);}
    public static void   set(SD sd, double val){       getState().set(sd, val);}

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
    public static double getHeading() {return get(SD.Angle);}
    public static final Point TEST_POINT_1 = new Point (6, 0);
    public static final double TEST_HEADING_1 = Geo.HORIZONTAL_ANGLE;
    public static final Point TEST_POINT_2 = new Point (9.144, 0);
    public static final double TEST_HEADING_2 = Geo.HORIZONTAL_ANGLE - Math.PI / 2;
    public static final Point TEST_POINT_3 = new Point (9.144, -5.1816);
    public static final double TEST_HEADING_3 = Geo.HORIZONTAL_ANGLE + Math.PI;
    public static final Point TEST_POINT_4 = new Point (0, -5.1816);
    public static final double TEST_HEADING_4 = Geo.HORIZONTAL_ANGLE + Math.PI;
    public static final Point ORIGINAL_POINT = new Point(0,0);
    public int lastPtHit = 0;
    public static final double ORIGINAL_ANGLE = Geo.HORIZONTAL_ANGLE;
    

    private int attemptsSinceLastLog;
    public static final int LOG_PERIOD = 5;

    private void allPeriodicLogs() {
        driveSubsystem.periodicLog();
        limelightSubsystem.periodicLog();
        pneumaticsSubsystem.periodicLog();
        following.periodicLog();
        logState();
    }
    
    private void allUpdateRobotStates() {
        driveSubsystem.updateRobotState();
        pneumaticsSubsystem.updateRobotState();
        pigeonSubsystem.updateRobotState();
    }

    public static void addSDToLog(SD sd, DefaultValue val) { Robot.dataToLog.add(new Pair<>(sd, val)); }
    public static void addSDToLog(SD sd)                   { addSDToLog(sd, DefaultValue.Empty); }

    public void logState() {
        for (Pair<SD, DefaultValue> pair : Robot.dataToLog) {
            SD sd = pair.first;
            if (getState().contains(sd)){
                Robot.logger.addData("State", sd.name(), get(sd), pair.second);
            }
        }
    }

    public void robotInit() {
        timer.start();
        // attemptsSinceLastLog = 0;
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
        allUpdateRobotStates();
        pneumaticsSubsystem.stopCompressor();
        //logger.incrementPrevious("robot.java", "deploy", DefaultValue.Previous);
        //logger.logData();
        addSDToLog(SD.X);
        addSDToLog(SD.Y);
        addSDToLog(SD.Angle);
    }

    public void logDataPeriodic() {
        logger.logData();
    }
 
    public void robotPeriodic() {
        stateHistory.addState(getState().copy());
        allUpdateRobotStates();
        positionModel.updateRobotState();
        allPeriodicLogs();
        logDataPeriodic();
        DelayedPrinter.print("x: " + getPos().x + "\ty: " + getPos().y + 
                             "\nheading: " + getHeading() + 
                             "\npigeon angle: " + Robot.get(SD.PigeonAngle));
        Scheduler.getInstance().run();
        DelayedPrinter.incTicks();
    }


    public void autonomousInit() {
        Robot.driveSubsystem.assistedDriveMode();
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
    }

    @Override
    public void autonomousPeriodic() {
        // pneumaticsSubsystem.startCompressor();
    }
    
    @Override
    public void teleopInit() {
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
    }

    public void teleopPeriodic() {
        // new TankDriveCommand().start();
        if(Geo.getDistance(TEST_POINT_1, getPos()) < .2) {
            lastPtHit = 1;
        } else if(Geo.getDistance(TEST_POINT_2, getPos()) < .2) {
            lastPtHit = 2;
        } else if(Geo.getDistance(TEST_POINT_3, getPos()) < .2) {
            lastPtHit = 3;
        }
        
        if(lastPtHit == 1) {
            circleController.update(getPos(), getHeading(), TEST_POINT_2, TEST_HEADING_2);
        } else if(lastPtHit == 2) {
            circleController.update(getPos(), getHeading(), TEST_POINT_3, TEST_HEADING_3);
        } else if(lastPtHit == 3) {
            circleController.update(getPos(), getHeading(), TEST_POINT_4, TEST_HEADING_4);
        } else {
            circleController.update(getPos(), getHeading(), TEST_POINT_1, TEST_HEADING_1);
        }
    }
        // pneumaticsSubsystem.startCompressor();
    

    public void testPeriodic() {
    }

    public void disabledInit() {
        allPeriodicLogs();
        logger.logData();
        logger.writeLoggedData();
    }
}
