package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import javax.swing.text.Position;

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
import frc.robot.sciSensorsActuators.SciSpark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.controllers.*;

public class Robot extends TimedRobot {
    private Timer timer = new Timer();
    private final String FILENAME = "Robot.java";

    public static Logger logger = new Logger();
    private static List<Pair<SD, DefaultValue>> dataToLog = new ArrayList<>();
    public static ArrayList<LogUpdater> logUpdaters = new ArrayList<>();

    public static ArrayList<RobotStateUpdater> robotStateUpdaters  = new ArrayList<>();
    public static RobotStateHistory stateHistory = new RobotStateHistory();
    static {
        if (stateHistory.numberOfStates() == 0){
            stateHistory.addState(new RobotState());
        }
    }

    public static DriveSubsystem      driveSubsystem      = new DriveSubsystem();
    public static PigeonSubsystem     pigeonSubsystem     = new PigeonSubsystem();
    public static LimelightSubsystem  limelightSubsystem  = new LimelightSubsystem();
    public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

    public static LimelightLocalization limelightLocalization = new LimelightLocalization();
    public static IntakeSubsystem     intakeSubsystem     = new IntakeSubsystem();
    
    public static Following following = new Following();
    public static CircleController circleController = new CircleController();
    public static OI oi = new OI();

    public static Updater positionUpdater = new EncoderLocalization();

    public static Model positionModel = new MaybeDefaultUpdater(new LimelightLocalization(), positionUpdater);

    public static RobotState getState(){ return stateHistory.currentState(); }
    public static RobotState statesAgo(int numTicks){return stateHistory.statesAgo(numTicks);}

    public static double get(SD sd)            {return getState().get(sd);}
    public static void   set(SD sd, double val){       getState().set(sd, val);}
    public static void optionalSet(Optional<SD> optionalSD , double val){       
        getState().optionalSet(optionalSD, val);
    }
    public static<K> void optionalMappedSet(BiHashMap<K, Double> biMap, Optional<SD> optionalSD, K val){       
        getState().optionalMappedSet(biMap, optionalSD, val);
    }


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

    public static double getWheelSpeed(SciSpark wheel){return StateInfo.getWheelSpeed(Robot.stateHistory, wheel);}

    // testing
    public static Point  getPos() {return new Point(get(SD.X),get(SD.Y));}
    public static double getHeading() {return get(SD.Angle);}

    public static final Waypoint TEST_POINT_1 = new Waypoint(new Point(Utils.inchesToMeters(36),0),     Geo.HORIZONTAL_ANGLE);
    public static final Waypoint TEST_POINT_2 = new Waypoint(new Point(Utils.inchesToMeters(96),Utils.inchesToMeters(36)),    Geo.HORIZONTAL_ANGLE + Math.PI);
    public static final Waypoint TEST_POINT_3 = new Waypoint(new Point(Utils.inchesToMeters(36),Utils.inchesToMeters(72)),     Geo.HORIZONTAL_ANGLE + Math.PI);
    public static final Waypoint TEST_POINT_4 = new Waypoint(new Point(0, Utils.inchesToMeters(72)), Geo.HORIZONTAL_ANGLE + Math.PI);
    public static final Point ORIGINAL_POINT = new Point(0,0);
    public static final double ORIGINAL_ANGLE = Geo.HORIZONTAL_ANGLE;
    public Waypoint[] arr = new Waypoint[] {TEST_POINT_1, TEST_POINT_2, TEST_POINT_3, TEST_POINT_4};
    public ArrayList <Waypoint> path = new ArrayList<Waypoint>(Arrays.asList(arr));

    public Sequential sequential = new Sequential(path);

    public static Point CURRENT_DESTINATION = ORIGINAL_POINT;
    public static double CURRENT_DESTINATION_HEADING = Geo.HORIZONTAL_ANGLE;

    public static Point newDestPoint = new Point(Utils.inchesToMeters(4), .458);
    public static double newDestHeading = Geo.HORIZONTAL_ANGLE;
    private int attemptsSinceLastLog;
    public static final int LOG_PERIOD = 5;

    public static void addLogUpdater(LogUpdater logUpdater) {
        logUpdaters.add(logUpdater);
    }

    private void allPeriodicLogs() {
        for (LogUpdater i : logUpdaters) {
            i.periodicLog();
        }
        logState();
    }
    
    public static void addRobotStateUpdater(RobotStateUpdater robotStateUpdater){
        robotStateUpdaters.add(robotStateUpdater);
    }
    private void allUpdateRobotStates() {
        for (RobotStateUpdater i : robotStateUpdaters) {
            i.updateRobotState();
        }
    }

    private void allModels(){
        positionModel.updateRobotState();
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
        // logger.incrementPrevious("robot.java", "deploy", DefaultValue.Previous);
        // logger.logData();
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
        allModels();
        allPeriodicLogs();
        logDataPeriodic();
        DelayedPrinter.print("x: " + getPos().x + "\ty: " + getPos().y + "\nheading: " + getHeading() + "\npigeon angle: " + Robot.get(SD.PigeonAngle));
        Scheduler.getInstance().run();
        DelayedPrinter.incTicks();
    }


    public void autonomousInit() {
        Robot.driveSubsystem.assistedDriveMode();
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
        intakeSubsystem.reverseIntake();
    }

    @Override
    public void autonomousPeriodic() {
        sequential.update();
    }
    
    @Override
    public void teleopInit() {
        intakeSubsystem.reverseIntake();
        pneumaticsSubsystem.startCompressor();
    }

    public void teleopPeriodic() {
        (new TankDriveCommand()).start();
    }
    

    public void testPeriodic() {}

    public void disabledInit() {
        allPeriodicLogs();
        logger.logData();
        logger.writeLoggedData();
    }
}
