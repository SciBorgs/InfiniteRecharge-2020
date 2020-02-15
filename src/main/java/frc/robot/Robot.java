package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.dataTypes.*;
import frc.robot.logging.*;
import frc.robot.shapes.*;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.controllers.*;
import frc.robot.stateEstimation.interfaces.*;
import frc.robot.stateEstimation.higherLevel.*;
import frc.robot.stateEstimation.explicit.*;

public class Robot extends TimedRobot implements LogUpdater {
    private Timer timer = new Timer();

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
    public static ShooterSubsystem    shooterSubsystem    = new ShooterSubsystem();

    public static IntakeSubsystem     intakeSubsystem     = new IntakeSubsystem();
    
    public static Following following = new Following();
    public static CircleController circleController = new CircleController();
    public static OI oi = new OI();

    public static Model positionModel = new MaybeDefaultUpdater(new LimelightLocalization(), new EncoderLocalization());

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
    private int attemptsSinceLastLog = 0;
    public static final int LOG_PERIOD = 5;

    public Robot() {
        addLogUpdater(this);
    }

    public static void addLogUpdater(LogUpdater logUpdater) {
        logUpdaters.add(logUpdater);
    }
    public static void addRobotStateUpdater(RobotStateUpdater robotStateUpdater){
        robotStateUpdaters.add(robotStateUpdater);
    }

    private void allPeriodicLogs() {
        for (LogUpdater i : logUpdaters) {
            i.periodicLog();
        }
    }
    private void allUpdateRobotStates() {
        set(SD.Time, this.timer.get());
        //double t = this.timer.get();
        for (RobotStateUpdater i : robotStateUpdaters) {
            i.updateRobotState();
            //System.out.println("time diff: " + (this.timer.get() - t));
            //System.out.println("updating: " + i.getClass());
            //t = this.timer.get();
        }
    }

    private void allModels(){
        positionModel.updateRobotState();
    }

    public static void addSDToLog(SD sd, DefaultValue val) { Robot.dataToLog.add(new Pair<>(sd, val)); }
    public static void addSDToLog(SD sd)                   { addSDToLog(sd, DefaultValue.Empty); }

    public void periodicLog() {
        for (Pair<SD, DefaultValue> pair : Robot.dataToLog) {
            SD sd = pair.first;
            if (getState().contains(sd)){
                Robot.logger.addData("State", sd.name(), get(sd), pair.second);
            }
        }
    }

    public void robotInit() {
        timer.start();
        attemptsSinceLastLog = 0;
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
        set(SD.DistanceToPort, 0);
        allUpdateRobotStates();
        pneumaticsSubsystem.stopCompressor();
        // logger.incrementPrevious("robot.java", "deploy", DefaultValue.Previous);
        // logger.logData();
        addSDToLog(SD.X);
        addSDToLog(SD.Y);
        addSDToLog(SD.Angle);
        addSDToLog(SD.Time);
    }

    public void logDataPeriodic() {
        if(attemptsSinceLastLog == 2) {
            logger.logData();
            attemptsSinceLastLog = 0;
        }
        attemptsSinceLastLog++;
    }
 
    public void robotPeriodic() {
        //System.out.println("robo periodic");
        stateHistory.addState(getState().copy());
        allUpdateRobotStates();
        allModels();
        Scheduler.getInstance().run();
        DelayedPrinter.incTicks();
    }


    public void autonomousInit() {
        Robot.driveSubsystem.assistedDriveMode();
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
        intakeSubsystem.reverseIntake();
        //new ShootCommand().start();
        shooterSubsystem.setShooterSpark(shooterSubsystem.RPMToOmega(3000));
    }

    @Override
    public void autonomousPeriodic() {
        System.out.println("OMEGA: " + shooterSubsystem.shooterSparkEncoder.getVelocity());
        //shooterSubsystem.testShooterSpark2(0.3);
        // shooterSubsystem.setHoodSpark(Math.toRadians(26));
        // System.out.println("HOOD ANGLE " + shooterSubsystem.absEncoder.getRadians());
        
        //sequential.update();
        //allPeriodicLogs();
        //logDataPeriodic();
    }
    
    @Override
    public void teleopInit() {
        /*intakeSubsystem.reverseIntake();
        pneumaticsSubsystem.startCompressor();
        Robot.driveSubsystem.l.ignoreSnap();
        Robot.driveSubsystem.r.ignoreSnap();*/
    }  

    public void teleopPeriodic() {
        //(new TankDriveCommand()).start();   
        allPeriodicLogs();
        logDataPeriodic();
    }

    @Override
    public void testInit() {
        shooterSubsystem.absEncoder.setAngle(Math.toRadians(57));
    }

    public void testPeriodic() {
        System.out.println("HOOD ANGLE " + shooterSubsystem.absEncoder.getRadians());
        System.out.println("DIST " + Robot.get(SD.DistanceToPort));
        shooterSubsystem.testHoodSpark(0);
    }

    public void disabledInit() {
        allPeriodicLogs();
        logger.logData();
        logger.writeLoggedData();
    }
}
