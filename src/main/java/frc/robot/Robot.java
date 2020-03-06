package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.*;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.autoProfiles.AutoRoutine;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.helpers.*;
import frc.robot.dataTypes.*;
import frc.robot.logging.*;
import frc.robot.shapes.*;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciJoystick;
import frc.robot.sciSensorsActuators.SciSolenoid;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.*;
import frc.robot.stateEstimation.interfaces.*;
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
    public static TurretSubsystem     turretSubsystem     = new TurretSubsystem();
    public static ShooterSubsystem    shooterSubsystem    = new ShooterSubsystem();

    public static IntakeSubsystem     intakeSubsystem     = new IntakeSubsystem();
    public static HopperSubsystem     hopperSubsystem     = new HopperSubsystem();
    
    public static Following following = new Following();
    public static OI oi = new OI();

    // public static Model positionModel = new MaybeDefaultUpdater(new LimelightLocalization(), new EncoderLocalization());
    public static Model positionModel = new EncoderLocalization();
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
    public static Waypoint getWaypoint () { return new Waypoint(new Point(get(SD.X), get(SD.Y)), get(SD.Angle)); }

    public static AutoRoutine autoRoutine = new AutoRoutine();

    public static final Waypoint TEST_POINT_0 = new Waypoint(new Point(Utils.inchesToMeters(128), Utils.inchesToMeters(69)), Geo.HORIZONTAL_ANGLE);
    public static final Waypoint TEST_POINT_1 = new Waypoint(new Point(0, 0), Geo.HORIZONTAL_ANGLE);
    public static final Waypoint TEST_POINT_2 = new Waypoint(new Point(0, 1), Geo.HORIZONTAL_ANGLE);
    public static final Waypoint TEST_POINT_3 = new Waypoint(new Point(0, 2), Geo.HORIZONTAL_ANGLE);
    public static final Waypoint TEST_POINT_4 = new Waypoint(new Point(0, 3), Geo.HORIZONTAL_ANGLE);
    // for tenBallAuto
    // public static final Point ORIGINAL_POINT  = new Point(3 - autoRoutine.xShift, -7.5 - autoRoutine.yShift);
    public static final Point ORIGINAL_POINT  = new Point(0, 0);
    public static final double ORIGINAL_ANGLE = Geo.HORIZONTAL_ANGLE;
    public static Waypoint[] arr = new Waypoint[] {TEST_POINT_0, TEST_POINT_1, TEST_POINT_2, TEST_POINT_3, TEST_POINT_4};
    public static ArrayList <Waypoint> path = new ArrayList<Waypoint>(Arrays.asList(arr));

    public static Waypoint CURRENT_DESTINATION = TEST_POINT_0;

    public static Point newDestPoint = new Point(Utils.inchesToMeters(4), .458);
    public static double newDestHeading = Geo.HORIZONTAL_ANGLE;
    private int attemptsSinceLastLog = 0;
    public static final int LOG_PERIOD = 5;

    NetworkTableEntry getPos;

    public Robot() {
        automateLogging();
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
        if(attemptsSinceLastLog == -1) {
            logger.logData();
            attemptsSinceLastLog = 0;
        }
        attemptsSinceLastLog++;
    }
 
    public void robotPeriodic() {
        stateHistory.addState(getState().copy());
        allUpdateRobotStates();
        allModels();
        Scheduler.getInstance().run();
        DelayedPrinter.print("x: " + getPos().x +"y: "+ getPos().y + "\nheading: "  + getWaypoint().heading, 5);
        DelayedPrinter.incTicks();
        SmartDashboard.putNumber("GetPos.x", getPos().x);
        SmartDashboard.putNumber("GetPos.y", getPos().y);
        SmartDashboard.putNumber("Heading", get(SD.Angle));
        SmartDashboard.putBoolean("reversed?", driveSubsystem.reversed);
        SmartDashboard.putNumber("left wheel speed", get(SD.LeftWheelSpeed));
    }


    public void autonomousInit() { 
        driveSubsystem.setReversed(false);       
        //Robot.driveSubsystem.assistedDriveMode();
        set(SD.X, ORIGINAL_POINT.x);
        set(SD.Y, ORIGINAL_POINT.y);
        set(SD.Angle, ORIGINAL_ANGLE);
        //intakeSubsystem.reverseIntake();
        //shooterSubsystem.setShooterOmega(314);
    }

    @Override
    public void autonomousPeriodic() {
        //shooterSubsystem.setHoodAngle(Math.toRadians(25));
        //System.out.println("HOOD ANGLE: " + Robot.get(SD.HoodAngle));
        //System.out.println("OMEGA: " + Robot.get(SD.ShooterOmega));
        //sequential.update();
        //allPeriodicLogs();
        //logDataPeriodic();
    }
    
    @Override
    public void teleopInit() {
        // intakeSubsystem.reverseIntake();
        Robot.driveSubsystem.l.ignoreSnap();
        Robot.driveSubsystem.r.ignoreSnap();
        //pneumaticsSubsystem.startCompressor();
    }

    public void teleopPeriodic() {
        //intakeSubsystem.setIntakeSpeed(1);
        //System.out.println("current: " + intakeSubsystem.intakeSpark.get());
       
        // (new TankDriveCommand()).start();
        //allPeriodicLogs();
        //logDataPeriodic();
    }

    @Override
    public void testInit() {
        shooterSubsystem.stopMotors();
    }

    public void testPeriodic() {
        System.out.println("HOOD ANGLE " + Robot.get(SD.HoodAngle));
        System.out.println("DIST " + Robot.get(SD.DistanceToPort));
    }

    public void disabledInit() {
        // allPeriodicLogs();
        // logger.logData();
        // logger.writeLoggedData();
    }
}