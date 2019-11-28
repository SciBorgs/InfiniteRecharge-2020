package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.stateEstimation.*;
import frc.robot.logging.*;
import frc.robot.shapes.*;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.RobotState.SD;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
    public static Logger logger = new Logger();
    
    public static DriveSubsystem      driveSubsystem      = new DriveSubsystem();
    public static GearShiftSubsystem  gearShiftSubsystem  = new GearShiftSubsystem();
    public static LimelightSubsystem  limelightSubsystem  = new LimelightSubsystem();
    public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    
    public static Following following     = new Following();
    public static Model     positionModel = new EncoderLocalization();
    public static OI oi = new OI();

    public static RobotStateHistory stateHistory = new RobotStateHistory();

    public static RobotState getState(){return stateHistory.currentState();}

    public static double get(SD sd)            {return getState().get(sd);}
    public static void   set(SD sd, double val){       getState().set(sd, val);}
    public static Value getSolenoidValue(SD sd){return getState().getSolenoidValue(sd);}

    public static Point getPos() {return new Point(get(SD.X),get(SD.Y));}

    private int attemptsSinceLastLog;    
    public static final int LOG_PERIOD = 5;

    private void allPeriodicLogs() {
        driveSubsystem.periodicLog();
        gearShiftSubsystem.periodicLog();
        limelightSubsystem.periodicLog();
        pneumaticsSubsystem.periodicLog();
        following.periodicLog();
    }
    private void allUpdateRobotStates() {
        driveSubsystem.updateRobotState();
        gearShiftSubsystem.updateRobotState();
        pneumaticsSubsystem.updateRobotState();
    }

    public void useModel(Model model){
        stateHistory.currentState().incorporateIntoNew(model.updatedRobotState(), model.getSDs());
    }

    public void robotInit() {
        attemptsSinceLastLog = 0;
        useModel(positionModel);
        pneumaticsSubsystem.stopCompressor();
        logger.incrementPrevious("robot.java", "deploy", DefaultValue.Previous);

        /* STARTS THE LIDAR     
        try {
            System.out.println("LIDAR status: starting");
            boolean started = LidarServer.getInstance().start();
            System.out.println("LIDAR status" + (started ? "started" : "failed to start"));
        } catch (Throwable t) {
            System.out.println("LIDAR status: crashed -" + t);
            t.printStackTrace();
            throw t;
        }*/

        logger.logData();
    }

    public void logDataPeriodic() {
        if (LOG_PERIOD == attemptsSinceLastLog) {
            attemptsSinceLastLog = 0;
            allPeriodicLogs();
            logger.logData();
        } else {
            attemptsSinceLastLog++;
        }
    }
 
    public void robotPeriodic() {
        allUpdateRobotStates();
        Scheduler.getInstance().run();
        stateHistory.addState(getState().copy());
        useModel(positionModel);
    }
        
    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
        new SwerveTankDriveCommand().start();
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
        allPeriodicLogs();
        logger.logData();
        logger.writeLoggedData();
    }
}