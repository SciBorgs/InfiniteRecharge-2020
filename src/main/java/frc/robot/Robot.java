package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.stateEstimation.*;
import frc.robot.logging.*;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.RobotState.RS;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.ArrayList;

public class Robot extends TimedRobot {
    public static Logger logger = new Logger();
    
    public static DriveSubsystem      driveSubsystem      = new DriveSubsystem();
    public static GearShiftSubsystem  gearShiftSubsystem  = new GearShiftSubsystem();
    public static EncoderSubsystem    encoderSubsystem    = new EncoderSubsystem();
    public static LimelightSubsystem  limelightSubsystem  = new LimelightSubsystem();
    public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    
    public static Following following     = new Following();
    public static Model     positionModel = new EncoderLocalization();
    public static OI oi = new OI();

    public static RobotStates robotStates = new RobotStates();

    public static RobotState getState(){return robotStates.currentState();}

    public static double get(RS rs)            {return getState().get(rs);}
    public static void   set(RS rs, double val){       getState().set(rs, val);}
    public static Value getSolenoidValue(RS rs){return getState().getSolenoidValue(rs);}

    private int attemptsSinceLastLog;    
    public static final int LOG_PERIOD = 5;

    private void allPeriodicLogs() {
        driveSubsystem.periodicLog();
        gearShiftSubsystem.periodicLog();
        limelightSubsystem.periodicLog();
        pneumaticsSubsystem.periodicLog();
        encoderSubsystem.periodicLog();
        following.periodicLog();
    }
    private void allUpdateRobotStates() {
        driveSubsystem.updateRobotState();
        gearShiftSubsystem.updateRobotState();
        pneumaticsSubsystem.updateRobotState();
    }

    public void robotInit() {
        attemptsSinceLastLog = 0;
        positionModel.updateRobotState();
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
        robotStates.addState(getState().copy());
        positionModel.updateRobotState();
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