package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.helpers.*;
import frc.robot.localization.*;
import frc.robot.logging.*;
import frc.robot.logging.Logger.DefaultValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
    public static Logger logger = new Logger();
    public static OI oi = new OI();
    
    public static DriveSubsystem      driveSubsystem      = new DriveSubsystem();
    public static EncoderSubsystem    encoderSubsystem    = new EncoderSubsystem();
    public static EncoderLocalization encoderLocalization = new EncoderLocalization();
    public static GearShiftSubsystem  gearShiftSubsystem  = new GearShiftSubsystem();
    public static LimelightSubsystem  limelightSubsystem  = new LimelightSubsystem();
    public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    
    public static Following following = new Following();

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

    public void robotInit() {
        attemptsSinceLastLog = 0;
        encoderLocalization.updatePositionTank();
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
        Scheduler.getInstance().run();
        encoderLocalization.updatePositionTank();
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
        /* 
        allPeriodicLogs();
        logger.logData();
        logger.writeLoggedData(); // Uncomment to write the data to the file
        */
    }
}