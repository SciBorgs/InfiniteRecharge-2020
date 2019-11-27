package frc.robot.logging;

import java.io.IOException;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.helpers.Pigeon;

public class OutputLogger {    
    private Pigeon pigeon;
    private BuiltInAccelerometer RIO;
    private Writer writer;

    private static final double GRAVITATIONAL_ACCELERATION = 9.81;
    private static final double Q_MULTIPLIER = Math.pow(2, -14);
    private static final double WHEEL_RADIUS_METERS = Utils.inchesToMeters(3);
    private static final double CHASSY_WIDTH = Utils.inchesToMeters(25.5);

    public OutputLogger(String fileName) throws IOException {
        pigeon = Robot.positionModel.pigeon;
        RIO = new BuiltInAccelerometer();
        writer = Files.newBufferedWriter(Paths.get(fileName), StandardCharsets.UTF_8, 
                                                              StandardOpenOption.CREATE, 
                                                              StandardOpenOption.APPEND);
    }

    public void logToFile(double time) throws IOException {
        String line = generateCSLine(Double.toString(time),                 Double.toString(getRIOAccel()),
                                     Double.toString(getIMUAccel()),        Double.toString(getIMUFusedHeading()),
                                     Double.toString(pigeon.getAngle()),    Double.toString(getIMUYawRate()), 
                                     Double.toString(getEncoderPositionL()), Double.toString(getEncoderPositionR()));

        writer.write(line + "\n");
    }

    public void destroyWriter() throws IOException {
        writer.close();
    }

    private String generateCSLine(String... inputs) {
        String ret = "";
        for (int i = 0; i < inputs.length; ++i) {
            if (i != inputs.length - 1) ret += inputs[i] + ", ";
            else ret += inputs[i];
        }
        return ret;
    }

    private double RPMtoMPS(double radius, double RPM) {
        return radius * 2 * Math.PI * (RPM / 60);
    }

    private double getEncoderPositionL() {
        double lEncoder = Robot.driveSubsystem.l.getEncoder()
                                     .getPosition() * Robot.gearShiftSubsystem.getCurrentGearRatio() * 2 * Math.PI * WHEEL_RADIUS_METERS;
        return lEncoder;
    }

    private double getEncoderPositionR() {
        double rEncoder =  Robot.driveSubsystem.r.getEncoder()
                                     .getPosition() * Robot.gearShiftSubsystem.getCurrentGearRatio() * 2 * Math.PI * WHEEL_RADIUS_METERS;
        return rEncoder;
    }

    private double getIMUYawRate() {
        double[] xyz = new double[3];
        pigeon.getPigeonIMU().getRawGyro(xyz);
        return Math.toRadians(xyz[2]);
    }

    private double getIMUAccel() {
        short[] xyz = new short[3];
        pigeon.getPigeonIMU().getBiasedAccelerometer(xyz);
        return xyz[1] * Q_MULTIPLIER * GRAVITATIONAL_ACCELERATION;
    }

    private double getIMUFusedHeading() {
        return pigeon.getPigeonIMU().getFusedHeading();
    }
    
    private double getRIOAccel() {
        return RIO.getY() * GRAVITATIONAL_ACCELERATION;
    }
}