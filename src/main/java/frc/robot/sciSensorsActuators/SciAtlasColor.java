package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.SerialPort;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashMap;
import java.lang.Double;

import frc.robot.Utils;
import frc.robot.colors.*;
import frc.robot.helpers.Colors;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciAtlasColor.SciAtlasColorSD;

public class SciAtlasColor implements RobotStateUpdater, SciSensorActuator<SciAtlasColorSD> {
    public static enum SciAtlasColorSD {H, S, V}
    public HashMap<SciAtlasColorSD, SD> sdMap;
    public static enum IRLEDBrightness {
        Low("L"), Medium("M"), High("H");

        public final String value;

        private IRLEDBrightness(String value) {
            this.value = value;
        }
    }

    public static enum OutputParameters {
        RGB, PROX, LUX, CIE;

        public final String value;

        private OutputParameters() {
            this.value = name();
        }
    }

    private int baudRate = 9600;
    private SerialPort.Port port;
    private SerialPort serial;

    public SciAtlasColor(SerialPort.Port port) {
        this.port = port;
        this.serial = new SerialPort(this.baudRate, this.port);
    }

    public SciAtlasColor(SerialPort.Port port, int baudRate) {
        this.baudRate = baudRate;
        this.port = port;
        this.serial = new SerialPort(this.baudRate, this.port);
    }

    @Override
    public HashMap<SciAtlasColorSD, SD> getSDMap() { return this.sdMap; }
    @Override
    public String getDeviceName() { return "Atlas Color Sensor " + port.name(); }

    private void send(String command) {
        byte[] commandBytes = command.getBytes(StandardCharsets.US_ASCII);
        byte[] bytes = new byte[commandBytes.length + 1];
        for (int i = 0; i < commandBytes.length; i++) {
            bytes[i] = commandBytes[i];
        }
        bytes[bytes.length - 1] = (byte) 13;
    }

    // TODO: Proper handling when continuous mode is on
    private String read() {
        String output = serial.readString();
        while (output.strip().equals("")) {
            output = serial.readString();
        }
        return output;
    }

    private String sendAndRead(String command) {
        send(command);
        return read();
    }

    // TODO: Better naming
    private boolean sendAndCheckError(String command) {
        send(command);
        return !read().equals("*ER");
    }

    private boolean sendAndCheckError(String command, boolean enabled) {
        return sendAndCheckError(command + "," + (enabled ? 1 : 0));
    }

    public String getTargetLEDBrightness() {
        return sendAndRead("L,?");
    }

    public boolean setTargetLEDBrightness(int brightness) {
        return setTargetLEDBrightness(brightness, false);
    }

    public boolean setTargetLEDBrightness(int brightness, boolean powerSaving) {
        return sendAndCheckError("L," + brightness + (powerSaving ? ",T" : ""));
    }

    public String getIndicatorLEDState() {
        return sendAndRead("iL,?");
    } 

    public boolean setIndicatorLEDState(boolean enabled) {
        return sendAndCheckError("iL", enabled);
    }

    public boolean find() {
        return sendAndCheckError("Find");
    }

    public String getContinuous() {
        return sendAndRead("C,?");
    }

    public boolean setContinuous(int rate) {
        return sendAndCheckError("C," + rate);
    }

    public String getRaw() {
        return sendAndRead("R");
    }

    public boolean calibrate() {
        return sendAndCheckError("Cal");
    }

    public String getProximityDetection() {
        return sendAndRead("P,?");
    }

    public boolean setProximityDetection(IRLEDBrightness brightness) {
        return sendAndCheckError("P," + brightness.value);
    }

    public boolean setProximityDetection(boolean enabled) {
        return sendAndCheckError("P", enabled);
    }

    public boolean setProximityDetection(int distance) {
        return sendAndCheckError("P," + distance);
    }

    public String getColorMatching() {
        return sendAndRead("M,?");
    }

    public boolean setColorMatching(boolean enabled) {
        return sendAndCheckError("M", enabled);
    }

    public String getGammaCorrection() {
        return sendAndRead("G,?");
    }

    public boolean setGammaCorrection(double value) {
        return sendAndCheckError("G," + value);
    }

    public String getOutputParameters() {
        return sendAndRead("O,?");
    }

    public boolean setOutputParameters(OutputParameters parameter, boolean enabled) {
        return sendAndCheckError("O," + parameter.value, enabled);
    }

    public String getName() {
        return sendAndRead("Name,?");
    }

    public boolean setName(String name) {
        return sendAndCheckError("Name," + name);
    }

    public String getInfo() {
        return sendAndRead("i");
    }

    // TODO: Response code functions, currently this will break the code so not implemented

    public String getStatus() {
        return sendAndRead("Status");
    }

    public boolean sleep() {
        return sendAndCheckError("Sleep");
    }

    public String getBaudRate() {
        return sendAndRead("Baud,?");
    }

    public boolean setBaudRate(int baudRate) {
        boolean result = sendAndCheckError("Baud," + baudRate);
        this.baudRate = baudRate;
        this.serial = new SerialPort(this.baudRate, this.port);
        return result;
    }

    public String getProtocolLock() {
        return sendAndRead("Plock,?");
    }

    public boolean setProtocolLock(boolean enabled) {
        return sendAndCheckError("Plock", enabled);
    }

    public boolean factoryReset() {
        return sendAndCheckError("Factory");
    }

    /**Note that this wrapper does not support I2C. */
    public boolean switchToI2C(int address) {
        return sendAndCheckError("I2C," + address);
    }

    // TODO: Properly handle output parameters & continous mode
    public RGB getRGB() {
        send("R");
        double[] parts = Arrays.stream(read().split(",")).limit(3).mapToDouble(Double::parseDouble).toArray();
        double max = Utils.max(parts);
        if (max > 255) {
            for (int i = 0; i < 3; i++) {
                parts[i] /= (max/255);
            }
        }
        return new RGB(parts[0], parts[1], parts[2]);
    }

    public HSV getHSV() {
        return Colors.toHSV(getRGB());
    }

    @Override
    public void updateRobotState() {
        HSV color = getHSV();
        sciSet(SciAtlasColorSD.H, color.getH());
        sciSet(SciAtlasColorSD.S, color.getS());
        sciSet(SciAtlasColorSD.V, color.getV());
    }
}