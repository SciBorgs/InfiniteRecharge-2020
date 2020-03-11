package frc.robot.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Robot;
import frc.robot.dataTypes.BiHashMap;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState.SD;

public class SciffleBoard {
    List<Pair<NetworkTableEntry, SD>> entries = new ArrayList<>();

    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    ShuffleboardTab climbTab = Shuffleboard.getTab("Climb");

    // NetworkTableEntry hoodAngle = shooterTab.add("hoodAngle", 17)
    // .withWidget(BuiltInWidgets.kDial)
    // .withProperties(Map.of("min", 17, "max", 65)).getEntry();

    // NetworkTableEntry shooterSpeed = shooterTab.add("shooterSpeed", 0)
    // .withWidget(BuiltInWidgets.kNumberBar)
    // .withProperties(Map.of("min", 0, "max", 10_000)).getEntry();

    // NetworkTableEntry turretAngle = shooterTab.add("turretAngle", 0)
    // .withWidget(BuiltInWidgets.kDial)
    // .withProperties(Map.of("min", - Math.PI / 2, "max", Math.PI / 2)).getEntry();

    // Regular(String tab, SD sd, double min, double max, BuiltInWidgets type)
    // Mapped(String tab, SD sd, BiMap biMap)

    public void addNetworkTableEntrys() {
        makeRegNTE(driveTab, SD.RightJoystick, -1, 1, BuiltInWidgets.kNumberBar);
        makeRegNTE(driveTab, SD.LeftJoystick,  -1, 1, BuiltInWidgets.kNumberBar);
        makeRegNTE(driveTab, SD.LeftWheelVal,  -1, 1, BuiltInWidgets.kNumberBar);
        makeRegNTE(driveTab, SD.RightWheelVal, -1, 1, BuiltInWidgets.kNumberBar);
        makeFunctionNTE(shooterTab, SD.HoodAngle,   17, 65, Math::toDegrees, BuiltInWidgets.kDial);
        makeRegNTE(shooterTab, SD.TurretAngle, - Math.PI / 2, Math.PI / 2, BuiltInWidgets.kNumberBar);
        makeRegNTE(shooterTab, SD.ShooterOmega, 0, 10000, BuiltInWidgets.kNumberBar);
    }

    public static final double DEFAULT_NUMBER = 1155.1155;
    public static final String DEFAULT_STRING = "-";

    private double defaultGet(SD sd) {
        return Robot.getState().contains(sd) ? Robot.get(sd) : DEFAULT_NUMBER;
    }
    private<T> String defaultGetMapped(SD sd, BiHashMap<T, Double> biMap){
        return Robot.getState().contains(sd) ? "" + Robot.getState().getMapped(biMap, sd) : DEFAULT_STRING;
    }
    private double defaultFunctionGet(SD sd, Function<Double, Double> f){
        return Robot.getState().contains(sd) ? f.apply(Robot.getState().get(sd)) : DEFAULT_NUMBER;
    }
    private<T> String defaultObjectFunctionGet(SD sd, Function<Double, T> f){
        return Robot.getState().contains(sd) ? "" + f.apply(Robot.getState().get(sd)) : DEFAULT_STRING;
    }

    private SimpleWidget addToTab(ShuffleboardTab tab, SD sd, Object value){
        return Shuffleboard.getTab(tab.getTitle()).add(sd.name(), value);
    }

    public void makeRegNTE(ShuffleboardTab tab, SD sd, double min, double max, BuiltInWidgets type) {
        entries.add(new Pair<>(addToTab(tab, sd, defaultGet(sd)).withWidget(type)
               .withProperties(Map.of("min", min, "max", max)).getEntry(), sd));
    }

    public void makeFunctionNTE(ShuffleboardTab tab, SD sd, double min, double max, Function<Double, Double> f, BuiltInWidgets type){
        entries.add(new Pair<>(addToTab(tab, sd, defaultFunctionGet(sd, f)).withWidget(type)
                .withProperties(Map.of("min", min, "max", max)).getEntry(), sd));
    }

    public void makeObjectFunctionNTE(ShuffleboardTab tab, SD sd, Function<Double, Object> f){
        entries.add(new Pair<>(addToTab(tab, sd, defaultObjectFunctionGet(sd, f))
                .withWidget(BuiltInWidgets.kTextView).getEntry(), sd));
    }

    public<T> void makeMappedNTE(ShuffleboardTab tab, SD sd, BiHashMap<T, Double> biMap) {
        entries.add(new Pair<>(Shuffleboard.getTab(tab.getTitle()).add(sd.name(), defaultGetMapped(sd, biMap))
                                           .withWidget(BuiltInWidgets.kTextView).getEntry(), sd));
    }

    public void update() {
        for (Pair<NetworkTableEntry, SD> p: this.entries) {
            p.first.setDouble(Robot.get(p.second));
        }
    }
}