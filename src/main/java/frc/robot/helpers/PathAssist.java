package frc.robot.helpers;

import java.util.ArrayList;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.helpers.PID;
import frc.robot.helpers.Point;

public class PathAssist {

    private static final double TURN_P = 0, TURN_I = 0, TURN_D = 0;
    private PID turnPID = new PID(TURN_P, TURN_I, TURN_D);

    private static final double SMOOTHER_A = .225; // points up to 2-2.5 meters away produce weights btwn .051 and .024
    private int lastPointHitIndex = 0;
    private double distanceTolerance = 0;
    private ArrayList<Double> angles, distances, weights;
    private ArrayList<Point> path;

    public PathAssist() {}

    public void setPath(ArrayList<Point> path) {
        reset();
        clear();
        Utils.deepCopy(path, this.path);
    }

    public void setPath(ArrayList<Point> path, int numPoints) {
        reset();
        clear();
        Utils.deepCopy(path, this.path);
    }

    public void reset() { this.lastPointHitIndex = 0; }
    public void clear() {
        this.angles.clear();
        this.distances.clear();
        this.weights.clear();
    }

    public void update(Point currPos, double heading) {
        clear();
        int lastPointHit = getLastPointHitIndex(currPos);
        getIntermediate(currPos, heading);
        double desiredHeading = getWeight(angles, weights);
        double error = desiredHeading - heading;
        turnPID.addMeasurement(error);
        double turnMagnitude = turnPID.getOutput();
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
    }

    private double weight(double distance) { return Math.pow(SMOOTHER_A, distance); }

    private double getDistCurrPoint(Point currPos) { return CoordinateSystemProcessing.getDistance(currPos, this.path.get(lastPointHitIndex + 1)); }

    private ArrayList<Double> getDistances(Point currPos) {
        ArrayList<Double> distList = new ArrayList<Double>();
        double distance = getDistCurrPoint(currPos);
        distList.add(distance);
        for (int i = lastPointHitIndex + 1; i < this.path.size() - 1; i++) {
            distance += CoordinateSystemProcessing.getDistance(this.path.get(i), this.path.get(i + 1));
            distList.add(distance);
        }
        return distList;
    }

    private ArrayList<Double> getWeights(Point currPos, ArrayList<Double> distances) {
        ArrayList<Double> weights = new ArrayList<Double>();
        double weight;
        for (int i = lastPointHitIndex + 1; i < this.path.size() - 1; i++) {
            weight = weight(distances.get(i));
            weights.add(weight);
        }
        return weights;
    }

    private ArrayList<Double> getAngles(Point currPos) {
        ArrayList<Double> angles = new ArrayList<Double>();
        double angle;
        for (int i = lastPointHitIndex + 1; i < this.path.size() - 1; i++) {
            angle = CoordinateSystemProcessing.angleBetween(currPos, this.path.get(i));
            angles.add(angle);
        }
        return angles;
    }

    private double getWeight(ArrayList<Double> angles, ArrayList<Double> weights) {
        double weight = 0;
        for(int i = lastPointHitIndex; i < this.path.size() - 1; i++) {
            weight += weights.get(i) * angles.get(i);
        }
        return weight;
    }

    private void getIntermediate(Point currPos, double heading) {
        Utils.deepCopy(getDistances(currPos), distances);        // find path distance to each point based on current position
        Utils.deepCopy(getWeights(currPos, distances), weights); // create weights based on those distances
        Utils.deepCopy(getAngles(currPos), angles);              // get angle to each point based on current position
    }

    private int getLastPointHitIndex(Point currPos) {
        int pointToHit = lastPointHitIndex + 1;
        double distance = CoordinateSystemProcessing.getDistance(this.path.get(pointToHit), currPos);
        if (distance < distanceTolerance) {
            lastPointHitIndex = pointToHit;
        }
        return lastPointHitIndex;
    }

    public boolean isDone() {
        Point currPos = new Point(Robot.encoderLocalization.getX(), Robot.encoderLocalization.getY());
        return getLastPointHitIndex(currPos) == (this.path.size() - 1);
    }
}
