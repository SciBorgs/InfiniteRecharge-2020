package frc.robot.controllers;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState.SD;
import java.io.File;
import java.io.IOException;

public class PowerCellTrajectoryController {
  private final String TRAJECTORY_DATA_PATH =
      "src\\main\\java\\frc\\robot\\controllers\\PowerCellTrajectoryData.json";
  private final double LIMELIGHT_OFFSET = -Utils.inchesToMeters(8.0);

  private JsonNode rootNode;

  private final double HOOD_ANGLE_OFFSET = 3;

  public PowerCellTrajectoryController() {/*
    try {
      this.rootNode = new ObjectMapper().readTree(new File(TRAJECTORY_DATA_PATH));
    } catch(IOException e) {
      throw new RuntimeException("FAILED TO INITIALIZE POWERCELLTRAJECTORYCONTROLLER");
    }*/
  }

  public Pair<Double, Double> getOptimalParameters() {
    double distanceToOuterPort = Robot.get(SD.DistanceToPort) - LIMELIGHT_OFFSET;
    JsonNode distanceField = this.rootNode.get(String.format("%.2f", distanceToOuterPort));
    return new Pair<Double, Double>(
        distanceField.get("hood_angle").asDouble() + HOOD_ANGLE_OFFSET, distanceField.get("omega").asDouble());
  }
}
