package frc.robot;

import java.util.ArrayList;

import frc.robot.RobotState;

public class RobotStateHistory{

    // allows us to more clearly pass through a history of states
    // and really allows it to be distinguished from just a list of RobotState objects

    private ArrayList<RobotState> robotStates;

    public RobotStateHistory(){
        this.robotStates = new ArrayList<>();
        this.robotStates.add(new RobotState());
    }
    public RobotStateHistory(ArrayList<RobotState> robotStates){
        this.robotStates = robotStates;
    }

    public ArrayList<RobotState> getArrayList(){return this.robotStates;}
    public RobotState statesAgo(int n){return this.robotStates.get(n);}
    public RobotState currentState(){return statesAgo(0);}
    public void setCurrentState(RobotState state){this.robotStates.set(0, state);}
    public void addState       (RobotState state){this.robotStates.add(0, state);}
    public void dropFirstState()                 {this.robotStates.remove(0);}

    public RobotStateHistory copy(){
        return new RobotStateHistory((ArrayList<RobotState>) this.robotStates.clone());
    }
}