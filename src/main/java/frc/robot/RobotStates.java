package frc.robot;

import java.util.ArrayList;

import frc.robot.RobotState;

public class RobotStates{

    // allows us to more clearly pass through a history of states
    // and really allows it to be distinguished from just a list of RobotState objects

    private ArrayList<RobotState> robotStates;

    public RobotStates(){
        this.robotStates = new ArrayList<>();
    }
    public RobotStates(ArrayList<RobotState> robotStates){
        this.robotStates = robotStates;
    }

    public ArrayList<RobotState> getArrayList(){return this.robotStates;}
    public RobotState statesAgo(int n){return robotStates.get(n);}
    public RobotState currentState(){return statesAgo(0);}
    public void setCurrentState(RobotState state){robotStates.set(0, state);}
    public void addState       (RobotState state){robotStates.add(0, state);}

    public void dropFirstState(){this.robotStates.remove(0);}

    public RobotStates copy(){
        return new RobotStates((ArrayList<RobotState>) this.robotStates.clone());
    }
}