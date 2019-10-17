package frc.robot;

import java.util.ArrayList;
import java.util.ListIterator;

import frc.robot.RobotState;
import frc.robot.helpers.QueuedArrayList;

public class RobotStateHistory{

    // allows us to more clearly pass through a history of states
    // and really allows it to be distinguished from just a list of RobotState objects

    private QueuedArrayList<RobotState> robotStates;
    public int DEFAULT_MAX_SIZE = 30;

    public RobotStateHistory() {
        this.robotStates = new QueuedArrayList<RobotState>(DEFAULT_MAX_SIZE);
        this.robotStates.add(new RobotState());
    }
    public RobotStateHistory(int maxLength){
        this.robotStates = new QueuedArrayList<RobotState>(maxLength);
        this.robotStates.add(new RobotState());
    }
    public RobotStateHistory(QueuedArrayList<RobotState> robotStates){
        this.robotStates = robotStates;
    }
    public RobotStateHistory(Iterable<RobotState> robotStates){
        this.robotStates = QueuedArrayList.fromIterable(robotStates, DEFAULT_MAX_SIZE);
    }
    public RobotStateHistory(Iterable<RobotState> robotStates, int maxLength){
        this.robotStates = QueuedArrayList.fromIterable(robotStates, maxLength);
    }

    public RobotState currentState(){return statesAgo(0);}
    public QueuedArrayList<RobotState> getQueuedArrayList(){return this.robotStates;}
    public ArrayList<RobotState> getArrayList(){return this.robotStates.toArrayList();}
    public RobotState statesAgo(int n)    {return this.robotStates.get(n);}
    public int numberOfStates()           {return this.robotStates.size();}
    public void setCurrentState(RobotState state){this.robotStates.set(0, state);}
    public void addState       (RobotState state){this.robotStates.add(state);}
    public void dropFirstState()                 {this.robotStates.remove(0);}
    
    public RobotStateHistory copy(){
        return new RobotStateHistory((QueuedArrayList<RobotState>) this.robotStates.clone());
    }
}
