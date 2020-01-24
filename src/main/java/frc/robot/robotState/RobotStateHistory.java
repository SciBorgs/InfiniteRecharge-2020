package frc.robot.robotState;

import java.util.Hashtable;
import java.util.List;
import frc.robot.dataTypes.Deque;

public class RobotStateHistory{

    // allows us to more clearly pass through a history of states
    // and really allows it to be distinguished from just a list of RobotState objects

    private Deque<RobotState> robotStates;
    public int DEFAULT_MAX_SIZE = 30;

    public RobotStateHistory() {
        this.robotStates = new Deque<RobotState>(DEFAULT_MAX_SIZE);
        this.robotStates.add(new RobotState());
    }
    public RobotStateHistory(int maxLength){
        this.robotStates = new Deque<RobotState>(maxLength);
        this.robotStates.add(new RobotState());
    }
    public RobotStateHistory(Deque<RobotState> robotStates){
        this.robotStates = robotStates;
    }
    public RobotStateHistory(Iterable<RobotState> robotStates){
        this.robotStates = new Deque<>(robotStates, DEFAULT_MAX_SIZE);
    }
    public RobotStateHistory(Iterable<RobotState> robotStates, int maxLength){
        this.robotStates = new Deque<>(robotStates, maxLength);
    }

    public void clear(){
        int maxSize = this.robotStates.maxLength;
        this.robotStates = new Deque<RobotState>(maxSize);
    }

    public RobotState currentState(){return statesAgo(0);}
    public Deque<RobotState> getQueuedArrayList(){return this.robotStates;}
    public List<RobotState> getArrayList(){return this.robotStates.getArrayList();}
    public RobotState statesAgo(int n)  {
        return n >= this.robotStates.getSize() ? this.robotStates.get(this.robotStates.head) : this.robotStates.get(n);
    }
    public int numberOfStates()           {return this.robotStates.getSize();}
    public void setCurrentState(RobotState state){this.robotStates.set(0, state);}
    public void addState       (RobotState state){this.robotStates.add(state);}
    public void dropFirstState()                 {this.robotStates.remove(0);}
    
    public RobotStateHistory copy(){
        return new RobotStateHistory((Deque<RobotState>) this.robotStates.clone());
    }
}