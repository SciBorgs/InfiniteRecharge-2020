package frc.robot.stateEstimation.higherLevel;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Hashtable;
import java.util.Set;

import frc.robot.stateEstimation.interfaces.*;
import frc.robot.Robot;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotState.SD;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.Utils;

// What's a particle filter?
// http://www.deepideas.net/robot-localization-particle-filter/

class Particle {
    public RobotStateHistory stateHistory;
    public double weight;

    public Particle(RobotStateHistory stateHistory, double weight){
        this.stateHistory = stateHistory;
        this.weight       = weight;
    }

    public Particle copy(){
        return new Particle(stateHistory.copy(), weight);
    }

    public Particle addStateIntoNew(RobotState state){
        Particle newParticle = this.copy();
        newParticle.stateHistory.addState(state);
        return newParticle;
    }

    public RobotState currentState(){
        return this.stateHistory.currentState();
    }
}

public class ParticleFilter implements Model{

    private int numOfParticles = 1000;
    private double weightSum = this.numOfParticles;
    // Need to combine particles and weights fields using Pair by merging master into this branch
    private ArrayList<Particle> particles;
    public Updater  updater;
    public Weighter weighter;
    public IllegalStateDeterminer illegalStateDeterminer;

    public ParticleFilter(Updater updater, Weighter weighter, RobotStateHistory startingStateHistory){
        this.updater  = updater;
        this.weighter = weighter;
        this.illegalStateDeterminer = new NeverIllegal(); // defaults to having no illegal states

        this.particles = new ArrayList<>();
        this.particles.add(new Particle(startingStateHistory, weightSum));
    }

    public ParticleFilter(Updater updater, Weighter weighter, IllegalStateDeterminer illegalStateDeterminer) {
        this.updater = updater;
        this.weighter = weighter;
        this.illegalStateDeterminer = illegalStateDeterminer;
    }

    public ParticleFilter(Updater updater, Weighter weighter, IllegalStateDeterminer illegalStateDeterminer, ArrayList<Particle> particles) {
        this.updater   = updater;
        this.weighter  = weighter;
        this.particles = particles;
        this.illegalStateDeterminer = illegalStateDeterminer;
    }

    public void nextGeneration() {
        // call to update everything
        updateParticles();
        filterParticles();
        computeWeights(); // this must come after filter particles so that the weight sum is correct
        sortParticle();
    }

    public ArrayList<RobotStateHistory> getStatesList(){ // converts the Particles to a list of RobotStates
        return Utils.toArrayList(this.particles.stream().map(particle -> particle.stateHistory));
    }
    public ArrayList<Double> getWeights(){ // converts the Particles to weights
        return Utils.toArrayList(this.particles.stream().map(particle -> particle.weight));
    }

    private Particle updateParticle(Particle particle) {
        // Starts by creating current state, with every SD from the global robot state
        // except for the ones
        // the particle filter is using
        RobotState currentState = Robot.getState().incorporateIntoNew(particle.currentState(), getSDs());
        Particle newParticle = particle.addStateIntoNew(currentState);
        updater.updateState(newParticle.stateHistory);
        Hashtable<SD, Double> stdDevs = updater.getStdDevs();
        Utils.addNoise(newParticle.currentState(), stdDevs);
        // incorporates state from Robot.java except for SD values being updated by
        // updater
        return newParticle;
    }

    private void updateParticles(){
        // Choses which particles to update and then updates them
        // Chance a new particle comes from a given particle = weight-of-particle/sum-of-weights
        ArrayList<Particle> newParticles = new ArrayList<>();
        // Next particleOriginValues has random numbers between 0 and weightSum
        // Each number will generate a particle updated from the cummWeights before it
        ArrayList<Double> particleOriginValues = Utils.randomArrayList(this.numOfParticles, 0, this.weightSum);
        ArrayList<Double> cummWeights = Utils.cummSums(getWeights()); 
        particleOriginValues.sort(Utils.ascendingDoubleComparator);
        // I'm sorry about this. I don't know how to clean this up so if you have a good idea, please do so!
        while (!particleOriginValues.isEmpty()) {
            if (particleOriginValues.get(0) > cummWeights.get(0)){
                cummWeights.remove(0);
                this.particles.remove(0);
            } else {
                newParticles.add(updateParticle(this.particles.get(0)));
            }
        }
        this.particles = newParticles;
    }

    private void adjustWeights(){
        // Makes the sum of all the weights = this.weightSum
        double scale = this.weightSum / Utils.sumArrayList(getWeights());
        for(Particle particle : this.particles){
            particle.weight *= scale;
        }
    }

    private void computeWeights() {
        for(Particle particle : this.particles){
            particle.weight *= this.weighter.weight(particle.stateHistory);
        }
        adjustWeights();
    }

    private void filterParticles() {
        this.particles.removeIf(particle -> illegalStateDeterminer.isStateIllegal(particle.stateHistory));
    }

    private void sortParticle(){
        // Descending comparator
        Comparator<Particle> particleComparator = Comparator.comparingDouble(particle -> -particle.weight);
        particles.sort(particleComparator);
    }

    public RobotStateHistory currentStates(){
        return particles.get(0).stateHistory;
    }
    
    // Model functionality:
    @Override
    public Set<SD> getSDs(){
        return this.updater.getStdDevs().keySet();
    }
    @Override
    public void updateRobotState(){
        nextGeneration();
        Robot.getState().incorporateOtherState(currentStates().currentState(), getSDs());
    }
}
