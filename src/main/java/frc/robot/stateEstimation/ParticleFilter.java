package frc.robot.stateEstimation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Hashtable;
import java.util.List;
import java.util.stream.Collectors;

import frc.robot.stateEstimation.Updater;
import frc.robot.stateEstimation.Weighter;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.RS;
import frc.robot.RobotStates;
import frc.robot.Utils;

class Particle {
    public RobotStates states;
    public double weight;

    public Particle(RobotStates states, double weight){
        this.states = states;
        this.weight = weight;
    }

    public Particle copy(){
        return new Particle(states.copy(), weight);
    }

    public Particle addStateIntoNew(RobotState state){
        Particle newParticle = this.copy();
        newParticle.states.addState(state);
        return newParticle;
    }
}

public class ParticleFilter {

    private int numOfParticles = 1000;
    private double weightSum = this.numOfParticles;
    // Need to combine particles and weights fields using Pair by merging master into this branch
    private ArrayList<Particle> particles;
    public Updater  updater;
    public Weighter weighter;
    public IllegalStateDeterminer illegalStateDeterminer;

    public ParticleFilter(Updater updater, Weighter weighter, RobotStates ogState){
        this.updater  = updater;
        this.weighter = weighter;
        this.illegalStateDeterminer = new NeverIllegal(); // defaults to having no illegal states

        this.particles = new ArrayList<>();
        this.particles.add(new Particle(ogState, weightSum));
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

    public ArrayList<RobotStates> getStatesList(){ // converts the Particles to a list of RobotStates
        return Utils.toArrayList(this.particles.stream().map(particle -> particle.states));
    }
    public ArrayList<Double> getWeights(){ // converts the Particles to weights
        return Utils.toArrayList(this.particles.stream().map(particle -> particle.weight));
    }

    private Particle updateParticle(Particle particle){ 
        // Updates a particle factoring in noise
        RobotState updatedState = updater.updateState(particle.states);
        Hashtable<RS, Double> stdDevs = updater.getStdDevs();
        Utils.addNoise(updatedState, stdDevs);
        // incorporates state from Robot.java except for RS values being updated by updater
        RobotState nextState = Robot.getState().incorporateIntoNew(updatedState, stdDevs.keySet());
        return particle.addStateIntoNew(nextState);
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
            particle.weight *= this.weighter.weight(particle.states);
        }
        adjustWeights();
    }

    private void filterParticles() {
        this.particles.removeIf(particle -> illegalStateDeterminer.isIllegalState(particle.states));
    }

    private void sortParticle(){
        // Descending comparator
        Comparator<Particle> particleComparator = Comparator.comparingDouble(particle -> -particle.weight);
        particles.sort(particleComparator);
    }

    public RobotStates currentStates(){
        return particles.get(0).states;
    }

    public void nextGeneration() {
        // call to update everything
        updateParticles();
        filterParticles();
        computeWeights(); // this must come after filter particles so that the weight sum is correct
        sortParticle();
    }
}
