package frc.robot.stateEstimation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Hashtable;

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
        ArrayList<RobotStates> statesList = new ArrayList<>();
        for(Particle particle : this.particles){
            statesList.add(particle.states);
        }
        return statesList;
    }
    public ArrayList<Double> getWeights(){ // converts the Particles to weights
        ArrayList<Double> weights = new ArrayList<>();
        for(Particle particle : this.particles){
            weights.add(particle.weight);
        }
        return weights;
    }

    private Particle updateParticle(Particle particle){ 
        // Updates a particle factoring noise
        RobotState updatedState = updater.updateState(particle.states);
        Hashtable<RS, Double> variances = updater.getVariances(); // variations = std_devs
        for (RS rs : variances.keySet()){
            // generate gaussian creates noise
            updatedState.set(rs, Utils.generateGaussian(updatedState.get(rs), variances.get(rs)));
        }
        Particle newParticle = particle.copy();
        // incorporates state from Robot.java except for RS values being updated by updater
        RobotState nextState = Robot.getState().incorporateIntoNew(updatedState, variances.keySet());
        newParticle.states.addState(nextState);
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
        double currentSum = 0;
        for(double weight : getWeights()){currentSum += weight;}
        int size = this.particles.size();
        double scale = weightSum / currentSum;
        for(int i = 0; i < size; i++){
            this.particles.get(i).weight *= scale;
        }
    }

    private void computeWeights() {
        int size = this.particles.size();
        for(int i = 0; i < size; i++){
            this.particles.get(i).weight *= this.weighter.weight(this.particles.get(i).states);
        }
        adjustWeights();
    }

    private void filterParticles() {
        // gets rid of illegal particles
        int size = particles.size();
        for(int i = 0; i < size; i++){
            if(illegalStateDeterminer.isIllegalState(this.particles.get(i).states)){
                this.particles.remove(i);
                i--; size--;
            }
        }
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
