package frc.robot.stateEstimation;

import java.util.ArrayList;
import java.util.Random;
import java.util.Collections;
import java.util.Hashtable;

import frc.robot.stateEstimation.Updater;
import frc.robot.stateEstimation.Weighter;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.RS;
import frc.robot.RobotStates;
import frc.robot.Utils;

public class ParticleFilter {

    private int numOfParticles = 1000;
    private int numOfIterations = 100;
    private double weightSum = this.numOfParticles;
    // Need to combine particles and weights fields using Pair by merging master into this branch
    private ArrayList<RobotStates> particles;
    private ArrayList<Double> weights;
    private Random generator = new Random();
    public Updater  updater;
    public Weighter weighter;
    public IllegalStateDeterminer illegalStateDeterminer;

    public ParticleFilter(Updater updater, Weighter weighter, RobotStates ogState){
        this.updater  = updater;
        this.weighter = weighter;
        this.illegalStateDeterminer = new NeverIllegal();

        this.particles = new ArrayList<>();
        this.particles.add(ogState);
    }

    public ParticleFilter(Updater updater, Weighter weighter, IllegalStateDeterminer illegalStateDeterminer) {
        this.updater = updater;
        this.weighter = weighter;
        this.illegalStateDeterminer = illegalStateDeterminer;
    }

    public ParticleFilter(Updater updater, Weighter weighter, IllegalStateDeterminer illegalStateDeterminer, ArrayList<RobotStates> particles) {
        this.updater   = updater;
        this.weighter  = weighter;
        this.particles = particles;
        this.illegalStateDeterminer = illegalStateDeterminer;
    }

    private RobotStates updateParticle(RobotStates particle){
        RobotState updatedState = updater.updateState(particle);
        Hashtable<RS, Double> variances = updater.getVariances();
        for (RS rs : variances.keySet()){
            updatedState.set(rs, Utils.generateGaussian(updatedState.get(rs), variances.get(rs)));
        }
        RobotStates newParticle = particle.copy();
        RobotState nextState = Robot.getState().incorporateIntoNew(updatedState, variances.keySet());
        newParticle.addState(nextState);
        return newParticle;
    }

    private void updateParticles(){
        ArrayList<RobotStates> newParticles = new ArrayList<>();
        ArrayList<Double> nextParticleGuesses = Utils.randomArrayList(this.numOfParticles, 0, this.weightSum);
        ArrayList<Double> cummWeights = Utils.cummSums(this.weights); 
        nextParticleGuesses.sort(Utils.doubleComparator);
        while (!nextParticleGuesses.isEmpty()) {
            if (nextParticleGuesses.get(0) > cummWeights.get(0)){
                cummWeights.remove(0);
                this.weights.remove(0);
                this.particles.remove(0);
            } else {
                newParticles.add(updateParticle(this.particles.get(0)));
            }
        }
        this.particles = newParticles;
    }

    private void adjustWeights(){
        double currentSum = 0;
        for(double weight : this.weights){currentSum += weight;}
        int size = this.weights.size();
        double scale = weightSum / currentSum;
        for(int i = 0; i < size; i++){
            this.weights.set(i, this.weights.get(i) * scale);
        }
    }

    private void computeWeights() {
        int size = this.particles.size();
        for(int i = 0; i < size; i++){
            this.weights.set(i, this.weights.get(i) * this.weighter.weight(this.particles.get(i)));
        }
        adjustWeights();
    }

    private void filterParticles() {
        int size = particles.size();
        for(int i = 0; i < size; i++){
            if(illegalStateDeterminer.isIllegalState(this.particles.get(i))){
                this.particles.remove(i);
                this.weights.remove(i);
                i--; size--;
            }
        }
    }

    public ArrayList<RobotStates> getParticles() {return this.particles;}

    public RobotStates bestParticle(){
        RobotStates bestParticle = this.particles.get(0);
        double bestWeight = this.weights.get(0);
        int size = this.particles.size();
        for(int i = 1; i < size; i++){
            if (this.weights.get(i) > bestWeight){
                bestParticle = this.particles.get(i);
                bestWeight = this.weights.get(i);
            }
        }
        return bestParticle;
    }

    public void nextGeneration() {
        updateParticles();
        filterParticles();
        computeWeights();
    }
}
