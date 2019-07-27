package frc.robot.helpers;

import java.util.ArrayList;
import java.util.Random;
import java.util.Collections;

public class ParticleFilter {

    private int numOfParticles = 100;
    private int numOfIterations = 100;
    private ArrayList<Double> particles;
    private ArrayList<Double> weights;
    private Random generator = new Random();

    public ParticleFilter(){}

    public void reset() {
        this.particles.clear(); 
        this.weights.clear();
    }

    public void generateParticles() {
        for (int i = 0; i<= this.numOfParticles; i++) {
            this.particles.add(generator.nextDouble() * 100);
        }
    }

    public void computeWeights(Double prediction, Double measurement) {
        double randomDouble = generator.nextDouble() * 5;
        double measurementNoise = this.computeGaussian(0, 5, randomDouble);
        double sumOfWeights = 0;

        for (int i = 0; i <= this.numOfParticles; i++) {
            Double weight = this.computeGaussian(prediction, measurementNoise, measurement);
            weights.add(weight);
            sumOfWeights += weight;
        }

        for (int i = 0; i <= this.numOfParticles; i++) {
            Double normalizedWeight = weights.get(i)/sumOfWeights;
            weights.set(i, normalizedWeight);
        }
    }

    public ArrayList<Double> getParticles() {return this.particles;}

    public void resampleParticles(ArrayList<Double> weights) {
        ArrayList<Double> newParticles = new ArrayList<Double>();

        int randomIndex = generator.nextInt() *numOfParticles;
        Double beta = 0.0;
        Double biggestWeight = Collections.max(this.weights);

        for (int i = 0; i <= this.numOfParticles; i++) {
            beta += generator.nextDouble() * 2.0 * biggestWeight;

            while(beta > this.weights.get(randomIndex)) {
                beta -= this.weights.get(randomIndex);
                randomIndex = (randomIndex + 1) % this.numOfParticles;
            }

            newParticles.add(this.particles.get(randomIndex));
        }

        this.particles = newParticles;

    }

    public ArrayList<Double> filter(Double prediction, Double measurement) {
        this.generateParticles();
        for (int i = 0; i<= this.numOfIterations; i++) {
            this.computeWeights(prediction, measurement);
            this.resampleParticles(this.weights);
        }

        return this.particles;
    }

    public double computeGaussian(double mean, double variance, double x) {
        double gaussianProbability = 1/(variance * Math.sqrt(2 * Math.PI)) * Math.pow(Math.E, -(1/2) * Math.pow(((x - mean)/variance), 2));
        return gaussianProbability;
    }
}