package frc.robot.dataTypes;

public class Triplet<First, Second, Third> {

    public First  first;
    public Second second;
    public Third  third;

    public Triplet(First first, Second second, Third third) {
        this.first  = first;
        this.second = second;
        this.third  = third;
    }
}