package frc.robot.helpers;

public class DelayedPrinter {
    public static final int INTERVAL = 2; // change this to change your frequency of prints
    public static double numTicks = 0;     // rmbr that each tick is .02 seconds, 50 ticks/second

    public static void print (String message) {
        if (numTicks % INTERVAL == 0) {
            System.out.println(message);
        }
        numTicks += 1;
    }
}