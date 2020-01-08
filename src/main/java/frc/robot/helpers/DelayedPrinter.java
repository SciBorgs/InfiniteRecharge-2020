package frc.robot.helpers;

public class DelayedPrinter {
    public static final int INTERVAL = 50; // default num of ticks/print
    public static double numTicks = 0;     // rmbr that each tick is .02 seconds, 50 ticks/second

    public static void print (String message, int interval) {
        if (numTicks % interval == 0) {
            System.out.println(message);
        }
    }
    public static void print (String message) {
        print(message, INTERVAL);
    }

    public static void incTicks(){  
        numTicks += 1;
    }
}