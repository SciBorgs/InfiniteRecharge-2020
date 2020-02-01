package frc.robot.stateEstimation;

public interface MaybeUpdater extends Updater {
    // an updater that will only update on can update
    public boolean canUpdate();
}