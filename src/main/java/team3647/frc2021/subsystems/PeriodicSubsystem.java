package team3647.frc2021.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * basically 254's code
 */
public interface PeriodicSubsystem extends Subsystem {

    default void init() {
    }

    default void end() {
    }

    default void readPeriodicInputs() {
    }

    default void writePeriodicOutputs() {
    }

    @Override
    default void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }
    
    public String getName();
}
