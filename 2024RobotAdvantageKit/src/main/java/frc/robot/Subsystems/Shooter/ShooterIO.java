package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        double launchPositionRad = 0.0;
        double launchVelocityRadPerSec = 0.0;
        double launchAppliedVolts = 0.0;
        double[] launchCurrentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Run the launcher wheel at the specified voltage. */
    default void setLaunchVoltage(double volts) {}
}
