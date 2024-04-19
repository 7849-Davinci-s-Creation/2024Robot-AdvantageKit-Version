package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        double feedPositionRad = 0.0;
        double feedVelocityRadPerSec = 0.0;
        double feedAppliedVolts = 0.0;
        double[] feedCurrentAmps = new double[] {};

        double intakePositionRad = 0.0;
        double intakeVelocityRadPerSec = 0.0;
        double intakeAppliedVolts = 0.0;
        double[] intakeCurrentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Run the feeder wheel at the specified voltage. */
    default void setFeedVoltage(double volts) {}

    default void setIntakeVoltage(double volts){}
}
