package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DriveTrainIO {
    @AutoLog
    class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};

        public Rotation2d gyroYaw = new Rotation2d();

        public boolean isBoosted = false;
        public boolean isCreeping = false;
        public boolean isNormal = true;
        public boolean isInverted = false;
    }

    default void updateInputs(DriveIOInputs inputs) {}

    default void setVoltage(double leftVolts, double rightVolts) {}

    default void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}
}
