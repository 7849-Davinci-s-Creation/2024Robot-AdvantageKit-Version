package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveIOInputsAutoLogged;
import lib.DashboardConfiguration;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveTrain extends SubsystemBase implements DashboardConfiguration {
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DriveTrainIO io;

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(inputs.gyroYaw, inputs.leftPositionRad, inputs.rightPositionRad);

    private boolean isBoosted = false;
    private boolean isCreeping = false;
    private boolean isNormal = true;
    private boolean isInverted = false;

    public DriveTrain(DriveTrainIO io) {
        this.io = io;
    }

    public void arcadeDrive(double rotate, double drive) {
        double maximum = Math.max(Math.abs(rotate), Math.abs(drive));
        double total = drive + rotate;
        double difference = drive - rotate;

        if (drive >= 0) {
            if (rotate >= 0) {
                io.setVoltage(maximum * 12, difference * 12);
            } else {
                io.setVoltage(total * 12, maximum * 12);
            }
        } else {
            if (rotate >= 0) {
                io.setVoltage(total * 12, -maximum * 12);
            } else {
                io.setVoltage(-maximum * 12, difference * 12);
            }
        }

    }

    public double applyCurve(double position) {
        // first part of equation is the same so extract to variable
        double part1 = (1 - Constants.DriveTrainConstants.TORQUE_RESISTANCE_THRESHOLD) * Math.pow(position, 3);

        // apply piecewise logic
        if (position > 0) {
            return part1 + Constants.DriveTrainConstants.TORQUE_RESISTANCE_THRESHOLD;
        } else if (position < 0) {
            return part1 - Constants.DriveTrainConstants.TORQUE_RESISTANCE_THRESHOLD;
        }

        // else joystick position is 0 so return 0
        return 0;
    }

    public double handleDeadzone(double value, double deadZone) {
        // why is this breaking?
        if (Math.abs(value) < deadZone) {
            return 0;
        }

        return value;
    }

    @AutoLogOutput(key = "odometry/robot")
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @AutoLogOutput
    public double getLeftEncoderPositionMeters() {
        return inputs.leftPositionRad * Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    @AutoLogOutput
    public double getRightEncoderPositionMeters() {
        return inputs.rightPositionRad * Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    @AutoLogOutput
    public double getHeading() {
        return inputs.gyroYaw.getDegrees();
    }

    @AutoLogOutput
    public double getLeftVelocityMetersPerSec() {
        return inputs.leftVelocityRadPerSec * Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    @AutoLogOutput
    public double getRightVelocityMetersPerSec() {
        return inputs.rightVelocityRadPerSec * Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    public Boolean isBoosted() {
        return this.isBoosted;
    }

    public Boolean isCreeping() {
        return this.isCreeping;
    }

    public boolean isNormal() {
        return this.isNormal;
    }

    public void setCreeping(boolean isCreeping) {
        this.isCreeping = isCreeping;
    }

    public void setBoosted(boolean isBoosted) {
        this.isBoosted = isBoosted;
    }

    public void setNormal(boolean isNormal) {
        this.isNormal = isNormal;
    }

    public boolean isInverted() {
        return this.isInverted;
    }

    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }

    public void setNormalDriving() {
        this.setBoosted(false);
        this.setCreeping(false);
        this.setNormal(true);
    }

    public void setBoostedDriving() {
        this.setBoosted(true);
        this.setCreeping(false);
        this.setNormal(false);
    }

    public void setCreepedDriving() {
        this.setBoosted(false);
        this.setCreeping(true);
        this.setNormal(false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);

        odometry = new DifferentialDriveOdometry(inputs.gyroYaw,
                getLeftEncoderPositionMeters(),
                getRightEncoderPositionMeters()
        );

        this.configureDashboard();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void configureDashboard() {
        // Log current driving mode to smart dashboard
        SmartDashboard.putBoolean("Inverted", this.isInverted());
        SmartDashboard.putBoolean("Boosted", this.isBoosted());
        SmartDashboard.putBoolean("Creep", this.isCreeping());
        SmartDashboard.putBoolean("Normal", this.isNormal());

        if (RobotContainer.debugMode) {
            SmartDashboard.putNumber("Gyro Heading", getHeading());
            SmartDashboard.putNumber("Left Encoder Value (feet)", getLeftEncoderPositionMeters());
            SmartDashboard.putNumber("Right Encoder Value (feet) ", getRightEncoderPositionMeters());
        }
    }
}
