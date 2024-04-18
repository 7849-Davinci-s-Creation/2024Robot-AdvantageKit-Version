package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lib.DashboardConfiguration;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveTrainConstants.GEAR_RATIO;

public class DriveTrain extends SubsystemBase implements DashboardConfiguration, DriveTrainIO {
    private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.MotorConstants.LEFT_FRONT_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.MotorConstants.LEFT_BACK_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.MotorConstants.RIGHT_FRONT_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.MotorConstants.RIGHT_BACK_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(inputs.gyroYaw, inputs.leftPositionRad, inputs.rightPositionRad);

    private boolean isBoosted = false;
    private boolean isCreeping = false;
    private boolean isNormal = true;
    private boolean isInverted = false;

    public DriveTrain() {
        zeroHeading();
        resetEncoders();

        // set back motors to follow the front ones.
        leftBackMotor.follow(leftFrontMotor);
        rightBackMotor.follow(rightFrontMotor);
        leftFrontMotor.setInverted(true);

        // put into break mode for safety and ease of use!
        leftFrontMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leftBackMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightFrontMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightBackMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void arcadeDrive(double rotate, double drive) {
        double maximum = Math.max(Math.abs(rotate), Math.abs(drive));
        double total = drive + rotate;
        double difference = drive - rotate;

        if (drive >= 0) {
            if (rotate >= 0) {
                setVoltage(maximum * 12, difference * 12);
            } else {
                setVoltage(total * 12, maximum * 12);
            }
        } else {
            if (rotate >= 0) {
                setVoltage(total * 12, -maximum * 12);
            } else {
                setVoltage(-maximum * 12, difference * 12);
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

    public void resetEncoders() {
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public void zeroHeading() {
        navx.reset();
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

    /** Returns the velocity of the left wheels in meters/second. */
    @AutoLogOutput
    public double getLeftVelocityMetersPerSec() {
        return inputs.leftVelocityRadPerSec * Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    /** Returns the velocity of the right wheels in meters/second. */
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
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);

        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / GEAR_RATIO);
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / GEAR_RATIO);

        inputs.leftAppliedVolts = leftFrontMotor.getAppliedOutput() * leftFrontMotor.getBusVoltage();
        inputs.rightAppliedVolts = rightFrontMotor.getAppliedOutput() * rightFrontMotor.getBusVoltage();

        inputs.leftCurrentAmps = new double[]{leftFrontMotor.getOutputCurrent(), leftBackMotor.getOutputCurrent()};
        inputs.rightCurrentAmps = new double[]{rightFrontMotor.getOutputCurrent(), rightBackMotor.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftFrontMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(rightVolts);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
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
