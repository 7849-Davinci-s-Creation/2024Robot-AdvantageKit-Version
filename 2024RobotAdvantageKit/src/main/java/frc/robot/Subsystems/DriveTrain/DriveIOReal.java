package frc.robot.Subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

import static frc.robot.Constants.DriveTrainConstants.GEAR_RATIO;

public class DriveIOReal implements DriveTrainIO {
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

    public DriveIOReal() {
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

    public void resetEncoders() {
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public void zeroHeading() {
        navx.reset();
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
}
