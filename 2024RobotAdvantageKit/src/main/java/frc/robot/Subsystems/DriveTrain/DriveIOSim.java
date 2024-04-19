package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;

public class DriveIOSim implements DriveTrainIO {
    private final DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
                    DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                    DifferentialDrivetrainSim.KitbotGearing.k8p45,
                    DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                    null);

    private double leftAppliedVolts =0.0;
    private double rightAppliedVolts =0.0;

    private boolean closedLoop = false;

    private PIDController leftController = new PIDController(0,0,0);
    private PIDController rightController = new PIDController(0,0,0);

    private double leftFeedForwardVolts = 0.0;
    private double rightFeedForwardVolts = 0.0;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        if (closedLoop) {
            leftAppliedVolts = MathUtil.clamp(
                    leftController.calculate(sim.getLeftVelocityMetersPerSecond() / Constants.DriveTrainConstants.WHEEL_RADIUS_METERS) + leftFeedForwardVolts,
                    -12.0,
                    12.0
            );

            rightAppliedVolts = MathUtil.clamp(
                    rightController.calculate(sim.getLeftVelocityMetersPerSecond() / Constants.DriveTrainConstants.WHEEL_RADIUS_METERS) + rightFeedForwardVolts,
                    -12.0,
                    12.0
            );

            sim.setInputs(leftAppliedVolts, rightAppliedVolts);
        }

        sim.update(0.2);

        inputs.leftPositionRad = sim.getLeftPositionMeters() / Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
        inputs.rightPositionRad = sim.getRightPositionMeters() / Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;

        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;
        inputs.rightVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Constants.DriveTrainConstants.WHEEL_RADIUS_METERS;

        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.rightAppliedVolts = rightAppliedVolts;

        inputs.leftCurrentAmps = new double[]{sim.getLeftCurrentDrawAmps()};
        inputs.rightCurrentAmps = new double[]{sim.getRightCurrentDrawAmps()};

        inputs.gyroYaw = sim.getHeading();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        closedLoop = false;

        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

        sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        closedLoop = true;

        leftController.setSetpoint(leftRadPerSec);
        rightController.setSetpoint(rightRadPerSec);

        this.leftFeedForwardVolts = leftFFVolts;
        this.rightFeedForwardVolts = rightFFVolts;
    }
}
