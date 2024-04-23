package frc.robot.Subsystems.Intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeReal implements IntakeIO {
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorConstants.INTAKE_MOTOR);
    private final DigitalInput digitalInput = new DigitalInput(Constants.IntakeConstants.BEAM_BREAKER_PORT);

    public IntakeReal() {
        intakeMotor.setInverted(true);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, volts * 12);
    }

    @Override
    public void setFeedVoltage(double volts) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, volts * 12);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = intakeMotor.getMotorOutputVoltage();
        inputs.intakeCurrentAmps = new double[]{intakeMotor.getSupplyCurrent()};

        inputs.feedAppliedVolts = intakeMotor.getMotorOutputVoltage();
        inputs.intakeCurrentAmps = new double[]{intakeMotor.getSupplyCurrent()};

        inputs.noteState = digitalInput.get();
    }
}
