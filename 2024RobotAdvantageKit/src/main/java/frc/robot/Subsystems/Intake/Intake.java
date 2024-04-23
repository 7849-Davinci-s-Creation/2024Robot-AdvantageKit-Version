package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();


    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void intake(double output) {
        io.setIntakeVoltage(output);
    }

    public void feed(double output) {
        io.setFeedVoltage(output);
    }

    @AutoLogOutput
    public boolean getNoteState() {
        return inputs.noteState;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
