package frc.robot.Subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ShooterReal implements ShooterIO {
    private final CANSparkMax topFlyWheel = new CANSparkMax(Constants.MotorConstants.FLY_WHEEL_TOP,
            CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax bottomFlyWheel = new CANSparkMax(Constants.MotorConstants.FLY_WHEEL_BOTTOM,
            CANSparkLowLevel.MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder = topFlyWheel.getEncoder();

    private final PIDController shooterPID = new PIDController(Constants.ShooterConstants.P,
            Constants.ShooterConstants.I, Constants.ShooterConstants.D);

    public ShooterReal() {
        topFlyWheel.setInverted(true);
        topFlyWheel.setIdleMode(CANSparkBase.IdleMode.kBrake);
        bottomFlyWheel.follow(topFlyWheel);
        bottomFlyWheel.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public  void updateInputs(ShooterIOInputs inputs) {

    }

    @Override
    public void setLaunchVoltage(double volts) {
        topFlyWheel.setVoltage(volts);
    }


}
