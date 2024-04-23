// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DriveTrain.DriveIOReal;
import frc.robot.Subsystems.DriveTrain.DriveIOSim;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainIO;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeReal;
import frc.robot.Subsystems.Intake.IntakeSim;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterReal;
import frc.robot.Subsystems.Shooter.ShooterSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Robot Modes
  public static Constants.Mode currentMode = Constants.Mode.REAL;

  //Controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Subsystems
  private final DriveTrain driveTrain;
  private final Shooter shooter;
  private final Intake intake;

  // Other
  private final LoggedDashboardChooser<Command> autoMenu;

  public RobotContainer() {
    switch (currentMode) {
      case REAL -> {
        this.driveTrain = new DriveTrain(new DriveIOReal());
        this.shooter = new Shooter(new ShooterReal());
        this.intake = new Intake(new IntakeReal());
      }

      case SIM -> {
        this.driveTrain = new DriveTrain(new DriveIOSim());
        this.shooter = new Shooter(new ShooterSim());
        this.intake = new Intake(new IntakeSim());
      }

      default -> {
        this.driveTrain = new DriveTrain(new DriveTrainIO() {});
        this.shooter = new Shooter(new ShooterIO() {});
        this.intake = new Intake(new IntakeIO() {});
      }
    }

    configureBindings();

    autoMenu = new LoggedDashboardChooser<>("Auto Menu", AutoBuilder.buildAutoChooser());
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return autoMenu.get();
  }
}
