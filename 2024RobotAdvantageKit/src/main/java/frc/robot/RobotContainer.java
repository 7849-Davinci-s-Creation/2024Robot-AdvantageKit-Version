// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.DriveTrain.DriveIOReal;
import frc.robot.Subsystems.DriveTrain.DriveIOSim;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainIO;

public class RobotContainer {
  public static boolean debugMode = false;

  public static Constants.Mode currentMode = Constants.Mode.REAL;

  private final DriveTrain driveTrain;

  public RobotContainer() {
    switch (currentMode) {
      case REAL -> this.driveTrain = new DriveTrain(new DriveIOReal());

      case SIM -> this.driveTrain = new DriveTrain(new DriveIOSim());

      default -> this.driveTrain = new DriveTrain(new DriveTrainIO() {});
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
