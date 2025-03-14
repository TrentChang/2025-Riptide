// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Swerve_CMD;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;


public class RobotDrive extends Command {
  private final CommandSwerveDrivetrain swerve;

  private final Command fieldDriveCmd;

  private SwerveRequest.RobotCentric drive;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private DoubleSupplier vX, vY, vR;

  public RobotDrive(CommandSwerveDrivetrain swerve, PS5Controller controller, Command fieldDriveCmd) {
    this.swerve = swerve;
    this.fieldDriveCmd = fieldDriveCmd;

    vX = () -> -controller.getLeftY();
    vY = () -> -controller.getLeftX();
    vR = () -> -controller.getRightX();

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    drive = new SwerveRequest.RobotCentric()
                             .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  }

  @Override
  public void execute() {
    swerve.setControl(drive.withVelocityX(vX.getAsDouble() * maxSpeed) // Drive forward with negative Y(forward)
                           .withVelocityY(vY.getAsDouble() * maxSpeed) // Drive left with negative X (left)
                           .withRotationalRate(vR.getAsDouble() * maxAngularRate * 4) // Drive counterclockwise with negative X (left)
    );
    if (Math.abs(vR.getAsDouble()) >= 0.2) {
      end(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());
    swerve.removeDefaultCommand();
    swerve.setDefaultCommand(fieldDriveCmd);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
