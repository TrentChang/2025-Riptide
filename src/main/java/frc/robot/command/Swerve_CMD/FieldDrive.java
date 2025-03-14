// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Swerve_CMD;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FieldDrive extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final Elevator elevator;
  private final PS5Controller Driver_Ctrl;

  private SwerveRequest.FieldCentric drive;

  private double ChassisSpeed;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private DoubleSupplier vX, vY, vR;

  /** Creates a new ChassisSpeed. */
  public FieldDrive(CommandSwerveDrivetrain swerve, Elevator elevator, PS5Controller Driver_Ctrl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.elevator = elevator;
    this.Driver_Ctrl = Driver_Ctrl;

    vX = () -> -Driver_Ctrl.getLeftY();
    vY = () -> -Driver_Ctrl.getLeftX();
    vR = () -> -Driver_Ctrl.getRightX();

    addRequirements(swerve, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Setting up bindings for necessary control of the swerve drive platform */
    drive = new SwerveRequest.FieldCentric()
                             .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.printf("%.2f %.2f %.2f\n", vX.getAsDouble(), vY.getAsDouble(), vR.getAsDouble());
    if (elevator.getAbsolutePosition() > -35) {
      ChassisSpeed = 1;
    }
    else {
      ChassisSpeed = 0.5;
    }
    swerve.setControl(drive.withVelocityX(vX.getAsDouble() * MaxSpeed * ChassisSpeed) // Drive forward with negative Y(forward)
                           .withVelocityY(vY.getAsDouble() * MaxSpeed * ChassisSpeed) // Drive left with negative X (left)
                           .withRotationalRate(vR.getAsDouble() * MaxAngularRate * 4 * ChassisSpeed) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
