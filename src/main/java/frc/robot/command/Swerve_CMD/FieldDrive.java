// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Swerve_CMD;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FieldDrive extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final Elevator elevator;

  private SwerveRequest.FieldCentric driveF;
  private SwerveRequest.RobotCentric driveR;

  private double rate;
  private boolean isRobotRelative;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private DoubleSupplier vX, vY, vR;
  private BooleanSupplier btnIsPressed;


  /** Creates a new ChassisSpeed. */
  public FieldDrive(CommandSwerveDrivetrain swerve, Elevator elevator, PS5Controller driveCtrl, GenericHID switchCtrl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.elevator = elevator;

    vX = () -> -driveCtrl.getLeftY();
    vY = () -> -driveCtrl.getLeftX();
    vR = () -> -driveCtrl.getRightX();
    btnIsPressed = () -> switchCtrl.getRawButtonPressed(2);  // TODO: confirm the button's ID

    addRequirements(swerve, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("FieldDrive init");
    /* Setting up bindings for necessary control of the swerve drive platform */
    driveF = new SwerveRequest.FieldCentric()
                              .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
                              .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    driveR = new SwerveRequest.RobotCentric()
                              .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                              .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (btnIsPressed.getAsBoolean()) {
      isRobotRelative = true;
    }
    
    if (isRobotRelative) {  // Robot Relative
      if (Math.abs(vR.getAsDouble()) >= 0.2) {
        isRobotRelative = false;
        return;
      }
      swerve.setControl(driveR.withVelocityX(vX.getAsDouble() * maxSpeed) // Drive forward with negative Y(forward)
                              .withVelocityY(vY.getAsDouble() * maxSpeed) // Drive left with negative X (left)
                              .withRotationalRate(vR.getAsDouble() * maxAngularRate * 4) // Drive counterclockwise with negative X (left)
      );
    } else {  // Field Relative
      // System.out.printf("%.2f %.2f %.2f\n", vX.getAsDouble(), vY.getAsDouble(), vR.getAsDouble());
      if (elevator.getAbsolutePosition() > -35) {
        rate = 1;
      }
      else {
        rate = 0.5;
      }
      swerve.setControl(driveF.withVelocityX(vX.getAsDouble() * maxSpeed * rate) // Drive forward with negative Y(forward)
                            .withVelocityY(vY.getAsDouble() * maxSpeed * rate) // Drive left with negative X (left)
                            .withRotationalRate(vR.getAsDouble() * maxAngularRate * 4 * rate) // Drive counterclockwise with negative X (left)
      );
    }
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
