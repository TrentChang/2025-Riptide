// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Swerve_CMD;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChassisSpeed extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final Elevator elevator;

  double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  public SwerveRequest.FieldCentric SwerveDrive;
  public double SpeedPercent;
  /** Creates a new ChassisSpeed. */
  public ChassisSpeed(CommandSwerveDrivetrain swerve, Elevator elevator) {
    this.swerve = swerve;
    this.elevator = elevator;
    addRequirements(swerve, elevator);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.getAbsolutePosition() > -35){
      SwerveDrive = new SwerveRequest.FieldCentric()
                                     .withDeadband(MaxSpeed * 0.1)
                                     .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                                     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

      SpeedPercent = 1;
    }
    else{
      SwerveDrive = new SwerveRequest.FieldCentric()
                                     .withDeadband(MaxSpeed * 0.1)
                                     .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                                     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
      
      SpeedPercent = 0.5;
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
