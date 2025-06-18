// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Swerve_CMD;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartDrive extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final Elevator elevator;
  private final Climber climber;

  private SwerveRequest.FieldCentric driveF;
  private SwerveRequest.RobotCentric driveR;

  private double rate, dX, dY, dR;
  private boolean isRobotRelative;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private DoubleSupplier vX, vY, vR;
  private BooleanSupplier btnIsPressed;


  /** Creates a new ChassisSpeed. */
  public SmartDrive(CommandSwerveDrivetrain swerve, Elevator elevator, Climber climber, XboxController driveCtrl, GenericHID switchCtrl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.elevator = elevator;
    this.climber =climber;

    vX = () -> driveCtrl.getRawAxis(5);
    vY = () -> driveCtrl.getRawAxis(4);
    vR = () -> -driveCtrl.getRawAxis(0);
    btnIsPressed = () -> switchCtrl.getRawButtonPressed(10);  // TODO: confirm the button's ID

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

  private double limitSpeed(double rawInput) {
    /*  0 ~ 0.1 --> 0 
     *  0.1 ~ 0.4 --> 0.2
    */
    if (Math.abs(rawInput) <= 0.1) {
      return 0;
    } else if (-0.4 >= rawInput || rawInput >= 0.4) {
      return rawInput;
    } else if (rawInput < 0) {
      return -0.1;
    } else {
      return 0.1;
    }
  }

  // private double ClimbingSpeed(double rawInput){
  //   if(climber.getAbsolutePosition() )
  // }
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
      swerve.setControl(driveR.withVelocityX(vX.getAsDouble() * maxSpeed * 0.25) // Drive forward with negative Y(forward)
                              .withVelocityY(vY.getAsDouble() * maxSpeed * 0.25) // Drive left with negative X (left)
      );
    } else {  // Field Relative
      // System.out.printf("%.2f %.2f %.2f\n", vX.getAsDouble(), vY.getAsDouble(), vR.getAsDouble());
        if (elevator.getAbsolutePosition() > -35) {
          rate = 1;
        }
        else {
          rate = 0.5;
        }
        
      swerve.setControl(driveF.withVelocityX(limitSpeed(vX.getAsDouble()) * maxSpeed * rate) // Drive forward with negative Y(forward)
                            .withVelocityY(limitSpeed(vY.getAsDouble()) * maxSpeed * rate) // Drive left with negative X (left)
                            .withRotationalRate(limitSpeed(vR.getAsDouble()) * maxAngularRate * 4 * rate) // Drive counterclockwise with negative X (left)
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