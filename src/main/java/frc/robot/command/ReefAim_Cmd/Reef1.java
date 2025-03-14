// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.ReefAim_Cmd;

import java.rmi.server.RMIClassLoader;
import java.security.cert.PKIXRevocationChecker.Option;
import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Reef1 extends Command {
  private CommandSwerveDrivetrain swerve;
  boolean isFinished = false;

  private final Field2d Pose = new Field2d();
  private Pose2d robotPose, llPose;
  private int aprilTagID;
  private Command driveToPose;
  private AllianceStationID  AllianceStation;
  private boolean RedAlliance;
  /** Creates a new Reef1. */
  public Reef1(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;  
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init");

    aprilTagID = (int)LimelightHelpers.getFiducialID("");
    llPose = LimelightHelpers.getBotPose2d_wpiBlue("");

    AllianceStation = DriverStationJNI.getAllianceStation();
    if(DriverStation.getAlliance().isPresent()){
      System.out.println("Alliance" + AllianceStation);
      if(AllianceStation == AllianceStationID.Red1 || AllianceStation == AllianceStationID.Red2 || AllianceStation == AllianceStationID.Red3){
          RedAlliance = true;
      }
      else if(AllianceStation == AllianceStationID.Blue1 || AllianceStation == AllianceStationID.Blue2 || AllianceStation == AllianceStationID.Blue3){
          RedAlliance = false;
      }
    }

    if( -1 != aprilTagID){
      if (llPose.getX() == 0 && llPose.getY() == 0) {  // invalid Pose2d data
        robotPose = swerve.getState().Pose;
      } else {
        robotPose = llPose;
        swerve.resetPose(llPose);
      }

      if(DriverStation.getAlliance().isPresent()){
        if(RedAlliance){
            driveToPose = swerve.driveToPose(new Pose2d(5.806, 3.886, Rotation2d.fromDegrees(180.0)));
        }
        else if(!RedAlliance){
            driveToPose = swerve.driveToPose(new Pose2d(11.718, 4.214, Rotation2d.fromDegrees(0.0)));
        }
    } else {
      isFinished = true;
    }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executed");
    
    driveToPose.execute();
    isFinished = true;
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
