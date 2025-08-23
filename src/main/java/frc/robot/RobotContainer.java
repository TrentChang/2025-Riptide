// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.time.Instant;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.util.ISO8601DateFormat;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;

import frc.robot.command.test;
import frc.robot.command.Aim_Cmd.AimCoralStation;
import frc.robot.command.Aim_Cmd.AutoAlignment;
import frc.robot.command.Aim_Cmd.AutoReefLevel;
import frc.robot.command.Auto_Cmd.AutoAim;
import frc.robot.command.Auto_Cmd.AutoL1;
import frc.robot.command.Auto_Cmd.AutoL2;
import frc.robot.command.Auto_Cmd.AutoL3;
import frc.robot.command.Auto_Cmd.AutoL4;
import frc.robot.command.Auto_Cmd.AutoShootCoral;
import frc.robot.command.Auto_Cmd.AutoSuckCoral;
import frc.robot.command.Group_Cmd.Barge;
import frc.robot.command.Group_Cmd.RL1;
import frc.robot.command.Group_Cmd.RL2;
import frc.robot.command.Group_Cmd.RL3;
import frc.robot.command.Group_Cmd.RL4;
import frc.robot.command.Group_Cmd.ReefAlgae;
import frc.robot.command.Group_Cmd.SetZero;
import frc.robot.command.Group_Cmd.CoralStation;
import frc.robot.command.Single_Cmd.CoralShoot;
import frc.robot.command.Single_Cmd.SetClimberAsHead;
import frc.robot.command.Swerve_CMD.SmartDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.limelight;
import static frc.robot.TargetChooser.reefMap;

public class RobotContainer {
    private final XboxController Driver_Ctrl = new XboxController(0);
    private final XboxController Driver_Ctrl2 = new XboxController(1);

    private final Joystick BIG_BUTTON = new Joystick(3); // BIG BUTTON
    private final Joystick REEF_BUTTON = new Joystick(4); // REEF BUTTON
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Arm arm = new Arm();
    //public final Candle candle = new Candle();
    //public final Climber climber = new Climber();   
    public final Claw claw = new Claw();
    public final Elevator elevator = new Elevator();
    public final limelight limelight = new limelight();
    public final TargetChooser targetChooser = new TargetChooser();

    //autoTarget station down Command
    public final ParallelCommandGroup stationAimStop = new ParallelCommandGroup(new InstantCommand(claw::Claw_Stop, claw), new InstantCommand(arm::Arm_Zero, arm));
    // Aim Command
    public final AimCoralStation CMD_AimCoralStation = new AimCoralStation(arm, drivetrain, claw, limelight);
    // public final AutoAlignment CMD_AutoAlignment = new AutoAlignment(drivetrain);

    // Auto Command
    public final AutoAim CMD_AutoAim = new AutoAim(drivetrain);
    public final CoralStation CMD_CoralStation = new CoralStation(elevator, claw, arm);
    public final AutoSuckCoral CMD_AutoSuckCoral = new AutoSuckCoral(elevator, claw, arm);
    public final AutoL4 CMD_AutoL4 = new AutoL4(arm, elevator);
    public final AutoL3 CMD_AutoL3 = new AutoL3(arm, elevator);
    public final AutoL2 CMD_AutoL2 = new AutoL2(arm, elevator);
    public final AutoL1 CMD_AutoL1 = new AutoL1(arm, elevator);
    public final AutoShootCoral CMD_AutoShootCoral = new AutoShootCoral(claw, arm, elevator, drivetrain);

    // Group Command
    public final Barge CMD_Barege = new Barge(arm, claw, elevator); 
    public final ReefAlgae CMD_ReefAlgae = new ReefAlgae(arm, elevator);
    public final RL1 CMD_RL1 = new RL1(arm, claw, elevator);
    public final RL2 CMD_RL2 = new RL2(arm, claw, elevator);
    public final RL3 CMD_RL3 = new RL3(arm, claw, elevator);
    public final RL4 CMD_RL4 = new RL4(arm, claw, elevator);
    public final SetZero CMD_SetZero = new SetZero(arm, claw, elevator);
    public final CoralStation CoralStation = new CoralStation(elevator, claw, arm);

    // Single Command
    public final CoralShoot CMD_CoralShoot = new CoralShoot(arm, claw, elevator);
    public final SetClimberAsHead CMD_SetClimberAsHead = new SetClimberAsHead(drivetrain);

    // Swerve Command
    public final SmartDrive CMD_SmartDrive = new SmartDrive(drivetrain, elevator, Driver_Ctrl, Driver_Ctrl2);

    public final test CMD_test = new test(drivetrain);

    private SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private Supplier<Command> DTP_CMD = () -> targetChooser.driveToClosestReef(drivetrain);

    private int reefLevel = 0;
    private Supplier<Integer> reefLevelSupplier = () -> {return reefLevel;};

    public RobotContainer() {
        configureBindings();
        Driver_ConfigureBindings();
        //Driver2_ConfigureBinding();
        // Assist_ConfigureBindings();
        BIG_BUTTON_ConfigureBingdings();
        REEF_BUTTON_ConfigureBindings();
        updateReefLevel();
        
        NamedCommands.registerCommand("SetClimberAsHead", CMD_SetClimberAsHead);
        NamedCommands.registerCommand("AutoShootCoral", CMD_AutoShootCoral);
        NamedCommands.registerCommand("AutoSuckCoral", CMD_AutoSuckCoral);
        NamedCommands.registerCommand("AutoCoralStation", CMD_AimCoralStation);
        NamedCommands.registerCommand("AutoRL4", CMD_AutoL4);
        NamedCommands.registerCommand("AutoRL3", CMD_AutoL3);
        NamedCommands.registerCommand("AutoRL2", CMD_AutoL2);
        NamedCommands.registerCommand("AutoRL1", CMD_AutoL1);
        NamedCommands.registerCommand("CoralSuck", new InstantCommand(claw::Claw_Suck));
        NamedCommands.registerCommand("AutoAim", Commands.defer(DTP_CMD, Set.of(drivetrain)));
        NamedCommands.registerCommand("ResetPose", new InstantCommand(() -> drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(""))));
        // NamedCommands.registerCommand("AutoAlignment", CMD_AutoAlignment);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto", autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void Driver_ConfigureBindings() {
        //Default pigeon
        new JoystickButton(Driver_Ctrl,3).onTrue(new InstantCommand(drivetrain::ResetPigeon, drivetrain));
        //Ds pose
        new JoystickButton(Driver_Ctrl, 1).onTrue(CoralStation);
        //coral
        new Trigger(() -> Driver_Ctrl.getLeftTriggerAxis() >= 0.5).whileTrue(new InstantCommand(claw::Claw_Suck, claw))
                                                        .onFalse(new InstantCommand(claw::Claw_Stop, claw));
        new Trigger(() -> Driver_Ctrl.getRightTriggerAxis() >= 0.5).whileTrue(new InstantCommand(claw::Claw_Shoot, claw))
                                                        .onFalse(new InstantCommand(claw::Claw_Stop, claw));
        //algae
        new JoystickButton(Driver_Ctrl, 5).whileTrue(new InstantCommand(intake::suck, intake))
                                                       .onFalse(new InstantCommand(intake::Stop, intake));
        new JoystickButton(Driver_Ctrl, 6).whileTrue(new InstantCommand(intake::shoot, intake))
                                                       .onFalse(new InstantCommand(intake::Stop, intake));
        new JoystickButton(Driver_Ctrl, 4).onTrue(new InstantCommand(intake::Intake_out, intake));
        new JoystickButton(Driver_Ctrl, 2).onTrue(new InstantCommand(intake::Intake_Zero, intake));
        //Elevator ctrl
        new POVButton(Driver_Ctrl, 0).onTrue(CMD_RL1);
        new POVButton(Driver_Ctrl, 90).onTrue(CMD_RL2);
        new POVButton(Driver_Ctrl, 180).onTrue(CMD_RL3);
        new POVButton(Driver_Ctrl, 270).onTrue(CMD_RL4);
        new JoystickButton(Driver_Ctrl, 3).onTrue(CMD_RL1);
        new JoystickButton(Driver_Ctrl, 7).onTrue(new InstantCommand(() -> {
            drivetrain.seedFieldCentric();
        }, drivetrain));
    }

    // private void Driver2_ConfigureBinding(){
    //     new JoystickButton(Driver_Ctrl2, 1).whileTrue(new InstantCommand(claw::Claw_Suck, claw))
    //                                                     .onFalse(new InstantCommand(claw::Claw_Stop, claw));
    //     new JoystickButton(Driver_Ctrl2, 2).whileTrue(new InstantCommand(claw::Claw_Shoot, claw))
    //                                                    .onFalse(new InstantCommand(claw::Claw_Stop, claw));

    //     new JoystickButton(Driver_Ctrl2, 3).onTrue(new InstantCommand(arm::Arm_Station, arm))
    //                                                     .onTrue(new InstantCommand(elevator::ELE_Floor, elevator))
    //                                                     .whileTrue(new InstantCommand(claw::Claw_Suck, claw ))
    //                                                     .onFalse(new InstantCommand(claw::Claw_Stop, claw));

    //     new JoystickButton(Driver_Ctrl2, 4).onTrue(new InstantCommand(arm::Arm_Zero, arm))
    //                                                     .onTrue(new InstantCommand(claw::Claw_Stop, claw))
    //                                                     .onTrue(new InstantCommand(elevator::ELE_Floor, elevator));

    //     new JoystickButton(Driver_Ctrl2, 5).whileTrue(new InstantCommand(intake::suck, intake))
    //                                                     .onFalse(new InstantCommand(intake::Stop, intake));

    //     new JoystickButton(Driver_Ctrl2, 6).whileTrue(new InstantCommand(intake::shoot, intake))
    //                                                      .onFalse(new InstantCommand(intake::Stop, intake));


    //     new POVButton(Driver_Ctrl2, 0).onTrue(CMD_RL1);
    //     new POVButton(Driver_Ctrl2, 90).onTrue(CMD_RL2);
    //     new POVButton(Driver_Ctrl2, 180).onTrue(CMD_RL3);
    //     new POVButton(Driver_Ctrl2, 270).onTrue(CMD_RL4);
    // }

    private void BIG_BUTTON_ConfigureBingdings(){
        //backup 
        // new JoystickButton(BIG_BUTTON, 4).onTrue(CMD_RL1);
        // new JoystickButton(BIG_BUTTON, 3).onTrue(CMD_RL2);
        // new JoystickButton(BIG_BUTTON, 2).onTrue(CMD_RL3);
        // new JoystickButton(BIG_BUTTON, 1).onTrue(CMD_RL4);
        //elevator default
        new JoystickButton(BIG_BUTTON, 8).onTrue(CMD_RL1);
        //coral station autotarget
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                //red right
                new JoystickButton(BIG_BUTTON, 5).whileTrue(drivetrain.driveToPose(reefMap.get(2).get(0)).alongWith(new CoralStation(elevator, claw, arm)))
                                                                .onFalse(stationAimStop);
                //red left
                new JoystickButton(BIG_BUTTON, 6).whileTrue(drivetrain.driveToPose(reefMap.get(1).get(0))
                                                                .alongWith(new CoralStation(elevator, claw, arm)))
                                                                .onFalse(stationAimStop);
            }
            else{
                //blue right
                new JoystickButton(BIG_BUTTON, 5).whileTrue(drivetrain.driveToPose(reefMap.get(12).get(0))
                                                                .alongWith(new CoralStation(elevator, claw, arm)))
                                                                .onFalse(stationAimStop);
                //blue left
                new JoystickButton(BIG_BUTTON, 6).whileTrue(drivetrain.driveToPose(reefMap.get(13).get(0))
                                                                .alongWith(new CoralStation(elevator, claw, arm)))
                                                                .onFalse(stationAimStop);
            }
        }
        else{
            System.out.println("WARNING: Alliance NOT DETECTED!");
        }

        // new JoystickButton(BIG_BUTTON, 5).onTrue(new InstantCommand(claw::L1ClawShoot, claw));
        // new JoystickButton(BIG_BUTTON, 7).whileTrue(new InstantCommand(claw::AlgaeClawShoot)).onFalse(new InstantCommand(claw::Claw_Stop));
        // new JoystickButton(BIG_BUTTON, 8).whileTrue(CMD_SetClimberAsHead);
        // new JoystickButton(BIG_BUTTON, 9).whileTrue(new InstantCommand(climber::Up, climber))
        //                                                .onFalse(new InstantCommand(climber::Stop, climber));
        // new JoystickButton(BIG_BUTTON, 10).whileTrue(new InstantCommand(climber::Down, climber))
        //                                                .onFalse(new InstantCommand(climber::Stop, climber));
        // new JoystickButton(BIG_BUTTON, 9).whileTrue(CMD_Barege);
        // new JoystickButton(BIG_BUTTON, 10).onTrue(CMD_ReefAlgae);
    }
    
    private void REEF_BUTTON_ConfigureBindings(){
       Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            System.out.printf("Alliance is now %s\n", alliance.get().name());
            if (alliance.get() == Alliance.Red) {
                new JoystickButton(REEF_BUTTON, 1).whileTrue(
                    drivetrain.driveToPose(reefMap.get(7).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 7, 0, reefLevelSupplier))
                    .alongWith(new PrintCommand("=========="))
                );
                new JoystickButton(REEF_BUTTON, 2).whileTrue(
                    drivetrain.driveToPose(reefMap.get(7).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 7, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 3).whileTrue(
                    drivetrain.driveToPose(reefMap.get(8).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 8, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 4).whileTrue(
                    drivetrain.driveToPose(reefMap.get(8).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 8, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 5).whileTrue(
                    drivetrain.driveToPose(reefMap.get(9).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 9, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 6).whileTrue(
                    drivetrain.driveToPose(reefMap.get(9).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 9, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 7).whileTrue(
                    drivetrain.driveToPose(reefMap.get(10).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 10, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 8).whileTrue(
                    drivetrain.driveToPose(reefMap.get(10).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 10, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 9).whileTrue(
                    drivetrain.driveToPose(reefMap.get(11).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 11, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 10).whileTrue(
                    drivetrain.driveToPose(reefMap.get(11).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 11, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 11).whileTrue(
                    drivetrain.driveToPose(reefMap.get(6).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 6, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 12).whileTrue(
                    drivetrain.driveToPose(reefMap.get(6).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 6, 1, reefLevelSupplier))
                );
            } else {
                new JoystickButton(REEF_BUTTON, 1).whileTrue(
                    drivetrain.driveToPose(reefMap.get(18).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 18, 0, reefLevelSupplier))
                    .alongWith(new PrintCommand("=========="))
                );
                new JoystickButton(REEF_BUTTON, 2).whileTrue(
                    drivetrain.driveToPose(reefMap.get(18).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 18, 1, reefLevelSupplier))
                    .alongWith(new PrintCommand("=========="))
                );
                new JoystickButton(REEF_BUTTON, 3).whileTrue(
                    drivetrain.driveToPose(reefMap.get(17).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 17, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 4).whileTrue(
                    drivetrain.driveToPose(reefMap.get(17).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 17, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 5).whileTrue(
                    drivetrain.driveToPose(reefMap.get(22).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 22, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 6).whileTrue(
                    drivetrain.driveToPose(reefMap.get(22).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 22, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 7).whileTrue(
                    drivetrain.driveToPose(reefMap.get(21).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 21, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 8).whileTrue(
                    drivetrain.driveToPose(reefMap.get(21).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 21, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 9).whileTrue(
                    drivetrain.driveToPose(reefMap.get(20).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 20, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 10).whileTrue(
                    drivetrain.driveToPose(reefMap.get(20).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 20, 1, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 11).whileTrue(
                    drivetrain.driveToPose(reefMap.get(19).get(0))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 19, 0, reefLevelSupplier))
                );
                new JoystickButton(REEF_BUTTON, 12).whileTrue(
                    drivetrain.driveToPose(reefMap.get(19).get(1))
                    .alongWith(new AutoReefLevel(drivetrain, arm, elevator, 19, 1, reefLevelSupplier))
                );
            }
        } else {
            System.out.println("WARNING: Alliance NOT DETECTED!");
        }
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(CMD_SmartDrive);
        
        drivetrain.registerTelemetry(logger::telemeterize);

        // new JoystickButton(test, 1).whileTrue(Commands.defer(DTP_CMD, Set.of(drivetrain)));
        // new JoystickButton(test, 2).whileTrue(CoralStation);
        // new JoystickButton(test, 3).onTrue(new InstantCommand(drivetrain::ResetPigeon));
        // new JoystickButton(test, 4).onTrue(new InstantCommand(arm::Arm_Algae));
        // new JoystickButton(test, 5).onTrue(CMD_Reef1);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateReefLevel() {
        for (int i = 1; i <= 4; i++) {
            if (BIG_BUTTON.getRawButton(i)) {
                reefLevel = 5 - i;
                System.out.printf("reefLevel is now %d\n", reefLevel);
            }
        }
    }

    /*
    private void Assist_ConfigureBindings() {
        // new JoystickButton(Assist_Ctrl, 2).whileTrue(new InstantCommand(climber::Up,climber))
        //                                                .onFalse(new InstantCommand(climber::Stop, climber));
        // new JoystickButton(Assist_Ctrl, 3).whileTrue(new InstantCommand(climber::Down, climber))
        //                                                .onFalse(new InstantCommand(climber::Stop, climber));
        new JoystickButton(Assist_Ctrl, 4).onTrue(CMD_SetZero);
        new JoystickButton(Assist_Ctrl, 5).onTrue(new InstantCommand(arm::Arm_Station, arm));
        new JoystickButton(Assist_Ctrl, 6).onTrue(new InstantCommand(drivetrain::ResetPigeon, drivetrain));
        new JoystickButton(Assist_Ctrl, 7).whileTrue(new InstantCommand(claw::L1ClawShoot, claw))
                                                       .onFalse(new InstantCommand(claw::Claw_Stop, claw));

        new POVButton(Assist_Ctrl, 0).whileTrue(new InstantCommand(elevator::ELE_Up, elevator))
                                           .onFalse(new InstantCommand(elevator::ELE_Stop, elevator));
        new POVButton(Assist_Ctrl, 180).whileTrue(new InstantCommand(elevator::ELE_Down, elevator))
                                             .onFalse(new InstantCommand(elevator::ELE_Stop, elevator));
        new POVButton(Assist_Ctrl, 90).whileTrue(new InstantCommand(arm::Arm_UP, arm))
                                            .onFalse(new InstantCommand(arm::Arm_Stop, arm));
        new POVButton(Assist_Ctrl, 270).whileTrue(new InstantCommand(arm::Arm_DOWN, arm))
                                             .onFalse(new InstantCommand(arm::Arm_Stop, arm));
    }
    */
}
