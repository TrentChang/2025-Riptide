// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{

    // public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    // public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(26)), ROBOT_MASS);
    // public static final double LOOP_TIME  = 0.02; //s, 20ms + 110ms sprk max velocity lag
    // public static final double MAX_SPEED  = Units.feetToMeters(16.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants{
      //  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
      //  public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants{
      // Hold time on motor brakes when disabled
      public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants{
    
    // Joystick Deadband
    public static final double DEADBAND        = 10000;
    public static final double LEFT_Y_DEADBAND = 10000;
    public static final double RIGHT_X_DEADBAND = 10000;

    
    public static final double TURN_CONSTANT    = 6;
  }

  // Auto Constants
  public static class AutoConstants{
    
      // Auto Drive PIDF
      public static class AutoDrivePIDF{
          public static final double P = 3; //4.5;
          public static final double I = 0.00;
          public static final double D = 0;
          public static final double I_ZONE = 0;
      }
    
      // Auto Turn PIDF
      public static class AutoTurnPIDF{
          public static final double P = 8;  //13;
          public static final double I = 0.0;
          public static final double D = 0.00;
          public static final double I_ZONE = 0.0;
      }
  }
  
//   public static class LimelightConstants{
//       public static final String LL1 = "limelight";
//       public static final String LL2 = "limelight-two";
//   }

  // Subsystems Constants.
    public static class AlgaeConstants{
        // Algae ID
        public static final int Algae_Ctrl_ID = 21;
        public static final int Algae_Roller_ID = 22;

        // Algae Config
        public static final boolean Algae_ctrl_Inverted = true;
        public static final boolean Algae_Roller_Inverted = false;

        public static final double Algae_Zero = 0;
        public static final double Algae_Out = -2.3;
        public static final double Algae_In = 0;

        public static final double MAX_ACCEL = 500;
        public static final double MAX_VELOCITY = 200;
        
        // Algae PIDF
        public static final double Algae_Out_P = 0.4;
        public static final double Algae_Out_I = 0;
        public static final double Algae_Out_D = 0;
        public static final double Algae_Out_F = 0;      
        
        public static final double Algae_Back_P = 10;
        public static final double Algae_Back_I = 0;
        public static final double Algae_Back_D = 0;
        public static final double Algae_Back_F = 0;        
        
    }

    public static class ArmConstants{
        // Arm ID
        public static final int Arm_ID = 31;
        
        public static final int Arm_Encoder_ID = 33;

        // Arm Config 
        public static final boolean Arm_Inverted = false;

        public static final double Arm_Zero = 0.1;
        public static final double Arm_StartUp = 0.1;
        public static final double Arm_Station = 0.35;
        public static final double Arm_Barge = 0.26;
        public static final double Arm_Algae = 0.27;
        public static final double Arm_RL1 = 0.17;
        public static final double Arm_RL2 = 0.19;
        public static final double Arm_RL3 = 0.19;
        public static final double Arm_RL4 = 0.2;

        public static final double MAX_ACCEL = 500;
        public static final double MAX_VELOCITY = 200;
        
        // Arm UP PIDF
        public static final double UP_Arm_P = 3.2;
        public static final double UP_Arm_I = 0.5;
        public static final double UP_Arm_D = 0;
        public static final double UP_Arm_F = -0.01;   
        
        // Arm DOWN PIDF
        public static final double DOWN_Arm_P = 3.3;
        public static final double DOWN_Arm_I = 0.2;
        public static final double DOWN_Arm_D = 0;
        public static final double DOWN_Arm_F = 0;   
    }

    // Climber Constants
    public static class ClimberConstants {
        // Climber ID
        public static final int Climb_Motor = 41;

   
        // Climber Config
        public static final boolean LeftMotor_Inverted = false;
        public static final boolean RightMotor_Inverted = false;
        public static final double Climb_Angle = -182;
        public static final double Climb_Zero = 0;
        public static final double Climb_StartUp = -24;

        // Climber PIDF
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;

        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;
    }

    // Elevator Constants
    public static class ElevatorConstants{
        // Elevator ID
        public static final int LeftMotor_ID = 51;
        public static final int RightMotor_ID = 52;

        public static final int Encoder_ID = 53;

        // Elevator Config
        public static final boolean LeftMotor_Inverted = true;
        public static final boolean RightMotor_Inverted = false;

        public static final double floor = -0.5;
        public static final double L1 = -0.5;
        public static final double L2 = -9;
        public static final double L3 = -28;
        public static final double L4 = -58;


        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;

        // Elevator PIDF
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;
    }

    public static class CoralConstants{
        // Coral ID
        public static final int Coral_Motor_ID = 61;

        public static final int Coral_Sensor_ID = 0;

        // Coral Config
        public static final boolean Coral_Inverted = true;
        public static final boolean Arm_Left_Inverted = false;
        public static final boolean Arm_Right_Inverted = false;

        public static final double MAX_ACCEL = 500;
        public static final double MAX_VELOCITY = 200;

        // Intake
        public static final double Coral_Open = 0;
        public static final double Coral_Close = 0;
        // Arm
        public static final double Arm_Station = 0;
        public static final double Arm_Reef = 0;
    }
}
