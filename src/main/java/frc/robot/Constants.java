// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MOTOR_IDS{
    public static class DRIVE{
      public static final int LEFT_CAN_ID = 1;
      public static final int LEFT_FOLLOWER_CAN_ID = 2;
      public static final int RIGHT_CAN_ID = 3;
      public static final int RIGHT_FOLLOWER_CAN_ID = 4;
    }
    public static class INTAKE{
      public static final int INTAKE_CAN_ID = 5;
    }
    public static class ARM{
      public static final int ARMRIGHT_CAN_ID = 6;
      public static final int ARMLEFT_FOLLOWER_CAN_ID = 7;
    }
    public static class SHOOTER{
      public static final int SHOOTER_CAN_ID = 8;
      public static final int SHOOTER_FOLLOWER_CAN_ID = 9;
    }
  }

  public static class SHOOTER_SPEEDS{
    public static final double subWooferSpeed = 0; //change as needed
    public static final double midSpeed = 0; //change as needed
    public static final double podiumSpeed = 0; //change as needed
    public static final double ampSpeed = 0; //change as needed 
  }

  public static class ARM_POSITIONS{
    public static final double subWooferPosition = 0; //change as needed
    public static final double midPosition = 0; //change as needed
    public static final double podiumPosition = 0; //change as needed
    public static final double ampPosition = 0; //change as needed 
    public static final double intakePosition = 0; //change as needed
    public static final double safePosition = 0; //change as needed
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double controllerDeadzone = 0.1; //change as needed
  }
}
