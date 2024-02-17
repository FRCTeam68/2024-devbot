/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class LED {
        public static final int PWMPORT = 0;
        public static final int BUFFERSIZE = 120;
    }
    
    public static final class INTAKE {
        public static final int CANID = 20;
        public static final double SPEED = 1.0;
    }

    public static final class SHOOTER {
        public static final int LEFT_CANID = 30;
        public static final int RIGHT_CANID = 31;
        public static final double SPEED = 20;
        public static final double RIGHT_OFFSET = 0;
    }

    public static final class ANGLE {
        public static final int LEFT_CANID = 32;
        public static final int RIGHT_CANID = 33;
        public static final double SPEAKER = 0;
        public static final double AMP = 2;
        public static final double TRAP = 4;
        public static final double INTAKE = -2;
        // public static final double VISION = 0;
    }

    public static final class CLIMBER {
        public static final int LEFT_CANID = 40;
        public static final int RIGHT_CANID = 41;
        public static final double MAX_HEIGHT = 9;   //inches
    }

    // public static final class CAMERA {
    //     public static final double BALLCAMERAANGLE = 0; // Degrees
    //     public static final double SHOOTERCAMERAANGLE = 0; // Degrees
    //     public static final double BALLCAMERAHEIGHT = .8; // Meters
    //     public static final double SHOOTERCAMERAHEIGHT = .8; // Meters
    //     //public static final double BALLTARGETHEIGHT = Units.inchesToMeters(9.5); // Height to top of the ball
    //     // public static final double kMinimumRange = 4; // Meters
    //     // public static final double kMaximumRange = -9; // Meters
    //     public static final int LIMELIGHTPIPELINE = 0;
    //     public static final int HD3000PIPELINE = 0;
    // }

    // public static final class SHOOTER {
    //     public static final int MOTORPORT = 14;
    //     public static final int MOTOR2PORT = 15;
    //     public static final int HOODPORT = 16;
    
    //     public static final double HOODkP = 0.01;
    //     public static final int HOODCIRCLE = -55500;
    //     public static final int HOODLOW = -30000;
    //     public static final int HOODDEFENSE = -65500;

    //     public static final double TOLERANCERPS = 6.0;
        
    //     public static final double WHEELDIAMETERINCHES = 4;
    //     public static final int ENCODERCPR = 2048;
    //     // Multiply by 10 to get Raw per second.  Divide by encoder CPR to get rotations
    //     public static final double RAWTOFLYWHEELRPS = 10 / (double) ENCODERCPR;

    //     // Multiply by encoder CPR to get raw counts per second.  Divide by 10 to get per decisecond
    //     public static final double RPSTORAW = (double) ENCODERCPR / 10;

    //     public static final double LOWFF = 0.24;
    //     public static final double FENDERFF = 0.4; //.45 for low energy shot
    //     public static final double CIRCLEFF = 0.45;//.5
    //     public static final double LAUNCHFF = 0.55;
    //     public static final double DEFENDFF = 0.36;
    //     public static final double P = 0;//999999999999999.0;
    //     public static final double D = 0;
    
    //     // On a real robot the feedforward constants should be empirically determined; these are
    //     // reasonable guesses.
    //     public static final double kSVOLTS = 0.73856;
    //     public static final double kVVOLTSECONDSPERROTATION = 0.11106;
    //     public static final double kA = 0.0028227;

    //     public static final double SPEEDCHANGE = 0.01;
    //     public static final double SETPOINT2 = 2.0;
    //     public static final double SETPOINT4 = 4.0;
    // }
}
