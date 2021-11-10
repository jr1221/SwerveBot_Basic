package frc.robot;

// Constants file, static final variables
public class Constants {
    
    // -- Swerve Drive Ports --

    // Drive Motor:
    public static final int kFront_Left_Drive_Port = 1;
    public static final int kFront_Right_Drive_Port = 3;
    public static final int kBack_Left_Drive_Port = 5;
    public static final int kBack_Right_Drive_Port = 7;

    // Drive Encoders (dummy values):
    public static final int kFront_Left_Drive_Encoder_Port_A = 0;
    public static final int kFront_Left_Drive_Encoder_Port_B = 2;

    public static final int kFront_Right_Drive_Encoder_Port_A = 4;
    public static final int kFront_Right_Drive_Encoder_Port_B = 4;

    public static final int kBack_Left_Drive_Encoder_Port_A = 6;
    public static final int kBack_Left_Drive_Encoder_Port_B = 6;

    public static final int kBack_Right_Drive_Encoder_Port_A = 8;
    public static final int kBack_Right_Drive_Encoder_Port_B = 8;
    
    // Turning Motor:
    public static final int kFront_Left_Turn_Port = 2;
    public static final int kFront_Right_Turn_Port = 4;
    public static final int kBack_Left_Turn_Port = 6;
    public static final int kBack_Right_Turn_Port = 8;

    // Turning Encoders:
    public static final int kFront_Left_Turn_Encoder_Port_A = 0;
    public static final int kFront_Left_Turn_Encoder_Port_B = 2;

    public static final int kFront_Right_Turn_Encoder_Port_A = 4;
    public static final int kFront_Right_Turn_Encoder_Port_B = 4;

    public static final int kBack_Left_Turn_Encoder_Port_A = 6;
    public static final int kBack_Left_Turn_Encoder_Port_B = 6;

    public static final int kBack_Right_Turn_Encoder_Port_A = 8;
    public static final int kBack_Right_Turn_Encoder_Port_B = 8;


    // -- Software Constants --

    public static final double kMax_Speed = 3.0; //  3 meters per second
    public static final double kMax_Angular_Speed = Math.PI; // 1/2 rotation per second
    public static final double kJoystick_Rate_Limit = 3;
    public static final double kWheel_Radius = 0.0508;
    public static final int kEncoderResolution = 4096;


    // -- Hardware Ports -- 

    public static final int kGyro_Port = 0;


}
