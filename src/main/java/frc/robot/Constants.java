package frc.robot;

public class Constants {
    public static final int pdp_id = 2;
    public static final int pcm_id = 11;

    // CAN ids for the drive motors - total 4 brushless motors
    public static final int drivemotor_left1_id = 7;
    public static final int drivemotor_left2_id = 6;
    public static final int drivemotor_right1_id = 9;
    public static final int drivemotor_right2_id = 10;
    // Solenoid channels for drive shifters
    public static final int driveshifter_1_solenoid = 6;
    public static final int driveshifter_2_solenoid = 9;

    // CAN ids for the intake - total 1 roller motor - brushed motors (talons)
    public static final int intakemotor_roller_id = 8;
    // Solenoid channel for intake lifter
    public static final int intake_1_solenoid = 7;
    public static final int intake_2_solenoid = 8;
    public static final int intake_3_solenoid = 10;
    public static final int intake_4_solenoid = 5;

    // CAN id for ball feed motor - total 1 motor - brushed motor (talons)
    public static final int ballfeedmotor_id = 17;
    // digital IO - switch for ball position / ball present
    public static final int ballfeed_dio_limit_sw = 0;

    // CAN ids shooter mech - total 3 motors - brushed motors (talons) - limit switches available
    public static final int shootermotor_tilt_id = 16;
    public static final int shootermotor_wheel_bottom_id = 15;
    public static final int shootermotor_wheel_top_id = 18;

    // CAN ids climber mech - total 6 motors - brushless motors
    public static final int climbmotor_left_pull_1_id = 3;
    public static final int climbmotor_left_pull_2_id = 4;
    public static final int climbmotor_right_pull_1_id = 12;
    public static final int climbmotor_right_pull_2_id = 13;
    public static final int climbmotor_left_tilt_id = 5;
    public static final int climbmotor_right_tilt_id = 14;

    

    //Height is in feet.
    //8'8"
    public static final double GoalHeight = 104.0/12.0;
    //1'10"
    public static final double ShooterHeight = 22.0/12.0;

    //Imperial/metric conversion constants.


    //public Constants(){}
    
}