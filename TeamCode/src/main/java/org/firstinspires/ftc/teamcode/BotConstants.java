package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class BotConstants {

    public static final double ARM_BACK_VOLTS = 0.0;
    public static final double horizontalTicks = 135;
    public static final double verticalTicks = 695;
    public static final double RADS_PER_TICK = Math.PI / 2 / (verticalTicks - horizontalTicks); //=0.002805
    public static final double RADS_PER_VOLT  = Math.PI * 2 / 3.22;
    public static final double VOLTS_PER_TICK = RADS_PER_TICK / RADS_PER_VOLT;
    public static final double HORIZONTAL_VOLTS = 0.858;


    public static final double MAX_VOLTAGE = 3.227;



    public static final double ARM_FRONT_PLACING_VOLTS = (0.03 + ARM_BACK_VOLTS) % MAX_VOLTAGE;/*HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_FRONT_PLACING_TICKS - ARM_HORIZONTAL_TICKS);*/
    /*public static final double ARM_GROUND_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_GROUND_TICKS - ARM_HORIZONTAL_TICKS);*/
    public static double ARM_GROUND_VOLTS_EXTENDED = (0.92 + ARM_BACK_VOLTS) % MAX_VOLTAGE;
    public static final double ARM_GROUND_VOLTS_RETRACTED = (1.098 + ARM_BACK_VOLTS) % MAX_VOLTAGE;
    public static final double ARM_UP_EXTENDABLE_VOLTS = (0.513 + ARM_BACK_VOLTS) % MAX_VOLTAGE;
    public static final double ARM_DOWN_EXTENDABLE_VOLTS = (0.313 + ARM_BACK_VOLTS) % MAX_VOLTAGE;

    public static final double ARM_VERTICAL_VOLTS = ARM_FRONT_PLACING_VOLTS;
    public static final double ROTATE_SHORT_DOWN_VOLTS = (ARM_BACK_VOLTS + 0.995) % MAX_VOLTAGE;
    //TODO
    public static final double ARM_SPECIMEN_PLACEMENT_POSITION = 0;

    public  static final double ARM_SPECIMEN_PICKUP_POSITION = 0;

    public static double armUpKp = 0.002;
    public static double armUpKi = 0.0002;
    public static double armUpKd = 0.0002;
    public static double armBasePower = 0.1;
    public static final double extendedIncrementalOutput = 16861;

    //placeholders--need to be set--voltages are measured from vertical position--some will be greater than
    //max voltage output because of the encoder making multiple full rotations

    //not too sure about these values
    public static final double LIFT_MIN_VOLTS = 1.936;
    public static final double LIFT_ROTATABLE_VOLTS = LIFT_MIN_VOLTS + 3 + MAX_VOLTAGE;

    public static final double LIFT_RETRACTED_VOLTS = LIFT_MIN_VOLTS + .3;

    public static final double LIFT_SPECIMEN_PLACEMENT_POSITION = 0;


    public static double LIFT_RETRACTED_SIDEWAYS_VOLTS = LIFT_MIN_VOLTS + 3.5;

    public static double LIFT_LOW_BUCKET = LIFT_MIN_VOLTS + 2.313;

    public static final double LIFT_MAX_VOLTS = LIFT_MIN_VOLTS + 0.173 + 4 * MAX_VOLTAGE;
    public static final double LIFT_EXTENDED_VOLTS = LIFT_MIN_VOLTS + 0.0 + 4 * MAX_VOLTAGE;
    public static final double LIFT_SHORT_DOWN_VOLTS = LIFT_MIN_VOLTS + 0.239 + MAX_VOLTAGE;
    public static final double INCREMENTAL_TO_VOLTS = (LIFT_MAX_VOLTS - LIFT_MIN_VOLTS) / extendedIncrementalOutput;

    public static double liftKp = 0.001;
    public static double liftKi = 0.00005;
    public static double liftKd = 0.0001;

    //public double kp = 0.00025;
    //public double ki = 0.000015;
    //public double kd = 0.00005;
    //this also needs to be set, and I'm not entirely sure if it works the same way as the arm one
    //after some testing, I'm not convinced that it's necessary at all
    public static double liftBasePower = 0;


    //not currently in use
    public static double armDownKp = 0.0006; //0.0003
    public static double armDownKi = 0.0005;
    public static double armDownKd = 0;


    public static final double frictionOffsetPower = 0.05;
    public static final int LIFT_RETRACTED_TICKS = 0;

    public static final int LIFT_EXTENDED_TICKS = -2135;


    //outdated values
    public static final int ARM_HORIZONTAL_TICKS = 100;
    public static final int ARM_FRONT_PLACING_TICKS = 540;
    public static final int ARM_GROUND_TICKS = 0;

    //Wrist Servo -----------------------------------------------------
    public static double SERVO_TEST_POS = 0;

    public static final double SERVO_PICKUP_POS = 0;

    public static final double SERVO_INIT_POS = 1-0.8;
    public static final double SERVO_PLACEMENT_POS = 1-0.5;

    public static final double SERVO_PARALLEL_POS = 1-0.4;
    public static final double SERVO_DOWN_POS = 0.96;

    public static final double SERVO_SPECIMEN_PLACEMENT_POS = 0;

    //Claw-
    public static final double CLAW_OPEN = 0.25;
    public static final double CLAW_CLOSED = 0;
}
