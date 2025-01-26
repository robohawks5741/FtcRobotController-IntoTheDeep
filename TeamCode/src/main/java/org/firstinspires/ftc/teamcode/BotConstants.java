package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class BotConstants {
    public static final double horizontalTicks = 135;
    public static final double verticalTicks = 695;
    public static final double RADS_PER_TICK = Math.PI / 2 / (verticalTicks - horizontalTicks); //=0.002805
    public static final double RADS_PER_VOLT  = Math.PI * 2 / 3.22;
    public static final double VOLTS_PER_TICK = RADS_PER_TICK / RADS_PER_VOLT;
    public static final double HORIZONTAL_VOLTS = 0.858;
    public static final double CLAW_OPEN = 0;
    public static final double CLAW_CLOSED = 0.25;
    public static final double servoPosInit = 0.05;
    public static final double servoPosUp = 0.25;
    public static final double getServoPosPlaceRetracted = 0.4;
    public static final double servoPosPlaceExtended = 0.5;
    public static final double MAX_VOLTAGE = 3.227;
    public static final double ARM_VERTICAL_VOLTS = 0;
    public static final double ARM_FRONT_PLACING_VOLTS = mod(ARM_VERTICAL_VOLTS + 3.129, MAX_VOLTAGE);/*HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_FRONT_PLACING_TICKS - ARM_HORIZONTAL_TICKS);*/
    /*public static final double ARM_GROUND_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_GROUND_TICKS - ARM_HORIZONTAL_TICKS);*/
    public static final double ARM_GROUND_VOLTS_EXTENDED = mod(ARM_VERTICAL_VOLTS + .955, MAX_VOLTAGE);
    public static final double ARM_GROUND_VOLTS_RETRACTED = mod(ARM_VERTICAL_VOLTS + 1.085, MAX_VOLTAGE);
    public static final double ARM_UP_EXTENDABLE_VOLTS = mod(ARM_VERTICAL_VOLTS + 0.5, MAX_VOLTAGE);
    public static final double ARM_DOWN_EXTENDABLE_VOLTS = mod(ARM_VERTICAL_VOLTS + 0.3, MAX_VOLTAGE);

    public static double armUpKp = 0.0065;
    public static double armUpKi = 0.00005;
    public static double armUpKd = 0.0;
    public static final double armBasePower = 0.38;
    public static final double extendedIncrementalOutput = 16861;

    //placeholders--need to be set--voltages are measured from vertical position--some will be greater than
    //max voltage output because of the encoder making multiple full rotations

    //not too sure about these values
    public static final double LIFT_RETRACTED_VOLTS = 0.3;
    public static final double LIFT_ROTATABLE_VOLTS = 5.927 + LIFT_RETRACTED_VOLTS;
    public static final double LIFT_MIN_VOLTS = LIFT_RETRACTED_VOLTS - 0.3;


    public static final double LIFT_RETRACTED_SIDEWAYS_VOLTS = 2.7 + LIFT_RETRACTED_VOLTS;


    public static final double LIFT_MAX_VOLTS = LIFT_RETRACTED_VOLTS + 12.781;

    public static final double LIFT_EXTENDED_VOLTS = LIFT_RETRACTED_VOLTS + 12.781;

    public static final double INCREMENTAL_TO_VOLTS = (LIFT_MAX_VOLTS - LIFT_MIN_VOLTS) / extendedIncrementalOutput;

    public static double liftKp = 0.001;
    public static double liftKi = 0.0005;
    public static double liftKd = 0.00005;

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


    public static double armDownHangKp = 0.001; //0.0003
    public static double armDownHangKi = 0.0001;
    public static double armDownHangKd = 0.0001;
    public static final double frictionOffsetPower = 0.05;
    public static final int LIFT_RETRACTED_TICKS = 0;

    public static final int LIFT_SIDEWAYS_TICKS = -150;
    public static final int LIFT_ROTATABLE_TICKS = -544;
    public static final int LIFT_EXTENDED_TICKS = -2135;


    //outdated values
    public static final int ARM_HORIZONTAL_TICKS = 100;
    public static final int ARM_FRONT_PLACING_TICKS = 540;
    public static final int ARM_GROUND_TICKS = 0;
    //doing this because mod is weird with negative numbers in java
    public static double mod(double a, double b) {
        while(a < 0) {
            a += b;
        }
        while(a > b) {
            a -= b;
        }
        return a;
    }
}
