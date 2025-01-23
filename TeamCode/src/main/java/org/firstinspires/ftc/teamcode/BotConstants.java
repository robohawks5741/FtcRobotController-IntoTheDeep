package org.firstinspires.ftc.teamcode;

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


    public static final double ARM_FRONT_PLACING_VOLTS = 0;/*HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_FRONT_PLACING_TICKS - ARM_HORIZONTAL_TICKS);*/
    /*public static final double ARM_GROUND_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_GROUND_TICKS - ARM_HORIZONTAL_TICKS);*/
    public static final double ARM_GROUND_VOLTS_EXTENDED = 0.955;
    public static final double ARM_GROUND_VOLTS_RETRACTED = 1.085;
    public static final double ARM_UP_EXTENDABLE_VOLTS = 0.5;
    public static final double ARM_DOWN_EXTENDABLE_VOLTS = 0.3;
    public static final double MAX_VOLTAGE = 3.227;
    public static final double armUpKp = 0.004;
    public static final double armUpKi = 0.00005;
    public static final double armUpKd = 0.0008;
    public static final double armBasePower = 0.38;
    public static final double extendedIncrementalOutput = 16861;

    //placeholders--need to be set--voltages are measured from vertical position--some will be greater than
    //max voltage output because of the encoder making multiple full rotations

    //not too sure about these values
    public static final double LIFT_ROTATABLE_VOLTS = 3 + MAX_VOLTAGE;
    public static final double LIFT_MIN_VOLTS = 0;
    public static final double LIFT_RETRACTED_VOLTS = 0.3;
    public static final double LIFT_MAX_VOLTS = 0.173 + 4 * MAX_VOLTAGE;
    public static final double LIFT_EXTENDED_VOLTS = 0.173 + 4 * MAX_VOLTAGE;

    public static final double INCREMENTAL_TO_VOLTS = (LIFT_MAX_VOLTS - LIFT_MIN_VOLTS) / extendedIncrementalOutput;

    public static final double liftKp = 0.00055;
    public static final double liftKi = 0.000015;
    public static final double liftKd = 0.00005;
    //this also needs to be set, and I'm not entirely sure if it works the same way as the arm one
    //after some testing, I'm not convinced that it's necessary at all
    public static double liftBasePower = 0;


    //not currently in use
    public static final double armDownKp = 0.0003;
    public static final double armDownKi = 0.0001;
    public static final double armDownKd = 0;
    public static final double frictionOffsetPower = 0.05;
    public static final int LIFT_RETRACTED_TICKS = 0;
    public static final int LIFT_ROTATABLE_TICKS = -544;
    public static final int LIFT_EXTENDED_TICKS = -2135;


    //outdated values
    public static final int ARM_HORIZONTAL_TICKS = 100;
    public static final int ARM_FRONT_PLACING_TICKS = 540;
    public static final int ARM_GROUND_TICKS = 0;

}
