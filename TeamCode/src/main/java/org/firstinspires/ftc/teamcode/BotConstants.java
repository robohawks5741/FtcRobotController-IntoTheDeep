package org.firstinspires.ftc.teamcode;

public final class BotConstants {
    public static final double horizontalTicks = 135;
    public static final double verticalTicks = 695;
    public static final double RADS_PER_TICK = Math.PI / 2 / (verticalTicks - horizontalTicks); //=0.002805
    public static final double RADS_PER_VOLT  = Math.PI * 2 / 3.22;
    public static final double VOLTS_PER_TICK = RADS_PER_TICK / RADS_PER_VOLT;
    public static final double HORIZONTAL_VOLTS = 0.23;
    public static final double CLAW_OPEN = 0.25;
    public static final double CLAW_CLOSED = 0;



    public static final double ARM_FRONT_PLACING_VOLTS = 2.62;/*HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_FRONT_PLACING_TICKS - ARM_HORIZONTAL_TICKS);*/
    /*public static final double ARM_GROUND_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_GROUND_TICKS - ARM_HORIZONTAL_TICKS);*/
    public static final double ARM_GROUND_VOLTS = 0.317;
    //TODO: retune these
    public static final double armUpKp = 0.001;
    public static final double armUpKi = 0.0001;
    public static final double armUpKd = 0.00001;
    public static final double armBasePower = 0.4;

    //placeholders--need to be set--voltages are measured from horizontal position--some will be greater than
    //max voltage output because of the encoder making multiple full rotations

    public static final double LIFT_ROTATABLE_VOLTS = 0.18;
    public static final double LIFT_EXTENDED_VOLTS = 3.064;
    public static final double LIFT_RETRACTED_VOLTS = 2.415;

    public static final double liftKp = 0;
    public static final double liftKi = 0;
    public static final double liftKd = 0;
    //this also needs to be set, and I'm not entirely sure if it works the same way as the arm one
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
