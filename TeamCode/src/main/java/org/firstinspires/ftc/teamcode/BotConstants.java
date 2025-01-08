package org.firstinspires.ftc.teamcode;

public final class BotConstants {
    public static final double horizontalTicks = 135;
    public static final double verticalTicks = 695;
    public static final double RADS_PER_TICK = Math.PI / 2 / (verticalTicks - horizontalTicks); //=0.002805
    public static final double RADS_PER_VOLT  = Math.PI * 2 / 3.22;
    public static final double VOLTS_PER_TICK = RADS_PER_TICK / RADS_PER_VOLT;
    public static final double HORIZONTAL_VOLTS = 1.455;
    public static final int ARM_HORIZONTAL_TICKS = 100;
    public static final int ARM_FRONT_PLACING_TICKS = 540;
    public static final int ARM_GROUND_TICKS = 0;
    public static final double ARM_FRONT_PLACING_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_FRONT_PLACING_TICKS - ARM_HORIZONTAL_TICKS);
    public static final double ARM_GROUND_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_GROUND_TICKS - ARM_HORIZONTAL_TICKS);
    public static final double armUpKp = 0.0014;
    public static final double armUpKi = 0.0001;
    public static final double armUpKd = 0;
    public static final double armDownKp = 0.0003;
    public static final double armDownKi = 0.0001;
    public static final double armDownKd = 0;
    public static final double armBasePower = 0.2;
    public static final double frictionOffsetPower = 0.05;
    public static final int LIFT_RETRACTED_TICKS = 1720;
    //placeholder value
    public static final int LIFT_ROTATABLE = 1000;
    public static final int LIFT_EXTENDED_TICKS = 0;
    public static final double CLAW_OPEN = 0.25;
    public static final double CLAW_CLOSED = 0;
}
