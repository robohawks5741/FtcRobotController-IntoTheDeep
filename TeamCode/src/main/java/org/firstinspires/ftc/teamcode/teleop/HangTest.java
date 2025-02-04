package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

@TeleOp
public class HangTest extends LinearOpMode {
    private DcMotorEx frontRotate, backRotate, frontLift, backLift;
    private DualMotor rotate;
    @Override
    public void runOpMode() {
        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "backRotate");
        try {
            rotate = new DualMotor(backRotate, frontRotate,
                    BotConstants.armUpKp / BotConstants.VOLTS_PER_TICK,
                    BotConstants.armUpKi / BotConstants.VOLTS_PER_TICK,
                    BotConstants.armUpKd / BotConstants.VOLTS_PER_TICK);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
        while(opModeIsActive()) {
            rotate.setPower(-1);
        }
    }
}
