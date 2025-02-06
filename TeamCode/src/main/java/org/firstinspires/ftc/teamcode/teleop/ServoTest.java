package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotConstants;

public class ServoTest extends LinearOpMode {
    protected Servo clawIntake;
    protected Servo clawRotate;
    @Override
    public void runOpMode() {
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_bumper){
                clawIntake.setPosition(BotConstants.SERVO_TEST_POS);
            } else if(gamepad1.right_bumper){
                clawIntake.setPosition(0);
            }
            if(gamepad1.x) {
                clawRotate.setPosition(BotConstants.SERVO_TEST_POS);
            }
            else if(gamepad1.b) {
                clawRotate.setPosition(0);
            }
        }
    }
}
