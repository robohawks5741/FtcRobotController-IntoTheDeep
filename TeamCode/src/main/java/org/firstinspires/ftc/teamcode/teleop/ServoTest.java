package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotConstants;

@TeleOp
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
                clawIntake.setPosition(BotConstants.SERVO_TEST_LEFT);
            } else if(gamepad1.right_bumper){
                clawIntake.setPosition(BotConstants.SERVO_TEST_RIGHT);
            } else if(gamepad1.dpad_up) {
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
