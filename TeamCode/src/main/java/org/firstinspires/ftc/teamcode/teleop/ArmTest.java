package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp
public class ArmTest extends LinearOpMode {
    private DcMotorEx lift, leftRotate, rightRotate;
    private Servo rotate, left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRotate = hardwareMap.get(DcMotorEx.class, "leftRotate");
        leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightRotate = hardwareMap.get(DcMotorEx.class, "rightRotate");
        rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean downPosition = true;
        while (opModeIsActive()) {
            leftRotate.setPower(0);
            rightRotate.setPower(0);
            lift.setPower(0);
            if(gamepad1.right_stick_y > 0.1 && leftRotate.getCurrentPosition() > -880) {
                leftRotate.setTargetPosition(-880);
                leftRotate.setPower(0.1);
                rightRotate.setTargetPosition(-880);
                rightRotate.setPower(0.1);
                if(downPosition) {
                    lift.setTargetPosition(100);
                    lift.setPower(0.1);
                }
            }
            else if(gamepad1.right_stick_y < -0.1 && leftRotate.getCurrentPosition() < 0) {
                leftRotate.setTargetPosition(0);
                leftRotate.setPower(0.1);
                rightRotate.setTargetPosition(0);
                rightRotate.setPower(0.1);
                if(downPosition) {
                    lift.setTargetPosition(0);
                    lift.setPower(0.1);
                }
            }
            if(gamepad1.dpad_down) {
                downPosition = true;
            }
            if(gamepad1.dpad_up) {
                downPosition = false;
            }
        }
    }
}