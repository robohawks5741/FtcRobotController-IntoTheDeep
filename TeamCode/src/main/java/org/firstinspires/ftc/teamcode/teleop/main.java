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

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "#Main")

public class main extends LinearOpMode {

    private DcMotorEx lift;
    private Servo rotate, left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotate = hardwareMap.get(Servo.class, "rotate");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        left.setPosition(0.0);
        right.setPosition(0.36);
        rotate.setPosition(0.5);
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            //Lift
            if(gamepad1.left_trigger > 0.1){
                rotate.setPosition(0);
                lift.setTargetPosition(2000);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            } else if (gamepad1.right_trigger > 0.1){
                rotate.setPosition(0.5);
                lift.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }

            //Claw
            if(gamepad1.left_bumper){
                //Open
                left.setPosition(0.0);
                right.setPosition(0.36);
            } else if (gamepad1.right_bumper){
                //Close
                left.setPosition(0.29);
                right.setPosition(0.07);
            }



            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("right", right.getPosition());
            telemetry.addData("left", left.getPosition());
            telemetry.addData("Rotate", rotate.getPosition());
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }

}
