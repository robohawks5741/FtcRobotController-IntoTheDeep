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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "#Main")

public class TeleOp1 extends LinearOpMode {
    private DcMotorEx lift, leftRotate, rightRotate;
    private Servo claw;

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

        claw = hardwareMap.get(Servo.class, "claw");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        DualMotor rotateArm;

      try {
            rotateArm = new DualMotor(rightRotate, leftRotate);
            rotateArm.setPower(0);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        int rotatePos = 0;
        boolean pressed = false;
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5
                    ),
                    -gamepad1.right_stick_x * 0.35
            ));

            drive.updatePoseEstimate();

            //Lift
            if ((gamepad1.left_trigger > 0.1) && !pressed) {
                pressed = true;
                rotatePos = 540;

            } else if ((gamepad1.right_trigger > 0.1) && !pressed) {
                pressed = true;
                rotatePos = 0;

            } else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }

            try{
                rotateArm.setTargetPosition(rotatePos);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(rotatePos <= Math.abs(leftRotate.getCurrentPosition())){
                    rotateArm.setPower(0.2);
                } else{
                    rotateArm.setPower(0.5);
                }
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
         /*   if (gamepad1.y) {
                lift.setTargetPosition(-1720);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }else if (gamepad1.a) {
                lift.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }*/

            if(gamepad1.left_bumper){
                claw.setPosition(0);
            } else if(gamepad1.right_bumper){
                claw.setPosition(0.25);
            }

                telemetry.addData("rotate pos", rotatePos);
                telemetry.addData("left motor", leftRotate.getCurrentPosition());
                telemetry.addData("right motor", rightRotate.getCurrentPosition());


                telemetry.addData("left motor target", leftRotate.getTargetPosition());
                telemetry.addData("right motor target", rightRotate.getTargetPosition());


                telemetry.addData("lift", lift.getCurrentPosition());

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

