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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "#Main")

public class TeleOp1 extends LinearOpMode {
    private DcMotorEx liftBot, liftTop, leftRotate, rightRotate;
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        liftBot = hardwareMap.get(DcMotorEx.class, "liftBot");
        liftBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftBot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftTop = hardwareMap.get(DcMotorEx.class, "liftTop");
        liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        DualMotor pullArm;

      try {
            rotateArm = new DualMotor(rightRotate, leftRotate);
            pullArm = new DualMotor(liftBot, liftTop);
            rotateArm.setPower(0);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        int rotatePos = 0;
        int armPos = 0;
        boolean pressed = false;
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.7,
                            -gamepad1.left_stick_x * 0.7
                    ),
                    -gamepad1.right_stick_x * 0.55
            ));

            drive.updatePoseEstimate();
            //rotatePos = 760;
            //Lift
            if ((gamepad1.left_trigger > 0.1) && !pressed) {
                sleep(0);
                if (armPos< 74){
                    armPos++;
                }

            } else if ((gamepad1.right_trigger > 0.1) && !pressed) {
                sleep(0);
                if (armPos>0){
                    armPos--;
                }

            } else if (gamepad1.dpad_left && !pressed){
                //Pick up and get out
                pressed = true;
                rotatePos = 100;
                rotateArm.setTargetPosition(rotatePos);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(0.55);
                pullArm.setTargetPosition(-1720);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);

            }else if(gamepad1.dpad_up && !pressed){
                //Turn up
                rotatePos = 540;
                rotateArm.setTargetPosition(rotatePos);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(0.55);
                pullArm.setTargetPosition(0);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);

            } else if(gamepad1.dpad_down){
                //Turn down
                pressed = true;
                rotatePos = 0;
                rotateArm.setTargetPosition(rotatePos);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateArm.setPower(0.55);
                claw.setPosition(0.25);
                pullArm.setTargetPosition(0);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);



            }else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }

            try{
                rotateArm.setTargetPosition(rotatePos);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(rotatePos <= Math.abs(leftRotate.getCurrentPosition())){
                    rotateArm.setPower(0.2); //0.2
                } else{
                    rotateArm.setPower(0.45);//0.45
                }

           /*     pullArm.setTargetPosition(armPos*-23);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);*/
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
            if (gamepad1.a) {
                pullArm.setTargetPosition(-1710);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(0.55);
            }else if (gamepad1.y) {
                pullArm.setTargetPosition(0);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);
            }

            if(gamepad1.left_bumper){
                claw.setPosition(0);
            } else if(gamepad1.right_bumper){
                claw.setPosition(0.25);
            }

            telemetry.addData("Arm Pos", armPos);
            telemetry.addData("rotate pos", rotatePos);
            telemetry.addData("lift top position", liftTop.getCurrentPosition());
            telemetry.addData("lift bottom position", liftBot.getCurrentPosition());
            telemetry.addData("lift top target", liftTop.getTargetPosition());
            telemetry.addData("lift bottom target", liftBot.getTargetPosition());
            telemetry.addData("lift top current", liftTop.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lift bottom current", liftBot.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("left motor target", leftRotate.getTargetPosition());
            telemetry.addData("right motor target", rightRotate.getTargetPosition());
            telemetry.addData("left motor position", leftRotate.getCurrentPosition());
            telemetry.addData("right motor position", rightRotate.getCurrentPosition());


            try {
                telemetry.addData("lift", pullArm.getCurrentPosition());
            } catch(Exception e) {
                throw new RuntimeException(e);
            }

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

