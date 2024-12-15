package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "#Main")

public class TeleOp1 extends LinearOpMode {
    private DcMotorEx liftBot, liftTop, leftRotate, rightRotate;
    private Servo claw;
    private AnalogInput potentiometer;
    private AnalogInput encoder;
    private final int ARM_HORIZONTAL_TICKS = 100;
    private final int ARM_FRONT_PLACING_TICKS = 540;
    private final int ARM_GROUND_TICKS = 0;
    private final int LIFT_RETRACTED_TICKS = -1720;
    //placeholder value
    private final int LIFT_ROTATABLE = 1000;
    private final int LIFT_EXTENDED_TICKS = 0;
    private final double CLAW_OPEN = 0.25;
    private final double CLAW_CLOSED = 0;
    private int rotatePos;
    private int liftPos;
    int rotateStage = 0;
    DualMotor rotateArm;
    DualMotor pullArm;
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
        encoder = hardwareMap.analogInput.get("encoder");
        //potentiometer = hardwareMap.analogInput.get("potentiometer");


      try {
            rotateArm = new DualMotor(rightRotate, leftRotate);
            pullArm = new DualMotor(liftBot, liftTop);
            rotateArm.setPower(0);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        rotatePos = 0;
        liftPos = 0;
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
                if (armPos < 74){
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
                rotateStage = 0;
                rotatePos = ARM_HORIZONTAL_TICKS;
                liftPos = LIFT_RETRACTED_TICKS;

            } else if(gamepad1.dpad_up && !pressed){
                //Turn up
                if(rotateStage == 0) {
                    rotateStage = 1;
                }

            } else if(gamepad1.dpad_down){
                //Turn down
                pressed = true;
                rotateStage = 0;
                rotatePos = ARM_GROUND_TICKS;
                //open/close claw
                openClaw();
                liftPos = LIFT_EXTENDED_TICKS;

            }else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }

            if (gamepad1.a) {
                liftPos = LIFT_RETRACTED_TICKS;
            }else if (gamepad1.y) {
                liftPos = LIFT_EXTENDED_TICKS;
            }
            if(rotateStage == 1) {
                liftPos = LIFT_RETRACTED_TICKS;
                try {
                    if (Math.abs(pullArm.getCurrentPosition()) > LIFT_ROTATABLE) {
                        rotateStage++;
                    }
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }
            try {
                if (rotateStage == 2) {
                    rotatePos = ARM_FRONT_PLACING_TICKS;
                    liftPos = LIFT_EXTENDED_TICKS;
                }
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
                //Update arm pos
            try {
                updateArm();
                updateLift();

           /*     pullArm.setTargetPosition(armPos*-23);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);*/
            } catch(Exception e) {
                throw new RuntimeException(e);
            }


            if(gamepad1.left_bumper){
                closeClaw();
            } else if(gamepad1.right_bumper){
                openClaw();
            }

            telemetry.addData("Arm Pos", armPos);
            telemetry.addData("rotate pos", rotatePos);
            telemetry.addData("lift top position", liftTop.getCurrentPosition());
            telemetry.addData("lift bottom position", liftBot.getCurrentPosition());
            telemetry.addData("lift top target", liftTop.getTargetPosition());
            telemetry.addData("lift bottom target", liftBot.getTargetPosition());
            telemetry.addData("lift top current", liftTop.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lift bottom current", liftBot.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rotate stage", rotateStage);
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
            telemetry.addData("encoder position(deg)", encoder.getVoltage() * 360 / 3.2);
/*
            telemetry.addData("potentiometer max output", potentiometer.getMaxVoltage());
            telemetry.addData("potentiometer output", potentiometer.getVoltage());
            double v = potentiometer.getVoltage();
            telemetry.addData("angle(simple calc)", v * 81.8);
            //I think this one is right:
            telemetry.addData("angle(complicated calc with -)", (270 * v + 445.5 - Math.sqrt(Math.pow(270 * v +
                        v + 445.5, 2) - 4 * v * (-36450 * v + 120285))) / (2 * v));
            //but im trying this one too
            telemetry.addData("angle(complicated calc with +)", (270 * v + 445.5 + Math.sqrt(Math.pow(270 * v +
                        v + 445.5, 2) - 4 * v * (-36450 * v + 120285))) / (2 * v));
*/
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    public void updateArm() {
        rotateArm.setTargetPosition(rotatePos);
        rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(rotatePos <= Math.abs(leftRotate.getCurrentPosition())){
            rotateArm.setPower(0.2); //0.2
        } else{
            rotateArm.setPower(0.45);//0.45
        }
    }
    public void updateLift() {
        pullArm.setTargetPosition(liftPos);
        pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pullArm.setPower(1);
    }
    public void openClaw() {
        claw.setPosition(CLAW_OPEN);
    }
    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED);
    }
}

