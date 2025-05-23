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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.ServoAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.SetArmPos;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.firstinspires.ftc.teamcode.subsystems.TagConstants;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "##MAIN")
public class Main extends Robot {


    /*
    * Gamepad 1:
    *   Joysticks: Drive
    *   Left Bumper: Close Claw
    *   Right Bumper: Open Claw
    *   Y: Get in Hang position
    *   A: Pull hang down
    *   Right Trigger: Place or pickup in current position and get to neutral position
    *   Left Trigger: Get to High Placement Position
    *   Dpad_Up: Go to low bucket position
    *   Dpad_Left: Go to Specimen placement position
    *   Dpad_Down: Do to neutral down placement position
    *
    *  Gamepad 2:
    *  No Drive
    *  Left Joystick runs out and in the arm
    *
    *  TODO:
    *   - Hang pid
    *   - Hang Placement Position
    *   - Specemin placement position
    *   - Low Bucket Position
    *   -
    *
    *   Notes:
    * - For command based:
    * - Have Integer positions for arm position
    * - Have Integer positions for arm states
    * - Have command set arm targets for extendedness and for rotation
    * - Have arm set `
    * - Have the placement check the arm position integer and place accordingly
    * */


//old:
    //negative numbers will be used for the runout positions
    //0 - Down Somewhat neutral
    //1 - parallel to ground and in (neutral pos)
    //2 - Short sample pickup position
    //3 - Specimen placement position
    //4 - Low bucket placement
    //5 - High bucket placement
    //6 - hang ready position
    //7 - hang down position
    //8 - up and retracted (for resetting)

    //new:
    //-1 - Down Somewhat neutral
    //-2 - parallel to ground and in (neutral pos)
    //-3 - Short sample pickup position
    //-4 - specimen placed position
    //-5 - initialized position
    //3 - Specimen placement position
    //4 - Low bucket placement
    //5 - High bucket placement
    //6 - hang ready position
    //7 - hang down position
    //8 - up and retracted (for resetting)


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        if(!CLAW_CONTINUOUS) {
            closeClaw();
        }
        else {
            stopClaw();
        }
        clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
        armPosition = -2;

        resetPosition();

        waitForStart(); //-------------------------------------------------------------------------------------------------------
        while(opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.7,
                            -gamepad1.left_stick_x * 0.7
                    ),
                    -gamepad1.right_stick_x * 0.55
            ));
         //   calculateLiftVoltage();
            if(gamepad1.left_bumper){
                if(CLAW_CONTINUOUS) {
                    runClaw();
                }
                else {
                    closeClaw();
                }
            } else if(gamepad1.right_bumper){
                if(CLAW_CONTINUOUS) {
                    runClawReverse();
                }
                else {
                    openClaw();
                }
            } else if (!(armPosition == -1) && CLAW_CONTINUOUS){
                stopClaw();
            }
            if((gamepad1.x || gamepad2.x) && !pressed) {
                pressed = true;
                lift.setPower(0.4);
                retracting = true;
            }
            else if((gamepad1.b || (gamepad2.b && !gamepad2.start)) && !pressed) {
                pressed = true;
                retracting = false;
                encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(CLAW_CONTINUOUS) {
                    stopClaw();
                }
                resetPosition();
                //sets to a retracted position based on where the arm currently is
                if(isDown) {
                    armPosition = -2;
                }
                else {
                    armPosition = 8;
                }
            }



            if ((gamepad1.dpad_down || gamepad2.dpad_down) && !pressed && armPosition != -1){
                //Rotate to the neutral down pos
                pressed = true;
                armPosition = -1;

                resetPosition();
                if(CLAW_CONTINUOUS) {
                    runClaw();
                }
                else {
                    clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
                }
            } else if((gamepad1.dpad_right || gamepad2.dpad_right) && !pressed && armPosition != -3) {
                pressed = true;
                armPosition = -3;

                resetPosition();
                clawRotate.setPosition(BotConstants.SERVO_DOWN_POS);
                if(CLAW_CONTINUOUS) {
                    runClaw();
                }
            } else if ((gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) && !pressed){
                //Rotate to the neutral pos after placing


                if (armPosition == 3){//Specimen placement position
                    clawRotate.setPosition(BotConstants.SERVO_SPECIMEN_PLACEMENT_POS);
                    resetPosition();
                    armPosition = -4; // Not quite down
                    if(CLAW_CONTINUOUS) {
                        stopClaw();
                    }

                } else if (armPosition == -4) {//Specimen place to all the way down
                    if(CLAW_CONTINUOUS) {
                        stopClaw();
                    }
                    else {
                        openClaw();
                        }
                    resetPosition();
                    clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
                }
                else if (armPosition == 5 || armPosition    == 4){ //bucket placement pos
                    resetPosition();
                    clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
                    if(CLAW_CONTINUOUS) {
                        stopClaw();
                    }
                } else if (armPosition == -1 || armPosition == -3) { //Down pos or specimen pickup pos
                    if(CLAW_CONTINUOUS) {
                        stopClaw();
                    }
                    else {
                        closeClaw();
                    }
                    resetPosition();
                    clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
                } else {
                    if(CLAW_CONTINUOUS) {
                        stopClaw();
                    }
                    resetPosition();
                    clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
                }
                pressed = true;
                if (armPosition != -4){
                    armPosition = -2;
                }




            } /*else if(gamepad1.dpad_right && !pressed && armPosition != 2 || gamepad2.dpad_right && !pressed && armPosition != 2){
                //go to specimen pickup position
                pressed = true;
                armPosition = 2;
                resetPid();
                rotateToPosition = BotConstants.ARM_SPECIMEN_PICKUP_POSITION;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
            }*/else if((gamepad1.dpad_left || gamepad2.dpad_left) && !pressed && armPosition != 3){
                //Go to high specimen placement position
                pressed = true;
                armPosition = 3;
                if(CLAW_CONTINUOUS) {
                    stopClaw();
                }
                clawRotate.setPosition(BotConstants.SERVO_SPECIMEN_READY_POS);
                resetPosition();
            } else if((gamepad1.dpad_up || gamepad2.dpad_up) && !pressed && armPosition != 4){
                //Go to low bucket position
                pressed = true;
                armPosition = 4;
                if(CLAW_CONTINUOUS) {
                    stopClaw();
                }
                clawRotate.setPosition(BotConstants.SERVO_PLACEMENT_POS);


                resetPosition();
            } else if((gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) && !pressed && armPosition != 5){
                //Go to high bucket position
                if(CLAW_CONTINUOUS) {
                    stopClaw();
                }
                pressed = true;
                armPosition = 5;

                clawRotate.setPosition(BotConstants.SERVO_PLACEMENT_POS);

                resetPosition();
            } else if((gamepad1.y || gamepad2.y) && !pressed && armPosition != 6){
                //Get ready to hang
                if(CLAW_CONTINUOUS) {
                    stopClaw();
                }
                hanging = false;
                pressed = true;
                armPosition = 6;
                clawRotate.setPosition(BotConstants.SERVO_DOWN_POS);

                resetPosition();


            } else if(((gamepad1.a && !gamepad1.start)|| gamepad2.a) && !pressed && armPosition == 6){
                //Pull down on hang
                armPosition = 7;
                pressed = true;
                hanging = true;
                clawRotate.setPosition(BotConstants.SERVO_INIT_POS);

                resetPosition();
                rotate.setPower(-1);


            } else if (!gamepad1.dpad_down && !(gamepad1.right_trigger > 0.1) && !gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !(gamepad1.left_trigger > 0.1) && !gamepad1.y && !gamepad1.a && !gamepad2.dpad_down && !(gamepad2.right_trigger > 0.1) && !gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_up && !(gamepad2.left_trigger > 0.1) && !gamepad2.y && !gamepad2.a) {
                pressed = false;


            }







            try {
                handleArm();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            if(tagFound){
                telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
                telemetry.addLine(String.format("Translation X: %.2f feet", tagOfInterest.pose.x*3.28084));
                telemetry.addLine(String.format("Translation Y: %.2f feet", tagOfInterest.pose.y*3.28084));
                telemetry.addLine(String.format("Translation Z: %.2f feet", tagOfInterest.pose.z*3.28084));
                telemetry.addLine(String.format("Estimated X: %.2f", botX));
                telemetry.addLine(String.format("Estimated Y: %.2f", botY));
                telemetry.addLine(String.format("Estimated Z: %.2f", botZ));
                Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
            }
                drive.updatePoseEstimate();
                telemetry.addData("Pose X", drive.pose.position.x);
                telemetry.addData("Pose Y", drive.pose.position.y);
                telemetry.addData("tag found?",tagFound);
                telemetry.addData("armPosition", armPosition);
                telemetry.addData("isIn", isIn);
                telemetry.addData("isDown", isDown);
                telemetry.addData("pressed", pressed);
                telemetry.addData("target", rotateTargetPosition);
            try {
                telemetry.addData("PID Power", rotate.getPIDPower(getAngle(), rotateTargetPosition));
            } catch (Exception e) {
                throw new RuntimeException(e);
            }


            telemetry.addData("angle", (rotateEncoder.getVoltage() * (180/Math.PI)));
                telemetry.addData("rotate power", rotatePower);
                telemetry.addData("lift power", liftPower);
                telemetry.addData("rotate encoder output", rotateEncoder.getVoltage());
                telemetry.addData("starting rotate voltage", startingRotateVoltage);
                telemetry.addData("lift encoder output", liftEncoder.getVoltage());
                telemetry.addData("rotations", liftEncoderRotations);
                telemetry.addData("real voltage", liftRealVoltage);
                telemetry.addData("incremental", -encoderMotor.getCurrentPosition());
                telemetry.addData("extendedness", extendedness);
                telemetry.addData("servo position", clawRotate.getPosition());

            telemetry.addData("actualD", rotate.getPid().getD());
            telemetry.addData("i", rotate.getI());
            telemetry.addData("d", rotate.getD());
                telemetry.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
       // executor.shutdownNow();
        }
    }





