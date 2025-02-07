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
    *   Dpad_Left: Go to Specimin placement position
    *   Dpad_Down: Do to neutral down placement position
    *
    *  Gamepad 2:
    *  No Drive
    *  Left Joystick runs out and in the arm
    *
    *  ToDo:
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



    //negative numbers will be used for the runout positions
    //0 - Down Somewhat neutral
    //1 - parallel to ground and in (neutral pos)
    //2 - Short sample pickup position
    //3 - Specimen placement position
    //4 - Low bucket placement
    //5 - High bucket placement
    //6 - hang ready position
    //7 - hang down position



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        closeClaw();
        clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
        rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
        extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
        armPosition = 1;

        resetPid();

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
                closeClaw();

            } else if(gamepad1.right_bumper){
                openClaw();

            }
            if(gamepad1.x) {
                rotateClawUp();
            }
            else if(gamepad1.b) {
                rotateClawDown();
            }

            if (gamepad1.dpad_down && !pressed && armPosition != 0 || gamepad2.dpad_down && !pressed && armPosition != 0){
                //Rotate to the neutral down pos
                pressed = true;
                armPosition = 0;

                resetPid();
                rotateToPosition = BotConstants.ARM_GROUND_VOLTS_EXTENDED;
                extendToPosition = BotConstants.LIFT_DOWN_EXTENDED_VOLTS;
                clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
            } else if((gamepad1.dpad_right || gamepad2.dpad_right) && !pressed && armPosition != 2) {
                pressed = true;
                armPosition = 2;

                resetPid();
                rotateToPosition = BotConstants.ROTATE_SHORT_DOWN_VOLTS;
                extendToPosition = BotConstants.LIFT_SHORT_DOWN_VOLTS;
                clawRotate.setPosition(BotConstants.SERVO_DOWN_POS);
            } else if (gamepad1.right_trigger > 0.1 && !pressed || gamepad2.right_trigger > 0.1 && !pressed ){
                //Place based on position and rotate to the neutral pos


                if (armPosition == 3){//Specimen placement position
                    clawRotate.setPosition(BotConstants.SERVO_SPECIMEN_PLACEMENT_POS);
                    resetPid();
                    rotateToPosition = BotConstants.ROTATE_SPECIMEN_PLACEMENT;
                    extendToPosition = BotConstants.LIFT_SPECIMEN_PLACEMENT_POSITION;
                    armPosition = 1.1; // Not quite down yk

                } else if (armPosition == 1.1){//Specimen place to all the way down yk
                    openClaw();
                    resetPid();
                    rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
                    extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                }
                else if (armPosition == 5 || armPosition == 4){ //bucket placement pos
                    resetPid();
                    rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
                    extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;

                } else if (armPosition == 0 || armPosition == 2){ //Down pos or specimen pickup pos
                    closeClaw();
                    resetPid();
                    rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
                    extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                } else {
                    resetPid();
                    rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
                    extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                }
                clawRotate.setPosition(BotConstants.SERVO_PARALLEL_POS);
                pressed = true;
                if (armPosition != 1.1){
                    armPosition = 1;

                }




            } /*else if(gamepad1.dpad_right && !pressed && armPosition != 2 || gamepad2.dpad_right && !pressed && armPosition != 2){
                //go to specimen pickup position
                pressed = true;
                armPosition = 2;
                resetPid();
                rotateToPosition = BotConstants.ARM_SPECIMEN_PICKUP_POSITION;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
            }*/else if(gamepad1.dpad_left && !pressed && armPosition != 3 || gamepad2.dpad_left && !pressed && armPosition != 3){
                //Go to high specimen placement position
                pressed = true;
                armPosition = 3;
                clawRotate.setPosition(BotConstants.SERVO_SPECIMEN_READY_POS);
                resetPid();
                rotateToPosition = BotConstants.ROTATE_SPECIMEN_PLACEMENT;
                extendToPosition = BotConstants.LIFT_SPECIMEN_PLACEMENT_POSITION;
            } else if(gamepad1.dpad_up && !pressed && armPosition != 4 || gamepad2.dpad_up && !pressed && armPosition != 4){
                //Go to low bucket position
                pressed = true;
                armPosition = 4;

                clawRotate.setPosition(BotConstants.SERVO_PLACEMENT_POS);


                resetPid();
                rotateToPosition = BotConstants.ARM_FRONT_PLACING_VOLTS;
                extendToPosition = BotConstants.LIFT_LOW_BUCKET;
            } else if(gamepad1.left_trigger > 0.1 && !pressed && armPosition != 5 || gamepad2.left_trigger > 0.1 && !pressed && armPosition != 5){
                //Go to high bucket position
                pressed = true;
                armPosition = 5;

                clawRotate.setPosition(BotConstants.SERVO_PLACEMENT_POS);

                resetPid();
                rotateToPosition = BotConstants.ARM_FRONT_PLACING_VOLTS;
                extendToPosition = BotConstants.LIFT_EXTENDED_VOLTS;
            } else if(gamepad1.y && !pressed && armPosition != 6 || gamepad2.y && !pressed && armPosition != 6){
                //Get ready to hang
                hanging = false;
                pressed = true;
                armPosition = 6;
                clawRotate.setPosition(BotConstants.SERVO_DOWN_POS);

                resetPid();
                rotateToPosition = BotConstants.ARM_BACK_VOLTS;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;


            } else if(gamepad1.a && !pressed && armPosition != 7 || gamepad2.a && !pressed && armPosition != 7){
                //Pull down on hang
                pressed = true;
                hanging = true;
                clawRotate.setPosition(BotConstants.SERVO_INIT_POS);

                resetPid();
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;

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
                telemetry.addData("debug", debug);
                telemetry.addData("armPosition", armPosition);
                telemetry.addData("isIn", isIn);
                telemetry.addData("isDown", isDown);
                telemetry.addData("pressed", pressed);
                telemetry.addData("target", rotateTargetVoltage);
                telemetry.addData("angle", getAngle());
                telemetry.addData("rotate power", rotatePower);
                telemetry.addData("lift power", liftPower);
                telemetry.addData("rotate encoder output", rotateEncoder.getVoltage());
                telemetry.addData("starting rotate voltage", startingRotateVoltage);
                telemetry.addData("lift encoder output", liftEncoder.getVoltage());
                telemetry.addData("rotations", liftEncoderRotations);
                telemetry.addData("real voltage", liftRealVoltage);
                telemetry.addData("incremental", -encoderMotor.getCurrentPosition());
                telemetry.addData("extendedness", extendedness);
                telemetry.addData("servo position", clawIntake.getPosition());
                telemetry.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        executor.shutdownNow();
        }
    }





