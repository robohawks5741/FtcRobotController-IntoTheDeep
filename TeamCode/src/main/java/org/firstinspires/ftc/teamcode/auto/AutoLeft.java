package org.firstinspires.ftc.teamcode.auto;
import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;


import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;
import org.firstinspires.ftc.teamcode.subsystems.TagConstants;
import org.firstinspires.ftc.teamcode.subsystems.actions.LoopArm;
import org.firstinspires.ftc.teamcode.subsystems.actions.ServoAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.SetArmPos;

@Autonomous(name = "Auto Left")
public class AutoLeft extends AutoSuper {

    Thread backgroundThread;

    double placementX = 5.00;//9.5  //Decreasing gets closer
    double placementY = 26.5;//24.03 //increaing gets closer


    private volatile boolean isRunning = true;




    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        isIn = true;
        isDown = true;
        armPosition = -5;
        rotateTargetPosition = BotConstants.ARM_STARTING_VOLTS;
     //   lift.setPower(0.001);
        if(CLAW_CONTINUOUS) {
            stopClaw();
        }
        else {
            closeClaw();
        }
        clawRotate.setPosition(BotConstants.SERVO_INIT_POS);
        //?
        clawIntake.setPosition(BotConstants.CLAW_CLOSED);

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Arm target", rotateTargetPosition);
            telemetry.addData("Rotate current output", rotateEncoder.getVoltage());
            telemetry.addData("target", rotateTargetPosition);
            telemetry.addData("armPos", armPosition);
            telemetry.addData("isin", isIn);
            telemetry.addData("lift", liftRealVoltage);
            telemetry.addData("rotate pos", rotateEncoder.getVoltage());
            telemetry.addData("liftthreshold", BotConstants.LIFT_ROTATABLE_VOLTS);
            telemetry.addData("isDown",isDown);
            telemetry.addData("debug", debug);
            telemetry.update();
            telemetry.update();

        }



        waitForStart();

        //this is the working code, not ideal

            try {
                rotate = new DualMotor(backRotate, frontRotate,
                        BotConstants.armUpKp - 0.3,
                        BotConstants.armUpKi,
                        BotConstants.armUpKd);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            lift = new DualMotor(backLift,
                    BotConstants.liftKp,
                    BotConstants.liftKi,
                    BotConstants.liftKd);


            resetPosition();

            backgroundThread = new Thread(() -> {
                while (!isStopRequested() && isRunning){
                    try {
                        rotateTargetPosition = normalizeRotateVoltage(rotateTargetPosition);
                        handleArm();
                        telemetry.addData("target", rotateTargetPosition);
                //        telemetry.addData("Arm I", rotate.getPid().integralSum);
                        telemetry.addData("armPos", armPosition);
                        telemetry.addData("isin", isIn);
                        telemetry.addData("lift", liftRealVoltage);
                        telemetry.addData("rotate pos", rotateEncoder.getVoltage());
                        telemetry.addData("liftthreshold", BotConstants.LIFT_ROTATABLE_VOLTS);
                        telemetry.addData("isDown",isDown);
                        telemetry.addData("debug", debug);
                        telemetry.update();

                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            });


            backgroundThread.start();

            //positions are in inches
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PLACEMENT_POS))
                            .lineToX(8)
                            .stopAndAdd(new SetArmPos(this, 5))


                            //Place the first
                            //  .setTangent(Math.toRadians(-45.0))
                            .splineToLinearHeading(new Pose2d(placementX+3, placementY-3, Math.toRadians(-45.0)), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(placementX, placementY), Math.toRadians(-45.0))
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_OPEN))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PARALLEL_POS))


                            //Pickup
                            .splineToLinearHeading(new Pose2d(11.28, 28.60, Math.toRadians(12)), Math.toRadians(-45.0))
                            .stopAndAdd(new SetArmPos(this, -1))
                            .waitSeconds(1.5)
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PLACEMENT_POS))

                            //Place second
                            .stopAndAdd(new SetArmPos(this, 5))
                    //        .splineToLinearHeading(new Pose2d(15.6, 18.4, Math.toRadians(-45.0)), 0)
                            .waitSeconds(1.5)
                            .splineToLinearHeading(new Pose2d(placementX+3, placementY-3, Math.toRadians(-45.0)), Math.toRadians(12))

                            .splineToConstantHeading(new Vector2d(placementX, placementY), Math.toRadians(-45.0))
                           // .splineToConstantHeading(new Vector2d(9.11, 24.03), Math.toRadians(-45.0))
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_OPEN))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PARALLEL_POS))

                            //Pickup third
                            .splineToLinearHeading(new Pose2d(9.40, 24.54, Math.toRadians(0)), Math.toRadians(-45.0))
                            .stopAndAdd(new SetArmPos(this, -1))
                            .waitSeconds(1.5)
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PLACEMENT_POS))

                            //Place the third
                            .stopAndAdd(new SetArmPos(this, 5))
                            //        .splineToLinearHeading(new Pose2d(15.6, 18.4, Math.toRadians(-45.0)), 0)
                            .waitSeconds(1.5)
                            .splineToLinearHeading(new Pose2d(placementX+3, placementY-3, Math.toRadians(-45.0)), Math.toRadians(0))

                            .splineToConstantHeading(new Vector2d(placementX, placementY), Math.toRadians(-45.0))                            // .splineToConstantHeading(new Vector2d(9.11, 24.03), Math.toRadians(-45.0))
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_OPEN))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PARALLEL_POS))


                            //Pickup fourth
                            .splineToLinearHeading(new Pose2d(12.37, 23.47, Math.toRadians(-17.16)), Math.toRadians(-45.0))
                            .stopAndAdd(new SetArmPos(this, -1))
                            .waitSeconds(1.5)
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PLACEMENT_POS))

                            //Place the fourth
                            .stopAndAdd(new SetArmPos(this, 5))
                            //        .splineToLinearHeading(new Pose2d(15.6, 18.4, Math.toRadians(-45.0)), 0)
                            .waitSeconds(1.5)
                            .splineToLinearHeading(new Pose2d(placementX+3, placementY-3, Math.toRadians(-45.0)), Math.toRadians(-18.12))

                            .splineToConstantHeading(new Vector2d(placementX, placementY), Math.toRadians(-45.0))                            // .splineToConstantHeading(new Vector2d(9.11, 24.03), Math.toRadians(-45.0))
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_OPEN))
                            .waitSeconds(0.5)
                            .stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PARALLEL_POS))

                            //Pick up fifth
                            //spline to correct position
                            //.waitSeconds(1.5)
                            //.stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                            //.waitSeconds(0.5)
                            //.stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PLACEMENT_POS))

                            //place fifth
                            //.stopAndAdd(new SetArmPos(this, 5))
                            //.waitSeconds(1.5)
                            //.splineToLinearHeading(new Pose2d(placementX+3, placementY-3, Math.toRadians(-45.0)), Math.toRadians(-18.12))

                            //.splineToConstantHeading(new Vector2d(placementX, placementY), Math.toRadians(-45.0))                            // .splineToConstantHeading(new Vector2d(9.11, 24.03), Math.toRadians(-45.0))
                            //.stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_OPEN))
                            //.waitSeconds(0.5)
                            //.stopAndAdd(new ServoAction(clawRotate,  BotConstants.SERVO_PARALLEL_POS))

                            //Park
                            .splineToConstantHeading(new Vector2d(placementX+4, placementY-3), Math.toRadians(-45.0))
                            .stopAndAdd(new SetArmPos(this, -2))
                            .waitSeconds(1.5)
                            .splineToConstantHeading(new Vector2d(2.19, 30.46), Math.toRadians(-45.0))

                            .build()
            ));
            lift.setPower(0.01);


        telemetry.addLine("Got to isStopRequested");
        armTicksOffset = encoderMotor.getCurrentPosition();
        isRunning = false; // Stop the thread safely

        if (backgroundThread != null) {
            try {
                backgroundThread.join(); // Ensure the thread exits before finishing
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}