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
import org.firstinspires.ftc.teamcode.subsystems.TagConstants;
import org.firstinspires.ftc.teamcode.subsystems.actions.LoopArm;
import org.firstinspires.ftc.teamcode.subsystems.actions.ServoAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.SetArmPos;

@Autonomous(name = "Auto Left")
public class AutoLeft extends AutoSuper {

    Thread backgroundThread;



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        super.init();

        waitForStart();

        if (opModeIsActive()){

             backgroundThread = new Thread(() -> {
                while (!isStopRequested()){
                    try {
                        handleArm();
                        telemetry.addData("armPos", armPosition);
                        telemetry.addData("isin", isIn);
                        telemetry.addData("lift", liftRealVoltage);
                        telemetry.addData("liftthreshold", BotConstants.LIFT_ROTATABLE_VOLTS);
                        telemetry.addData("isDown",isDown);
                        telemetry.addData("debug", debug);
                        telemetry.addData("target", rotateTargetVoltage);
                        telemetry.update();

                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }
                }
            });
            backgroundThread.start();


            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            .stopAndAdd(new SetArmPos(this, BotConstants.ARM_FRONT_PLACING_VOLTS, BotConstants.LIFT_EXTENDED_VOLTS, 5))
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                          //  .setTangent(Math.toRadians(-45.0))
                            .splineToLinearHeading(new Pose2d(7.18, 23.15, Math.toRadians(-45.0)), 0)
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_OPEN))
                            .waitSeconds(0.5)
                            .stopAndAdd(new SetArmPos(this, BotConstants.ARM_GROUND_VOLTS_EXTENDED, BotConstants.LIFT_EXTENDED_VOLTS, 0))
                            .splineTo(new Vector2d(12.28, 29.60), Math.toRadians(12))
                            .stopAndAdd(new ServoAction(clawIntake,  BotConstants.CLAW_CLOSED))
                            .waitSeconds(0.5)
                            .stopAndAdd(new SetArmPos(this, BotConstants.ARM_FRONT_PLACING_VOLTS, BotConstants.LIFT_EXTENDED_VOLTS, 5))
                            .splineToLinearHeading(new Pose2d(7.18, 23.15, Math.toRadians(-45.0)), 0)
                            .waitSeconds(3)
                            .build()
            ));



        }

        if (isStopRequested()){
            backgroundThread.join();

        }
    }

}