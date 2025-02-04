package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

@TeleOp
public class HangTest extends LinearOpMode {
    private DcMotorEx frontRotate, backRotate, frontLift, backLift;
    private DualMotor rotate;
    @Override
    public void runOpMode() {
        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "backRotate");
        MecanumDrive drive= new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        try {
            rotate = new DualMotor(backRotate, frontRotate,
                    BotConstants.armUpKp / BotConstants.VOLTS_PER_TICK,
                    BotConstants.armUpKi / BotConstants.VOLTS_PER_TICK,
                    BotConstants.armUpKd / BotConstants.VOLTS_PER_TICK);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while(opModeIsActive()) {
            rotate.setPower(-1);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
    }
}
