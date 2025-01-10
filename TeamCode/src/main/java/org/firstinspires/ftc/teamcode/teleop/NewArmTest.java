package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp
public class NewArmTest extends LinearOpMode {

    private DcMotorEx frontRotate, backRotate, frontLift, backLift;
    private DualMotor rotate, lift;
    private AnalogInput rotateEncoder, liftEncoder;
    private int rotatePos;
    private int liftPos;
    private double rotateTargetVoltage;
    private double liftTargetVoltage;
    private double startingRotateVoltage;
    private double startingLiftVoltage;
    @Override
    public void runOpMode() throws InterruptedException {
        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        frontLift = hardwareMap.get(DcMotorEx.class, "liftFront");
        backLift = hardwareMap.get(DcMotorEx.class, "liftBack");
        try {
            rotate = new DualMotor(backRotate, frontRotate,
                    MecanumDrive.PARAMS.kp / BotConstants.VOLTS_PER_TICK,
                    MecanumDrive.PARAMS.ki / BotConstants.VOLTS_PER_TICK,
                    MecanumDrive.PARAMS.kd / BotConstants.VOLTS_PER_TICK);
            lift = new DualMotor(backLift, frontLift);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        rotateEncoder = hardwareMap.get(AnalogInput.class, "rotateEncoder");
        liftEncoder = hardwareMap.get(AnalogInput.class, "liftEncoder");
        startingRotateVoltage = normalizeRotateVoltage(rotateEncoder.getVoltage());
        startingLiftVoltage = liftEncoder.getVoltage();
        rotateTargetVoltage = startingRotateVoltage;
        liftTargetVoltage = startingLiftVoltage;
        boolean pressed = false;
        int rotateStage = 0;
        double power;
        waitForStart();
        while(opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.7,
                            -gamepad1.left_stick_x * 0.7
                    ),
                    -gamepad1.right_stick_x * 0.55
            ));

            /*if (gamepad1.dpad_left && !pressed){
                //Pick up and get out
                pressed = true;
                rotateStage = 0;
                rotatePos = BotConstants.ARM_HORIZONTAL_TICKS;
                //liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;

            } else if(gamepad1.dpad_up && !pressed){
                //Turn up
                if(rotateStage == 0) {
                    rotateStage = 1;
                }

            } else if(gamepad1.dpad_down){
                //Turn down
                pressed = true;
                rotateStage = 0;
                rotateTargetVoltage = normalizeRotateVoltage(BotConstants.ARM_GROUND_VOLTS);
                //open/close claw
                //liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;

            }else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }

            if (gamepad1.a) {
                //liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
            }else if (gamepad1.y) {
                //liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
            }
            if(rotateStage == 1) {
                //liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
                try {
                    /*if (Math.abs(liftEncoder.getVoltage()) > BotConstants.LIFT_ROTATABLE_VOLTS) {
                        rotateStage++;
                    }*//*
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }
            try {
                if (rotateStage == 2) {
                    rotateTargetVoltage = BotConstants.ARM_FRONT_PLACING_VOLTS;
                    //liftPos = BotConstants.LIFT_EXTENDED_VOLTS;
                }
            } catch(Exception e) {
                throw new RuntimeException(e);
            }*/
            //Update arm pos
            rotateTargetVoltage = normalizeRotateVoltage(BotConstants.HORIZONTAL_VOLTS);
            rotate.setTargetPosition((int)((startingRotateVoltage
                    - rotateTargetVoltage) / BotConstants.VOLTS_PER_TICK));
            rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            try {
                power = -rotate.getPIDPower(rotateTargetVoltage, normalizeRotateVoltage(rotateEncoder.getVoltage()))
                        + BotConstants.armBasePower * Math.cos(getAngle());
                rotate.setPower(clamp(power, -1, 1));
            } catch(Exception e) {
                throw new RuntimeException(e);
            }

            try {
                //updateRotate();
                //updateLift();
                /*pullArm.setTargetPosition(armPos*-23);
                pullArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pullArm.setPower(1);*/
            } catch(Exception e) {
                throw new RuntimeException(e);
            }

            
            rotatePos = -(int)((rotateTargetVoltage - startingRotateVoltage) / BotConstants.VOLTS_PER_TICK);
            liftPos = (int)((liftTargetVoltage - startingLiftVoltage) / BotConstants.VOLTS_PER_TICK);
            //positive is clockwise when back is to the right
            try {
                telemetry.addData("target", rotateTargetVoltage);
                telemetry.addData("angle", getAngle());
                telemetry.addData("power", power);
                telemetry.addData("rotate encoder output", rotateEncoder.getVoltage());
                telemetry.addData("lift encoder output", liftEncoder.getVoltage());
                telemetry.update();
            } catch(Exception e) {
                throw new RuntimeException(e);
            }

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    //these updates aren't used right now/need to be rewritten for PID
    public void updateRotate() {
        if(Math.abs(rotateTargetVoltage - rotateEncoder.getVoltage()) > 0.25) {
            rotate.setTargetPosition(rotatePos);
            rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(rotateTargetVoltage < normalizeRotateVoltage(rotateEncoder.getVoltage())) {
                rotate.setPower(0.4);
            }
            else {
                rotate.setPower(-0.2);
            }
        }
        else if(Math.abs(rotateTargetVoltage - rotateEncoder.getVoltage()) > 0.15) {
            rotate.setTargetPosition(rotatePos);
            rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(rotateTargetVoltage < normalizeRotateVoltage(rotateEncoder.getVoltage())) {
                rotate.setPower(0.2);
            }
            else {
                rotate.setPower(-0.2);
            }
        }
        else {
            rotate.setPower(0);
        }
    }
    public void updateLift() {
        lift.setTargetPosition(liftPos - (rotatePos - BotConstants.ARM_HORIZONTAL_TICKS));
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0.15);
    }
    //makes sure voltage is a single continuous scale rather than having a jump from 0 to 3.2
    public double normalizeRotateVoltage(double v) {
        if(v < 2.4) {
            v += 3.2;
        }
        return v;
    }
    public static double clamp(double input, double min, double max){
        if(input > max) {
            return max;
        }
        if(input < min) {
            return min;
        }
        return input;
    }
    //I think the sign of this might be wrong but it doesn't really matter for now because it's used for cosine
    public double getAngle() {
        return (rotateEncoder.getVoltage() - BotConstants.HORIZONTAL_VOLTS) * BotConstants.RADS_PER_VOLT;
    }
}
