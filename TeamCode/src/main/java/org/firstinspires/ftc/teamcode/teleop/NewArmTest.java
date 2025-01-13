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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class NewArmTest extends LinearOpMode {

    private DcMotorEx frontRotate, backRotate, frontLift, backLift;
    private DualMotor rotate;
    private DualMotor lift;
    private AnalogInput rotateEncoder, liftEncoder;
    private int rotatePos;
    private int liftPos;
    private double rotateTargetVoltage;
    private double liftTargetVoltage;
    private double startingRotateVoltage;
    private double startingLiftVoltage;
    private double liftPreviousVoltage;
    //this tracks voltage beyond the 0-3.2 scale and goes all the way to the LIFT_EXTENDED_VOLTS value
    private double liftRealVoltage;
    private Servo claw;

    private int liftEncoderRotations;
    @Override
    public void runOpMode() throws InterruptedException {
        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "backRotate");
        frontLift = hardwareMap.get(DcMotorEx.class, "liftFront");
        backLift = hardwareMap.get(DcMotorEx.class, "liftBack");
        claw = hardwareMap.get(Servo.class, "claw");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            //when tuning, it's easier to take these from mecanumdrive instead b/c botconstants
            //doesn't show up on first dashboard for whatever reason

            //dividing by voltspertick because all the inputs are in units of volts, so the
            //pid constants need to be in units of inverse volts rather than inverse ticks
            rotate = new DualMotor(backRotate, frontRotate,
                    BotConstants.armUpKp / BotConstants.VOLTS_PER_TICK,
                    BotConstants.armUpKi / BotConstants.VOLTS_PER_TICK,
                    BotConstants.armUpKd / BotConstants.VOLTS_PER_TICK);
            lift = new DualMotor(backLift,
                    BotConstants.liftKp / BotConstants.VOLTS_PER_TICK,
                    BotConstants.liftKi / BotConstants.VOLTS_PER_TICK,
                    BotConstants.liftKd / BotConstants.VOLTS_PER_TICK);
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
        liftPreviousVoltage = startingLiftVoltage;
        frontRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPos = BotConstants.LIFT_RETRACTED_TICKS;

        /*Since the encoder makes multiple full revolutions in the lift extension, it's necessary to
          track how many rotations it's made so far to know the actual position for PID. This requires that
          the encoder is within the closest rotation to retracted on initialization. I don't think there's
          a great solution to this, because no matter what code you have here, the encoder can never tell
          the difference between initializing at a position and one revolution past that position
         */
        liftEncoderRotations = 0;
        boolean pressed = false;
        int rotateStage = 0;
        double rotatePower = 0;
        double liftPower = 0;
        waitForStart();
        while(opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.7,
                            -gamepad1.left_stick_x * 0.7
                    ),
                    -gamepad1.right_stick_x * 0.55
            ));
            calculateLiftVoltage();
            if (gamepad1.dpad_left && !pressed){
                //Pick up and get out
                pressed = true;
                rotateStage = 0;
                rotateTargetVoltage = BotConstants.HORIZONTAL_VOLTS;
                //resetting was done differently before and might have been causing some of the weird issues
                rotate.resetPID();
                liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
                lift.resetPID();

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
                rotate.resetPID();
                //open/close claw
                openClaw();
                liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                lift.resetPID();

            }else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }

            if (gamepad1.a) {
                liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
                lift.resetPID();
            }else if (gamepad1.y) {
                liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                lift.resetPID();
            }
            if(rotateStage == 1) {
                liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
                lift.resetPID();
                try {
                    //sign needs to be checked for this, checks the arm is retracted enough before lifting
                    if (liftRealVoltage < BotConstants.LIFT_ROTATABLE_VOLTS) {
                        rotateStage++;
                    }
                } catch(Exception e) {
                    throw new RuntimeException(e);
                }
            }
            try {
                if (rotateStage == 2) {
                    rotateTargetVoltage = BotConstants.ARM_FRONT_PLACING_VOLTS;
                    rotate.resetPID();
                    liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                    lift.resetPID();
                }
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
            //Update arm pos


            //sample no controller code to test individual parts
            //comment out update functions as needed
            rotateTargetVoltage = BotConstants.ARM_FRONT_PLACING_VOLTS;
            //liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;


            //solves for target positions in ticks for rotate and lift based on the voltage values
            rotatePos = (int)((rotateTargetVoltage - startingRotateVoltage) / BotConstants.VOLTS_PER_TICK);
            //this sign needs to be checked
            liftPos = (int)((liftTargetVoltage - startingLiftVoltage) / BotConstants.VOLTS_PER_TICK);

            //checks whether the lift encoder voltage has ticked over one way or the other
            checkLiftEncoder();

            try {
                //return value is just for telemetry purposes
                rotatePower = updateRotate();
                //liftPower = updateLift();
            } catch(Exception e) {
                throw new RuntimeException(e);
            }


            if(gamepad1.left_bumper){
                closeClaw();
            } else if(gamepad1.right_bumper){
                openClaw();
            }
            //positive ticks for rotate is clockwise when back is to the right
            try {
                telemetry.addData("target", rotateTargetVoltage);
                telemetry.addData("angle", getAngle());
                telemetry.addData("rotate power", rotatePower);
                telemetry.addData("lift power", liftPower);
                telemetry.addData("rotate encoder output", rotateEncoder.getVoltage());
                telemetry.addData("starting rotate voltage", startingRotateVoltage);
                telemetry.addData("lift encoder output", liftEncoder.getVoltage());
                telemetry.addData("lift back position", backLift.getCurrentPosition());
                telemetry.addData("lift front position", frontLift.getCurrentPosition());
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.addData("lift pos", liftPos);
                telemetry.addData("lift target position", lift.getTargetPosition());

            } catch(Exception e) {
                throw new RuntimeException(e);
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public double updateRotate() {
        rotate.setTargetPosition(rotatePos);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            //signs of each part of this are based on direction of motor and encoder
            double power = -(rotate.getPIDPower(rotateTargetVoltage, normalizeRotateVoltage(rotateEncoder.getVoltage()))
                    + BotConstants.armBasePower * Math.cos(getAngle()));
            rotate.setPower(clamp(power, -1, 1));
            return power;
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    public double updateLift() {
        lift.setTargetPosition(liftPos);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            //sign needs to be checked
            double power = -(lift.getPIDPower(liftTargetVoltage, liftRealVoltage)
                    + BotConstants.liftBasePower * Math.cos(getAngle()));
            lift.setPower(clamp(power, -1, 1));
            return power;
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    //makes sure voltage is a single continuous scale rather than having a jump from 0 to 3.2
    public double normalizeRotateVoltage(double v) {
        if(v < 2.4) {
            v += rotateEncoder.getMaxVoltage();
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
    public void openClaw() {
        claw.setPosition(BotConstants.CLAW_OPEN);
    }
    public void closeClaw() {
        claw.setPosition(BotConstants.CLAW_CLOSED);
    }
    public void calculateLiftVoltage() {
        liftRealVoltage = liftEncoder.getVoltage() + liftEncoderRotations * liftEncoder.getMaxVoltage();
    }
    public void checkLiftEncoder() {
        //this is one idea for doing this, might need to be tweaked/reworked depending on how jumpy the encoder is
        if(liftEncoder.getVoltage() < 0.5 && liftPreviousVoltage > liftEncoder.getMaxVoltage() - 0.5) {
            liftEncoderRotations++;
        }
        else if(liftEncoder.getVoltage() > liftEncoder.getMaxVoltage() - 0.5 && liftPreviousVoltage < 0.5) {
            liftEncoderRotations--;
        }
        //Which of these if statements is incrementing/decrementing depends on the direction positive voltage
        //is on the extension, which needs to be checked
        liftPreviousVoltage = liftEncoder.getVoltage();
    }
    //TODO: create code for moving claw along ground
}
