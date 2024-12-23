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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Tuning")

public class PIDTuning extends LinearOpMode {
    private DcMotorEx lift, leftRotate, rightRotate;
    private Servo rotate, left, right;
    /*public double armKp = 0.05;
    public double armKi = 0;
    public double armKd = 0;*/



    private final double horizontalTicks = 135;
    private final double verticalTicks = 695;
    private final double RADS_PER_TICK = Math.PI / 2 / (verticalTicks - horizontalTicks); //=0.002805
    private final double RADS_PER_VOLT  = Math.PI * 2 / 3.22;
    private final double VOLTS_PER_TICK = RADS_PER_TICK / RADS_PER_VOLT;
    private final double HORIZONTAL_VOLTS = 1.455;
    private final int ARM_HORIZONTAL_TICKS = 100;
    private final int ARM_FRONT_PLACING_TICKS = 540;
    private final int ARM_GROUND_TICKS = 0;
    private double startTime = -1;
    private final double ARM_FRONT_PLACING_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_FRONT_PLACING_TICKS - ARM_HORIZONTAL_TICKS);
    private final double ARM_GROUND_VOLTS = HORIZONTAL_VOLTS - VOLTS_PER_TICK *
            (ARM_GROUND_TICKS - ARM_HORIZONTAL_TICKS);
    private AnalogInput encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        //lift = hardwareMap.get(DcMotorEx.class, "lift");
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        leftRotate = hardwareMap.get(DcMotorEx.class, "leftRotate");
        leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightRotate = hardwareMap.get(DcMotorEx.class, "rightRotate");
        rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*rotate = hardwareMap.get(Servo.class, "rotate");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");*/
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        DualMotor rotateArm;


        try {
            rotateArm = new DualMotor(leftRotate, rightRotate,
                    MecanumDrive.PARAMS.armKp / VOLTS_PER_TICK,
                    MecanumDrive.PARAMS.armKi / VOLTS_PER_TICK,
                    MecanumDrive.PARAMS.armKd / VOLTS_PER_TICK);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        int rotatePos = 0;
        boolean pressed = false;
        waitForStart();
        rotatePos = 880;
        try {
            rotateArm.setTargetPosition(rotatePos);
            rotateArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();
            double frictionOffset;
            double target = HORIZONTAL_VOLTS;
            try {
                rotateArm.setTargetPosition((int)(target / VOLTS_PER_TICK));
                rotateArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frictionOffset = (target > encoder.getVoltage())?-MecanumDrive.PARAMS.frictionOffsetPower:MecanumDrive.PARAMS.frictionOffsetPower;
                rotateArm.setPower(clamp(/*rotateArm.getPIDPower(-target, -encoder.getVoltage()) +*/
                        MecanumDrive.PARAMS.armBasePower * Math.cos(RADS_PER_VOLT * (HORIZONTAL_VOLTS -
                                encoder.getVoltage())) /*+ frictionOffset*/, -1, 1));
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            //Lift
            /*
            try {
                //Purely for constant power to oppose gravity
                rotateArm.setPower(MecanumDrive.PARAMS.armBasePower);
                //overall PID power

                if(rotatePos == 0) {
                    rotateArm.setPower(rotateArm.getPIDPower() - MecanumDrive.PARAMS.armBasePower);
                }
                else {
                    rotateArm.setPower(rotateArm.getPIDPower() + MecanumDrive.PARAMS.armBasePower);
                }

            } catch (Exception e) {
                throw new RuntimeException(e);
            }*/
            /*
            try {
                if (rotateArm.getCurrentPosition() > -5 && rotatePos == 0) {
                    if(startTime == -1) {
                        startTime = timer.milliseconds();
                    }
                    if(timer.milliseconds() > startTime + 1000) {
                        rotatePos = -880;
                        rotateArm.setTargetPosition(rotatePos);
                        startTime = -1;
                    }
                } else if (rotateArm.getCurrentPosition() < -875 && rotatePos == -880) {
                    if(startTime == -1) {
                        startTime = timer.milliseconds();
                    }
                    if(timer.milliseconds() > startTime + 1000) {
                        rotatePos = 0;
                        rotateArm.setTargetPosition(rotatePos);
                        startTime = -1;
                    }
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            */
            if ((gamepad1.left_trigger > 0.1) && !pressed) {
                pressed = true;
                rotatePos = 540;

            } else if ((gamepad1.right_trigger > 0.1) && !pressed) {
                pressed = true;
                rotatePos = 30;

            } else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }
/*
            try{
                rotateArm.setTargetPosition(rotatePos);
                rotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(rotatePos <= Math.abs(leftRotate.getCurrentPosition())){
                    rotateArm.setPower(0.2);
                } else{
                    rotateArm.setPower(1);
                }
            } catch(Exception e) {
                throw new RuntimeException(e);
            }
            */
            telemetry.addData("rotate pos", rotatePos);
            telemetry.addData("left motor", leftRotate.getCurrentPosition());
            telemetry.addData("right motor", rightRotate.getCurrentPosition());

            telemetry.addData("current angle", Math.toDegrees((-encoder.getVoltage() + HORIZONTAL_VOLTS) * RADS_PER_VOLT));
            try {
                telemetry.addData("PID power", clamp(rotateArm.getPIDPower(-target, -encoder.getVoltage()) +
                        MecanumDrive.PARAMS.armBasePower * Math.cos(RADS_PER_VOLT * (target -
                                encoder.getVoltage())) + frictionOffset, -1, 1));
                telemetry.addData("voltage", encoder.getVoltage());
                telemetry.addData("target voltage", HORIZONTAL_VOLTS);
                telemetry.addData("base power", MecanumDrive.PARAMS.armBasePower *
                                    Math.cos(RADS_PER_VOLT * (target - encoder.getVoltage())));
                telemetry.addData("error", rotateArm.PID.getError());
                telemetry.addData("last error", rotateArm.PID.getLastError());
                telemetry.addData("derivative term", rotateArm.PID.getDerivativeTerm());
                telemetry.addData("Friction offset", frictionOffset);
            } catch(Exception e) {
                throw new RuntimeException(e);
            }

            //telemetry.addData("lift", lift.getCurrentPosition());
            //telemetry.addData("right", right.getPosition());
            //telemetry.addData("left", left.getPosition());
            //telemetry.addData("Rotate", rotate.getPosition());
            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }

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

}