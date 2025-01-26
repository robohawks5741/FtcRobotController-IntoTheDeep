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
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DualMotor;

@TeleOp(name = "TestingTeleop")
public class TestingMain extends LinearOpMode {

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

    private DcMotorEx frontRotate, backRotate, frontLift, backLift;
    private DualMotor rotate;
    private DualMotor lift;
    private Servo clawIntake;
    private Servo clawRotate;
    private AnalogInput rotateEncoder, liftEncoder;
    private int rotatePos;
    private int liftPos;
    private double rotateTargetVoltage;
    private double liftTargetVoltage;
    private double startingRotateVoltage;
    private double startingLiftVoltage;
    private double liftPreviousVoltage;
    private double desiredExtendedness;
    //this tracks voltage beyond the 0-3.2 scale and goes all the way to the LIFT_EXTENDED_VOLTS value
    private double liftRealVoltage;
    private DcMotorEx encoderMotor;
    private double extendedness;


    private int liftEncoderRotations;

    private double rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
    private double extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;

    private int armPosition = 1;

    private boolean isDown; //Checks to see if the arm is down (Parallel to the ground or lower)

    private boolean isIn; //Checks to see if the arm is pulled in below a certain intake threshold

    public double liftPower = 0;
    public double rotatePower = 0;

    //negative numbers will be used for the runout positions
    //0 - Down Somewhat neutral
    //1 - parallel to ground and in (neutral pos)
    //2 - Specimen pickup position
    //3 - Specimen placement position
    //4 - Low bucket placement
    //5 - High bucket placement
    //6 - hang ready position
    //7 - hang down position

    private int rotateStage;


    @Override
    public void runOpMode() throws InterruptedException {
        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "backRotate");
        frontLift = hardwareMap.get(DcMotorEx.class, "liftFront");
        backLift = hardwareMap.get(DcMotorEx.class, "liftBack");
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean goingToGround = false;
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
        encoderMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        rotateEncoder = hardwareMap.get(AnalogInput.class, "rotateEncoder");
        liftEncoder = hardwareMap.get(AnalogInput.class, "liftEncoder");
        startingRotateVoltage = normalizeRotateVoltage(rotateEncoder.getVoltage());
        startingLiftVoltage = liftEncoder.getVoltage();
        rotateTargetVoltage = startingRotateVoltage;
        liftTargetVoltage = startingLiftVoltage;
        liftPreviousVoltage = startingLiftVoltage;
        desiredExtendedness = 1;
        frontRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPos = BotConstants.LIFT_RETRACTED_TICKS;
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        /*Since the encoder makes multiple full revolutions in the lift extension, it's necessary to
          track how many rotations it's made so far to know the actual position for PID. This requires that
          the encoder is within the closest rotation to retracted on initialization. I don't think there's
          a great solution to this, because no matter what code you have here, the encoder can never tell
          the difference between initializing at a position and one revolution past that position
         */
        liftEncoderRotations = 0;
        boolean pressed = false;
        if(normalizeRotateVoltage(rotateEncoder.getVoltage()) < 0.4) {
            rotateStage = 4;
        }
        else {
            rotateStage = -4;
        }


        clawRotate.setPosition(BotConstants.servoPosInit);
        openClaw();
        lift.setPower(0.001);
        waitForStart(); //-------------------------------------------------------------------------------------------------------
        while(opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.7,
                            -gamepad1.left_stick_x * 0.7
                    ),
                    -gamepad1.right_stick_x * 0.55
            ));
            calculateLiftVoltage();
            /*
            if(gamepad1.left_bumper) {
                runClaw();
            }
            else if(gamepad1.right_bumper) {
                runClawReverse();
            }
            else {
                stopClaw();
            }*/
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

                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_GROUND_VOLTS_EXTENDED;
                extendToPosition = BotConstants.LIFT_EXTENDED_VOLTS;
                rotateClawDown();
            } else if (gamepad1.right_trigger > 0.1 && !pressed && armPosition != 1 || gamepad2.right_trigger > 0.1 && !pressed && armPosition != 1){
                //Place based on position and rotate to the neutral pos
                pressed = true;
                armPosition = 1;

                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.HORIZONTAL_VOLTS;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
            } else if(gamepad1.dpad_right && !pressed && armPosition != 2 || gamepad2.dpad_right && !pressed && armPosition != 2){
                //go to specimen pickup position
                pressed = true;
                armPosition = 2;
            }else if(gamepad1.dpad_left && !pressed && armPosition != 3 || gamepad2.dpad_left && !pressed && armPosition != 3){
                //Go to high specimen placement position
                pressed = true;
                armPosition = 3;
            } else if(gamepad1.dpad_up && !pressed && armPosition != 4 || gamepad2.dpad_up && !pressed && armPosition != 4){
                //Go to low bucket position
                pressed = true;
                armPosition = 4;
            } else if(gamepad1.left_trigger > 0.1 && !pressed && armPosition != 5 || gamepad2.left_trigger > 0.1 && !pressed && armPosition != 5){
                //Go to high bucket position
                pressed = true;
                armPosition = 5;
                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_FRONT_PLACING_VOLTS;
                extendToPosition = BotConstants.LIFT_EXTENDED_VOLTS;
            } else if(gamepad1.y && !pressed && armPosition != 6 || gamepad2.y && !pressed && armPosition != 6){
                //Get ready to hang
                pressed = true;
                armPosition = 6;
            } else if(gamepad1.a && !pressed && armPosition != 7 || gamepad2.a && !pressed && armPosition != 7){
                //Pull down on hang
                pressed = true;
                armPosition = 7;
            } else if (!gamepad1.dpad_down && !(gamepad1.right_trigger > 0.1) && !gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !(gamepad1.left_trigger > 0.1) && !gamepad1.y && !gamepad1.a && !gamepad2.dpad_down && !(gamepad2.right_trigger > 0.1) && !gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_up && !(gamepad2.left_trigger > 0.1) && !gamepad2.y && !gamepad2.a) {
                pressed = false;
            }

            //Checks to see if the arm is pulled in
            if (liftRealVoltage < BotConstants.LIFT_ROTATABLE_VOLTS) {
                isIn = true;
            } else {
                isIn = false;
            }

            //Checks to see if the arm is down or up
            if(normalizeRotateVoltage(rotateEncoder.getVoltage()) < BotConstants.ARM_UP_EXTENDABLE_VOLTS) {
                isDown = false;
            } else {
                isDown = true;
            }

/*
            if (gamepad1.dpad_left && !pressed){
                pressed = true;
                if(rotateStage > 0) {
                    rotateStage = -1;
                    goingToGround = false;
                    lift.resetPID();
                }
                else if(rotateStage == -4) {
                    goingToGround = false;
                    rotateStage = -2;
                    //Pick up and get out
                    liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                }
            } else if(gamepad1.dpad_up && !pressed){
                //Turn up
                if(rotateStage <= 0) {
                    rotateStage = 1;
                    clawRotate.setPosition(BotConstants.servoPosUp);
                    lift.resetPID();
                }
                else if(rotateStage == 4 && liftTargetVoltage != BotConstants.LIFT_EXTENDED_VOLTS) {
                    liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                    lift.resetPID();
                }
            } else if(gamepad1.dpad_down){
                pressed = true;
                if(rotateStage > 0) {
                    rotateStage = -1;
                    goingToGround = true;
                    lift.resetPID();
                } else if(gamepad1.dpad_down){ //Go from middle to down
                    pressed = true;
                    if (rotateStage == -2){
                        goingToGround = true;
                    }
                }

                else if(rotateStage == -4){
                    //Turn down
                    rotateTargetVoltage = BotConstants.ARM_GROUND_VOLTS_EXTENDED;
                    rotate.resetPID();
                    //open/close claw
                    //openClaw();
                    liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                    lift.resetPID();
                }

            }else if (!(gamepad1.left_trigger > 0.1) && !(gamepad1.right_trigger > 0.1)) {
                pressed = false;
            }

            if (gamepad1.a && !(rotateStage == -4 && goingToGround)) {
                if(liftTargetVoltage != BotConstants.LIFT_RETRACTED_VOLTS && (rotateStage == -4 || rotateStage == 4)) {
                    liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
                    lift.resetPID();
                }
            }else if (gamepad1.y && !(rotateStage == -4 && goingToGround)) {
                if(liftTargetVoltage != BotConstants.LIFT_EXTENDED_VOLTS && (rotateStage == -4 || rotateStage == 4)) {
                    liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                    lift.resetPID();
                }
            }

            try {
                if(rotateStage == 1) { //Pull the arm in
                    liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;

                    try {
                        //checks the arm is retracted enough before lifting
                        if (liftRealVoltage < BotConstants.LIFT_ROTATABLE_VOLTS) {
                            rotateStage++;
                            rotate.resetPID();

                        }
                    } catch(Exception e) {
                        throw new RuntimeException(e);
                    }
                }
                else if (rotateStage == 2) {
                    rotateTargetVoltage = BotConstants.ARM_FRONT_PLACING_VOLTS;
                    if(normalizeRotateVoltage(rotateEncoder.getVoltage()) < BotConstants.ARM_UP_EXTENDABLE_VOLTS) {
                        rotateStage++;
                        lift.resetPID();
                    }
                }
                else if(rotateStage == 3) {
                    liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                    rotateStage++;
                }
                else if(rotateStage == -1) { //Pull In arm
                    if (goingToGround){
                        liftTargetVoltage = BotConstants.LIFT_RETRACTED_VOLTS;
                    } else {
                        liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                    }

                    if (liftRealVoltage < BotConstants.LIFT_ROTATABLE_VOLTS) { //If arm is below threshhold, then rotate the arm
                        rotateStage--;
                        rotate.resetPID();
                    }
                }
                else if(rotateStage == -2) { //Rotate the arm this is the end of the sideways pos
                    rotateTargetVoltage =  goingToGround ? BotConstants.ARM_GROUND_VOLTS_EXTENDED:BotConstants.HORIZONTAL_VOLTS;
                    if(goingToGround && normalizeRotateVoltage(rotateEncoder.getVoltage()) > BotConstants.ARM_DOWN_EXTENDABLE_VOLTS) { //If the arm is rotated below a certain threshold, then extend again
                        rotateStage--;
                        lift.resetPID();
                    }
                }
                else if(rotateStage == -3) { //Extend the arm and rotate the claw down
                    rotateClawDown();
                    liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                    lift.resetPID();
                    desiredExtendedness = 1;
                    rotateStage--;
                }


                else if(rotateStage == -4 && goingToGround) { //Manually Run out when arm is down
                    if(gamepad1.a && desiredExtendedness < 1) {
                        desiredExtendedness += .01;
                    }
                    if(gamepad1.x && desiredExtendedness > 0) {
                        desiredExtendedness -= .01;
                    }
                    rotateTargetVoltage = (BotConstants.ARM_GROUND_VOLTS_EXTENDED -
                            BotConstants.ARM_GROUND_VOLTS_RETRACTED) * desiredExtendedness +
                            BotConstants.ARM_GROUND_VOLTS_EXTENDED;
                    liftTargetVoltage = (BotConstants.LIFT_EXTENDED_VOLTS - BotConstants.LIFT_RETRACTED_VOLTS) *
                            desiredExtendedness + BotConstants.ARM_GROUND_VOLTS_EXTENDED;


                }



            } catch(Exception e) {
                throw new RuntimeException(e);
            }
            */


            //Update arm pos


            //sample no controller code to test individual parts
            //comment out update functions as needed






            rotateTargetVoltage = normalizeRotateVoltage(rotateTargetVoltage);
            extendedness = (liftRealVoltage - BotConstants.LIFT_RETRACTED_VOLTS)
                    / (BotConstants.LIFT_EXTENDED_VOLTS - BotConstants.LIFT_RETRACTED_VOLTS);
            //solves for target positions in ticks for rotate and lift based on the voltage values
            rotatePos = (int)((rotateTargetVoltage - startingRotateVoltage) / BotConstants.VOLTS_PER_TICK);
            //this sign needs to be checked
            liftPos = (int)((liftTargetVoltage - startingLiftVoltage) / BotConstants.VOLTS_PER_TICK);
            //checks whether the lift encoder voltage has ticked over one way or the other
            checkLiftEncoder();

            try {
                //return value is just for telemetry purposes
                handleArm();
              // rotatePower = updateRotate();
                //liftPower = updateLift();
            } catch(Exception e) {
                throw new RuntimeException(e);
            }


            try {
                telemetry.addData("armPosition", armPosition);
                telemetry.addData("isIn", isIn);
                telemetry.addData("isDown", isDown);
                telemetry.addData("pressed", pressed);
                telemetry.addData("target", liftTargetVoltage);
                telemetry.addData("Rotate Stage", rotateStage);
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
    public void handleArm(){
        if (isDown && armPosition > 2 && !isIn || !isDown && armPosition < 4 && !isIn){ //Sees if it needs to pull in
            liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;

        }else if((isIn && armPosition > 2 && isDown)|| (!isDown && armPosition < 3 && isIn)){ //Waits until it is pulled in yo rotate it
            //Only activates when it is down and the target is the specimen placement, high bucket, low bucket, and hang position
            rotateTargetVoltage = rotateToPosition;
        } else {
            liftTargetVoltage = extendToPosition;
        }
        updateRotate();
        updateLift();
    }
    public void updateRotate() {
        rotate.setTargetPosition(rotatePos);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            //signs of each part of this are based on direction of motor and encoder
            //currently up from ground = positive power, negative encoder voltage
            rotatePower = (-rotate.getPIDPower(rotateTargetVoltage, normalizeRotateVoltage(rotateEncoder.getVoltage()))
                    + BotConstants.armBasePower * Math.cos(getAngle()));
            double powerScaleFactor = (1 + extendedness) / 2;
            rotate.setPower(powerScaleFactor * clamp(rotatePower, -1, 1));
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    public void updateLift() {
        lift.setTargetPosition(liftPos);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            //sign needs to be checked
            liftPower = -(lift.getPIDPower(liftTargetVoltage, liftRealVoltage)
                    + BotConstants.liftBasePower * Math.sin(getAngle()));
            lift.setPower(clamp(liftPower, -1, 1));
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    //makes sure voltage is a single continuous scale rather than having a jump from 0 to 3.2
    public double normalizeRotateVoltage(double v) {
        if(v > 3) {
            v = 0;
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

    public double getAngle() {
        return -(rotateEncoder.getVoltage() - BotConstants.HORIZONTAL_VOLTS) * BotConstants.RADS_PER_VOLT;
    }
    public void runClaw() {
        clawIntake.setPosition(0.5);
    }
    public void runClawReverse() {
        clawIntake.setPosition(-0.5);
    }
    public void stopClaw() {
        clawIntake.setPosition(0);
    }
    //these constants need to be redetermined
    public void rotateClawUp() {
        clawRotate.setPosition(MecanumDrive.PARAMS.servoPos1);
    }
    public void rotateClawDown() {
        clawRotate.setPosition(MecanumDrive.PARAMS.servoPos2);
    }

    public void calculateLiftVoltage() {
        liftRealVoltage = startingLiftVoltage + -encoderMotor.getCurrentPosition() * BotConstants.INCREMENTAL_TO_VOLTS;
        liftRealVoltage += normalizeRotateVoltage(rotateEncoder.getVoltage());
        // this works bc vertical is approximately 0 volts for the rotate encoder
    }
    public void checkLiftEncoder() {
        //this is one idea for doing this, might need to be tweaked/reworked depending on how jumpy the encoder is
        if(liftEncoder.getVoltage() < 1.5 && liftPreviousVoltage > BotConstants.MAX_VOLTAGE - 1.5) {
            liftEncoderRotations++;
        }
        else if(liftEncoder.getVoltage() > liftEncoder.getMaxVoltage() - 0.5 && liftPreviousVoltage < 0.5) {
            liftEncoderRotations--;
        }
        //Which of these if statements is incrementing/decrementing depends on the direction positive voltage
        //is on the extension, which needs to be checked
        liftPreviousVoltage = liftEncoder.getVoltage();
    }
    public void openClaw() {
        clawIntake.setPosition(BotConstants.CLAW_OPEN);
    }
    public void closeClaw() {
        clawIntake.setPosition(BotConstants.CLAW_CLOSED);
    }
    //TODO: create code for moving claw along ground
}
