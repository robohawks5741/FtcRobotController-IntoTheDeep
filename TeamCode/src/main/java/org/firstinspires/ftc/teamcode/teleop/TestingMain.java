package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
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
import org.firstinspires.ftc.teamcode.subsystems.TagConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "TestingTeleop")
public class TestingMain extends LinearOpMode {



    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;

    final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 909.781;
    double fy = 909.781;
    double cx = 627.946;
    double cy = 354.307;

    // UNITS ARE METERS
    double tagsize = 0.1016;
    AprilTagDetection tagOfInterest = null;




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

    public int debug = 0;

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
        //TODO: set these in relation to april tag of interest
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,800, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;

                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                    drive = new MecanumDrive(hardwareMap,
                            new Pose2d(tagOfInterest.pose.x, tagOfInterest.pose.y, rot.firstAngle));
                    telemetry.addLine("reset pose at x = " + tagOfInterest.pose.x + ", y = " + tagOfInterest.pose.y
                    + "heading = " + rot.firstAngle);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);



        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }




        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "backRotate");
        frontLift = hardwareMap.get(DcMotorEx.class, "liftFront");
        backLift = hardwareMap.get(DcMotorEx.class, "liftBack");
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean goingToGround = false;
        /*
        Drive motors:
        leftFront
        rightFront
        leftBack
        rightBack
        Expansion hub motors:
        frontRotate
        backRotate
        liftBack
        liftFront
         */
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
                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_SPECIMEN_PICKUP_POSITION;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
            }else if(gamepad1.dpad_left && !pressed && armPosition != 3 || gamepad2.dpad_left && !pressed && armPosition != 3){
                //Go to high specimen placement position
                pressed = true;
                armPosition = 3;

                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_SPECIMEN_PLACEMENT_POSITION;
                extendToPosition = BotConstants.LIFT_SPECIMEN_PLACEMENT_POSITION;
            } else if(gamepad1.dpad_up && !pressed && armPosition != 4 || gamepad2.dpad_up && !pressed && armPosition != 4){
                //Go to low bucket position
                pressed = true;
                armPosition = 4;

                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_FRONT_PLACING_VOLTS;
                extendToPosition = BotConstants.LIFT_LOW_BUCKET;
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

                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_FRONT_PLACING_VOLTS;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;

            } else if(gamepad1.a && !pressed && armPosition != 7 || gamepad2.a && !pressed && armPosition != 7){
                //Pull down on hang
                pressed = true;
                armPosition = 7;
                lift.resetPID();
                rotate.resetPID();
                rotateToPosition = BotConstants.ARM_GROUND_VOLTS_RETRACTED;
                extendToPosition = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                
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

            rotateTargetVoltage = normalizeRotateVoltage(rotateTargetVoltage);
            extendedness = (liftRealVoltage - BotConstants.LIFT_RETRACTED_VOLTS)
                    / (BotConstants.LIFT_EXTENDED_VOLTS - BotConstants.LIFT_RETRACTED_VOLTS);
            //solves for target positions in ticks for rotate and lift based on the voltage values
            rotatePos = (int)((rotateTargetVoltage - startingRotateVoltage) / BotConstants.VOLTS_PER_TICK);
            //this sign needs to be checked
            liftPos = (int)((liftTargetVoltage - startingLiftVoltage) / BotConstants.VOLTS_PER_TICK);
            //checks whether the lift encoder voltage has ticked over one way or the other
            checkLiftEncoder();

            handleArm();
            try {
                //return value is just for telemetry purposes

              // rotatePower = updateRotate();
                //liftPower = updateLift();
            } catch(Exception e) {
                throw new RuntimeException(e);
            }


            try {
                telemetry.addData("debug", debug);
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
        if (isDown && armPosition > 2 && !isIn || !isDown && armPosition < 3 && !isIn){ //Sees if it needs to pull in
            liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
            debug = 1;

        }else if((!isDown && armPosition < 3 && isIn) || (isDown && armPosition > 2 && isIn) || (isDown && armPosition < 3) || (!isDown && armPosition > 2)){
            //Waits until it is pulled in yo rotate it
            //is up, is going down, and is in
            //is down, is going up and is in
            //is down, is staying down
            //is up and is staying up
            //Only activates when it is down and the target is the specimen placement, high bucket, low bucket, and hang position
            rotateTargetVoltage = rotateToPosition;
            debug = 2;
            if((armPosition < 3 && isDown) || (armPosition > 2) && !isDown){

                //Is going down and is down
                //Is going up and is up

                liftTargetVoltage = extendToPosition;
                debug = 3;
            }

        }

        //Update arm and rotate
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
            liftPower = -(lift.getPIDPower(liftTargetVoltage, liftRealVoltage)
                    + BotConstants.liftBasePower * Math.sin(getAngle()));
            lift.setPower(clamp(liftPower, -1, 1));
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    //makes sure voltage is a single continuous scale rather than having a jump from 0 to 3.2
    public double normalizeRotateVoltage(double v) {
        if(v > mod(BotConstants.ARM_VERTICAL_VOLTS - 0.2, BotConstants.MAX_VOLTAGE)) {
            v -= BotConstants.MAX_VOLTAGE;
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
        clawRotate.setPosition(MecanumDrive.PARAMS.servoPos2);

    }
    public void rotateClawDown() {
        clawRotate.setPosition(MecanumDrive.PARAMS.servoPos1);

    }

    public void calculateLiftVoltage() {
        liftRealVoltage = startingLiftVoltage + -encoderMotor.getCurrentPosition() * BotConstants.INCREMENTAL_TO_VOLTS;
        liftRealVoltage += normalizeRotateVoltage(rotateEncoder.getVoltage()) -
                normalizeRotateVoltage(BotConstants.ARM_VERTICAL_VOLTS);
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

    public static double mod(double a, double b) {
        while(a < 0) {
            a += b;
        }
        while(a > b) {
            a -= b;
        }
        return a;
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        double botX = -detection.pose.x;
        double botY = -detection.pose.y;
        double botZ = -detection.pose.z;

        switch(detection.id) {
            case(11):
                botX += TagConstants.TAG_POSITIONS.TAG11.x();
                botY += TagConstants.TAG_POSITIONS.TAG11.y();
                botZ += TagConstants.TAG_POSITIONS.TAG11.z();
                break;

            case(12):
                botX += TagConstants.TAG_POSITIONS.TAG12.x();
                botY += TagConstants.TAG_POSITIONS.TAG12.y();
                botZ += TagConstants.TAG_POSITIONS.TAG12.z();
                break;

            case(13):
                botX += TagConstants.TAG_POSITIONS.TAG13.x();
                botY += TagConstants.TAG_POSITIONS.TAG13.y();
                botZ += TagConstants.TAG_POSITIONS.TAG13.z();
                break;

            case(14):
                botX += TagConstants.TAG_POSITIONS.TAG14.x();
                botY += TagConstants.TAG_POSITIONS.TAG14.y();
                botZ += TagConstants.TAG_POSITIONS.TAG14.z();
                break;

            case(15):
                botX += TagConstants.TAG_POSITIONS.TAG15.x();
                botY += TagConstants.TAG_POSITIONS.TAG15.y();
                botZ += TagConstants.TAG_POSITIONS.TAG15.z();
                break;

            case(16):
                botX += TagConstants.TAG_POSITIONS.TAG16.x();
                botY += TagConstants.TAG_POSITIONS.TAG16.y();
                botZ += TagConstants.TAG_POSITIONS.TAG16.z();
                break;
        }

        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine(String.format("Estimated X:", botX));
        telemetry.addLine(String.format("Estimated Y:", botY));
        telemetry.addLine(String.format("Estimated Z:", botZ));
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));



    }

}
