package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "#Encoder")
public class Encoder extends LinearOpMode {

    private double position;
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "encoder");
        waitForStart();
        while (opModeIsActive()) {

             position = encoder.getVoltage() / 3.2 * 360;
            telemetry.addData("Encoder", position);
            telemetry.addData("Test", 3);

        }
    }
}