package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Utilities {
    private static final double MAX_MOTOR_CURRENT = 20;
    public void checkPower(DcMotorEx motor) {
        double power = motor.getPower();
        if(motor.getCurrent(CurrentUnit.AMPS) > MAX_MOTOR_CURRENT) {
            motor.setPower(power * MAX_MOTOR_CURRENT / motor.getCurrent(CurrentUnit.AMPS));
        }
        else if(motor.getCurrent(CurrentUnit.AMPS) > MAX_MOTOR_CURRENT * 3 / 4) {
            motor.setPower(1);
        }
    }
}
