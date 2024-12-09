package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * A utility class to contain two <code>DcMotor</code>s and use them together.
 * Contains most methods of <code>DcMotor</code>.
 *
 */

public class DualMotor {
    private static String noMotorEx = "Ninevolt: No motors remaining!";
    private DcMotor motor1;
    private DcMotor motor2;
    private boolean isSingleMotor = false;
    private PIDController PID;
    private double Kp, Ki, Kd;

    /**
     * Constructs a DcMotorPair containing and controlling at least one motor.
     * @param motor1 The first motor to be controlled by this pair
     * @param motor2 The second motor to be controlled by this pair
     * @throws Exception Thrown when no motors are provided
     */
    public DualMotor(DcMotorEx motor1, DcMotorEx motor2) throws Exception {
        this.motor1 = motor1;
        this.motor2 = motor2;
        if (motor1 == null ^ motor2 == null) {
            isSingleMotor = true;
        } else if (motor1 == null) {
            throw new Exception(noMotorEx);
        }
        //can put actual defaults for these later
        Kp = 0;
        Ki = 0;
        Kd = 0;
        PID = new PIDController(Kp, Ki, Kd);
    }
    /**
     * Constructs a DcMotorPair containing and controlling at least one motor, with constants for PID control.
     * @param motor1 The first motor to be controlled by this pair
     * @param motor2 The second motor to be controlled by this pair
     * @param K_P The PID constant for proportional
     * @param K_I The PID constant for integral
     * @param K_D The PID constant for derivative
     * @throws Exception Thrown when no motors are provided
     */
    public DualMotor(DcMotorEx motor1, DcMotorEx motor2, double K_P, double K_I, double K_D) throws Exception {
        this.motor1 = motor1;
        this.motor2 = motor2;
        if (motor1 == null ^ motor2 == null) {
            isSingleMotor = true;
        } else if (motor1 == null) {
            throw new Exception(noMotorEx);
        }
        Kp = K_P;
        Ki = K_I;
        Kd = K_D;
        PID = new PIDController(Kp, Ki, Kd);
    }

    /**
     * Constructs a DcMotorPair controlling one motor.
     * @param motor1 - The DcMotor to be controlled by this motor pair.
     */
    public DualMotor(DcMotor motor1) {
        this.motor1 = motor1;
        isSingleMotor = true;
        //can put actual defaults for these later
        Kp = 0;
        Ki = 0;
        Kd = 0;
        PID = new PIDController(Kp, Ki, Kd);
    }
    /**
     * Constructs a DcMotorPair controlling one motor, with constants for PID control.
     * @param motor1 - The DcMotor to be controlled by this motor pair.
     * @param K_P The PID constant for proportional
     * @param K_I The PID constant for integral
     * @param K_D The PID constant for derivative
     */
    public DualMotor(DcMotor motor1, double K_P, double K_I, double K_D) {
        this.motor1 = motor1;
        isSingleMotor = true;
        Kp = K_P;
        Ki = K_I;
        Kd = K_D;
        PID = new PIDController(Kp, Ki, Kd);
    }

    /**
     * Gets the first DcMotor controlled by this motor pair.
     * @return The first motor controlled by this pair.
     */
    public DcMotor getMotor1() {
        return motor1;
    }

    /**
     * Gets the second motor controlled by this motor pair.
     * @return The second motor controlled by this pair.
     */
    public DcMotor getMotor2() {
        return motor2;
    }

    /**
     * Sets the first motor controlled by this motor pair.
     * @param motor1 The motor to be controlled by this pair.
     * @throws Exception Thrown if no motors are remaining in this pair.
     */
    public void setMotor1(DcMotor motor1)throws Exception {
        this.motor1 = motor1;
        if (this.motor1 == null && motor2 != null) {
            isSingleMotor = true;
        } else if (this.motor1 != null && motor2 != null) {
            isSingleMotor = false;
        } else {
            throw new Exception(noMotorEx);
        }

    }

    /**
     * Sets the second motor controlled by this motor pair.
     * @param motor2 The motor to be controlled by this pair.
     * @throws Exception Thrown if no motors are remaining in this pair.
     */
    public void setMotor2(DcMotor motor2) throws Exception {
        this.motor2 = motor2;
        if (motor1 != null && this.motor2 == null) {
            isSingleMotor = true;
        } else if (motor1 != null && this.motor2 != null) {
            isSingleMotor = false;
        } else {
            throw new Exception(noMotorEx);
        }
    }

    /**
     * Returns true if this pair only controls one motor.
     * @return true if this pair only controls one motor.
     */
    public boolean isSingleMotor() {
        return isSingleMotor;
    }


    /**
     * Sets the power of all motors in this pair.
     * @param power The power to set to each motor.
     */
    public void setPower(double power) {
        if(motor1 != null)
            motor1.setPower(power);
        if(motor2 != null)
            motor2.setPower(-power);
    }

    /**
     * Sets the direction of all motors in the pair.
     * Calls <code>DcMotor.setDirection(Direction)</code> on each motor.
     * @param direction The <code>DcMotor.Direction</code> to set the direction of the motors to.
     */
    public void setDirection(DcMotor.Direction direction) {
        if(motor1 != null)
            motor1.setDirection(direction);
        if(motor2 != null)
            motor2.setDirection(direction);
    }

    /**
     * Sets the current run mode of all motors in the pair.
     * @param runMode The run mode to set for each motor.
     */
    public void setMode(DcMotor.RunMode runMode) {
        if(motor1 != null)
            motor1.setMode(runMode);
        if(motor2 != null) {
            motor2.setMode(runMode);
        }
    }

    /**
     * Sets the target position of all motors in the pair.
     * @param targetPosition The target position (in encoder ticks) to set each
     *                       motor's target position to.
     */
    public void setTargetPosition(int targetPosition) {
        PID.reset();
        if(motor1 != null)
            motor1.setTargetPosition(targetPosition);
        if(motor2 != null) {
            motor2.setTargetPosition(-targetPosition);
        }
    }

    /**
     * Gets the current position of the first motor if there are two motors,
     * otherwise, gets the current position of the single motor.
     * @return The position of the motor(s) in encoder ticks.
     * @throws Exception Thrown when no motors were found in the current pair.
     */
    public int getCurrentPosition() throws Exception {
        if(motor1 != null)
            //sign weirdness again
            return -motor1.getCurrentPosition();
        else if(motor2 != null)
            return motor2.getCurrentPosition();
        else
            throw new Exception(noMotorEx);
    }

    /**
     * Gets the target position of the first motor if there are two motors,
     * otherwise, the target position of the single motor
     * @return The target position of the motor(s) in encoder ticks
     * @throws Exception Thrown when no motors are found in the current pair
     */
    public int getTargetPosition() throws Exception {
        if(motor1 != null)
            return motor1.getTargetPosition();
        else if(motor2 != null)
            return motor2.getTargetPosition();
        else
            throw new Exception(noMotorEx);
    }

    /**
     * Gets the motor power given by the PID algorithm to reach the
     * target position based on the current position
     * @return The determined power of the motor, between -1.0 and 1.0
     * @throws Exception Thrown when no motors are found in the current pair
     */
    public double getPIDPower() throws Exception {
        try {
            return PID.PIDControl(getTargetPosition(), getCurrentPosition());
        }
        catch(Exception e) {
            throw new Exception(noMotorEx);
        }
    }
}