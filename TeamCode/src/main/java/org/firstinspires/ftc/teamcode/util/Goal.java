package org.firstinspires.ftc.teamcode.util;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

public class Goal {
    private Servo[] Servos;
    private DcMotor[] Motors;
    private int[] Positions;
    private final ReentrantLock lock = new ReentrantLock();
    private final Condition motorsNotBusy = lock.newCondition();

    private boolean hardwareIsBusy() {
        // checks if the motors as busy (if you can make it check the servos too please do!)
        for (DcMotor motor : Motors) {
            if (motor.getCurrentPosition() != motor.getTargetPosition() || motor.isBusy()) {
                return true;
            }
        }
        return false;
    }
    private void waitForHardwareToStop(){
        // this function will stop the thread it is called in until the hardware has stopped
        lock.lock();
        try {
            while (hardwareIsBusy()) {
                try {
                    motorsNotBusy.await();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        } finally {
            lock.unlock();
        }
    }
>>>>>>> refs/rewritten/origin-main

    public Goal(DcMotor[] m_motors, Servo[] s_servo, int[] iPositions) {
        this.motors = m_motors;
        this.positions = iPositions;
        this.servos = s_servo;
    }

<<<<<<< HEAD
    public void RunToGoal(double power, double waitAfter) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power);
            motors[i].setTargetPosition(positions[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(positions[motors.length + i]);
=======
    public void RunToGoal(double power, double waitAfter) throws InterruptedException {
        // iterate over motors and set target pos and power them up
        for (int i = 0; i < Motors.length; i++) {
            Motors[i].setPower(power);
            Motors[i].setTargetPosition(Positions[i]);
            Motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // iterate over servos and set target pos for those
        for (int i = 0; i < Servos.length; i++) {
            Servos[i].setPosition(Positions[Motors.length + i]);
>>>>>>> refs/rewritten/origin-main
        }

        // wait for hardware to stop
        waitForHardwareToStop();
        wait((long) waitAfter);
    }

    public boolean isBusy() {
        if (motors.length != positions.length) {
            throw new IllegalStateException("Motor and target position arrays must be the same size");
        }
        for (int i = 0; i < motors.length; i++) {
            if (motors[i].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                if (motors[i].isBusy()) {
                    return true;
                }
            } else {
                if (motors[i].getCurrentPosition() != positions[i]) {
                    return true;
                }
            }
        }
        return false;
    }
}
