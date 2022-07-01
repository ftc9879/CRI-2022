package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotorEx intake;
    private Servo frontIntake;
    private Servo rearIntake;

    private double intakePower;

    private boolean intakeOn = false;
    private boolean intakePrev = false;

    private double frontIntakeBack;
    private double frontIntakeForward;
    private double rearIntakeBack;
    private double rearIntakeForward;

    private boolean frontRunning = false;
    private boolean rearRunning = false;
    private boolean retractingFront = false;
    private boolean retractingRear = false;

    private ElapsedTime timer;

    public Intake(HardwareMap hardwareMap, double intakePower, double frontIntakeBack, double frontIntakeForward,
                  double frontIntakeStart, double rearIntakeBack, double rearIntakeForward, double rearIntakeStart){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontIntake = hardwareMap.get(Servo.class, "frontIntake");
        rearIntake = hardwareMap.get(Servo.class, "rearIntake");

        frontIntake.setPosition(frontIntakeStart);
        rearIntake.setPosition(rearIntakeStart);

        this.frontIntakeBack = frontIntakeBack;
        this.rearIntakeBack = rearIntakeBack;
        this.frontIntakeForward = frontIntakeForward;
        this.rearIntakeForward = rearIntakeForward;

        this.intakePower = intakePower;

        timer = new ElapsedTime();
    }

    public void update(boolean front, boolean rear, boolean retract){
        if (front){
            frontIntake.setPosition(frontIntakeForward);
            rearIntake.setPosition(rearIntakeBack);
            intake.setPower(-this.intakePower);
            frontRunning = true;
            rearRunning = false;
        } else if (rear){
            rearIntake.setPosition(rearIntakeForward);
            frontIntake.setPosition(frontIntakeBack);
            intake.setPower(this.intakePower);
            frontRunning = false;
            rearRunning = true;
        } else if (retract){
            frontIntake.setPosition(frontIntakeBack);
            rearIntake.setPosition(rearIntakeBack);
            intake.setPower(0);
            if (frontRunning){
                retractingFront = true;
                timer.reset();
            } else if (rearRunning){
                retractingRear = true;
                timer.reset();
            }

            frontRunning = false;
            rearRunning = false;
        }

        if (retractingFront && timer.time() > 1){
            intake.setPower(this.intakePower);
            if (timer.time() > 1.75){
                intake.setPower(0);
                retractingFront = false;
            }
        } else if (retractingRear && timer.time() > 1){
            intake.setPower(-this.intakePower);
            if (timer.time() > 1.75){
                intake.setPower(0);
                retractingRear = false;
            }
        }
    }
}
