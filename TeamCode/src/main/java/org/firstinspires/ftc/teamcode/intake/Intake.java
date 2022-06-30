package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public Intake(HardwareMap hardwareMap, double intakePower, double frontIntakeBack, double frontIntakeForward,
                  double rearIntakeBack, double rearIntakeForward){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontIntake = hardwareMap.get(Servo.class, "frontIntake");
        rearIntake = hardwareMap.get(Servo.class, "rearIntake");

        frontIntake.setPosition(frontIntakeBack);
        rearIntake.setPosition(rearIntakeBack);

        this.frontIntakeBack = frontIntakeBack;
        this.rearIntakeBack = rearIntakeBack;
        this.frontIntakeForward = frontIntakeForward;
        this.rearIntakeForward = rearIntakeForward;

        this.intakePower = intakePower;
    }

    public void update(boolean front, boolean rear, boolean retract){
        if (front){
            frontIntake.setPosition(frontIntakeForward);
            intake.setPower(this.intakePower);
        } else if (rear){
            rearIntake.setPosition(rearIntakeForward);
            intake.setPower(-this.intakePower);
        } else if (retract){
            frontIntake.setPosition(frontIntakeBack);
            rearIntake.setPosition(rearIntakeBack);
            intake.setPower(0);
        }
    }
}
