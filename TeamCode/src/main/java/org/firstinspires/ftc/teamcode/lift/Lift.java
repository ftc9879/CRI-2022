package org.firstinspires.ftc.teamcode.lift;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {

    private Motor lift;
    private Servo pivot1;
    private Servo pivot2;
    private Servo scoring;

    private double upPower;
    private double downPower;
    private int outDist;

    private boolean goingUp = false;
    private boolean goingDown = false;

    private double scoringClosed;
    private double scoringOpen;
    private double pivot1Back;
    private double pivot1Forward;
    private double pivot2Low;
    private double pivot2Mid;
    private double pivot2High;

    private boolean dumping = false;
    private boolean dropping = false;

    private ElapsedTime timer;


    public Lift(HardwareMap hardwareMap, double upPower, double downPower, int outDist,
                double scoringClosed, double scoringOpen, double pivot1Back, double pivot1Forward,
                double pivot2Low, double pivot2Mid, double pivot2High) {

        lift = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_435);
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lift.setInverted(true);
        lift.resetEncoder();
        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(0.75);
        lift.setPositionTolerance(10);
        lift.setTargetPosition(0);
        lift.set(0);

        pivot1 = hardwareMap.get(Servo.class, "pivot1");
        pivot2 = hardwareMap.get(Servo.class, "pivot2");
        scoring = hardwareMap.get(Servo.class, "scoring");

        pivot1.setPosition(pivot1Back);
        pivot2.setPosition(pivot2Mid);
        scoring.setPosition(scoringOpen);

        this.upPower = upPower;
        this.downPower = downPower;
        this.outDist = outDist;

        this.scoringClosed = scoringClosed;
        this.scoringOpen = scoringOpen;
        this.pivot1Back = pivot1Back;
        this.pivot1Forward = pivot1Forward;
        this.pivot2Low = pivot2Low;
        this.pivot2Mid = pivot2Mid;
        this.pivot2High = pivot2High;

        timer = new ElapsedTime();

    }

    /*public void update(boolean up, boolean down){
        if (up){
            lift.setPower(this.upPower);
        } else if (down){
            lift.setPower(this.downPower);
        } else {
            lift.setPower(0);
        }
    }*/

    public String getCounts(){
        return new Integer(lift.getCurrentPosition()).toString();
    }

    public String getServoData(){
        return new Double(pivot1.getPosition()).toString() + " " + new Double(pivot2.getPosition()).toString() + " " + new Double(scoring.getPosition()).toString();
    }

    public void score(boolean low, boolean mid, boolean high, boolean down){
        if (low || mid || high){

            if (low){
                lift.setTargetPosition(outDist-800);
            } else if (mid){
                lift.setTargetPosition(outDist-400);
            } else if (high){
                lift.setTargetPosition(outDist);
            }

           // pivot1.setPosition(pivot1Forward);
            goingUp = true;
        } else if (down){
            lift.setTargetPosition(0);
            pivot1.setPosition(pivot1Back);
            pivot2.setPosition(pivot2Mid);
            goingDown = true;
        }

        if (goingUp){
            lift.set(upPower);
        } else if (goingDown) {
            lift.set(downPower);
        }

        if (goingUp && lift.atTargetPosition()){
            lift.stopMotor();
            pivot1.setPosition(pivot1Forward);
            pivot2.setPosition(pivot2High);
            scoring.setPosition(scoringOpen);
            goingUp = false;
        }

        if (goingDown && lift.atTargetPosition()){
            lift.stopMotor();
            goingDown = false;
        }

    }

    public void dump(boolean button){
        if(button){
            pivot2.setPosition(0);
            dumping = true;
            timer.reset();
        }

        if(dumping && timer.time() > 1){
            scoring.setPosition(scoringOpen);
            dropping = true;
            dumping = false;
        }

        if(dropping && timer.time() > 1.5){
            pivot2.setPosition(pivot2Mid);
            dropping = false;
        }

    }
}
