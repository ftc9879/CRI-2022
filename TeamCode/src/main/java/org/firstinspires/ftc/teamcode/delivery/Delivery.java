package org.firstinspires.ftc.teamcode.delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Delivery {

    private DcMotor frontDelivery, rearDelivery;

    private boolean frontOn = false;
    private boolean rearOn = false;

    private boolean frontPrev = false;
    private boolean rearPrev = false;

    private double frontPower;
    private double rearPower;

    public Delivery(HardwareMap hardwareMap, double frontPower, double rearPower){
        frontDelivery = hardwareMap.get(DcMotor.class, "frontDelivery");
        rearDelivery = hardwareMap.get(DcMotor.class, "rearDelivery");
        this.frontPower = frontPower;
        this.rearPower = rearPower;
    }

    public void frontUpdate(boolean toggle){
        if (toggle && !frontPrev){
            if (frontOn){
                frontDelivery.setPower(0);
                frontOn = false;
            } else {
                frontDelivery.setPower(this.frontPower);
                frontOn = true;
            }
        }

        frontPrev = toggle;
    }

    public void rearUpdate(boolean toggle){
        if (toggle && !rearPrev){
            if (rearOn){
                rearDelivery.setPower(0);
                rearOn = false;
            } else {
                rearDelivery.setPower(this.rearPower);
                rearOn = true;
            }
        }

        rearPrev = toggle;
    }
}
