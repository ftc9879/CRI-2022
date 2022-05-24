package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SilverRobot2 extends OpMode {

DcMotor rightFront;
DcMotor rightBack;
DcMotor leftFront;
DcMotor leftBack;
DcMotor lift; 
DcMotor deliveryMotor; 
DcMotor intakeMotor; 
Servo armServo;
Servo clawServo; 
    double leftFrontpower;
    double rightFrontpower;
    double leftBackpower;
    double rightBackpower;
    double leftsticky;
    double leftstickx;
    double rightstickx;
    double r;
    double robotangle;
    double rightX;
    double divide;
    @Override
public void init() {

   leftFront = hardwareMap.dcMotor.get("LF");
   rightFront = hardwareMap.dcMotor.get("RF");
   leftBack = hardwareMap.dcMotor.get("LB");
   rightBack = hardwareMap.dcMotor.get("RB");
   lift = hardwareMap.dcMotor.get("L");
   deliveryMotor = hardwareMap.dcMotor.get("DM");
   intakeMotor = hardwareMap.dcMotor.get("IM");
   armServo = hardwareMap.get(Servo.class, "AS");
   clawServo = hardwareMap.get(Servo.class, "CS");
   leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

   
}
@Override
public void loop() {
        leftstickx = -gamepad1.left_stick_x;
        leftsticky = gamepad1.left_stick_y;
        rightstickx = gamepad1.right_stick_x;
        r = Math.hypot(leftstickx, leftsticky);
        robotangle = Math.atan2(leftsticky, leftstickx) - Math.PI / 4;
        rightX = rightstickx;
        leftFrontpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) - rightX;
        leftBackpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) - rightX;
        rightFrontpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) + rightX;
        rightBackpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) + rightX;
        if (leftFrontpower > 1 || leftFrontpower < -1) {
            divide = Math.abs(leftFrontpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (leftBackpower > 1 || leftBackpower < -1) {
            divide = Math.abs(leftBackpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (rightFrontpower > 1 || rightFrontpower < -1) {
            divide = Math.abs(rightFrontpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (rightBackpower > 1 || rightBackpower < -1) {
            divide = Math.abs(rightBackpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        }
        if(gamepad1.x){
                intakeMotor.setPower(-1);
        }
        else if (gamepad1.y){
                intakeMotor.setPower(1);
        }
        else if (gamepad1.b){
                intakeMotor.setPower(.5);
        }
        else if (gamepad1.a){
                intakeMotor.setPower(0);
        }
        if(gamepad2.dpad_left){
                deliveryMotor.setPower(.75);
        }
        else if (gamepad2.dpad_right){
                deliveryMotor.setPower(-.75);
        }
        else {
                deliveryMotor.setPower(0);
        }
        if (gamepad2.x){
            armServo.setPosition(1);
        }
        else if (gamepad2.y) {
            armServo.setPosition(0);
        }
        if (gamepad2.b){
            clawServo.setPosition(1);
        }
        else if (gamepad2.a){
            clawServo.setPosition(0);
        }
/*        if (gamepad2.x){
                if(lift.getCurrentPosition()<450){
                        while(lift.getCurrentPosition()<450){
                               // drive();
                                                 leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                                intake();
                                deliver();
                                lift.setPower(1);
                        }
                }
                else if(lift.getCurrentPosition()>550){
                        while(lift.getCurrentPosition()>550){
                            //    drive();
                                              leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                                intake();
                                deliver();
                                lift.setPower(-.5);
                        }
                }
        }
        else if (gamepad2.y){
                if(lift.getCurrentPosition()<850){
                        while(lift.getCurrentPosition()<850){
                           //     drive();
                                             leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                                intake();
                                deliver();
                                lift.setPower(1);
                        }
                }
                else if(lift.getCurrentPosition()>900){
                        while(lift.getCurrentPosition()>900){
                            //    drive();
                                              leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                                intake();
                                deliver();
                                lift.setPower(-.5);
                        }
                }
        }
        else if (gamepad2.b){
                if(lift.getCurrentPosition()<1200){
                        while(lift.getCurrentPosition()<1200){
                            //    drive();
                                              leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                                intake();
                                deliver();
                                lift.setPower(1);
                        }
                }
                else if(lift.getCurrentPosition()>1100){
                        while(lift.getCurrentPosition()>1100){
                            //    drive();
                                              leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                                intake();
                                deliver();
                                lift.setPower(-.5);
                        }
                }
        }
        else if (gamepad2.a){
                while(lift.getCurrentPosition()>2){
                  //      drive();
                  leftFront.setPower(0);
                  rightFront.setPower(0);
                  leftBack.setPower(0);
                  rightBack.setPower(0);
                        intake();
                        deliver();
                        if (lift.getCurrentPosition()<300){
                            lift.setPower(-0.75);
                        } else {
                            lift.setPower(-.5);        
                        }
                        
                }
        }*/
        if (gamepad2.dpad_up){
                lift.setPower(1);
        }
        else if (gamepad2.dpad_down){
                lift.setPower(-.75);
        }
        else {
                lift.setPower(0);
        }

        
        if(gamepad1.dpad_left){
           lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        leftFront.setPower(-leftFrontpower);
        rightFront.setPower(rightFrontpower);
        leftBack.setPower(-leftBackpower);
        rightBack.setPower(rightBackpower);
        telemetry.addData("leftFront", leftFrontpower);
        telemetry.addData("rightFront", rightFrontpower);
        telemetry.addData("leftBack", leftBackpower);
        telemetry.addData("rightBack", rightBackpower);
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.update();

        }
        public void intake(){
        if(gamepad1.x){
                intakeMotor.setPower(1);
        }
        else if (gamepad1.y){
                intakeMotor.setPower(-1);
        }
        else if (gamepad1.b){
                intakeMotor.setPower(-.5);
        }
        else if (gamepad1.a){
                intakeMotor.setPower(0);
        }
        }
        public void deliver(){
              if(gamepad2.dpad_left){
                        deliveryMotor.setPower(.75);
        }
                else if (gamepad2.dpad_right){
                        deliveryMotor.setPower(-.75);
        }
                else {
                        deliveryMotor.setPower(0);
        } 
        }
        public void drive(){
                leftstickx = -gamepad1.left_stick_x;
        leftsticky = gamepad1.left_stick_y;
        rightstickx = -gamepad1.right_stick_x*.85;
        r = Math.hypot(leftstickx, leftsticky);
        robotangle = Math.atan2(leftsticky, leftstickx) - Math.PI / 4;
        rightX = rightstickx;
        leftFrontpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) - rightX;
        leftBackpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) - rightX;
        rightFrontpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) + rightX;
        rightBackpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) + rightX;
        if (leftFrontpower > 1 || leftFrontpower < -1) {
            divide = Math.abs(leftFrontpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (leftBackpower > 1 || leftBackpower < -1) {
            divide = Math.abs(leftBackpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (rightFrontpower > 1 || rightFrontpower < -1) {
            divide = Math.abs(rightFrontpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (rightBackpower > 1 || rightBackpower < -1) {
            divide = Math.abs(rightBackpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        }
        }
}