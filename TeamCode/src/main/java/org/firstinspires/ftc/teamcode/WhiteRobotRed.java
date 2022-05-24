package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WhiteRobotRed extends OpMode {

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
    double rightsticky;
    double r;
    double robotangle;
    double rightX;
    double divide;
    double minPower = 0;
    double maxPower = .70;
    double accelerationTime = .5;
    double totalTime = 2;
    boolean delivering;
        ElapsedTime timer; 

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
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    deliveryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    delivering = false; 
    timer = new ElapsedTime();

}
@Override
public void loop() {
        leftsticky = -gamepad1.left_stick_y;
        rightstickx = gamepad1.right_stick_x;
        rightsticky = -gamepad1.right_stick_y;
        leftBackpower = Range.clip(leftsticky + rightstickx*.75, -1.0, 1.0);
        leftFrontpower = Range.clip(leftsticky + rightstickx*.75, -1.0, 1.0);
        rightBackpower = Range.clip(leftsticky - rightstickx*.75, -1.0, 1.0);
       rightFrontpower = Range.clip(leftsticky - rightstickx*.75, -1.0, 1.0);
    
/*        leftBackpower = leftsticky;
        leftFrontpower = leftsticky;
        rightBackpower = rightsticky;
        rightFrontpower = rightsticky; */
        
        //cap scoring position
    
        
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
        if(gamepad2.b){
            delivering = true;
        }

        
        quitIntake();
        
        if (delivering){
            timer.reset();
            while (timer.time() < totalTime) {
                quitIntake();
                while (timer.time() < accelerationTime){
                    quitIntake();
                    deliveryMotor.setPower((minPower+(-1*(maxPower-minPower)/accelerationTime*timer.time())));
                }
                    deliveryMotor.setPower(-maxPower);
            }
            
            deliveryMotor.setPower(0);
            timer.reset();
            while (timer.time() < .5){
                quitIntake();
            }
        }
        
   /*     else if (gamepad2.dpad_right){
            
            timer.reset();
            while (timer.time() < totalTime) {
                while (timer.time() < accelerationTime){
                deliveryMotor.setPower(-1*(minPower+((maxPower-minPower)/accelerationTime*timer.time())));
            }
                deliveryMotor.setPower(-maxPower);
            }
            if (gamepad2.x){
            deliveryMotor.setPower(0);
                
            }
        } */
        else {
                deliveryMotor.setPower(0);
        }
        if (gamepad2.x){
            armServo.setPosition(0);
        }
        else if (gamepad2.y) {
            armServo.setPosition(.7);
        }
        else if(gamepad2.left_bumper)
        {
            armServo.setPosition(.235);
            
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
        else if (gamepad2.dpad_down & lift.getCurrentPosition()> 100){
                lift.setPower(-.6);
        }
        else if (gamepad1.dpad_down){
                lift.setPower(-.6);
        }
        else {
                lift.setPower(0);
        }

        
        if(gamepad1.dpad_left){
           lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        leftFront.setPower(leftFrontpower);
        leftBack.setPower(leftBackpower);
        rightFront.setPower(rightFrontpower);
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
        }
        
        public void quitIntake(){
            if (gamepad2.x){
            deliveryMotor.setPower(0);
            delivering = false;
                
            }
        
        }
 
              
    }
       
