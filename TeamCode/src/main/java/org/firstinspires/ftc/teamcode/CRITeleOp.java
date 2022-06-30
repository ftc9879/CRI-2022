package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.delivery.Delivery;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.lift.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class CRITeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Delivery delivery = new Delivery(hardwareMap, -1, 1);
        Lift lift = new Lift(hardwareMap, 1, 1, 3000,
                0, 1, 0.03, 0.98, 0.63, 0.73 ,0.83);
        Intake intake = new Intake(hardwareMap, 1, 0, 1, 1, 0);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Ready, press start to begin", ":)");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    )
            );

            drive.update();

            delivery.frontUpdate(gamepad1.x);
            delivery.rearUpdate(gamepad1.a);

            // lift.update(gamepad1.dpad_up, gamepad1.dpad_down);

            lift.score(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);

            lift.dump(gamepad1.y);

            intake.update(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_right);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Lift pos", lift.getCounts());
            telemetry.addData("Servo Data", lift.getServoData());
            telemetry.update();
        }
    }
}
