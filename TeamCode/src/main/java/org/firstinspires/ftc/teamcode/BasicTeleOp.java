package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Basic mecanum drive TeleOp. Left stick on gamepad1 drives translation
 * (forward/back and strafe), right stick x drives rotation.
 *
 * <p>Hardware setup: configure four DcMotors named {@code leftFront},
 * {@code rightFront}, {@code leftRear}, {@code rightRear}.
 */
@TeleOp(name = "Basic TeleOp", group = "drive")
public class BasicTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftRearPower = axial - lateral + yaw;
            double rightRearPower = axial + lateral - yaw;

            double max = Math.max(1.0, Math.max(
                    Math.abs(leftFrontPower), Math.max(
                            Math.abs(rightFrontPower), Math.max(
                                    Math.abs(leftRearPower), Math.abs(rightRearPower)))));

            leftFront.setPower(leftFrontPower / max);
            rightFront.setPower(rightFrontPower / max);
            leftRear.setPower(leftRearPower / max);
            rightRear.setPower(rightRearPower / max);

            telemetry.addData("axial", axial);
            telemetry.addData("lateral", lateral);
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }
    }
}
