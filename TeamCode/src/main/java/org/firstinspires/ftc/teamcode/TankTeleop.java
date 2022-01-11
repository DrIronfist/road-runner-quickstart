package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "test Tank", group = "TeleOp")
public class TankTeleop extends OpMode {

    boolean carouselDirection = true;

    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {new CustomMotor("leftFront"),new CustomMotor("rightFront"),new CustomMotor("leftBack"),new CustomMotor("rightBack"),new CustomMotor("rotateArm"),new CustomMotor("intake")};
    Servo leftCarousel;
    Servo rightCarousel;
    double timeLastPressed = 0;
    boolean pressed = false;
    double velocityMultiplier = 1;
    @Override
    public void init() {
        leftCarousel = hardwareMap.get(Servo.class, "leftCarousel");
        rightCarousel = hardwareMap.get(Servo.class, "rightCarousel");

        //Motor Initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");
        motors[4].motor = hardwareMap.get(DcMotorEx.class, "rotating Arm");
        motors[5].motor = hardwareMap.get(DcMotorEx.class, "intake");

        //Motor Direction
        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[4].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[5].motor.setDirection(DcMotorEx.Direction.FORWARD);

        //Motor Zero Power Behavior
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Motor PID Coefficients
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);

    }
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        //Velocity Multiplier

        if(pressed&& !gamepad1.right_bumper) {
            pressed = false;
        } else if(pressed && gamepad1.right_bumper && runtime.time() - timeLastPressed > 0.25){
            if (velocityMultiplier == 1) {
                velocityMultiplier = 0.2;
            } else {
                velocityMultiplier = 1;
            }
            pressed = false;
            timeLastPressed = 0;

            gamepad1.rumble(100);
        }

        if (gamepad1.right_bumper) {
            if(!pressed){
                pressed = true;
                timeLastPressed = runtime.time();
            }
        }

        double r = -gamepad1.left_stick_y; //forward
        double y = gamepad1.right_stick_x; //turn

        //Individual Wheel Velocity
        double[] velocity = {
                (y + r),    //Front Left
                (y - r),    //Front Right
                (y + r),    //Back Left
                (y - r)     //Back Right
        };

        double highestValue = 0;
        for (double ix : velocity) {
            if (Math.abs(ix) > highestValue) {
                highestValue = Math.abs(ix);
            }
        }

        if (highestValue > 1) {
            for (double ix : velocity) {
                ix /= highestValue;
            }
        }

        //Velocity Multiplier, Motor RPM
        for (int i = 0; i < 4; i++) {
            motors[i].motor.setPower(velocity[i]*velocityMultiplier);
        }

        //Carousel
        if(gamepad1.right_bumper){
            carouselDirection = !carouselDirection;
            leftCarousel.setPosition(1);
            rightCarousel.setPosition(1);
        } else if (gamepad1.left_bumper) {
            leftCarousel.setPosition(-1);
            rightCarousel.setPosition(-1);
        } else {
            leftCarousel.setPosition(0);
            rightCarousel.setPosition(0);
        }

        //Rotating Arm
        if(Math.abs(gamepad2.right_stick_y) > 0){
            telemetry.addData("arm", gamepad2.right_stick_y);
            motors[4].motor.setVelocity(gamepad2.right_stick_y*2500);
        } else {
            motors[4].motor.setPower(0);
        }

        //Intake
        if (gamepad2.right_trigger>0) {
            motors[5].motor.setVelocity(5000*gamepad2.right_trigger);
        } else if (gamepad2.left_trigger>0) {
            motors[5].motor.setVelocity(-5000*gamepad2.left_trigger);
        } else {
            motors[5].motor.setVelocity(0);
        }

        //Captions, Information Display
        telemetry.addData("Status",             "Run Time: " + runtime.toString());
        telemetry.addData("a: ",                       velocityMultiplier);
        telemetry.addData("FRONT LEFT Motor",   motors[0].motor.getVelocity() + "rps");
        telemetry.addData("FRONT RIGHT Motor",  motors[1].motor.getVelocity() + "rps");
        telemetry.addData("BACK LEFT Motor",    motors[2].motor.getVelocity() + "rps");
        telemetry.addData("BACK RIGHT Motor",   motors[3].motor.getVelocity() + "rps");
        telemetry.addData("Carousel",           motors[4].motor.getVelocity() + "rps");
        telemetry.addData("Rotating Arm",       motors[5].motor.getPower() + "rps");
        telemetry.addData("Intake",             motors[6].motor.getVelocity() + "rps");

        telemetry.update();
    }

}
