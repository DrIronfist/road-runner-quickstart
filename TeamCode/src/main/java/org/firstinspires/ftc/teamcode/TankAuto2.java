package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "testTankAutonRed", group = "test")
public class TankAuto2 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    IMUController imuController = null;
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightFront"),              //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("leftBack"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightBack"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("carousel"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rotateArm"),                //new PIDCoefficients(15, 0, 1))
            new CustomMotor("intake")
    };

    @Override
    public void runOpMode(){
        //Motor Initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");
        motors[4].motor = hardwareMap.get(DcMotorEx.class, "carousel");
        motors[5].motor = hardwareMap.get(DcMotorEx.class, "rotating Arm");
        motors[6].motor = hardwareMap.get(DcMotorEx.class, "intake");


        //Motor Direction
        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[4].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[5].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[6].motor.setDirection(DcMotorEx.Direction.REVERSE);

        //Motor Zero Power Behavior
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[6].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Motor PID Coefficients
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[6].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);


        waitForStart();
        runtime.reset();
        double start = runtime.seconds();
        boolean finished = false;
        do{
            telemetry.addData("time", runtime.toString());
            telemetry.addData("nan", runtime.nanoseconds());
            telemetry.addData("seconds",runtime.seconds());

            finished = AutonMethods.Move(motors,0,-0.3,runtime.seconds(), start, 1.5);
            telemetry.addData("finished", finished);
            telemetry.update();
        } while(opModeIsActive() && !finished);
         start = runtime.seconds();
         finished = false;
        do{
            telemetry.addData("time", runtime.toString());
            telemetry.addData("nan", runtime.nanoseconds());
            telemetry.addData("seconds",runtime.seconds());

            finished = AutonMethods.Move(motors,-0.3,0,runtime.seconds(), start, 1.7);
            telemetry.addData("finished", finished);
            telemetry.update();
        } while(opModeIsActive() && !finished);
        start = runtime.seconds();
        finished = false;
        do{
            telemetry.addData("time", runtime.toString());
            telemetry.addData("nan", runtime.nanoseconds());
            telemetry.addData("seconds",runtime.seconds());

            finished = AutonMethods.carousel(motors,runtime.seconds(), start, 10);
            telemetry.addData("finished", finished);
            telemetry.update();
        } while(opModeIsActive() && !finished);
        while(opModeIsActive()){
            telemetry.addData("time", runtime.toString());
            telemetry.addData("nan", runtime.nanoseconds());
            telemetry.update();
        }



    }
}
