package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

public class PID {
    public double kP;
    public double kI;
    public double kD;
    public double ticksPerRev;

    public double lastTime = 0;
    public double maxI;
    public double lastVelocity = 0;
    public double i = 0;
    public boolean oscillationsOccuring = false;
    public int occurences = 0;
    public ArrayList<Double> lastErrors;
    public double lastError = 0;
    public double lastPos = 0;
    PID(double p, double i, double d, double gearRatio, double maxI){
        kP = p;
        kI = i;
        kD = d;
        ticksPerRev = 28.0*gearRatio;
        this.maxI = maxI;
        lastErrors = new ArrayList<>();
    }
    public void setTimeBeforeStart(double currentTime){
        lastTime = currentTime;
    }
    public double calculate(double currentTime, double currentPos, double wantedVelocity){
        double currentVelocity = ((currentPos-lastPos)/ticksPerRev*2)/(currentTime-lastTime);//*2 because I want it in radians/pi
        double velocityError = wantedVelocity - currentVelocity;
        double p = kP * velocityError;
        i += kI*(velocityError*(currentTime - lastTime));
        if(i>maxI){
            i = maxI;
        } else if(i < -maxI){
            i = -maxI;
        }
        double d = kD*(velocityError- lastError)/(currentTime-lastTime);

        //setting variables for next call
        lastPos = currentPos;
        lastError = velocityError;
        lastTime = currentTime;

        return p+i+d;
    }

    public void adjust(@NonNull DcMotorEx motor, double wantedVelocity){
        PIDFCoefficients coeffs = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        double velocity = motor.getVelocity()/ticksPerRev*2;
        double velocityError = wantedVelocity - velocity;
        if(wantedVelocity >= 0){
            if(velocityError < 0){
                oscillationsOccuring = true;
                occurences = 0;
            } else{
                occurences++;
            }
        } else{
            if(velocityError > 0){
                oscillationsOccuring = true;
                occurences = 0;
            } else{
                occurences++;
            }
        }
        if(lastErrors.size() > 10){
            lastErrors.add(new Double(velocityError));
        } else{
            lastErrors.add(0,new Double(velocityError));
            lastErrors.remove(lastErrors.size() -1 );
            lastErrors.add(0,new Double(velocityError));
        }
        





    }
}
