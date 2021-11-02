package org.firstinspires.ftc.teamcode;

public class PID {

    private double kp = 0 , ki = 0, kd = 0;
    private double target = 0;
    private double integral = 0;
    private double prevError = 0;
    private double dt = 0.03; //delta time
    private double mx = 1, mn =-1; //used for clamping
    private double tolerance = 10;

    public PID(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public PID(){}

    public void setKs(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setTarget(double target)
    {
        this.reset();
        this.target=target;
    }

    //set clamping values
    public void setBoundaries(double mx, double mn)
    {
        this.mx = mx;
        this.mn = mn;
    }

    public void setTargetWithoutReset(double target)
    {
        this.target=target;
    }

    public double output(double input)
    {
        if(!done()) {
            double error = target - input;
            integral += error * dt;
            double out = kp * error + ki * integral + kd * (error - prevError) / dt;
            //oldError = prevError;
            prevError = error;
            out = clamp(out/100);
            return out;
        }
        return 0;
    }

    public void reset()
    {
        //this.target=0;
        this.integral=0;
        this.prevError=0;
    }


    public boolean done()
    {
        /*
        if(Math.abs(prevError) <= tolerance && oldError<=prevError)
            return true;
        return false;

         */
        if(Math.abs(prevError) <= tolerance && prevError!=0)
            return true;
        return false;
    }

    public void setTolerance(double t)
    {
        tolerance = t;
    }

    private double clamp(double value)
    {
        if (value > mx) return mx;
        else if (value < mn) return mn;
        return value;
    }
}
