package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.specifications;

public class odometry implements Runnable {
    private double x, y, theta; // coordonates

    //readings from the 3 encoders
    private double current_enc_dreapta = 0;
    private double current_enc_stanga = 0;
    private double current_enc_orizontal = 0;

    //previous encoder readings
    private double prev_enc_dreapta = 0;
    private double  prev_enc_stanga = 0;
    private double prev_enc_orizontal = 0;

    private hardware hardware;

    private boolean isRunning = true; //thread running condition
    private int sleepTime; //pause between execution of thread

    //set the start position of the robot (origin), hardware, and thread delay
    public odometry(double x, double y, double theta, hardware hardware, int threadSleepDelay)
    {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.hardware = hardware;
        this.sleepTime = threadSleepDelay;
    }

    //getters
    double getX() {return this.x;}
    double getY() {return this.y;}
    double getAngle() {return this.theta;}//returns angle in radians

    //updates x, y and theta
    public void updatePosition()
    {
        //store previous readings
        prev_enc_dreapta = current_enc_dreapta;
        prev_enc_stanga = current_enc_stanga;
        prev_enc_orizontal = current_enc_orizontal;

        //get new readings
        current_enc_dreapta = -hardware.encDreapta.getCurrentPosition();
        current_enc_stanga = -hardware.encStanga.getCurrentPosition();
        current_enc_orizontal = hardware.encOrizontal.getCurrentPosition();

        //difference in encoder readings
        double enc_dreapta = current_enc_dreapta - prev_enc_dreapta;
        double enc_stanga = current_enc_stanga - prev_enc_stanga;
        double enc_orizontal = current_enc_orizontal - prev_enc_orizontal;

        //calculate difference in field coordinates
        double dtheta = specifications.cm_per_tick * (enc_dreapta - enc_stanga) / 2;
        double dx = specifications.cm_per_tick * (enc_stanga + enc_dreapta) / 2;
        double dy = specifications.cm_per_tick * (enc_orizontal - specifications.B * (enc_dreapta - enc_stanga) / 2);

        //TODO:watch out for errors
        //add to the global coordonates
        x += dx * Math.cos(dtheta) - dy * Math.sin(dtheta);
        y += dy * Math.cos(dtheta) + dx * Math.sin(dtheta);
        theta += dtheta;
    }



    //thread function
    @Override
    public void run() {
        while (isRunning) {
            updatePosition();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
