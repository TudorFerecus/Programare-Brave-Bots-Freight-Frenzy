package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.specifications;

public class odometry {
    public double x, y, theta; // coordonates
    private double current_enc_dreapta = 0, current_enc_stanga = 0, current_enc_orizontal = 0; //readings from the 3 encoders
    private double prev_enc_dreapta = 0, prev_enc_stanga = 0, prev_enc_orizontal = 0; //previous readings

    private hardware hardware;

    //give the start position of the robot
    public odometry(double x, double y, double theta, hardware hardware)
    {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.hardware = hardware;
    }

    public void updatePosition()
    {
        prev_enc_dreapta = current_enc_dreapta;
        prev_enc_stanga = current_enc_stanga;
        prev_enc_orizontal = current_enc_orizontal;

        current_enc_dreapta = -hardware.encDreapta.getCurrentPosition();
        current_enc_stanga = -hardware.encStanga.getCurrentPosition();
        current_enc_orizontal = hardware.encOrizontal.getCurrentPosition();

        double enc_dreapta = current_enc_dreapta - prev_enc_dreapta;
        double enc_stanga = current_enc_stanga - prev_enc_stanga;
        double enc_orizontal = current_enc_orizontal - prev_enc_orizontal;

        double dtheta = specifications.cm_per_tick * (enc_dreapta - enc_stanga) / 2;
        double dx = specifications.cm_per_tick * (enc_stanga + enc_dreapta) / 2;
        double dy = specifications.cm_per_tick * (enc_orizontal - specifications.B * (enc_dreapta - enc_stanga) / 2);

        //might be errors here
        x += dx * Math.cos(dtheta) - dy * Math.sin(dtheta);
        y += dy * Math.cos(dtheta) + dx * Math.sin(dtheta);
        theta += dtheta;
    }
}
