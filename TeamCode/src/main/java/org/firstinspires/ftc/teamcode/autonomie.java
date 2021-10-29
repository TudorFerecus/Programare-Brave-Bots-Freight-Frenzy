package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autonomie extends LinearOpMode {

    hardware hardware;
    odometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new hardware(hardwareMap);
        odometry = new odometry(0, 0, 0, hardware);

        //misca robotul pana la x = 10;
        while(odometry.y < 10)
        {
            miscare(0, 1, 0);
            odometry.updatePosition();
        }
    }

    //wheel movement
    private void miscare(float fata, float lateral, float rotire)
    {
        // power applied to the robot wheel by wheel
        double[] power = new double[4];

        power[0] =   (- fata + lateral + rotire)*specifications.moving_speed;   //+
        power[1] =   (+ fata + lateral + rotire)*specifications.moving_speed;   //-
        power[2] =   (- fata - lateral + rotire)*specifications.moving_speed;   //-
        power[3] =   (+ fata - lateral + rotire)*specifications.moving_speed;   //+

        // applying the power
        for(int i=0; i<4; i++)
        {
            hardware.motor[i].setPower(power[i]);
        }
    }
}
