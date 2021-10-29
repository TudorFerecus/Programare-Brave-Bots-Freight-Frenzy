package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class odometryCalibration extends LinearOpMode {


    private double horizontalTickOffset;

    //IMU Sensor
    BNO055IMU imu;
    ElapsedTime timer = new ElapsedTime();
    hardware hardware;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        hardware = new hardware(hardwareMap);

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 90 && opModeIsActive()){
            hardware.motor[0].setPower(-specifications.pivot_speed);
            hardware.motor[1].setPower(-specifications.pivot_speed);
            hardware.motor[2].setPower(specifications.pivot_speed);
            hardware.motor[3].setPower(specifications.pivot_speed);
            if(getZAngle() < 60) {
                setPowerAll(-specifications.pivot_speed, -specifications.pivot_speed,
                        specifications.pivot_speed, specifications.pivot_speed);
            }else{
                setPowerAll(-specifications.pivot_speed/2, -specifications.pivot_speed/2,
                        specifications.pivot_speed/2, specifications.pivot_speed/2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
        */

        // diferenta de pozitie dintre ultimul ciclu si cel curent
        double encoderDifference = Math.abs(hardware.encStanga.getCurrentPosition()) + (Math.abs(hardware.encDreapta.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI * specifications.counts_per_inch);

        horizontalTickOffset = hardware.encOrizontal.getCurrentPosition()/Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -hardware.encStanga.getCurrentPosition());
            telemetry.addData("Vertical Right Position", hardware.encDreapta.getCurrentPosition());
            telemetry.addData("Horizontal Position", hardware.encOrizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-hardware.imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        hardware.motor[0].setPower(rf);
        hardware.motor[1].setPower(rb);
        hardware.motor[2].setPower(lf);
        hardware.motor[3].setPower(lb);
    }

}