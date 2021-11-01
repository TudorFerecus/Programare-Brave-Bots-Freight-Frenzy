package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.odometryGlobalCoordinatePosition;


@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class globalCoordinatePositionUpdateSample extends LinearOpMode
{
    private hardware hardware;
    @Override
    public void runOpMode() throws InterruptedException
    {
        // init hardware
        hardware = new hardware(hardwareMap);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        hardware.encDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.encStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.encOrizontal.setDirection(DcMotorSimple.Direction.REVERSE);


        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        odometryGlobalCoordinatePosition globalPositionUpdate = new odometryGlobalCoordinatePosition(hardware.encStanga, hardware.encDreapta, hardware.encOrizontal, specifications.counts_per_inch, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / specifications.counts_per_inch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / specifications.counts_per_inch);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}