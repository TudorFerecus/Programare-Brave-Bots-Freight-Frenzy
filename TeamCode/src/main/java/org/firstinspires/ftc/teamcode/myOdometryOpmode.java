package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.odometryGlobalCoordinatePosition;


@TeleOp(name = "My Odometry OpMode")
public class myOdometryOpmode extends LinearOpMode {

    private hardware hardware;
    private odometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values.
        hardware = new hardware(hardwareMap);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new odometryGlobalCoordinatePosition(hardware.encStanga, hardware.encDreapta, hardware.encOrizontal, specifications.counts_per_inch, specifications.odometry_delay);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //se va modifica dupa teste
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / specifications.counts_per_inch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / specifications.counts_per_inch);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", hardware.encStanga.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", hardware.encDreapta.getCurrentPosition());
            telemetry.addData("horizontal encoder position", hardware.encOrizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }


    public void goToPosition(double targetX, double targetY, double power, double desiredOrientation, double allowableDistanceError)
    {
        double distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToX, distanceToY);

        while(opModeIsActive() && distance > allowableDistanceError)
        {
            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            double x_movement_component = calculateX(movementAngle, power);
            double y_movement_component = calculateY(movementAngle, power);
            double pivotCorrection = desiredOrientation - globalPositionUpdate.returnOrientation();
        }
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
