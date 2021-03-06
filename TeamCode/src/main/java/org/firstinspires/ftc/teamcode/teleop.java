/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class teleop extends LinearOpMode {

    // hardware class instance
    private hardware hardware;


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

    private void miscaBratIntake(float miscareBratIntake)
    {
        if(miscareBratIntake != 0)
        {
            if(miscareBratIntake > 0)
            {
                hardware.motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                hardware.motorIntake.setPower(miscareBratIntake * specifications.intake_speed);
            }
            else
            {
                hardware.motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                hardware.motorIntake.setPower(-miscareBratIntake * specifications.intake_speed);
            }
        }
    }

    private void miscaMaturiCuva(float intakeCuva, float outtakeCuva)
    {
        if(intakeCuva != 0 && outtakeCuva == 0)
        {
            hardware.motorCuva.setDirection(DcMotorSimple.Direction.FORWARD);
            hardware.motorCuva.setPower(intakeCuva * specifications.motor_cuva_speed);
        }
        else if(outtakeCuva != 0 && intakeCuva == 0)
        {
            hardware.motorCuva.setDirection(DcMotorSimple.Direction.REVERSE);
            hardware.motorCuva.setPower(outtakeCuva * specifications.motor_cuva_speed);
        }
    }

    @Override
    public void runOpMode()
    {
        //verify teleop stage
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // init hardware
        hardware = new hardware(hardwareMap);

        while(opModeIsActive())
        {
            // drivers' moving requests
            float fata = gamepad1.left_stick_y; // forward movement
            float lateral = gamepad1.left_stick_x; // strafe movement
            float rotire = gamepad1.right_stick_x; // rotate

            // drivers' intake requests
            float miscareBratIntake = gamepad2.right_stick_y; //comanda bratului de intake
            float intakeCuva =  gamepad2.right_trigger;
            float outtakeCuva =  gamepad2.left_trigger;



            // moving the robot depending on drivers' requests
            miscare(fata, lateral, rotire);

            // miscarea brat intake
            miscaBratIntake(miscareBratIntake);

            // miscare maturi cuva
            miscaMaturiCuva(intakeCuva, outtakeCuva);






        }
    }

}
