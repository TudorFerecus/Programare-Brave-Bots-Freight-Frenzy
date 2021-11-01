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

import android.provider.Settings;

//import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="autonomie_doamneajuta", group="autonomie")
//@Disabled
public class autonomie extends OpMode
{
    // Declare OpMode members.


    @Override
    public void init() {

        initPID();
        telemetry.addData("Status", "Initialized");

    }

    private void ridicabrat(int poz)
    {
        if(sus==poz)
        {
            //wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            wobble.setTargetPosition((int)(2786*0.1));
            wobble.setPower(0.5);
            wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(Math.abs(wobble.getCurrentPosition()-(2786*0.1))<100)
            {
                prindere.setPosition(1);


                sus++;

            }
        }

    }

    private void initPID(){
        brainX = new PID(0.7,0,0);
        brainY = new PID(1,0,0);
        brainU = new PID(1.5,0,0);
        brainX.setTolerance(10);
        brainY.setTolerance(10);
        brainU.setTolerance(2);
        brainX.reset();
        brainY.reset();
        brainU.reset();
    }

    private void movement_pid()
    {
        double[] power = new double[4];
        power[0] =   + brainX.output(globalX)*speed - brainY.output(globalY)*speed + brainU.output(globalU);//+
        power[1] =   - brainX.output(globalX)*speed - brainY.output(globalY)*speed + brainU.output(globalU);//-
        power[2] =   + brainX.output(globalX)*speed + brainY.output(globalY)*speed + brainU.output(globalU);//-
        power[3] =   - brainX.output(globalX)*speed + brainY.output(globalY)*speed + brainU.output(globalU);//+
        for(int i=0; i<4; i++)
        {
            motor[i].setPower(power[i]);
        }
    }
    private void STOP(int poz)
    {
        if(sus<poz)
        {
            movement_pid();


        }
        else {
            for(int i=0; i<4; i++)
            {
                motor[i].setPower(0);
            }
        }
    }
    private void go(double x, double y, double u, int poz)
    {

        if(sus==poz)
        {
            brainX.SetTarget(x);
            brainY.SetTarget(y);
            brainU.SetTarget(u);
            if(donexy(x,y))
            {
                brainX.reset();
                brainU.reset();
                brainY.reset();
                sus++;

            }
        }


        telemetry.addData("done?", Boolean.toString(donexy(x,y)));
    }

    private boolean done()
    {
        return brainX.done() && brainY.done() && brainU.done();
    }

    private boolean donexy(double x, double y)
    {
        if(Math.abs(globalX-x)<10 && Math.abs(globalY-y)<10)
            return true;

        return false;



    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
        //state[0]=1;
    }
    private void pauza(double interval,int poz)
    {

        if(sus==poz)
        {
            prezent = prezent == 0 ? runtime.milliseconds() : prezent;
            //wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            if(runtime.milliseconds()-prezent > interval)
            {
                prezent = 0;


                sus++;

            }
        }

    }

    private void cazC()
    {
        go(70,20,0,k++);
        go(150,20,0,k++);
        go(280,20,0,k++);
        pauza(1000,k++);
        lasajosdetot(k++);
        pauza(1000, k++);
        ridicabrat(k++);
        pauza(1000,k++);
        go(215,150,0,k++);
    }

    private void cazB()
    {
        go(70,20,0,k++);
        go(150,20,0,k++);
        go(220,65,0,k++);
        pauza(1000,k++);
        lasajosdetot(k++);
        pauza(1000, k++);
        ridicabrat(k++);
    }
    private void cazA()
    {
        go(70,20,0,k++);
        go(160,20,0,k++);
        //go(205,20,0,k++);
        pauza(1000,k++);
        lasajosdetot(k++);
        pauza(1000, k++);
        ridicabrat(k++);
        pauza(1000,k++);
        go(205,102,0,k++);
    }
    private void caz()
    {
        if(abc<2) {
            if (djos.getState())
                abc = 1;
            else if (!djos.getState() && dsus.getState())
                abc = 2;
            else if (!djos.getState() && !dsus.getState())
                abc = 3;
        }
    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        k=0;
        pozitie_curenta();
        //go(50, 150,0,k++);
        go(97,95,0,k++);
        /*go(150,35,0,5);
        go(220,90,0,6);*/
        //go(60, 60, 0,0);
        //go(210, 30, 0,1);
        caz();
        if(abc==1)
        {
            cazA();
        }
        else if(abc==2)
        {
            cazB();
        }
        else if(abc==3)
        {
            cazC();
        }

        go(40,100,-10,k++);
        arunca(4, k++); //4 ori wtf???
        go(200, 140,0, k++);
        //brainU.SetTarget(30);
        //movement_pid();
        STOP(k++);
        telemetry.addData("am aruncat ", Integer.toString(deatateaori));
        telemetry.addData("SUS/JOS", Boolean.toString(dsus.getState()) + "/" + Boolean.toString(djos.getState()));
        telemetry.addData("CAZUL ESTE", Integer.toString(abc));
        telemetry.addData("sussy", Integer.toString(sus));
        telemetry.addData("Status", "ce  a vrut Oprea x:" +  Double.toString(globalX) +"y:" + Double.toString(globalY)+" u:" + Double.toString(globalU));
    }

    @Override
    public void stop() {
    }

    private void arunca(int decateori,int poz)
    {

        if(sus==poz)
        {
            prezent1 = prezent1 == 0 ? runtime.milliseconds() : prezent1;
            prezent2 = prezent2 == 0 ? runtime.milliseconds() : prezent2;
            outtake.setPower(0.81);
            //inpinge.setPosition(0.5);
            //wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(runtime.milliseconds()-prezent2 > 2100){
                inpinge.setPosition(0.5);
                prezent2=0;
                deatateaori++;

            }
            if(runtime.milliseconds()-prezent1 > 1800){
                prezent1=0;
                inpinge.setPosition(1);

            }

            if(deatateaori>=decateori)
            {
                //prezent1 = 0;
                outtake.setPower(0);
                inpinge.setPosition(0.5);
                sus++;

            }
        }

    }

    private void pozitie_curenta()
    {
        //AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        //Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//-angles.firstAngle;
        globalU = -angles.firstAngle;

        //globalX = s_spate.getDistance(DistanceUnit.CM);
        double stanga = s_stanga.getDistance(DistanceUnit.CM);
        double dreapta = s_dreapta.getDistance(DistanceUnit.CM);
        double spate_o = s_spate.getDistance(DistanceUnit.CM);
        double spate_s = sonar.getVoltage()*522.745 + 3.493;
        telemetry.addData("Sonar v", Double.toString(spate_s));
        /*
        if(stanga > 150)
        {
            globalY = 2400 - stanga - 230;
        }else if(dreapta > 150)
        {
            globalY = dreapta + 230;
        }*/
        if(spate_s<30)
        {
            globalX=Math.cos(Math.toRadians(globalU))*spate_o;
        }
        else
        {
            globalX=Math.cos(Math.toRadians(globalU))*spate_s;
        }
        //globalX = Math.cos(Math.toRadians(globalU)) * (s_spate.getDistance(DistanceUnit.CM) < 90 ? s_spate.getDistance(DistanceUnit.CM) : sonar.getVoltage()*522.745+3.493);// / 0.004883;
        globalY = dreapta < 120 ? Math.cos(Math.toRadians(globalU))*dreapta : 240-30-Math.cos(Math.toRadians(globalU))*stanga;

        //globalU = 0;
    }
}
