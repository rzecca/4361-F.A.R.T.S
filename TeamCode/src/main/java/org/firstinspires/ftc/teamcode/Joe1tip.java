package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@Autonomous(name="Joe 1 Tip + Park")
public class Joe1tip extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    public int i=1;


    @Override public void runOpMode() {


        stuff.initializeAuto();
        stuff.telemetryupdate();
        stuff.PersonalUpdateAuto();

        waitForStart();
        DcMotorEx turret;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stuff.StartUpAuto();
        stuff.telemetryupdate();
        stuff.PersonalUpdateAuto();

        while (opModeIsActive()) {

            stuff.DriveUpdateAuto();
            stuff.telemetryupdate();
            stuff.PersonalUpdateAuto();
            telemetry.addData("ier", i);
            i = stuff.ithing();

            if (i == 1) {
                //speed up launcher
                stuff.iaamrunning();
                turret.setPower(0);
                stuff.ramp.setPosition(.84);
                stuff.stopper.setPosition(.95);

                stuff.Counter(.5);
            }
            if (i == 2) {
                //speed up launcher
                stuff.iaamrunning();
                stuff.ramp.setPosition(.95);

                stuff.driveTo(0,0);
                    stuff.setLauncherSpeed(4500);
            stuff.Counter(3);
            }

            else if (i == 3) {
                //SHOOT
                //NEED TO ADJUST STOPPER POSITIONS
                stuff.setLauncherSpeed(1000000000);
                stuff.intake.setPower(-1);

                stuff.stopper.setPosition(.85);
                stuff.intake.setPower(-1);
            stuff.Counter(3);
            }

            else if (i == 4) {
                stuff.setLauncherSpeed(0);

                //ADJUST RAMP POS
                stuff.stopper.setPosition(.9);
                stuff.intake.setPower(0);

                stuff.driveTo(20,20, -90);
                stuff.Counter(1);
            }

            else if (i == 5) {
                stuff.driveTo(90,40, 90, 1);

                stuff.Counter(10);

            }



            else if (i == 6) {
                    break;
            }

        }
    }
}