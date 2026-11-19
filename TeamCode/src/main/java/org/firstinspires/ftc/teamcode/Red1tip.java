package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="1 Tip")
public class Red1tip extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    public int i=1;


    @Override public void runOpMode() {


        stuff.initializeAuto();
        stuff.telemetryupdate();
        stuff.PersonalUpdateAuto();

        waitForStart();

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
                stuff.ramp.setPosition(.84);
                stuff.stopper.setPosition(.95);

                stuff.Counter(.5);
            }
            if (i == 2) {
                //speed up launcher
                stuff.iaamrunning();
                stuff.ramp.setPosition(.95);

                stuff.driveTo(0,0);
                    stuff.setLauncherSpeed(36000);
                    stuff.intake.setPower(-1);
            stuff.Counter(2);
            }

            else if (i == 3) {
                //SHOOT
                //NEED TO ADJUST STOPPER POSITIONS
                stuff.setLauncherSpeed(1000000000);
                stuff.stopper.setPosition(.85);
                stuff.intake.setPower(-1);
            stuff.Counter(3);
            }

            else if (i == 4) {
                //DRIVE TO FLOWER TO INTAKE
                stuff.setLauncherSpeed(3600);

                //ADJUST RAMP POS
                stuff.stopper.setPosition(.9);
                stuff.intake.setPower(-.5);

                stuff.driveTo(3,35, -90);
                stuff.Counter(10);
            }

            else if (i == 5) {
                stuff.driveTo(3,20, -90);

                stuff.Counter(2);

            }

            else if (i == 6) {
                stuff.driveTo(70,20, -90);

                stuff.Counter(2);

            }



            else if (i == 7) {
                    break;
            }

        }
    }
}