package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="Aootoo")
public class BLUE3 extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    public int i;


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


            if (i == 1) {
                //speed up launcher
                    stuff.driveTo(0,0);
                    stuff.setLauncherSpeed(12000);
                    stuff.intake.setPower(.75);
            stuff.Counter(.5);
            }

            else if (i == 2) {
                //SHOOT
                //NEED TO ADJUST STOPPER POSITIONS
                stuff.stopper.setPosition(.8);
            stuff.Counter(1);
            }

            else if (i == 3) {
                //DRIVE TO FLOWER TO INTAKE
                //ADJUST RAMP POS
                stuff.ramp.setPosition(.5);
                    stuff.driveTo(10,0);
            stuff.Counter(2);
            }

            else if (i == 4) {
                //DRIVE TO 2SHOOT POS
                    stuff.driveTo(0,0);
            stuff.Counter(3);
            }


            else if (i == 5) {
                    break;
            }

        }
    }
}