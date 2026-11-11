package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name="Aootoo")
public class testauto extends LinearOpMode {

    private Kitchenet stuff = new Kitchenet(this);

    public int i;


    @Override public void runOpMode() {


        stuff.initializeAuto();
        stuff.telemetryupdate();


        waitForStart();

        stuff.StartUpAuto();
        stuff.telemetryupdate();


        while (opModeIsActive()) {

            stuff.DriveUpdateAuto();
            stuff.telemetryupdate();


            if (i == 1) {
                    stuff.driveTo(-10,10);
            stuff.Counter(5);
            }


            else if (i == 2) {
                    break;
            }

        }
    }
}