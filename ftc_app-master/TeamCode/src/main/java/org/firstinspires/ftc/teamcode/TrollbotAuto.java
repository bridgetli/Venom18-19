package org.firstinspires.ftc.teamcode;

public class TrollbotAuto extends CustomLinearOpMode {

    public boolean isBlock1onLeft{
        // **needs a new name
        //if first block is on the left
            //return true
        //else
            //return false
    }

    public boolean isBlock2onLeft{
        // **needs a new name
        //if second block is on the left
            //return true
        //else
            //return false
    }

    public boolean isBlock2onRight{
        // **needs a new name
        //if second block is on the right
            //return true
        //else
            //return false
    }

    public boolean isBlock2onCenter{
        // **needs a new name
        //if second block is on the center
            //return true
        //else
            //return false
    }

    public boolean isBlock1onRight{
        // **needs a new name
        //if first block is on the right
            //return true
        //else
            //return false
    }

    public boolean isBlock1onCenter{
        // **needs a new name
        //if first block is on the center
            //return true
        //else
            //return false
    }

    public void goForward(double distance){
        // goes foward a certain distance after we add the sensor in
        // distance is in inches
    }

    public void depositMarker() {
        // deposits the marker in the crater
    }


    @Override
    public void runOpMode() {
        if (isBlock1onLeft == true) {
            goForward(25.4558);
            Turn(45.0);
            goForward(36);
            Turn(90.0);
            goForward(24);
            depositMarker();
            Turn(180.0);
            goForward(48.0);
        }
        else if (isBlock2onLeft == true){
            Turn(90.0);
            goForward(12.0);
            Turn(-90.0);
            goForward(36.0);
            depositMarker();
            Turn(180.0);
            goForward(48.0);
        }
        else if (isBlock2onRight == true){
            Turn(90.0);
            goForward(36.0);
            Turn(-90.0);
            goForward(60.0);
            depositMarker();
            Turn(180.0);
            goForward(48.0);
        }
        else if (isBlock2onCenter == true){
            Turn(90.0);
            goForward(24.0);
            Turn(-90.0);
            goForward(48.0);
            depositMarker();
            Turn(180.0);
            goForward(48.0);
        }
        else if (isBlock1onRight == true){
            goForward(25.4558);
            Turn(-45.0);
            goForward(24.0);
            Turn(90.0);
            goForward(36.0);
            Turn(-90.0);
            depositMarker();
            Turn(180.0);
            goForward(48.0);
        }

        else if (isBlock1onCenter){

        }


    }

}
