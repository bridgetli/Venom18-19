package org.firstinspires.ftc.teamcode;
/*
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

            if (isBlock2onLeft){
                Turn(90.0);
                goForward(12.0);
                Turn(-90.0);
                goForward(36.0);
                // You are on the crater wall
                //You could turn around a little more
                // than 45 degrees right or something and go like
                //12 sqrt 2 inches to be around the center of the crater,
                // could turn another 90 degrees right to be
                // facing the rover
            }

            else if (isBlock2onRight){
                Turn(90.0);
                goForward(36.0);
                Turn(-90.0);
                goForward(60.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees right
                // and go backwards around 24 sqrt 2 inches to be
                // around the center of the crater, could turn another
                // 90 degrees right to be facing the rover
            }

            else if (isBlock2onCenter){
                Turn(90.0);
                goForward(24.0);
                Turn(-90.0);
                goForward(48.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees
                // right and go backwards around 12 sqrt 2 inches
                // to be around the center of the crater, could
                // turn another 90 degrees right to be facing the rover
            }

        }

        if (isBlock1onRight){
            goForward(25.4558);
            Turn(-45.0);
            goForward(24.0);
            Turn(90.0);
            goForward(36.0);
            Turn(-90.0);
            depositMarker();
            Turn(180.0);
            goForward(48.0);

            if (isBlock2onLeft){
                Turn(90.0);
                goForward(12.0);
                Turn(-90.0);
                goForward(36.0);
                // You are on the crater wall
                //You could turn around a little more than 45
                // degrees right or something and go like 12
                // sqrt 2 inches to be around the center of the crater,
                // could turn another 90 degrees right to be facing the rover
            }

            else if (isBlock2onRight){
                Turn(90.0);
                goForward(36.0);
                Turn(-90.0);
                goForward(60.0);
                // You are on the crater wall
                // You could turn a little less than 45 degrees right
                // and go backwards around 24 sqrt 2 inches to be around
                // the center of the crater, could turn another 90 degrees
                // right to be facing the rover
            }

            else if (isBlock2onCenter){
                Turn(90.0);
                goForward(24.0);
                Turn(-90.0);
                goForward(48.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees right
                // and go backwards around 12 sqrt 2 inches to be
                // around the center of the crater, could turn another
                // 90 degrees right to be facing the rover
            }

        }

        if (isBlock1onCenter){
            goForward(59.397);
            Turn(45.0);
            goForward(12.0);
            Turn(-90.0);
            depositMarker();
            Turn(180);
            goForward(48.0);

            if (isBlock2onLeft){
                Turn(90);
                goForward(12.0);
                Turn(-90.0);
                goForward(36.0);
                // You are on the crater wall
                //You could turn around a little more than
                // 45 degrees right or something and go like
                //12 sqrt 2 inches to be around the center of the crater,
                // could turn another 90 degrees right to be facing the rover
            }

            else if (isBlock2onRight){
                Turn(90);
                goForward(36.0);
                Turn(-90.0);
                goForward(60.0);
                // You are on the crater wall
                //You could turn around a little more than
                // 45 degrees right or something and go like
                //12 sqrt 2 inches to be around the center of the
                // crater, could turn another 90 degrees right to be
                // facing the rover
            }

            else if (isBlock2onCenter){
                Turn(90.0);
                goForward(24.0);
                Turn(-90.0);
                goForward(48.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees
                // right and go backwards around 12 sqrt 2 inches
                // to be around the center of the crater,
                // could turn another 90 degrees right to be facing the rover
            }

        }


    }

}
*/