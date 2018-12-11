package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.os.Environment;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * Created by bodeng on 10/19/18.
 */

@Autonomous (name = "SingleSampleDepot", group = "Autonomous")
public class SingleSampleDepot extends CustomLinearOpMode {

    private ElapsedTime time = new ElapsedTime();
    private char blockPos = 'C';

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        initizialize();
        telemetry.addLine("Vuforia initialization complete");

        waitForStart();

        /*ByteBuffer byteBuffer = image.getPixels();
                if (frameBuffer == null) {
                    frameBuffer = new byte[byteBuffer.capacity()];
                }
                byteBuffer.get(frameBuffer);
                if (this.frame == null) {
                    this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);

                    if (overlayView != null) {
                        overlayView.setImageSize(imageWidth, imageHeight);
                    }
                }
                this.frame.put(0, 0, frameBuffer);

                Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                if (parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                    Core.flip(this.frame, this.frame, 1);
                }

                onFrame(this.frame, vuforiaFrame.getTimeStamp());
            }


            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            */

        getBlock();


        telemetry.addData("Block Pos", blockPos);
        telemetry.update();

        sleep(100);

        moveToEncoder(560, .2, 0);
        stopAllMotors();
        sleep(250); //1000
        //TODO: optimize all of these paths; averages 17 sec currently; maybe get to 15 or less
        if (blockPos == 'R' || blockPos == '?') { //also, default path should be center, as it is slightly faster
            //turn towards block & move through it
            Pturn(45, 2500);
            sleep(250); //500
            moveToEncoder(1500, .35, 45); //increase encoder/power (if we change path)?
            sleep(250); //500


            Pturn(135, 2500); //replace this; instead, move straight through the block?
            sleep(250); //500
            moveToEncoderT(-1500, .35, 135, 2000);
            Pturn(-135, 2500); //turn to depot (adjust angle or remove statement)

            moveToEncoderT(700, .35, -135, 2000); //adjust angle as needed
            servoWinchArm.setPosition(servoWinchArmDepositPos);
            sleep(750);
            servoWinchArm.setPosition((servoWinchArmDepositPos+servoWinchArmInitPos) / 1.5);
            sleep(200); //750
            servoWinchArm.setPosition(servoWinchArmInitPos);
            moveTimeP(2000, .8, -134);

        } else if (blockPos == 'C') { //why do we not just drive straight through??
            moveToEncoder(1500, .25, 0); //all the way to depot
            sleep(250); //500


            Pturn(-45, 2000); //turn towards crater
            moveToEncoderT(400, .35, -45, 2000); //move a little forward while turning to align with wall
            Pturn(-135, 2500); //drop marker


            moveToEncoderT(300, .35, -135, 2000);
            servoWinchArm.setPosition(servoWinchArmDepositPos);
            sleep(750); //shrink this if we can for all autos
            servoWinchArm.setPosition((servoWinchArmDepositPos+servoWinchArmInitPos) / 1.5);
            sleep(200); //750
            servoWinchArm.setPosition(servoWinchArmInitPos);
            moveTimeP(2000, .8, -134); //drive to crater
        } else {
            Pturn(-45, 2500);
            sleep(250); //500
            moveToEncoder(1300, .35, -45);
            sleep(250); //500


            Pturn(-135, 2500); //remove, just drive straight to wall?
            moveTimeP(400, -.35, -135); //turn towards crater, and rest is the same
            //moveToEncoderT(1000, .35, -135, 2000);


            servoWinchArm.setPosition(servoWinchArmDepositPos);
            sleep(750);
            servoWinchArm.setPosition((servoWinchArmDepositPos+servoWinchArmInitPos) / 1.5);
            sleep(200); //750
            servoWinchArm.setPosition(servoWinchArmInitPos);
            moveTimeP(2000, .8, -134);
        }
    }

    private void Pturn(double angle, int msTimeout) {
        double kP = .35/90;
        double minSpeed = .2;
        double maxSpeed = .6;
        time.reset();

        while (Math.abs(imu.getTrueDiff(angle)) > .5 && time.milliseconds() < msTimeout && opModeIsActive()) {
            double angleError = imu.getTrueDiff(angle);

            double PIDchange = kP * angleError;

            if (PIDchange > 0 && PIDchange < minSpeed)
                PIDchange = minSpeed;
            else if (PIDchange < 0 && PIDchange > -minSpeed)
                PIDchange = -minSpeed;

            motorBL.setPower(Range.clip(-PIDchange, -maxSpeed, maxSpeed));
            motorFL.setPower(Range.clip(-PIDchange, -maxSpeed, maxSpeed));
            motorBR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));
            motorFR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));

            telemetry.addData("angleError: ", angleError);
            telemetry.addData("PIDCHANGE: ", PIDchange);
            telemetry.update();
        }
        stopMotors();
    }

    public void moveTimeP(double msTime, double power, double angle) throws InterruptedException {
        time.reset();

        double kPangle = 1.0/90.0;

        while (time.milliseconds() < msTime) {
            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            //if (power > 0) {
            motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            //} else if (power < 0) {
            //  motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            // motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            //motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            //motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            //}
        }
        stopMotors();
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }

    public void moveToDistP(double inches, double angle, double timeout) {
        double kPdist = .03;
        double kPangle = .9/90.0;

        double minDrive = .15;
        double maxDrive = .5;

        time.reset();
        while ((Math.abs(getDistB() - inches) > .25 || imu.getTrueDiff(angle) > .5) && opModeIsActive() && time.milliseconds() < timeout) {

            double distError = inches - getDistB();
            double PIDchangeDist = -Range.clip(-kPdist * distError, -maxDrive, maxDrive);

            if (PIDchangeDist < minDrive && PIDchangeDist > 0) {
                PIDchangeDist = minDrive;
            } else if (PIDchangeDist > -minDrive && PIDchangeDist < 0) {
                PIDchangeDist = -minDrive;
            }

            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            motorBL.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorFL.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorBR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
        }
        stopMotors();
    }

    public void moveToEncoder(double encoder, double power, double angle) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double kPangle = 1.0/90.0;

        if (encoder > 0) {
            while (motorFL.getCurrentPosition() < encoder && opModeIsActive()) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            }
        }
        else {
            while (motorFL.getCurrentPosition() > encoder && opModeIsActive()) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
            }
        }
        stopMotors();
    }

    public void moveToEncoderT(double encoder, double power, double angle, double msTimeout) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double kPangle = 1.0/90.0;
        time.reset();
        if (encoder > 0) {
            while (motorFL.getCurrentPosition() < encoder && opModeIsActive() && time.milliseconds() < msTimeout) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            }
        }
        else {
            while (motorFL.getCurrentPosition() > encoder && opModeIsActive() && time.milliseconds() < msTimeout) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
            }
        }
        stopMotors();
    }

    public void getBlock() throws InterruptedException {
        blockPos = 'C';

        Bitmap bitmap = takePic();


        // basic brute force counter
        int startRow = 14;
        int endRow = 23;
        int leftSrow = 13;
        int leftErow = 22;
        int centerSrow = 35;
        int centerErow = 44;
        int rightSrow = 56;
        int rightErow = 65;

        BoundingBox left = new BoundingBox(startRow, leftSrow, endRow, leftErow);    //look at images taken from consistent
        BoundingBox center = new BoundingBox(startRow, centerSrow, endRow, centerErow);  //spot in auto and get pixel range
        BoundingBox right = new BoundingBox(startRow, rightSrow, endRow, rightErow);   //of left center and right

        if (yellowValOfBox(bitmap, left) > yellowValOfBox(bitmap, center)) {
            blockPos = 'L';
            if (yellowValOfBox(bitmap, right) > yellowValOfBox(bitmap, left))
                blockPos = 'R';
        }
        else if (yellowValOfBox(bitmap, right) > yellowValOfBox(bitmap, center))
            blockPos = 'R';

        //saveBox(bitmap, left);
        //saveBox(bitmap.copy(Bitmap.Config.RGB_565, true), center);
        //saveBox(bitmap.copy(Bitmap.Config.RGB_565, true), right);

        // multi location pixel scanner (better but much slower)

        /*int N = 4; // the approx height and width of an object

        for (int r = 0; r < */
    }

    public void getBlockAcc() throws InterruptedException {
        Bitmap bitmap = takePic();

        int N = 4;
        BoundingBox block = null;
        BoundingBox ball1 = null;
        BoundingBox ball2 = null;
        int yellow = -1;
        int white = -1;
        int white2 = -1;
        for (int r = 0; r < bitmap.getHeight() - N; r++) {
            for (int c = 0; c < bitmap.getWidth() - N; c++) {
                BoundingBox testBox = new BoundingBox(r, c, r+4, c+4);
                int testYellow = yellowValOfBox(bitmap, testBox);
                int testWhite = whiteValOfBox(bitmap, testBox);
                if (testYellow > yellow) {
                    yellow = testYellow;
                    block = testBox;
                }
                if (testWhite > white) {
                    white = testWhite;
                    ball1 = testBox;
                }
                else if (testWhite > white2 && Math.abs(testBox.startCol - ball1.startCol) > 10) {
                    white2 = testWhite;
                    ball2 = testBox;
                }
            }
        }

        if (block.startCol < ball1.startCol && block.startCol < ball2.startCol)
            blockPos = 'L';
        else if (block.startCol > ball1.startCol && block.startCol > ball2.startCol)
            blockPos = 'R';
        else
            blockPos = 'C';


    }

    public int whiteValOfBox(Bitmap bmp, BoundingBox bb) {
        int whiteSum = 0;

        for (int r = bb.startRow; r < bb.endRow && opModeIsActive(); r++) {
            for (int c = bb.startCol;  c < bb.endCol && opModeIsActive(); c++) {
                int color = bmp.getPixel(c, r);
                int R = (color >> 16) & 0xff;
                int G = (color >>  8) & 0xff;
                int B = color & 0xff;
                int white = Math.min(R, Math.min(G, B));
                whiteSum += white;
            }
        }

        return whiteSum;
    }

    public int yellowValOfBox(Bitmap bmp, BoundingBox bb) {
        int ySum = 0;

        //scans bounding box
        for (int r = bb.startRow; r < bb.endRow && opModeIsActive(); r++) {
            for (int c = bb.startCol;  c < bb.endCol && opModeIsActive(); c++) {
                int color = bmp.getPixel(c, r);
                int R = (color >> 16) & 0xff;
                int G = (color >>  8) & 0xff;
                int B = color & 0xff;
                int yellow = Math.min(R, G) - B > 0 ? Math.min(R, G) - B : 0;
                ySum += yellow;
            }
        }

        //scans entire bitmap
        /*or (int r = 0; r < bmp.getHeight(); r++) {
            for (int c = 0;  c < bmp.getWidth(); c++) {
                int color = bmp.getPixel(c, r);
                int R = (color >> 16) & 0xff;
                int G = (color >>  8) & 0xff;
                int yellow = Math.min(R, G);
                ySum += yellow;
            }
        }*/
        return ySum;
    }

    private class BoundingBox {
        int startRow;
        int endRow;
        int startCol;
        int endCol;

        public BoundingBox(int startRow, int startCol, int endRow, int endCol) {
            this.startRow = startRow;
            this.endRow = endRow;
            this.startCol = startCol;
            this.endCol = endCol;
        }
    }

    public Bitmap getResizedBitmap(Bitmap bm, int newWidth, int newHeight) {
        int width = bm.getWidth();
        int height = bm.getHeight();
        float scaleWidth = ((float) newWidth) / width;
        float scaleHeight = ((float) newHeight) / height;
        // CREATE A MATRIX FOR THE MANIPULATION
        Matrix matrix = new Matrix();
        // RESIZE THE BIT MAP
        matrix.postScale(scaleWidth, scaleHeight);

        // "RECREATE" THE NEW BITMAP
        Bitmap resizedBitmap = Bitmap.createBitmap(
                bm, 0, 0, width, height, matrix, false);
        bm.recycle();
        return resizedBitmap;
    }

    public Bitmap takePic() throws InterruptedException {
        File dir = Environment.getExternalStorageDirectory();
        File[] files = dir.listFiles();
        int highestFileNum = -1;
        for (File f : files) {
            if (f.getName().contains("BoTest_pic_")) {
                highestFileNum = Math.max(highestFileNum, Integer.parseInt(f.getName().replaceAll("\\D", "")));
            }
        }
        int currFileNum = highestFileNum + 1;

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        telemetry.addData("Num images in frame", "" + frame.getNumImages());
        telemetry.update();
        //for (int i = 0; i < frame.getNumImages() && opModeIsActive(); i++) {
        Image image = frame.getImage(0);

        int imageWidth = image.getWidth(), imageHeight = image.getHeight();
        Bitmap bmp = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
        bmp.copyPixelsFromBuffer(image.getPixels());
        try {
            File sdCard = Environment.getExternalStorageDirectory();
            telemetry.addLine(sdCard.getAbsolutePath());
            telemetry.update();
            //File dir = new File(sdCard.getAbsolutePath() + "/dir1");
            //dir.mkdirs();

            File file = new File(sdCard, "BoTest_pic_" + currFileNum++ + ".png");

            FileOutputStream fos = new FileOutputStream(file);

            bmp = getResizedBitmap(bmp, 78, 59);
            bmp.compress(Bitmap.CompressFormat.PNG, 100, fos);

        } catch (FileNotFoundException ex) {
            telemetry.addLine(ex.toString());
            telemetry.update();
        }
        //ByteBuffer byteBuffer = image.getPixels();
                    /*if (frameBuffer == null) {
                        frameBuffer = new byte[byteBuffer.capacity()];
                    }
                    byteBuffer.get(frameBuffer);
                    if (this.frame == null) {
                        this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);

                        if (overlayView != null) {
                            overlayView.setImageSize(imageWidth, imageHeight);
                        }
                    }
                    this.frame.put(0, 0, frameBuffer);

                    Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                    if (parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                        Core.flip(this.frame, this.frame, 1);
                    }

                    onFrame(this.frame, vuforiaFrame.getTimeStamp());*/
        return bmp;
    }

    public Bitmap saveBox(Bitmap bmp, BoundingBox bb) throws InterruptedException{
        File dir = Environment.getExternalStorageDirectory();
        File[] files = dir.listFiles();
        int highestFileNum = -1;
        for (File f : files) {
            if (f.getName().contains("pixel_match_")) {
                highestFileNum = Math.max(highestFileNum, Integer.parseInt(f.getName().replaceAll("\\D", "")));
            }
        }
        int currFileNum = highestFileNum + 1;

        try {
            File sdCard = Environment.getExternalStorageDirectory();
            telemetry.addLine(sdCard.getAbsolutePath());
            telemetry.update();
            //File dir = new File(sdCard.getAbsolutePath() + "/dir1");
            //dir.mkdirs();

            File file = new File(sdCard, "pixel_match_" + currFileNum++ + ".png");

            FileOutputStream fos = new FileOutputStream(file);

            bmp = getResizedBitmap(bmp, 78, 59);
            Bitmap drawnBmp = bmp.copy(Bitmap.Config.RGB_565, true);

            Canvas canvas = new Canvas(drawnBmp);
            Paint p = new Paint();
            p.setStyle(Paint.Style.FILL_AND_STROKE);
            p.setAntiAlias(true);
            p.setFilterBitmap(true);
            p.setDither(true);
            p.setColor(Color.RED);

            canvas.drawLine(bb.startCol, bb.startRow, bb.startCol, bb.endRow, p);//left
            canvas.drawLine(bb.startCol, bb.startRow, bb.endCol, bb.startRow, p);//top
            canvas.drawLine(bb.endCol, bb.startRow, bb.endCol, bb.endRow, p);//right
            canvas.drawLine(bb.startCol, bb.endRow, bb.endCol, bb.endRow, p);//bottom

// rect ...
//canvas.drawRect(/*all of my end coordinates*/, p);

            //ImageView iView = (ImageView)findViewById(R.id.imageViewPreview);
            //iView.setImageBitmap(drawnBmp);
            //iView.draw(canvas);

            drawnBmp.compress(Bitmap.CompressFormat.PNG, 100, fos);

        } catch (FileNotFoundException ex) {
            telemetry.addLine(ex.toString());
            telemetry.update();
        }

        return bmp;
    }
}