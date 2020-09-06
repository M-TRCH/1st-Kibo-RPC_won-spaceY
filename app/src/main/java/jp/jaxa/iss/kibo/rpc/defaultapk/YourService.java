package jp.jaxa.iss.kibo.rpc.thailand;

// android library
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

// zxing library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

// opencv library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;

// java library
import java.util.ArrayList;
import java.util.List;

// astrobee library
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


public class YourService extends KiboRpcService
{
    String mode = "sim"; // sim/ iss mode
    int navCam_width = 1280, navCam_height = 960;
    int cropSize_width = 2000, cropSize_height = 1500;
    double AR_diagonal = 0.07071067812;



    @Override
    protected void runPlan1()
    {
        api.judgeSendStart();

        double pos_x = 10.950, pos_z = 5.410;


        final float[] pos = QR_sequence(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 1.0f, 0.0f, 6, 40, 0); // p1 point

        // p1 to p2 path
        moveTo(10.7600f, -5.7100f, 4.9100f, 0.0f, 0.0f, 0.0f, 0.0f);
        moveTo(10.4600f, -6.0800f, 5.2400f, 0.0f, 0.0f, 0.0f, 0.0f);
        moveTo(10.4600f, -7.6800f, 5.2400f, 0.0f, 0.0f, 0.0f, 0.0f);
        //

        final float[] qua = QR_sequence(10.9774f, -7.6378f, 5.4400f, 0.0f, -0.7071f, 0.0f, 0.7071f, 6, 40, 1); // p2 point

        api.laserControl(true);
        judgeSendFinish();
    }
    @Override
    protected void runPlan2()
    {

    }
    @Override
    protected void runPlan3()
    {

    }
    public void moveTo(Point point, Quaternion quaternion)
    {
        int count = 0, count_max = 3;
        Result result;
        do
        {
            result = api.moveTo(point, quaternion, true);
            count++;
        }
        while (!result.hasSucceeded() && count < count_max);
    }
    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
    {
        moveTo(new Point(px, py, pz), new Quaternion(qx, qy, qz, qw));
    }
    public float[] QR_sequence(float px, float py, float pz, float qx, float qy, float qz, float qw, int count_max, int crop_percent, int no)
    {
        String contents = null;
        int count = 0;

        while (contents == null && count < count_max)
        {
            moveTo(px, py, pz, qx, qy, qz, qw);

            int offset_row = crop_percent / navCam_height / 200;
            int offset_col = crop_percent / navCam_width  / 200;
            int rows = navCam_height - (offset_row * 2);
            int cols = navCam_width  - (offset_col * 2);

            Mat src_resize = new Mat(cropSize_width, cropSize_height, CvType.CV_8UC1);
            Imgproc.resize(new Mat(undistord(api.getMatNavCam()), new Rect(offset_col, offset_row, cols, rows)), src_resize, new Size(cropSize_width, cropSize_height));

            contents = QR_decoder(src_resize);
            count++;
        }
        api.judgeSendDiscoveredQR(no, contents);


        String[] multi_contents = contents.split(", ");
        final double final_x = Double.parseDouble(multi_contents[1]);
        final double final_y = Double.parseDouble(multi_contents[3]);
        final double final_z = Double.parseDouble(multi_contents[5]);
        double final_w = 0;


        Log.d("QR[" + no + "][x]:", " " + final_x);
        Log.d("QR[" + no + "][y]:", " " + final_y);
        Log.d("QR[" + no + "][z]:", " " + final_z);

        if(no == 1)
        {
            final_w = Math.sqrt(1 - final_x*final_x - final_y*final_y - final_z*final_z);
            Log.d("QR[" + no + "][w]:", " " + final_w);
        }
        return new float[]{(float)final_x, (float)final_y, (float)final_z, (float)final_w};
    }
    public void judgeSendFinish()
    {
             if(mode == "sim") api.judgeSendFinishSimulation();
        else if(mode == "iss") api.judgeSendFinishISS();
    }
    public Mat undistord(Mat source)
    {
        Mat undis = new Mat(navCam_width, navCam_height, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs   = new Mat(1, 5, CvType.CV_32FC1);

        // constant value
        double[] cameraMatrix_sim =
        {
            344.173397,   0.000000, 630.793795,
              0.000000, 344.277922, 487.033834,
              0.000000,   0.000000,   1.000000
        };
        double[] cameraMatrix_iss =
        {
            692.827528,   0.000000, 571.399891,
              0.000000, 691.919547, 504.956891,
              0.000000,   0.000000,   1.000000
        };
        double[] distCoeffs_sim = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
        double[] distCoeffs_iss = {-0.312191, 0.073843, -0.000918,  0.001890, 0.000000};


        if(mode == "sim")
        {
            cameraMatrix.put(0, 0, cameraMatrix_sim);
            distCoeffs.put(0, 0, distCoeffs_sim);
        }
        else if(mode == "iss")
        {
            cameraMatrix.put(0, 0, cameraMatrix_iss);
            distCoeffs.put(0, 0, distCoeffs_iss);
        }
        Imgproc.undistort(source, undis, cameraMatrix, distCoeffs);
        return undis;
    }
    public double[] intersection(org.opencv.core.Point[] A, org.opencv.core.Point[] B)
    {
        double[] contents = new double[3];

        double a = (A[1].x-A[0].x) * (B[1].x-B[0].x);
        double b = (A[1].x-A[0].x) * (B[1].y-B[0].y);
        double c = (B[1].x-B[0].x) * (A[1].y-A[0].y);

        contents[0] = (a*A[0].y + b*B[0].x - a*B[0].y - c*A[0].x) / (b-c);
        contents[1] = ((A[1].y-A[0].y) * (contents[0]-A[0].x) / (A[1].x-A[0].x)) + A[0].y;

        double line_ax = Math.pow(A[0].x - A[1].x, 2);
        double line_ay = Math.pow(A[0].y - A[1].y, 2);
        double line_bx = Math.pow(B[1].x - B[0].x, 2);
        double line_by = Math.pow(B[1].y - B[1].y, 2);

        double avg = (Math.sqrt(line_ax+line_ay) + Math.sqrt(line_bx+line_by)) / 2.0;
        contents[2]  = avg / AR_diagonal;

        return contents;
    }
    public String QR_decoder(Mat src_mat)
    {
        Log.d("qr_code[status]:", " start");
        String contents = null;

        Bitmap src_bmp = Bitmap.createBitmap(src_mat.width(), src_mat.height(), Bitmap.Config.ARGB_8888);
        matToBitmap(src_mat, src_bmp, false);

        Log.d("qr_code[size]:", " " + src_bmp.getWidth() + ", " + src_bmp.getHeight());

        int[] intArray = new int[src_bmp.getWidth() * src_bmp.getHeight()];
        src_bmp.getPixels(intArray, 0, src_bmp.getWidth(), 0, 0, src_bmp.getWidth(), src_bmp.getHeight());

        LuminanceSource source = new RGBLuminanceSource(src_bmp.getWidth(), src_bmp.getHeight(), intArray);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

        try
        {
            com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
            contents = result.getText();
            Log.d("qr[value]:", " " + contents);
        }
        catch (Exception e)
        {
            Log.d("qr[value]:", " no detected");
        }

        Log.d("qr_code[status]:", " stop");
        return contents;
    }
//    org.opencv.core.Point[] lineA =
//            {
//                    new org.opencv.core.Point((int) corners.get(0).get(0, 0)[0], (int) corners.get(0).get(0, 0)[1]),
//                    new org.opencv.core.Point((int) corners.get(0).get(0, 2)[0], (int) corners.get(0).get(0, 2)[1])
//            };
//    org.opencv.core.Point[] lineB =
//            {
//                    new org.opencv.core.Point((int) corners.get(0).get(0, 1)[0], (int) corners.get(0).get(0, 1)[1]),
//                    new org.opencv.core.Point((int) corners.get(0).get(0, 3)[0], (int) corners.get(0).get(0, 3)[1])
//            };
}