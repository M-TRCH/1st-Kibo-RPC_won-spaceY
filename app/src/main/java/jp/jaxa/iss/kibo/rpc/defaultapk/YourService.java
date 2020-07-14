package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;


import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.qrcode.detector.Detector;


import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;


import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static org.opencv.android.Utils.matToBitmap;

public class YourService extends KiboRpcService
{
    @Override
    protected void runPlan1()
    {
        for(int i=0; i<3; i++)
        {
            api.judgeSendStart();
            Log.d("[API("+i+")]:" , "Judge send start");
            SystemClock.sleep(5000);
        }



//        moveTo(10.9263f, -5.2426f, 4.4622f, 0.0f, 0.0f, 0.0f, 0.0f);
//        moveTo(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 1.0f, 0.0f);

        moveTo(10.4600f, -4.5000f, 5.2400f, 0.000f,  0.000f, 0.000f, 0.000f);
        moveTo(10.4600f, -7.6800f, 5.2400f, 0.000f,  0.000f, 0.000f, 0.000f);



        String contents = null;
        int count = 0;
        while(contents == null)
        {
            moveTo(10.9774f, -7.6378f, 5.4000f, 0.000f, -0.707f, 0.000f, 0.707f);

            Kinematics current = api.getTrustedRobotKinematics();
            Point pos = current.getPosition();
            Quaternion qua = current.getOrientation();
            double px = pos.getX();
            double py = pos.getY();
            double pz = pos.getZ();
            double qx = qua.getX();
            double qy = qua.getY();
            double qz = qua.getZ();
            double qw = find_w(qx, qy, qz);

            Log.d("Current_Position["+count+"]:", " "+px+", "+py+", "+pz+", "+qx+", "+qy+", "+qz+", "+qw);





            Mat src_mat = api.getMatNavCam();
            Size size = new Size(4000,3000);
            Imgproc.resize(src_mat, src_mat, size);


            src_mat = undistord(src_mat);

/*
            double ratio = 1280/960;

            double percent_row = 20;
            double percent_col = percent_row*ratio;

            int max_row = 960;
            int max_col = 1280;

            int offset_row = ((int)percent_row*max_row)/100;
            int offset_col = ((int)percent_col*max_col)/100;

            double rows = max_row-(offset_row*2);
            double cols = max_col-(offset_col*2);

            Rect rect = new Rect(offset_col, offset_row , (int)cols, (int)rows);
            Mat crop = new Mat(src_mat, rect);
*/

            Bitmap bMap = Bitmap.createBitmap(4000, 3000, Bitmap.Config.ARGB_8888);
            matToBitmap(src_mat, bMap, false);






            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try
            {
                // com.google.zxing.qrcode.detector.Detector result = new Detector(bitmap);

                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                contents = result.getText();
                Log.d("QR[value]:", " "+contents);
            }
            catch (Exception e)
            {
                Log.d("QR[status]:", " Not detected");
            }





            Log.d("QR[count]:", " "+count);
            count++;
        }
        api.judgeSendDiscoveredQR(1, contents);

        String[] multi_contents = contents.split(", ");
        Log.d("QR[x]:"," "+multi_contents[1]);
        Log.d("QR[y]:"," "+multi_contents[3]);
        Log.d("QR[z]:"," "+multi_contents[5]);






        





        api.laserControl(false);
        api.judgeSendFinishSimulation();
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
        int count = 0, count_max = 8;
        while (true)
        {
            Result result = api.moveTo(point, quaternion, true);
            if (result.hasSucceeded()) { count++; }
            if(count >= count_max){ break; }
        }
    }
    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
    {
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        moveTo(point, quaternion);
    }
    public double moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, int no)
    {
        String contents = null;
        int count = 0, count_max = 40;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        while (contents == null && count < count_max)
        {
            Log.d("QR[Count]:"," ["+count+"]");
            moveTo(point, quaternion);

            // NEW //
            Mat mat_src = api.getMatNavCam();

            mat_src = undistord(mat_src);

//            Rect crop = new Rect(, , 320ผ, 320);
//            Mat src_crop = new Mat(mat_src, crop);


            Bitmap source;
            source = Bitmap.createBitmap(1280, 960, Bitmap.Config.ARGB_8888);
            matToBitmap(mat_src, source, false);
            //     //

            // Bitmap source = api.getBitmapNavCam();

            int[] pixel = new int[source.getWidth() * source.getHeight()];
            source.getPixels(pixel, 0, source.getWidth(), 0, 0, source.getWidth(), source.getHeight());
            Image barcode = new Image(source.getWidth(), source.getHeight(), "RGB4");

            barcode.setData(pixel);
            ImageScanner reader = new ImageScanner();
            reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
            reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
            reader.scanImage(barcode.convert("Y800"));


            SymbolSet syms = reader.getResults();
            for (Symbol sym : syms)
            {
                contents = sym.getData();
                // Log.d("QR[" + no + "]: ", contents);
                Log.d("QR[NEW]: ", contents);
            }
            count++;
        }
        String[] val_array = contents.split(", ");
        double val_return = 0; //Double.parseDouble(val_array[1]);
        api.judgeSendDiscoveredQR(0, "pos_x, "+val_array[1]);
        api.judgeSendDiscoveredQR(1, "pos_y, "+val_array[3]);
        api.judgeSendDiscoveredQR(2, "pos_z, "+val_array[5]);
        return val_return;
    }
    public double[] moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, String ar)
    {
        int AR_int = 0, count = 0;
        int x[] = new int[4], y[] = new int[4];
        double avg[] = new double[5], center[] = new double[6], result[] = new double[6];
        float px_out = px, pz_out = pz;
        Point point = new Point();

        while (AR_int == 0 && count < 15)
        {
            Log.d("AR_counter: ", "" + count);

            moveTo(px_out, py, pz_out, qx, qy, qz, qw);

            //Mat source = undistord(api.getMatNavCam());
            Mat source = api.getMatNavCam();
            Mat ids = new Mat();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            DetectorParameters setting = null;

            try
            {
                Aruco.detectMarkers(source, dictionary, corners, ids);
                AR_int = (int) ids.get(0, 0)[0];
                Kinematics current = api.getTrustedRobotKinematics();
                point = current.getPosition();
            }
            catch (Exception e)
            {
                AR_int = 0;
            }
            if (AR_int != 0)
            {
                Log.d("AR[" + count + "]: ", "" + AR_int);
                x[0] = (int) corners.get(0).get(0, 0)[0]; // x BL
                y[0] = (int) corners.get(0).get(0, 0)[1]; // y BL
                x[1] = (int) corners.get(0).get(0, 1)[0]; // x TL
                y[1] = (int) corners.get(0).get(0, 1)[1]; // y TL
                x[2] = (int) corners.get(0).get(0, 2)[0]; // x TR
                y[2] = (int) corners.get(0).get(0, 2)[1]; // y TR
                x[3] = (int) corners.get(0).get(0, 3)[0]; // x BR
                y[3] = (int) corners.get(0).get(0, 3)[1]; // y BR

                avg[0] = (double) Math.abs(y[2] - y[0]);  // r
                avg[1] = (double) Math.abs(x[0] - x[1]);  // t
                avg[2] = (double) Math.abs(y[1] - y[3]);  // l
                avg[3] = (double) Math.abs(x[3] - x[2]);  // b
                avg[4] = (avg[0] + avg[1] + avg[2] + avg[3]) / 4;
                center[0] = avg[0] / 2;
                center[1] = avg[1] / 2;
                center[2] = avg[2] / 2;
                center[3] = avg[3] / 2;
                center[4] = (center[1] + center[3]) / 2; // half x
                center[5] = (center[0] + center[2]) / 2; // half y

                result[0] = x[0] - center[4]; // x point > range 0-1279
                result[1] = y[0] + center[5]; // y point > range 0-959
                result[2] = avg[4]/0.05;    // ratio > pixel:meter
                Log.d("AR_center[OLD]: ",result[0]+", "+result[1]);
                Log.d("AR_center[OLD]:"," "+result[2]);

                double p[][] =
                        {
                                {x[0], y[0]},
                                {x[2], y[2]},
                                {x[1], y[1]},
                                {x[3], y[3]}
                        };
                double[] c = interceptLine(p);
//                result[0] = c[0];
//                result[1] = c[1];
//                result[2] = c[2];
            }
            count++;
        }
        String AR_value = Integer.toString(AR_int);
        api.judgeSendDiscoveredAR(AR_value);
        result[3] = point.getX();
        result[4] = point.getY();
        result[5] = point.getZ();

        return result;
    }
    public double find_w(double qx, double qy, double qz)
    {
        double qw = Math.sqrt(1 - qx * qx - qy * qy - qz * qz);
        return qw;
    }
    public double limit(char axis, double val)
    {
        double result = val;

        if (axis == 'x')
        {
            if (result > 11.49){ result = 11.49; }
            if (result < 10.41){ result = 10.41; }
        }
        if (axis == 'y')
        {
            if (result > -3.16) { result = -3.16; }
            if (result < -9.59) { result = -9.59; }
        }
        if (axis == 'z')
        {
            if (result > 5.44) { result = 5.44; }
            if (result < 4.36) { result = 4.36; }
        }
        return result;
    }
    public void targetShoot(double px, double py, double d, double pos_a, double pos_b, double pos_c)
    {
        double targetShift = 0.2*Math.sin(Math.toRadians(45));
        double navShift_z = 0.0826;
        double navShift_x = 0.0422;
        double laserShift_x = 0.0572;
        double laserShift_z = 0.1111;

        double pos_x = ((px - 640) / d) + pos_a + targetShift + navShift_x - laserShift_x;
        double pos_z = ((py - 480) / d) + pos_c + targetShift + navShift_z + laserShift_z;
        double pos_y = -10.35;

        double magnitude = Math.sqrt(((pos_x - pos_a) * (pos_x - pos_a))
                + ((pos_y - pos_b) * (pos_y - pos_b))
                + ((pos_z - pos_c) * (pos_z - pos_c)));

        double x_unit = (pos_x - pos_a) / magnitude;
        double y_unit = (pos_y - pos_b) / magnitude;
        double z_unit = (pos_z - pos_c) / magnitude;

        double matrix[][] =
                {
                        {1, 0, 0},
                        {x_unit, y_unit, z_unit}
                };

        double x = matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2];
        double y = matrix[0][2] * matrix[1][0] - matrix[1][2] * matrix[0][0];
        double z = matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];

        double i = matrix[1][0] - matrix[0][0];
        double j = matrix[1][1] - matrix[0][1];
        double k = matrix[1][2] - matrix[0][2];

        double q = Math.sqrt(x * x + y * y + z * z);
        double p = Math.sqrt(i * i + j * j + k * k);
        double r = Math.sqrt(matrix[0][0]*matrix[0][0]+matrix[0][1]*matrix[0][1]+matrix[0][2]*matrix[0][2]);

        double theta = Math.acos((p*p-2)/2*q*r*(-1));

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        moveTo((float) pos_a, (float) pos_b, (float) pos_c, (float) a, (float) b, (float) c, (float) w);
    }
    public Mat undistord(Mat src)
    {
        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        int row = 0, col = 0;
        double cameraMatrix_data[] =
                {
                        344.173397,   0.000000, 630.793795,
                        0.000000, 344.277922, 487.033834,
                        0.000000,   0.000000,   1.000000
                };
        double distCoeffs_data[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};

        cameraMatrix.put(row, col, cameraMatrix_data);
        distCoeffs.put(row, col, distCoeffs_data);

        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
        return dst;
    }
    public  double[] interceptLine(double p[][])
    {
        double center[] = new double[3];

        double a = (p[1][0]-p[0][0])*(p[3][0]-p[2][0]);
        double b = (p[1][0]-p[0][0])*(p[3][1]-p[2][1]);
        double c = (p[3][0]-p[2][0])*(p[1][1]-p[0][1]);

        center[0] = (a*p[0][1]+b*p[2][0]-a*p[2][1]-c*p[0][0])/(b-c);
        center[1] = ((p[1][1]-p[0][1])*(center[0]-p[0][0])/(p[1][0]-p[0][0]))+p[0][1];

        double x_l1 = Math.pow(p[0][0]-p[1][0], 2);
        double y_l1 = Math.pow(p[0][1]-p[1][1], 2);
        double x_l2 = Math.pow(p[2][0]-p[2][0], 2);
        double y_l2 = Math.pow(p[3][1]-p[3][1], 2);

        double avg = (Math.sqrt(x_l1+y_l1)+Math.sqrt(x_l2+y_l2))/2;

        center[2] = avg/0.07071067812;

        Log.d("AR_center[NEW]: ",center[0]+", "+center[1]);
        Log.d("AR_center[NEW]:",""+center[2]);

        return  center;
    }

}
