package jp.jaxa.iss.kibo.rpc.thailand;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;


import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

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
        api.judgeSendStart();

        final double[] pos, qua;
        double qua_w;

        moveToWrapper(10.9263f, -5.2426f, 4.4622f, 0.0f, 0.0f, 0.0f, 0.0f); //p1

        pos = moveTo(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 1.0f, 0.0f, 0); //p1

        moveToWrapper(10.7600f, -5.7100f, 4.9100f, 0.0f, 0.0f, 0.0f, 0.0f);
        moveToWrapper(10.4600f, -6.0800f, 5.2400f, 0.0f, 0.0f, 0.0f, 0.0f);
        moveToWrapper(10.4600f, -7.6800f, 5.2400f, 0.0f, 0.0f, 0.0f, 0.0f);

        qua = moveTo(10.9774f, -7.6378f, 5.4000f, 0.0f, -0.7071f, 0.0f, 0.7071f, 1); //p2
        qua_w = find_w(qua[0], qua[1], qua[2]);

        moveToWrapper(10.8000f, -8.5000f, 5.1500f, 0.0f, 0.0f, 0.7071f, -0.7071f);
        final double[] pos_ar = moveTo(10.950f, -9.590f, 5.410f, 0.000f, 0.000f, 0.707f, -0.707f, "");
//		  final double[] pos_ar = moveTo((float)pos[0], (float)pos[1], (float)pos[2], (float)qua[0], (float)qua[1], (float)qua[2], (float)qua_w,"");

        targetShoot(pos_ar[0], pos_ar[1], pos_ar[2], pos_ar[3], pos_ar[4], pos_ar[5]);

        api.laserControl(true);
        api.judgeSendFinishISS();
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
        Result result;
        do {
            result = api.moveTo(point, quaternion, true);
        }
        while (!result.hasSucceeded());
    }

    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw) {
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        moveTo(point, quaternion);
    }

    public double[] moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, int no) {
        String contents = null;
        int count = 0, count_max = 10;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        while (contents == null && count < count_max) {
//            moveTo(point, quaternion);
            moveToWrapper(px, py, pz, qx, qy, qz, qw);

            Mat src_mat = api.getMatNavCam();
            src_mat = undistord(src_mat);

            double ratio = 1280.0 / 960.0;
            double percent_row = 20;
            double percent_col = percent_row * ratio;
            int max_row = 960;
            int max_col = 1280;
            int offset_row = ((int) percent_row * max_row) / 100;
            int offset_col = ((int) percent_col * max_col) / 100;
            double rows = max_row - (offset_row * 2);
            double cols = max_col - (offset_col * 2);

            Rect rect = new Rect(offset_col, offset_row, (int) cols, (int) rows);
            Mat crop = new Mat(src_mat, rect);

            Size size = new Size(2000, 1500);
            Imgproc.resize(crop, crop, size);

            Bitmap bMap = Bitmap.createBitmap(2000, 1500, Bitmap.Config.ARGB_8888);
            matToBitmap(crop, bMap, false);
            Log.d("QR[crop]:", " " + bMap.getWidth() + ", " + bMap.getHeight());

            Log.d("QR[" + no + "][status]:", " start");

            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                contents = result.getText();
                Log.d("QR[" + no + "][value]:", " " + contents);
            } catch (Exception e) {
                Log.d("QR" + no + "[status]:", " Not detected");
            }

            Log.d("QR[" + no + "][count]:", " " + count);
            Log.d("QR[" + no + "][status]:", " stop");

            count++;
        }
        api.judgeSendDiscoveredQR(no, contents);

        String[] multi_contents = contents.split(", ");
        Log.d("QR[" + no + "][x]:", " " + multi_contents[1]);
        Log.d("QR[" + no + "][y]:", " " + multi_contents[3]);
        Log.d("QR[" + no + "][z]:", " " + multi_contents[5]);

        final double final_x = Double.parseDouble(multi_contents[1]);
        final double final_y = Double.parseDouble(multi_contents[3]);
        final double final_z = Double.parseDouble(multi_contents[5]);

        final double[] val_return = {final_x, final_y, final_z};

        return val_return;
    }

    public double[] moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, String ar) {
        int AR_int = 0, count = 0;
        double result[] = new double[6];
        Point point = new Point();

        while (AR_int == 0) {
            Log.d("AR_counter: ", "" + count);

            moveToWrapper(px, py, pz, qx, qy, qz, qw);
//            moveTo(px, py, pz, qx, qy, qz, qw);

            Mat source = undistord(api.getMatNavCam());
            //Mat source = api.getMatNavCam();
            Mat ids = new Mat();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            DetectorParameters setting = null;

            try {
                Aruco.detectMarkers(source, dictionary, corners, ids);
                AR_int = (int) ids.get(0, 0)[0];
                Kinematics current = api.getTrustedRobotKinematics();
                point = current.getPosition();
            } catch (Exception e) {
                AR_int = 0;
            }
            if (AR_int != 0) {
                Log.d("AR[" + count + "]: ", "" + AR_int);

                double[][] AR_corners =
                        {
                                {(int) corners.get(0).get(0, 0)[0], (int) corners.get(0).get(0, 0)[1]},
                                {(int) corners.get(0).get(0, 2)[0], (int) corners.get(0).get(0, 2)[1]},
                                {(int) corners.get(0).get(0, 1)[0], (int) corners.get(0).get(0, 1)[1]},
                                {(int) corners.get(0).get(0, 3)[0], (int) corners.get(0).get(0, 3)[1]}
                        };
                double[] AR_info = interceptLine(AR_corners);

                result[0] = AR_info[0];
                result[1] = AR_info[1];
                result[2] = AR_info[2];
            }
        }
        String AR_value = Integer.toString(AR_int);
        api.judgeSendDiscoveredAR(AR_value);
        result[3] = point.getX();
        result[4] = point.getY();
        result[5] = point.getZ();

        return result;
    }

    public double find_w(double qx, double qy, double qz) {
        double qw = Math.sqrt(1 - qx * qx - qy * qy - qz * qz);
        return qw;
    }

    public double limit(char axis, double val)
    {
        double result = val;

        if (axis == 'x') {
            if (result > 11.49)
            {
                result = 11.49;
            }
            if (result < 10.41)
            {
                result = 10.41;
            }
        }
        if (axis == 'y')
        {
            if (result > -3.16)
            {
                result = -3.16;
            }
            if (result < -9.59)
            {
                result = -9.59;
            }
        }
        if (axis == 'z')
        {
            if (result > 5.44)
            {
                result = 5.44;
            }
            if (result < 4.36)
            {
                result = 4.36;
            }
        }
        return result;
    }

    public void targetShoot(double px, double py, double d, double pos_a, double pos_b, double pos_c)
    {
        double targetShift = 0.2 * Math.sin(Math.toRadians(45));
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
        double r = Math.sqrt(matrix[0][0] * matrix[0][0] + matrix[0][1] * matrix[0][1] + matrix[0][2] * matrix[0][2]);

        // double theta = Math.acos((p*p-2)/2*q*r*(-1));
        double theta = Math.acos((r * r + q * q - p * p) / (2 * q * r));

        double a = Math.sin(theta / 2) * x / q;
        double b = Math.sin(theta / 2) * y / q;
        double c = Math.sin(theta / 2) * z / q;
        double w = Math.cos(theta / 2);

        moveTo((float) pos_a, (float) pos_b, (float) pos_c, (float) a, (float) b, (float) c, (float) w);
    }

    public Mat undistord(Mat src)
    {
        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        int row = 0, col = 0;

        double cameraMatrix_sim[] =
                {
                        344.173397, 0.000000, 630.793795,
                        0.000000, 344.277922, 487.033834,
                        0.000000, 0.000000, 1.000000
                };
        double distCoeffs_sim[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};

        double cameraMatrix_orbit[] =
                {
                        692.827528, 0.000000, 571.399891,
                        0.000000, 691.919547, 504.956891,
                        0.000000, 0.000000, 1.000000
                };
        double distCoeffs_orbit[] = {-0.312191, 0.073843, -0.000918, 0.001890, 0.000000};

        cameraMatrix.put(row, col, cameraMatrix_orbit);
        distCoeffs.put(row, col, distCoeffs_orbit);

        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
        return dst;
    }

    public double[] interceptLine(double p[][])
    {
        double center[] = new double[3];

        double a = (p[1][0] - p[0][0]) * (p[3][0] - p[2][0]);
        double b = (p[1][0] - p[0][0]) * (p[3][1] - p[2][1]);
        double c = (p[3][0] - p[2][0]) * (p[1][1] - p[0][1]);

        center[0] = (a * p[0][1] + b * p[2][0] - a * p[2][1] - c * p[0][0]) / (b - c);
        center[1] = ((p[1][1] - p[0][1]) * (center[0] - p[0][0]) / (p[1][0] - p[0][0])) + p[0][1];

        double x_l1 = Math.pow(p[0][0] - p[1][0], 2);
        double y_l1 = Math.pow(p[0][1] - p[1][1], 2);
        double x_l2 = Math.pow(p[2][0] - p[2][0], 2);
        double y_l2 = Math.pow(p[3][1] - p[3][1], 2);

        double avg = (Math.sqrt(x_l1 + y_l1) + Math.sqrt(x_l2 + y_l2)) / 2;

        center[2] = avg / 0.07071067812;

        Log.d("AR_center[NEW]: ", center[0] + ", " + center[1]);
        Log.d("AR_center[NEW]:", "" + center[2]);

        return center;
    }

    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w)
    {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }
}