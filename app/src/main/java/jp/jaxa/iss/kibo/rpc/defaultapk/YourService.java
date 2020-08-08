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
import org.opencv.core.MatOfPoint;
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

        moveTo(10.9263f, -5.2426f, 4.4622f, 0.0f, 0.0f, 0.0f, 0.0f);

        boolean run = true;
        Mat src_mat = new Mat(1280,960, CvType.CV_8UC1);

        while(run)
        {
            moveTo(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 1.0f, 0.0f); // p1

            src_mat = api.getMatNavCam();
            src_mat = undistord(src_mat);
            Rect crop = qr_detect(src_mat);

            if(crop.x != 0 && crop.y != 0 && crop.width != 1280 && crop.height != 960)
            {
                src_mat = new Mat(src_mat, crop);
                run = false;
            }
        }

        int width  = src_mat.width()/2;
        int height = src_mat.height()/2;

        Size size = new Size(width, height);
//        Imgproc.resize(src_mat, src_mat, size);

        int count=1;
        for(int i=0; i<src_mat.height(); i++)
        {
            for (int j = 0; j < src_mat.width(); j++)
            {
                double[] val = src_mat.get(i, j);

                Log.d("Mat["+i+", "+j+"]: ",""+val[0]);
                SystemClock.sleep(1);
            }
            if (i > 20*count)
            {
                moveTo(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 1.0f, 0.0f);
                count++;
            }
        }




        api.laserControl(true);
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
        Result result;
        do
            {
            result = api.moveTo(point, quaternion, true);
        }
        while (!result.hasSucceeded());
    }

    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
    {
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        moveTo(point, quaternion);
    }

    public double[] moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, int no)
    {
        String contents = null;
        int count = 0, count_max = 5;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        while (contents == null && count < count_max)
        {
            moveTo(point, quaternion);
            SystemClock.sleep(1000);

            Mat src_mat = api.getMatNavCam();
            src_mat = undistord(src_mat);

            double scale = 12;
            int width = 1280, height = 960;


            Rect crop = qr_detect(src_mat);
            if(crop.x != 0 && crop.y != 0 && crop.width != 1280 && crop.height != 960)
            {
                src_mat = new Mat(src_mat, crop);

//                width  = (int)(scale*src_mat.width());
//                height = (int)(scale*src_mat.height());
//
//                Size size = new Size(width, height);
//                Imgproc.resize(src_mat, src_mat, size);
            }

            Bitmap bMap = Bitmap.createBitmap(src_mat.width(), src_mat.height(), Bitmap.Config.ARGB_8888);
            matToBitmap(src_mat, bMap, false);
            Log.d("QR[crop]:", " " + bMap.getWidth() + ", " + bMap.getHeight());



            Log.d("QR[" + no + "][status]:", " start");

            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try
            {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                contents = result.getText();
                Log.d("QR[" + no + "][value]:", " " + contents);
            }
            catch (Exception e)
            {
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

    public double[] moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, String ar)
    {
        int AR_int = 0, count = 0;
        double result[] = new double[6];
        Point point = new Point();

        while (AR_int == 0) {
            Log.d("AR_counter: ", "" + count);

            moveTo(px, py, pz, qx, qy, qz, qw);

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

    public double find_w(double qx, double qy, double qz)
    {
        double qw = Math.sqrt(1 - qx * qx - qy * qy - qz * qz);
        return qw;
    }

    public double limit(char axis, double val) {
        double result = val;

        if (axis == 'x') {
            if (result > 11.49) {
                result = 11.49;
            }
            if (result < 10.41) {
                result = 10.41;
            }
        }
        if (axis == 'y') {
            if (result > -3.16) {
                result = -3.16;
            }
            if (result < -9.59) {
                result = -9.59;
            }
        }
        if (axis == 'z') {
            if (result > 5.44) {
                result = 5.44;
            }
            if (result < 4.36) {
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

        cameraMatrix.put(row, col, cameraMatrix_sim);
        distCoeffs.put(row, col, distCoeffs_sim);

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

    public Rect qr_detect(Mat src)
    {
        Rect crop = new Rect(0,0,1280,960);

        int x_max=0, y_max=0, x_min=960, y_min=1280;
        boolean state = false;

        Log.e("Rect[status]:", " start");
        Mat src_convert = new Mat(src.size(), CvType.CV_8UC3);
        Mat src_blur    = new Mat(src.size(), CvType.CV_8UC3);
        Mat src_out     = new Mat(src.size(), CvType.CV_8UC3);


        Imgproc.cvtColor(src, src_convert, Imgproc.COLOR_GRAY2BGR);
        Imgproc.cvtColor(src_convert, src_convert, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(src_convert, src_blur, new Size(5, 5), 0);
        Imgproc.adaptiveThreshold(src_blur, src_out, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 21, 1);


        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(src_out, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Log.e("Rect[status]:", " stop");


        for (int i = 0; i < contours.size(); i++)
        {
            if ((Imgproc.contourArea(contours.get(i)) > 500 && Imgproc.contourArea(contours.get(i)) < 8000))
            {
                Rect rect = Imgproc.boundingRect(contours.get(i));

                if(Math.abs(rect.height-rect.width) < 5)//&& rect.width > 150 && rect.height > 150)
                {
                    Log.d("Rect/"+i+"/", " x: " + rect.x + ", y: " + rect.y + ", width: " + rect.width + ", height: " + rect.height);

                    if(x_max < rect.x+rect.width)  x_max = rect.x+rect.width;
                    if(y_max < rect.y+rect.height) y_max = rect.y+rect.height;
                    if(x_min > rect.x)  x_min = rect.x;
                    if(y_min > rect.y)  y_min = rect.y;
                    state = true;
                }
                else
                {
                    Log.d("Rect["+i+"]", " x: " + rect.x + ", y: " + rect.y + ", width: " + rect.width + ", height: " + rect.height);
                }
            }
        }
        if(state == true)
        {
            if(Math.abs(x_max-x_min-y_max+y_min) < 20)
            {
                int offset = 5;
                crop = new Rect(x_min-offset, y_min-offset, x_max-x_min+(2*offset), y_max-y_min+(2*offset));
            }
        }
        Log.d("Rect[]: ",""+x_min+", "+y_min+", "+x_max+", "+y_max);

        return crop;
    }
}