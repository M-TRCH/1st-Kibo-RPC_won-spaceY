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
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

import static org.opencv.android.Utils.matToBitmap;

public class YourService extends KiboRpcService
{
    @Override
    protected void runPlan1()
    {
        api.judgeSendStart();

        float z_shift = 0.0f;


        moveTo(10.76f, -5.16f, 4.52f+z_shift, 10.70f, -5.16f, 4.42f+z_shift, 0);

        //moveTo(10.9263f, -5.2426f, 4.4622f, 0.0f, 0.0f, 0.0f, 0.0f);
        //moveTo(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 1.0f, 0.0f,0); // p1




        /*
        boolean run = true;
        Mat src_mat = new Mat(960,1280, CvType.CV_8UC1);

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
        */

//        PointCloud point_cloud = api.getPointCloudHazCam();
//        Point[] point = point_cloud.getPointArray();
//
//        double[] y_point = new double[point_cloud.getWidth()*point_cloud.getHeight()];
//
//        for(int row=1; row<point_cloud.getHeight()+1; row++)
//        {
//            for(int col=0; col<point_cloud.getWidth(); col++)
//            {
//                y_point[row*col] = point[row*col].getY();
//            }
//        }







//        moveTo(10.7600f, -5.2426f, 4.4622f, 0.0f, 0.0f, 0.7071f, -0.7071f);

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
        int count=0, max_count=3;

        do
        {
            result = api.moveTo(point, quaternion, true);
            count++;
        }
        while (!result.hasSucceeded() && count<=max_count);
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
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);



        double percent = 40;
        int max_row = 960;
        int max_col = 1280;
        int offset_row = (int)percent/2*max_row/100;
        int offset_col = (int)percent/2*max_col/100;
        int rows = max_row-(offset_row*2);
        int cols = max_col-(offset_col*2);


        Mat[] src_ = new Mat[5];

        int count_out = 0, maxCount_out = 6;

        do
        {
            moveTo(point, quaternion);


            Log.d("Capture[]:", " start");
            src_[0] = undistord(api.getMatNavCam());
            src_[1] = undistord(api.getMatNavCam());
            Log.d("Capture[]:", " stop");

            int count = 0, count_max = 2;
            while (contents == null && count < count_max)
            {

//            boolean i=false, j=false, k=false;
//            Kinematics current;
//            Point p;
//            do
//            {
//                moveTo(point, quaternion);
//
//
//                current = api.getTrustedRobotKinematics();
//                p = current.getPosition();
//                if(p.getX()-px+0.1 < 0.1 && p.getX()-px-0.1 > 0.1){ i=true; }
//                if(p.getY()-py+0.1 < 0.1 && p.getY()-py-0.1 > 0.1){ j=true; }
//                if(p.getZ()-pz+0.1 < 0.1 && p.getZ()-pz-0.1 > 0.1){ k=true; }
//            }
//            while(i && j && k);

                Mat src_mat = src_[count];

                Rect crop = qr_detect(src_mat, 500, 8000, 5, 200);


                Rect rect = new Rect(offset_col, offset_row, cols, rows);
                if (crop.x != 0 && crop.y != 0 && crop.width != 1280 && crop.height != 960)
                {
                    rect = new Rect(crop.x, crop.y, cols, rows);
                    Log.d("", "NO FIX");
                    ////
                }

                src_mat = new Mat(src_mat, rect);
                Log.d("QR[crop]:", " " + src_mat.width() + ", " + src_mat.height());
                Size size = new Size(2000, 1500);
                Imgproc.resize(src_mat, src_mat, size);
                Log.d("QR[crop]:", " " + src_mat.width() + ", " + src_mat.height());


                /*
                src_mat = new Mat(src_mat, crop);
                Log.d("QR[crop]:", " " + src_mat.width() + ", " + src_mat.height());

                double scale = 24;
                int width  = (int)(scale*src_mat.width());
                int height = (int)(scale*src_mat.height());

                Size size = new Size(width, height);
                Imgproc.resize(src_mat, src_mat, size);
                */

                Log.d("QR[crop]:", " " + src_mat.width() + ", " + src_mat.height());
                Bitmap bMap = Bitmap.createBitmap(src_mat.width(), src_mat.height(), Bitmap.Config.ARGB_8888);
                matToBitmap(src_mat, bMap, false);
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
        }
        while ((contents == null && count_out < maxCount_out));



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

        while (AR_int == 0)
        {
            Log.d("AR_counter: ", "" + count);

            moveTo(px, py, pz, qx, qy, qz, qw);

            Mat source = undistord(api.getMatNavCam());
            //Mat source = api.getMatNavCam();
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
        if (axis == 'y') {
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

    public Rect qr_detect(Mat src, int min_contour, int max_contour, int dif_rect, int offset)
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


        for (int no = 0; no < contours.size(); no++)
        {
            if (Imgproc.contourArea(contours.get(no)) > min_contour && Imgproc.contourArea(contours.get(no)) < max_contour)
            {
                Rect rect = Imgproc.boundingRect(contours.get(no));

                if(Math.abs(rect.height-rect.width) < dif_rect)
                {
                    Log.d("Rect/"+no+"/", " x: " + rect.x + ", y: " + rect.y + ", width: " + rect.width + ", height: " + rect.height);

                    if(x_max < rect.x+rect.width)  x_max = rect.x+rect.width;
                    if(y_max < rect.y+rect.height) y_max = rect.y+rect.height;
                    if(x_min > rect.x)  x_min = rect.x;
                    if(y_min > rect.y)  y_min = rect.y;
                    state = true;
                }
                else
                {
                    Log.d("Rect["+no+"]", " x: " + rect.x + ", y: " + rect.y + ", width: " + rect.width + ", height: " + rect.height);
                }
            }
        }
        if(state && Math.abs(x_max-x_min-y_max+y_min) < dif_rect)
        {
//            crop = new Rect(x_min-offset, y_min-offset, x_max-x_min+(2*offset), y_max-y_min+(2*offset));


            int widht  = x_max-x_min;
            int height = y_max-y_min;
            int offset_x = (512-widht)/2;
            int offset_y = (384-height)/2;

//            int offset_y = 0; //(672-height)/2;
//            int offset_x = (int)((height*4.0/3.0-widht)/2);




            crop = new Rect(x_min-offset_x, y_min-offset_y, widht+(2*offset_x), height+(2*offset_y));
        }
        Log.d("Rect[]: ",""+x_min+", "+y_min+", "+x_max+", "+y_max);
        Log.d("Rect[]: ",""+crop.x+", "+crop.y+", "+crop.width+", "+crop.height);

        return crop;
    }

    public void moveTo(float x_org, float y_org, float z_org, float x_des, float y_des, float z_des)
    {
        double navShift_z = 0.0826;
        double navShift_y = 0.0422;

        double dx = x_des-x_org;
        double dy = y_des-y_org;
        double dz = z_des-z_org;

        double magnitude = Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
        double x_unit = dx/magnitude;
        double y_unit = dy/magnitude;
        double z_unit = dz/magnitude;

        double matrix[][] =
                {
                        {1, 0, 0},
                        {x_unit, y_unit, z_unit}
                };

        double x = matrix[0][1]*matrix[1][2] - matrix[1][1]*matrix[0][2];
        double y = matrix[0][2]*matrix[1][0] - matrix[1][2]*matrix[0][0];
        double z = matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1];

        double i = matrix[1][0]-matrix[0][0];
        double j = matrix[1][1]-matrix[0][1];
        double k = matrix[1][2]-matrix[0][2];

        double q = Math.sqrt(x*x + y*y + z*z);
        double p = Math.sqrt(i*i + j*j + k*k);
        double r = Math.sqrt(matrix[0][0]*matrix[0][0]
                            +matrix[0][1]*matrix[0][1]
                            +matrix[0][2]*matrix[0][2]);

        double theta = Math.acos((r*r + q*q - p*p) / (2*q*r));

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        moveTo(x_org, y_org+(float)navShift_y, z_org, (float)a, (float)b, (float)c, (float)w);
    }
    public double[] moveTo(float x_org, float y_org, float z_org, float x_des, float y_des, float z_des, int no)
    {
        String contents = null;

        double percent = 40;
        int max_row = 960;
        int max_col = 1280;
        int offset_row = (int)percent/2*max_col/100;
        int offset_col = (int)percent/2*max_col/100;
        int rows = max_row-(offset_row*2);
        int cols = max_col-(offset_col*2);
        Rect crop = new Rect(offset_col, offset_row, cols, rows);



        int count = 0, count_max = 4;


        while (contents == null && count < count_max)
        {
            moveTo( x_org,  y_org,  z_org,  x_des,  y_des, z_des);
            Log.d("Capture[]:", " start");
            Mat src_mat = api.getMatNavCam();
            Log.d("Capture[]:", " stop");


            src_mat = new Mat(src_mat, crop);
            Size size = new Size(2000, 1500);
            Imgproc.resize(src_mat, src_mat, size);


            Log.d("QR[crop]:", " " + src_mat.width() + ", " + src_mat.height());
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
}