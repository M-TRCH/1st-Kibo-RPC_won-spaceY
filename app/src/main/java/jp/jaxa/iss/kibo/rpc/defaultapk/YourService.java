package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class YourService extends KiboRpcService
{
    @Override
    protected void runPlan1()
    {
        api.judgeSendStart();
        api.getBitmapNavCam();api.getMatNavCam();





        final double final_pz = moveTo(11.042f,-5.583f,4.360f,0.500f,0.500f,-0.500f, 0.500f,2);
        final double final_px = moveTo(11.440f,-5.659f,4.583f,0.000f,0.000f, 0.000f, 1.000f,0);
        final double final_py = moveTo(11.083f,-6.042f,5.440f,0.707f,0.000f, 0.707f, 0.000f,1);
        /* bay_3:KOZ_2-KOZ_3 */ moveTo(10.500f,-6.450f,5.262f,0.000f,0.000f, 0.000f, 0.000f);
        final double final_qy = moveTo(11.490f,-7.958f,5.083f,0.000f,0.000f, 0.000f, 1.000f,4);
        final double final_qz = moveTo(11.083f,-7.742f,5.440f,0.707f,0.000f, 0.707f, 0.000f,5);
        final double final_qx = moveTo(10.410f,-7.542f,4.783f,0.000f,0.000f, 1.000f, 0.000f,3);
        final double final_qw = find_w(final_qx, final_qy, final_qz);
        final double[] pos_ar = moveTo(10.950f,-9.590f,5.410f,0.000f,0.000f, 0.707f,-0.707f,"");





        double pos_x = ((pos_ar[0]-640)/pos_ar[2])+10.95+0.0994-0.0572+0.1414+0.00;
        double pos_z = ((pos_ar[1]-480)/pos_ar[2])+5.410-0.0285+0.1111+0.1414+0.00;
        double pos_y = -9.590;

        pos_x = limit('x', pos_x);
        pos_y = limit('y', pos_y);
        pos_z = limit('z', pos_z);




        moveTo((float)pos_x, (float)pos_y, (float)pos_z,0.000f,0.000f,0.707f,-0.707f);

        Log.d("[FINAL.POINT]\n","px : "+final_px+"\npy : "+final_py+"\npz : "+final_pz);
        Log.d("[FINAL.QUATERNION]\n","qx : "+final_qx+"\nqy : "+final_qy+"\nqz : "+final_qz+"\nqw : "+final_qw);
        Log.d("[AR.POSITION]\n","x : "+pos_ar[0]+"\ny : "+pos_ar[1]+"\nscale : "+pos_ar[2]);

        /*
        PointCloud Haz_Cam = api.getPointCloudHazCam();
        int height = Haz_Cam.getHeight();
        int wight = Haz_Cam.getWidth();
        Point[] final_point;

        final_point = Haz_Cam.getPointArray();
        double o = final_point[0].getZ();

        Log.d("Final_Point  1: ", Integer.toString(wight));
        Log.d("Final_Point  2: ", Integer.toString(height));
        Log.d("Final_Point  3: ", Double.toString(o));
        */

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
        Result result = api.moveTo(point, quaternion, true);

        while (!result.hasSucceeded())
        {
            result = api.moveTo(point, quaternion, true);
        }
    }
    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
    {
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        Result result = api.moveTo(point, quaternion, true);

        while (!result.hasSucceeded())
        {
            result = api.moveTo(point, quaternion, true);
        }
    }
    public double moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, int no)
    {
        String contents = null;
        int count = 0, count_max = 3;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        while(contents == null && count < count_max)
        {
            moveTo(point, quaternion);
            Bitmap source = api.getBitmapNavCam();

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
                Log.d("QR["+no+"]: ", contents);
            }
            count++;
        }
        String[] val_array = contents.split(", ");
        double val_return = Double.parseDouble(val_array[1]);
        api.judgeSendDiscoveredQR(no, contents);
        return val_return;
    }
    public double[] moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw, String ar)
    {
        int AR_int = 0;
        int x[] = new int[4], y[] = new int[4];
        double avg[] = new double[5], center[] = new double[6], result[] = new double[3];

        while(AR_int == 0)
        {
            moveTo(px, py, pz, qx, qy, qz, qw);

            Mat source = api.getMatNavCam();
            Mat ids = new Mat();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            DetectorParameters setting = null;

            try
            {
                // CORNER_REFINE_NONE, CORNER_REFINE_SUBPIX, CORNER_REFINE_CONTOUR, CORNER_REFINE_APRILTAG
                //setting.set_cornerRefinementMethod(CORNER_REFINE_APRILTAG);
                Aruco.detectMarkers(source, dictionary, corners, ids);
                AR_int = (int)ids.get(0, 0)[0];
            }
            catch (Exception e)
            {
                AR_int = 0;
            }
            if(AR_int != 0)
            {
                Log.d("AR["+ar+"]: ", ""+AR_int);
                x[0] = (int)corners.get(0).get(0, 0)[0]; // x ขวาบน
                y[0] = (int)corners.get(0).get(0, 0)[1]; // y ขวาบน
                x[1] = (int)corners.get(0).get(0, 1)[0]; // x ซ้ายบน
                y[1] = (int)corners.get(0).get(0, 1)[1]; // y ซ้ายบน
                x[2] = (int)corners.get(0).get(0, 2)[0]; // x ขวาล่าง
                y[2] = (int)corners.get(0).get(0, 2)[1]; // y ขวาล่าง
                x[3] = (int)corners.get(0).get(0, 3)[0]; // x ซ้ายล่าง
                y[3] = (int)corners.get(0).get(0, 3)[1]; // y ซ้ายล่าง
            }
        }
        String AR_value = Integer.toString(AR_int); api.judgeSendDiscoveredAR(AR_value);

        avg[0] = (double)Math.abs(y[2]-y[0]);  // r
        avg[1] = (double)Math.abs(x[0]-x[1]);  // t
        avg[2] = (double)Math.abs(y[1]-y[3]);  // l
        avg[3] = (double)Math.abs(x[3]-x[2]);  // b
        avg[4] = (avg[0]+avg[1]+avg[2]+avg[3])/4;
        center[0] = avg[0]/2;
        center[1] = avg[1]/2;
        center[2] = avg[2]/2;
        center[3] = avg[3]/2;
        center[4] = (center[1]+center[3])/2; // half x
        center[5] = (center[0]+center[2])/2; // half y

        result[0] = x[0]-center[4]; // x point > range 0-1279
        result[1] = y[0]+center[5]; // y point > range 0-959
        result[2] = avg[4]/0.05;    // ratio > pixel:meter

        return result;
    }
    public double find_w(double qx,double qy,double qz)
    {
        double qw = Math.sqrt(1-qx*qx-qy*qy-qz*qz);
        return qw;
    }
    public double limit(char axis, double val)
    {
        double result = val;

        if(axis == 'x')
        {
            if (result > 11.49) { result = 11.49; }
            if (result < 10.41) { result = 10.41; }
        }
        if(axis == 'y')
        {
            if (result > -3.16) { result = -3.16; }
            if (result < -9.59) { result = -9.59; }
        }
        if(axis == 'z')
        {
            if (result > 5.44) { result = 5.44; }
            if (result < 4.36) { result = 4.36; }
        }
        return result;
    }
    public Quaternion vectorToquaternion(double pos_x, double pos_y, double pos_z)
    {
        double magnitude = Math.sqrt(pos_x*pos_x + pos_y*pos_y + pos_z*pos_z);
        double x_unit = pos_x/magnitude;
        double y_unit = pos_y/magnitude;
        double z_unit = pos_z/magnitude;

        double matrix[][] =
                {
                        {-0.189 , 0.841 , -0.507},
                        {x_unit, y_unit, z_unit}
                };

        double x, y, z, i, j, k, p, q,
               theta, a , b , c , w;

        x = (matrix[0][1]*matrix[1][2])-(matrix[1][1]*matrix[0][2]);
        y = (matrix[0][2]*matrix[1][0])-(matrix[1][2]*matrix[0][0]);
        z = (matrix[0][0]*matrix[1][1])-(matrix[1][0]*matrix[0][1]);

        i = matrix[0][0]-matrix[1][0];
        j = matrix[0][1]-matrix[1][1];
        k = matrix[0][2]-matrix[1][2];

        q = 1/Math.sqrt(x*x + y*y +z*z);
        p = i*i + j*j + k*k;
        theta = Math.acos((1-p/2)/2);

        a = Math.sin(theta)*x*q;
        b = Math.sin(theta)*y*q;
        c = Math.sin(theta)*z*q;
        w = Math.cos(theta);

        Quaternion final_qua = new Quaternion((float)a, (float)b, (float)c, (float)w);

        return final_qua;
    }
}
