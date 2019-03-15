using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Series6AxisRobotArmKinematicsSimulator
{
    public class IsometricDraw
    {
        /* 2D座標用構造体
         */
        public struct Point2f
        {
            public float x;
            public float y;
            
        }

        /* 3D座標用構造体
         */
        public struct Point3f
        {
            public float x;
            public float y;
            public float z;
        }

        public Point2f originAtImage;       //画像座標系での原点
        private Point2f[] axisAtImage;      //画像座標での原点軸の頂点

        public float scale;                 //入力座標と画像座標の長さの係数

        /* コンストラクタ
         * @param[in] width 表示領域の幅
         * @param[in] height 表示領域の高さ
         * @param[in] length スケール(表示領域全体の長さをいくつと対応付けるか
         */ 
        public IsometricDraw(int width, int height, float length)
        {
            float imageLength;
            if (width > height) { imageLength = height; }
            else { imageLength = width; }
            this.originAtImage.x = width / 2;
            this.originAtImage.y = height / 2;
            scale = (float)imageLength / length;
            //scale = length/ (float)imageLength;

            Point3f[] axisAt3D = new Point3f[3];
            Point2f[] axisAt2D = new Point2f[3];
            axisAtImage= new Point2f[3];
            axisAt3D[0].x = 0.2f*length;
            axisAt3D[1].y = 0.2f*length;
            axisAt3D[2].z = 0.2f*length;

            for (int i = 0; i < 3; i++)
            {
                axisAt2D[i] = Convert3DTo2D(axisAt3D[i]);
                axisAtImage[i] = Convert2DToImage(axisAt2D[i]);
            }
        }

        /* ビットマップに座標軸を表示
         * @param[in]  bitmap 描画するビットマップ
         */ 
        public void DrawOriginalAxis(Bitmap bitmap)
        {
            Graphics g = Graphics.FromImage(bitmap);
            Pen redPen = new Pen(Color.Red, 3);
            Pen greenPen = new Pen(Color.Green, 3);
            Pen bluePen = new Pen(Color.Blue, 3);

            redPen.EndCap = LineCap.ArrowAnchor;
            greenPen.EndCap = LineCap.ArrowAnchor;
            bluePen.EndCap = LineCap.ArrowAnchor;

            g.DrawLine(redPen, originAtImage.x, originAtImage.y, axisAtImage[0].x, axisAtImage[0].y);
            g.DrawLine(greenPen, originAtImage.x, originAtImage.y, axisAtImage[1].x, axisAtImage[1].y);
            g.DrawLine(bluePen, originAtImage.x, originAtImage.y, axisAtImage[2].x, axisAtImage[2].y);

            Font fnt = new Font("MS UI Gothic", 10);
            //文字列を位置(0,0)、青色で表示
            g.DrawString("X", fnt,Brushes.Red, axisAtImage[0].x, axisAtImage[0].y);
            g.DrawString("Y", fnt, Brushes.Green, axisAtImage[1].x-10, axisAtImage[1].y);
            g.DrawString("Z", fnt, Brushes.Blue, axisAtImage[2].x, axisAtImage[2].y-10);

            redPen.Dispose();
            greenPen.Dispose();
            bluePen.Dispose();
            g.Dispose();
        }


        /* 通常の２次元座標を画像座標系に変換
         * @param[in] coordinate2D 変換元の２次元座標
         * @return 変換された座標
         */
        public Point2f Convert2DToImage(Point2f coordinate2D)
        {
            Point2f coordinateImage;
            coordinateImage.x = originAtImage.x + scale * coordinate2D.x;
            coordinateImage.y = originAtImage.y - scale * coordinate2D.y;
            return coordinateImage;
        }


        /* ３次元座標を等角投影法で表された2次元座標に変換
         * @param[in] coordinate3D 変換元の3次元座標
         * @return 変換された座標
         */
        public Point2f Convert3DTo2D(Point3f coordinate3D)
        {
            Point2f coordinate2D;
            coordinate2D.x = coordinate3D.x * (float)Math.Cos(Math.PI * (30.0 / 180.0)) +
                             coordinate3D.y * (float)Math.Cos(Math.PI * (150.0 / 180.0));
            coordinate2D.y = coordinate3D.x * (float)Math.Sin(Math.PI * (30.0 / 180.0)) +
                             coordinate3D.y * (float)Math.Sin(Math.PI * (150.0 / 180.0)) +
                             coordinate3D.z;
            return coordinate2D;
        }

        /* 3次元座標を画像座標に変換
         * @param[in] coordinate3D 変換元の3次元座標
         * @return 変換された座標
         */
        public Point2f Convert3DToImage(Point3f coordinate3D)
        {
            Point2f coordinate2D, coordinateImage;
            coordinate2D = Convert3DTo2D(coordinate3D);
            coordinateImage = Convert2DToImage(coordinate2D);
            return coordinateImage;
        }
    }
}
