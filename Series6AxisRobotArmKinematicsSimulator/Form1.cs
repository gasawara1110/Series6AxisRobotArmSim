using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Series6AxisRobotArmKinematicsSimulator
{
    public partial class Form1 : Form
    {
        
        RobotArmState Robot;
        IsometricDraw Idraw;

        /// <summary>
        /// 角度を[deg]から[rad]に変換
        /// </summary>
        /// <param name="angleDeg">入力角度[deg]</param>
        /// <returns>変換された角度[rad]</returns>
        public float ConvertDegToRad(float angleDeg)
        {
            return (float)Math.PI * angleDeg / 180.0f;
        }

        /// <summary>
        /// 角度を[deg]から[rad]に変換
        /// </summary>
        /// <param name="angleDeg">入力角度[deg]</param>
        /// <returns>変換された角度[rad]</returns>
        public double ConvertDegToRad(double angleDeg)
        {
            return Math.PI * angleDeg / 180.0;
        }

        /// <summary>
        /// 角度を[rad]から[deg]に変換
        /// </summary>
        /// <param name="angleRad">入力角度[rad]</param>
        /// <returns>変換された角度[deg]</returns>
        public float ConvertRadToDeg(float angleRad)
        {
            return 180.0f * angleRad / (float)Math.PI;
        }

        /// <summary>
        /// 角度を[rad]から[deg]に変換
        /// </summary>
        /// <param name="angleRad">入力角度[rad]</param>
        /// <returns>変換された角度[deg]</returns>
        public double ConvertRadToDeg(double angleRad)
        {
            return 180.0 * angleRad / Math.PI;
        }

        /// <summary>
        /// トラックバーから角度を読み取る
        /// </summary>
        /// <returns>読み取った角度配列[rad],要素数6</returns>
        public double[] ReadAngleFromTrackbar()
        {
            double[] angle = new double[6];
            angle[0] = ConvertDegToRad((float)tbJ0.Value * 0.1f);
            angle[1] = ConvertDegToRad((float)tbJ1.Value * 0.1f);
            angle[2] = ConvertDegToRad((float)tbJ2.Value * 0.1f);
            angle[3] = ConvertDegToRad((float)tbJ3.Value * 0.1f);
            angle[4] = ConvertDegToRad((float)tbJ4.Value * 0.1f);
            angle[5] = ConvertDegToRad((float)tbJ5.Value * 0.1f);
            return angle;
        }

        /// <summary>
        /// トラックバーから手先角度を読み取り
        /// </summary>
        /// <returns>手先角度配列[rad]、XYZの順、要素数3</returns>
        public double[] ReadEndAngleFromTrackBar()
        {
            double[] endAngle = new double[3];
            endAngle[0] = ConvertDegToRad((double)(tbRX.Value * 0.1));
            endAngle[1] = ConvertDegToRad((double)(tbRY.Value * 0.1));
            endAngle[2] = ConvertDegToRad((double)(tbRZ.Value * 0.1));
            return endAngle;
        }


        /// <summary>
        /// トラックバーに角度を代入
        /// </summary>
        /// <param name="endAngleRad">手先角度配列[rad]、XYZの順、要素数3</param>
        public void WriteEndAngleToTrackBar(double[] endAngleRad)
        {
            tbRX.Value = (int)(ConvertRadToDeg(endAngleRad[0]) * 10.0);
            tbRY.Value = (int)(ConvertRadToDeg(endAngleRad[1]) * 10.0);
            tbRZ.Value = (int)(ConvertRadToDeg(endAngleRad[2]) * 10.0);
        }


        /// <summary>
        /// トラックバーに角度を代入
        /// </summary>
        /// <param name="angleRad">書き込む角度配列[rad]、要素数6</param>
        public void WriteAngleToTrackbar(double[] angleRad)
        {
            tbJ0.Value = (int)(ConvertRadToDeg(angleRad[0]) * 10.0);
            tbJ1.Value = (int)(ConvertRadToDeg(angleRad[1]) * 10.0);
            tbJ2.Value = (int)(ConvertRadToDeg(angleRad[2]) * 10.0);
            tbJ3.Value = (int)(ConvertRadToDeg(angleRad[3]) * 10.0);
            tbJ4.Value = (int)(ConvertRadToDeg(angleRad[4]) * 10.0);
            tbJ5.Value = (int)(ConvertRadToDeg(angleRad[5]) * 10.0);
        }
        
        /// <summary>
        /// テキストボックスに角度を表示
        /// </summary>
        /// <param name="angleRad">関節角度配列[rad]、要素数6</param>
        public void WriteAngleToTextBox(double[] angleRad)
        {
            txtbJ0.Text = ConvertRadToDeg(angleRad[0]).ToString("F2");
            txtbJ1.Text = ConvertRadToDeg(angleRad[1]).ToString("F2");
            txtbJ2.Text = ConvertRadToDeg(angleRad[2]).ToString("F2");
            txtbJ3.Text = ConvertRadToDeg(angleRad[3]).ToString("F2");
            txtbJ4.Text = ConvertRadToDeg(angleRad[4]).ToString("F2");
            txtbJ5.Text = ConvertRadToDeg(angleRad[5]).ToString("F2");
        }

        /// <summary>
        /// テキストボックスに手先位置を表示
        /// </summary>
        /// <param name="endPos">手先位置構造体</param>
        public void WriteEndPosToTextBox(Kinematics.Position endPos)
        {
            txtbPosX.Text = endPos.x.ToString("F2");
            txtbPosY.Text = endPos.y.ToString("F2");
            txtbPosZ.Text = endPos.z.ToString("F2");
        }

        /// <summary>
        /// テキストボックスから手先位置を読み取り
        /// </summary>
        /// <returns>手先位置構造体</returns>
        public Kinematics.Position ReadEndPosFromTextBox()
        {
            Kinematics.Position endPos = new Kinematics.Position(
                                        double.Parse(txtbPosX.Text),
                                        double.Parse(txtbPosY.Text),
                                        double.Parse(txtbPosZ.Text));
            return endPos;
        }

        /// <summary>
        /// 姿勢角をテキストボックスに表示
        /// </summary>
        /// <param name="angleRad"></param>
        public void WriteEndAngleToTextBox(double[] angleRad)
        {
            txtbRotX.Text = ConvertRadToDeg(angleRad[0]).ToString("F2");
            txtbRotY.Text = ConvertRadToDeg(angleRad[1]).ToString("F2");
            txtbRotZ.Text = ConvertRadToDeg(angleRad[2]).ToString("F2");
        }

        public void UpdateAllUIFromState(RobotArmState state)
        {
            WriteAngleToTrackbar(state.JointAngle);
            WriteAngleToTextBox(state.JointAngle);
            WriteEndPosToTextBox(state.EndPos);
            WriteEndAngleToTextBox(state.CoordinateAngle);
            WriteEndAngleToTrackBar(state.CoordinateAngle);
        }
        
         /// <summary>
         /// bitmapにロボットを描画
         /// </summary>
         /// <param name="bitmap">描画対象のbitmapオブジェクト</param>
         /// <param name="linkPos3D">リンク位置構造体配列</param>
        public void DrawRobot(Bitmap bitmap, IsometricDraw.Point3f[] linkPos3D)
        {
            Graphics g = Graphics.FromImage(bitmap);
            IsometricDraw.Point2f[] linkPos2D = new IsometricDraw.Point2f[7];
            Point[] points = new Point[7];
            for (int i = 0; i < 7; i++)
            {
                linkPos2D[i] = Idraw.Convert3DToImage(linkPos3D[i]);
                points[i].X = (int)linkPos2D[i].x;
                points[i].Y = (int)linkPos2D[i].y;
            }
            Pen blackPen = new Pen(Color.Black, 3);
            for (int i = 0; i < 6; i++)
            { 
                g.DrawEllipse(blackPen, points[i].X - 3, points[i].Y - 3, 6, 6);
            }
            g.DrawLines(blackPen, points);
            pb3DView.Image = bitmap;
            g.Dispose();
        }

        /// <summary>
        /// bitmapにロボットを描画
        /// </summary>
        /// <param name="bitmap">描画対象のbitmapオブジェクト</param>
        /// <param name="linkPos">リンク位置構造体配列</param>
        public void DrawRobot(Bitmap bitmap,Kinematics.Position[] linkPos)
        {
            IsometricDraw.Point3f[] linkPos3D = new IsometricDraw.Point3f[7];
            for(int i = 0; i < linkPos3D.Length; i++)
            {
                linkPos3D[i].x = (float)linkPos[i].x;
                linkPos3D[i].y = (float)linkPos[i].y;
                linkPos3D[i].z = (float)linkPos[i].z;
            }
            DrawRobot(bitmap, linkPos3D);
        }


        /* トラックバー初期化用
         * トラックバーの値は小数点が表示できないので1=0.1[deg]として表示
         */
        public void InitTrackBar(TrackBar trackBar, float angleMinDeg, float angleMaxDeg)
        {
            trackBar.Minimum = (int)(angleMinDeg / 0.1f);
            trackBar.Maximum = (int)(angleMaxDeg / 0.1f);
            trackBar.Value = 0;
        }

        public Form1()
        {
            InitializeComponent();

            double[] L = { 25, 25, 120, 60, 60, 25, 25 };
            Robot = new RobotArmState(L, RobotArmState.EXPRESSION_METHOD.EULER_ZYX);
            Idraw = new IsometricDraw(pb3DView.Width, pb3DView.Height, 500);
            

            double[] angle = { 0, 0, 0, 0, 0, 0, 0 };
            
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);

            InitTrackBar(tbJ0, -180, 180);
            InitTrackBar(tbJ1, -180, 180);
            InitTrackBar(tbJ2, -180, 180);
            InitTrackBar(tbJ3, -180, 180);
            InitTrackBar(tbJ4, -180, 180);
            InitTrackBar(tbJ5, -180, 180);

            InitTrackBar(tbRX, -180, 180);
            InitTrackBar(tbRY, -180, 180);
            InitTrackBar(tbRZ, -180, 180);

            UpdateAllUIFromState(Robot);

        }

        private void tbJ0_Scroll(object sender, EventArgs e)
        {
            Robot.UpdateByFK(ReadAngleFromTrackbar());
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void tbJ1_Scroll(object sender, EventArgs e)
        {
            Robot.UpdateByFK(ReadAngleFromTrackbar());
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void tbJ2_Scroll(object sender, EventArgs e)
        {
            Robot.UpdateByFK(ReadAngleFromTrackbar());
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void tbJ3_Scroll(object sender, EventArgs e)
        {
            Robot.UpdateByFK(ReadAngleFromTrackbar());
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void tbJ4_Scroll(object sender, EventArgs e)
        {
            Robot.UpdateByFK(ReadAngleFromTrackbar());
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void tbJ5_Scroll(object sender, EventArgs e)
        {
            Robot.UpdateByFK(ReadAngleFromTrackbar());
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void btXPlus_Click(object sender, EventArgs e)
        {
            //var endPos = Robot.
            var endPos = Robot.EndPos;
            endPos.x += 10;
            try
            {
                Robot.UpdateByIK(endPos, Robot.RotMat);
            }
            catch(Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー",MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void btXMinus_Click(object sender, EventArgs e)
        {
            var endPos = Robot.EndPos;
            endPos.x -= 10;
            try
            {
                Robot.UpdateByIK(endPos, Robot.RotMat);
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void btYPlus_Click(object sender, EventArgs e)
        {
            var endPos = Robot.EndPos;
            endPos.y += 10;
            try
            {
                Robot.UpdateByIK(endPos, Robot.RotMat);
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void btYMinus_Click(object sender, EventArgs e)
        {
            var endPos = Robot.EndPos;
            endPos.y -= 10;
            try
            {
                Robot.UpdateByIK(endPos, Robot.RotMat);
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void btZPlus_Click(object sender, EventArgs e)
        {
            var endPos = Robot.EndPos;
            endPos.z += 10;
            try
            {
                Robot.UpdateByIK(endPos, Robot.RotMat);
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void btZMinus_Click(object sender, EventArgs e)
        {
            var endPos = Robot.EndPos;
            endPos.z -= 10;
            try
            {
                Robot.UpdateByIK(endPos, Robot.RotMat);
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
        }

        private void tbRX_Scroll(object sender, EventArgs e)
        {
            try
            {
                Robot.UpdateByIK(ReadEndPosFromTextBox(), ReadEndAngleFromTrackBar());
            }
            catch(Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            WriteAngleToTrackbar(Robot.JointAngle);
            WriteAngleToTextBox(Robot.JointAngle);
            txtbRotX.Text = ((double)tbRX.Value / 10.0).ToString();

        }

        private void tbRY_Scroll(object sender, EventArgs e)
        {
            try
            {
                Robot.UpdateByIK(ReadEndPosFromTextBox(), ReadEndAngleFromTrackBar());
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            WriteAngleToTrackbar(Robot.JointAngle);
            WriteAngleToTextBox(Robot.JointAngle);
            txtbRotY.Text = ((double)tbRY.Value / 10.0).ToString();
        }

        private void tbRZ_Scroll(object sender, EventArgs e)
        {
            try
            {
                Robot.UpdateByIK(ReadEndPosFromTextBox(), ReadEndAngleFromTrackBar());
            }
            catch (Exception)
            {
                MessageBox.Show("動作範囲外です", "エラー", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Bitmap bitmap = new Bitmap(pb3DView.Width, pb3DView.Height);
            Idraw.DrawOriginalAxis(bitmap);
            DrawRobot(bitmap, Robot.LinkPos);
            UpdateAllUIFromState(Robot);
            WriteAngleToTrackbar(Robot.JointAngle);
            WriteAngleToTextBox(Robot.JointAngle);
            txtbRotZ.Text = ((double)tbRZ.Value / 10.0).ToString();
            //WriteEndPosToTextBox(Robot.EndPos);
        }
    }
}
