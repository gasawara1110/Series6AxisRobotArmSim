using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Series6AxisRobotArmKinematicsSimulator
{
    public class Kinematics
    {
        public struct Position
        {
            public double x;
            public double y;
            public double z;
            
            /*コンストラクタ
             * 
             * 
             * 
             */ 
            public Position(double x,double y,double z)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }
        }
        public enum AXIS { X, Y, Z };

        private double[] L;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="linkLength">各リンク+手先の長さ配列[mm]、要素数7</param>
        public Kinematics(double[] linkLength)
        {
            if (linkLength.Length != 7)
            {
                throw new Exception();
            }

            this.L = linkLength;
        }

        
        
        /// <summary>
        /// 順運動学を解く
        /// </summary>
        /// <param name="angle"> 各関節の角度[rad]、根本から順番に、要素数6</param>
        /// <returns>各リンク+手先の位置構造体配列、要素数7</returns>
        public Position[] SolveFK(double[] angle)
        {
            Position[] linkPos = new Position[7];           //関節位置格納用共用体
            Matrix<double>[] T = new Matrix<double>[7];     //次の関節への同時変換行列
            //対角要素1の対角行列で初期化
            for (int i = 0;i<T.Length; i++)
            {
                //T[i] = DenseMatrix.Build.DiagonalIdentity(4, 4);
                T[i] = DenseMatrix.Build.DenseIdentity(4);
            }

            //リンク構造から同時変換行列を導出
            //原点->1軸目
            T[0].SetSubMatrix(0, 0, GenerateRotMat(angle[0], AXIS.Z));
            T[0][2, 3] = this.L[0];

            //1軸目->２軸目
            T[1].SetSubMatrix(0, 0, GenerateRotMat(-angle[1], AXIS.Y));
            //T[1].SetSubMatrix(0, 0, GenerateRotMat(angle[0], AXIS.Z));
            //GenerateRotMat(angle[0], AXIS.Z).CopyTo(T[1]);
            T[1][2, 3] = this.L[1];

            //２軸目->3軸目
            T[2].SetSubMatrix(0, 0, GenerateRotMat(-angle[2], AXIS.Y));
            //T[2].SetSubMatrix(0, 0, GenerateRotMat(-angle[1], AXIS.Y));
            //GenerateRotMat(-angle[1], AXIS.Y).CopyTo(T[2]);
            T[2][0, 3] = this.L[2];

            //3軸目->4軸目
            T[3].SetSubMatrix(0, 0, GenerateRotMat(angle[3], AXIS.X));
            //T[3].SetSubMatrix(0, 0, GenerateRotMat(-angle[2], AXIS.Y));
            //GenerateRotMat(-angle[2], AXIS.Y).CopyTo(T[3]);
            T[3][0, 3] = this.L[3];

            //4軸目->5軸目
            T[4].SetSubMatrix(0, 0, GenerateRotMat(-angle[4], AXIS.Y));
            //T[4].SetSubMatrix(0, 0, GenerateRotMat(angle[3], AXIS.X));
            //GenerateRotMat(angle[3], AXIS.X).CopyTo(T[4]);
            T[4][0, 3] = this.L[4];

            //5軸目->6軸目
            T[5].SetSubMatrix(0, 0, GenerateRotMat(angle[5], AXIS.Z));
            //T[5].SetSubMatrix(0, 0, GenerateRotMat(-angle[4], AXIS.Y));
            //GenerateRotMat(-angle[4], AXIS.Y).CopyTo(T[5]);
            T[5][2, 3] = -this.L[5];

            //6軸目->手先
            //T[6].SetSubMatrix(0, 0, GenerateRotMat(angle[5], AXIS.Z));
            //GenerateRotMat(angle[5], AXIS.Z).CopyTo(T[6]);
            T[6][2, 3] = -this.L[6];

            Matrix<double> temp = DenseMatrix.Build.DenseIdentity(4, 4);

            for(int i = 0; i < T.Length; i++)
            {
                temp = temp * T[i];
                linkPos[i].x = temp[0, 3];
                linkPos[i].y = temp[1, 3];
                linkPos[i].z = temp[2, 3];
            }
            

            return linkPos;
        }

        
        /// <summary>
        /// 順運動学を解く(オーバーロード1)
        /// </summary>
        /// <param name="angle">各関節の角度[rad]、根本から順番に、要素数6</param>
        /// <param name="rotMat">手先の3x3姿勢行列</param>
        /// <returns>各関節と手先の位置配列、根本から順番に手先まで、要素数7</returns>
        public Position[] SolveFK(double[] angle, out double[,] rotMat)
        {
            Position[] linkPos = new Position[7];           //関節位置格納用共用体
            Matrix<double>[] T = new Matrix<double>[7];     //次の関節への同時変換行列
            //対角要素1の対角行列で初期化
            for (int i = 0; i < T.Length; i++)
            {
                //T[i] = DenseMatrix.Build.DiagonalIdentity(4, 4);
                T[i] = DenseMatrix.Build.DenseIdentity(4);
            }

            //リンク構造から同時変換行列を導出
            //原点->1軸目
            T[0].SetSubMatrix(0, 0, GenerateRotMat(angle[0], AXIS.Z));
            T[0][2, 3] = this.L[0];

            //1軸目->２軸目
            T[1].SetSubMatrix(0, 0, GenerateRotMat(-angle[1], AXIS.Y));
            //T[1].SetSubMatrix(0, 0, GenerateRotMat(angle[0], AXIS.Z));
            //GenerateRotMat(angle[0], AXIS.Z).CopyTo(T[1]);
            T[1][2, 3] = this.L[1];

            //２軸目->3軸目
            T[2].SetSubMatrix(0, 0, GenerateRotMat(-angle[2], AXIS.Y));
            //T[2].SetSubMatrix(0, 0, GenerateRotMat(-angle[1], AXIS.Y));
            //GenerateRotMat(-angle[1], AXIS.Y).CopyTo(T[2]);
            T[2][0, 3] = this.L[2];

            //3軸目->4軸目
            T[3].SetSubMatrix(0, 0, GenerateRotMat(angle[3], AXIS.X));
            //T[3].SetSubMatrix(0, 0, GenerateRotMat(-angle[2], AXIS.Y));
            //GenerateRotMat(-angle[2], AXIS.Y).CopyTo(T[3]);
            T[3][0, 3] = this.L[3];

            //4軸目->5軸目
            T[4].SetSubMatrix(0, 0, GenerateRotMat(-angle[4], AXIS.Y));
            //T[4].SetSubMatrix(0, 0, GenerateRotMat(angle[3], AXIS.X));
            //GenerateRotMat(angle[3], AXIS.X).CopyTo(T[4]);
            T[4][0, 3] = this.L[4];

            //5軸目->6軸目
            T[5].SetSubMatrix(0, 0, GenerateRotMat(angle[5], AXIS.Z));
            //T[5].SetSubMatrix(0, 0, GenerateRotMat(-angle[4], AXIS.Y));
            //GenerateRotMat(-angle[4], AXIS.Y).CopyTo(T[5]);
            T[5][2, 3] = -this.L[5];

            //6軸目->手先
            //T[6].SetSubMatrix(0, 0, GenerateRotMat(angle[5], AXIS.Z));
            //GenerateRotMat(angle[5], AXIS.Z).CopyTo(T[6]);
            T[6][2, 3] = -this.L[6];

            Matrix<double> temp = DenseMatrix.Build.DenseIdentity(4, 4);
            Matrix<double> rotM = DenseMatrix.Build.DenseIdentity(3, 3);

            for (int i = 0; i < T.Length; i++)
            {
                temp = temp * T[i];
                linkPos[i].x = temp[0, 3];
                linkPos[i].y = temp[1, 3];
                linkPos[i].z = temp[2, 3];
            }
            
            for(int i = 0; i < 3; i++)
            {
                for(int j = 0; j < 3; j++)
                {
                    rotM[i, j] = temp[i, j];
                }
            }

            rotMat = rotM.ToArray();
            return linkPos;
        }

        
        /// <summary>
        /// 逆運動学解を解く
        /// </summary>
        /// <param name="px">手先のx座標</param>
        /// <param name="py"></param>
        /// <param name="pz"></param>
        /// <param name="rotMat"></param>
        /// <returns></returns>
        public double[] SolveIK(double px, double py, double pz, double[,] rotMat)
        {
            double[] j5Pos = new double[3];
            j5Pos = CalcJ5Pos(px, py, pz, rotMat);

            double[,] theta123temp = new double[2, 3];
            theta123temp = CalcJointAngle123(j5Pos[0], j5Pos[1], j5Pos[2]);
            double[] theta456 = CalcJointAngle456(theta123temp[0, 0], theta123temp[0, 1], theta123temp[0, 2], rotMat);
            double[] ans = new double[6];

            ans[0] = theta123temp[0, 0];
            ans[1] = theta123temp[0, 1];
            ans[2] = theta123temp[0, 2];
            ans[3] = theta456[0];
            ans[4] = theta456[1];
            ans[5] = theta456[2];

            return ans;
        }

        
        /// <summary>
        /// 逆運動学を解く(オーバーロード1)
        /// </summary>
        /// <param name="endPos">手先位置構造体</param>
        /// <param name="rotMat">3x3姿勢行列</param>
        /// <returns></returns>
        public double[] SolveIK(Position endPos,double[,] rotMat)
        {
            return SolveIK(endPos.x, endPos.y, endPos.z, rotMat);
        }


        /// <summary>
        /// 位置、姿勢からJ5の位置を計算
        /// </summary>
        /// <param name="px">第5関節x座標[mm]</param>
        /// <param name="py">第5関節y座標[mm]</param>
        /// <param name="pz">第5関節z座標[mm]</param>
        /// <param name="rotMat">姿勢回転行列(3x3)</param>
        /// <returns>J5の位置行列,[x,y,z]の順</returns>
        private double[] CalcJ5Pos(double px, double py, double pz, double[,] rotMat)
        {
            Matrix<double> endPosMat = DenseMatrix.OfArray(new double[,] { { px }, { py }, { pz } });
            //Matrix<double> j5PosMat = DenseMatrix(3, 1);

            if ((rotMat.GetLength(0) != 3) || (rotMat.GetLength(1) != 3))
            {
                //回転行列が3x3行列ではない場合例外を投げる
                throw new ArgumentException();
            }
            Matrix<double> RMat = DenseMatrix.OfArray(rotMat);
            Matrix<double> J5ToHandMat = DenseMatrix.OfArray(new double[,] { { 0 }, { 0 }, { -L[5] - L[6] } });
            //J5->Handへの位置行列を算出
            J5ToHandMat = RMat * J5ToHandMat;
            //J5の位置を導出
            var j5PosMat = endPosMat - J5ToHandMat;
            return j5PosMat.ToRowMajorArray();
            //return j5PosMat.ToColumnMajorArray();
        }

       
        /// <summary>
        /// 第５関節の位置から1～3関節の角度を計算
        /// </summary>
        /// <param name="px5">第5関節x座標[mm]</param>
        /// <param name="py5">第5関節y座標[mm]</param>
        /// <param name="pz5">第5関節z座標[mm]</param>
        /// <returns>解行列(2x3) dimension1:解番号 dimension2:関節番号</returns>
        private double[,] CalcJointAngle123(double px5, double py5, double pz5)
        {
            //th1の解
            double th1 = Math.Atan2(py5, px5);
            //x^2+y^2
            double Lxy = Math.Sqrt(px5 * px5 + py5 * py5);
            //平行２リンクマニピュレータ問題を説いてth2、th3を導出
            double[,] ansTemp = new double[2, 2];
            try
            {
                ansTemp = CalcFlat2LinkIK(Lxy, pz5 - L[0] - L[1], L[2], L[3] + L[4]);
            }
            catch (ArgumentOutOfRangeException)
            {
                throw new ArgumentOutOfRangeException();
            }
            

            //2種の解を導出
            double[,] ans = new double[2, 3];
            //th1
            ans[0, 0] = ans[1, 0] = th1;
            //th2および3
            ans[0, 1] = ansTemp[0, 0];
            ans[0, 2] = ansTemp[0, 1];
            ans[1, 1] = ansTemp[1, 0];
            ans[1, 2] = ansTemp[1, 1];
            return ans;

        }

        
        /// <summary>
        /// 平面２リンク問題を解く(2リンクが初期角0[rad]のときpx = L1+L2,py = 0)
        /// </summary>
        /// <param name="px">手先x座標[mm]</param>
        /// <param name="py">手先y座標[mm]</param>
        /// <param name="L1">リンク1長さ[mm]</param>
        /// <param name="L2">リンク2長さ[mm]</param>
        /// <returns>解行列(2x2) dimension1:解番号,dimension2：関節番号 ex)[0,1]で１つ目の解の第２関節角度[rad]</returns>
        private double[,] CalcFlat2LinkIK(double px, double py, double L1, double L2)
        {
            double[,] theta = new double[2, 2];

            //中間変数
            double d1 = (px * px + py * py + L1 * L1 - L2 * L2) / (2 * L1);
            double d2 = (px * px + py * py - L1 * L1 + L2 * L2) / (2 * L2);
            if (Math.Sqrt(px * px + py * py) > (L1+L2))
            {
                //解が存在しない場合例外を投げる
                throw new ArgumentOutOfRangeException();
            }
            //解1の導出
            theta[0, 0] = Math.Atan2(py, px) + Math.Atan2(Math.Sqrt(px * px + py * py - d1 * d1), d1);
            theta[0, 1] = -Math.Atan2(Math.Sqrt(px * px + py * py - d1 * d1), d1) - Math.Atan2(Math.Sqrt(px * px + py * py - d2 * d2), d2);
            //解２の導出
            theta[1, 0] = Math.Atan2(py, px) - Math.Atan2(Math.Sqrt(px * px + py * py - d1 * d1), d1);
            theta[1, 1] = -Math.Atan2(Math.Sqrt(px * px + py * py - d1 * d1), d1) - Math.Atan2(Math.Sqrt(px * px + py * py - d2 * d2), d2);

            return theta;
        }

        /* 手先の姿勢とjoint1～3の角度からJ4～J6を計算
         * @param[in] angle1 関節1の角度[rad]
         * @param[in] angle2 関節2の角度[rad]
         * @param[in] angle3 関節3の角度[rad]
         * @param[in] rotMat 手先の姿勢変換行列(3x3)
         * @return 
         */
        private double[] CalcJointAngle456(double angle1, double angle2, double angle3, double[,] rotMat)
        {
            double[] angle = new double[3];
            double th4, th5, th6;
            Matrix<double> RMat = DenseMatrix.OfArray(rotMat);
            Matrix<double> R4to6Mat = RMat * GenerateRotMat(-angle1, AXIS.Z) * GenerateRotMat(angle2, AXIS.Y) * GenerateRotMat(angle3, AXIS.Y);
            th6 = Math.Atan2(R4to6Mat[1, 0], R4to6Mat[0, 0]);
            th4 = Math.Atan2(R4to6Mat[2, 1], R4to6Mat[2, 2]);
            th5 = Math.Atan2(R4to6Mat[2, 0], R4to6Mat[2, 2] / Math.Cos(th4));
            angle[0] = th4;
            angle[1] = th5;
            angle[2] = th6;
            return angle;
        }

        /* 回転行列を生成
         * @param[in] angle 回転角度[rad]
         * @param[in] axis どの軸周りに回すか
         * @return 回転行列
         */
        public static Matrix<double> GenerateRotMat(double angle, AXIS axis)
        {
            Matrix<double> rotMat = Matrix<double>.Build.Dense(3, 3, 0.0);  //3x3の零行列
            switch (axis)
            {
                case AXIS.X:
                    rotMat[0, 0] = 1;
                    rotMat[1, 1] = Math.Cos(angle);
                    rotMat[1, 2] = -Math.Sin(angle);
                    rotMat[2, 1] = Math.Sin(angle);
                    rotMat[2, 2] = Math.Cos(angle);
                    break;
                case AXIS.Y:
                    rotMat[0, 0] = Math.Cos(angle);
                    rotMat[0, 2] = Math.Sin(angle);
                    rotMat[1, 1] = 1;
                    rotMat[2, 0] = -Math.Sin(angle);
                    rotMat[2, 2] = Math.Cos(angle);
                    break;
                case AXIS.Z:
                    rotMat[0, 0] = Math.Cos(angle);
                    rotMat[0, 1] = -Math.Sin(angle);
                    rotMat[0, 2] = 0;
                    rotMat[1, 0] = Math.Sin(angle);
                    rotMat[1, 1] = Math.Cos(angle);
                    rotMat[2, 2] = 1;
                    break;
            }
            return rotMat;

        }

    }
}
