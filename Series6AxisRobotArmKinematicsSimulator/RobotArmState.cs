using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Series6AxisRobotArmKinematicsSimulator
{
    public class RobotArmState
    {
        //姿勢角表現選択用列挙体
        public enum EXPRESSION_METHOD
        {
            //EULER_XYZ,
            //EULER_XZY,
            //EULER_YXZ,
            //EULER_YZX,
            //EULER_ZXY,
            EULER_ZYX
        }

        private EXPRESSION_METHOD method;           //姿勢角表現方法保存用変数
        private double[] jointAngle;                     //関節の角度配列(要素数6)
        private Kinematics.Position[] linkPos;      //リンク+手先の位置(要素数7)
        private Kinematics.Position endPos;         //手先の位置
        private double[,] rotMat;                   //手先の姿勢行列
        private double[] coordinateAngle;           //姿勢角[rad]
        private Kinematics arm;

        /// <summary>
        /// 関節角度配列プロパティ
        /// </summary>
        public double[] JointAngle
        {
            get { return this.jointAngle; }
        }

        /// <summary>
        /// リンク位置プロパティ
        /// </summary>
        public Kinematics.Position[] LinkPos
        {
            get { return this.linkPos; }
        }

        /// <summary>
        /// 手先位置プロパティ
        /// </summary>
        public Kinematics.Position EndPos
        {
            get { return this.endPos; }
        }

        /// <summary>
        /// 手先3x3姿勢行列プロパティ
        /// </summary>
        public double[,] RotMat
        {
            get { return this.rotMat; }
        }

        /// <summary>
        /// 姿勢角プロパティ
        /// </summary>
        public double[] CoordinateAngle
        {
            get { return this.coordinateAngle; }
        }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="linkLength">リンク長配列、要素数7</param>
        /// <param name="method">姿勢角表現方法列挙体</param>
        public RobotArmState(double[] linkLength,EXPRESSION_METHOD method)
        {
            if(linkLength.Length != 7)
            {
                throw new ArgumentException();
            }
            this.method = method;                           //姿勢角表現方法の保存
            arm = new Kinematics(linkLength);
            this.jointAngle = new double[] { 0,0,0,0,0,0};      //関節角度初期化
            this.rotMat = new double[3, 3];
            this.linkPos = new Kinematics.Position[7];
            this.linkPos = arm.SolveFK(this.jointAngle,out rotMat);          //リンク位置および手先の姿勢行列計算
            this.endPos = this.linkPos[6];                                   //手先の位置
            this.coordinateAngle = new double[] { 0, 0, 0 };
            this.coordinateAngle = ConvertRotMatToEuler(rotMat, method);          //手先姿勢計算
        }
        
        /// <summary>
        /// 関節角度から順運動学を解き、各種状態変数を更新
        /// </summary>
        /// <param name="jointAngle">関節角度配列[rad]、要素数6</param>
        public void UpdateByFK(double[] jointAngle)
        {
            if(jointAngle.Length != 6)
            {
                throw new ArgumentException();
            }
            this.jointAngle = jointAngle;
            this.linkPos = arm.SolveFK(this.jointAngle,out this.rotMat);
            this.endPos = this.linkPos[6];
            this.coordinateAngle = ConvertRotMatToEuler(this.rotMat, this.method);
        }

        /// <summary>
        /// 入力された手先位置構造体、3x3姿勢行列から逆運動学を解き各種状態変数を更新
        /// </summary>
        /// <param name="endPos">手先位置構造体</param>
        /// <param name="rotMat">3x3姿勢行列</param>
        public void UpdateByIK(Kinematics.Position endPos,double[,] rotMat)
        {
            if(rotMat.GetLength(0) != 3 || rotMat.GetLength(1) != 3)
            {
                throw new ArgumentException();
            }
            
            //try
            //{
                this.jointAngle = arm.SolveIK(endPos, rotMat);
                this.endPos = endPos;
                this.rotMat = rotMat;
            //}
            //catch(Exception)
            //{
            //    throw new ArgumentOutOfRangeException();
            //}
            
            this.linkPos = arm.SolveFK(this.jointAngle);
            this.endPos = this.linkPos[6];
            this.coordinateAngle = ConvertRotMatToEuler(this.rotMat, this.method);
        }
        /// <summary>
        /// 入力された手先位置構造体、3x3姿勢行列から逆運動学を解き各種状態変数を更新
        /// </summary>
        /// <param name="endPos">手先位置構造体</param>
        /// <param name="coordinateAngle">オイラー角(x,y,zの順)</param>
        public void UpdateByIK(Kinematics.Position endPos,double[] coordinateAngle)
        {
            if(coordinateAngle.Length != 3)
            {
                throw new ArgumentException();
            }
            double[,] rotMat = new double[3, 3];
            rotMat = ConvertEulerToRotMat(coordinateAngle, this.method);
            UpdateByIK(endPos, rotMat);
        }

        /// <summary>
        /// 3x3姿勢行列を指定された表現のオイラー角に変換
        /// </summary>
        /// <param name="rotMat">3x3姿勢行列</param>
        /// <param name="method">姿勢角表現方法列挙体</param>
        /// <returns>オイラー角配列[rad](x,y,zの順)</returns>
        private double[] ConvertRotMatToEuler(double[,] rotMat,EXPRESSION_METHOD method)
        {
            if(rotMat.GetLength(0) != 3 || rotMat.GetLength(1) != 3)
            {
                throw new ArgumentException();
            }
            double[] rotAngle = new double[3];
            switch (method)
            {
                case EXPRESSION_METHOD.EULER_ZYX:
                    rotAngle[0] = -Math.Atan2(rotMat[1,2],rotMat[2,2]);
                    rotAngle[1] = -Math.Atan2(-rotMat[0,2],Math.Sqrt(rotMat[1,2]*rotMat[1,2] + rotMat[2,2]*rotMat[2,2]));
                    rotAngle[2] = -Math.Atan2(rotMat[0, 1], rotMat[0, 0]);
                    break;
            }
            return rotAngle;
        }

        /// <summary>
        /// 指定された表現のオイラー角を3x3姿勢行列に変換
        /// </summary>
        /// <param name="rotAngle"></param>
        /// <param name="method"></param>
        /// <returns></returns>
        private double[,] ConvertEulerToRotMat(double[] rotAngle, EXPRESSION_METHOD method)
        {
            if(rotAngle.Length != 3)
            {
                throw new ArgumentException();
            }
            double[,] rotMat = new double[3, 3];
            switch(method)
            {
                case EXPRESSION_METHOD.EULER_ZYX:
                    rotMat = (Kinematics.GenerateRotMat(rotAngle[0], Kinematics.AXIS.X) *
                             Kinematics.GenerateRotMat(rotAngle[1], Kinematics.AXIS.Y) *
                             Kinematics.GenerateRotMat(rotAngle[2], Kinematics.AXIS.Z)).ToArray();
                    break;
            }
            return rotMat;
        }

    }
}
