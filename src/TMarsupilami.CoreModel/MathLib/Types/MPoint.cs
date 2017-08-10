using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public struct MPoint
    {
        private double x;
        private double y;
        private double z;

        public double X { get { return x; } set { x = value; } }
        public double Y { get { return y; } set { y = value; } }
        public double Z { get { return z; } set { z = value; } }

        public static double SquaredDistance(MPoint fromPoint, MPoint toPoint) {
            double x = toPoint.X - fromPoint.X;
            double y = toPoint.Y - fromPoint.Y;
            double z = toPoint.Z - fromPoint.Z;
            return x * x + y * y + z * z;
        }
        public static double Distance(MPoint fromPoint, MPoint toPoint)
        {
            double x = toPoint.X - fromPoint.X;
            double y = toPoint.Y - fromPoint.Y;
            double z = toPoint.Z - fromPoint.Z;
            return Math.Sqrt(x * x + y * y + z * z);
        }

        public void Move(MVector dispVector) {
            x += dispVector.X;
            y += dispVector.Y;
            z += dispVector.Z;
        }

    }
}