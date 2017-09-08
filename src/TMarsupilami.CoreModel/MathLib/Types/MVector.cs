using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public struct MVector
    {
        private double x;
        private double y;
        private double z;

        public double X { get { return x; } set { x = value; } }
        public double Y { get { return y; } set { y = value; } }
        public double Z { get { return z; } set { z = value; } }

        public double GetLength() { return Math.Sqrt(x * x + y * y + z * z); }
        public double GetSquaredLength() { return x * x + y * y + z * z; }

    }
}