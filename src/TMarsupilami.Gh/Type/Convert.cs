using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.Gh
{
    /// <summary>
    /// Make explicit casts from Rhino types to MathLib types for the Grasshopper interface of
    /// Marsupilami
    /// </summary>
    public static class Convert
    {
        public static Point3d Cast(this MathLib.Point p)
        {
            return new Point3d(p.X, p.Y, p.Z);
        }
        public static MathLib.Point Cast(this Point3d p)
        {
            return new MathLib.Point(p.X, p.Y, p.Z);
        }
        public static Vector3d Cast(this MathLib.Vector v)
        {
            return new Vector3d(v.X, v.Y, v.Z);
        }
        public static MathLib.Vector Cast(this Vector3d v)
        {
            return new MathLib.Vector(v.X, v.Y, v.Z);
        }
    }
}
