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
        // Point <=> Point3d
        public static Point3d Cast(this MathLib.MPoint point)
        {
            return new Point3d(point.X, point.Y, point.Z);
        }
        public static MathLib.MPoint Cast(this Point3d point_rh)
        {
            return new MathLib.MPoint(point_rh.X, point_rh.Y, point_rh.Z);
        }
        public static List<Point3d> Cast(this List<MathLib.MPoint> points)
        {
            var points_rh = new List<Point3d>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                points_rh[i] = points[i].Cast();
            }
            return points_rh;
        }
        public static List<MathLib.MPoint> Cast(this List<Point3d> points_rh)
        {
            var points = new List<MathLib.MPoint>(points_rh.Count);
            for (int i = 0; i < points_rh.Count; i++)
            {
                points[i] = points_rh[i].Cast();
            }
            return points;
        }
        public static Point3d[] Cast(this MathLib.MPoint[] points)
        {
            var points_rh = new Point3d[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                points_rh[i] = points[i].Cast();
            }
            return points_rh;
        }
        public static MathLib.MPoint[] Cast(this Point3d[] points_rh)
        {
            var points = new MathLib.MPoint[points_rh.Length];
            for (int i = 0; i < points_rh.Length; i++)
            {
                points[i] = points_rh[i].Cast();
            }
            return points;
        }

        // Vector <=> Vector3d
        public static Vector3d Cast(this MathLib.MVector vector)
        {
            return new Vector3d(vector.X, vector.Y, vector.Z);
        }
        public static MathLib.MVector Cast(this Vector3d vector_rh)
        {
            return new MathLib.MVector(vector_rh.X, vector_rh.Y, vector_rh.Z);
        }
        public static List<Vector3d> Cast(this List<MathLib.MVector> vectors)
        {
            var vectors_rh = new List<Vector3d>(vectors.Count);
            for (int i = 0; i < vectors.Count; i++)
            {
                vectors_rh[i] = vectors[i].Cast();
            }
            return vectors_rh;
        }
        public static List<MathLib.MVector> Cast(this List<Vector3d> vectors_rh)
        {
            var vectors = new List<MathLib.MVector>(vectors_rh.Count);
            for (int i = 0; i < vectors_rh.Count; i++)
            {
                vectors[i] = vectors_rh[i].Cast();
            }
            return vectors;
        }
        public static Vector3d[] Cast(this MathLib.MVector[] vectors)
        {
            var vectors_rh = new Vector3d[vectors.Length];
            for (int i = 0; i < vectors.Length; i++)
            {
                vectors_rh[i] = vectors[i].Cast();
            }
            return vectors_rh;
        }
        public static MathLib.MVector[] Cast(this Vector3d[] vectors_rh)
        {
            var vectors = new MathLib.MVector[vectors_rh.Length];
            for (int i = 0; i < vectors_rh.Length; i++)
            {
                vectors[i] = vectors_rh[i].Cast();
            }
            return vectors;
        }

        // Frame <=> Plane
        public static Plane Cast(this MathLib.MFrame frame)
        {
            return new Plane(frame.Origin.Cast(), frame.XAxis.Cast(), frame.YAxis.Cast());
        }
        public static MathLib.MFrame Cast(this Plane plane)
        {
            return new MathLib.MFrame(plane.Origin.Cast(), plane.XAxis.Cast(), plane.YAxis.Cast());
        }
        public static List<Plane> Cast(this List<MathLib.MFrame> frames)
        {
            var planes = new List<Plane>(frames.Count);
            for (int i = 0; i < frames.Count; i++)
            {
                planes[i] = new Plane(frames[i].Origin.Cast(), frames[i].XAxis.Cast(), frames[i].YAxis.Cast());
            }
            return planes;
        }
        public static List<MathLib.MFrame> Cast(this List<Plane> planes)
        {
            var frames = new List<MathLib.MFrame>(planes.Count);
            for (int i = 0; i < planes.Count; i++)
            {
                frames[i] = new MathLib.MFrame(planes[i].Origin.Cast(), planes[i].XAxis.Cast(), planes[i].YAxis.Cast());
            }
            return frames;
        }
        public static Plane[] Cast(this MathLib.MFrame[] frames)
        {
            var planes = new Plane[frames.Length];
            for (int i = 0; i < frames.Length; i++)
            {
                planes[i] = new Plane(frames[i].Origin.Cast(), frames[i].XAxis.Cast(), frames[i].YAxis.Cast());
            }
            return planes;
        }
        public static MathLib.MFrame[] Cast(this Plane[] planes)
        {
            var frames = new MathLib.MFrame[planes.Length];
            for (int i = 0; i < planes.Length; i++)
            {
                frames[i] = new MathLib.MFrame(planes[i].Origin.Cast(), planes[i].XAxis.Cast(), planes[i].YAxis.Cast());
            }
            return frames;
        }
    }
}
