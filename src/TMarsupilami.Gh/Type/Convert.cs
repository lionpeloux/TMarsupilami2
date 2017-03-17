using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh
{
    /// <summary>
    /// Make explicit casts from Rhino types to MathLib types for the Grasshopper interface of
    /// Marsupilami
    /// </summary>
    public static class Convert
    {
        // Point <=> Point3d
        public static Point3d Cast(this MPoint point)
        {
            return new Point3d(point.X, point.Y, point.Z);
        }
        public static MPoint Cast(this Point3d point_rh)
        {
            return new MPoint(point_rh.X, point_rh.Y, point_rh.Z);
        }
        public static List<Point3d> Cast(this List<MPoint> points)
        {
            var points_rh = new List<Point3d>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                points_rh.Add(points[i].Cast());
            }
            return points_rh;
        }
        public static List<MPoint> Cast(this List<Point3d> points_rh)
        {
            var points = new List<MPoint>(points_rh.Count);
            for (int i = 0; i < points_rh.Count; i++)
            {
                points.Add(points_rh[i].Cast());
            }
            return points;
        }
        public static Point3d[] Cast(this MPoint[] points)
        {
            var points_rh = new Point3d[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                points_rh[i] = points[i].Cast();
            }
            return points_rh;
        }
        public static MPoint[] Cast(this Point3d[] points_rh)
        {
            var points = new MPoint[points_rh.Length];
            for (int i = 0; i < points_rh.Length; i++)
            {
                points[i] = points_rh[i].Cast();
            }
            return points;
        }

        // Vector <=> Vector3d
        public static Vector3d Cast(this MVector vector)
        {
            return new Vector3d(vector.X, vector.Y, vector.Z);
        }
        public static MVector Cast(this Vector3d vector_rh)
        {
            return new MVector(vector_rh.X, vector_rh.Y, vector_rh.Z);
        }
        public static List<Vector3d> Cast(this List<MVector> vectors)
        {
            var vectors_rh = new List<Vector3d>(vectors.Count);
            for (int i = 0; i < vectors.Count; i++)
            {
                vectors_rh.Add(vectors[i].Cast());
            }
            return vectors_rh;
        }
        public static List<MVector> Cast(this List<Vector3d> vectors_rh)
        {
            var vectors = new List<MVector>(vectors_rh.Count);
            for (int i = 0; i < vectors_rh.Count; i++)
            {
                vectors.Add(vectors_rh[i].Cast());
            }
            return vectors;
        }
        public static Vector3d[] Cast(this MVector[] vectors)
        {
            var vectors_rh = new Vector3d[vectors.Length];
            for (int i = 0; i < vectors.Length; i++)
            {
                vectors_rh[i] = vectors[i].Cast();
            }
            return vectors_rh;
        }
        public static MVector[] Cast(this Vector3d[] vectors_rh)
        {
            var vectors = new MVector[vectors_rh.Length];
            for (int i = 0; i < vectors_rh.Length; i++)
            {
                vectors[i] = vectors_rh[i].Cast();
            }
            return vectors;
        }

        // Frame <=> Plane
        public static Plane Cast(this MFrame frame)
        {
            return new Plane(frame.Origin.Cast(), frame.XAxis.Cast(), frame.YAxis.Cast());
        }
        public static MFrame Cast(this Plane plane)
        {
            return new MFrame(plane.Origin.Cast(), plane.XAxis.Cast(), plane.YAxis.Cast());
        }
        public static List<Plane> Cast(this List<MFrame> frames)
        {
            var planes = new List<Plane>(frames.Count);
            for (int i = 0; i < frames.Count; i++)
            {
                planes.Add(new Plane(frames[i].Origin.Cast(), frames[i].XAxis.Cast(), frames[i].YAxis.Cast()));
            }
            return planes;
        }
        public static List<MFrame> Cast(this List<Plane> planes)
        {
            var frames = new List<MFrame>(planes.Count);
            for (int i = 0; i < planes.Count; i++)
            {
                frames.Add(new MFrame(planes[i].Origin.Cast(), planes[i].XAxis.Cast(), planes[i].YAxis.Cast()));
            }
            return frames;
        }
        public static Plane[] Cast(this MFrame[] frames)
        {
            var planes = new Plane[frames.Length];
            for (int i = 0; i < frames.Length; i++)
            {
                planes[i] = new Plane(frames[i].Origin.Cast(), frames[i].XAxis.Cast(), frames[i].YAxis.Cast());
            }
            return planes;
        }
        public static MFrame[] Cast(this Plane[] planes)
        {
            var frames = new MFrame[planes.Length];
            for (int i = 0; i < planes.Length; i++)
            {
                frames[i] = new MFrame(planes[i].Origin.Cast(), planes[i].XAxis.Cast(), planes[i].YAxis.Cast());
            }
            return frames;
        }
    }
}
