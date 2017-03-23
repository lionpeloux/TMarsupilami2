using Rhino.Geometry;
using System;

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    public static class ArrayExtension
    {
        public static void Populate<T>(this T[] array, T value)
        {
            for (int i = 0; i < array.Length; i++)
            {
                array[i] = value;
            }
        }
        /// <summary>
        /// Return an array populated with the same element.
        /// Warning : If T is a reference type, all the array elements will point to the same value object.
        /// </summary>
        /// <typeparam name="T">The type of array's elements.</typeparam>
        /// <param name="value">The value to populate the array with.</param>
        /// <param name="n">The array length.</param>
        /// <returns></returns>
        public static T[] ToArray<T>(this T value, int n)
        {
            T[] array = new T[n];
            for (int i = 0; i < n; i++)
            {
                array[i] = value;
            }
            return array;
        }
    }

    public static class PlaneExtension
    {
        /// <summary>
        /// Parallel Transport a plane from its Origin and ZAxis to a given point and direction.
        /// Uses standard Rhino transform matrix.
        /// </summary>
        /// <param name="plane">The plane to be parallel transported.</param>
        /// <param name="toPoint">The targeted Origin.</param>
        /// <param name="toZAxis">The targeted ZAxis.</param>
        /// <returns>The transported plane as a new plane.</returns>
        public static Plane ParallelTransport(this Plane plane, Point3d toPoint, Vector3d toZAxis)
        {
            // this leads to allocate a new plane, which might be not efficient.
            // this migth be faster with rodriguez formula => to implement
            // this migth be even faster with a polynomial approximaiton of sin and cos for small angles

            // Align ZAxis to vector
            Transform xform = Transform.Rotation(plane.ZAxis, toZAxis, toPoint);
            plane.Transform(xform);

            // Move frame to point
            plane.Origin = toPoint;
            return plane;
        }

        /// <summary>
        /// Parallel Transport a plane from its ZAxis to a given direction.
        /// This only realigned the ZAxiswith the given direction unsing the minimal rotation.
        /// Uses standard Rhino transform matrix.
        /// </summary>
        /// <param name="plane">The plane to be parallel transported.</param>
        /// <param name="toZAxis">The targeted ZAxis.</param>
        /// <returns>The transported plane as a new plane with the same origin and a ZAxis aligned with the given direction</returns>
        public static Plane ParallelTransport(this Plane plane, Vector3d toZAxis)
        {
            // this leads to allocate a new plane, which might be not efficient.
            // this migth be faster with rodriguez formula => to implement
            // this migth be even faster with a polynomial approximaiton of sin and cos for small angles

            // Align ZAxis to vector
            Transform xform = Transform.Rotation(plane.ZAxis, toZAxis, plane.Origin);
            plane.Transform(xform);

            return plane;
        }

        /// <summary>
        /// Twist/Rotate a plane around its ZAxis by an oriented angle dθ.
        /// Uses standard Rhino transform matrix.
        /// </summary>
        /// <param name="plane">The plane to be twisted/rotated.</param>
        /// <param name="dθ">The algebric amout of twist ]-π,π].</param>
        /// Could be positive (counterclockwise rotation) or negative (clockwise rotation).</param>
        /// <returns>The twisted/rotated plane as a new plane.</returns>
        public static Plane Twist(this Plane plane, double dθ)
        {
            // this leads to allocate a new plane, which might be not efficient.
            // this migth be faster with rodriguez formula => to implement
            // this migth be even faster with a polynomial approximaiton of sin and cos for small angles

            Transform xform = Transform.Rotation(dθ, plane.ZAxis, plane.Origin);
            plane.Transform(xform);
            return plane;
        }

        /// <summary>
        /// Gets the twist angle between two planes along their ZAxis.
        /// Parallel tranport the fromPlane to the toPlane Origin and ZAxis.
        /// Rotate it by the said twist angle around its ZAxis and you will get exactlly the toPlane.
        /// </summary>
        /// <param name="fromPlane">First plane.</param>
        /// <param name="toPlane">Second plane.</param>
        /// <returns>Oriented twist angle between the planes in ]-π,π].</returns>
        public static double GetTwistAngle(this Plane fromPlane, Plane toPlane)
        {
            Plane plane_pt = fromPlane.ParallelTransport(toPlane.Origin, toPlane.ZAxis);
            double theta = Vector3d.VectorAngle(plane_pt.XAxis, toPlane.XAxis, toPlane);

            if (theta > Math.PI) // ensure theta is in ]-π,π]
            {
                theta -= 2 * Math.PI;
            }

            return theta;
        }

        /// <summary>
        /// Get the interpolated plane between plane1 and plane2. 
        /// The interpolated plane is equidistant to plane1 and plane2. 
        /// The circle going through the 3 planes has the mean curvature : κb = (κb1 + κb2)/2.
        /// The twist angle of the interpolated plane is half the total twist between plane1 and plane2.
        /// </summary>
        /// <param name="plane1">First plane.</param>
        /// <param name="κb1">Curvature binormal vector at first plane.</param>
        /// <param name="plane2">Second plane.</param>
        /// <param name="κb2">Curvature binormal vector at second plane.</param>
        /// <returns>Interpolated plane.</returns>
        public static Plane Interpolate(Plane plane1, Vector3d κb1, Plane plane2, Vector3d κb2)
        {
            // Interpolation with mean curvature
            Vector3d κb = 0.5 * (κb1 + κb2);
            Vector3d e = new Vector3d(plane2.Origin - plane1.Origin);
            double l = e.Length;
            double r = 1 / κb.Length;

            // Interpolation point
            double h = r * (1 - Math.Sqrt(1 - 0.25 * l * l / (r * r)));
            Vector3d u = e / l;
            Vector3d n = r * Vector3d.CrossProduct(κb, u);
            Point3d origin = plane1.Origin + 0.5 * e - h * n;

            // Interpolation frame (twist is half the twist between plane1 and plane2)
            double θ = plane1.GetTwistAngle(plane2);
            Plane plane = plane1.ParallelTransport(origin, u);
            plane = plane.Twist(θ / 2);

            return plane;
        }

        /// <summary>
        /// Get the interpolated point between plane1 and plane2 origins. 
        /// The interpolated point is equidistant to plane1 and plane2 origins. 
        /// The circle going through the points has the mean curvature : κb = (κb1 + κb2)/2.
        /// </summary>
        /// <param name="plane1">First plane.</param>
        /// <param name="κb1">Curvature binormal vector at first plane.</param>
        /// <param name="plane2">Second plane.</param>
        /// <param name="κb2">Curvature binormal vector at second plane.</param>
        /// <returns>Interpolated plane.</returns>
        public static Point3d InterpolatePosition(Plane plane1, Vector3d κb1, Plane plane2, Vector3d κb2)
        {
            // Interpolation with mean curvature
            Vector3d κb = 0.5 * (κb1 + κb2);
            Vector3d e = new Vector3d(plane2.Origin - plane1.Origin);
            double l = e.Length;
            double r = 1 / κb.Length;

            // Interpolation point
            double h = r * (1 - Math.Sqrt(1 - 0.25 * l * l / (r * r)));
            Vector3d u = e / l;
            Vector3d n = r * Vector3d.CrossProduct(κb, u);
            Point3d origin = plane1.Origin + 0.5 * e - h * n;

            return origin;
        }

        /// <summary>
        /// Get the interpolated point between plane1 and plane2 origins. 
        /// The interpolated point is equidistant to plane1 and plane2 origins. 
        /// The circle going through the points has the mean curvature : κb = (κb1 + κb2)/2.
        /// </summary>
        /// <param name="plane1">First plane.</param>
        /// <param name="plane2">Second plane.</param>
        /// <param name="point">Interpolated point.</param>
        /// <param name="t">Tangent at interpolated point.</param>
        /// <returns>Interpolated plane.</returns>
        public static Plane InterpolateTwist(Plane plane1, Plane plane2, Point3d point, Vector3d t)
        {
            // Interpolation frame (twist is half the twist between plane1 and plane2)
            double θ = plane1.GetTwistAngle(plane2);
            Plane plane = plane1.ParallelTransport(point, t);
            plane = plane.Twist(θ / 2);

            return plane;
        }

        public static Plane[] GetValidCenterline(Plane[] ph, out Point3d[] x, out Vector3d[] e, out double[] l, out Vector3d[] κb, out Vector3d[] t)
        {
            // "h" denotes handle points
            // "g" denotes ghost points (interpolated)

            int Nn = ph.Length;
            int Ne = Nn - 1;


            // 1. compute xh
            // =============

            var xh = new Point3d[Nn];

            for (int i = 0; i < Nn; i++)
            {
                xh[i] = ph[i].Origin;
            }


            // 2. compute κbh
            // ==============

            var κbh = new Vector3d[Nn];
            var eh = new Vector3d[Ne];
            var lh = new double[Ne];
            var llh = new double[Ne];

            // i = 0
            eh[0] = new Vector3d(xh[1] - xh[0]);
            lh[0] = eh[0].Length;
            llh[0] = xh[0].DistanceTo(xh[2]);
            κbh[0] = -2 / (lh[0] * lh[0]) * Vector3d.CrossProduct(eh[0], ph[0].ZAxis);

            // i = 1, ..., Ne-2
            for (int i = 1; i < Ne - 1; i++)
            {
                eh[i] = new Vector3d(xh[i + 1] - xh[i]);
                lh[i] = eh[i].Length;
                llh[i] = xh[i].DistanceTo(xh[i + 2]);
                κbh[i] = 2 * Vector3d.CrossProduct(eh[i - 1], eh[i]) / (lh[i - 1] * lh[i] * llh[i - 1]);
            }

            // i = Ne-1
            eh[Ne - 1] = new Vector3d(xh[Ne] - xh[Ne - 1]);
            lh[Ne - 1] = eh[Ne - 1].Length;
            κbh[Ne - 1] = 2 * Vector3d.CrossProduct(eh[Ne - 2], eh[Ne - 1]) / (lh[Ne - 2] * lh[Ne - 1] * llh[Ne - 2]);


            // i = Ne
            κbh[Nn - 1] = 2 / (lh[Ne - 1] * lh[Ne - 1]) * Vector3d.CrossProduct(eh[Ne - 1], ph[Nn - 1].ZAxis);


            // 3. compute xg and x
            // ===================

            x = new Point3d[Nn + Ne];
            var xg = new Point3d[Ne];

            x[0] = xh[0];

            for (int i = 0; i < Ne; i++)
            {
                var pt = InterpolatePosition(ph[i], κbh[i], ph[i + 1], κbh[i + 1]);
                xg[i] = pt;
                x[2 * i + 1] = pt;
                x[2 * i + 2] = xh[i + 1];
            }


            // 4. compute t and κb
            // ===================

            e = new Vector3d[2 * Ne];
            l = new double[2 * Ne];
            κb = new Vector3d[Nn + Ne];

            var ll = new double[2 * Ne - 1];
            t = new Vector3d[Nn + Ne];

            // i = 0
            e[0] = new Vector3d(x[1] - x[0]);
            l[0] = e[0].Length;
            ll[0] = x[0].DistanceTo(x[2]);
            t[0] = ph[0].ZAxis;
            κb[0] = -2 / (l[0] * l[0]) * Vector3d.CrossProduct(e[0], t[0]);


            // i = 1, ..., Ne-2
            for (int i = 1; i < 2 * Ne - 1; i++)
            {
                e[i] = new Vector3d(x[i + 1] - x[i]);
                l[i] = e[i].Length;
                ll[i] = x[i].DistanceTo(x[i + 2]);
                t[i] = l[i] / (l[i - 1] * ll[i - 1]) * e[i - 1] + l[i - 1] / (l[i] * ll[i - 1]) * e[i];
                κb[i] = 2 * Vector3d.CrossProduct(e[i - 1], e[i]) / (l[i - 1] * l[i] * ll[i - 1]);
            }

            // i = 2*Ne - 1
            e[2 * Ne - 1] = new Vector3d(x[Nn + Ne - 1] - x[Nn + Ne - 2]);
            l[2 * Ne - 1] = e[2 * Ne - 1].Length;
            t[2 * Ne - 1] = l[2 * Ne - 1] / (l[2 * Ne - 2] * ll[2 * Ne - 2]) * e[2 * Ne - 2] + l[2 * Ne - 2] / (l[2 * Ne - 1] * ll[2 * Ne - 2]) * e[2 * Ne - 1];
            κb[2 * Ne - 1] = 2 * Vector3d.CrossProduct(e[2 * Ne - 2], e[2 * Ne - 1]) / (l[2 * Ne - 2] * l[2 * Ne - 1] * ll[2 * Ne - 2]);

            // i = 2*Ne
            t[Nn + Ne - 1] = ph[Nn - 1].ZAxis;
            κb[Nn + Ne - 1] = 2 / (l[2 * Ne - 1] * l[2 * Ne - 1]) * Vector3d.CrossProduct(e[2 * Ne - 1], t[Nn + Ne - 1]);


            // 5. realigned ph to new t
            // ========================

            var p = new Plane[Nn + Ne];

            p[0] = ph[0];

            for (int i = 1; i < Nn - 1; i++)
            {
                p[2 * i] = ph[i].ParallelTransport(t[2 * i]);
            }

            p[Ne + Nn - 1] = ph[Nn - 1];

            // 6. compute pg
            // =============

            var pg = new Plane[Ne];

            for (int i = 0; i < Ne; i++)
            {
                Plane plane = InterpolateTwist(ph[i], ph[i + 1], xg[i], t[2 * i + 1]);
                pg[i] = plane;
                p[2 * i + 1] = plane;
            }

            return p;
        }
    }

    public static class PointExtension
    {
        /// <summary>
        /// Return all useful properties for a circle going through 3 non-aligned points.
        /// </summary>
        /// <param name="p1">Start point.</param>
        /// <param name="p">Point on arc.</param>
        /// <param name="p2">End point.</param>
        /// <returns>An array of Vector3d [κb, t1, t, t2, n1, n, n2] where ti is the unit tangent and ni the unit normal at point pi.</returns>
        public static Vector3d[] Circle3Pts(Point3d p1, Point3d p, Point3d p2)
        {
            var e = new Vector3d(p2 - p1);
            var e1 = new Vector3d(p - p1);
            var e2 = new Vector3d(p2 - p);

            var l = e.Length;
            var l1 = e1.Length;
            var l2 = e2.Length;

            var u = e / l;
            var u1 = e1 / l1;
            var u2 = e2 / l2;

            var κb = (2 / l) * Vector3d.CrossProduct(u1, u2);
            var κ = κb.Length;
            var b = κb / κ;

            var t = l2 / l * u1 + l1 / l * u2;
            var t1 = (2 * (t * u1)) * u1 - t;
            var t2 = (2 * (t * u2)) * u2 - t;

            var n = Vector3d.CrossProduct(b, t);
            var n1 = Vector3d.CrossProduct(b, t1);
            var n2 = Vector3d.CrossProduct(b, t2);

            return new Vector3d[7] { κb, t1, t, t2, n1, n, n2 };
        }

        public static void Circle3Pts(Point3d p1, Point3d p, Point3d p2,
            ref Vector3d κb,
            ref Vector3d e1, ref Vector3d e2,
            ref double l1, ref double l2,
            ref Vector3d u1, ref Vector3d u2,
            ref Vector3d t1, ref Vector3d t, ref Vector3d t2)
        {
            var e = new Vector3d(p2 - p1);
            e1 = new Vector3d(p - p1);
            e2 = new Vector3d(p2 - p);

            var l = e.Length;
            l1 = e1.Length;
            l2 = e2.Length;

            var u = e / l;
            u1 = e1 / l1;
            u2 = e2 / l2;

            κb = (2 / l) * Vector3d.CrossProduct(u1, u2);

            t = l2 / l * u1 + l1 / l * u2;
            t1 = (2 * (t * u1)) * u1 - t;
            t2 = (2 * (t * u2)) * u2 - t;
        }

    }



}
