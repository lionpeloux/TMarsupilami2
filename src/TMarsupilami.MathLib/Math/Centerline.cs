using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class Centerline
    {
        // Force frames ZAlignment

        /// <summary>
        /// In place align frame's ZAxis with given direction.
        /// </summary>
        /// <param name="frames">The frames to align.</param>
        /// <param name="t">Directions to align the frames to. Must be of unit length.</param>
        public static void ZAlignFrames(MFrame[] frames, MVector[] t)
        {
            int nv = frames.Length;

            // i = 0, ..., nv-1
            for (int i = 0; i < nv; i++)
            {
                // forces frames[i] to be parallel to the centerline tangent vector (t)
                ParallelTransportation.ZPT_Rotation(frames[i], frames[i].ZAxis, frames[i].Origin, t[i], ref frames[i]);
            }
        }

        // CURVATURE FOR CONIINUOUS CENTERLINE

        /// <summary>
        /// Gets the curvature and geometric invariants of a standard centerline (no ghost vertices).
        /// </summary>
        /// /// <remarks>
        /// If it is closed, Nv = Ne, Nvh = Nv, Nvg = 0.
        /// If it is open, Nv = Ne + 1, Nvh = Nv, Nvg = 0.
        /// </remarks>
        /// <param name="frames">The centerline as a set of frames.</param>
        /// <param name="x">The centerline vertices.</param>
        /// <param name="e">The centerline edge vectors.</param>
        /// <param name="u">The centerline unit edge vectors.</param>
        /// <param name="l">The centerline edge lengths.</param>
        /// <param name="t">The centerline unit tangent vectors.</param>
        /// <param name="κb">The centerline curvature binormal vector.</param>
        /// <param name="τ">The centerline rate of twist.</param>
        /// <param name="isClosed">True if the centerline is closed. False otherwise.</param>
        public static void GetCurvature(
            MFrame[] frames, MPoint[] x, 
            MVector[] e, MVector[] u, double[] l, 
            MVector[] t, MVector[] κb, 
            bool isClosed)
        {
            if (isClosed)
            {
                GetCurvature_Closed(frames, x, e, u, l, t, κb);
            }
            else
            {
                GetCurvature_Open(frames, x, e, u, l, t, κb);
            }
        }

        private static void GetCurvature_Open(
            MFrame[] frames, MPoint[] x, 
            MVector[] e, MVector[] u, double[] l, 
            MVector[] t, MVector[] κb)
        {
            int nv = frames.Length;
            int ne = nv - 1; // open centerline

            // i = 0
            x[0] = frames[0].Origin;
            e[0] = new MVector(frames[0].Origin, frames[1].Origin);
            l[0] = e[0].Length();
            u[0] = (1 / l[0]) * e[0];
            t[0] = frames[0].ZAxis;
            κb[0] = (-2 / l[0]) * MVector.CrossProduct(u[0], t[0]);

            // i = 1, ..., nv-2
            for (int i = 1; i < nv - 1; i++)
            {
                x[i] = frames[i].Origin;
                e[i] = new MVector(frames[i].Origin, frames[i + 1].Origin);
                l[i] = e[i].Length();
                u[i] = (1 / l[i]) * e[i];

                double _ll = 1 / MPoint.DistanceTo(frames[i - 1].Origin, frames[i + 1].Origin);
                t[i] = _ll * (l[i] * u[i - 1] + l[i - 1] * u[i]);
                κb[i] = (2 * _ll) * MVector.CrossProduct(u[i - 1], u[i]);
            }

            // i = nv-1
            x[nv - 1] = frames[nv - 1].Origin;
            e[ne - 1] = new MVector(frames[nv - 2].Origin, frames[nv - 1].Origin);
            l[ne - 1] = e[ne - 1].Length();
            u[ne - 1] = (1 / l[ne - 1]) * e[ne - 1];
            t[nv - 1] = frames[nv - 1].ZAxis;
            κb[nv - 1] = (2 / l[ne - 1]) * MVector.CrossProduct(u[ne - 1], t[nv - 1]);
        }

        private static void GetCurvature_Closed
            (MFrame[] frames, MPoint[] x, 
            MVector[] e, MVector[] u, double[] l, 
            MVector[] t, MVector[] κb)
        {
            int nv = frames.Length;
            int ne = nv; // closed centerline

            double _ll;

            // i = 0
            x[0] = frames[0].Origin;
            e[0] = new MVector(frames[0].Origin, frames[1].Origin);
            l[0] = e[0].Length();
            u[0] = (1 / l[0]) * e[0];

            // i = 1, ..., nv-2
            for (int i = 1; i < nv-1; i++)
            {
                x[i] = frames[i].Origin;
                e[i] = new MVector(frames[i].Origin, frames[i + 1].Origin);
                l[i] = e[i].Length();
                u[i] = (1 / l[i]) * e[i];

                _ll = 1 / MPoint.DistanceTo(frames[i - 1].Origin, frames[i + 1].Origin);
                t[i] = _ll * (l[i] * u[i - 1] + l[i - 1] * u[i]);
                κb[i] = (2 * _ll) * MVector.CrossProduct(u[i - 1], u[i]);
            }

            // i = nv-1
            x[nv - 1] = frames[nv - 1].Origin;
            e[ne - 1] = new MVector(frames[nv - 1].Origin, frames[0].Origin);
            l[ne - 1] = e[ne - 1].Length();
            u[ne - 1] = (1 / l[ne - 1]) * e[ne - 1];

            _ll = 1 / MPoint.DistanceTo(frames[nv - 2].Origin, frames[0].Origin);
            t[nv - 1] = _ll * (l[ne - 1] * u[ne - 2] + l[ne - 2] * u[ne - 1]);
            κb[nv - 1] = (2 * _ll) * MVector.CrossProduct(u[ne - 2], u[ne - 1]);

            // i = 0
            _ll = 1 / MPoint.DistanceTo(frames[nv - 1].Origin, frames[1].Origin);
            t[0] = _ll * (l[0] * u[ne - 1] + l[ne - 1] * u[0]);
            κb[0] = (2 * _ll) * MVector.CrossProduct(u[ne - 1], u[0]);
        }

        // CURVATURE FOR DICONTINUOUS CENTERLINE

        /// <summary>
        /// Gets the curvature and geometric invariants of a centerline with ghost vertices.
        /// </summary>
        /// <remarks>
        /// The total number of edges is always even for such a centerline.
        /// If it is closed, Nv = Ne, Nvg = Ne/2, Nvh = Nvg.
        /// If it is open, Nv = Ne + 1, Nvg = Ne/2, Nvh = Nvg + 1.
        /// </remarks>
        /// <param name="x">The centerline vertices composed of handle and ghost vertices.</param>
        /// <param name="e">The centerline edge vector.</param>
        /// <param name="u">The centerline unit edge vector.</param>
        /// <param name="l">The centerline edge length.</param>
        /// <param name="t_h_r">The right tangent vector at handle vertex.</param>
        /// <param name="t_h_r">The tangent vector at ghost vertex.</param>
        /// <param name="t_h_r">The left tangent vector at handle vertex.</param>
        /// <param name="κb_g">The curvature binormal vector at ghost vertex.</param>
        /// <param name="isClosed">True if the centerline is closed. False otherwise.</param>
        public static void GetCurvature(
            MPoint[] x,
            MVector[] e, MVector[] u, double[] l,
            MVector[] t_h_r, MVector[] t_g, MVector[] t_h_l, MVector[] κb_g,
            bool isClosed)
        {
            if (isClosed)
            {
                GetCurvature_Closed(x, e, u, l, t_h_r, t_g, t_h_l, κb_g);
            }
            else
            {
                GetCurvature_Open(x, e, u, l, t_h_r, t_g, t_h_l, κb_g);
            }
        }

        private static void GetCurvature_Open(
            MPoint[] x, 
            MVector[] e, MVector[] u, double[] l,
            MVector[] t_h_r, MVector[] t_g, MVector[] t_h_l, 
            MVector[] κb_g)
        {
            int nv_g = e.Length / 2; // must have an even number of edges (either open or closed centerline).

            int index;
            double l1, l2, _ll;
            MVector u1, u2, t;

            // i = 0, ..., nv_g-1
            for (int i = 0; i < nv_g; i++)
            {
                // 2i
                index = 2 * i;
                e[index] = new MVector(x[index], x[index + 1]);
                l1 = e[index].Length();
                l[index] = l1;
                u1 = (1 / l[index]) * e[index];
                u[index] = u1;

                // 2i+1
                index += 1;
                e[index] = new MVector(x[index], x[index + 1]);
                l2 = e[index].Length();
                l[index] = l2;
                u2 = (1 / l[index]) * e[index];
                u[index] = u2;

                // i
                _ll = 1 / new MVector(x[index - 1], x[index + 1]).Length();
                κb_g[i] = (2 * _ll) * MVector.CrossProduct(u1, u2);

                t = (l2 * _ll) * u1 + (l1 * _ll) * u2;
                t_h_r[i] = (2 * (t * u1)) * u1 - t;
                t_g[i] = t;
                t_h_l[i + 1] = (2 * (t * u2)) * u2 - t;
            }
        }

        private static void GetCurvature_Closed(
            MPoint[] x,
            MVector[] e, MVector[] u, double[] l,
            MVector[] t_h_r, MVector[] t_g, MVector[] t_h_l,
            MVector[] κb_g)
        {
            int nv_g = e.Length / 2; // must have an even number of edges (either open or closed centerline).

            int i, index;
            double l1, l2, _ll;
            MVector u1, u2, t;

            // i = 0, ..., nv_g-2
            for (i = 0; i < nv_g - 1; i++)
            {
                // 2i
                index = 2 * i;
                e[index] = new MVector(x[index], x[index + 1]);
                l1 = e[index].Length();
                l[index] = l1;
                u1 = (1 / l[index]) * e[index];
                u[index] = u1;

                // 2i+1
                index += 1;
                e[index] = new MVector(x[index], x[index + 1]);
                l2 = e[index].Length();
                l[index] = l2;
                u2 = (1 / l[index]) * e[index];
                u[index] = u2;

                // i
                _ll = 1 / new MVector(x[index - 1], x[index + 1]).Length();
                κb_g[i] = (2 * _ll) * MVector.CrossProduct(u1, u2);

                t = (l2 * _ll) * u1 + (l1 * _ll) * u2;
                t_h_r[i] = (2 * (t * u1)) * u1 - t;
                t_g[i] = t;
                t_h_l[i + 1] = (2 * (t * u2)) * u2 - t;
            }

            // i = nv_g-1
            i = nv_g - 1;

            // 2i
            index = 2 * i;
            e[index] = new MVector(x[index], x[index + 1]);
            l1 = e[index].Length();
            l[index] = l1;
            u1 = (1 / l[index]) * e[index];
            u[index] = u1;

            // 2i+1
            index += 1;
            e[index] = new MVector(x[index], x[0]);
            l2 = e[index].Length();
            l[index] = l2;
            u2 = (1 / l[index]) * e[index];
            u[index] = u2;

            // i
            _ll = 1 / new MVector(x[index - 1], x[0]).Length();
            κb_g[i] = (2 * _ll) * MVector.CrossProduct(u1, u2);

            t = (l2 * _ll) * u1 + (l1 * _ll) * u2;
            t_h_r[i] = (2 * (t * u1)) * u1 - t;
            t_g[i] = t;
            t_h_l[i + 1] = (2 * (t * u2)) * u2 - t;
        }

        // CURVATURE FOR DICONTINUOUS CENTERLINE (V2)
        // same as V1 but with different input & output parameters

        /// <summary>
        /// Gets the curvature and geometric invariants of a centerline with ghost vertices (V2).
        /// </summary>
        /// <remarks>
        /// The total number of edges is always even for such a centerline.
        /// If it is closed, Nv = Ne, Nvg = Ne/2, Nvh = Nvg.
        /// If it is open, Nv = Ne + 1, Nvg = Ne/2, Nvh = Nvg + 1.
        /// </remarks>
        /// <param name="x">The centerline vertices composed of handle and ghost vertices.</param>
        /// <param name="e">The centerline edge vector.</param>
        /// <param name="u">The centerline unit edge vector.</param>
        /// <param name="l">The centerline edge length.</param>
        /// <param name="t">The tangent vector at handle or ghost vertex.</param>
        /// <param name="κb_g">The curvature binormal vector at ghost vertex.</param>
        /// <param name="isClosed">True if the centerline is closed. False otherwise.</param>
        public static void GetCurvature(
            MPoint[] x,
            MVector[] e, MVector[] u, double[] l,
            MVector[] t, MVector[] κb_g,
            bool isClosed)
        {
            if (isClosed)
            {
                GetCurvature_Closed(x, e, u, l, t, κb_g);
            }
            else
            {
                GetCurvature_Open(x, e, u, l, t, κb_g);
            }
        }

        private static void GetCurvature_Open(
            MPoint[] x,
            MVector[] e, MVector[] u, double[] l,
            MVector[] t, MVector[] κb_g)
        {
            int nv_g = e.Length / 2; // must have an even number of edges (either open or closed centerline).

            int index;
            double l1, l2, _ll;
            MVector u1, u2, t_h_r, t_g, t_h_l;

            // necessary
            t[0] = MVector.Zero;

            // i = 0, ..., nv_g-1
            for (int i = 0; i < nv_g; i++)
            {
                // 2i
                index = 2 * i;
                e[index] = new MVector(x[index], x[index + 1]);
                l1 = e[index].Length();
                l[index] = l1;
                u1 = (1 / l[index]) * e[index];
                u[index] = u1;

                // 2i+1
                index += 1;
                e[index] = new MVector(x[index], x[index + 1]);
                l2 = e[index].Length();
                l[index] = l2;
                u2 = (1 / l[index]) * e[index];
                u[index] = u2;

                // i
                _ll = 1 / new MVector(x[index - 1], x[index + 1]).Length();
                κb_g[i] = (2 * _ll) * MVector.CrossProduct(u1, u2);

                t_g = (l2 * _ll) * u1 + (l1 * _ll) * u2;    // = t_g[i]
                t_h_r = (2 * (t_g * u1)) * u1 - t_g;        // = t_h_r[i]
                t_h_l = (2 * (t_g * u2)) * u2 - t_g;        // = t_h_l[i + 1]

                t[index] = t_g;
                t[index - 1] += t_h_r;
                t[index - 1].Normalize();  // t[2i] = (t_h_l[i] + t_h_r[i]) / | t_h_l[i] + t_h_r[i] |
                t[index + 1] = t_h_l;
            }
        }

        private static void GetCurvature_Closed(
            MPoint[] x,
            MVector[] e, MVector[] u, double[] l,
            MVector[] t, MVector[] κb_g)
        {
            int nv_g = e.Length / 2; // must have an even number of edges (either open or closed centerline).

            int i, index;
            double l1, l2, _ll;
            MVector u1, u2, t_h_r, t_g, t_h_l;

            // necessary
            t[0] = MVector.Zero;

            // i = 0, ..., nv_g-2
            for (i = 0; i < nv_g - 1; i++)
            {
                // 2i
                index = 2 * i;
                e[index] = new MVector(x[index], x[index + 1]);
                l1 = e[index].Length();
                l[index] = l1;
                u1 = (1 / l[index]) * e[index];
                u[index] = u1;

                // 2i+1
                index += 1;
                e[index] = new MVector(x[index], x[index + 1]);
                l2 = e[index].Length();
                l[index] = l2;
                u2 = (1 / l[index]) * e[index];
                u[index] = u2;

                // i
                _ll = 1 / new MVector(x[index - 1], x[index + 1]).Length();
                κb_g[i] = (2 * _ll) * MVector.CrossProduct(u1, u2);

                t_g = (l2 * _ll) * u1 + (l1 * _ll) * u2;    // = t_g[i]
                t_h_r = (2 * (t_g * u1)) * u1 - t_g;        // = t_h_r[i]
                t_h_l = (2 * (t_g * u2)) * u2 - t_g;        // = t_h_l[i + 1]

                t[index] = t_g;
                t[index - 1] += t_h_r;
                t[index - 1].Normalize();   // t[2i] = (t_h_l[i] + t_h_r[i]) / | t_h_l[i] + t_h_r[i] |
                t[index + 1] = t_h_l;
            }

            // i = nv_g-1
            i = nv_g - 1;

            // 2i
            index = 2 * i;
            e[index] = new MVector(x[index], x[index + 1]);
            l1 = e[index].Length();
            l[index] = l1;
            u1 = (1 / l[index]) * e[index];
            u[index] = u1;

            // 2i+1
            index += 1;
            e[index] = new MVector(x[index], x[0]);
            l2 = e[index].Length();
            l[index] = l2;
            u2 = (1 / l[index]) * e[index];
            u[index] = u2;

            // i
            _ll = 1 / new MVector(x[index - 1], x[0]).Length();
            κb_g[i] = (2 * _ll) * MVector.CrossProduct(u1, u2);

            t_g = (l2 * _ll) * u1 + (l1 * _ll) * u2;    // = t_g[nv_g-1]
            t_h_r = (2 * (t_g * u1)) * u1 - t_g;        // = t_h_r[nv_h-1]
            t_h_l = (2 * (t_g * u2)) * u2 - t_g;        // = t_h_l[0]

            t[index] = t_g;
            t[index - 1] += t_h_r;
            t[index - 1].Normalize(); 
            t[0] += t_h_l;
            t[0].Normalize();
        }

        // Get Twist

        public static void GetTwist(MFrame[] frames, double[] l, double[] τ, bool isClosed)
        {
            if (isClosed)
            {
                GetTwist_Closed(frames, l, τ);
            }
            else
            {
                GetTwist_Open(frames, l, τ);
            }
        }
        private static void GetTwist_Open(MFrame[] frames, double[] l, double[] τ)
        {
            int ne = l.Length; // open centerline

            double twistAngle;

            // i = 0, ..., ne-1
            for (int i = 0; i < ne; i++)
            {
                // compute twist

                twistAngle = -Rotation.ZAngle_Rotation(frames[i], frames[i].ZAxis, frames[i + 1], frames[i + 1].ZAxis);
                τ[i] = twistAngle / l[i];
            }
        }
        private static void GetTwist_Closed(MFrame[] frames, double[] l, double[] τ)
        {
            int nv = frames.Length;
            int ne = nv; // closed centerline

            double twistAngle;

            // i = 0, ..., ne-2
            for (int i = 0; i < ne-1; i++)
            {
                // compute twist
                twistAngle = -Rotation.ZAngle_Rotation(frames[i], frames[i].ZAxis, frames[i + 1], frames[i + 1].ZAxis);
                τ[i] = twistAngle / l[i];
            }

            // i = ne-1
            twistAngle = -Rotation.ZAngle_Rotation(frames[nv - 1], frames[nv - 1].ZAxis, frames[0], frames[0].ZAxis);
            τ[ne - 1] = twistAngle / l[ne - 1];
        }

        // Centerline Interpolation & Refine   
        /// <summary>
        /// Interpolates a frame at mid span between two frames.
        /// </summary>
        /// <remarks>
        /// Assumes the new frame lies on the circle passing throug f1 and f2 with mean curvature κb = 0.5 * (κb1 + κb2).
        /// The rate of twist is assumed to be uniform and thus the interpolated frame is rotated by (0.5 * l * τ) around it's ZAxis where l = |e|
        /// The ZAxis of the interpolated frame is nothing but the normalized edge : u = e/|e|
        /// </remarks>
        /// <param name="f1">The first frame.</param>
        /// <param name="f2">The second frame.</param>
        /// <param name="κb1">The curvature at first frame.</param>
        /// <param name="κb2">The curvature at first frame.</param>
        /// <param name="τ">The constant rate of twist between the frames.</param>
        /// <returns>The interpolated frame, equidistant from f1 and f2.</returns>
        public static MFrame Interpolate(MFrame f1, MFrame f2, MVector κb1, MVector κb2, double τ1, double τ2)
        {
            MPoint x;
            MFrame f = new MFrame();

            var κb = 0.5 * (κb1 + κb2);
            var κ = κb.Length();
            var e = new MVector(f1.Origin, f2.Origin);
            var l = e.Length();
            var u = (1 / l) * e;

            var τ = 0.5 * (τ1 + τ2);                // assumes that τ(s) = τ1 + s/l(τ2 - τ1)
            var dθ = (l / 8) * (3 * τ1 + τ2);       // dθ = int(τ(s)ds, [0,l/2])

            if (κ == 0)
            {
                x = 0.5 * (f1.Origin + f2.Origin);
            }
            else
            {
                var r = 1 / κ;
                var h = r - Math.Sqrt(r * r - l * l / 4);
                var n = -r * MVector.CrossProduct(u, κb);

                x = f1.Origin + 0.5 * e - h * n;
            }

            ParallelTransportation.ZPT_Rotation(f1, f1.ZAxis, x, u, ref f);
            Rotation.ZRotate(f, dθ, ref f);

            return f;
        }
        public static MFrame Interpolate(MFrame f1, MFrame f2, MVector κb1, MVector κb2, double τ)
        {
            return Interpolate(f1, f2, κb1, κb2, τ, τ);
        }
        public static MPoint Interpolate(MPoint p1, MPoint p2, MVector κb1, MVector κb2)
        {
            MPoint x;
            MPoint p = new MPoint();

            var κb = 0.5 * (κb1 + κb2);
            var κ = κb.Length();
            var e = new MVector(p1, p2);
            var l = e.Length();
            var u = (1 / l) * e;

            if (κ == 0)
            {
                x = 0.5 * (p1 + p2);
            }
            else
            {
                var r = 1 / κ;
                var h = r - Math.Sqrt(r * r - l * l / 4);
                var n = -r * MVector.CrossProduct(u, κb);

                x = p1 + 0.5 * e - h * n;
            }
            return p;
        }

        /// <summary>
        /// Applies the <see cref="Interpolate">Interpolate</see> process over a whole centerline given by its frames.
        /// </summary>
        /// <param name="frames">The frames that represent the intial centerline.</param>
        /// <param name="κb">The curvature at each frame.</param>
        /// <param name="τ">The rate of twist at each edge.</param>
        /// <param name="isClosed">True if the centerline is closed. False otherwise.</param>
        /// <param name="recursionCount">How many times to apply recursively the interpolation process.</param>
        /// <returns>The set of interpolated frames representing the centerline.</returns>
        public static MFrame[] Refine(MFrame[] frames, MVector[] κb, double[] τ, bool isClosed, int recursionCount = 1)
        {
            if (isClosed)
            {
                return Refine_Closed(frames, κb, τ, recursionCount);
            }
            else
            {
                return Refine_Open(frames, κb, τ, recursionCount);
            }
        }
        private static MFrame[] Refine_Open(MFrame[] frames, MVector[] κb, double[] τ, int recursionCount = 1)
        {
            MFrame[] framesInterp = Refine_Open(frames, κb, τ);

            for (int i = 1; i < recursionCount; i++)
            {
                int nv = framesInterp.Length;
                int ne = nv - 1;

                var x_tmp = new MPoint[nv];
                var e_tmp = new MVector[ne];
                var u_tmp = new MVector[ne];
                var t_tmp = new MVector[nv];
                var κb_tmp = new MVector[nv];
                var l_tmp = new double[ne];
                var τ_tmp = new double[ne];

                Centerline.GetCurvature_Open(framesInterp, x_tmp, e_tmp, u_tmp, l_tmp, t_tmp, κb_tmp);
                Centerline.ZAlignFrames(framesInterp, t_tmp);
                Centerline.GetTwist_Open(framesInterp, l_tmp, τ_tmp);

                framesInterp = Refine_Open(framesInterp, κb_tmp, τ_tmp);
            }

            return framesInterp;
        }
        private static MFrame[] Refine_Open(MFrame[] frames, MVector[] κb, double[] τ)
        {
            int nv = frames.Length;
            int ne = nv - 1; // open centerline

            var framesInterp = new MFrame[nv + ne];

            for (int i = 0; i < ne; i++)
            {
                framesInterp[2 * i] = frames[i];
                framesInterp[2 * i + 1] = Interpolate(frames[i], frames[i + 1], κb[i], κb[i + 1], τ[i]);
            }

            framesInterp[nv + ne - 1] = frames[nv - 1];


            return framesInterp;
        }
        private static MFrame[] Refine_Closed(MFrame[] frames, MVector[] κb, double[] τ, int recursionCount = 1)
        {
            MFrame[] framesInterp = Refine_Closed(frames, κb, τ);

            for (int i = 1; i < recursionCount; i++)
            {
                int nv = framesInterp.Length;
                int ne = nv;

                var x_tmp = new MPoint[nv];
                var e_tmp = new MVector[ne];
                var u_tmp = new MVector[ne];
                var t_tmp = new MVector[nv];
                var κb_tmp = new MVector[nv];
                var l_tmp = new double[ne];
                var τ_tmp = new double[ne];

                Centerline.GetCurvature_Closed(framesInterp, x_tmp, e_tmp, u_tmp, l_tmp, t_tmp, κb_tmp);
                Centerline.ZAlignFrames(framesInterp, t_tmp);
                Centerline.GetTwist_Closed(framesInterp, l_tmp, τ_tmp);

                framesInterp = Refine_Closed(framesInterp, κb_tmp, τ_tmp);
            }

            return framesInterp;
        }
        private static MFrame[] Refine_Closed(MFrame[] frames, MVector[] κb, double[] τ)
        {
            int nv = frames.Length;
            int ne = nv; // open centerline

            var framesInterp = new MFrame[nv + ne];

            for (int i = 0; i < ne-1; i++)
            {
                framesInterp[2 * i] = frames[i];
                framesInterp[2 * i + 1] = Interpolate(frames[i], frames[i + 1], κb[i], κb[i + 1], τ[i]);
            }

            framesInterp[nv + ne - 2] = frames[nv - 1];
            framesInterp[nv + ne - 1] = Interpolate(frames[nv - 1], frames[0], κb[nv - 1], κb[0], τ[ne - 1]);
            
            return framesInterp;
        }
        
        public static MVector[] RefineVertexBasedQuantity(MVector[] vertexBasedQuantity, bool isClosed, int recursionCount = 1)
        {
            var tmp = RefineVertexBasedQuantity(vertexBasedQuantity, isClosed);

            for (int i = 1; i < recursionCount; i++)
            {
                tmp = RefineVertexBasedQuantity(tmp, isClosed);
            }

            return tmp;
        }
        private static MVector[] RefineVertexBasedQuantity(MVector[] vertexBasedQuantity, bool isClosed)
        {
            if (!isClosed)
            {
                int nv = vertexBasedQuantity.Length;
                int ne = nv - 1; // open centerline

                var refinedArray = new MVector[nv + ne];

                refinedArray[0] = vertexBasedQuantity[0];

                for (int i = 0; i < ne; i++)
                {
                    refinedArray[2 * i + 1] = 0.5 * (vertexBasedQuantity[i] + vertexBasedQuantity[i + 1]);
                    refinedArray[2 * i + 2] = vertexBasedQuantity[i + 1];
                }
                return refinedArray;

            }
            else
            {
                int nv = vertexBasedQuantity.Length;
                int ne = nv; // closed centerline

                MVector[] refinedArray = new MVector[nv + ne];

                refinedArray[0] = vertexBasedQuantity[0];

                for (int i = 0; i < ne-1; i++)
                {
                    refinedArray[2 * i + 1] = 0.5 * (vertexBasedQuantity[i] + vertexBasedQuantity[i + 1]);
                    refinedArray[2 * i + 2] = vertexBasedQuantity[i + 1];
                }
                refinedArray[nv + ne - 1] = 0.5 * (vertexBasedQuantity[nv - 1] + vertexBasedQuantity[0]);

                return refinedArray;
            }  
        }
        public static double[] RefineVertexBasedQuantity(double[] vertexBasedQuantity, bool isClosed, int recursionCount = 1)
        {
            var tmp = RefineVertexBasedQuantity(vertexBasedQuantity, isClosed);

            for (int i = 1; i < recursionCount; i++)
            {
                tmp = RefineVertexBasedQuantity(tmp, isClosed);
            }

            return tmp;
        }
        private static double[] RefineVertexBasedQuantity(double[] vertexBasedQuantity, bool isClosed)
        {
            if (!isClosed)
            {
                int nv = vertexBasedQuantity.Length;
                int ne = nv - 1; // open centerline

                var refinedArray = new double[nv + ne];

                refinedArray[0] = vertexBasedQuantity[0];

                for (int i = 0; i < ne; i++)
                {
                    refinedArray[2 * i + 1] = 0.5 * (vertexBasedQuantity[i] + vertexBasedQuantity[i + 1]);
                    refinedArray[2 * i + 2] = vertexBasedQuantity[i + 1];
                }
                return refinedArray;

            }
            else
            {
                int nv = vertexBasedQuantity.Length;
                int ne = nv; // closed centerline

                var refinedArray = new double[nv + ne];

                refinedArray[0] = vertexBasedQuantity[0];

                for (int i = 0; i < ne - 1; i++)
                {
                    refinedArray[2 * i + 1] = 0.5 * (vertexBasedQuantity[i] + vertexBasedQuantity[i + 1]);
                    refinedArray[2 * i + 2] = vertexBasedQuantity[i + 1];
                }
                refinedArray[nv + ne - 1] = 0.5 * (vertexBasedQuantity[nv - 1] + vertexBasedQuantity[0]);

                return refinedArray;
            }
        }

        public static MVector[] RefineEdgeBasedQuantity(MVector[] edgeBasedQuantity, bool isClosed, int recursionCount = 1)
        {
            var tmp = RefineEdgeBasedQuantity(edgeBasedQuantity, isClosed);
            for (int i = 1; i < recursionCount; i++)
            {
                tmp = RefineVertexBasedQuantity(tmp, isClosed);
            }
            return tmp;
        }
        private static MVector[] RefineEdgeBasedQuantity(MVector[] edgeBasedQuantity, bool isClosed)
        {
            int ne = edgeBasedQuantity.Length;
            var refinedArray = new MVector[ne + ne];
            refinedArray[0] = edgeBasedQuantity[0];
            for (int i = 0; i < ne; i++)
            {
                refinedArray[2 * i] = edgeBasedQuantity[i];
                refinedArray[2 * i + 1] = edgeBasedQuantity[i];
            }
            return refinedArray;     
        }
        public static double[] RefineEdgeBasedQuantity(double[] edgeBasedQuantity, bool isClosed, int recursionCount = 1)
        {
            var tmp = RefineEdgeBasedQuantity(edgeBasedQuantity, isClosed);
            for (int i = 1; i < recursionCount; i++)
            {
                tmp = RefineVertexBasedQuantity(tmp, isClosed);
            }
            return tmp;
        }
        private static double[] RefineEdgeBasedQuantity(double[] edgeBasedQuantity, bool isClosed)
        {
            int ne = edgeBasedQuantity.Length;
            var refinedArray = new double[ne + ne];
            refinedArray[0] = edgeBasedQuantity[0];
            for (int i = 0; i < ne; i++)
            {
                refinedArray[2 * i] = edgeBasedQuantity[i];
                refinedArray[2 * i + 1] = edgeBasedQuantity[i];
            }
            return refinedArray;
        }

    }
}
