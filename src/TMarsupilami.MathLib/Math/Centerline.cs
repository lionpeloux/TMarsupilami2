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

        // Get Curvature and other geometric properties

        /// <summary>
        /// Gets the curvature and geometric invariants of a given centerline.
        /// </summary>
        /// <param name="frames">The centerline as a set of frames.</param>
        /// <param name="x">The centerline vertices.</param>
        /// <param name="e">The centerline edge vectors.</param>
        /// <param name="u">The centerline unit edge vectors.</param>
        /// <param name="l">The centerline edge length.</param>
        /// <param name="t">The centerline unit tangent vectors.</param>
        /// <param name="κb">The centerline curvature binormal vector.</param>
        /// <param name="τ">The centerline rate of twist.</param>
        /// <param name="isClosed">True is the centerline is closed. False otherwise.</param>
        public static void GetCurvature(MFrame[] frames, MPoint[] x, MVector[] e, MVector[] u, double[] l, MVector[] t, MVector[] κb, double[] τ, bool isClosed)
        {
            if (isClosed)
            {
                GetCurvature_Closed(frames, x, e, u, l, t, κb, τ);
            }
            else
            {
                GetCurvature_Open(frames, x, e, u, l, t, κb, τ);
            }
        }
        private static void GetCurvature_Open(MFrame[] frames, MPoint[] x, MVector[] e, MVector[] u, double[] l, MVector[] t, MVector[] κb, double[] τ)
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
        private static void GetCurvature_Closed(MFrame[] frames, MPoint[] x, MVector[] e, MVector[] u, double[] l, MVector[] t, MVector[] κb, double[] τ)
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


        private static void GetCurvature2_Open(MFrame[] frames, MPoint[] x, MVector[] e, MVector[] u, double[] l, MVector[] t, MVector[] κb, double[] τ)
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
        public static MFrame Interpolate(MFrame f1, MFrame f2, MVector κb1, MVector κb2, double τ)
        {
            MPoint x;
            MFrame f = new MFrame();

            var κb = 0.5 * (κb1 + κb2);
            var κ = κb.Length();
            var e = new MVector(f1.Origin, f2.Origin);
            var l = e.Length();
            var u = (1 / l) * e;

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
            Rotation.ZRotate(f, 0.5 * τ * l, ref f);

            return f;
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

                Centerline.GetCurvature_Open(framesInterp, x_tmp, e_tmp, u_tmp, l_tmp, t_tmp, κb_tmp, τ_tmp);
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

                Centerline.GetCurvature_Closed(framesInterp, x_tmp, e_tmp, u_tmp, l_tmp, t_tmp, κb_tmp, τ_tmp);
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
    }
}
