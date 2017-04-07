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
        public static void Open_GetCurvature(MFrame[] frames, MPoint[] x, MVector[] e, MVector[] u, double[] l, MVector[] t, MVector[] κb, double[] τ)
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
        public static void Open_AlignFrames(MFrame[] frames, MVector[] t)
        {
            int nv = frames.Length;
            int ne = nv - 1; // open centerline

            // i = 1, ..., nv-2
            for (int i = 1; i < nv - 1; i++)
            {
                // forces frames[i] to be parallel to the centerline tangent vector (t)
                ParallelTransportation.ZPT_Rotation(frames[i], frames[i].ZAxis, frames[i].Origin, t[i], ref frames[i]);
            }
        }
        public static void Open_GetTwist(MFrame[] frames, double[] l, double[] τ)
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

        public static void Closed_GetCurvature(MFrame[] frames, MPoint[] x, MVector[] e, MVector[] u, double[] l, MVector[] t, MVector[] κb, double[] τ)
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
        public static void Closed_AlignFrames(MFrame[] frames, MVector[] t)
        {
            int nv = frames.Length;
            int ne = nv; // closed centerline

            // i = 0, ..., nv-1
            for (int i = 0; i < nv; i++)
            {
                // forces frames[i] to be parallel to the centerline tangent vector (t)
                ParallelTransportation.ZPT_Rotation(frames[i], frames[i].ZAxis, frames[i].Origin, t[i], ref frames[i]);
            }
        }
        public static void Closed_GetTwist(MFrame[] frames, double[] l, double[] τ)
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
    }
}
