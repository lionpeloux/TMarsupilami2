using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class Rotation
    {
        /// <summary>
        /// Rotates a frame by an angle θ around its ZAxis.
        /// ------------------------------------
        /// add  | sub   | mul   | div   | sqrt
        /// 3    | 6     | 12    | 0     | 0
        /// ------------------------------------
        /// cos  | sin   | tan   | acos  | asin
        /// 1    | 1     | 0     | 0     | 0
        /// ------------------------------------
        /// cost = 177
        /// </summary>
        /// <param name="frame">The frame to be rotated.</param>
        /// <param name="θ">The oriented angle of rotation (around the ZAxis).</param>
        /// <param name="frameROT">The rotated frame.</param>
        public static void ZRotate(MFrame frame, double θ, ref MFrame frameROT)
        {
            /* ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 3    | 6     | 12    | 0     | 0
             * ------------------------------------          
             * cos  | sin   | tan   | acos  | asin
             * 1    | 1     | 0     | 0     | 0
             * ------------------------------------   
             */

            // cos :  1 | sin :  1
            double c = Math.Cos(θ);
            double s = Math.Sin(θ);

            var d1 = frame.XAxis;
            var d2 = frame.YAxis;

            // add :  3 | sub :  3 | mul :  12 | div :  0 | sqrt :  0
            var d1_rot = c * d1 + s * d2;
            var d2_rot = c * d2 - s * d1;

            frameROT.Origin = frame.Origin;
            frameROT.XAxis = d1_rot;
            frameROT.YAxis = d2_rot;
        }

        /// <summary>
        /// Rotates a frame by an angle dθ around its ZAxis. 
        /// dθ is assumed to be very small so that tan is approximated with the Taylor developpement of order 3 :
        /// tan(dθ/2) = (dθ/2) + 1/3*(dθ/2)^3 + o(dθ^3)
        /// ------------------------------------
        /// add  | sub   | mul   | div   | sqrt
        /// 3    | 6     | 12    | 0     | 0
        /// ------------------------------------
        /// cos  | sin   | tan   | acos  | asin
        /// 1    | 1     | 0     | 0     | 0
        /// ------------------------------------
        /// cost = 177
        /// </summary>
        /// <param name="frame">The frame to be rotated.</param>
        /// <param name="dθ">The oriented small angle of rotation (around the ZAxis).</param>
        /// <param name="frameROT">The rotated frame.</param>
        public static void ZDiffRotate_Taylor_3(MFrame frame, double dθ, ref MFrame frameROT)
        {
            /* ------------------------------------
             * NOTES : fast rotation
             * ------------------------------------ 
             * Here we consider that dθ is very closed to zero.
             * We estimate t = tan(dθ/2) = dθ * (1/2 + 1/18 * dθ^2) + o(dθ^3)
             * 
             * We compute sin and cos with :
             *  s = 2t/(1+t^2)
             *  c = (1-t^2)/(1+t^2)
             *  
             * We compute the rotated vectors as :
             *  d1_rot = c * d1 + s * d2
             *  d2_rot = c * d2 - s * d1
             *  
             * Thus, the identity c^2+s^2 = 1 is exactly verified. This ensure that the rotation operation keep d1_rot and d2_rot normalized.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 5    | 4     | 19    | 1     | 0
             * ------------------------------------          
             * cos  | sin   | tan   | acos  | asin
             * 0    | 0     | 0     | 0     | 0
             * ------------------------------------   
             * score = 95
             */

            // add :  1 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            // 1/2 and 1/12 are donne at compile time
            double t = dθ * (0.5 + (1 / 18) * dθ * dθ);

            // add :  1 | sub :  0 | mul :  1 | div :  1 | sqrt :  0
            double t2 = t * t;
            double _d = 1 / (1 + t2);

            // add :  0 | sub :  1 | mul :  3 | div :  0 | sqrt :  0
            double s = 2 * t * _d;
            double c = (1 - t2) * _d;

            // add :  3 | sub :  3 | mul :  12 | div :  0 | sqrt :  0
            var d1 = frame.XAxis;
            var d2 = frame.YAxis;
            var d1_rot = c * d1 + s * d2;
            var d2_rot = c * d2 - s * d1;

            frameROT.Origin = frame.Origin;
            frameROT.XAxis = d1_rot;
            frameROT.YAxis = d2_rot;
        }

        /// <summary>
        /// Gets the Z angle (θz) to align two frames (f1 and f2)
        /// Firstly, f1 is parallel transported on f2 such that f1_para and f2 share the same ZAxis.
        /// Secondly, f1_para is rotated around its ZAxis by an angle θz to perfectly align with f2.
        /// </summary>
        /// <param name="fromFrame">The frame to align.</param>
        /// <param name="toFrame">The target frame to align to.</param>
        /// <returns>The Oriented Z angle (θz) between the frames in ]-π,π].</returns>
        public static double ZAngle(MFrame fromFrame, MFrame toFrame)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - XAxis and YAxis musrt be of unit length
             *  
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 10   | 18    | 33    | 2     | 0
             * ------------------------------------          
             */

            MFrame ptFrame = new MFrame();
            fromFrame.ZParallelTransport_Rotation(toFrame.Origin, toFrame.ZAxis, ref ptFrame); // warning : toFrame.ZAxis is expensive

            double dot_1 = MVector.DotProduct(ptFrame.XAxis, toFrame.XAxis);
            double dot_2 = MVector.DotProduct(ptFrame.YAxis, toFrame.XAxis);
            
            if (dot_2 >= 0) // θz in ]-π, 0]
            {
                return Math.Acos(dot_1);
            }
            else // θz in ]0, π]
            {
                return -Math.Acos(dot_1);
            }
        }

        ///// <summary>
        ///// Computes the angle on a plane between two vectors.
        ///// </summary>
        ///// <param name="v1">First vector.</param>
        ///// <param name="v2">Second vector.</param>
        ///// <param name="plane">Two-dimensional plane on which to perform the angle measurement.</param>
        ///// <returns>On success, the angle (in radians) between a and b as projected onto the plane; RhinoMath.UnsetValue on failure.</returns>
        //public static double VectorAngle(MVector v1, MVector v2, MFrame frame)
        //{

        //    { // Project vectors onto plane.
        //        Point3d pA = frame.Origin + v1;
        //        Point3d pB = frame.Origin + v2;

        //        pA = plane.ClosestPoint(pA);
        //        pB = plane.ClosestPoint(pB);

        //        v1 = pA - plane.Origin;
        //        v2 = pB - plane.Origin;
        //    }

        //    // Abort on invalid cases.
        //    if (!v1.Unitize()) { return RhinoMath.UnsetValue; }
        //    if (!v2.Unitize()) { return RhinoMath.UnsetValue; }

        //    double dot = v1 * v2;
        //    { // Limit dot product to valid range.
        //        if (dot >= 1.0)
        //        { dot = 1.0; }
        //        else if (dot < -1.0)
        //        { dot = -1.0; }
        //    }

        //    double angle = Math.Acos(dot);
        //    { // Special case (anti)parallel vectors.
        //        if (Math.Abs(angle) < 1e-64) { return 0.0; }
        //        if (Math.Abs(angle - Math.PI) < 1e-64) { return Math.PI; }
        //    }

        //    Vector3d cross = Vector3d.CrossProduct(v1, v2);
        //    if (plane.ZAxis.IsParallelTo(cross) == +1)
        //        return angle;
        //    return 2.0 * Math.PI - angle;
        //}

    }
}
