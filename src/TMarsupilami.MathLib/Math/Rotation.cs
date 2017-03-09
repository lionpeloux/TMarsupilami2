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
    }
}
