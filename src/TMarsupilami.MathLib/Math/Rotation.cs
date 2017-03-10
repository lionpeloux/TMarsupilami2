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
        /// Rotates a frame by an angle θ around its ZAxis (cost index = 177).
        /// </summary>
        /// <remarks>
        /// Requires expensive cos(θ) and sin(θ) computation.
        /// </remarks>
        /// <param name="frame">The frame to be rotated.</param>
        /// <param name="θ">The oriented angle of rotation, around the ZAxis.</param>
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
        /// Rotates a frame by a small angle dθ around its ZAxis (cost index = 101).
        /// </summary>
        /// <remarks>
        /// dθ is assumed to be very small so that tan is approximated with the Taylor developpement of order 3 :
        /// tan(dθ/2) = (dθ/2) + 1/3*(dθ/2)^3 + o(dθ^3)
        /// </remarks>
        /// <param name="frame">The frame to be rotated.</param>
        /// <param name="dθ">The oriented small angle of rotation, around the frame ZAxis.</param>
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
             * 5    | 4     | 18    | 2     | 0
             * ------------------------------------          
             * cos  | sin   | tan   | acos  | asin
             * 0    | 0     | 0     | 0     | 0
             * ------------------------------------   
             * score = 95
             */

            // add :  1 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            // 1/2 and 1/12 are donne at compile time
            double t = dθ * (0.5 + (1 / 18) * dθ * dθ);

            // add :  1 | sub :  0 | mul :  1 | div :  0 | sqrt :  0
            double t2 = t * t;

            // add :  0 | sub :  1 | mul :  3 | div :  2 | sqrt :  0
            double s = 2 * t / (1 + t2);
            double c = (1 - t2) / (1 + t2);

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
        /// Gets the Z angle (θz) to align two frames using the rotation method (cost index = 182 + acos).
        /// </summary>
        /// <remarks>
        /// Firstly, fromFrame is parallel transported, with the rotation method, on toFrame such that fromFramePT and toFrame share the same ZAxis.
        /// Secondly, fromFramePT is rotated around its ZAxis by an angle θz to perfectly align with toFrame.
        /// </remarks>
        /// <param name="fromFrame">The frame to align.</param>
        /// <param name="fromZAxis">The ZAxis of the initial frame. Must be of unit length.</param>
        /// <param name="toFrame">The target frame to align to.</param>
        /// <param name="toZAxis">The ZAxis of the target frame. Must be of unit length.</param>
        /// <returns>The Oriented Z angle (θz) between the frames in ]-π,π] around toZAxis.</returns>
        public static double ZAngle_Rotation(MFrame fromFrame, MVector fromZAxis, MFrame toFrame, MVector toZAxis)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - Frame axis must be of unit length
             *  
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 15   | 9     | 37    | 1     | 0
             * ------------------------------------          
             * cos  | sin   | tan   | acos  | asin
             * 0    | 0     | 0     | 1     | 0
             * ------------------------------------   
             */

            MFrame framePT = new MFrame();

            // add : 11 | sub :  9 | mul : 30 | div :  1 | sqrt :  0
            fromFrame.ZParallelTransport_Rotation(fromZAxis, toFrame.Origin, toZAxis, ref framePT);

            // add :  4 | sub :  0 | mul :  6 | div :  0 | sqrt :  0
            double dot_1 = MVector.DotProduct(framePT.XAxis, toFrame.XAxis);
            double dot_2 = MVector.DotProduct(framePT.YAxis, toFrame.XAxis);

            if (dot_1 > 1)
            {
                return 0;
            }
            if (dot_1 < -1)
            {
                return Math.PI;
            }

            if (Math.Abs(dot_1) <= Math.Sqrt(2)/2) // => better if XAxis and YAxis are nearly perpendicular
            {
                if (dot_2 >= 0) // θz in [π/4, 3π/4]
                {
                    return Math.Acos(dot_1);      
                }
                else // θz in [-3π/4, -π/4]
                {
                    return -Math.Acos(dot_1);     
                }
            }
            else // => better if XAxis and YAxis are nearly colinear
            {
                if (dot_1 >= 0) // θz in ]-π/4, π/4[
                {
                    return Math.Asin(dot_2);       
                }
                else // θz in ]0, π]
                {
                    if (dot_2 >= 0) // θz in ]3π/4, π]
                    {
                        return (Math.PI) - Math.Asin(dot_2);
                    }
                    else // θz in ]-π, -3π/4]
                    {
                        return (-Math.PI) - Math.Asin(dot_2);
                    }
                }
            }
        }

        /// <summary>
        /// Gets the Z angle (θz) to align two frames using the double reflection method (cost index = 182 + acos).
        /// </summary>
        /// <remarks>
        /// Firstly, fromFrame is parallel transported, with the double reflection method, on toFrame such that fromFramePT and toFrame share the same ZAxis.
        /// Secondly, fromFramePT is rotated around its ZAxis by an angle θz to perfectly align with toFrame.
        /// </remarks>
        /// <param name="fromFrame">The frame to align.</param>
        /// <param name="fromZAxis">The ZAxis of the initial frame. Must be of unit length.</param>
        /// <param name="toFrame">The target frame to align to.</param>
        /// <param name="toZAxis">The ZAxis of the target frame. Must be of unit length.</param>
        /// <returns>The Oriented Z angle (θz) between the frames in ]-π,π] around toZAxis.</returns>
        public static double ZAngle_Reflection(MFrame fromFrame, MVector fromZAxis, MFrame toFrame, MVector toZAxis)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - Frame axis must be of unit length
             *  
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 14   | 18    | 39   | 2     | 0
             * ------------------------------------          
             * cos  | sin   | tan   | acos  | asin
             * 0    | 0     | 0     | 1     | 0
             * ------------------------------------   
             */

            MFrame framePT = new MFrame();

            // add : 10 | sub : 18 | mul : 33 | div :  2 | sqrt :  0
            fromFrame.ZParallelTransport_Reflection(fromFrame.Origin, fromZAxis, toFrame.Origin, toZAxis, ref framePT);

            // add :  4 | sub :  0 | mul :  6 | div :  0 | sqrt :  0
            double dot_1 = MVector.DotProduct(framePT.XAxis, toFrame.XAxis);
            double dot_2 = MVector.DotProduct(framePT.YAxis, toFrame.XAxis);

            if (dot_1 > 1)
            {
                return 0;
            }
            if (dot_1 < -1)
            {
                return Math.PI;
            }

            if (Math.Abs(dot_1) <= Math.Sqrt(2) / 2) // => better if XAxis and YAxis are nearly perpendicular
            {
                if (dot_2 >= 0) // θz in [π/4, 3π/4]
                {
                    return Math.Acos(dot_1);
                }
                else // θz in [-3π/4, -π/4]
                {
                    return -Math.Acos(dot_1);
                }
            }
            else // => better if XAxis and YAxis are nearly colinear
            {
                if (dot_1 >= 0) // θz in ]-π/4, π/4[
                {
                    return Math.Asin(dot_2);
                }
                else // θz in ]0, π]
                {
                    if (dot_2 >= 0) // θz in ]3π/4, π]
                    {
                        return (Math.PI) - Math.Asin(dot_2);
                    }
                    else // θz in ]-π, -3π/4]
                    {
                        return (-Math.PI) - Math.Asin(dot_2);
                    }
                }
            }
        }
    }
}
