using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class Rotation
    {

        #region FRAME

        /// <summary>
        /// In-place rotation of a frame around a given axis.
        /// </summary>
        /// <param name="vector">The source vector to rotate.</param>
        /// <param name="angle">Angle of rotation (in radians).</param>
        /// <param name="axis">Axis of rotation. Must be a unit vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MFrame Rotate(MFrame frame, double angle, MVector axis)
        {
            // warning : axis must be of unit length

            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);
            return  Rotate(frame, sin, cos, axis);
        }

        /// <summary>
        /// In-place rotation of a frame around a given axis.
        /// </summary>
        /// <param name="frame">The source frame to rotate.</param>
        /// <param name="sin">Sinus of the angle of rotation.</param>
        /// <param name="cos">Cosinus of the angle of rotation.</param>
        /// <param name="axis">Axis of rotation. Must be a unit vector.</param>
        public static MFrame Rotate(MFrame frame, double sin, double cos, MVector axis)
        {
            // warning : axis must be of unit length
            // assumse that ZAxis = XAxis x YAxis
            
            double ax = axis.X;
            double ay = axis.Y;
            double az = axis.Z;

            double sax = sin * axis.X;
            double say = sin * axis.Y;
            double saz = sin * axis.Z;

            double d = (1 - cos);
            double cax = d * ax;
            double cay = d * ay;
            double caz = d * az;

            // rotate XAxis

            double vx = frame.XAxis.X;
            double vy = frame.XAxis.Y;
            double vz = frame.XAxis.Z;

            double kx = cax * vx;
            double ky = cay * vy;
            double kz = caz * vz;

            double vrx = vx * cos + kx * ax + ky * ax + kz * ax + say * vz - saz * vy;
            double vry = vy * cos + kx * ay + ky * ay + kz * ay + saz * vx - sax * vz;
            double vrz = vz * cos + kx * az + ky * az + kz * az + sax * vy - say * vx;

            var xaxis = new MVector(vrx, vry, vrz);

            // rotate YAxis

            vx = frame.YAxis.X;
            vy = frame.YAxis.Y;
            vz = frame.YAxis.Z;

            kx = cax * vx;
            ky = cay * vy;
            kz = caz * vz;

            vrx = vx * cos + kx * ax + ky * ax + kz * ax + say * vz - saz * vy;
            vry = vy * cos + kx * ay + ky * ay + kz * ay + saz * vx - sax * vz;
            vrz = vz * cos + kx * az + ky * az + kz * az + sax * vy - say * vx;

            var yaxis = new MVector(vrx, vry, vrz);

            // update ZAxis with cross product
            var zaxis = MVector.CrossProduct(xaxis, yaxis);

            return new MFrame(frame.Origin, xaxis, yaxis, zaxis);
        }

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
        public static MFrame ZRotate(MFrame frame, double sin, double cos)
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

            var d1 = frame.XAxis;
            var d2 = frame.YAxis;

            // add :  3 | sub :  3 | mul :  12 | div :  0 | sqrt :  0
            var d1_rot = cos * d1 + sin * d2;
            var d2_rot = cos * d2 - sin * d1;

            return new MFrame(frame.Origin, d1_rot, d2_rot, frame.ZAxis, false);
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
        public static void ZDiffRotate_T3(MFrame frame, double dθ, ref MFrame frameROT)
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
            return ZAngle(framePT.XAxis, framePT.YAxis, toFrame.XAxis);
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
            return ZAngle(framePT.XAxis, framePT.YAxis, toFrame.XAxis);
        }

        /// <summary>
        /// Returns the angle in ]-pi,pi] between two frames (t,d1,d2) and (t,d1*,d2*) that share the same ZAxis.
        /// </summary>
        /// <param name="d1"></param>
        /// <param name="d2"></param>
        /// <param name="d1_star"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double ZAngle(MVector d1, MVector d2, MVector d1_star)
        {
            double dot_1 = MVector.DotProduct(d1_star, d1);
            double dot_2 = MVector.DotProduct(d1_star, d2);

            if (dot_1 > 1)
                return 0;
            
            if (dot_1 >= Math.Sqrt(2)/2)
                return -Math.Asin(dot_2);
            
            if (dot_1 > - Math.Sqrt(2)/2)
                return Math.Sign(dot_2) * (Math.Asin(dot_1) - (Math.PI / 2));
            
            if (dot_1 > -1)
                return Math.Asin(dot_2) - Math.Sign(dot_2) * Math.PI;

            return Math.PI;
        }

        #endregion

        #region VECTOR

        /// <summary>
        /// In-place rotation of a vector around a given axis.
        /// </summary>
        /// <param name="vector">The source vector to rotate.</param>
        /// <param name="angle">Angle of rotation (in radians).</param>
        /// <param name="axis">Axis of rotation. Must be a unit vector.</param>
        /// <returns>The rotated vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Rotate(ref MVector vector, double angle, MVector axis)
        {
            // warning : axis must be of unit length

            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);
            Rotate(ref vector, sin, cos, axis);
        }

        /// <summary>
        /// In-place rotation of a vector around a given axis.
        /// </summary>
        /// <param name="vector">The source vector to rotate.</param>
        /// <param name="sin">Sinus of the angle of rotation.</param>
        /// <param name="cos">Cosinus of the angle of rotation.</param>
        /// <param name="axis">Axis of rotation. Must be a unit vector.</param>
        public static void Rotate(ref MVector vector, double sin, double cos, MVector axis)
        {
            // warning : axis must be of unit length

            double vx = vector.X;
            double vy = vector.Y;
            double vz = vector.Z;

            double ax = axis.X;
            double ay = axis.Y;
            double az = axis.Z;

            double kx = (1 - cos) * vx * ax;
            double ky = (1 - cos) * vy * ay;
            double kz = (1 - cos) * vz * az;

            double vrx = vx * cos + kx * ax + ky * ax + kz * ax + sin * (ay * vz - az * vy);
            double vry = vy * cos + kx * ay + ky * ay + kz * ay + sin * (az * vx - ax * vz);
            double vrz = vz * cos + kx * az + ky * az + kz * az + sin * (ax * vy - ay * vx);

            vector.X = vrx;
            vector.Y = vry;
            vector.Z = vrz;

            //return new Vector(
            //    vx * c + (1 - c) * (vx * ax * ax + vy * ax * ay + vz * ax * az) + s * (ay * vz - az * vy),
            //    vy * c + (1 - c) * (vx * ax * ay + vy * ay * ay + vz * ay * az) + s * (az * vx - ax * vz),
            //    vz * c + (1 - c) * (vx * ax * az + vy * ay * az + vz * az * az) + s * (ax * vy - ay * vx)
            //    );
        }

        #endregion  
    }
}
