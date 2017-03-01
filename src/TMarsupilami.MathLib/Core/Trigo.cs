using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public delegate double TrigoFunction(double angle);

    public static partial class Trigo
    {
        private const double _2_PI  = 2 * System.Math.PI;    // touch carrefully
        private const double _PI_2  = System.Math.PI / 2;    // touch carrefully
        private const double _PI_64 = System.Math.PI / 64;  // touch carrefully

        /// <summary>
        /// For a given angle in R, returns the principal angle in ]-π,π]
        /// </summary>
        /// <param name="angle">The given angle in radians.</param>
        /// <returns>The principal angle in ]-pi;pi].</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double PrincipalAngle(double angle)
        {
            double k = System.Math.Floor((System.Math.PI - angle) / _2_PI);
            return angle + k * _2_PI;
        }

        /// <summary>
        /// This methods gives a fast approximation of sin(x) assuming x is in [-pi/2;pi/2]
        /// WARNING : if x is not in the recommended range the results are unreliable
        /// </summary>
        /// <param name="x"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Sin(double x)
        {
#if FAST
            // using a remez interpolation over [-pi/2,pi/2]
            double s1 = 1.0;
            double s3 = -0.16607862421693137;
            double s5 = 0.007633773374658546;
            double x2 = x * x;
            return x * (s1 + x2 * (s3 + x2 * s5));
#else
            return System.Math.Sin(x);
#endif
        }

        /// <summary>
        /// This method gives a fast approximation of sin(x) cos(x) assuming x is in [-pi/2;pi/2]
        /// This method guaranty the trigonometric identity sin^2 + cos^2 = 1 in double precision.
        /// Thus, this method must be used to compute fast rotation if x is in [-pi/2;pi/2].
        /// NOTE : in [-pi/2;pi/2] x * sin(x) > 0 & cos(x) > 0
        /// WARNING : if x is not in the recommended range the results are unreliable
        /// </summary>
        /// <param name="x"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SinCos(double x, out double sin, out double cos)
        {
#if FAST
            // using a remez interpolation of sin over [-pi/2,pi/2]
            double s1 = 1.0;
            double s3 = -0.16607862421693137;
            double s5 = 0.007633773374658546;
            double x2 = x * x;
            sin = x * (s1 + x2 * (s3 + x2 * s5));
            cos = Math.Sqrt(1 - sin * sin);
#else
            sin = System.Math.Sin(x);
            cos = System.Math.Cos(x);
#endif
        }

    }
}
