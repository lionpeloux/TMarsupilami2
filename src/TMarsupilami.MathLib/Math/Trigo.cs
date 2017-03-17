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

        #region TAYLOR
        public static double Sin_T1(double x)
        {
            // add :  0
            // mul :  0

            double p1 = 1;
            return x * p1;
        }
        public static double Sin_T3(double x)
        {
            // add :  1
            // mul :  3

            double p1 = 1;
            double p3 = -1 / 6;

            double x2 = x * x;
            return x * (p1 + x2 * p3);
        }
        public static double Sin_T5(double x)
        {
            // add :  1
            // mul :  3

            double p1 = 1;
            double p3 = -1 / 6;
            double p5 = 1 / 120;

            double x2 = x * x;
            return x * (p1 + x2 * (p3 + x2 * p5));
        }
        public static double Sin_T7(double x)
        {
            // add :  1
            // mul :  3

            double p1 = 1;
            double p3 = -1 / 6;
            double p5 = 1 / 120;
            double p7 = -1 / 5040;

            double x2 = x * x;
            return x * (p1 + x2 * (p3 + x2 * (p5 + x2 * p7)));
        }
        public static double Sin_T9(double x)
        {
            // add :  1
            // mul :  3

            double p1 = 1;
            double p3 = -1 / 6;
            double p5 = 1 / 120;
            double p7 = -1 / 5040;
            double p9 = 1 / 362880;

            double x2 = x * x;
            return x * (p1 + x2 * (p3 + x2 * (p5 + x2 * (p7 + x2 * p9))));
        }
        #endregion

        #region REMEZ
        public static double Sin_R3(double x)
        {
            // optimization over [-pi/4, pi/4] 
            // add :  1
            // mul :  3

            double p1 = 1;
            double p3 = -0.16662833806923272476;

            double x2 = x * x;
            return x * (p1 + x2 * p3);
        }
        public static double Sin_R5(double x)
        {
            // optimization over [-pi/4, pi/4] 
            // add :  2
            // mul :  4

            double p1 = 1;
            double p3 = -0.16662833806923272476;
            double p5 = 8.15299234169417208e-3;

            double x2 = x * x;
            return x*(p1+ x2*(p3 + x2*p5));
        }
        public static double Sin_R7(double x)
        {
            // optimization over [-pi/4, pi/4] 
            // add :  3
            // mul :  5

            double p1 = 1;
            double p3 = -0.1666665066929876438;
            double p5 = 8.331978663399449201e-3;
            double p7 = -1.9495636264301826503e-4;

            double x2 = x * x;
            return x * (p1 + x2 * (p3 + x2 * (p5 + x2 * p7)));
        }
        public static double Sin_R9(double x)
        {
            // optimization over [-pi/4, pi/4] 
            // add :  4
            // mul :  6

            double p1 = 1;
            double p3 = -0.166666666279990720609642152324994629615;
            double p5 = 8.33332823871530822441580014641203750665e-3;
            double p7 = -1.98390437703319082008068095794456309537e-4;
            double p9 = 2.716014015640842288168588815794202478e-6;

            double x2 = x * x;
            return x * (p1 + x2 * (p3 + x2 * (p5 + x2 * (p7 + x2 * p9))));
        }
        #endregion
    }
}
