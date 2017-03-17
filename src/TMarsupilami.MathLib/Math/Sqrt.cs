using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{

    public static partial class Sqrt
    {
        /// <summary>
        /// Polynomial approximation of order 2 of 1/sqrt(x) over I = [0.9 ; 1.1].
        /// Use it to renormalize a vector that is almost already normalized |u| in I.
        /// Somme Newton steps can be added to increase accuracy.
        /// </summary>
        /// <remarks>
        /// P has the form 1 + (1-x) * Q(x) | Q(x) = q0 + q1 * x.
        /// This ensure P(1) = 1.
        /// P is given by a Remez optimization. The global error over I is : err = 1.5779393878238536512e-4
        /// </remarks>
        /// <param name="x">Input number in I = [0.9 ; 1.1]</param>
        /// <returns>An approximation of 1/sqrt(x).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double InvSqrt_R2(double x)
        {
            // add :  2
            // mul :  2

            double p0 = 1.87932901308411809475;
            double p1 = -1.25708614483672273372;
            double p2 = 0.37775713175260463897;

            double y = p0 + x * (p1 + x * p2);
            return y;
        }

        /// <summary>
        /// Combined approximation of 1/sqrt(x).
        /// Uses a polynomial approximation of order 2 over I = [0.9 ; 1.1] and 1 Newton step.
        /// </summary>
        /// <param name="x">Input number in I = [0.9 ; 1.1]</param>
        /// <returns>An approximation of 1/sqrt(x).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double InvSqrt_R2_N1(double x)
        {
            // add :  2
            // sub :  1
            // mul :  5

            double y;
            double x2 = 0.5 * x;

            y = InvSqrt_R2(x);          // Remez approximation
            y = y * (1.5 - x2 * y * y); // Newton step 1
            return y;
        }

        /// <summary>
        /// Combined approximation of 1/sqrt(x).
        /// Uses a polynomial approximation of order 2 over I = [0.9 ; 1.1] and 2 Newton steps.
        /// </summary>
        /// <param name="x">Input number in I = [0.9 ; 1.1]</param>
        /// <returns>An approximation of 1/sqrt(x).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double InvSqrt_R2_N2(double x)
        {
            // add :  2
            // sub :  2
            // mul :  8

            double y;
            double x2 = 0.5 * x;

            y = InvSqrt_R2(x);          // Remez approximation
            y = y * (1.5 - x2 * y * y); // Newton step 1
            y = y * (1.5 - x2 * y * y); // Newton step 2
            return y;
        }

    }
}
