using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class ParallelTransport
    {
        /// COST MODEL
        /// add  : 1
        /// sub  : 1
        /// mul  : 4
        /// div  : 10
        /// sqrt :
        /// cos  :
        /// sin  :
        /// tan  :

        #region VECTOR

        /// <summary>
        /// Parallel transports a vector using the rotation method (cost index = 123).
        /// </summary>
        /// <remarks>
        /// </remarks>
        /// <param name="vector">The vector to be parallel tansported.</param>
        /// <param name="fromUDir">The initial direction (must be a unit vector).</param>
        /// <param name="toUDir">The target direction (must be a unit vector).</param>
        /// <param name="vectorPT">The parallel transported vector.</param>
        public static void PT_Rotation(MVector vector, MVector fromUDir, MVector toUDir, ref MVector vectorPT)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromUDir and toUDir must be of unit length
             * - aligning t1 to t2 requiers a rotation of alpha in [0, pi] around t1 x t2
             * 
             * ------------------------------------
             * NOTES : rotation method
             * ------------------------------------ 
             * Here we consider the case where θ = α in Rodrigues' rotation formula :
             *  α : angle between t1 and t2(sin(α) = |t1 x t2| where |t1|=1 |t2|=1)
             *  θ : rotation angle to apply around k = t1 x t2
             *  b : k/|k| unit vector of rotation axis
             * 
             * Thus, the rotated vector (v') is given by :
             * v' = v.cos(θ) + (b x v).sin(θ) + b(b.v).(1-cos(θ))
             * v' = v + (b x v).sin(θ) + b x (b x v).(1-cos(θ))
             * 
             * We use the equivalent but more computationally efficient form (c = cos(α) and c <> -1) :
             * v' = c.v + (k x v) + (k.v)/(1+c) k
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 11   | 6     | 24    | 1     | 0
             * ------------------------------------          
             */

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var k = MVector.CrossProduct(fromUDir, toUDir);

            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            var c = MVector.DotProduct(fromUDir, toUDir);

            // add :  3 | sub :  0 | mul :  3 | div :  1 | sqrt :  0
            var m = MVector.DotProduct(k, vector) / (1 + c);

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            vectorPT = c * vector; // 3mul
            vectorPT += MVector.CrossProduct(k, vector); // 3add 3sub 6mul
            vectorPT += m * k; // 3add 3mul
        }

        /// <summary>
        /// Parallel transports 2 vectors using the rotation method (cost index = 204).
        /// </summary>
        /// <remarks>
        /// </remarks>
        /// <param name="vector_1">The first vector to be parallel tansported.</param>
        /// <param name="vector_2">The second vector to be parallel tansported.</param>
        /// <param name="fromPoint">The initial point.</param>
        /// <param name="fromUDir">The initial direction (must be a unit vector).</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toUDir">The target direction (must be a unit vector).</param>
        /// <param name="vectorPT_1">The first parallel transported vector.</param>
        /// <param name="vectorPT_2">The second parallel transported vector.</param>
        public static void PT_Rotation(MVector vector_1, MVector vector_2, MPoint fromPoint, MVector fromUDir, MPoint toPoint, MVector toUDir, ref MVector vectorPT_1, ref MVector vectorPT_2)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromUDir and toUDir must be of unit length
             * - aligning t1 to t2 requiers a rotation of alpha in [0, pi] around t1 x t2
             * 
             * ------------------------------------
             * NOTES : rotation method
             * ------------------------------------ 
             * Here we consider the case where θ = α in Rodrigues' rotation formula :
             *  α : angle between t1 and t2(sin(α) = |t1 x t2| where |t1|=1 |t2|=1)
             *  θ : rotation angle to apply around k = t1 x t2
             *  b : k/|k| unit vector of rotation axis
             * 
             * Thus, the rotated vector (v') is given by :
             * v' = v.cos(θ) + (b x v).sin(θ) + b(b.v).(1-cos(θ))
             * v' = v + (b x v).sin(θ) + b x (b x v).(1-cos(θ))
             * 
             * We use the equivalent but more computationally efficient form (c = cos(α) and c <> -1) :
             * v' = c.v + (k x v) + (k.v)/(1+c) k
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 19   | 9     | 39    | 2     | 0
             * ------------------------------------          
             */

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var k = MVector.CrossProduct(fromUDir, toUDir);

            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            var c = MVector.DotProduct(fromUDir, toUDir);

            // add :  5 | sub :  0 | mul :  6 | div :  2 | sqrt :  0
            var n = (1 + c);
            var m1 = MVector.DotProduct(k, vector_1) / n;
            var m2 = MVector.DotProduct(k, vector_2) / n;

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            vectorPT_1 = c * vector_1; // 3mul
            vectorPT_1 += MVector.CrossProduct(k, vector_1); // 3add 3sub 6mul
            vectorPT_1 += m1 * k; // 3add 3mul

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            vectorPT_2 = c * vector_2; // 3mul
            vectorPT_2 += MVector.CrossProduct(k, vector_2); // 3add 3sub 6mul
            vectorPT_2 += m2 * k; // 3add 3mul
        }

        /// <summary>
        /// Parallel transports a vector using the double reflection method (cost index = 141).
        /// </summary>
        /// <param name="vector">The vector to be parallel tansported.</param>
        /// <param name="fromPoint">The initial point.</param>
        /// <param name="fromUDir">The initial direction (must be a unit vector).</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toUDir">The target direction (must be a unit vector).</param>
        /// <param name="vectorPT">The parallel transported vector.</param>
        public static void PT_Reflection(MVector vector, MPoint fromPoint, MVector fromUDir, MPoint toPoint, MVector toUDir, ref MVector vectorPT)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromDir and toDir must be of unit length
             * - (frame.Origin - toPoint- must be <> 0
             * - (t*-t2) must be <> 0
             * 
             * ------------------------------------
             * NOTES : double reflection method
             * ------------------------------------ 
             * Here we considere the double reflection method with :
             *  n1 : the first reflection unit normal vector, n1 = u1 = e1/|e1|
             *  n2 : the second reflection unit normal vector, n2 = (t2-t1*)/|t2-t1*|
             * 
             * Thus, the first reflected frame (*) is given by :
             * t*   = t1 - 2(n1.t1) * n1
             * d1*  = d1 - 2(n1*d1) * n1
             * d2*  = d2 - 2(n1*d2) * n1 = d1* x t*
             * 
             * Thus, the second reflected frame (//) is given by :
             * t_para = t2
             * d1_para = d1* - 2(n2.d1*) * n2
             * d2_para = d2* - 2(n2.d2*) * n2 = t2 x d1_para
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 10   | 15    | 27    | 2     | 0
             * ------------------------------------          
             */

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N1 = new MVector(toPoint.X - fromPoint.X, toPoint.Y - fromPoint.Y, toPoint.Z - fromPoint.Z);
            double c1 = 2 / MVector.DotProduct(N1, N1);

            // add :  4 | sub :  6 | mul : 14 | div :  0 | sqrt :  0
            var t_star = fromUDir - (c1 * MVector.DotProduct(fromUDir, N1)) * N1; // 2add 3sub 7mul
            var vector_star = vector - (c1 * MVector.DotProduct(vector, N1)) * N1; // 2add 3sub 7mul

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N2 = toUDir - t_star;
            double c2 = 2 / MVector.DotProduct(N2, N2);

            // add :  2 | sub :  3 | mul :  7 | div :  0 | sqrt :  0
            vectorPT = vector_star - (c2 * MVector.DotProduct(vector_star, N2)) * N2; // 2add 3sub 7mul
        }

        /// <summary>
        /// Parallel transports 2 vectors using the double reflection method (cost index = 219).
        /// </summary>
        /// <param name="vector_1">The first vector to be parallel tansported.</param>
        /// <param name="vector_2">The second vector to be parallel tansported.</param>
        /// <param name="fromPoint">The initial point.</param>
        /// <param name="fromUDir">The initial direction (must be a unit vector).</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toUDir">The target direction (must be a unit vector).</param>
        /// <param name="vectorPT_1">The first parallel transported vector.</param>
        /// <param name="vectorPT_2">The second parallel transported vector.</param>
        public static void PT_Reflection(MVector vector_1, MVector vector_2, MPoint fromPoint, MVector fromUDir, MPoint toPoint, MVector toUDir, ref MVector vectorPT_1, ref MVector vectorPT_2)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromDir and toDir must be of unit length
             * - (frame.Origin - toPoint- must be <> 0
             * - (t*-t2) must be <> 0
             * 
             * ------------------------------------
             * NOTES : double reflection method
             * ------------------------------------ 
             * Here we considere the double reflection method with :
             *  n1 : the first reflection unit normal vector, n1 = u1 = e1/|e1|
             *  n2 : the second reflection unit normal vector, n2 = (t2-t1*)/|t2-t1*|
             * 
             * Thus, the first reflected frame (*) is given by :
             * t*   = t1 - 2(n1.t1) * n1
             * d1*  = d1 - 2(n1*d1) * n1
             * d2*  = d2 - 2(n1*d2) * n1 = d1* x t*
             * 
             * Thus, the second reflected frame (//) is given by :
             * t_para = t2
             * d1_para = d1* - 2(n2.d1*) * n2
             * d2_para = d2* - 2(n2.d2*) * n2 = t2 x d1_para
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 14   | 21    | 41    | 2     | 0
             * ------------------------------------          
             */

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N1 = new MVector(toPoint.X - fromPoint.X, toPoint.Y - fromPoint.Y, toPoint.Z - fromPoint.Z);
            double c1 = 2 / MVector.DotProduct(N1, N1);

            // add :  6 | sub :  9 | mul : 21 | div :  0 | sqrt :  0
            var t_star = fromUDir - (c1 * MVector.DotProduct(fromUDir, N1)) * N1; // 2add 3sub 7mul
            var vector_1_star = vector_1 - (c1 * MVector.DotProduct(vector_1, N1)) * N1; // 2add 3sub 7mul
            var vector_2_star = vector_2 - (c1 * MVector.DotProduct(vector_2, N1)) * N1; // 2add 3sub 7mul

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N2 = toUDir - t_star;
            double c2 = 2 / MVector.DotProduct(N2, N2);

            // add :  4 | sub :  6 | mul : 14 | div :  0 | sqrt :  0
            vectorPT_1 = vector_1_star - (c2 * MVector.DotProduct(vector_1_star, N2)) * N2; // 2add 3sub 7mul
            vectorPT_2 = vector_2_star - (c2 * MVector.DotProduct(vector_2_star, N2)) * N2; // 2add 3sub 7mul
        }

        #endregion

        #region FRAME

        /// <summary>
        /// Parallel transports a frame along its ZAxis using the rotation method (cost index = 150).
        /// </summary>
        /// <remarks>
        /// WARNING : seems unable to preserve the frame orthonormality after few iterations unless t=d1xd2 is properly normalized after each iteration.
        /// The double reflection method does not suffer from this problem.
        /// </remarks>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="fromZUDir">The intial ZAxis direction. Must the unit ZAxis of the frame parameter.</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toZUDir">The target direction. Must be a unit vector.</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void ZPT_Rotation(MFrame frame, MVector fromZUDir, MPoint toPoint, MVector toZUDir, ref MFrame framePT)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromDir and toDir must be of unit length
             * - aligning t1 to t2 requiers a rotation of alpha in [0, pi] around t1 x t2
             * 
             * ------------------------------------
             * NOTES : rotation method
             * ------------------------------------ 
             * Here we consider the case where θ = α in Rodrigues' rotation formula :
             *  α : angle between t1 and t2(sin(α) = |t1 x t2| where |t1|=1 |t2|=1)
             *  θ : rotation angle to apply around k = t1 x t2
             *  b : k/|k| unit vector of rotation axis
             * 
             * Thus, the rotated vector (v') is given by :
             * v' = v.cos(θ) +  (b x v).sin(θ) + b(b.v).(1-cos(θ))
             * v' = v + (b x v).sin(θ) + b x (b x v).(1-cos(θ))
             * 
             * We use the equivalent but more computationally efficient form (c = cos(θ) and c <> -1) :
             * v' = c.v + (k x v) + (k.v)/(1+c) k
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 11   | 9     | 30    | 1     | 0
             * ------------------------------------          
             */
            
            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var k = MVector.CrossProduct(fromZUDir, toZUDir);

            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            var c = MVector.DotProduct(fromZUDir, toZUDir);

            // add :  3 | sub :  0 | mul :  3 | div :  1 | sqrt :  0
            var d1 = frame.XAxis;
            var m = MVector.DotProduct(k, d1) / (1 + c);

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            var d1_para = c * d1; // 3mul
            d1_para += MVector.CrossProduct(k, d1); // 3add 3sub 6mul
            d1_para += m * k; // 3add + 3mul

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var d2_para = MVector.CrossProduct(toZUDir, d1_para);

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }

        /// <summary>
        /// Parallel transports a frame along its ZAxis using the double reflection method (cost index = 180).
        /// </summary>
        /// <remarks>
        /// </remarks>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="fromPoint">The initial point.</param>
        /// <param name="fromZUDir">The intial ZAxis direction. Must be the unit ZAxis of the frame parameter.</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toZUDir">The target direction. Must be a unit vector.</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void ZPT_Reflection(MFrame frame, MPoint fromPoint, MVector fromZUDir, MPoint toPoint, MVector toZUDir, ref MFrame framePT)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromDir and toDir must be of unit length
             * - (frame.Origin - toPoint- must be <> 0
             * - (t*-t2) must be <> 0
             *  
             * ------------------------------------
             * NOTES : double reflection method
             * ------------------------------------ 
             * Here we considere the double reflection method with :
             *  n1 : the first reflection unit normal vector, n1 = u1 = e1/|e1|
             *  n2 : the second reflection unit normal vector, n2 = (t2-t1*)/|t2-t1*|
             * 
             * Thus, the first reflected frame (*) is given by :
             * t*   = t1 - 2(n1.t1) * n1
             * d1*  = d1 - 2(n1*d1) * n1
             * d2*  = d2 - 2(n1*d2) * n1 = d1* x t*
             * 
             * Thus, the second reflected frame (//) is given by :
             * t_para = t2
             * d1_para = d1* - 2(n2.d1*) * n2
             * d2_para = d2* - 2(n2.d2*) * n2 = t2 x d1_para
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 10   | 18    | 33    | 2     | 0
             * ------------------------------------          
             */

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N1 = new MVector(toPoint.X - fromPoint.X, toPoint.Y - fromPoint.Y, toPoint.Z - fromPoint.Z);
            double c1 = 2 / MVector.DotProduct(N1, N1);

            // add :  4 | sub :  6 | mul :  14 | div :  0 | sqrt :  0
            var d1 = frame.XAxis;
            var t_star = fromZUDir - (c1 * MVector.DotProduct(fromZUDir, N1)) * N1; // 2add 3sub 7mul
            var d1_star = d1 - (c1 * MVector.DotProduct(d1, N1)) * N1;              // 2add 3sub 7mul

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N2 = toZUDir - t_star;
            double c2 = 2 / MVector.DotProduct(N2, N2);

            // add :  2 | sub :  3 | mul :  7 | div :  0 | sqrt :  0
            var d1_para = d1_star - (c2 * MVector.DotProduct(d1_star, N2)) * N2; // 2add 3sub 7mul

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var d2_para = MVector.CrossProduct(toZUDir, d1_para);

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }

        /// <summary>
        /// Parallel transports a frame using the rotation method (cost index = 204).
        /// </summary>
        /// <remarks>
        /// </remarks>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="fromUDir">The initial direction. Must be a unit vector.</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toUDir">The target direction. Must be a unit vector.</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void PT_Rotation(MFrame frame, MVector fromUDir, MPoint toPoint, MVector toUDir, ref MFrame framePT)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromDir and toDir must be of unit length
             * - aligning t1 to t2 requiers a rotation of alpha in [0, pi] around t1 x t2
             * 
             * ------------------------------------
             * NOTES : rotation method
             * ------------------------------------ 
             * Here we consider the case where θ = α in Rodrigues' rotation formula :
             *  α : angle between t1 and t2(sin(α) = |t1 x t2| where |t1|=1 |t2|=1)
             *  θ : rotation angle to apply around k = t1 x t2
             *  b : k/|k| unit vector of rotation axis
             * 
             * Thus, the rotated vector (v') is given by :
             * v' = v.cos(θ) +  (b x v).sin(θ) + b(b.v).(1-cos(θ))
             * v' = v + (b x v).sin(θ) + b x (b x v).(1-cos(θ))
             * 
             * We use the equivalent but more computationally efficient form (c = cos(θ) and c <> -1) :
             * v' = c.v + (k x v) + (k.v)/(1+c) k
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 19   | 9     | 39    | 2     | 0
             * ------------------------------------          
             */

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var k = MVector.CrossProduct(fromUDir, toUDir);

            // add :  3 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            var c = MVector.DotProduct(fromUDir, toUDir);
            var n = 1 + c;

            // add :  2 | sub :  0 | mul :  3 | div :  1 | sqrt :  0
            var d1 = frame.XAxis;
            var m1 = MVector.DotProduct(k, d1) / n;

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            var d1_para = c * d1; // 3mul
            d1_para += MVector.CrossProduct(k, d1); // 3add 3sub 6mul
            d1_para += m1 * k; // 3add 3mul

            // add :  2 | sub :  0 | mul :  3 | div :  1 | sqrt :  0
            var d2 = frame.YAxis;
            var m2 = MVector.DotProduct(k, d2) / n;

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            var d2_para = c * d2; // 3mul
            d2_para += MVector.CrossProduct(k, d2); // 3add 3sub 6mul
            d2_para += m2 * k; // 3add 3mul

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }

        /// <summary>
        /// Parallel transports a frame using the double reflection method (cost index = 213).
        /// </summary>
        /// <remarks>
        /// </remarks>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="fromUDir">The initial direction (must be a unit vector).</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toUDir">The target direction (must be a unit vector).</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void PT_Reflection(MFrame frame, MPoint fromPoint, MVector fromUDir, MPoint toPoint, MVector toUDir, ref MFrame framePT)
        {
            /* ------------------------------------
             * WARNING
             * ------------------------------------
             * - fromDir and toDir must be of unit length
             * - (frame.Origin - toPoint- must be <> 0
             * - (t*-t2) must be <> 0
             * 
             * ------------------------------------
             * NOTES : double reflection method
             * ------------------------------------ 
             * Here we considere the double reflection method with :
             *  n1 : the first reflection unit normal vector, n1 = u1 = e1/|e1|
             *  n2 : the second reflection unit normal vector, n2 = (t2-t1*)/|t2-t1*|
             * 
             * Thus, the first reflected frame (*) is given by :
             * t*   = t1 - 2(n1.t1) * n1
             * d1*  = d1 - 2(n1*d1) * n1
             * d2*  = d2 - 2(n1*d2) * n1 = d1* x t*
             * 
             * Thus, the second reflected frame (//) is given by :
             * t_para = t2
             * d1_para = d1* - 2(n2.d1*) * n2
             * d2_para = d2* - 2(n2.d2*) * n2 = t2 x d1_para
             * 
             * REFERENCE :
             * Wang, W., Jüttler, B., Zheng, D., &Liu, Y. (2008). Computation of rotation minimizing frames. ACM Transactions on Graphics.
             * 
             * ------------------------------------
             * TOTAL COST
             * ------------------------------------
             * add  | sub   | mul   | div   | sqrt
             * 12   | 21    | 40    | 2     | 0
             * ------------------------------------          
             */

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N1 = new MVector(toPoint.X - fromPoint.X, toPoint.Y - fromPoint.Y, toPoint.Z - fromPoint.Z);
            double c1 = 2 / MVector.DotProduct(N1, N1);

            // add :  4 | sub :  6 | mul : 14 | div :  0 | sqrt :  0
            var d1 = frame.XAxis;
            var t_star = fromUDir - (c1 * MVector.DotProduct(fromUDir, N1)) * N1;   // 2add 3sub 7mul
            var d1_star = d1 - (c1 * MVector.DotProduct(d1, N1)) * N1;              // 2add 3sub 7mul

            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  0
            var N2 = toUDir - t_star;
            double c2 = 2 / MVector.DotProduct(N2, N2);

            // add :  4 | sub :  6 | mul : 14 | div :  0 | sqrt :  0
            var t_para = t_star - (c2 * MVector.DotProduct(t_star, N2)) * N2;      // 2add 3sub 7mul
            var d1_para = d1_star - (c2 * MVector.DotProduct(d1_star, N2)) * N2;   // 2add 3sub 7mul

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var d2_para = MVector.CrossProduct(t_para, d1_para);

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }
        
        #endregion

    }
}
