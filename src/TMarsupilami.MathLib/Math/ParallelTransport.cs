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
        #region VECTOR
        #endregion

        #region FRAME

        /// <summary>
        /// Parallel transports a frame from its origin point and ZAxis to a target point and a target direction.
        /// Use the rotation method.
        /// WARNING : seems unable to preserve the frame orthonormality after few iterations unless t=d1xd2 is properly normalized after each iteration.
        /// The double reflection method does not suffer from this problem.
        /// ------------------------------------
        /// add  | sub   | mul   | div   | sqrt
        /// 11   | 9     | 30    | 1     | 0
        /// ------------------------------------ 
        /// cost = 150
        /// </summary>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toZDir">The target direction (must be a unit vector).</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void ZPT_Rotation(MFrame frame, MPoint toPoint, MVector toZDir, ref MFrame framePT)
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

            var t1 = frame.ZAxis; // Warning, this operation hiddes a CrossProduct.

            // This is necessary to not propagate numerical errors
            // if ommitted, after few recursive iterations the parallel transported frame is no more orthonormal and 
            // things get wierd
            // the double reflection method seems to be a lot more stable regarding this problem.
            t1.Normalize();
            
            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var k = MVector.CrossProduct(t1, toZDir);

            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            var c = MVector.DotProduct(t1, toZDir);

            // add :  3 | sub :  0 | mul :  3 | div :  1 | sqrt :  0
            var d1 = frame.XAxis;
            var m = MVector.DotProduct(k, d1) / (1 + c);

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            var d1_para = c * d1; // 3mul
            d1_para += MVector.CrossProduct(k, d1); // 3add 3sub 6mul
            d1_para += m * k; // 3add + 3mul

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var d2_para = MVector.CrossProduct(toZDir, d1_para);

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }

        /// <summary>
        /// Parallel transports a frame from its origin point and ZAxis to a target point and a target direction.
        /// Use the double reflection method.
        /// ------------------------------------
        /// add  | sub   | mul   | div   | sqrt
        /// 10   | 18    | 33    | 2     | 0
        /// ------------------------------------  
        /// cost = 180
        /// </summary>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toZDir">The target direction (must be a unit vector).</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void ZPT_Reflection(MFrame frame, MPoint toPoint, MVector toZDir, ref MFrame framePT)
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

            MVector d1_star, d1_para, d2_para;
            MVector t_star;

            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            var e1 = new MVector(toPoint.X - frame.Origin.X, toPoint.Y - frame.Origin.Y, toPoint.Z - frame.Origin.Z);
            double l1_2 = MVector.DotProduct(e1, e1);

            // add :  4 | sub :  6 | mul :  14 | div :  1 | sqrt :  0
            var t1 = frame.ZAxis; // Warning, this operation hiddes a CrossProduct.
            var d1 = frame.XAxis;
            double c1 = 2 / l1_2; // 1div
            d1_star = d1 - (c1 * MVector.DotProduct(d1, e1)) * e1; // 2add 3sub 7mul
            t_star = t1 - (c1 * MVector.DotProduct(t1, e1)) * e1;  // 2add 3sub 7mul

            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            var e2 = toZDir - t_star;
            double l2_2 = MVector.DotProduct(e2, e2);

            // add :  2 | sub :  3 | mul :  7 | div :  1 | sqrt :  0
            double c2 = 2 / l2_2;  // 1div
            d1_para = d1_star - (c2 * MVector.DotProduct(d1_star, e2)) * e2; // 2add 3sub 7mul

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            d2_para = MVector.CrossProduct(toZDir, d1_para);

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }

        /// <summary>
        /// Parallel transports a frame from its origin point and a given intial direction to a target point and a target direction.
        /// Use the rotation method.
        /// ------------------------------------
        /// add  | sub   | mul   | div   | sqrt
        /// 19   | 9     | 41    | 1     | 0
        /// ------------------------------------
        /// cost = 202
        /// </summary>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="fromDir">The initial direction (must be a unit vector).</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toZDir">The target direction (must be a unit vector).</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void PT_Rotation(MFrame frame, MVector fromDir, MPoint toPoint, MVector toDir, ref MFrame framePT)
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
             * 19   | 9     | 41    | 1     | 0
             * ------------------------------------          
             */

            // add :  0 | sub :  3 | mul :  6 | div :  0 | sqrt :  0
            var k = MVector.CrossProduct(fromDir, toDir);

            // add :  3 | sub :  0 | mul :  3 | div :  1 | sqrt :  0
            var c = MVector.DotProduct(fromDir, toDir);
            var _c = 1 / (1 + c);

            // add :  2 | sub :  0 | mul :  4 | div :  0 | sqrt :  0
            var d1 = frame.XAxis;
            var m1 = MVector.DotProduct(k, d1) * _c;

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            var d1_para = c * d1; // 3mul
            d1_para += MVector.CrossProduct(k, d1); // 3add 3sub 6mul
            d1_para += m1 * k; // 3add 3mul

            // add :  2 | sub :  0 | mul :  4 | div :  0 | sqrt :  0
            var d2 = frame.YAxis;
            var m2 = MVector.DotProduct(k, d2) * _c;

            // add :  6 | sub :  3 | mul : 12 | div :  0 | sqrt :  0
            var d2_para = c * d2; // 3mul
            d2_para += MVector.CrossProduct(k, d2); // 3add 3sub 6mul
            d2_para += m2 * k; // 3add 3mul

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }

        /// <summary>
        /// Parallel transports a frame from its origin point and a given intial direction to a target point and a target direction.
        /// Use the double reflection method.
        /// ------------------------------------
        /// add  | sub   | mul   | div   | sqrt
        /// 12   | 21    | 40    | 2     | 0
        /// ------------------------------------     
        /// cost = 213
        /// </summary>
        /// <param name="frame">The frame to be parallel tansported.</param>
        /// <param name="fromDir">The initial direction (must be a unit vector).</param>
        /// <param name="toPoint">The target point.</param>
        /// <param name="toZDir">The target direction (must be a unit vector).</param>
        /// <param name="framePT">The parallel transported frame.</param>
        public static void PT_Reflection(MFrame frame, MVector fromDir, MPoint toPoint, MVector toDir, ref MFrame framePT)
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

            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            var e1 = new MVector(toPoint.X - frame.Origin.X, toPoint.Y - frame.Origin.Y, toPoint.Z - frame.Origin.Z);
            double l1_2 = MVector.DotProduct(e1, e1);

            // add :  4 | sub :  6 | mul : 14 | div :  1 | sqrt :  0
            var d1 = frame.XAxis;
            var d2 = frame.YAxis;   
            double c1 = 2 / l1_2; // 1div
            var t_star = fromDir - (c1 * MVector.DotProduct(fromDir, e1)) * e1; // 2add 3sub 7mul
            var d1_star = d1 - (c1 * MVector.DotProduct(d1, e1)) * e1;          // 2add 3sub 7mul

            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            var e2 = toDir - t_star;
            double l2_2 = MVector.DotProduct(e2, e2);

            // add :  4 | sub :  9 | mul : 20 | div :  1 | sqrt :  0
            double c2 = 2 / l2_2; // 1div
            var t_para = t_star - (c2 * MVector.DotProduct(t_star, e2)) * e2;      // 2add 3sub 7mul
            var d1_para = d1_star - (c2 * MVector.DotProduct(d1_star, e2)) * e2;    // 2add 3sub 7mul
            var d2_para = MVector.CrossProduct(t_para, d1_para);                    //      3sub 6mul

            framePT.Origin = toPoint;
            framePT.XAxis = d1_para;
            framePT.YAxis = d2_para;
        }
        
        #endregion

    }
}
