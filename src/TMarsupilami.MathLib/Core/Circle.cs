using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class Circle
    {
        #region CIRCUMSCRIBED

        public static void CircumscribedCircle_Current(Point ps, Point p, Point pe, out double κ, out Vector κb, out Vector ts, out Vector t, out Vector te, out double fs, out double f, out double fe)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add : 16 | sub :  15 | mul : 46 | div :  3 | sqrt :  2
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  5 | sub :  0 | mul :  6 | div :  2 | sqrt :  2
             *      acos : 2
             *     
             */
           
            // e1 = p - ps
            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            double e1_x = p.X - ps.X;
            double e1_y = p.Y - ps.Y;
            double e1_z = p.Z - ps.Z;
            double l1_2 = e1_x * e1_x + e1_y * e1_y + e1_z * e1_z;

            // e2 = pe - p
            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            double e2_x = pe.X - p.X;
            double e2_y = pe.Y - p.Y;
            double e2_z = pe.Z - p.Z;
            double l2_2 = e2_x * e2_x + e2_y * e2_y + e2_z * e2_z;

            // e = e1 + e2 = pe - ps
            // add :  3 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            double e_x = e1_x + e2_x;
            double e_y = e1_y + e2_y;
            double e_z = e1_z + e2_z;
            double l_2 = e_x * e_x + e_y * e_y + e_z * e_z;

            // d = 2/|e1||e2||e1+e2|
            // add :  0 | sub :  0 | mul :  2 | div :  1 | sqrt :  1
            double ll = l1_2 * l2_2; // if ll = 0 => it's a point
            double d = 2 / System.Math.Sqrt(ll * l_2);

            // κb = (2 / |e1||e2||e1+e2|) * e1 x e2
            // add :  0 | sub :  3 | mul :  9 | div :  0 | sqrt :  0
            double κb_x = d * (e1_y * e2_z - e1_z * e2_y);
            double κb_y = d * (e1_z * e2_x - e1_x * e2_z);
            double κb_z = d * (e1_x * e2_y - e1_y * e2_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = System.Math.Sqrt(κ_2);

            // t = (l2 / (l1 * l)) * e1 + (l1 / (l2 * l)) * e2 = (l2 / l) * u1 + (l1 / l) * u2
            // add :  3 | sub :  0 | mul :  9 | div :  0 | sqrt :  0
            double c = 0.5 * d;
            double c1 = c * l2_2;
            double c2 = c * l1_2;
            double t_x = c1 * e1_x + c2 * e2_x;
            double t_y = c1 * e1_y + c2 * e2_y;
            double t_z = c1 * e1_z + c2 * e2_z;
            t = new Vector(t_x, t_y, t_z);

            // t1 = 2 * (t.u1) u1 - t
            // add :  2 | sub :  3 | mul :  7 | div :  1 | sqrt :  0
            double f1 = 2 * (t_x * e1_x + t_y * e1_y + t_z * e1_z) / l1_2;
            double t1_x = f1 * e1_x - t_x;
            double t1_y = f1 * e1_y - t_y;
            double t1_z = f1 * e1_z - t_z;
            ts = new Vector(t1_x, t1_y, t1_z);

            // t1 = 2 * (t.u2) u2 - t
            // add :  2 | sub :  3 | mul :  7 | div :  1 | sqrt :  0
            double f2 = 2 * (t_x * e2_x + t_y * e2_y + t_z * e2_z) / l2_2;
            double t2_x = f2 * e2_x - t_x;
            double t2_y = f2 * e2_y - t_y;
            double t2_z = f2 * e2_z - t_z;
            te = new Vector(t2_x, t2_y, t2_z);

            // turning angle (very costly)
            // add :  5 | sub :  0 | mul :  6 | div :  2 | sqrt :  2
            // cos :  0 | sin :  0 | tan :  0 | acos : 2 | asin :  0 | atan :  0
            double cs = (e1_x * t_x + e1_y * t_y + e1_z * t_z) / System.Math.Sqrt(l1_2);
            fs = System.Math.Acos(cs);
            double ce = (e2_x * t_x + e2_y * t_y + e2_z * t_z) / System.Math.Sqrt(l2_2);
            fe = System.Math.Acos(ce);
            f = fs + fe;
        }
        public static void CircumscribedCircle_Current_bis(Point ps, Point p, Point pe, out double κ, out Vector κb, out Vector ts, out Vector t, out Vector te, out double fs, out double f, out double fe)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add : 18 | sub :  15 | mul : 47 | div :  4 | sqrt :  4
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  5 | sub :  0 | mul :  6 | div :  0 | sqrt :  0
             *      acos : 2
             *     
             */

            //e1 = p - ps
            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  1
            double e1_x = p.X - ps.X;
            double e1_y = p.Y - ps.Y;
            double e1_z = p.Z - ps.Z;
            double l1_2 = e1_x * e1_x + e1_y * e1_y + e1_z * e1_z;
            double _l1 = 1/Math.Sqrt(l1_2);

            // u1 = e1 / |e1|
            // add :  0 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            double u1_x = _l1 * e1_x;
            double u1_y = _l1 * e1_y;
            double u1_z = _l1 * e1_z;

            // e2 = pe - p
            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  1
            double e2_x = pe.X - p.X;
            double e2_y = pe.Y - p.Y;
            double e2_z = pe.Z - p.Z;
            double l2_2 = e2_x * e2_x + e2_y * e2_y + e2_z * e2_z;
            double _l2 = Math.Sqrt(l2_2);

            // u2 = e2 / |e2|
            // add :  0 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            double u2_x = _l2 * e2_x;
            double u2_y = _l2 * e2_y;
            double u2_z = _l2 * e2_z;

            // e = e1 + e2 = pe - ps
            // add :  5 | sub :  0 | mul :  3 | div :  1 | sqrt :  1
            double e_x = e1_x + e2_x;
            double e_y = e1_y + e2_y;
            double e_z = e1_z + e2_z;
            double l_2 = e_x * e_x + e_y * e_y + e_z * e_z;
            double _l = 1 / Math.Sqrt(l_2);

            // κb = (2 / |e1+e2|) * u1 x u2
            // add :  0 | sub :  3 | mul :  9 | div :  0 | sqrt :  0
            double d = 2 * _l;
            double κb_x = d * (u1_y * u2_z - u1_z * u2_y);
            double κb_y = d * (u1_z * u2_x - u1_x * u2_z);
            double κb_z = d * (u1_x * u2_y - u1_y * u2_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = System.Math.Sqrt(κ_2);

            // t = (l2 / l) * u1 + (l1 / l) * u2
            // add :  3 | sub :  0 | mul :  6 | div :  2 | sqrt :  0
            double c1 = _l / _l2;
            double c2 = _l / _l1;
            double t_x = c1 * e1_x + c2 * e2_x;
            double t_y = c1 * e1_y + c2 * e2_y;
            double t_z = c1 * e1_z + c2 * e2_z;
            t = new Vector(t_x, t_y, t_z);

            // t1 = 2 * (t.u1) u1 - t
            // add :  2 | sub :  3 | mul :  7 | div :  0 | sqrt :  0
            double f1 = 2 * (t_x * u1_x + t_y * u1_y + t_z * u1_z);
            double t1_x = f1 * u1_x - t_x;
            double t1_y = f1 * u1_y - t_y;
            double t1_z = f1 * u1_z - t_z;
            ts = new Vector(t1_x, t1_y, t1_z);

            // t1 = 2 * (t.u2) u2 - t
            // add :  2 | sub :  3 | mul :  7 | div :  0 | sqrt :  0
            double f2 = 2 * (t_x * u2_x + t_y * u2_y + t_z * u2_z);
            double t2_x = f2 * u2_x - t_x;
            double t2_y = f2 * u2_y - t_y;
            double t2_z = f2 * u2_z - t_z;
            te = new Vector(t2_x, t2_y, t2_z);

            // turning angle (very costly)
            // add : 5 | sub : 0 | mul : 6 | div : 0 | sqrt : 0
            // cos : 0 | sin : 0 | tan : 0 | acos : 2 | asin : 0 | atan : 0
            double cs = (u1_x * t_x + u1_y * t_y + u1_z * t_z);
            fs = System.Math.Acos(cs);
            double ce = (u2_x * t_x + u2_y * t_y + u2_z * t_z);
            fe = System.Math.Acos(ce);
            f = fs + fe;
        }
        public static void CircumscribedCircle_Start(Vector ts, Point ps, Point p, out double κ, out Vector κb, out Vector t, out double fs)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add :  6 | sub :  9 | mul : 22 | div :  1 | sqrt :  0
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  2 | sub :  0 | mul :  3 | div :  1 | sqrt :  1
             *      acos : 1
             *     
             */

            // assumes that |ts| = 1

            // e = p - ps
            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            double e_x = p.X - ps.X;
            double e_y = p.Y - ps.Y;
            double e_z = p.Z - ps.Z;
            double l_2 = e_x * e_x + e_y * e_y + e_z * e_z;

            // κb = (2 / |e1||e2||e1+e2|) * e1 x e2
            // add :  0 | sub :  3 | mul :  9 | div :  1 | sqrt :  0
            double d = 2 / l_2;
            double ts_x = ts.X;
            double ts_y = ts.Y;
            double ts_z = ts.Z;
            double κb_x = d * (ts_y * e_z - ts_z * e_y);
            double κb_y = d * (ts_z * e_x - ts_x * e_z);
            double κb_z = d * (ts_x * e_y - ts_y * e_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = System.Math.Sqrt(κ_2);

            // t = 2 * (t.u) u - t
            // add :  2 | sub :  3 | mul :  7 | div :  0 | sqrt :  0
            double ds = d * (ts_x * e_x + ts_y * e_y + ts_z * e_z);
            double t_x = ds * e_x - ts_x;
            double t_y = ds * e_y - ts_y;
            double t_z = ds * e_z - ts_z;
            t = new Vector(t_x, t_y, t_z);

            // turning angle
            // add :  2 | sub :  0 | mul :  3 | div :  1 | sqrt :  1
            // cos :  0 | sin :  0 | tan :  0 | acos : 1 | asin :  0 | atan :  0
            double cs = (e_x * t_x + e_y * t_y + e_z * t_z) / Math.Sqrt(l_2);
            fs = Math.Acos(cs);
        }
        public static void CircumscribedCircle_End(Point p, Point pe, Vector te, out double κ, out Vector κb, out Vector t, out double fe)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add :  6 | sub :  9 | mul : 22 | div :  1 | sqrt :  0
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  2 | sub :  0 | mul :  3 | div :  1 | sqrt :  1
             *      acos : 1
             *     
             */

            // assumes that |te| = 1

            // e = pe - p
            // add :  2 | sub :  3 | mul :  3 | div :  0 | sqrt :  0
            double e_x = pe.X - p.X;
            double e_y = pe.Y - p.Y;
            double e_z = pe.Z - p.Z;
            double l_2 = e_x * e_x + e_y * e_y + e_z * e_z;

            // κb = (2 / |e|) * u x te
            // add :  0 | sub :  3 | mul :  9 | div :  1 | sqrt :  0
            double d = 2 / l_2;
            double te_x = te.X;
            double te_y = te.Y;
            double te_z = te.Z;
            double κb_x = d * (e_y * te_z - e_z * te_y);
            double κb_y = d * (e_z * te_x - e_x * te_z);
            double κb_z = d * (e_x * te_y - e_y * te_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = System.Math.Sqrt(κ_2);

            // t = 2 * (t.u) u - t
            // add :  2 | sub :  3 | mul :  7 | div :  0 | sqrt :  0
            double ds = d * (te_x * e_x + te_y * e_y + te_z * e_z);
            double t_x = ds * e_x - te_x;
            double t_y = ds * e_y - te_y;
            double t_z = ds * e_z - te_z;
            t = new Vector(t_x, t_y, t_z);

            // turning angle
            // add :  2 | sub :  0 | mul :  3 | div :  1 | sqrt :  1
            // cos :  0 | sin :  0 | tan :  0 | acos : 1 | asin :  0 | atan :  0
            double cs = (e_x * t_x + e_y * t_y + e_z * t_z) / Math.Sqrt(l_2);
            fe = Math.Acos(cs);
        }

        #endregion

        #region INSCRIBED

        public static void InscribedCircle_Current(Point ps, Point p, Point pe, out double κ, out Vector κb, out Vector t, out double f)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add : 16 | sub :  15 | mul : 46 | div :  3 | sqrt :  2
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  5 | sub :  0 | mul :  6 | div :  2 | sqrt :  2
             *      acos : 2
             *     
             */

            //e1 = p - ps
            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  1
            double e1_x = p.X - ps.X;
            double e1_y = p.Y - ps.Y;
            double e1_z = p.Z - ps.Z;
            double l1_2 = e1_x * e1_x + e1_y * e1_y + e1_z * e1_z;
            double _l1 = 1 / Math.Sqrt(l1_2);

            // u1 = e1 / |e1|
            // add :  0 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            double u1_x = _l1 * e1_x;
            double u1_y = _l1 * e1_y;
            double u1_z = _l1 * e1_z;

            // e2 = pe - p
            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  1
            double e2_x = pe.X - p.X;
            double e2_y = pe.Y - p.Y;
            double e2_z = pe.Z - p.Z;
            double l2_2 = e2_x * e2_x + e2_y * e2_y + e2_z * e2_z;
            double _l2 = 1 / Math.Sqrt(l2_2);

            // u2 = e2 / |e2|
            // add :  0 | sub :  0 | mul :  3 | div :  0 | sqrt :  0
            double u2_x = _l2 * e2_x;
            double u2_y = _l2 * e2_y;
            double u2_z = _l2 * e2_z;

            // add :  3 | sub :  0 | mul :  4 | div :  1 | sqrt :  0
            double u1u2 = u1_x * u2_x + u1_y * u2_y + u1_z * u2_z;
            double d = (4 * _l1 * _l2) / (_l1 + _l2);

            // κb = 4 / (|e1| + |e2|) * u1 x u2 / (1 + u1.u2) 
            // add :  0 | sub :  3 | mul :  9 | div :  0 | sqrt :  0
            double c = d / (1 + u1u2);
            double κb_x = c * (u1_y * u2_z - u1_z * u2_y);
            double κb_y = c * (u1_z * u2_x - u1_x * u2_z);
            double κb_z = c * (u1_x * u2_y - u1_y * u2_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = System.Math.Sqrt(κ_2);

            // t =  (u1 + u2) / |u1+u1|
            // add :  3 | sub :  0 | mul :  9 | div :  0 | sqrt :  0
            c = 1 / Math.Sqrt(2 * (1 + u1u2));
            double t_x = c * (u1_x + u2_x);
            double t_y = c * (u1_y + u2_y);
            double t_z = c * (u1_z + u2_z);
            t = new Vector(t_x, t_y, t_z);

            f = Math.Acos(u1u2);
        }
        public static void InscribedCircle_Start(Vector ts, Point ps, Point p, out double κ, out Vector κb, out double fs)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add : 16 | sub :  15 | mul : 46 | div :  3 | sqrt :  2
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  5 | sub :  0 | mul :  6 | div :  2 | sqrt :  2
             *      acos : 2
             *     
             */

            //e = p - ps
            // add :  2 | sub :  3 | mul :  3 | div :  1 | sqrt :  1
            double e_x = p.X - ps.X;
            double e_y = p.Y - ps.Y;
            double e_z = p.Z - ps.Z;
            double l_2 = e_x * e_x + e_y * e_y + e_z * e_z;
            double l = Math.Sqrt(l_2);

            // κb = 2/|e| * (ts x e)/(ts.e) 
            // add :  2 | sub :  3 | mul :  13 | div :  2 | sqrt :  0     
            double ts_x = ts.X;
            double ts_y = ts.Y;
            double ts_z = ts.Z;
            double c = (ts_x * e_x + ts_y * e_y + ts_z * e_z) / l;
            double d = 2 / (c * l_2);
            double κb_x = d * (ts_y * e_z - ts_z * e_y);
            double κb_y = d * (ts_z * e_x - ts_x * e_z);
            double κb_z = d * (ts_x * e_y - ts_y * e_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = Math.Sqrt(κ_2);

            // turning angle (very costly)
            // add :  0 | sub :  0 | mul :  0 | div :  0 | sqrt :  0
            // cos :  0 | sin :  0 | tan :  0 | acos : 1 | asin :  0 | atan :  0
            fs = Math.Acos(c);
        }
        public static void InscribedCircle_End(Point p, Point pe, Vector te, out double κ, out Vector κb, out double fe)
        {
            /*
             *  TOTAL COST (without turning angles)
             *      add : 16 | sub :  15 | mul : 46 | div :  3 | sqrt :  2
             * 
             *  TOTAL COST (for the turning angles)
             *      add :  5 | sub :  0 | mul :  6 | div :  2 | sqrt :  2
             *      acos : 2
             *     
             */

            //e = pe - p
            // add :  2 | sub :  3 | mul :  3 | div : 0 | sqrt :  1
            double e_x = pe.X - p.X;
            double e_y = pe.Y - p.Y;
            double e_z = pe.Z - p.Z;
            double l_2 = e_x * e_x + e_y * e_y + e_z * e_z;
            double l = Math.Sqrt(l_2);

            // κb = 2/|e| * (e x te)/(e.te) 
            // add :  2 | sub :  3 | mul :  13 | div :  2 | sqrt :  0     
            double te_x = te.X;
            double te_y = te.Y;
            double te_z = te.Z;
            double c = (te_x * e_x + te_y * e_y + te_z * e_z) / l;
            double d = 2 / (c * l_2);
            double κb_x = d * (e_y * te_z - e_z * te_y);
            double κb_y = d * (e_z * te_x - e_x * te_z);
            double κb_z = d * (e_x * te_y - e_y * te_x);
            κb = new Vector(κb_x, κb_y, κb_z);

            // κ = |κb|
            // add :  2 | sub :  0 | mul :  3 | div :  0 | sqrt :  1
            double κ_2 = κb_x * κb_x + κb_y * κb_y + κb_z * κb_z;
            κ = System.Math.Sqrt(κ_2);

            // turning angle (very costly)
            // add :  0 | sub :  0 | mul :  0 | div :  0 | sqrt :  0
            // cos :  0 | sin :  0 | tan :  0 | acos : 1 | asin :  0 | atan :  0
            fe = Math.Acos(c);
        }


        #endregion

    }
}
