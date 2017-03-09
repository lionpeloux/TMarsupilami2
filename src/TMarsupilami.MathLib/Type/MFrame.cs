using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    /// <summary>
    /// A structure encapsulating a 3D Plane.
    /// It is defined by its Origin point and its XAxis and YAxis vectors.
    /// The plane is supposed orthonormal, that is |XAxis| = |YAxis| = 1 and XAxis.YAxis = 0
    /// The ZAxis is normal to the plane and is computed as XAxis x YAxis.
    /// The ZAxis is never stored in the datastructure to ensure lowmemory usage. 
    /// It is only computed "ondemand" when accessing the "ZAxis property.
    /// </summary>
    public struct MFrame //: IEquatable<Frame>
    {
        #region FIELDS
        
        /// <summary>
        /// The Origin of the frame.
        /// </summary>
        private MPoint origin;

        /// <summary>
        /// The XAxis of the frame.
        /// </summary>
        private MVector xaxis;
        
        /// <summary>
        /// The YAxis of the frame.
        /// </summary>
        private MVector yaxis;
        
        #endregion

        #region CONSTRUCTORS

        /// <summary>
        /// Constructs a frame with the given individual elements.
        /// </summary>
        /// <param name="xaxis">The xaxis vector.</param>
        /// <param name="yaxis">The yaxis vector.</param>
        /// <param name="normalized">If true, frame axis will be normalized</param>
        public MFrame(MPoint origin, MVector xaxis, MVector yaxis, bool normalized = false)
        {
            this.origin = origin;
            this.xaxis = xaxis;
            this.yaxis = yaxis;

            if (normalized)
            {             
                xaxis.Normalize();
                yaxis.Normalize();
            }
        }

        /// <summary>
        /// Constructs a new frame with the given frame.
        /// </summary>
        /// <param name="f">The given frame.</param>
        public MFrame(MFrame f)
        {
            origin = f.origin;
            xaxis = f.xaxis;
            yaxis = f.yaxis;
        }

        #endregion

        #region INSTANCE PROPERTIES

        /// <summary>
        /// Gets or sets the origin point of the frame.
        /// </summary>
        public MPoint Origin
        {
            get { return origin; }
            set { origin = value; }
        }

        /// <summary>
        /// Gets or sets the X axis vector of the frame.
        /// </summary>
        public MVector XAxis
        {
            get { return xaxis; }
            set { xaxis = value; }
        }

        /// <summary>
        /// Gets or sets the Y axis vector of the frame.
        /// </summary>
        public MVector YAxis
        {
            get { return yaxis; }
            set { yaxis = value; }
        }

        /// <summary>
        /// Gets the Z axis or Normal vector of the frame.
        /// </summary>
        public MVector ZAxis
        {
            get
            {
                return MVector.CrossProduct(this.xaxis, this.yaxis);
            }
        }

        #endregion

        #region INSTANCE METHODS

        /// <summary>
        /// Re-normalized this frame in-place.
        /// This ensure the frame is orthonormal assuming it was almost orthonormal before the call.
        /// </summary>
        public void ReNormalize()
        {
#if FAST
            throw new NotImplementedException();
#else       
           // normalize XAxis
            this.XAxis.Normalize();

            // get a normalized ZAxis
            MVector vz = this.ZAxis;
            vz.Normalize();

            // compute a new YAxis
            this.YAxis = MVector.CrossProduct(vz, this.XAxis);
#endif
        }

        public void ZRotate(double θ, ref MFrame frameROT)
        {
            Rotation.ZRotate(this, θ, ref frameROT);
        }
        public void ZRotate(double θ)
        {
            Rotation.ZRotate(this, θ, ref this);
        }

        public void ZDiffRotate(double θ, ref MFrame frameROT)
        {
            Rotation.ZDiffRotate_Taylor_3(this, θ, ref frameROT);
        }
        public void ZDiffRotate(double θ)
        {
            Rotation.ZDiffRotate_Taylor_3(this, θ, ref this);
        }

        public void ZParallelTransport_Rotation(MPoint toPoint, MVector toZDir, ref MFrame framePT)
        {
            ParallelTransport.ZPT_Rotation(this, toPoint, toZDir, ref framePT);
        }
        public void ZParallelTransport_Rotation(MPoint toPoint, MVector toZDir)
        {
            ParallelTransport.ZPT_Rotation(this, toPoint, toZDir, ref this);
        }

        public void ZParallelTransport_Reflection(MPoint toPoint, MVector toZDir, ref MFrame framePT)
        {
            ParallelTransport.ZPT_Reflection(this, toPoint, toZDir, ref framePT);
        }
        public void ZParallelTransport_Reflection(MPoint toPoint, MVector toZDir)
        {
            ParallelTransport.ZPT_Reflection(this, toPoint, toZDir, ref this);
        }

        public void ParallelTransport_Rotation(MVector fromDir, MPoint toPoint, MVector toDir, ref MFrame framePT)
        {
            ParallelTransport.PT_Rotation(this, fromDir, toPoint, toDir, ref framePT);
        }
        public void ParallelTransport_Rotation(MVector fromDir, MPoint toPoint, MVector toDir)
        {
            ParallelTransport.PT_Rotation(this, fromDir, toPoint, toDir, ref this);
        }

        public void ParallelTransport_Reflection(MVector fromDir, MPoint toPoint, MVector toDir, ref MFrame framePT)
        {
            ParallelTransport.PT_Reflection(this, fromDir, toPoint, toDir, ref framePT);
        }
        public void ParallelTransport_Reflection(MVector fromDir, MPoint toPoint, MVector toDir)
        {
            ParallelTransport.PT_Reflection(this, fromDir, toPoint, toDir, ref this);
        }
        #endregion

        #region STATIC PROPERTIES

        /// <summary>
        /// Gets the world XY plane.
        /// </summary>
        public static MFrame XY
        {
            get { return new MFrame(new MPoint(0, 0, 0), MVector.XAxis, MVector.YAxis); }
        }

        /// <summary>
        /// Gets the world YZ plane.
        /// </summary>
        public static MFrame YZ
        {
            get { return new MFrame(new MPoint(0, 0, 0), MVector.YAxis, MVector.ZAxis); }
        }

        /// <summary>
        /// Gets the world ZX plane.
        /// </summary>
        public static MFrame ZX
        {
            get { return new MFrame(new MPoint(0, 0, 0), MVector.ZAxis, MVector.XAxis); }
        }

        #endregion

        #region STATIC METHODS

        /// <summary>
        /// Fast Rotates this frame in place, off a given angle around its z-axis.
        /// </summary>
        /// <param name="angle">Angle of rotation (in radians).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MFrame ZRotate(MFrame f, double angle)
        {
            // fast sin & cos computation
            double s, c;
            Trigo.SinCos(angle, out s, out c);

            // rotate XAxis and store the results in a tmp vector
            MVector xaxis = MVector.LinearComb(c, f.XAxis, s, f.YAxis);
            MVector yaxis = MVector.LinearComb(-s, f.XAxis, c, f.YAxis);
            return new MFrame(f.Origin, xaxis, yaxis);
        }

        /// <summary>
        /// Fast Rotates this frame in place, off a given tuple {sin(x), cos(x)} around its z-axis.
        /// </summary>
        /// <param name="angle">Angle of rotation (in radians).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MFrame ZRotate(MFrame f, double sin, double cos)
        {
            // rotate XAxis and store the results in a tmp vector
            MVector xaxis = MVector.LinearComb(cos, f.XAxis, sin, f.YAxis);
            MVector yaxis = MVector.LinearComb(-sin, f.XAxis, cos, f.YAxis);
            return new MFrame(f.Origin, xaxis, yaxis);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double ZAngle(MFrame f1, MFrame f2)
        {
            return 0;
        }


        //public static MFrame ZParallelTransport_Rotation(MFrame frame, MPoint toPoint, MVector toZDir)
        //{
        //    /*
        //     * WARNING : fromDir and toDir must be of unit length
        //     * 
        //     * aligning t1 to t2 requiers a rotation of alpha in [0, pi] around t1 x t2
        //     * 
        //     * Rodrigues' Formula with θ = α, where :
        //     * α : angle between t1 and t2(sin(α) = |t1 x t2| where |t1|=1 |t2|=1)
        //     * θ : rotation angle to apply around k = t1 x t2
        //     * b : k/|k| unit vector of rotation axis
        //     * v' = v.cos(θ) +  (b x v).sin(θ) + b(b.v).(1-cos(θ))
        //     * v' = v + (b x v).sin(θ) + b x (b x v).(1-cos(θ))
        //     * 
        //     * this is written as (c = cos(θ)) where c <> -1 :
        //     * v' = c.v + (k x v) + (k.v)/(1+c) k
        //     * 
        //     * TOTAL COST
        //     * add  :  6 
        //     * sub  :  6
        //     * mul  : 22
        //     * div  :  1
        //     * sqrt :  0
        //     *     
        //     */

        //    // 3sub 6mul
        //    var t1 = frame.ZAxis;
        //    var k = MVector.CrossProduct(t1, toZDir);

        //    // 2add 3mul
        //    var c = MVector.DotProduct(t1, toZDir);

        //    // 3add 3mul 1div
        //    var d1 = frame.XAxis;
        //    var m = MVector.DotProduct(k, d1) / (1 + c);

        //    // 6add 3sub 12mul
        //    var d1_para = c * d1; // 3mul
        //    d1_para += MVector.CrossProduct(k, d1); // 3add 3sub 6mul
        //    d1_para += m * k; // 3add + 3mul

        //    // 3sub 6mul
        //    var d2_para = MVector.CrossProduct(toZDir, d1_para);

        //    var f_para = new MFrame(toPoint, d1_para, d2_para);
        //    return f_para;
        //}
        //public static MFrame ZParallelTransport_Reflection(MFrame frame, MPoint toPoint, MVector toZDir)
        //{
        //    /*
        //     * WARNING : fromDir and toDir must be of unit length
        //     * 
        //     * Double Reflection Method
        //     * 
        //     * TOTAL COST
        //     * add  :  6 
        //     * sub  :  6
        //     * mul  : 22
        //     * div  :  1
        //     * sqrt :  0
        //     *     
        //     */

        //    MVector d1_star, d1_para, d2_para;
        //    MVector t_star;

        //    // 2add 3sub 3mul
        //    var e1 = new MVector(toPoint.X - frame.Origin.X, toPoint.Y - frame.Origin.Y, toPoint.Z - frame.Origin.Z);
        //    double l1_2 = MVector.DotProduct(e1, e1);

        //    var t1 = frame.ZAxis;
        //    var d1 = frame.XAxis;
        //    if (l1_2 == 0)
        //    {
        //        d1_star = d1;
        //        t_star = t1;
        //    }
        //    else
        //    {
        //        // 2add 3sub 7mul 1diuv
        //        double c1 = 2 / l1_2;
        //        d1_star = d1 - (c1 * MVector.DotProduct(d1, e1)) * e1;

        //        // 2add 3sub 7mul
        //        t_star = t1 - (c1 * MVector.DotProduct(t1, e1)) * e1;
        //    }

        //    // 1add 3sub 3mul
        //    var e2 = toZDir - t_star;
        //    double l2_2 = MVector.DotProduct(e2, e2);
        //    if (l1_2 == 0)
        //    {
        //        d1_para = d1_star;
        //    }
        //    else
        //    {
        //        // 2add 3sub 7mul 1div 
        //        double c2 = 2 / l2_2;
        //        d1_para = d1_star - (c2 * MVector.DotProduct(d1_star, e2)) * e2;
        //    }
                
        //    d2_para = MVector.CrossProduct(toZDir, d1_para);
        //    var f_para = new MFrame(toPoint, d1_para, d2_para);
        //    return f_para;
        //}
        //public static MFrame ParallelTransport_Rotation(MFrame frame, MVector fromDir, MPoint toPoint, MVector toDir)
        //{
        //    /*
        //     * WARNING : fromDir and toDir must be of unit length
        //     * 
        //     * aligning t1 to t2 requiers a rotation of alpha in [0, pi] around t1 x t2
        //     * 
        //     * Rodrigues' Formula with θ = α, where :
        //     * α : angle between t1 and t2(sin(α) = |t1 x t2| where |t1|=1 |t2|=1)
        //     * θ : rotation angle to apply around k = t1 x t2
        //     * b : k/|k| unit vector of rotation axis
        //     * v' = v.cos(θ) +  (b x v).sin(θ) + b(b.v).(1-cos(θ))
        //     * v' = v + (b x v).sin(θ) + b x (b x v).(1-cos(θ))
        //     * 
        //     * this is written as (c = cos(θ)) where c <> -1 :
        //     * v' = c.v + (k x v) + (k.v)/(1+c) k
        //     * 
        //     * TOTAL COST
        //     * add  :  6 
        //     * sub  :  6
        //     * mul  : 22
        //     * div  :  1
        //     * sqrt :  0
        //     *     
        //     */

        //    // 3sub 6mul
        //    var k = MVector.CrossProduct(fromDir, toDir);

        //    // 2add 3mul
        //    var c = MVector.DotProduct(fromDir, toDir);

        //    // 3add 3mul 1div
        //    var d1 = frame.XAxis;
        //    var m1 = MVector.DotProduct(k, d1) / (1 + c);

        //    // 6add 3sub 12mul
        //    var d1_para = c * d1; // 3mul
        //    d1_para += MVector.CrossProduct(k, d1); // 3add 3sub 6mul
        //    d1_para += m1 * k; // 3add + 3mul

        //    // 3add 3mul 1div
        //    var d2 = frame.YAxis;
        //    var m2 = MVector.DotProduct(k, d2) / (1 + c);

        //    // 6add 3sub 12mul
        //    var d2_para = c * d2; // 3mul
        //    d2_para += MVector.CrossProduct(k, d2); // 3add 3sub 6mul
        //    d2_para += m2 * k; // 3add + 3mul

        //    var f_para = new MFrame(toPoint, d1_para, d2_para);
        //    return f_para;
        //}
        //public static MFrame ParallelTransport_Reflection(MFrame frame, MVector fromDir, MPoint toPoint, MVector toDir)
        //{
        //    /*
        //     * WARNING : fromDir and toDir must be of unit length
        //     * 
        //     * Double Reflection Method
        //     * 
        //     * TOTAL COST
        //     * add  :  6 
        //     * sub  :  6
        //     * mul  : 22
        //     * div  :  1
        //     * sqrt :  0
        //     *     
        //     */

        //    MVector d1_star, d1_para, d2_star, d2_para;
        //    MVector t_star;

        //    // 2add 3sub 3mul
        //    var e1 = new MVector(toPoint.X - frame.Origin.X, toPoint.Y - frame.Origin.Y, toPoint.Z - frame.Origin.Z);
        //    double l1_2 = MVector.DotProduct(e1, e1);

        //    var d1 = frame.XAxis;
        //    var d2 = frame.YAxis;
        //    if (l1_2 == 0)
        //    {
        //        d1_star = d1;
        //        d2_star = d2;
        //        t_star = fromDir;
        //    }
        //    else
        //    {
        //        // 6add 9sub 21mul 1div
        //        double c1 = 2 / l1_2;
        //        d1_star = d1 - (c1 * MVector.DotProduct(d1, e1)) * e1;
        //        d2_star = d2 - (c1 * MVector.DotProduct(d2, e1)) * e1;
        //        t_star = fromDir - (c1 * MVector.DotProduct(fromDir, e1)) * e1;
        //    }

        //    // 1add 3sub 3mul
        //    var e2 = toDir - t_star;
        //    double l2_2 = MVector.DotProduct(e2, e2);
        //    if (l1_2 == 0)
        //    {
        //        d1_para = d1_star;
        //        d2_para = d2_star;
        //    }
        //    else
        //    {
        //        // 2add 3sub 7mul 1div 
        //        double c2 = 2 / l2_2;
        //        d1_para = d1_star - (c2 * MVector.DotProduct(d1_star, e2)) * e2;
        //        d2_para = d2_star - (c2 * MVector.DotProduct(d2_star, e2)) * e2;
        //    }

        //    var f_para = new MFrame(toPoint, d1_para, d2_para);
        //    return f_para;
        //}


        #endregion

    }
}
