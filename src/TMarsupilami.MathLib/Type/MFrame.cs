﻿using System;
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
    /// It is lazyly "on demand" evaluated when accessing the ZAxis property.
    /// </summary>
    public struct MFrame : IDeepCopy<MFrame>
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

        /// <summary>
        /// The ZAxis of the frame.
        /// </summary>
        private MVector zaxis;

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
                this.xaxis.Normalize();
                this.yaxis.Normalize();
            }

            this.zaxis = MVector.CrossProduct(this.xaxis, this.yaxis);
        }

        public MFrame(MPoint origin, MVector xaxis, MVector yaxis, MVector zaxis, bool normalized = false)
        {
            this.origin = origin;
            this.xaxis = xaxis;
            this.yaxis = yaxis;
            this.zaxis = zaxis;

            if (normalized)
            {
                this.xaxis.Normalize();
                this.yaxis.Normalize();
                this.zaxis.Normalize();
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
            zaxis = f.zaxis;
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
            get { return zaxis; }
            set { zaxis = value; }
        }

        #endregion

        #region INSTANCE METHODS

        public MFrame DeepCopy()
        {
            return new MFrame(this);
        }
        public override string ToString()
        {
            return string.Format(
                    "O({0:0.00},{1:0.00},{2:0.00}) Z({3:0.00},{4:0.00},{5:0.00})",
                    this.Origin.X, this.Origin.Y, this.Origin.Z,
                    this.ZAxis.X, this.ZAxis.Y, this.ZAxis.Z
                );
        }

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

        ///// <summary>
        ///// In-place rotation of a frame around a given axis.
        ///// </summary>
        ///// <param name="angle">Angle of rotation (in radians).</param>
        ///// <param name="axis">Axis of rotation. Must be a unit vector.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public void Rotate(double angle, MVector axis)
        //{
        //    Rotation.Rotate(ref this, angle, axis);
        //}

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
            Rotation.ZDiffRotate_T3(this, θ, ref frameROT);
        }
        public void ZDiffRotate(double θ)
        {
            Rotation.ZDiffRotate_T3(this, θ, ref this);
        }

        public void ZParallelTransport_Rotation(MVector fromZUDir, MPoint toPoint, MVector toZUDir, ref MFrame framePT)
        {
            ParallelTransportation.ZPT_Rotation(this, fromZUDir, toPoint, toZUDir, ref framePT);
        }
        public void ZParallelTransport_Rotation(MVector fromZUDir, MPoint toPoint, MVector toZUDir)
        {
            ParallelTransportation.ZPT_Rotation(this, fromZUDir, toPoint, toZUDir, ref this);
        }

        public void ZParallelTransport_Reflection(MPoint fromPoint, MVector fromZUDir, MPoint toPoint, MVector toZUDir, ref MFrame framePT)
        {
            ParallelTransportation.ZPT_Reflection(this, fromPoint, fromZUDir, toPoint, toZUDir, ref framePT);
        }
        public void ZParallelTransport_Reflection(MPoint fromPoint, MVector fromZUDir, MPoint toPoint, MVector toZUDir)
        {
            ParallelTransportation.ZPT_Reflection(this, fromPoint, fromZUDir, toPoint, toZUDir, ref this);
        }

        public void ParallelTransport_Rotation(MVector fromUDir, MPoint toPoint, MVector toUDir, ref MFrame framePT)
        {
            ParallelTransportation.PT_Rotation(this, fromUDir, toPoint, toUDir, ref framePT);
        }
        public void ParallelTransport_Rotation(MVector fromUDir, MPoint toPoint, MVector toUDir)
        {
            ParallelTransportation.PT_Rotation(this, fromUDir, toPoint, toUDir, ref this);
        }

        public void ParallelTransport_Reflection(MPoint fromPoint, MVector fromUDir, MPoint toPoint, MVector toUDir, ref MFrame framePT)
        {
            ParallelTransportation.PT_Reflection(this, fromPoint, fromUDir, toPoint, toUDir, ref framePT);
        }
        public void ParallelTransport_Reflection(MPoint fromPoint, MVector fromUDir, MPoint toPoint, MVector toUDir)
        {
            ParallelTransportation.PT_Reflection(this, fromPoint, fromUDir, toPoint, toUDir, ref this);
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

        public static double DistanceTo(MFrame f1, MFrame f2)
        {
            return MPoint.DistanceTo(f1.Origin, f2.Origin);
        }

    }
}
