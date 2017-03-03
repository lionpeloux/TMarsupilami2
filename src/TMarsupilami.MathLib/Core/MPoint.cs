﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public struct MPoint
    {
        #region FIELDS

        /// <summary>
        /// The X component of the point.
        /// </summary>
        private double x;

        /// <summary>
        /// The Y component of the point.
        /// </summary>
        private double y;

        /// <summary>
        /// The Z component of the point.
        /// </summary>
        private double z;

        #endregion

        #region CONSTRUCTORS

        /// <summary>
        /// Constructs a point with the given individual elements.
        /// </summary>
        /// <param name="x">The X component.</param>
        /// <param name="y">The Y component.</param>
        /// <param name="z">The Z component.</param>
        public MPoint(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        /// <summary>
        /// Constructs a point whose elements are all the single specified value.
        /// </summary>
        /// <param name="value">The element to fill the point with.</param>
        public MPoint(double value) : this(value, value, value) { }

        /// <summary>
        /// Constructs a new point with the given point.
        /// </summary>
        /// <param name="v">The given point.</param>
        public MPoint(MPoint p)
        {
            x = p.x;
            y = p.y;
            z = p.z;
        }

        /// <summary>
        /// Constructs a point with the given array.
        /// </summary>
        /// <param name="xyz">An array with the [x,y,z] components.</param>
        public MPoint(double[] xyz)
        {
            if (xyz.Length == 3)
            {
                this.x = xyz[0];
                this.y = xyz[1];
                this.z = xyz[2];
            }
            else
                throw new IndexOutOfRangeException();
        }

        #endregion

        #region INSTANCE PROPERTIES

        /// <summary>
        /// Gets or sets the X (first) component of the point.
        /// </summary>
        public double X
        {
            get { return x; }
            set { x = value; }
        }

        /// <summary>
        /// Gets or sets the Y (second) component of the point.
        /// </summary>
        public double Y
        {
            get { return y; }
            set { y = value; }
        }

        /// <summary>
        /// Gets or sets the Z (third) component of the point.
        /// </summary>
        public double Z
        {
            get { return z; }
            set { z = value; }
        }

        /// <summary>
        /// Gets or sets a point component at the given index.
        /// </summary>
        /// <param name="index">Index of point component. Valid values are: 
        /// <para>0 = X-component</para>
        /// <para>1 = Y-component</para>
        /// <para>2 = Z-component</para>
        /// .</param>
        public double this[int index]
        {
            get
            {
                if (0 == index)
                    return x;
                if (1 == index)
                    return y;
                if (2 == index)
                    return z;
                throw new IndexOutOfRangeException();
            }
            set
            {
                if (0 == index)
                    x = value;
                else if (1 == index)
                    y = value;
                else if (2 == index)
                    z = value;
                else
                    throw new IndexOutOfRangeException();
            }
        }

        #endregion

        #region STATIC OPERATORS
        /// <summary>
        /// Adds two points together.
        /// </summary>
        /// <param name="p1">The first source point.</param>
        /// <param name="v2">The second source point.</param>
        /// <returns>The summed point.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MPoint operator +(MPoint p1, MPoint p2)
        {
            return new MPoint(p1.X + p2.X, p1.Y + p2.Y, p1.Z + p2.Z);
        }

        /// <summary>
        /// Substract two points.
        /// </summary>
        /// <param name="p1">The first source point.</param>
        /// <param name="p2">The second source point.</param>
        /// <returns>The point p1-p2.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MPoint operator -(MPoint p1, MPoint p2)
        {
            return new MPoint(p1.X - p2.X, p1.Y - p2.Y, p1.Z - p2.Z);
        }

        /// <summary>
        /// Add a vector to a point.
        /// </summary>
        /// <param name="p">The source point.</param>
        /// <param name="v">The translation vector.</param>
        /// <returns>The translated point.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MPoint operator +(MPoint p, MVector v)
        {
            return new MPoint(p.X + v.X, p.Y + v.Y, p.Z + v.Z);
        }

        /// <summary>
        /// Add a vector to a point.
        /// </summary>
        /// <param name="v">The translation vector.</param>
        /// <param name="p">The source point.</param>
        /// <returns>The translated point.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MPoint operator +(MVector v, MPoint p)
        {
            return new MPoint(v.X + p.X, v.Y + p.Y, v.Z + p.Z);
        }
        #endregion

    }
}
