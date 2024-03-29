﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public struct MVector : IDeepCopy<MVector>, IEquatable<MVector>
    {
        #region FIELDS
        
        /// <summary>
        /// The X component of the vector.
        /// </summary>
        private double x;
        
        /// <summary>
        /// The Y component of the vector.
        /// </summary>
        private double y;
        
        /// <summary>
        /// The Z component of the vector.
        /// </summary>
        private double z;
        #endregion

        #region CONSTRUCTORS

        /// <summary>
        /// Constructs a vector with the given individual elements.
        /// </summary>
        /// <param name="x">The X component.</param>
        /// <param name="y">The Y component.</param>
        /// <param name="z">The Z component.</param>
        public MVector(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        /// <summary>
        /// Constructs a vector whose elements are all the single specified value.
        /// </summary>
        /// <param name="value">The element to fill the vector with.</param>
        public MVector(double value) : this(value, value, value) { }
        
        /// <summary>
        /// Constructs a new vector with the given vector.
        /// </summary>
        /// <param name="vector">The given vector.</param>
        public MVector(MVector vector)
        {
            x = vector.x;
            y = vector.y;
            z = vector.z;
        }

        /// <summary>
        /// Constructs a new vector with a given point.
        /// </summary>
        /// <param name="point">The given vector.</param>
        public MVector(MPoint point)
        {
            x = point.X;
            y = point.Y;
            z = point.Z;
        }

        /// <summary>
        /// Constructs a new vector with two given point.
        /// </summary>
        /// <param name="fromPoint">The start point.</param>
        /// <param name="toPoint">The end point.</param>
        public MVector(MPoint fromPoint, MPoint toPoint)
        {
            x = toPoint.X - fromPoint.X;
            y = toPoint.Y - fromPoint.Y;
            z = toPoint.Z - fromPoint.Z;
        }

        /// <summary>
        /// Constructs a vector with the given array.
        /// </summary>
        /// <param name="xyz">An array with the [x,y,z] components.</param>
        public MVector(double[] xyz)
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
        /// Gets or sets the X (first) component of the vector.
        /// </summary>
        public double X { 
            get { return x; } 
            set { x = value; } 
        }
        
        /// <summary>
        /// Gets or sets the Y (second) component of the vector.
        /// </summary>
        public double Y { 
            get { return y; } 
            set { y = value; } 
        }
        
        /// <summary>
        /// Gets or sets the Z (third) component of the vector.
        /// </summary>
        public double Z { 
            get { return z; } 
            set { z = value; } 
        }
        
        /// <summary>
        /// Gets or sets a vector component at the given index.
        /// </summary>
        /// <param name="index">Index of vector component. Valid values are: 
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
        
        /// <summary>
        /// Gets a value indicating whether or not this is a unit vector. 
        /// A unit vector has length 1.
        /// </summary>
        public bool IsUnitVector
        {
            get
            {
                // result is returned according to a certain accuracy
                return System.Math.Abs(Length() - 1.0) <= 1e-9;
            }
        }
        #endregion

        #region INSTANCE METHODS

        public MVector DeepCopy()
        {
            return new MVector(this);
        }

        public bool IsNull()
        {
            return (x == 0) && (y == 0) && (z == 0);
        }
        public bool IsNull(double tolerance)
        {
            return (Math.Abs(x - 0) <= tolerance) && (Math.Abs(y - 0) <= tolerance) && (Math.Abs(z - 0) <= tolerance);
        }

        /// <summary>
        /// Computes the length of this vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double Length()
        {
            return Length(this);
        }

        /// <summary>
        /// Computes the squared length of this vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double LengthSquared()
        {
            return LengthSquared(this);
        }
        
        /// <summary>
        /// Normalizes this vector in place. A unit vector has length 1 unit. 
        /// </summary>
        /// <returns>true on success or false on failure.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Normalize()
        {
            double l = Length();
            if (l == 0) { return false; }
            else if (l == 1) { return true; }
            else {
                l = 1 / l;
                x = x * l;
                y = y * l;
                z = z * l; 
                return true; 
            }
        }

        ///<summary>
        /// Reverses this vector in place.
        ///</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Reverse()
        {
            x = -x;
            y = -y;
            z = -z;
        }

        /// <summary>
        /// Reflects this vector in place, off a plane that has the specified normal v = v - (2(v.n)/|n|^2)n
        /// The normal is not necessary of unit l.
        /// </summary>
        /// <param name="normal">The normal of the surface being reflected off.</param>
        /// <returns>true on success, false on failure.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Reflect(MVector normal)
        {
            double l2 = normal.LengthSquared();
            if (l2 == 0) { return false; }
            else
            {
                double alpha = 2 * (x * normal.x + y * normal.y + z * normal.z) / l2;
                x = x - alpha * normal.x;
                y = y - alpha * normal.y;
                z = z - alpha * normal.z;
                return true;
            }        
        }

        /// <summary>
        /// Rotates this vector in place, off a given angle around a given axis.
        /// </summary>
        /// <param name="angle">Angle of rotation (in radians).</param>
        /// <param name="axis">Axis of rotation.</param>
        /// <returns>True on success, false on failure.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Rotate(double angle, MVector axis)
        {
            axis.Normalize();
            Rotation.Rotate(ref this, angle, axis);
            

            //double c = System.Math.Cos(angle);
            //double s = System.Math.Sin(angle);

            //double ax = (1 - c) * x * axis.x;
            //double ay = (1 - c) * y * axis.y;
            //double az = (1 - c) * z * axis.z;

            //double xtmp = x * c + ax * axis.x + ay * axis.x + az * axis.x + s * (axis.y * z - axis.z * y);
            //double ytmp = y * c + ax * axis.y + ay * axis.y + az * axis.y + s * (axis.z * x - axis.x * z);
            //double ztmp = z * c + ax * axis.z + ay * axis.z + az * axis.z + s * (axis.x * y - axis.y * x);

                
            ////double xtmp = x * c + (1 - c) * (x * axis.x * axis.x + y * axis.x * axis.y + z * axis.x * axis.z) + s * (axis.y * z - axis.z * y);
            ////double ytmp = y * c + (1 - c) * (x * axis.x * axis.y + y * axis.y * axis.y + z * axis.y * axis.z) + s * (axis.z * x - axis.x * z);
            ////double ztmp = z * c + (1 - c) * (x * axis.x * axis.z + y * axis.y * axis.z + z * axis.z * axis.z) + s * (axis.x * y - axis.y * x);

            //x = xtmp;
            //y = ytmp;
            //z = ztmp;            
        }

        /// <summary>
        /// Returns a boolean indicating whether the given Vector is equal to this Vector instance.
        /// </summary>
        /// <param name="other">The Vector to compare this instance to.</param>
        /// <returns>True if the other Vector is equal to this instance; False otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(MVector other)
        {
            return this.x == other.X &&
                   this.Y == other.Y &&
                   this.Z == other.Z;
        }

        /// <summary>
        /// Returns a boolean indicating whether the given Object is equal to this Vector instance.
        /// </summary>
        /// <param name="obj">The Object to compare against.</param>
        /// <returns>True if the Object is equal to this Vector; False otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            if (!(obj is MVector))
                return false;
            return Equals((MVector)obj);
        }

        /// <summary>
        /// Check that all values in other are within epsilon of the values in this vector.
        /// </summary>
        /// <param name="other"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        public bool Equals(MVector other, double epsilon)
        {
            return
                    System.Math.Abs(x - other.x) < epsilon &&
                    System.Math.Abs(y - other.y) < epsilon &&
                    System.Math.Abs(z - other.z) < epsilon
                ;
        }

        /// <summary>
        /// Computes the hash code for the current vector.
        /// </summary>
        /// <returns>A non-unique number that represents the components of this vector.</returns>
        public override int GetHashCode()
        {
            // MSDN docs recommend XOR'ing the internal values to get a hash code
            return x.GetHashCode() ^ y.GetHashCode() ^ z.GetHashCode();
        }

        /// <summary>
        /// Returns the string representation of the current vector, in the form X,Y,Z.
        /// </summary>
        /// <returns>A string with the current coordinates of the vector.</returns>
        public override string ToString()
        {
            return String.Format("{0:F6},{1:F6},{2:F6}", x, y, z);
        }
        #endregion

        #region STATIC OPERATORS
        /// <summary>
        /// Adds two vectors together.
        /// </summary>
        /// <param name="v1">The first source vector.</param>
        /// <param name="v2">The second source vector.</param>
        /// <returns>The summed vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator +(MVector v1, MVector v2)
        {
            return new MVector(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        /// <summary>
        /// Subtracts the second vector from the first.
        /// </summary>
        /// <param name="v1">The first source vector.</param>
        /// <param name="v2">The second source vector.</param>
        /// <returns>The difference vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator -(MVector v1, MVector v2)
        {
            return new MVector(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        /// <summary>
        /// Multiplies two vectors together.
        /// </summary>
        /// <param name="v1">The first source vector.</param>
        /// <param name="v2">The second source vector.</param>
        /// <returns>The product vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double operator *(MVector v1, MVector v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
        }

        /// <summary>
        /// Multiplies a vector by the given scalar.
        /// </summary>
        /// <param name="v">The source vector.</param>
        /// <param name="λ">The scalar value.</param>
        /// <returns>The scaled vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator *(MVector v, double λ)
        {
            return new MVector(λ * v.X, λ * v.Y, λ * v.Z);
        }

        /// <summary>
        /// Multiplies a vector by the given scalar.
        /// </summary>
        /// <param name="λ">The scalar value.</param>
        /// <param name="v">The source vector.</param>
        /// <returns>The scaled vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator *(double λ, MVector v)
        {
            return new MVector(λ * v.X, λ * v.Y, λ * v.Z);
        }

        /// <summary>
        /// Divides the first vector by the second.
        /// </summary>
        /// <param name="v1">The first source vector.</param>
        /// <param name="v2">The second source vector.</param>
        /// <returns>The vector resulting from the division.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator /(MVector v1, MVector v2)
        {
            return new MVector(v1.X / v2.X, v1.Y / v2.Y, v1.Z / v2.Z);
        }

        /// <summary>
        /// Divides the vector by the given scalar.
        /// </summary>
        /// <param name="v">The source vector.</param>
        /// <param name="λ">The scalar value.</param>
        /// <returns>The result of the division.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator /(MVector v, double λ)
        {
            return new MVector(v.X / λ, v.Y / λ, v.Z / λ);
        }

        /// <summary>
        /// Negates a given vector.
        /// </summary>
        /// <param name="vector">The source vector.</param>
        /// <returns>The negated vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector operator -(MVector vector)
        {
            return new MVector(-vector.X, -vector.Y, -vector.Z);
        }

        /// <summary>
        /// Returns a boolean indicating whether the two given vectors are equal.
        /// </summary>
        /// <param name="v1">The first vector to compare.</param>
        /// <param name="v2">The second vector to compare.</param>
        /// <returns>True if the vectors are equal; False otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(MVector v1, MVector v2)
        {
            return
                    v1.X == v2.X &&
                    v1.Y == v2.Y &&
                    v1.Z == v2.Z
                ;
        }

        /// <summary>
        /// Returns a boolean indicating whether the two given vectors are not equal.
        /// </summary>
        /// <param name="v1">The first vector to compare.</param>
        /// <param name="v2">The second vector to compare.</param>
        /// <returns>True if the vectors are not equal; False if they are equal.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(MVector v1, MVector v2)
        {
            return
                    v1.X != v2.X ||
                    v1.Y != v2.Y ||
                    v1.Z != v2.Z
                ;
        }
        #endregion Public Static Operators

        #region STATIC PROPERTIES
        /// <summary>
        /// Gets the value of the vector with components 0,0,0.
        /// </summary>
        public static MVector Zero
        {
            get { return new MVector(); }
        }

        /// <summary>
        /// Gets the value of the vector with components 1,1,1.
        /// </summary>
        public static MVector One
        {
            get { return new MVector(1.0, 1.0, 1.0); }
        }

        /// <summary>
        /// Gets the value of the vector with components (1,0,0).
        /// </summary>
        public static MVector XAxis
        {
            get { return new MVector(1.0, 0.0, 0.0); }
        }

        /// <summary>
        /// Gets the value of the vector with components (0,1,0).
        /// </summary>
        public static MVector YAxis
        {
            get { return new MVector(0.0, 1.0, 0.0); }
        }

        /// <summary>
        /// Gets the value of the vector with components (0,0,1).
        /// </summary>
        public static MVector ZAxis
        {
            get { return new MVector(0.0, 0.0, 1.0); }
        }
        #endregion

        #region STATIC METHODS
        /// <summary>
        /// Returns the length of a vector |v|.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Length(MVector v)
        {
            return Math.Sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
        }

        /// <summary>
        /// Computes the squared length of a vector |v|^2.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double LengthSquared(MVector v)
        {
            return (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
        }

        /// <summary>
        /// Returns a vector with the same direction as the given vector, but with a length of 1.
        /// </summary>
        /// <param name="value">The vector to normalize.</param>
        /// <returns>The normalized vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector Normalize(MVector v)
        {
            double l = v.Length();
            if (l == 0) { return MVector.Zero; }
            else
            {
                l = 1 / l;
                return new MVector
                (
                    v.x * l,
                    v.y * l,
                    v.z * l
                );
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ReNormalize_R2(MVector v)
        {
            double l2 = v.LengthSquared();
            double _l = Sqrt.InvSqrt_R2(l2); // fast inv sqrt for l2 in [0.9, 1.1]
            return new MVector
            (
                v.x * _l,
                v.y * _l,
                v.z * _l
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ReNormalize_R2_N1(MVector v)
        {
            double l2 = v.LengthSquared();
            double _l = Sqrt.InvSqrt_R2_N1(l2); // fast inv sqrt for l2 in [0.9, 1.1]
            return new MVector
            (
                v.x * _l,
                v.y * _l,
                v.z * _l
            );
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ReNormalize_R2_N2(MVector v)
        {
            double l2 = v.LengthSquared();
            double _l = Sqrt.InvSqrt_R2_N2(l2); // fast inv sqrt for l2 in [0.9, 1.1]
            return new MVector
            (
                v.x * _l,
                v.y * _l,
                v.z * _l
            );
        }

        ///<summary>
        /// Return the reversed vector v = -v.
        ///</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector Reverse(MVector v)
        {
            return new MVector
                (
                    v.x = -v.x,
                    v.y = -v.y,
                    v.z = -v.z
                );
        }

        /// <summary>
        /// Returns the Euclidean distance between the two given vectors : |v2-v1|.
        /// </summary>
        /// <param name="v1">The first point.</param>
        /// <param name="v2">The second point.</param>
        /// <returns>The distance |v2-v1|.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Distance(MVector v1, MVector v2)
        {
            return System.Math.Sqrt
                (
                (v2.x - v1.x) * (v2.x - v1.x) +
                (v2.y - v1.y) * (v2.y - v1.y) +
                (v2.z - v1.z) * (v2.z - v1.z)
                );
        }

        /// <summary>
        /// Returns the Euclidean distance squared between the two given vectors : |v2-v1|^2.
        /// </summary>
        /// <param name="v1">The first point.</param>
        /// <param name="v2">The second point.</param>
        /// <returns>The distance squared |v2-v1|^2..</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double DistanceSquared(MVector v1, MVector v2)
        {
            return
                (v2.x - v1.x) * (v2.x - v1.x) +
                (v2.y - v1.y) * (v2.y - v1.y) +
                (v2.z - v1.z) * (v2.z - v1.z)
                ;
        }

        /// <summary>
        /// Returns a vector whose elements are the absolute values of each of the source vector's elements.
        /// </summary>
        /// <param name="v">The source vector.</param>
        /// <returns>The absolute value vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector Abs(MVector v)
        {
            return new MVector(System.Math.Abs(v.X), System.Math.Abs(v.Y), System.Math.Abs(v.Z));
        }

        /// <summary>
        /// Returns a vector whose elements are the square root of each of the source vector's elements.
        /// </summary>
        /// <param name="v">The source vector.</param>
        /// <returns>The square root vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector SquareRoot(MVector v)
        {
            return new MVector(System.Math.Sqrt(v.X), System.Math.Sqrt(v.Y), System.Math.Sqrt(v.Z));
        }

        /// <summary>
        /// Returns the dot product of two vectors.
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>The dot product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double DotProduct(MVector v1, MVector v2)
        {
            return v1.X * v2.X +
                   v1.Y * v2.Y +
                   v1.Z * v2.Z;
        }

        /// <summary>
        /// Computes the cross product of two vectors.
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>The cross product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector CrossProduct(MVector v1, MVector v2)
        {
            return new MVector(
                v1.Y * v2.Z - v1.Z * v2.Y,
                v1.Z * v2.X - v1.X * v2.Z,
                v1.X * v2.Y - v1.Y * v2.X);
        }

        /// <summary>
        /// Computes the cross product of three vectors v = v1 x (v2 x v3).
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <param name="v3">The second vector.</param>
        /// <returns>The cross product v1 x (v2 x v3).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector CrossProduct(MVector v1, MVector v2, MVector v3)
        {
            double ac = v1.x * v3.x + v1.y * v3.y + v1.z * v3.z;
            double ab = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
            return new MVector(
                ac * v2.x + ab * v3.x,
                ac * v2.y + ab * v3.y,
                ac * v2.z + ab * v3.z
                );
        }

        /// <summary>
        /// Computes the triple (or mixed) product of three vectors v = v1 . (v2 x v3)
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <param name="v3">The third vector.</param>
        /// <returns>The triple (or mixed) product v = v1 . (v2 x v3).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double TripleProduct(MVector v1, MVector v2, MVector v3)
        {
            return
                v1.x * (v2.y * v2.z - v2.z * v3.y) +
                v1.y * (v2.z * v3.x - v2.x - v3.z) +
                v1.z * (v2.x * v3.y - v2.y * v3.x);
        }

        /// <summary>
        /// Computes the linear combinaison v = λ1 * v1 + v2
        /// </summary>
        /// <param name="λ1">The scalar.</param>
        /// <param name="v1">The first vector.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>The linear combinaison v = λ1 * v1 + v2</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector LinearComb(double λ1, MVector v1, MVector v2)
        {
            return new MVector(
                λ1 * v1.X + v2.X,
                λ1 * v1.Y + v2.Y,
                λ1 * v1.Z + v2.Z);
        }

        /// <summary>
        /// Computes the linear combinaison v = v1 + λ2 * v2.
        /// </summary>
        /// <param name="v1">The first vector.</param>
        /// <param name="λ2">The scalar.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>The linear combinaison v = v1 + λ2 * v2</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector LinearComb(MVector v1, double λ2, MVector v2)
        {
            return new MVector(
                v1.X + λ2 * v2.X,
                v1.Y + λ2 * v2.Y,
                v1.Z + λ2 * v2.Z);
        }

        /// <summary>
        /// Computes the linear combinaison v = λ2 * v1 + λ2 * v2.
        /// </summary>
        /// <param name="λ1">The first scalar.</param>
        /// <param name="v1">The first vector.</param>
        /// <param name="λ2">The second scalar.</param>
        /// <param name="v2">The second vector.</param>
        /// <returns>The linear combinaison v = λ2 * v1 + λ2 * v2</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector LinearComb(double λ1, MVector v1, double λ2, MVector v2)
        {
            return new MVector(
                λ1 * v1.X + λ2 * v2.X,
                λ1 * v1.Y + λ2 * v2.Y,
                λ1 * v1.Z + λ2 * v2.Z);
        }

        /// <summary>
        /// Computes the linear combinaison v = λ2 * v1 + λ2 * v2 + λ3 * v3.
        /// </summary>
        /// <param name="λ1">The first scalar.</param>
        /// <param name="v1">The first vector.</param>
        /// <param name="λ2">The second scalar.</param>
        /// <param name="v2">The second vector.</param>
        /// <param name="λ3">The third scalar.</param>
        /// <param name="v3">The third vector.</param>
        /// <returns>The linear combinaison v = λ2 * v1 + λ2 * v2 + λ3 * v3.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector LinearComb(double λ1, MVector v1, double λ2, MVector v2, double λ3, MVector v3)
        {
            return new MVector
                (
                λ1 * v1.x + λ2 * v2.x + λ3 * v3.x,
                λ1 * v1.y + λ2 * v2.y + λ3 * v3.y,
                λ1 * v1.z + λ2 * v2.z + λ3 * v3.z
                );
        }

        /// <summary>
        /// Returns the reflection of a vector off a plane that has the specified normal v = v - (2(v.n)/|n|^2)n
        /// The normal is not necessary of unit l.
        /// </summary>
        /// <param name="v">The source vector.</param>
        /// <param name="normal">The normal of the plane being reflected off.</param>
        /// <returns>The reflected vector v = v - (2(v.n)/|n|^2)n.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector Reflect(MVector v, MVector normal)
        {
            double l2 = normal.LengthSquared();

            if (l2 == 0) { return new MVector(); }
            else
            {
                double alpha = 2 * (v.x * normal.x + v.y * normal.y + v.z * normal.z) / l2;
                return new MVector
                    (
                    v.x - alpha * normal.x,
                    v.y - alpha * normal.y,
                    v.z - alpha * normal.z
                    );
            }
        }

        /// <summary>
        /// Basis change of a vector.
        /// </summary>
        /// <param name="vector">The vector expressed in the initial basis.</param>
        /// <param name="fromBasis">Initial basis as an othonormal frame.</param>
        /// <param name="toBasis">Target basis as an othonormal frame.</param>
        /// <returns>The changed vector expressed in the target basis.</returns>
        public static MVector ChangeBasis(MVector vector, MFrame fromBasis, MFrame toBasis)
        {         
            double dxx = fromBasis.XAxis * toBasis.XAxis;
            double dxy = fromBasis.XAxis * toBasis.YAxis;
            double dxz = fromBasis.XAxis * toBasis.ZAxis;

            double dyx = fromBasis.YAxis * toBasis.XAxis;
            double dyy = fromBasis.YAxis * toBasis.YAxis;
            double dyz = fromBasis.YAxis * toBasis.ZAxis;

            double dzx = fromBasis.ZAxis * toBasis.XAxis;
            double dzy = fromBasis.ZAxis * toBasis.YAxis;
            double dzz = fromBasis.ZAxis * toBasis.ZAxis;
           
            double x1 = vector.X;
            double y1 = vector.Y;
            double z1 = vector.Z;

            double x2 = x1 * dxx + y1 * dxy + z1 * dxz;
            double y2 = x1 * dyx + y1 * dyy + z1 * dyz;
            double z2 = x1 * dzx + y1 * dzy + z1 * dzz;

            return new MVector(x2, y2, z2);
        }

        /// <summary>
        /// In-place basis change of a vector.
        /// </summary>
        /// <param name="fromVector">The vector expressed in the initial basis.</param>
        /// <param name="fromBasis">Initial basis as an othonormal frame.</param>
        /// <param name="toBasis">Target basis as an othonormal frame.</param>
        /// <param name="toVector">The changed vector expressed in the target basis.</param>
        public static void ChangeBasis(MVector fromVector, MFrame fromBasis, MFrame toBasis, ref MVector toVector)
        {
            double dxx = fromBasis.XAxis * toBasis.XAxis;
            double dxy = fromBasis.XAxis * toBasis.YAxis;
            double dxz = fromBasis.XAxis * toBasis.ZAxis;

            double dyx = fromBasis.YAxis * toBasis.XAxis;
            double dyy = fromBasis.YAxis * toBasis.YAxis;
            double dyz = fromBasis.YAxis * toBasis.ZAxis;

            double dzx = fromBasis.ZAxis * toBasis.XAxis;
            double dzy = fromBasis.ZAxis * toBasis.YAxis;
            double dzz = fromBasis.ZAxis * toBasis.ZAxis;

            double x1 = fromVector.X;
            double y1 = fromVector.Y;
            double z1 = fromVector.Z;

            double x2 = x1 * dxx + y1 * dxy + z1 * dxz;
            double y2 = x1 * dyx + y1 * dyy + z1 * dyz;
            double z2 = x1 * dzx + y1 * dzy + z1 * dzz;

            toVector.X = x2;
            toVector.Y = y2;
            toVector.Z = z2;
        }


        /// <summary>
        /// Set v1 to v2 (v2 = v1).
        /// </summary>
        /// <param name="v1">The constant vector to set to each item of v2.</param>
        /// <param name="v2">The vectors to be modified.</param>
        public static void Set(MVector v1, ref MVector[] v2)
        {
            for (int i = 0; i < v2.Length; i++)
            {
                v2[i] = v1;
            }
        }


        /// <summary>
        /// Add v1 to v2 (v2 = v1 + v2).
        /// </summary>
        /// <param name="v1">The vectors to add.</param>
        /// <param name="v2">The vectors to add to.</param>
        public static void Add(MVector[] v1, ref MVector[] v2)
        {
            int n1 = v1.Length;
            int n2 = v2.Length;

            if (n1 != n2)
                throw new ArgumentException("MVector arrays must be of same length.");

            for (int i = 0; i < v1.Length; i++)
            {
                v2[i] += v1[i];
            }
        }

        /// <summary>
        /// Add v1 to v2 (v2 = v1 + v2).
        /// </summary>
        /// <param name="v1">The constant vector to add to each item of v2.</param>
        /// <param name="v2">The vectors to add to.</param>
        public static void Add(MVector v1, ref MVector[] v2)
        {
            for (int i = 0; i < v2.Length; i++)
            {
                v2[i] += v1;
            }
        }



        public static MVector[] Add(MVector[] v1, MVector[] v2)
        {
            int n1 = v1.Length;
            int n2 = v2.Length;

            if (n1 != n2)
                throw new ArgumentException("MVector arrays must be of same length.");

            var sum = new MVector[n1];
            for (int i = 0; i < sum.Length; i++)
            {
                sum[i] = v1[i] + v2[i];
            }
            return sum;
        }
        public static MVector[] Substract(MVector[] v1, MVector[] v2)
        {
            int n1 = v1.Length;
            int n2 = v2.Length;

            if (n1 != n2)
                throw new ArgumentException("MVector arrays must be of same length.");

            var sub = new MVector[n1];
            for (int i = 0; i < sub.Length; i++)
            {
                sub[i] = v1[i] - v2[i];
            }
            return sub;
        }
        public static MVector[] Multiply(double λ, MVector[] v)
        {
            var mul = new MVector[v.Length];
            for (int i = 0; i < v.Length; i++)
            {
                mul[i] = λ * v[i];
            }
            return mul;
        }
        #endregion

    }
}
