using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class Projection
    {

        #region VECTOR

        // For UNIT projection vectors

        /// <summary>
        /// Projects a vector on the plane perpendicular to the specified unit direction.
        /// </summary>
        /// <param name="vector">The source vector to project.</param>
        /// <param name="uDir">The direction vector being projected on. Must be of unit length.</param>
        /// <returns>The projected vector v = v - ((v.n)/|n|^2) * n.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ProjectPerpendicular_Unit(MVector vector, MVector uDir)
        {
            // warning : uDir must be of unit length
            double alpha = (vector.X * uDir.X + vector.Y * uDir.Y + vector.Z * uDir.Z);
            return new MVector
                (
                vector.X - alpha * uDir.X,
                vector.Y - alpha * uDir.Y,
                vector.Z - alpha * uDir.Z
                );
        }

        /// <summary>
        /// Projects a vector on the specified direction.
        /// </summary>
        /// <param name="vector">The source vector to project.</param>
        /// <param name="uDir">The direction of the plane being projected on. Must be of unit length.</param>
        /// <returns>The projected vector v = (v.n/|n|^2) * n.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ProjectParallel_Unit(MVector vector, MVector uDir)
        {
            // warning : uDir must be of unit length
            double alpha = (vector.X * uDir.X + vector.Y * uDir.Y + vector.Z * uDir.Z);
            return new MVector
                (
                    alpha * uDir.X,
                    alpha * uDir.Y,
                    alpha * uDir.Z
                );
        }

        /// <summary>
        /// Decompose a vector into it's parallel and perpendicular contributions relative to a given plane defined by its normal vector.
        /// The normal is not necessary of unit l.
        /// </summary>
        /// <param name="vector">The source vector to decompose.</param>
        /// <param name="uNormal">The direction vector or the normal vector of the plane to decompose on. Must be of unit length.</param>
        /// <param name="vpar">The parallel component : vpar = (v.n/|n|^2) * n.</param>
        /// <param name="vperp">The perpendicular component : vperp = v - (v.n/|n|^2) * n.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Project_Unit(MVector vector, MVector uNormal, out MVector vpar, out MVector vperp)
        {
            // warning : uNormal must be of unit length
            double alpha = (vector.X * uNormal.X + vector.Y * uNormal.Y + vector.Z * uNormal.Z);
            vpar = new MVector
                (
                    alpha * uNormal.X,
                    alpha * uNormal.Y,
                    alpha * uNormal.Z
                );
            vperp = new MVector
                (
                    vector.X - vpar.X,
                    vector.Y - vpar.Y,
                    vector.Z - vpar.Z
                );
            return true;
        }

        // For NON-UNIT projection vectors

        /// <summary>
        /// Projects a vector on the plane perpendicular to the specified direction.
        /// </summary>
        /// <param name="vector">The source vector to project.</param>
        /// <param name="dir">The direction vector being projected on.</param>
        /// <returns>The projected vector v = v - ((v.n)/|n|^2) * n.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ProjectPerpendicular(MVector vector, MVector dir)
        {
            double l2 = MVector.LengthSquared(vector);
            double alpha = (vector.X * dir.X + vector.Y * dir.Y + vector.Z * dir.Z) / l2;
            return new MVector
                (
                vector.X - alpha * dir.X,
                vector.Y - alpha * dir.Y,
                vector.Z - alpha * dir.Z
                );
        }
        
        /// <summary>
        /// Projects a vector on the specified direction.
        /// </summary>
        /// <param name="vector">The source vector to project.</param>
        /// <param name="dir">The direction of the plane being projected on.</param>
        /// <returns>The projected vector v = (v.n/|n|^2) * n.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MVector ProjectParallel(MVector vector, MVector dir)
        {
            double l2 = MVector.LengthSquared(vector);
            double alpha = (vector.X * dir.X + vector.Y * dir.Y + vector.Z * dir.Z) / l2;
            return new MVector
                (
                    alpha * dir.X,
                    alpha * dir.Y,
                    alpha * dir.Z
                );
        }

        /// <summary>
        /// Decompose a vector into it's parallel and perpendicular contributions relative to a given plane defined by its normal vector.
        /// 
        /// 
        /// The normal is not necessary of unit l.
        /// </summary>
        /// <param name="vector">The source vector to decompose.</param>
        /// <param name="normal">The direction vector or the normal vector of the plane to decompose on.</param>
        /// <param name="vpar">The parallel component : vpar = (v.n/|n|^2) * n.</param>
        /// <param name="vperp">The perpendicular component : vperp = v - (v.n/|n|^2) * n.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Project(MVector vector, MVector normal, out MVector vpar, out MVector vperp)
        {
            double l2 = MVector.LengthSquared(vector);
            double alpha = (vector.X * normal.X + vector.Y * normal.Y + vector.Z * normal.Z) / l2;
            vpar = new MVector
                (
                    alpha * normal.X,
                    alpha * normal.Y,
                    alpha * normal.Z
                );
            vperp = new MVector
                (
                    vector.X - vpar.X,
                    vector.Y - vpar.Y,
                    vector.Z - vpar.Z
                );
            return true;
        }

        #endregion


        #region FRAME

        #endregion

    }
}
