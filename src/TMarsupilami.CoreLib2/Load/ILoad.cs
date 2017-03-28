using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    /// <summary>
    ///  Types of coordinate systems. 
    ///  Can be either the material coordinate system of the element (t, d1, d2) or the world coordinate system (x, y, z).
    /// </summary>
    public enum CoordinateSystem
    {
        Material,
        Global,
    }

    // force, moment, pressure, thermal, prestress, distortion, applied displacement,  ...
    public abstract class Load
    {
        /// <summary>
        /// Gets the spatial dimension of the load.
        /// </summary>
        /// <remarks>
        ///     - 0 if the load is punctual
        ///     - 1 if the load is distributed over a 1D element (curve)
        ///     - 2 if the load is distributed over a 2D element (surface)
        ///     - 3 if the load is distribuded over a 3D element (volume)
        /// </remarks>
        public int Dimension { get; protected set; }

        /// <summary>
        ///  Returns true if the load is static.
        ///  Returns false if the load is dynamic, that is time dependent.
        /// </summary>
        public bool IsStatic { get; protected set; }

        /// <summary>
        /// Returns true if the load is expressed in the global/world coordinate system.
        /// Returns false if the load is expressed in the material/local coordinate system of the element.
        /// </summary>
        public bool IsGlobal { get; protected set; }

        /// <summary>
        /// Returns true if the load is uniform over the element.
        /// Returns false otherwise.
        /// </summary>
        public bool IsUniform { get; protected set; }

        protected Load(int dimension, bool isStatic, bool isUniform, bool isGlobal)
        {
            Dimension = dimension;
            IsStatic = isStatic;
            IsGlobal = isGlobal;
            IsUniform = isUniform;
        }

        //public abstract bool CastTo<T>(T target);
    }

}
