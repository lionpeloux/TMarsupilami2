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
        /// Returns true if the load is expressed in the material/local coordinate system of the element.
        /// Returns false if the load is expressed in the global/world coordinate system.
        /// </summary>
        public bool IsLocal { get; protected set; }

        /// <summary>
        /// Returns true if the load is uniform over the element.
        /// Returns false otherwise.
        /// </summary>
        public bool IsUniform { get; protected set; }

        protected Load(int dimension, bool isStatic, bool isLocal, bool isUniform)
        {
            Dimension = dimension;
            IsStatic = isStatic;
            IsLocal = isLocal;
            IsUniform = isUniform;
        }

        //public abstract bool CastTo<T>(T target);
    }

    public class LoadCase
    {
        public int Loads_0D { get; private set; }
        public int Loads_1D { get; set; }
        public int Loads_2D { get; set; }
    }

    // force, moment, pressure, thermal, applied displacement,  ...

    public abstract class VectorLoad : Load
    { 
        /// <summary>
        /// Returns true if the load is distributed.
        /// Returns false if the load is concentrated.
        /// </summary>
        public bool IsDistributed { get; protected set; }

        protected VectorLoad(bool isLocal, bool isUniform, bool isDistributed) 
            : base(0, true, isLocal, isUniform)
        {
        }

        /// <summary>
        /// Creates a concentrated vector load to be applied at an element vertex.
        /// </summary>
        /// <param name="tensorValue">The vector load to be applied to the element vertex.</param>
        /// <param name="vertexIndex">The vertex index in the element.</param>
        /// <param name="isLocal">True if the load is given in the material/local coordinate system of the element. Default to false.</param>
        /// <returns>A VectorLoad object.</returns>
        public static VectorLoad CreateSingleCVL(MVector tensorValue, int vertexIndex, bool isLocal = false)
        {
            return new SingleConcentratedVectorLoad(tensorValue, vertexIndex, isLocal);
        }

        /// <summary>
        /// Creates a uniform vector load to be applied at all element vertices.
        /// </summary>
        /// <param name="tensorValue">The uniform vector load to be applied to all the element vertices.</param>
        /// <param name="isLocal">True if the load is given in the material/local coordinate system of the element. Default to false.</param>
        /// <returns>A VectorLoad object.</returns>
        public static VectorLoad CreateUniformCVL(MVector tensorValue, bool isLocal = false)
        {
            return new UniformConcentratedVectorLoad(tensorValue, isLocal);
        }

        /// <summary>
        /// Creates a group of concentrated vector loads to be applied to all element vertices.
        /// </summary>
        /// <param name="F">The vector loads to be applied on the element vertices. Must match the internal structure of the layout of the element vertices.</param>
        /// <param name="isLocal">True if the  vector loads are given in the material/local coordinate system of the element. Default to false.</param>
        /// <returns>A VectorLoad object.</returns>
        public static VectorLoad CreateMultipleCVL(MVector[] tensorValues, bool isLocal = false)
        {
            return new MultipleConcentratedVectorLoad(tensorValues, isLocal);
        }


        public abstract bool GetValueAt(int vertexIndex, out MVector value, CoordinateSystem cs = CoordinateSystem.Global);

        private sealed class SingleConcentratedVectorLoad : VectorLoad
        {
            private int index;
            private MVector value;

            public SingleConcentratedVectorLoad(MVector tensorValue, int vertexIndex, bool isLocal)
                : base(isLocal, false, false)
            {
                index = vertexIndex;
                value = tensorValue;
            }
            public override bool GetValueAt(int vertexIndex, out MVector value, CoordinateSystem cs = CoordinateSystem.Global)
            {
                if (vertexIndex == index)
                {
                    value = this.value;
                    return true;
                }
                else
                {
                    value = new MVector(0, 0, 0);
                    return false;
                }
            }
        }
        private sealed class UniformConcentratedVectorLoad : VectorLoad
        {
            private MVector value;

            public UniformConcentratedVectorLoad(MVector tensorValue, bool isLocal)
                : base(isLocal, false, false)
            {
                value = tensorValue;
            }
            public override bool GetValueAt(int vertexIndex, out MVector value, CoordinateSystem cs = CoordinateSystem.Global)
            {
                value = this.value;
                return true;
            }
        }
        private sealed class MultipleConcentratedVectorLoad : VectorLoad
        {
            private MVector[] values;

            public MultipleConcentratedVectorLoad(IEnumerable<MVector> tensorValues, bool isLocal)
                : base(isLocal, false, false)
            {
                values = tensorValues.ToArray();
            }
            public override bool GetValueAt(int vertexIndex, out MVector value, CoordinateSystem cs = CoordinateSystem.Global)
            {
                if (vertexIndex >= 0 && vertexIndex < values.Length)
                {
                    value = values[vertexIndex];
                    return true;
                }
                value = new MVector(0, 0, 0);
                return false;
            }
        }
    }

   

    
    }
}
