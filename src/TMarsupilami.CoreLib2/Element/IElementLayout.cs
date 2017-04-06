using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    public interface IElementLayout
    {
        int Id { get; set; }

        /// <summary>
        /// Gets the spatial dimension of the elemennt.
        /// </summary>
        /// <remarks>
        ///     - 0 if the load is punctual
        ///     - 1 if the load is distributed over a 1D element (curve)
        ///     - 2 if the load is distributed over a 2D element (surface)
        ///     - 3 if the load is distribuded over a 3D element (volume)
        /// </remarks>
        int Dimension { get; }
    }

    public interface IElementLayout1D : IElementLayout
    {
        int Nv { get; }
        int Ne { get; }
        bool IsClosed { get; }

        /// <summary>
        /// Converts vector coordinates from global/world coordinate system to material/local coordinate system.
        /// </summary>
        /// <param name="globalVector">The vector to convert expressed in the global/world coordinate system.</param>
        /// <param name="index">The element vertex or edge index where the quantity is located.</param>
        /// <param name="isEdgeQuantity">True if the vector describes an edge quantity. False if it describes a vertex quantity. Default to false.</param>
        /// <returns>The converted vector expressed in the material/local coordinate system.</returns>
        MVector ConvertToMaterialCS(MVector globalVector, int index, bool isEdgeQuantity = false);

        /// <summary>
        /// Converts vector coordinates from material/local coordinate system to global/world coordinate system.
        /// </summary>
        /// <param name="materialVector">The vector to convert expressed in the material/local coordinate system.</param>
        /// <param name="index">The element vertex or edge index where the quantity is located.</param>
        /// <param name="isEdgeQuantity">True if the vector describes an edge quantity. False if it describes a vertex quantity. Default to false.</param>
        /// <returns>The converted vector expressed in the global/world coordinate system.</returns>
        MVector ConvertToGlobalCS(MVector materialVector, int index, bool isEdgeQuantity = false);

        bool IsEdgeIndexValid(int edgeIndex, bool throwError);
        bool IsVertexIndexValid(int vertexIndex, bool throwError);
    }

    public interface IBeamLayout : IElementLayout1D
    {
        // Geometric layaout
        MFrame[] ActualConfiguration { get; }
        MFrame[] RestConfiguration { get; }

        // Dynamic layout
        double[] LMx { get; }
        double[] LMθ { get; }
        MVector[] Vx { get; }      // in global CS
        MVector[] Vθ { get; }      // in material CS
        MVector[] Ax { get; }      // in global CS
        MVector[] Aθ { get; }      // in material CS

        // Applied loads manager
        BeamLoadManager LoadManager { get; }
    }

    public interface IBarLayout : IElementLayout1D
    {
        MVector[] ActualConfiguration { get; }
        MVector[] RestConfiguration { get; }
    }
}
