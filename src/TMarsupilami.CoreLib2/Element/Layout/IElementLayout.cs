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

        // Applied loads manager
        BeamLoadManager LoadManager { get; }

    }

    public interface IBarLayout : IElementLayout1D
    {
        MVector[] ActualConfiguration { get; }
        MVector[] RestConfiguration { get; }
    }
}
