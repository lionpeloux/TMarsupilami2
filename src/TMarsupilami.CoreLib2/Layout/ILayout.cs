using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2.Layout
{
    interface ILayout
    {
    }

    interface ILayout1D : ILayout
    {
        int Nv { get; }
        int Ne { get; }
        bool IsClosed { get; }

        /// <summary>
        /// Converts vector coordinates from global/world to material/local coordinate system.
        /// </summary>
        /// <param name="globalVector">The vector to convert expressed in the global/world coordinate system.</param>
        /// <param name="index">The element vertex or edge index where the quantity is located.</param>
        /// <param name="isVertexQuantity">True if the vector describes a vertex quantity. False if it describes an edge quantity. Default to true.</param>
        /// <returns>The converted vector expressed in the material/local coordinate system.</returns>
        MVector ConvertToMaterialCS(MVector globalVector, int index, bool isVertexQuantity = true);

        /// <summary>
        /// Converts vector coordinates from material/local to global/world coordinate system.
        /// </summary>
        /// <param name="materialVector">The vector to convert expressed in the material/local coordinate system.</param>
        /// <param name="index">The element vertex or edge index where the quantity is located.</param>
        /// <param name="isVertexQuantity">True if the vector describes a vertex quantity. False if it describes an edge quantity. Default to true.</param>
        /// <returns>The converted vector expressed in the global/world coordinate system.</returns>
        MVector ConvertToGlobalCS(MVector materialVector, int index, bool isVertexQuantity = true);

    }

    interface IBeamLayout : ILayout1D
    {
        MFrame[] ActualConfiguration { get; }
        MFrame[] RestConfiguration { get; }

        MVector[] Fext_g { get; }
        MVector[] Fext_m { get; }
        MVector[] fext_g { get; }
        MVector[] fext_m { get; }

        /// <summary>
        /// Gets applied concentrated vector loads
        /// </summary>
        MVector[] fext { get; }

        MVector[] Mext_g { get; }
        MVector[] Mext_m { get; }

        MVector[] mext { get; }

    }

    interface IBarLayout : ILayout1D
    {
        MVector[] ActualConfiguration { get; }
        MVector[] RestConfiguration { get; }
    }
}
