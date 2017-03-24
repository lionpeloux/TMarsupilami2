using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Elements;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilami.CoreLib.Element
{
    public enum Layout : int
    {
        // Unknown
        Undef = 0,

        // 1D
        Beam = 11,
        Bar = 12,
        Tie = 13,
        Strut = 14,

        // 2D
        Shell = 21,
        Plate = 22,
        Membrane = 23,

        // 3D
        Solid = 31,
    }

    public enum Dimension : int
    {
        One = 1,    // 1D
        Two = 2,    // 2D
        Three = 3,    // 3D
    }

    interface ILayout
    {
        int Id { get; }
        string Label { get; }
        string Description { get; }

        /// <summary>
        /// The spatial dimension of the element layout.
        /// </summary>
        /// <remarks>
        /// Can be either :
        ///     - 1D : beam, cable, chain, bar, tie, strut, spring, ...
        ///     - 2D : shell, plate, membrane, ...
        ///     - 3D : solid, ...
        /// </remarks>
        int SpatialDimension { get; }

        /// <summary>
        /// The total number of vertices involved in the element layout.
        /// </summary>
        int VertexCount { get; }

        bool IsCapableOfCompression { get; }
        bool IsCapableOfTension { get; }
        bool IsCapableOfShear { get; }
        bool IsCapableOfBending { get; }
        bool IsCapableOfTorsion { get; }
    }
}
