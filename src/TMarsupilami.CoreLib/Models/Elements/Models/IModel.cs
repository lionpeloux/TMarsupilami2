using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.CoreLib.Elements.Models
{
    interface IModel
    {
        /// <summary>
        /// The reference (or Id) of the mechanical model (B_XX_YY).
        /// </summary>
        string ModelRef { get; }

        /// <summary>
        /// The description of the mechanical model.
        /// </summary>
        string Description { get; }

        /// <summary>
        /// The spatial dimension of the element mechanical model.
        /// </summary>
        /// <remarks>
        /// Can be either :
        ///     - 1D : beam, cable, chain, bar, tie, strut, spring, ...
        ///     - 2D : shell, plate, membrane, ...
        ///     - 3D : solid, ...
        /// </remarks>
        int SpatialDimension { get; }

        /// <summary>
        /// The number of translational degrees of freedom (Dof) in the mechanical model.
        /// </summary>
        int NumberOfTranslationalDof { get; }

        /// <summary>
        /// The number of rotational degrees of freedom (Dof) in the mechanical model.
        /// </summary>
        int NumberOfRotationalDof { get; }

        /// <summary>
        /// The total number of degrees of freedom (Dof) in the mechanical model.
        /// </summary>
        int NumberOfDof { get; }
    }
}
