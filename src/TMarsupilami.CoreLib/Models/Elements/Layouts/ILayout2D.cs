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
    interface ILayout2D : ILayout
    {
        /// <summary>
        /// Returns the section of the element.
        /// </summary>
        Section Section { get; }

        /// <summary>
        /// Returns the material of the element.
        /// </summary>
        Material Material { get; }
    }
}
