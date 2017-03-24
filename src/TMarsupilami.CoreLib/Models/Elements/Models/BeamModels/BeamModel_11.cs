using System;
using TMarsupilami.CoreLib.Elements.Models;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Elements;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilami.CoreLib.Element
{
    /// <summary>
    /// A beam is subject to compression, tension, shear, bending and torsion.
    /// </summary>
    public class BeamModel_11 : BeamModel
    {
        public BeamModel_11()
        {
            this.ModelRef = "B_11";
            this.Description = "4Dof Beam Model with Disconinuities";
            this.NumberOfRotationalDof = 1;
        }

    }
}
