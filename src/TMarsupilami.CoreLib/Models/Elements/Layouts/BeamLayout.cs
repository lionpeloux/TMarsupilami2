using System;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Elements;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilami.CoreLib.Element
{
    /// <summary>
    /// A beam is subject to compression, tension, shear, bending and torsion.
    /// </summary>
    public class BeamLayout : ILayout1D
    {
        public int Id { get; protected set; }
        public string Label
        {
            get
            {
                return "Beam";
            }
        }
        public string Description
        {
            get
            {
                return "Generic Beam Element Layout.";
            }
        }

        public int SpatialDimension
        {
            get { return 1; }
        }
        public int VertexCount
        {
            get
            {
                return Nv;
            }
        }

        public bool IsCapableOfCompression
        {
            get { return true; }
        }
        public bool IsCapableOfTension
        {
            get { return true; }
        }
        public bool IsCapableOfShear
        {
            get { return true; }
        }
        public bool IsCapableOfBending
        {
            get { return true; }
        }
        public bool IsCapableOfTorsion
        {
            get { return true; }
        }

        public bool IsClosed { get; protected set; }
        public bool HasUniformSection { get; protected set; }
        public bool HasUniformMaterial { get; protected set; }

        public int Nv { get; protected set; }
        public int Ne { get; protected set; }       
        public Section[] Sections { get; protected set; }
        public Material[] Materials { get; protected set; }
    
        protected MFrame[] mframes_0;   // material frames in REST configuration
        protected MFrame[] mframes;     // material frames in ACTUAL configuration
    }
}
