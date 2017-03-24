using System;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Elements;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilami.CoreLib.Element
{
    /// <summary>
    /// A bar is subject to compression and tension.
    /// </summary>
    public class Bar : ILayout1D
    {
        public int Id { get; protected set; }
        public string Label
        {
            get
            {
                return "Bar";
            }
        }
        public string Description
        {
            get
            {
                return "Generic Bar Element Layout.";
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
            get { return false; }
        }
        public bool IsCapableOfBending
        {
            get { return false; }
        }
        public bool IsCapableOfTorsion
        {
            get { return false; }
        }
     
        public bool IsClosed { get; protected set; }
        public bool HasUniformSection { get; protected set; }
        public bool HasUniformMaterial { get; protected set; }

        public int Nv { get; protected set; }
        public int Ne { get; protected set; }
        public Section[] Sections { get; protected set; }
        public Material[] Materials { get; protected set; }

        protected MPoint[] x_0;   // centerline vertices in REST configuration
        protected MPoint[] x;     // centerline vertices in ACTUAL configuration
    }
}
