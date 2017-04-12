using System;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum SectionType
    {
        Circular,
        Rectangular,
    }
    public class Section : IDeepCopy<Section>
    {
        #region FIELDS
        private SectionType _type;      // section type
        private bool _is_isotopic;      // indicates weather I1 = I2

        // GEOMETRY
        private MFrame _material_frame;  // section frame about centroid (G) and principal axes {d3,d1,d2}

        // GEOMETRIC PROPERTIES
        private double _S;              // section [m2]
        private double _I1;             // area moment of inertia [m4] around first material axis (d1)
        private double _I2;             // area moment of inertia [m4] around second material axis (d2)
        private double _J;              // area moment of inertia [m4] around third material axis (d3)
        
        #endregion

        #region PROPERTIES
        /// <summary>
        /// Section type.
        /// </summary>
        public SectionType SectionType
        {
            get { return _type; }
            protected set { _type = value; }
        }

        /// <summary>
        /// Material frame or pincipal axes frame {d3,d1,d2}
        /// </summary>
        public MFrame MaterialFrame
        {
            get { return _material_frame; }
            protected set
            {
                _material_frame = value;
            }
        }

        /// <summary>
        /// Area in [m2]
        /// </summary>
        public double S
        {
            get { return _S; }
            protected set { _S = value; }
        }
        
        /// <summary>
        /// Area moment of inertia [m4] around first material axis (d1)
        /// </summary>
        public double I1
        {
            get { return _I1; }
            protected set { _I1 = value; }
        }

        /// <summary>
        /// Area moment of inertia [m4] around second material axis (d2)
        /// </summary>
        public double I2
        {
            get { return _I2; }
            protected set { _I2 = value; }
        }

        /// <summary>
        /// Area moment of inertia [m4] around third material axis (d3)
        /// </summary>
        public double J
        {
            get { return _J; }
            protected set { _J = value; }
        }

        #endregion

        #region CONSTRUCTORS
        public Section(SectionType type)
        {
            SectionType = type;
        }

        public static Section RectangularSection(double b1, double b2)
        {
            Section section_prop = new Section(SectionType.Rectangular);

            // outline
            section_prop.MaterialFrame = MFrame.XY;

            // section properties
            section_prop.S = b1 * b2;
            section_prop.I1 = b1 * Math.Pow(b2, 3) / 12;
            section_prop.I2 = b2 * Math.Pow(b1, 3) / 12;

            double a = Math.Max(b1, b2);
            double b = Math.Min(b1, b2);
            section_prop.J = a * Math.Pow(b, 3) * (0.333 - 0.21 * (b / a) * (1 - Math.Pow(b / a, 4) / 12));

            //Rhino.RhinoApp.WriteLine("I1 = " + I1);
            //Rhino.RhinoApp.WriteLine("I2 = " + I2);
            //Rhino.RhinoApp.WriteLine("J = " + J);

            return section_prop;
        }

        #endregion

        #region METHODS
        // return the strains in the section frame
        public void Strains(double N, double M1, double M2, double Q)
        {
            throw new NotImplementedException();
        }
        
        // return Von Mises criterion
        public void VonMises(double N, double M1, double M2, double Q)
        {
            throw new NotImplementedException();
        }

        // return Tresca Criterion
        public void Tresca(double N, double M1, double M2, double Q)
        {
            throw new NotImplementedException();
        }

        public Section DeepCopy()
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}
