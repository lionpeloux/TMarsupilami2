
using TMarsupilami.MathLib;

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{

    /// <summary>
    /// Types of applied displacements.
    /// </summary>
    public enum AppliedDisplacementType
    {    
        TranslationOnly,
        RotationOnly,
        Combined,
    }

    /// <summary>
    /// Wether the applied displacement is set in the material frame (t, d1, d2) or in the global coordinate system (x, y, z).
    /// </summary>
    public enum CoordinateSystem
    {
        Material,
        Global,
    }

    /// <summary>
    /// Element's local / material axis
    /// </summary>
    public enum MaterialAxis
    {
        d1,
        d2,
        t,
    }

    /// <summary>
    /// Element's global axis
    /// </summary>
    public enum GlobalAxis
    {
        x,
        y,
        z
    }

    public class AppliedDisplacement : Constraint
    {
        #region FIELD
        protected AppliedDisplacementType _type;    // APD type
        protected CoordinateSystem _cs;             // APD coordinate system
        protected CurvedBeam _element;              // holds a reference to the concerned element
        protected int nj;                           // the element's node index concerned by the AD
        protected MFrame _mframe_apd;                // the new material frame
        #endregion

        #region PROPERTIES

        /// <summary>
        /// Applied displacement type.
        /// </summary>
        public AppliedDisplacementType Type
        {
            get { return _type; }
            protected set { _type = value; }
        }

        /// <summary>
        /// The coordinate system in which the applied displacement is formulated.
        /// Can be either the `Material` (d1, d2, t) or `Global` (x, y, z) coordinate system.
        /// </summary>
        public CoordinateSystem CoordinateSystem
        {
            get { return _cs; }
            protected set { _cs = value; }
        }

        /// <summary>
        /// The element concerned by the applied displacement.
        /// </summary>
        public CurvedBeam Element
        {
            get { return _element; }
            protected set { _element = value; }
        }

        /// <summary>
        /// The material frame after the given applied displacement.
        /// </summary>
        public MFrame MaterialFrame
        {
            get { return _mframe_apd; }
            protected set { _mframe_apd = value; }
        }

        #endregion

        #region CONSTRUCTORS
        protected AppliedDisplacement(ref CurvedBeam element, int nj, MFrame mframe_apd, AppliedDisplacementType type, CoordinateSystem cs) : base()
        {
            ConstraintType = ConstraintType.AppliedDisplacement;
            Element = element;
            this.nj = nj;
            MaterialFrame = mframe_apd;
            Type = type;
            CoordinateSystem = cs;
        }
        #endregion

        public static Constraint AddAppliedDisplacement(ref CurvedBeam element, int nj, MVector translation, CoordinateSystem cs)
        {
            MFrame mframe_apd = element.MaterialFrame[nj];

            if (cs == CoordinateSystem.Material)
            {
                mframe_apd.Origin += translation.X * mframe_apd.XAxis
                                    + translation.Y * mframe_apd.YAxis
                                    + translation.Z * mframe_apd.ZAxis;
            }
            else
            {
                mframe_apd.Origin += translation;
            }

            AppliedDisplacement apd = new AppliedDisplacement(ref element, nj, mframe_apd, AppliedDisplacementType.TranslationOnly, cs);
            return apd;
        }
        public static Constraint AddAppliedDisplacement(ref CurvedBeam element, int nj, double angle, MaterialAxis axis)
        {
            MFrame mframe_apd = element.MaterialFrame[nj];

            switch (axis)
            {
                case MaterialAxis.d1:
                    mframe_apd = Rotation.Rotate(mframe_apd, angle, mframe_apd.XAxis);
                    break;
                case MaterialAxis.d2:
                    mframe_apd = Rotation.Rotate(mframe_apd, angle, mframe_apd.YAxis);
                    break;
                case MaterialAxis.t:
                    mframe_apd = Rotation.Rotate(mframe_apd, angle, mframe_apd.ZAxis);
                    break;
            }

            AppliedDisplacement apd = new AppliedDisplacement(ref element, nj, mframe_apd, AppliedDisplacementType.RotationOnly, CoordinateSystem.Material);
            return apd;
        }
        public static Constraint AddAppliedDisplacement(ref CurvedBeam element, int nj, double angle, GlobalAxis axis)
        {
            MFrame mframe_apd = element.MaterialFrame[nj];

            switch (axis)
            {
                case GlobalAxis.x:
                    mframe_apd = Rotation.Rotate(mframe_apd, angle, MVector.XAxis);
                    break;
                case GlobalAxis.y:
                    mframe_apd = Rotation.Rotate(mframe_apd, angle, MVector.YAxis);
                    break;
                case GlobalAxis.z:
                    mframe_apd = Rotation.Rotate(mframe_apd, angle, MVector.ZAxis);
                    break;
            }

            AppliedDisplacement apd = new AppliedDisplacement(ref element, nj, mframe_apd, AppliedDisplacementType.RotationOnly, CoordinateSystem.Global);
            return apd;
        }

        public override string ToString()
        {
            return "[APPLIED DISPLACEMENT] : " + Type;
        }        
        public override void Init()
        {
            // penser à faire la modification de x automatiquement via l'accesseur MaterialFrame ?
            Element.x[nj] = MaterialFrame.Origin;
            Element.MaterialFrame[nj] = MaterialFrame;    
        }
    }

    
}
