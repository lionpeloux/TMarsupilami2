
namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    public enum ConstraintType
    {
        BoundaryCondition,
        Joint,
        AppliedDisplacement,
    }
    
    /*
    public enum KinematicConstraintType
    {
        Spherical,      // fixed position | free rotation
        Clamped,        // fixed position | fixed rotation
        Cylindrical,    // Slides and rotates freely around an axis 
    }
    public enum MechanicalConstraintType
    {
        Spring,
        PointAttractor,
        PlaneAttractor,
        SurfaceAttractor
    }
    public enum LinkConstraintType
    {
        SingleJoint,
        ExcentricSingleJoint,
        MultipleJoint
    }
    */

    public abstract class Constraint : Relation
    {
        #region FIELD
        private ConstraintType _type;           // constraint type
        #endregion

        #region PROPERTIES

        /// <summary>
        /// Constraint type.
        /// </summary>
        public ConstraintType ConstraintType
        {
            get { return _type; }
            protected set { _type = value; }
        }
        #endregion

        public abstract override string ToString(); // A UTILIER POUR AFFICHER LES INFOS D'UNE CONTRAINTE 

        // Spécifique DR à mettre dans des interfaces.
        // peut-être à éclater dans les sous types (BoundaryCondition, AppliedDisplacement, ...)
        public virtual void Init()
        { }
        public virtual void Enforce_Mr()
        { }
        public virtual void Enforce_Qr()
        { }
        public virtual void Enforce_Fr()
        { }

    }

}
