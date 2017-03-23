

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    // interface pour les relations à 2 noeuds
    public enum RelationType
    {
        Element,
        Constraint,
    }
    public abstract class Relation
    {
        private int _id;                // relation ID
        private string _name;           // relation name
        private RelationType _type;     // relation type
        private int _cardinality;       // relation cardinality (1-N)

        /// <summary>
        /// Relation unique identifier.
        /// </summary>
        public int Id
        {
            get { return _id; }
            protected set { _id = value; }
        }

        /// <summary>
        /// Relation name.
        /// </summary>
        public string Name
        {
            get { return _name; }
            protected set { _name = value; }
        }

        /// <summary>
        /// Relation type.
        /// </summary>
        public RelationType RelationType
        {
            get { return _type; }
            protected set { _type = value; }
        }

        /// <summary>
        /// Relation cardinality. The numbers of frames in interaction (>1).
        /// </summary>
        public int RelationCardinality
        {
            get { return _cardinality; }
            protected set { _cardinality = value; }
        }
    }

}
