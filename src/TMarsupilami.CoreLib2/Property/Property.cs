

namespace TMarsupilami.CoreLib2
{
    public enum PropertyType
    {
        Section,
        Material,
    }
    public abstract class Property
    {
        #region FIELDS
        private int _id;                // propertie ID
        private string _name;           // propertie name
        private PropertyType _type;     // propertie type
        #endregion

        #region PROPERTIES
        /// <summary>
        /// Properties unique identifier.
        /// </summary>
        public int Id
        {
            get { return _id; }
            protected set { _id = value; }
        }

        /// <summary>
        /// Properties name.
        /// </summary>
        public string Name
        {
            get { return _name; }
            protected set { _name = value; }
        }

        /// <summary>
        /// Properties type
        /// </summary>
        public PropertyType PropertyType
        {
            get { return _type; }
            protected set { _type = value; }
        }

        #endregion

        #region PROPERTIES
        protected Property(int id, PropertyType type) 
        {
            Id = id;
            Name = "";
            PropertyType = type;
        }
        protected Property(int id, string name, PropertyType type)
        {
            Id = id;
            Name = name;
            PropertyType = type;
        }
        #endregion
    }
}
