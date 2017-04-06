using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Display;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.CoreLib3;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh.Type
{
    public class GH_MBeam : GH_Goo<Beam>
    {

        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Marsupilami Beam."; } }

        public override string TypeName { get { return "Beam"; } }


        #endregion

        #region CONSTRUCTOR
        public GH_MBeam() { }
        public GH_MBeam(GH_MBeam ghBeam)
        {
            this.Value = ghBeam.Value;
        }
        public GH_MBeam(Beam beam)
        {
            this.Value = beam;
        }
        #endregion

        #region INSTANCE METHODS

        public override object ScriptVariable()
        {
            return this.Value;
        }
        public override string ToString()
        {
            return Value.ToString();
        }
        public override IGH_Goo Duplicate()
        {
            return new GH_MBeam(this);
        }

        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();

            if (type == typeof(Beam))
            {
                this.Value = (Beam)source;
                return true;
            }

            if (type == typeof(GH_MBeam))
            {
                this.Value = ((GH_MBeam)source).Value;
                return true;
            }

            var beam = source as Beam;
            if (beam != null)
            {
                this.Value = beam;
                return true;
            }

            return false;
        }
        public override bool CastTo<T>(ref T target)
        {
            var type = typeof(T);

            if (type.IsAssignableFrom(typeof(Beam)))
            {
                object ptr = this.Value;
                target = (T)ptr;
                return true;
            }

            if (type.IsAssignableFrom(typeof(GH_MBeam)))
            {
                object ptr = new GH_MBeam(this);
                target = (T)ptr;
                return true;
            }

            return false;
        }
        #endregion

    }
}
