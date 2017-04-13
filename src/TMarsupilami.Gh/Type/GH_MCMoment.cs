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
    public class GH_MCMoment : GH_Goo<CMoment>
    {
        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Concentrated Moment."; } }

        public override string TypeName { get { return "MCMoment"; } }

        #endregion

        #region CONSTRUCTOR
        public GH_MCMoment() { }
        public GH_MCMoment(GH_MCMoment ghMoment)
        {
            this.Value = ghMoment.Value;
        }
        public GH_MCMoment(CMoment moment)
        {
            this.Value = moment;
        }
        #endregion

        #region INSTANCE METHODS
        public override object ScriptVariable()
        {
            return this.Value;
        }
        public override string ToString()
        {
            return this.Value.ToString();
        }
        public override IGH_Goo Duplicate()
        {
            return new GH_MCMoment(this);
        }
        
        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();

            if (type == typeof(CMoment))
            {
                this.Value = (CMoment)source;
                return true;
            }

            if (type == typeof(GH_MCMoment))
            {
                this.Value = ((GH_MCMoment)source).Value;
                return true;
            }

            return false;
        }
        public override bool CastTo<T>(ref T target)
        {
            if (typeof(T).IsAssignableFrom(typeof(MVector)))
            {
                object ptr = this.Value.Value;
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_MVector)))
            {
                object ptr = new GH_MVector(this.Value.Value);
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(Vector3d)))
            {
                object ptr = this.Value.Value.Cast();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Vector)))
            {
                object ptr = new GH_Vector(this.Value.Value.Cast());
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(double)))
            {
                object ptr = this.Value.Value.Length();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Number)))
            {
                object ptr = new GH_Number(this.Value.Value.Length());
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(Plane)))
            {
                object ptr = this.Value.LocalFrame.Cast();
                target = (T)ptr;
                return true;
            }

            if (typeof(T).IsAssignableFrom(typeof(GH_Plane)))
            {
                object ptr = new GH_Plane(this.Value.LocalFrame.Cast());
                target = (T)ptr;
                return true;
            }

            return false;
        }

    }
        #endregion
}
