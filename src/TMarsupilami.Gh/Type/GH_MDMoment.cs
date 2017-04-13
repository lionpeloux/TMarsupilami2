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
    public class GH_MDMoment : GH_Goo<DMoment>
    {
        #region FIELDS

        public override bool IsValid { get { return true; } }

        public override string TypeDescription { get { return "A Distributed Force."; } }

        public override string TypeName { get { return "MDMoment"; } }

        #endregion

        #region CONSTRUCTOR
        public GH_MDMoment() { }
        public GH_MDMoment(GH_MDMoment ghForce)
        {
            this.Value = ghForce.Value;
        }
        public GH_MDMoment(DMoment force)
        {
            this.Value = force;
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
            return new GH_MDMoment(this);
        }
        
        public override bool CastFrom(object source)
        {
            if (source == null) { return false; }

            var type = source.GetType();

            if (type == typeof(DMoment))
            {
                this.Value = (DMoment)source;
                return true;
            }

            if (type == typeof(GH_MDMoment))
            {
                this.Value = ((GH_MDMoment)source).Value;
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
