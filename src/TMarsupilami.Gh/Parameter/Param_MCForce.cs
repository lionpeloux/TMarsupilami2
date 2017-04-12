using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.Gh.Type;
using Rhino.Geometry;
using Rhino.Collections;
using Grasshopper;
using Rhino.Display;
using System.Drawing;
using TMarsupilami.Gh.Properties;

namespace TMarsupilami.Gh.Parameter
{
    public class Param_MCForce : GH_Param<GH_MCForce>
    {

        // Fields
        private bool m_hidden;

        public Param_MCForce()
          : base("Concentrated Force", "CForce", "Contains a collection of concentrated forces.", "TMarsupilami", "Params", GH_ParamAccess.item)
        {
            this.m_hidden = false;
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("{CBA95A37-FF7E-4B58-8490-09F3ED8C4E29}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.CForce;
            }
        }
        public override string ToString()
        {
            return this.m_data.ToString();
        }

        public override bool Locked
        {
            get
            {
                return base.Locked;
            }
            set
            {
                if (base.Locked != value)
                {
                    base.Locked = value;
                }
            }
        }
        public override void ClearData()
        {
            base.ClearData();
        }
    }
}
