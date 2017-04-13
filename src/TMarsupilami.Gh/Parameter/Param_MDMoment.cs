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
    public class Param_MDMoment : GH_Param<GH_MDMoment>
    {

        // Fields
        private bool m_hidden;

        public Param_MDMoment()
          : base("Distributed Moment", "DMoment", "Contains a collection of distributed moments.", "TMarsupilami", "Params", GH_ParamAccess.item)
        {
            this.m_hidden = false;
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("{C99042DA-2B33-492C-A739-D306C73A4FB6}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.secondary; }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.DMoment;
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
