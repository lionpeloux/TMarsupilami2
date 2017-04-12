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
    public class Param_MCMoment : GH_Param<GH_MCMoment>
    {

        // Fields
        private bool m_hidden;

        public Param_MCMoment()
          : base("Concentrated Moment", "CMoment", "Contains a collection of concentrated moments.", "TMarsupilami", "Params", GH_ParamAccess.item)
        {
            this.m_hidden = false;
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("{BFDFCBDC-1686-4FE5-92E0-AB872ACF8700}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.CMoment;
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
