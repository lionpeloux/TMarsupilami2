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
    public class Param_MBeam : GH_Param<GH_MBeam>
    {

        // Fields
        private bool m_hidden;

        public Param_MBeam()
          : base("Beam", "MBeam", "Represent a beam element.", "TMarsupilami", "Params", GH_ParamAccess.item)
        {
            this.m_hidden = false;
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("{6519BDE8-90A5-4879-81CC-6B0AC71CEC71}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
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

        // IGH_PreviewObject
        public bool Hidden
        {
            get
            {
                return this.m_hidden;
            }
            set
            {
                this.m_hidden = value;
            }
        }
        public bool IsPreviewCapable
        {
            get
            {
                return false;
            }
        }
    }
}
