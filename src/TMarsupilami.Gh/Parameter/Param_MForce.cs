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
    public class Param_MForce : GH_Param<GH_MForce>, IGH_PreviewObject
    {

        // Fields
        private bool m_hidden;

        public Param_MForce()
          : base("Force", "MForce", "Contains a collection of forces.", "TMarsupilami", "Params", GH_ParamAccess.item)
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
                return Resources.MVector;
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
                return true;
            }
        }
        public BoundingBox ClippingBox
        {
            get
            {
                return this.Preview_ComputeClippingBox();
            }
        }
        public void DrawViewportMeshes(IGH_PreviewArgs args)
        {
        }
        public void DrawViewportWires(IGH_PreviewArgs args)
        {
            // specific preview of the data relative to the parameter component

            Color color;
            if (Attributes.GetTopLevel.Selected)
            {
                color = Color.Yellow;
            }
            else
            {
                color = Color.Red;
            }

            int dataCount = base.m_data.DataCount;
            if (dataCount != 0)
            {
                for (int b = 0; b < m_data.Branches.Count; b++)
                {
                    var branch = m_data.Branches[b];
                    for (int i = 0; i < branch.Count; i++)
                    {
                        var force = branch[i];
                        if (force != null)
                        {
                            force.DrawForce(args.Display, CentralSettings.PreviewPlaneRadius, color);
                        }
                    }
                }
            }
        }   
    }
}
