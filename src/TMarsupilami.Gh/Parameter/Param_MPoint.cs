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
    public class Param_MPoint : GH_Param<GH_MPoint>, IGH_PreviewObject
    {

        // Fields
        private bool m_hidden;

        public Param_MPoint()
          : base("Point", "MPoint", "Contains a collection of 3d points.", "TMarsupilami", "Params", GH_ParamAccess.item)
        {
            this.m_hidden = false;
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("{29C0F753-3AAF-4A24-BF5E-F7C16C435F66}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.MPoint;
            }
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
            Color color;
            if (this.Attributes.GetTopLevel.Selected)
            {
                color = args.WireColour_Selected;
            }
            else
            {
                color = args.WireColour;
            }


            int dataCount = base.m_data.DataCount;
            if (dataCount != 0)
            {
                for (int b = 0; b < m_data.Branches.Count; b++)
                {
                    var branch = m_data.Branches[b];
                    for (int i = 0; i < branch.Count; i++)
                    {
                        var point = branch[i];
                        if (point != null)
                        {
                            point.DrawMPoint(args.Display, 5, color);
                        }
                    }
                }
            }
        }
    }
}
