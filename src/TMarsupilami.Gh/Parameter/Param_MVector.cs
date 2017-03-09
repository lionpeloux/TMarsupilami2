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
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Expressions;
using System.Windows.Forms;
using System.Runtime.CompilerServices;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Properties;
using GH_IO.Serialization;

namespace TMarsupilami.Gh.Parameter
{
    public class Param_MVector : GH_Param<GH_MVector>
    {

        // Fields
        private bool m_unitize;

        public Param_MVector()
          : base("Vector", "MVector", "Contains a collection of 3d vectors.", "TMarsupilami", "Params", GH_ParamAccess.item)
        {
            this.m_unitize = false;
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{90D59027-11FB-4AE7-827D-7009CF17F8B6}"); }
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

        public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
        {
            base.AppendAdditionalMenuItems(menu);
            ToolStripItem item = GH_DocumentObject.Menu_AppendItem(menu, "Unitize", new EventHandler(this.Menu_UnitizeClicked), Resources.Normalise_16x16, true, this.Unitize);
            item.ToolTipText = "Unitize all vectors in this parameter";
            GH_DocumentObject.Menu_MoveItem(item, new string[] { "Simplify", "Graft", "Flatten", "Reverse" });
        }
        private void Menu_UnitizeClicked(object sender, EventArgs e)
        {
            this.RecordUndoEvent("Unitize");
            this.Unitize = !this.Unitize;
            if (this.Kind == GH_ParamKind.output)
            {
                this.ExpireOwner();
            }
            this.ExpireSolution(true);
        }
        protected override void OnVolatileDataCollected()
        {
            base.OnVolatileDataCollected();
            if (this.m_unitize)
            {
                foreach (List<GH_MVector> list in base.m_data.Branches)
                {
                    for (int i = 0; i < list.Count; i++)
                    {
                        if (list[i] != null)
                        {
                            MVector vector = list[i].Value;
                            vector.Normalize();
                            list[i] = new GH_MVector(vector);
                        }
                    }
                }
            }
        }

        public override GH_StateTagList StateTags
        {
            get
            {
                GH_StateTagList stateTags = base.StateTags;
                if (this.Unitize)
                {
                    stateTags.Add(new GH_StateTag_Unitize());
                }
                return stateTags;
            }

        }
        internal class GH_StateTag_Unitize : GH_StateTag
        {
            public override string Name
            {
                get
                {
                    return "Unitize";
                }
            }
            public override string Description
            {
                get
                {
                    return "Vectors inside this parameter are unitized";
                }
            }
            public override System.Drawing.Bitmap Icon
            {
                get
                {
                    return Resources.Normalise_16x16;
                }
            }
            public override void Render(System.Drawing.Graphics graphics)
            {
                this.RenderTagBlankIcon(graphics, new GH_StateTag.DrawCallback(this.RenderWave));
            }
            private void RenderWave(System.Drawing.Graphics graphics, double alpha)
            {
                float x = (float)this.Stage.X + 1.5f;
                float y = (float)this.Stage.Y + 12f;
                System.Drawing.Drawing2D.GraphicsPath path = new System.Drawing.Drawing2D.GraphicsPath();
                path.AddLine(x + 0f, y + 0f, x + 0f, y - 2f);
                path.AddBezier(x + 0f, y - 2f, x + 3f, y - 2f, x + 2.5f, y - 8f, x + 6f, y - 8f);
                path.AddBezier(x + 6f, y - 8f, x + 9.5f, y - 8f, x + 9f, y - 2f, x + 12f, y - 2f);
                path.AddLine(x + 12f, y - 2f, x + 12f, y - 0f);
                path.AddBezier(x + 12f, y - 0f, x + 7f, y - 0f, x + 8f, y - 6f, x + 6f, y - 6f);
                path.AddBezier(x + 6f, y - 6f, x + 4f, y - 6f, x + 5f, y - 0f, x + 0f, y - 0f);
                this.RenderFreeformIcon(graphics, path);
                path.Dispose();
            }
        }
        public bool Unitize
        {
            get
            {
                return this.m_unitize;
            }
            set
            {
                this.m_unitize = value;
            }
        }

        public override bool Write(GH_IWriter writer)
        {
            if (this.Unitize)
            {
                writer.SetBoolean("Unitize", true);
            }
            return base.Write(writer);
        }
        public override bool Read(GH_IReader reader)
        {
            this.Unitize = false;
            reader.TryGetBoolean("Unitize", ref this.m_unitize);
            return base.Read(reader);
        }
        public override void RemoveEffects()
        {
            base.RemoveEffects();
            this.m_unitize = false;
        }

    }
}
