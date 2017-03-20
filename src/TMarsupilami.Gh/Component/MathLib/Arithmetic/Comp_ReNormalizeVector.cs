using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using TMarsupilami.MathLib;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ReNormalizeVector : GH_Component
    {

        public Comp_ReNormalizeVector()
          : base("Renormalize a Vector", "ReNorm",
              "Fast normalization of a vector that is already almost normalized.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.quarternary;
            }
        }
        //protected override System.Drawing.Bitmap Icon
        //{
        //    get
        //    {
        //        return Resources.ZRotatePlane_Diff;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{4EC8C087-096B-4B9E-9CB6-5B9814DBF293}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector(s)", "v", "Vectors(s) to re-normalize.", GH_ParamAccess.list);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector(s)", "v1", "The re-normalized vector(s).", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vector(s)", "v2", "The re-normalized vector(s).", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vector(s)", "v3", "The re-normalized vector(s).", GH_ParamAccess.list);

        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var vector_list = new List<Vector3d>();

            if (!DA.GetDataList(0, vector_list)) { return; }

            int n = vector_list.Count;
            var vectors = vector_list.Cast();
            var v1 = new MVector[n];
            var v2 = new MVector[n];
            var v3 = new MVector[n];

            var watch = Stopwatch.StartNew();
            for (int i = 0; i < vectors.Count; i++)
            {
                v1[i] = MVector.ReNormalize_R2(vectors[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time 1 = " + (watch.Elapsed.TotalMilliseconds * 1000) + " us");

            watch = Stopwatch.StartNew();
            for (int i = 0; i < vectors.Count; i++)
            {
                v2[i] = MVector.ReNormalize_R2_N1(vectors[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time 2 = " + (watch.Elapsed.TotalMilliseconds * 1000) + " us");

            watch = Stopwatch.StartNew();
            for (int i = 0; i < vectors.Count; i++)
            {
                v3[i] = MVector.ReNormalize_R2_N2(vectors[i]);
            }
            watch.Stop();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed time 3 = " + (watch.Elapsed.TotalMilliseconds * 1000) + " us");

            DA.SetDataList(0, v1.Cast());
            DA.SetDataList(1, v2.Cast());
            DA.SetDataList(2, v3.Cast());

        }
    }
}
