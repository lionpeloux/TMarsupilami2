using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using System.Drawing;
using GH_IO.Types;
using Grasshopper;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Type;
using TMarsupilami.CoreLib3;

namespace TMarsupilami.Gh.Component
{
    public class Comp_SectionProperties : GH_Component
    {

        public Comp_SectionProperties()
          : base("Section Properties", "SProp",
              "Extract the geometric properties of a planar section.",
              "TMarsupilami", "Section")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.primary;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{150DF5F8-42B1-457C-AD14-F74E5CA62B6E}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddSurfaceParameter("The section.", "srf", "The section to analyse as a planar surface", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frame", "F", "The principal frame of the section (XAxis and YAxis are the first and second principal axis, ZAxis is perpendicular to the section).", GH_ParamAccess.item);
            pManager.AddNumberParameter("Area", "A", "The area of the section.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Moment of Inertia", "I1", "The moment of inertia with respect to the first principal axis of the section frame (F.XAxis).", GH_ParamAccess.item);
            pManager.AddNumberParameter("Moment of Inertia", "I2", "The moment of inertia with respect to the second principal axis of the section frame (F.YAxis).", GH_ParamAccess.item);
            pManager.AddNumberParameter("Moment of Inertia", "I3", "The moment of inertia with respect to the third principal axis of the section frame (F.ZAxis).", GH_ParamAccess.item);
            pManager.AddMatrixParameter("Tensor of Inertia", "I", "The tensor of inertia of the section as a 3x3 matrix given in the section frame coordinate system (F).", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double tolerance = 1e-5;
            Surface srf = null;

            if (!DA.GetData(0, ref srf)){ return; }

            // Make sure the surface is planar
            Plane surfacePlane;
            if(!srf.TryGetPlane(out surfacePlane, tolerance)) { return; };

            // Position the surface plane at the centroid
            var amp = AreaMassProperties.Compute(srf);
            var centroid = amp.Centroid;
            var a = centroid.X;
            var b = centroid.Y;
            var c = centroid.Z;
            surfacePlane.Origin = centroid;

            // Get section area
            var A = amp.Area;

            // Compute the inertia tensor in the world coordinate system ROxyz = {O,ex,ey,ez}
            var IO_xx = amp.WorldCoordinatesMomentsOfInertia.X;
            var IO_yy = amp.WorldCoordinatesMomentsOfInertia.Y;
            var IO_zz = amp.WorldCoordinatesMomentsOfInertia.Z;
            var IO_xy = -amp.WorldCoordinatesProductMoments.X;
            var IO_xz = -amp.WorldCoordinatesProductMoments.Z;
            var IO_yz = -amp.WorldCoordinatesProductMoments.Y;

            // Compute the inertia tensor in the world coordinate system RGxyz = {G,ex,ey,ez} translated to the centroid (G)
            // this requires Huygens theorem
            var IG_xx = IO_xx - A * (b * b + c * c);
            var IG_yy = IO_yy - A * (a * a + c * c);
            var IG_zz = IO_zz - A * (a * a + b * b);
            var IG_xy = IO_xy + A * a * b;
            var IG_xz = IO_xz + A * a * c;
            var IG_yz = IO_yz + A * b * c;

            var IGxyz = new Matrix(3, 3);
            IGxyz[0, 0] = IG_xx;    IGxyz[0, 1] = IG_xy;    IGxyz[0, 2] = IG_xz;
            IGxyz[1, 0] = IG_xy;    IGxyz[1, 1] = IG_yy;    IGxyz[1, 2] = IG_yz;
            IGxyz[2, 0] = IG_xz;    IGxyz[2, 1] = IG_yz;    IGxyz[2, 2] = IG_zz;

            // #####################
            // first change of basis
            // #####################

            // We express the inertia tensor in the surfacePlane coordinate system RGijk = {G, ei, ej, ek}.
            // We know that ek is normal to the surface and that ei and ej lie on the surface.
            // Thus the tensor components : IG_ik = IG_jk = 0 and IG_kk will be the polar inertia and third eigen value

            // We call P1 the transformation matrix from RGijk to RGxyz, that is the columns of P1 are {ei,ej,ek}.
            // We call TP1 the transpose of P1.
            var ei = surfacePlane.XAxis;
            var ej = surfacePlane.YAxis;
            var ek = surfacePlane.ZAxis;
            var P1 = new Matrix(3, 3);
            P1[0, 0] = ei.X;       P1[0, 1] = ej.X;       P1[0, 2] = ek.X;
            P1[1, 0] = ei.Y;       P1[1, 1] = ej.Y;       P1[1, 2] = ek.Y;
            P1[2, 0] = ei.Z;       P1[2, 1] = ej.Z;       P1[2, 2] = ek.Z;
            
            // The Inertia tensor in RGijk is now : IGijk = TP1 x IG_RGxyz x P1
            var TP1 = P1.Duplicate(); TP1.Transpose();
            var IGijk = TP1 * IGxyz * P1;
            var IG_ii = IGijk[0, 0];
            var IG_jj = IGijk[1, 1];
            var IG_kk = IGijk[2, 2];
            var IG_ij = IGijk[0, 1];

            // #####################
            // second change of basis
            // #####################

            // We know the third eigen value and corresponding eigen vector for IGxyz :
            var e3 = ek;
            var IG_33 = IG_kk;

            // We have now a simpler 2x2 diagonalisation problem to solve, that is a rotation in the plane perpendicular to e3 = ek
            // The remaining eigen values satisfy the following equation : l^2 - (IG_ii + IG_jj)l + (IG_ii*IG_jj) - IG_ij^2 = 0
            var D = Math.Sqrt((IG_ii - IG_jj) * (IG_ii - IG_jj) + 4 * IG_ij * IG_ij);
            var IG_11 = 0.5 * ((IG_ii + IG_jj) + D); // strong axis 
            var IG_22 = 0.5 * ((IG_ii + IG_jj) -D); // weak axis (IG_22 < IG_11)

            // The eigen vector e1 associated with eigen value IG_11 is given by :
            var e1 = 2 * IG_ij * ei + (IG_jj - IG_ii + D) * ej;
            var b1 = e1.Unitize();

            // The eigen vector e2 associated with eigen value IG_22 is given by :
            var e2 = 2 * IG_ij * ei + (IG_jj - IG_ii - D) * ej;
            var b2 = e2.Unitize();

            if (b1)
            {
                e2 = Vector3d.CrossProduct(e3, e1);
            }
            else if (b2)
            {
                e1 = Vector3d.CrossProduct(e2, e3);
            }
            var sectionPlane = new Plane(centroid, e1, e2);

            // We call P2 the transformation matrix from RG123 to RGxyz, that is the columns of P2 are {e1,e2,e3}.
            // We call TP2 the transpose of P2.
            var P2 = new Matrix(3, 3);
            P2[0, 0] = e1.X; P2[0, 1] = e2.X; P2[0, 2] = e3.X;
            P2[1, 0] = e1.Y; P2[1, 1] = e2.Y; P2[1, 2] = e3.Y;
            P2[2, 0] = e1.Z; P2[2, 1] = e2.Z; P2[2, 2] = e3.Z;
            var TP2 = P2.Duplicate(); TP2.Transpose();
            var IG123 = TP2 * IGxyz * P2;

            // the previous computation is not necessary has we already know that IG123 = diag(IG_11, IG_22, IG_33)
            // but it's a way to check the results
            //var IG123 = new Matrix(3, 3);
            //IG123[0, 0] = IG_11;
            //IG123[1, 1] = IG_22;
            //IG123[2, 2] = IG_33;

            DA.SetData(0, sectionPlane.Cast());
            DA.SetData(1, A);
            DA.SetData(2, IG_11);
            DA.SetData(3, IG_22);
            DA.SetData(4, IG_33);
            DA.SetData(5, IG123);
        }

    }
}
