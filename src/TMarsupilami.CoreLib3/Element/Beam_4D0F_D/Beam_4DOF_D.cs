﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum Boundary
    {
        Start,
        End,
    }


    public sealed class Beam_4DOF_D : Beam
    {
        // Fext_g : beam load buffer 0
        // fext_g : beam load buffer 1
        // Mext_g : beam load buffer 2
        // mext_g : beam load buffer 3
        // Fext_m : beam load buffer 4
        // fext_m : beam load buffer 5
        // Mext_m : beam load buffer 6
        // mext_m : beam load buffer 7

        public MFrame[] MaterialFrames
        {
            get { return mframes; }
        }

        #region FIELDS

        // REST CONFIGURATION      
        private double[] l_0;                   // rest length of edges
        private double[] τ_0;                   // twist angle and rate of twist (τ) in rest configuration

        private MVector[] κb_mid_0;           // curvature binormal at mid-edge
        private MVector[] κb_g_0;             // curvature at a ghost vertex
        private MVector[] κb_h_l_0, κb_h_r_0;   // curvature at a left/right of a handle vertex

        // ACTUAL CONFIGURATION
        //private MVector[] e;                // edge : e[i] = x[i+1] - x[i]
        //private double[] l;                 // edge length : l[i] = |e[i]|

        public MVector[] t_mid;           // tangent vector at mid (u = t3_mid = ei/|ei|)
        public MFrame[] mframes_mid;

        private double[] ε;                 // axial strain : ε = l[i]/l[i]_0 - 1

        private MVector[] κb_mid;           // curvature binormal at mid-edge
        private MVector[] κb_g;             // curvature at a ghost vertex
        private MVector[] κb_h_l, κb_h_r;   // curvature at a left/right of a handle vertex

        public double[] τ;           // twist angle and rate of twist : τ[i] = twist[i] / l[i]
        public double[] τ_mid;                                 // torsional moment : Q = GJ.(τ-τ_0)
        public double[] τ_l, τ_r;                          // torsion moment at left/rigth of each node, ghost and handle

        // INTERNAL FORCES & MOMENTS
        public double[] N_mid;                                 // axial force : N = ES.ε
        public MVector[] N_l, N_r;                          // axial force at left/rigth of each node, ghost and handle

        private MVector[] V_M;                              // shear force (bending contribution) : V_M = M'
        private MVector[] V_Q;                              // shear force (torsion contribution) : V_Q = Qκb
        public MVector[] V_mid, V_l, V_r;                                // total shear force : V = V_M + V_Q
        private MVector[] V_M_g, V_M_h_l, V_M_h_r;          // shear (du au moment de flexion) aux ghost et à droite / gauche des handles
        private MVector[] V_Q_g, V_Q_h_l, V_Q_h_r;          // shear (du au moment de torsion) aux ghost et à droite / gauche des handles

        private double[] M1_h_l, M1_h_r, M2_h_l, M2_h_r;    // bending moment around d1 material axis : M1 = EI1.(κ1-κ1_0)
        public MVector[] M_g, M_h_l, M_h_r;

        public double[] Q_mid;                                 // torsional moment : Q = GJ.(τ-τ_0)
        public double[] Q_l, Q_r;                          // torsion moment at left/rigth of each node, ghost and handle

        // RESULTANTE FORCES (DR SPECIFIC ?? => si oui, à déplacer)
        private MVector[] Rint_x_axial, Rint_x_shear_M, Rint_x_shear_Q;
        private double[] Rint_θ_torsion_Q, Rint_θ_torsion_M;
        private MVector[] Rint_θ_bending;

        private MVector[] dθ;

        #endregion

        // pour l'instant, on passe l'ensemble des frames et des sections
        public Beam_4DOF_D(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, IEnumerable<Section> sections, IEnumerable<Material> materials, bool isClosed = false)
            :base(isClosed)
        {           
            // KINEMATIC CAPABILITIES
            TranslationalDOF = 3;
            RotationalDOF = 1;

            // MECHANICAL CAPABILITIES 
            IsCompressionCapable = true;
            IsTractionCapable = true;
            IsShearCapable = true;
            IsBendingCapable = true;
            IsTorsionCapable = true;

            // AUTOMATIC CREATION OF GHOST VERTICES
            CreateGhostVertices(restFrames, actualFrames, isClosed);

            // TOPOLOGY
            if (IsClosed)
            {
                ne = mframes.Length;
                nv_g = ne / 2;
                nv_h = ne / 2;
                nv = nv_g + nv_h;
            }
            else
            {
                ne = mframes.Length - 1;
                nv_g = ne / 2;
                nv_h = nv_g + 1;
                nv = nv_g + nv_h;
            }

            // PREFORM CHECKS
            if (mframes_0.Length != mframes_i.Length)
                throw new ArgumentOutOfRangeException("actualFrames", "actualFrames and restFrames must have the same number of items.");

            // INSTANCIATE INTERNAL ARRAYS
            CreateInternalArrays(nv, nv_h, nv_g, ne);

            // DEEP COPY
            this.sections = sections.ToArray();
            this.materials = materials.ToArray();

            for (int i = 0; i < nv_g; i++)
            {
                var section = this.sections[0];
                var material = this.materials[0];

                ES[i] = material.E * section.S;
                EI1[i] = material.E * section.I1;
                EI2[i] = material.E * section.I2;
                GJ[i] = material.G * section.J;
            }

            // SET CONFIGURATIONS
            SetRestConfig(mframes_0);
            SetInitialConfig(mframes);
        }
        private void CreateGhostVertices(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, bool isClosed)
        {
            var frames = restFrames.ToArray();
            int nv = frames.Length;
            int ne = IsClosed ? nv : nv - 1;

            var x = new MPoint[nv];
            var e = new MVector[ne];
            var u = new MVector[ne];
            var t = new MVector[nv];
            var κb = new MVector[nv];
            var l = new double[ne];
            var τ = new double[ne];

            // Interpolate restFrames
            Centerline.GetCurvature(frames, x, e, u, l, t, κb, isClosed);
            Centerline.ZAlignFrames(frames, t);
            Centerline.GetTwist(frames, l, τ, isClosed);
            mframes_0 = Centerline.Refine(frames, κb, τ, isClosed, 1);

            // Interpolate actualFrames
            frames = actualFrames.ToArray();
            Centerline.GetCurvature(frames, x, e, u, l, t, κb, isClosed);
            Centerline.ZAlignFrames(frames, t);
            Centerline.GetTwist(frames, l, τ, isClosed);
            mframes = Centerline.Refine(frames, κb, τ, isClosed, 1);
            mframes_i = mframes.DeepCopy();
        }
        private void CreateInternalArrays(int nv, int nv_h, int nv_g, int ne)
        {
            // SECTION & MATERIAL 
            ES = new double[nv_g];
            EI1 = new double[nv_g];
            EI2 = new double[nv_g];
            GJ = new double[nv_g];

            // EXTERNAL FORCES & MOMENTS
            Fext_g = new MVector[nv_h];
            Mext_m = new MVector[nv_h];
            fext_g = new MVector[nv_g];
            mext_m = new MVector[nv_g];

            // REACTION MOMENT AND FORCES
            Fr_g = new MVector[nv_h];
            Mr_m = new MVector[nv_h];

            // APPLIED LOADS
            loadManager = new BeamLoadManager(this);

            // DYNAMIC CONFIGURATION
            lm_x = new double[nv];
            lm_θ = new double[nv];
            ld_x = new double[nv];
            ld_θ = new double[nv];
            v_x = new MVector[nv];
            v_θ = new MVector[nv];
            a_x = new MVector[nv];
            a_θ = new MVector[nv];

            // REST CONFIGURATION
            l_0 = new double[ne];
            κb_mid_0 = new MVector[ne];
            κb_g_0 = new MVector[nv_g];        // tangente à un ghost node
            κb_h_l_0 = new MVector[nv_h];      // tangente à gauche d'un handle node (non défini en 0)
            κb_h_r_0 = new MVector[nv_h];      // tangente à droite d'un handle node (non défini en nv_h-1)
            τ_0 = new double[ne];

            // DEFORMED CONFIGURATION
            x = new MPoint[nv];
            e = new MVector[ne];
            l = new double[ne];

            t = new MVector[nv];
            t_mid = new MVector[ne];
            mframes_mid = new MFrame[ne];

            ε = new double[ne];

            κb_mid = new MVector[ne];
            κb_g = new MVector[nv_g];     // courbure 3pts aux ghost nodes
            κb_h_l = new MVector[nv_h];   // courbure à gauche d'un handle node (non défini en 0)
            κb_h_r = new MVector[nv_h];   // courbure à droute d'un handle node (non défini en nv_h-1)

            τ = new double[ne];
            τ_mid = new double[ne];
            τ_l = new double[nv];
            τ_r = new double[nv];

            // INTERNAL FORCES & MOMENTS
            N_mid = new double[ne];
            N_l = new MVector[nv];
            N_r = new MVector[nv];

            V_l = new MVector[nv];
            V_r = new MVector[nv];

            V_mid = new MVector[ne];
            V_M = new MVector[ne];
            V_M_g = new MVector[nv_g];
            V_M_h_l = new MVector[nv_h];
            V_M_h_r = new MVector[nv_h];
            V_Q = new MVector[ne];
            V_Q_g = new MVector[nv_g];
            V_Q_h_l = new MVector[nv_h];
            V_Q_h_r = new MVector[nv_h];

            M_g = new MVector[nv_g];   // moment à un ghost node
            M_h_l = new MVector[nv_h];   // moment à gauche d'un handle node
            M_h_r = new MVector[nv_h];   // moment à gauche d'un handle node

            Q_mid = new double[ne];
            Q_l = new double[nv];
            Q_r = new double[nv];

            // DR RESULTANT
            R_x = new MVector[nv];
            Rint_x = new MVector[nv];
            Rint_x_axial = new MVector[nv];
            Rint_x_shear_M = new MVector[nv];
            Rint_x_shear_Q = new MVector[nv];

            R_θ = new MVector[nv];
            Rint_θ = new MVector[nv];
            Rint_θ_torsion_Q = new double[nv];
            Rint_θ_torsion_M = new double[nv];
            Rint_θ_bending = new MVector[nv];

            // internal rotation to realign frames with the centerline at each x step.
            dθ = new MVector[nv];
    }

        public override string ToString()
        {
            return "Beam_4DOF_D";
        }

        private void SetRestConfig(MFrame[] restFrames)
        {
            for (int i = 0; i < nv; i++)
            {
                x[i] = restFrames[i].Origin;
            }
            Centerline.GetCurvature(x, e, t_mid, l_0, t, κb_g_0, IsClosed);
            Centerline.GetTwist(restFrames, l_0, τ_0, IsClosed);

            int index;
            MVector κb, κb_l, κb_r, d1, d2;

            // HANDLE

            // i = 0
            κb_r = (2 / l_0[0]) * MVector.CrossProduct(t[0], t_mid[0]);
            d1 = restFrames[0].XAxis;
            d2 = restFrames[0].YAxis;
            κb_h_r_0[0].X = κb_r * d1;
            κb_h_r_0[0].Y = κb_r * d2;

            for (int i = 1; i < nv_h-1; i++)
            {
                index = 2 * i;
                κb_l = (2 / l_0[index - 1]) * MVector.CrossProduct(t_mid[index - 1], t[index]);
                κb_r = (2 / l_0[index]) * MVector.CrossProduct(t[index], t_mid[index]);

                d1 = restFrames[index].XAxis;
                d2 = restFrames[index].YAxis;

                // WARNING : given in the material frame coordinate system
                κb_h_l_0[i].X = κb_l * d1;
                κb_h_l_0[i].Y = κb_l * d2;
                κb_h_r_0[i].X = κb_r * d1;
                κb_h_r_0[i].Y = κb_r * d2;
            }

            // i = nv_h-1
            κb_l = 2 / (l_0[ne - 1]) * MVector.CrossProduct(t_mid[ne - 1], t[nv - 1]);
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;
            κb_h_l_0[nv_h - 1].X = κb_l * d1;
            κb_h_l_0[nv_h - 1].Y = κb_l * d2;

            // GHOST
            for (int i = 0; i < nv_g; i++)
            {
                index = 2 * i + 1;
                κb = κb_g_0[i];

                d1 = restFrames[index].XAxis;
                d2 = restFrames[index].YAxis;

                // WARNING : given in the material frame coordinate system
                κb_g_0[i].X = κb * d1;
                κb_g_0[i].Y = κb * d2;
                κb_g_0[i].Z = 0;

                κb_mid_0[2 * i] = 0.5 * (κb_h_r_0[i] + κb_g[i]);
                κb_mid_0[2 * i + 1] = 0.5 * (κb_g[i] + κb_h_l_0[i + 1]);
            }
        }
        private void SetInitialConfig(MFrame[] initialFrames)
        {
            // The initial set of material frames impose the starting configuration.
            // Thus ti is not interpolated but immediately given by mframe ZAxis.

            // on initialise x et MaterialFrame avec les donvées de départ (on en fait une copie)
            for (int i = 0; i < nv; i++)
            {
                x[i] = initialFrames[i].Origin;
                mframes[i] = initialFrames[i];
            }
            Centerline.GetCurvature(x, e, t_mid, l, t, κb_g, IsClosed);
            Centerline.GetTwist(mframes, l, τ, IsClosed);
        }

        public override void Init()
        {
        }

        public override void Move_x(MVector[] dx)
        {
            // x = x + dx
            for (int i = 0; i < nv; i++)
            {
                x[i] = mframes[i].Origin + dx[i];
            }
            OnFramesTranslated(dx);

            Centerline.GetCurvature(x, e, t_mid, l, t, κb_g, IsClosed);
            if (!IsClosed) // assumes that edges are free (default) before imposing any tangent constraint
            {
                t[0] = e[0] / l[0];
                t[nv - 1] = e[ne - 1] / l[ne - 1];
            }

            // call for tangent constraint
            OnTangentsUpdated(t);

            // realign material frames to stick to the centerline (this is a rotation by dθ[i] = MVector.CrossProduct(mframes[i].ZAxis, t[i]);
            // if the tangent vector has not changed, the frame remains unchanged.
            for (int i = 0; i < nv; i++)
            {
                dθ[i] = MVector.CrossProduct(mframes[i].ZAxis, t[i]);
                ParallelTransportation.ZPT_Rotation(mframes[i], mframes[i].ZAxis, x[i], t[i], ref mframes[i]);
            }

            //OnFramesRotated(dθ);

            Centerline.GetTwist(mframes, l, τ, IsClosed);
        }
        public override void Move_θ(MVector[] dθ)
        {
            // θ = θ + dθ
            for (int i = 0; i < nv; i++)
            {
                mframes[i].ZRotate(dθ[i].Z);
            }
            //OnFramesRotated(dθ);
            Centerline.GetTwist(mframes, l, τ, IsClosed);
        }

        public override void Calculate_x()
        {
            UpdateBendingMoment();
            UpdateTwistingMoment();

            UpdateShearForce();
            UpdateAxialForce();

            UpdateInternalNodalMoment();
            UpdateInternalNodalForce();

            UpdateResultantNodalMoment();
            UpdateResultantNodalForce();
        }
        public override void Calculate_θ()
        {
            UpdateBendingMoment();
            UpdateTwistingMoment();

            UpdateShearForce();
            UpdateAxialForce();

            UpdateInternalNodalMoment();
            UpdateInternalNodalForce();

            UpdateResultantNodalMoment();
            UpdateResultantNodalForce();
        }

        public void ClearAppliedReactions()
        {
            for (int i = 0; i < nv_h; i++)
            {
                Fext_g[i] = MVector.Zero;
                Mext_m[i] = MVector.Zero;
            }
            for (int i = 0; i < nv_g; i++)
            {
                fext_g[i] = MVector.Zero;
                mext_m[i] = MVector.Zero;
            }
        }

        // MOMENTS

        private void UpdateBendingMoment()
        {
            MVector d1, d2;
            MVector κb, κb_0;
            MVector κb_l, κb_l_0, κb_r, κb_r_0;
            double dM1, dM2, M1, M2;
            MVector dM, Mmean;

            // HANDLE

            // i = 0
            κb_r = (2 / l[0]) * MVector.CrossProduct(t[0], t_mid[0]);
            κb_r_0 = κb_h_r_0[0];
            d1 = mframes[0].XAxis;
            d2 = mframes[0].YAxis;
            M1 = EI1[0] * (κb_r * d1 - κb_r_0.X);
            M2 = EI2[0] * (κb_r * d2 - κb_r_0.Y);
            M_h_r[0] = M1 * d1 + M2 * d2;
            κb_h_r[0] = κb_r;

            // i = 1, ..., nv_h-1
            for (int i = 1; i < nv_h - 1; i++)
            {
                // left curvature
                κb_l = (2 / l[2 * i - 1]) * MVector.CrossProduct(t_mid[2 * i - 1], t[2 * i]);
                κb_l_0 = κb_h_l_0[i];

                // right curvature
                κb_r = (2 / l[2 * i]) * MVector.CrossProduct(t[2 * i], t_mid[2 * i]);
                κb_r_0 = κb_h_r_0[i];

                d1 = mframes[2 * i].XAxis;
                d2 = mframes[2 * i].YAxis;

                // mean moment 
                M1 = 0.5 * (EI1[i - 1] * (κb_l * d1 - κb_l_0.X) + EI1[i] * (κb_r * d1 - κb_r_0.X));
                M2 = 0.5 * (EI2[i - 1] * (κb_l * d2 - κb_l_0.Y) + EI2[i] * (κb_r * d2 - κb_r_0.Y));
                Mmean = M1 * d1 + M2 * d2;

                // left & right moments that satisfy the static condition -Ml + Mr + Mext_m + Mr_m = 0
                dM1 = 0.5 * (Mext_m[i].X + Mr_m[i].X);
                dM2 = 0.5 * (Mext_m[i].Y + Mr_m[i].Y);
                dM = dM1 * d1 + dM2 * d2;
                M_h_l[i] = Mmean + dM;
                M_h_r[i] = Mmean - dM;

                // left curvatures from moment
                var κ1_h_l = (M1 + dM1) / EI1[i - 1];
                var κ2_h_l = (M2 + dM2) / EI2[i - 1];
                κb_h_l[i] = κ1_h_l * d1 + κ2_h_l * d2;

                // right curvatures from moment
                var κ1_h_r = (M1 - dM1) / EI1[i];
                var κ2_h_r = (M2 - dM2) / EI2[i];
                κb_h_r[i] = κ1_h_r * d1 + κ2_h_r * d2;
            }

            // i = nv_h - 1
            κb_l = (2 / l[ne - 1]) * MVector.CrossProduct(t_mid[ne - 1], t[nv - 1]);
            κb_l_0 = κb_h_l_0[nv_h - 1];
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;
            M1 = EI1[nv_g - 1] * (κb_l * d1 - κb_l_0.X);
            M2 = EI2[nv_g - 1] * (κb_l * d2 - κb_l_0.Y);
            M_h_l[nv_h - 1] = M1 * d1 + M2 * d2;
            κb_h_l[nv_h - 1] = κb_l;

            // GHOST
            for (int i = 0; i < nv_g; i++)
            {
                κb = κb_g[i];
                κb_0 = κb_g_0[i];
                d1 = mframes[2 * i + 1].XAxis;
                d2 = mframes[2 * i + 1].YAxis;

                M1 = EI1[i] * (κb * d1 - κb_0.X);
                M2 = EI2[i] * (κb * d2 - κb_0.Y);
                M_g[i] = M1 * d1 + M2 * d2;

                κb_mid[2 * i] = 0.5 * (κb_h_r[i] + κb_g[i]);
                κb_mid[2 * i] = κb_mid[2 * i] - (κb_mid[2 * i] * t_mid[2 * i]) * t_mid[2 * i]; // make sure kb is perpendicular to d3

                κb_mid[2 * i + 1] = 0.5 * (κb_g[i] + κb_h_l[i + 1]);
                κb_mid[2 * i + 1] = κb_mid[2 * i + 1] - (κb_mid[2 * i + 1] * t_mid[2 * i + 1]) * t_mid[2 * i + 1]; // make sure kb is perpendicular to d3
            }
        }

        /// <summary>
        /// Q : twist, τ
        /// Q is given at each mid-edge i+1/2
        /// </summary>
        private void UpdateTwistingMoment()
        {
            double l0, l1;
            double τmean, τ0, τ1, τ2, τmid;
            // interpolation de l'effort tranchant aux noeuds
            // non nécessaire pour la DR => à décplacer dans l'affichage
            for (int i = 0; i < nv_g; i++)
            {
                l0 = l[2 * i];
                l1 = l[2 * i + 1];

                var GJ = this.GJ[i];
                var m3 = mext_m[i].Z;

                // au repos, nécessairement τ_0(s) est uniforme sur [x_h_r, x_g, x_h_l]
                // et donc on a bien Q' = GJτ' = -(κbxM + m).d3 sur cet intervalle (et non Q' = GJ(τ'-τ_0')
                var dτ0 = -(1 / GJ) * (MVector.CrossProduct(κb_h_r[i], M_h_r[i]) * mframes[2 * i].ZAxis + m3);
                var dτ1 = -(1 / GJ) * (MVector.CrossProduct(κb_g[i], M_g[i]) * mframes[2 * i + 1].ZAxis + m3);
                var dτ2 = -(1 / GJ) * (MVector.CrossProduct(κb_h_l[i + 1], M_h_l[i + 1]) * mframes[2 * i + 2].ZAxis + m3);

                // 0-1
                τmean = τ[2 * i];
                τ0 = τmean - l0 / 6 * (2 * dτ0 + dτ1);
                τ1 = τmean + l0 / 6 * (dτ0 + 2 * dτ1);
                τmid = τmean + l0 / 24 * (dτ0 - dτ1);

                τ_r[2 * i] = τ0;
                Q_r[2 * i] = GJ * (τ0 - τ_0[2 * i]);

                τ_mid[2 * i] = τmid;
                Q_mid[2 * i] = GJ * (τmid - τ_0[2 * i]);

                τ_l[2 * i + 1] = τ1;
                Q_l[2 * i + 1] = GJ * (τ1 - τ_0[2 * i]);

                // 1-2
                τmean = τ[2 * i + 1];
                τ1 = τmean - l1 / 6 * (2 * dτ1 + dτ2);
                τ2 = τmean + l1 / 6 * (dτ1 + 2 * dτ2);
                τmid = τmean + l1 / 24 * (dτ1 - dτ2);

                τ_r[2 * i + 1] = τ1;
                Q_r[2 * i + 1] = GJ * (τ1 - τ_0[2 * i + 1]);

                τ_mid[2 * i + 1] = τmid;
                Q_mid[2 * i + 1] = GJ * (τmid - τ_0[2 * i + 1]);

                τ_l[2 * i + 2] = τ2;
                Q_l[2 * i + 2] = GJ * (τ2 - τ_0[2 * i + 1]);
            }

            // Calcul les frames au milieu des edges
            // GJ * dθ_mid = li/8 *(3*Qr + Ql)

            for (int i = 0; i < ne; i++)
            {
                var dθ_mid = l[i] / (8 * GJ[i/2]) * (3 * Q_r[i] + Q_l[i + 1]);
                ParallelTransportation.ZPT_Rotation(mframes[i], mframes[i].ZAxis, 0.5 * (x[i] + x[i + 1]), t_mid[i], ref mframes_mid[i]); // la position ne tient pas compte de la courbure. Mais ce n'est pas nécessaire pour la suite
                mframes_mid[i].ZRotate(dθ_mid);
            }

        }

        /// <summary>
        /// Q : twist, τ
        /// Q is given at each mid-edge i+1/2
        /// </summary>
        private void UpdateInternalNodalMoment()
        {
            Rint_θ[0] = MVector.Zero;
            for (int i = 0; i < nv_g; i++)
            {
                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                // au repos, nécessairement τ_0(s) est uniforme sur [x_h_r, x_g, x_h_l]
                // et donc on a bien Q' = GJτ' = -(κbxM + m).d3 sur cet intervalle (et non Q' = GJ(τ'-τ_0')
                var κM0 = MVector.CrossProduct(κb_h_r[i], M_h_r[i]) * mframes[2 * i].ZAxis;
                var κM1 = MVector.CrossProduct(κb_g[i], M_g[i]) * mframes[2 * i + 1].ZAxis;
                var κM2 = MVector.CrossProduct(κb_h_l[i + 1], M_h_l[i + 1]) * mframes[2 * i + 2].ZAxis;

                Rint_θ[2 * i].X += M_h_r[i].X;
                Rint_θ[2 * i].Y += M_h_r[i].Y;
                Rint_θ[2 * i].Z += Q_mid[2 * i] + l0/4 * (κM0 + κM1);

                Rint_θ[2 * i + 1].X = 0;
                Rint_θ[2 * i + 1].Y = 0;
                Rint_θ[2 * i + 1].Z = Q_mid[2 * i + 1] - Q_mid[2 * i] + l0 / 4 * (κM0 + κM1) + l1 / 4 * (κM1 + κM2);

                Rint_θ[2 * i + 2].X = -M_h_l[i + 1].X;
                Rint_θ[2 * i + 2].Y = -M_h_l[i + 1].Y;
                Rint_θ[2 * i + 2].Z = -Q_mid[2 * i + 1] + l1 / 4 * (κM1 + κM2);
            }
        }

        private void UpdateResultantNodalMoment()
        {
            // ADD APPLIED LOADS TO INTERNAL RESULTANT
            R_θ[0] = Rint_θ[0] + Mext_m[0] + (0.5 * l[0]) * mext_m[0];
            for (int i = 1; i < nv_h - 1; i++)
            {
                R_θ[2 * i] = Rint_θ[2 * i] 
                           + Mr_m[i] + Mext_m[i] 
                           + 0.5 * (mext_m[i - 1] * l[2 * i - 1] + mext_m[i] * l[2 * i]);
            }
            R_θ[nv - 1] = Rint_θ[nv - 1] + Mext_m[nv_h - 1] + (0.5 * l[2 * nv_g - 1]) * mext_m[nv_g - 1];

            for (int i = 0; i < nv_g; i++)
            {
                R_θ[2 * i + 1] = Rint_θ[2 * i + 1] + (0.5 * (l[2 * i] + l[2 * i + 1])) * mext_m[i];
            }

            OnReactionMomentUpdating(Mr_m, R_θ);

            // ADD REACTION MOMENTS TO INTERNAL RESULTANT
            //for (int i = 0; i < nv_h; i++)
            //{
            //    R_θ[2 * i].Z += Mr_m[i].Z;
            //}
        }

        // FORCES       
        /// <summary>
        /// V : shear force
        /// V_M is the bending contribution
        /// V_Q is the twisting contribution
        /// V, V_M and V_Q are given for each mid edge i+1/2. Their size is ne.
        /// </summary>
        private void UpdateShearForce()
        {
            MVector M0, M1, M2, Vmid;
            MVector dM01, dM12;
            MFrame mframe;
            double l0, l1;

            for (int i = 0; i < nv_g; i++)
            {
                // calcul des dérivées :
                M0 = M_h_r[i];
                M1 = M_g[i];
                M2 = M_h_l[i + 1];

                l0 = l[2 * i];
                l1 = l[2 * i + 1];

                dM01 = (M1 - M0) / l0; // dérivée en i+1/2 selon interpolation parabolique
                dM12 = (M2 - M1) / l1; // dérivée en i+3/2 selon interpolation parabolique

                var GJ = this.GJ[i];
                var m = this.mext_m[i];

                mframe = mframes[2 * i];
                Vmid = MVector.CrossProduct(t_mid[2 * i], dM01);    // d3 x M'
                Vmid += -m.Y * mframe.XAxis + m.X * mframe.YAxis;       // d3 x m
                Vmid += Q_mid[2 * i] * κb_mid[2 * i];                   // Qκb
                Vmid += -0.5 * τ_mid[2 * i] * (M0 + M1);               // -τM
                V_mid[2 * i] = Vmid;

                mframe = mframes[2 * i + 1];
                Vmid = MVector.CrossProduct(t_mid[2 * i + 1], dM12);    // d3 x M'
                Vmid += -m.Y * mframe.XAxis + m.X * mframe.YAxis;       // d3 x m
                Vmid += Q_mid[2 * i + 1] * κb_mid[2 * i + 1];           // Qκb
                Vmid += -0.5 * τ_mid[2 * i + 1] * (M1 + M2);            // -τM
                V_mid[2 * i + 1] = Vmid;


                V_M[2 * i] = MVector.CrossProduct(t_mid[2 * i], dM01 + m);
                V_Q[2 * i] = Q_mid[2 * i] * κb_mid[2 * i];
                var V_MQ = -(τ[2 * i] / 2) * (M0 + M1); // manque le terme -τM
                V_mid[2 * i] = V_M[2 * i] + V_Q[2 * i];

                V_M[2 * i + 1] = MVector.CrossProduct(t_mid[2 * i + 1], dM12 + m);
                V_Q[2 * i + 1] = Q_mid[2 * i + 1] * κb_mid[2 * i + 1];
                V_MQ = -(τ[2 * i + 1] / 2) * (M1 + M2); // manque le terme -τM
                V_mid[2 * i + 1] = V_M[2 * i + 1] + V_Q[2 * i + 1];
            }
        }

        private void UpdateAxialForce()
        {

            // A EVACUER
            for (int i = 0; i < ne; i++)
            {
                //double εε = l[i] * l[0] * (κ1[i] * κ1[i] + κ2[i] * κ2[i]) / 24; // second ordre avec la courbure
                //ε[i] = l[i] / l_0[i] - 1 + εε;
                var ε = l[i] / l_0[i] - 1;
                N_mid[i] = this.ES[i / 2] * ε;
            }

            for (int i = 0; i < ne; i++)
            {
                // dN est évalué à partir de N' + κ1T2 - κ2T1 + f3 = 0 (à l'équilibre statique)
                // et donc N' = dN/ds = -m3 - (κ1T2 - κ2T1)
                // on remarque que κb x T = (κ1T2 - κ2T1) * d3

                var f = this.fext_g[i / 2];
                var ES = this.ES[i / 2];

                var κT = MVector.CrossProduct(κb_mid[i], V_mid[i]) ;
                var dN = (-0.5 * l[i]) * ((κT + f) * t_mid[i]);
                var d3 = mframes[i].ZAxis;
                N_r[i] = (N_mid[i] - dN) * d3 ;
                N_l[i + 1] = (N_mid[i] + dN) * d3;


                //var κN = MVector.CrossProduct(κb_mid[i], N_mid[i] * t_mid[i]);
                //var τT = MVector.CrossProduct(τ[i] * t_mid[i], V_mid[i]);

                //var d1 = mframes[i].XAxis;
                //var d2 = mframes[i].YAxis;

                //var dT1 = (-0.5 * l[i]) * (κN + κT + f) * d1;   // normalement utiliser d1_mid
                //var dT2 = (-0.5 * l[i]) * (κN + κT + f) * d2;   // normalement utiliser d2_mid

                //V_r[i] = (V_mid[i] * d1 - dT1) * d1 + (V_mid[i] * d2 - dT2) * d2;
                //V_l[i + 1] = (V_mid[i] * d1 + dT1) * d1 + (V_mid[i] * d2 + dT2) * d2;

                //// 2i
                //κT = MVector.CrossProduct(κb_mid[i], V_mid[i]) * t_mid[i];

                //ε = l[i] / l_0[i] - 1;
                //Nmid = ES * ε;

            }

            UpdateShearForceAtNodes();
        }

        private void UpdateInternalNodalForce2()
        {         
            // RESULTANT FORCE
            for (int i = 1; i < nv - 1; i++)
            {
                var N = -N_mid[i - 1] * t_mid[i - 1] + N_mid[i] * t_mid[i];
                Rint_x[i] = N - V_mid[i - 1] + V_mid[i];
            }

            //// RESULTANT FORCE
            //for (int i = 1; i < nv - 1; i++)
            //{
            //    Rint_x[i] = (N_r[i] - N_l[i - 1]) + (V_r[i] - V_l[i - 1]);
            //}
        }

        private void UpdateInternalNodalForce()
        {
            // AXIAL FORCE : attention à ne pas répéter cette opération pour les variations sur θ

            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // ne prend pas en compte les efforts linéiques
            Rint_x_axial[0] = ((N_mid[0] * t_mid[0] + V_mid[0]) * t[0]) * t[0];
            Rint_x_axial[0] = N_mid[0] * t_mid[0];
            Rint_x_axial[0] = N_r[0];

            Rint_x_axial[1] = -N_mid[0] * t_mid[0];
            for (int i = 1; i < ne - 1; i++)
            {
                MVector force = N_mid[i] * t_mid[i];
                Rint_x_axial[i] += force;
                Rint_x_axial[i + 1] = -force;
            }
            Rint_x_axial[nv - 2] += N_mid[ne - 1] * t_mid[ne - 1];
            Rint_x_axial[nv - 1] = -((N_mid[ne - 1] * t_mid[ne - 1] + V_M[ne - 1] + V_Q[ne - 1]) * t[nv - 1]) * t[nv - 1];

            // SHEAR FORCE | bending contribution
            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // ne prend pas en compte les efforts linéiques
            Rint_x_shear_M[0] = (N_mid[0] * t_mid[0]) - ((N_mid[0] * t_mid[0] + V_M[0]) * t[0]) * t[0];
            for (int i = 0; i < ne; i++)
            {
                Rint_x_shear_M[i] += V_M[i];
                Rint_x_shear_M[i + 1] = -V_M[i];
            }
            Rint_x_shear_M[nv - 1] += -N_mid[ne - 1] * t_mid[ne - 1] + ((N_mid[ne - 1] * t_mid[ne - 1] + V_M[ne - 1]) * t[nv - 1]) * t[nv - 1];

            // SHEAR FORCE | twisting contribution
            Rint_x_shear_Q[0] = -(V_Q[0] * t[0]) * t[0];
            for (int i = 0; i < ne; i++)
            {
                Rint_x_shear_Q[i] += V_Q[i];
                Rint_x_shear_Q[i + 1] = -V_Q[i];
            }
            Rint_x_shear_Q[nv - 1] += (V_Q[nv_h - 1] * t[nv - 1]) * t[nv - 1];

            // RESULTANT FORCE
            for (int i = 0; i < nv; i++)
            {
                Rint_x[i] = Rint_x_axial[i] + Rint_x_shear_M[i] + Rint_x_shear_Q[i];
            }
        }

        private void UpdateResultantNodalForce()
        {
            // RESULTANT FORCE

            // ADD APPLIED LOADS TO INTERNAL RESULTANT
            R_x[0] = Rint_x[0] + Fext_g[0] + (0.5 * l[0]) * fext_g[0];
            for (int i = 1; i < nv_h - 1; i++)
            {
                R_x[2 * i] = Rint_x[2 * i] 
                           + Fr_g[i] + Fext_g[i] 
                           + 0.5 * (fext_g[i - 1] * l[2 * i - 1] + fext_g[i] * l[2 * i]);
            }
            R_x[nv - 1] = Rint_x[nv - 1] + Fext_g[nv_h - 1] + (0.5 * l[2 * nv_g - 1]) * fext_g[nv_g - 1];
            for (int i = 0; i < nv_g; i++)
            {
                R_x[2 * i + 1] = Rint_x[2 * i + 1] + (0.5 * (l[2 * i] + l[2 * i + 1])) * fext_g[i];
            }

            OnReactionForceUpdating(Fr_g, R_x);

            // ADD REACTION FORCES TO INTERNAL RESULTANT
            //for (int i = 0; i < nv_h; i++)
            //{
            //    R_x[2 * i] += Fr_g[i];
            //}
        }

        // ENERGIES
        public double GetAxialElasticEnergy()
        {
            double E_axial = 0;
            for (int i = 0; i < ne; i++)
            {
                E_axial += ES[i] * Math.Pow(ε[i], 2) * l[i];
            }
            E_axial = E_axial / 2;
            return E_axial;
        }
        public double GetBendingElasticEnergy()
        {
            double E_bending = 0;
            //E_bending += (EI1[0] * Math.Pow((κ1[0] - κ1_0[0]), 2) + EI2[0] * Math.Pow((κ2[0] - κ2_0[0]), 2)) * (l[0] / 4);
            //for (int i = 1; i < nv - 1; i++)
            //{
            //    E_bending += (EI1[i/2] * Math.Pow((κ1[i] - κ1_0[i]), 2) + EI2[i/2] * Math.Pow((κ2[i] - κ2_0[i]), 2)) * (l[i - 1] + l[i]) / 4;
            //}
            //E_bending += (EI1[nv_g - 1] * Math.Pow((κ1[nv - 1] - κ1_0[nv - 1]), 2) + EI2[nv_g - 1] * Math.Pow((κ2[nv - 1] - κ2_0[nv - 1]), 2)) * (l[ne - 1] / 4);
            return E_bending;
        }
        public double GetTwistingElasticEnergy()
        {
            double E_twisting = 0;
            for (int i = 0; i < ne; i++)
            {
                double τ2 = (τ[i] - τ_0[i]) * (τ[i] - τ_0[i]);
                E_twisting += GJ[i/2] * Math.Pow((τ[i] - τ_0[i]), 2) * l[i];
            }
            E_twisting = E_twisting / 2;
            return E_twisting;
        }

        public void UpdateShearForceAtNodes()
        {
            // interpolation de l'effort tranchant aux noeuds
            // non nécessaire pour la DR => à décplacer dans l'affichage
            for (int i = 0; i < nv_g; i++)
            {
                // calcul des dérivées :
                var M0 = M_h_r[i];
                var M1 = M_g[i];
                var M2 = M_h_l[i + 1];

                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                var dM0 = -(2 * l0 + l1) / (l0 * (l0 + l1)) * M0 + (l0 + l1) / (l0 * l1) * M1 - l0 / (l1 * (l0 + l1)) * M2;
                var dM1 = -l1 / (l0 * (l1 + l0)) * M0 - (l0 - l1) / (l0 * l1) * M1 + l0 / (l1 * (l0 + l1)) * M2;
                var dM2 = l1 / (l0 * (l0 + l1)) * M0 - (l1 + l0) / (l0 * l1) * M1 + (2 * l1 + l0) / (l1 * (l1 + l0)) * M2;


                V_M_h_r[i] = MVector.CrossProduct(mframes[2 * i].ZAxis, dM0 + mext_m[i]);
                V_M_g[i] = MVector.CrossProduct(mframes[2 * i + 1].ZAxis, dM1 + mext_m[i]);
                V_M_h_l[i + 1] = MVector.CrossProduct(mframes[2 * i + 2].ZAxis, dM2 + mext_m[i]);

                V_Q_h_r[i] = Q_r[2 * i] * κb_h_r[i];
                V_Q_g[i] = 0.5 * (Q_l[2 * i + 1] + Q_r[2 * i + 1]) * κb_g[i];
                V_Q_h_l[i + 1] = Q_l[2 * i + 2] * κb_h_l[i + 1]; ;

                //V_M_h_r[i] += V_Q_h_r[i];
                //V_M_g[i] += V_Q_g[i];
                //V_M_h_l[i + 1] += V_Q_h_l[i + 1];

                V_r[2 * i] = V_M_h_r[i] + V_Q_h_r[i];
                V_r[2 * i + 1] = V_M_g[i] + V_Q_g[i];
                V_l[2 * i + 1] = V_M_g[i] + V_Q_g[i];
                V_l[2 * i + 2] = V_M_h_l[i + 1] + V_Q_h_l[i + 1];

            }
        }
        public void InterpolateTwistingnMoment()
        {
           
        }

        public MVector GetReactionForce(Boundary boundary)
        {
            if (boundary == Boundary.Start)
            {
                return -(Rint_x_axial[0] + Rint_x_shear_M[0] + Rint_x_shear_Q[0]);
            }
            else
            {
                return -(Rint_x_axial[nv - 1] + Rint_x_shear_M[nv - 1] + Rint_x_shear_Q[nv - 1]);
            }

        }
        public MVector GetReactionMoment(Boundary boundary)
        {
            if (boundary == Boundary.Start)
            {
                return -(M_h_r[0] + Q_r[0] * mframes[0].ZAxis);
            }
            else
            {
                return (M_h_l[nv_h - 1] + Q_l[nv - 1] * mframes[nv - 1].ZAxis);
            }
        }
        public MVector[] GetMext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.Mext_m;
            }
            else
            {
                MVector[] Mext_m = new MVector[nv];
                for (int i = 0; i < Mext_m.Length; i++)
                {
                    Mext_m[i] = this.Mext_m[i].X * mframes[i].XAxis
                                + this.Mext_m[i].Y * mframes[i].YAxis
                                + this.Mext_m[i].Z * mframes[i].ZAxis;
                }

                return Mext_m;
            }
        }
        public MVector[] Getmext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.mext_m;
            }
            else
            {
                MVector[] mext_m = new MVector[ne];
                for (int i = 0; i < mext_m.Length; i++)
                {
                    mext_m[i] = this.mext_m[i].X * mframes[i].XAxis
                                + this.mext_m[i].Y * mframes[i].YAxis
                                + this.mext_m[i].Z * mframes[i].ZAxis;
                }

                return mext_m;
            }
        }
        public MVector[] GetFext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.Fext_g;
            }
            else
            {
                MVector[] Fext_g = new MVector[nv];
                for (int i = 0; i < nv; i++)
                {
                    Fext_g[i].X = this.Fext_g[i] * mframes[i].XAxis;
                    Fext_g[i].Y = this.Fext_g[i] * mframes[i].YAxis;
                    Fext_g[i].Z = this.Fext_g[i] * mframes[i].ZAxis;
                }
                return Fext_g;
            }
        }
        public MVector[] Getfext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.fext_g;
            }
            else
            {
                MVector[] Fext_g = new MVector[nv];
                for (int i = 0; i < nv; i++)
                {
                    Fext_g[i].X = this.Fext_g[i] * mframes[i].XAxis;
                    Fext_g[i].Y = this.Fext_g[i] * mframes[i].YAxis;
                    Fext_g[i].Z = this.Fext_g[i] * mframes[i].ZAxis;
                }
                return Fext_g;
            }
        }

        public MVector[] GetFr(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.Fr_g;
            }
            else
            {
                MVector[] Fr_g = new MVector[nv];
                for (int i = 0; i < Fr_g.Length; i++)
                {
                    Fr_g[i].X = this.Fr_g[i] * mframes[i].XAxis;
                    Fr_g[i].Y = this.Fr_g[i] * mframes[i].YAxis;
                    Fr_g[i].Z = this.Fr_g[i] * mframes[i].ZAxis;
                }
                return Fr_g;
            }
        }
        public MVector[] GetMr(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.Mr_m;
            }
            else
            {
                MVector[] Mr_m = new MVector[nv];
                for (int i = 0; i < Mr_m.Length; i++)
                {
                    Mr_m[i] = this.Mr_m[i].X * mframes[i].XAxis
                                + this.Mr_m[i].Y * mframes[i].YAxis
                                + this.Mr_m[i].Z * mframes[i].ZAxis;
                }
                return Mr_m;
            }
        }

        // doit être customizable
        public override void Update_lm_x(ref double[] lm_x)
        {
            double lm;
            lm_x[0] = 0.0;
            for (int i = 0; i < nv - 1; i++)
            {
                lm = 0.5 * (ES[i/2] / l_0[i] + 1.5 * Math.Abs(N_mid[i]) / l[i]);
                lm_x[i] += lm;
                lm_x[i + 1] = lm;
            }
        }
        public override void Update_lm_θ(ref double[] lm_θ)
        {
            double lm;
            lm_θ[0] = 0.0;
            for (int i = 0; i < ne; i++)
            {
                lm = 0.5 * (GJ[i/2] / l_0[i] + 1.5 * Math.Abs(Q_mid[i]) / l[i]);
                lm_θ[i] += lm;
                lm_θ[i + 1] = lm;
            }
        }

    }
}
