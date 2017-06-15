using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
   


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
        private double[] l_0;                 
        private double[] τ_0;

        // in LCS (X <=> d1 | Y <=> d2 | Z = 0)
        private MVector[] κb_mid_0;
        private MVector[] κb_l_0;
        private MVector[] κb_g_0;
        private MVector[] κb_r_0;

        // ACTUAL CONFIG (in GCS)
        private MVector[] t_mid;
        private MFrame[] mframes_mid;
       
        // in GCS
        private MVector[] κb_mid;
        private MVector[] κb_l;       
        private MVector[] κb_g;
        private MVector[] κb_r;

        private double[] κ1_mid, κ1_l, κ1_g, κ1_r;
        private double[] κ2_mid, κ2_l, κ2_g, κ2_r;

        private double[] τ;       
        private double[] τ_l;
        private double[] τ_r;

        private double[] ε;
        private double[] ε_l;
        private double[] ε_r;

        // in GCS
        private MVector[] M_l;
        private MVector[] M_g;
        private MVector[] M_r;

        private double[] M1_l, M1_g, M1_r;
        private double[] M2_l, M2_g, M2_r;

        private double[] Q_mid;
        private double[] Q_l;
        private double[] Q_r;


        // in GCS
        private MVector[] V_mid;
        private MVector[] V_l;
        private MVector[] V_r;


        private double[] F1_mid, F1_l, F1_r;
        private double[] F2_mid, F2_l, F2_r;

        private double[] N_mid;                                 
        private double[] N_l;
        private double[] N_r;

        // LCS ou GCS ??
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
            this.sections = new Section[nv_g];
            if (sections.Count() == 1)
            {
                var s = sections.First();
                for (int i = 0; i < nv_g; i++)
                {
                    this.sections[i] = s; // DeepCopy
                }
            }
            else if (sections.Count() != nv_g)
            {
                throw new ArgumentException("sections must have either one item of ne items.");
            }
            else
            {
                int count = 0;
                foreach (var s in sections)
                {
                    this.sections[count] = s;
                    count++;
                }
            }

            this.materials = new Material[nv_g];
            if (materials.Count() == 1)
            {
                var m = materials.First();
                for (int i = 0; i < nv_g; i++)
                {
                    this.materials[i] = m; // DeepCopy
                }
            }
            else if (materials.Count() != nv_g)
            {
                throw new ArgumentException("materials must have either one item of ne items.");
            }
            else
            {
                int count = 0;
                foreach (var m in materials)
                {
                    this.materials[count] = m;
                    count++;
                }
            }


            for (int i = 0; i < nv_g; i++)
            {
                var section = this.sections[i];
                var material = this.materials[i];

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
            κb_l_0 = new MVector[nv_h];
            κb_g_0 = new MVector[nv_g];
            κb_r_0 = new MVector[nv_h];

            τ_0 = new double[ne];

            // DEFORMED CONFIGURATION
            x = new MPoint[nv];
            e = new MVector[ne];
            l = new double[ne];

            t = new MVector[nv];
            t_mid = new MVector[ne];
            mframes_mid = new MFrame[ne];

            ε = new double[ne];
            ε_l = new double[nv];
            ε_r = new double[nv];

            κb_mid = new MVector[ne];
            κb_l = new MVector[nv_h];       
            κb_g = new MVector[nv_g];       
            κb_r = new MVector[nv_h];

            κ1_mid = new double[ne];
            κ1_l = new double[nv_h];
            κ1_g = new double[nv_g];
            κ1_r = new double[nv_h];

            κ2_mid = new double[ne];
            κ2_l = new double[nv_h];
            κ2_g = new double[nv_g];
            κ2_r = new double[nv_h];

            τ = new double[ne];
            τ_l = new double[nv];
            τ_r = new double[nv];

            // INTERNAL FORCES & MOMENTS
            N_mid = new double[ne];
            N_l = new double[nv];
            N_r = new double[nv];

            V_mid = new MVector[ne];
            V_l = new MVector[nv];
            V_r = new MVector[nv];

            F1_mid = new double[ne];
            F1_l = new double[nv];
            F1_r = new double[nv];
            F2_mid = new double[ne];
            F2_l = new double[nv];
            F2_r = new double[nv];

            M_g = new MVector[nv_g];
            M_l = new MVector[nv_h];  
            M_r = new MVector[nv_h];

            M1_g = new double[nv_g];
            M1_l = new double[nv_h];
            M1_r = new double[nv_h];
            M2_g = new double[nv_g];
            M2_l = new double[nv_h];
            M2_r = new double[nv_h];

            Q_mid = new double[ne];
            Q_l = new double[nv];
            Q_r = new double[nv];

            // DR RESULTANT
            R_x = new MVector[nv];
            Rint_x = new MVector[nv];


            R_θ = new MVector[nv];
            Rint_θ = new MVector[nv];

            dθ = new MVector[nv];
    }

        public int[] Refine()
        {
            var indexMap = new int[nv];
            for (int i = 0; i < nv; i++)
            {
                indexMap[i] = 2 * i;
            }

            CreateGhostVertices(mframes_0, mframes, IsClosed);

            // Refine actual & rest frames
            nv = nv + ne;
            nv_h = nv_h + nv_g;
            nv_g = 2 * nv_g;
            ne = 2 * ne;

            // backu

            CreateInternalArrays(nv, nv_h, nv_g, ne);

            // Refine Sections & Material
            var sections_r = new Section[nv_g];
            var materials_r = new Material[nv_g];

            for (int i = 0; i < nv_g / 2; i++)
            {
                sections_r[2 * i] = sections[i];
                sections_r[2 * i + 1] = sections[i];
                materials_r[2 * i] = materials[i];
                materials_r[2 * i + 1] = materials[i];
            }

            this.sections = sections_r;
            this.materials = materials_r;

            for (int i = 0; i < nv_g; i++)
            {
                var material = materials[i];
                var section = sections[i];
                ES[i] = material.E * section.S;
                EI1[i] = material.E * section.I1;
                EI2[i] = material.E * section.I2;
                GJ[i] = material.G * section.J;
            }

            // SET CONFIGURATIONS
            SetRestConfig(mframes_0);
            SetInitialConfig(mframes);

            // Refine loads
            // move supports
            OnTopologyChanged(indexMap);

            return indexMap;
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
            κb_r_0[0].X = κb_r * d1;
            κb_r_0[0].Y = κb_r * d2;

            for (int i = 1; i < nv_h-1; i++)
            {
                index = 2 * i;
                κb_l = (2 / l_0[index - 1]) * MVector.CrossProduct(t_mid[index - 1], t[index]);
                κb_r = (2 / l_0[index]) * MVector.CrossProduct(t[index], t_mid[index]);

                d1 = restFrames[index].XAxis;
                d2 = restFrames[index].YAxis;

                // WARNING : given in the material frame coordinate system
                κb_l_0[i].X = κb_l * d1;
                κb_l_0[i].Y = κb_l * d2;
                κb_r_0[i].X = κb_r * d1;
                κb_r_0[i].Y = κb_r * d2;
            }

            // i = nv_h-1
            κb_l = 2 / (l_0[ne - 1]) * MVector.CrossProduct(t_mid[ne - 1], t[nv - 1]);
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;
            κb_l_0[nv_h - 1].X = κb_l * d1;
            κb_l_0[nv_h - 1].Y = κb_l * d2;

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

                κb_mid_0[2 * i] = 0.5 * (κb_r_0[i] + κb_g_0[i]);
                κb_mid_0[2 * i] = κb_mid_0[2 * i] - (κb_mid_0[2 * i] * t_mid[2 * i]) * t_mid[2 * i]; // make sure kb is perpendicular to d3

                κb_mid_0[2 * i + 1] = 0.5 * (κb_g_0[i] + κb_l_0[i + 1]);
                κb_mid_0[2 * i + 1] = κb_mid_0[2 * i + 1] - (κb_mid_0[2 * i + 1] * t_mid[2 * i + 1]) * t_mid[2 * i + 1]; // make sure kb is perpendicular to d3
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

            // befor any moves
            for (int i = 0; i < nv; i++)
            {
                t[i] = mframes[i].ZAxis;
            }
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
            MVector κbl, κbl_0, κbr, κbr_0;
            double dM1, dM2, M1, M2;
            MVector dM, Mmean;

            // HANDLE

            // i = 0
            d1 = mframes[0].XAxis;
            d2 = mframes[0].YAxis;

            // right curvature
            κb_r[0] = (2 / l[0]) * MVector.CrossProduct(t[0], t_mid[0]);
            κ1_r[0] = κb_r[0] * d1;
            κ2_r[0] = κb_r[0] * d2;

            // right bending moment
            M1 = EI1[0] * (κ1_r[0] - κb_r_0[0].X);
            M2 = EI2[0] * (κ2_r[0] - κb_r_0[0].Y);
            M1_r[0] = M1;
            M2_r[0] = M2;
            M_r[0] = M1 * d1 + M2 * d2;

            // i = 1, ..., nv_h-1
            for (int i = 1; i < nv_h - 1; i++)
            {
                d1 = mframes[2 * i].XAxis;
                d2 = mframes[2 * i].YAxis;

                // left curvature
                κbl = (2 / l[2 * i - 1]) * MVector.CrossProduct(t_mid[2 * i - 1], t[2 * i]);
                κbl_0 = κb_l_0[i];

                // right curvature
                κbr = (2 / l[2 * i]) * MVector.CrossProduct(t[2 * i], t_mid[2 * i]);
                κbr_0 = κb_r_0[i];

                // mean moment 
                M1 = 0.5 * (EI1[i - 1] * (κbl * d1 - κbl_0.X) + EI1[i] * (κbr * d1 - κbr_0.X));
                M2 = 0.5 * (EI2[i - 1] * (κbl * d2 - κbl_0.Y) + EI2[i] * (κbr * d2 - κbr_0.Y));
                Mmean = M1 * d1 + M2 * d2;

                // left & right moments that satisfy the static condition -Ml + Mr + Mext_m + Mr_m = 0
                dM1 = 0.5 * (Mext_m[i].X + Mr_m[i].X);
                dM2 = 0.5 * (Mext_m[i].Y + Mr_m[i].Y);
                M1_l[i] = M1 + dM1;
                M2_l[i] = M2 + dM2;
                M1_r[i] = M1 - dM1;
                M2_r[i] = M2 - dM2;

                dM = dM1 * d1 + dM2 * d2;
                M_l[i] = Mmean + dM;
                M_r[i] = Mmean - dM;

                // left curvatures from moment
                κ1_l[i] = (M1 + dM1) / EI1[i - 1];
                κ2_l[i] = (M2 + dM2) / EI2[i - 1];
                κb_l[i] = κ1_l[i] * d1 + κ2_l[i] * d2;

                // right curvatures from moment
                κ1_r[i] = (M1 - dM1) / EI1[i];
                κ2_r[i] = (M2 - dM2) / EI2[i];
                κb_r[i] = κ1_r[i] * d1 + κ2_r[i] * d2;
            }

            // i = nv_h - 1
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;

            // left curvature
            κb_l[nv_h - 1] = (2 / l[ne - 1]) * MVector.CrossProduct(t_mid[ne - 1], t[nv - 1]);
            κ1_l[nv_h - 1] = κb_l[nv_h - 1] * d1;
            κ2_l[nv_h - 1] = κb_l[nv_h - 1] * d2;

            // left bending moment
            M1 = EI1[nv_g - 1] * (κ1_l[nv_h - 1] - κb_l_0[nv_h - 1].X);
            M2 = EI2[nv_g - 1] * (κ2_l[nv_h - 1] - κb_l_0[nv_h - 1].Y);
            M1_l[nv_h - 1] = M1;
            M2_l[nv_h - 1] = M2;
            M_l[nv_h - 1] = M1 * d1 + M2 * d2;

            // GHOST
            for (int i = 0; i < nv_g; i++)
            {
                d1 = mframes[2 * i + 1].XAxis;
                d2 = mframes[2 * i + 1].YAxis;

                // curvature
                κ1_g[i] = κb_g[i] * d1;
                κ2_g[i] = κb_g[i] * d2;

                // bending moment
                M1 = EI1[i] * (κ1_g[i] - κb_g_0[i].X);
                M2 = EI2[i] * (κ2_g[i] - κb_g_0[i].Y);
                M1_g[i] = M1;
                M2_g[i] = M2;
                M_g[i] = M1_g[i] * d1 + M2_g[i] * d2;

                // parabolic interpolation
                // Parabolic interpolation of the components in the material frame basis
                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];
                Interpolation.Quadratic(l0, l1, κ1_r[i], κ1_g[i], κ1_l[i + 1], out κ1_mid[2 * i], out κ1_mid[2 * i + 1]);
                Interpolation.Quadratic(l0, l1, κ2_r[i], κ2_g[i], κ2_l[i + 1], out κ2_mid[2 * i], out κ2_mid[2 * i + 1]);

                //κ1_mid[2 * i] = 0.5 * (κ1_r[i] + κ1_g[i]);
                //κ2_mid[2 * i] = 0.5 * (κ2_r[i] + κ2_g[i]);

                //κ1_mid[2 * i + 1] = 0.5 * (κ1_g[i] + κ1_l[i + 1]);
                //κ2_mid[2 * i + 1] = 0.5 * (κ2_g[i] + κ2_l[i + 1]);

                // passer en interpolation parabolique ??
                κb_mid[2 * i] = 0.5 * (κb_r[i] + κb_g[i]);
                κb_mid[2 * i] = κb_mid[2 * i] - (κb_mid[2 * i] * t_mid[2 * i]) * t_mid[2 * i]; // make sure kb is perpendicular to d3

                κb_mid[2 * i + 1] = 0.5 * (κb_g[i] + κb_l[i + 1]);
                κb_mid[2 * i + 1] = κb_mid[2 * i + 1] - (κb_mid[2 * i + 1] * t_mid[2 * i + 1]) * t_mid[2 * i + 1]; // make sure kb is perpendicular to d3
            }
        }
        private void UpdateTwistingMoment()
        {
            for (int i = 0; i < nv_g; i++)
            {
                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                var GJ = this.GJ[i];
                var m3 = mext_m[i].Z; // in LCS => à passer en GCS pour uniformiser

                // Q' + (κb x M + m).d3 = 0
                // Q' + κ1M2 - κ2M1 + m3 = 0
                var κM0 = κ1_r[i] * M2_r[i] - κ2_r[i] * M1_r[i];
                var κM1 = κ1_g[i] * M2_g[i] - κ2_g[i] * M1_g[i];
                var κM2 = κ1_l[i + 1] * M2_l[i + 1] - κ2_l[i + 1] * M1_l[i + 1];

                // 0-1
                var τ01 = τ[2 * i];
                var Q01 = GJ * (τ01 - τ_0[2 * i]);
                var dQ01 = (l0 / 4) * (κM0 + κM1) + (l0 / 2) * m3;
                var dτ01 = dQ01 / GJ;

                Q_mid[2 * i] = Q01;
                Q_r[2 * i] = Q01 + dQ01;
                Q_l[2 * i + 1] = Q01 - dQ01;
                τ_r[2 * i] = τ01 + dτ01;
                τ_l[2 * i + 1] = τ01 - dτ01;

                // 1-2
                var τ12 = τ[2 * i + 1];
                var dQ12 = (l1 / 4) * (κM1 + κM2) + (l1 / 2) * m3;
                var Q12 = GJ * (τ12 - τ_0[2 * i + 1]);
                var dτ12 = dQ12 / GJ;

                Q_mid[2 * i + 1] = Q12;
                Q_r[2 * i + 1] = Q12 + dQ12;
                Q_l[2 * i + 2] = Q12 - dQ12;
                τ_r[2 * i + 1] = τ12 + dτ12;
                τ_l[2 * i + 2] = τ12 - dτ12;
            }

            for (int i = 0; i < nv_g; i++)
            {
                mframes_mid[2 * i] = Centerline.Interpolate(mframes[2 * i], mframes[2 * i + 1], κb_r[i], κb_g[i], τ_r[2 * i], τ_l[2 * i + 1]);
                mframes_mid[2 * i + 1] = Centerline.Interpolate(mframes[2 * i + 1], mframes[2 * i + 2], κb_g[i], κb_l[i + 1], τ_r[2 * i + 1], τ_l[2 * i + 2]);
            }
        }
        private void UpdateInternalNodalMoment()
        {
            Rint_θ[0] = MVector.Zero;
            for (int i = 0; i < nv_g; i++)
            {
                // ATTENTION : confusion entre GCS pour XY et LCS pour Z ...
                // Pour l'instant ca marche mais uniquement car seulement Rint_θ.Z est pris en compte dans une relaxation dynamique sur θ = θ3 et non θZ.
                // => tout passer en GCS
                Rint_θ[2 * i].X += M_r[i].X;
                Rint_θ[2 * i].Y += M_r[i].Y;
                Rint_θ[2 * i].Z += Q_r[2 * i];

                Rint_θ[2 * i + 1].X = 0;
                Rint_θ[2 * i + 1].Y = 0;
                Rint_θ[2 * i + 1].Z = -Q_l[2 * i + 1] + Q_r[2 * i + 1];

                Rint_θ[2 * i + 2].X = -M_l[i + 1].X;
                Rint_θ[2 * i + 2].Y = -M_l[i + 1].Y;
                Rint_θ[2 * i + 2].Z = -Q_l[2 * i + 2];
            }
        }
        private void UpdateResultantNodalMoment()
        {
            // ADD APPLIED LOADS TO INTERNAL RESULTANT
            R_θ[0] = Rint_θ[0] + Mext_m[0] + (0.5 * l[0]) * mext_m[0];
            for (int i = 1; i < nv_h - 1; i++)
            {
                R_θ[2 * i] = Rint_θ[2 * i]
                           + Mr_m[i] + Mext_m[i];
                           //+ 0.0 * (mext_m[i - 1] * l[2 * i - 1] + mext_m[i] * l[2 * i]);
            }
            R_θ[nv - 1] = Rint_θ[nv - 1] + Mext_m[nv_h - 1] + (0.5 * l[2 * nv_g - 1]) * mext_m[nv_g - 1];

            for (int i = 0; i < nv_g; i++)
            {
                R_θ[2 * i + 1] = Rint_θ[2 * i + 1];
                    //+ (0.0 * (l[2 * i] + l[2 * i + 1])) * mext_m[i];
            }

            OnReactionMomentUpdating(Mr_m, R_θ);
        }

        // FORCES       
        private void UpdateShearForce()
        {
            MVector dM01, dM12, dM0, dM1, dM2, M01, M12;
            var M1_mid = new double[ne];
            var M2_mid = new double[ne];

            for (int i = 0; i < nv_g; i++)
            {
                var m = this.mext_m[i];

                // Parabolic interpolation of the components in the material frame basis
                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];
                var M0 = new MVector(M1_r[i], M2_r[i], 0);
                var M1 = new MVector(M1_g[i], M2_g[i], 0);
                var M2 = new MVector(M1_l[i + 1], M2_l[i + 1], 0);
                Interpolation.Quadratic(l0, l1, M0, M1, M2, out dM0, out dM1, out dM2, out dM01, out dM12, out M01, out M12);

                dM01 = new MVector((M1_g[i] - M1_r[i]) / l0, (M2_g[i] - M2_r[i]) / l0, 0);
                dM12 = new MVector((M1_l[i + 1] - M1_g[i]) / l1, (M2_l[i + 1] - M2_g[i]) / l1, 0);

                M01 = new MVector((M1_g[i] + M1_r[i]) / 2, (M2_g[i] + M2_r[i]) / 2, 0);
                M12 = new MVector((M1_l[i + 1] + M1_g[i]) / 2, (M2_l[i + 1] + M2_g[i]) / 2, 0);

                M1_mid[2*i] = M01.X;
                M2_mid[2*i] = M01.Y;

                M1_mid[2 * i+1] = M12.X;
                M2_mid[2 * i+1] = M12.Y;

                F1_mid[2 * i] = -dM01.Y + κ1_mid[2 * i] * Q_mid[2 * i] - τ[2 * i] * M01.X - m.Y;
                F2_mid[2 * i] = dM01.X + κ2_mid[2 * i] * Q_mid[2 * i] - τ[2 * i] * M01.Y + m.X;
                V_mid[2 * i] = F1_mid[2 * i] * mframes_mid[2 * i].XAxis + F2_mid[2 * i] * mframes_mid[2 * i].YAxis;

                F1_mid[2 * i + 1] = -dM12.Y + κ1_mid[2 * i + 1] * Q_mid[2 * i + 1] - τ[2 * i + 1] * M12.X - m.Y;
                F2_mid[2 * i + 1] = dM12.X + κ2_mid[2 * i + 1] * Q_mid[2 * i + 1] - τ[2 * i + 1] * M12.Y + m.X;
                V_mid[2 * i + 1] = F1_mid[2 * i + 1] * mframes_mid[2 * i + 1].XAxis + F2_mid[2 * i + 1] * mframes_mid[2 * i + 1].YAxis;
            }
        }
        private void UpdateAxialForce()
        {
            for (int i = 0; i < nv_g; i++)
            {
                // 0-1-2
                var ES = this.ES[i];
                var f1 = fext_g[i] * mframes[2 * i + 1].XAxis;
                var f2 = fext_g[i] * mframes[2 * i + 1].YAxis;
                var f3 = fext_g[i] * mframes[2 * i + 1].ZAxis;

                // N' + (κb x F).d3 = 0
                // N' + κ1F2 - κ2F1 + f3 = 0
                var κF01 = κ1_mid[2 * i] * F2_mid[2 * i] - κ2_mid[2 * i] * F1_mid[2 * i];
                var κF12 = κ1_mid[2 * i + 1] * F2_mid[2 * i + 1] - κ2_mid[2 * i + 1] * F1_mid[2 * i + 1];

                // PENSER A VERIFIER f3

                // 0-1
                var l0 = l[2 * i];
                var ε01 = l0 / l_0[2 * i] - 1;
                var N01 = ES * ε01;
                var dN01 = (l0 / 2) * (κF01 + f3);
                var dε01 = dN01 / ES;

                // 0-2
                var l1 = l[2 * i + 1];
                var ε12 = l1 / l_0[2 * i + 1] - 1;
                var N12 = ES * ε12;
                var dN12 = (l1 / 2) * (κF12 + f3);
                var dε12 = dN12 / ES;

                // 0
                ε[2 * i] = ε01;
                N_mid[2 * i] = N01; 
                N_r[2 * i] = N01 + dN01;
                ε_r[2 * i] = ε01 + dε01;

                // 1
                ε_l[2 * i + 1] = ε01 - dε01;
                ε_r[2 * i + 1] = ε12 + dε12;

                N_l[2 * i + 1] = N01 - dN01;
                N_r[2 * i + 1] = N12 + dN12;

                // 2
                ε[2 * i + 1] = ε12;
                N_mid[2 * i + 1] = ES * ε12;
                N_l[2 * i + 2] = N12 - dN12;
                ε_l[2 * i + 2] = ε12 - dε12;

                // =========================== SHEAR INTERPOLATION =====================================

                var dF1_01 = (l0 / 2) * (κ2_mid[2 * i] * N_mid[2 * i] - τ[2 * i] * F2_mid[2 * i] + f1);
                var dF1_12 = (l0 / 2) * (κ2_mid[2 * i + 1] * N_mid[2 * i + 1] - τ[2 * i + 1] * F2_mid[2 * i + 1] + f1);

                var dF2_01 = (l0 / 2) * (-κ1_mid[2 * i] * N_mid[2 * i] + τ[2 * i] * F1_mid[2 * i] + f2);
                var dF2_12 = (l0 / 2) * (-κ1_mid[2 * i + 1] * N_mid[2 * i + 1] + τ[2 * i + 1] * F1_mid[2 * i + 1] + f2);

                // 2i
                F1_r[2*i] = F1_mid[2 * i] + dF1_01;
                F2_r[2 * i] = F2_mid[2 * i] + dF2_01;
                V_r[2 * i] = F1_r[2 * i] * mframes[2 * i].XAxis + F2_r[2 * i] * mframes[2 * i].YAxis;


                // 2i + 1
                F1_l[2 * i + 1] = F1_mid[2 * i] - dF1_01;
                F2_l[2 * i + 1] = F2_mid[2 * i] - dF2_01;
                V_l[2 * i + 1] = F1_l[2 * i + 1] * mframes[2 * i + 1].XAxis + F2_l[2 * i + 1] * mframes[2 * i + 1].YAxis;


                F1_r[2 * i + 1] = F1_mid[2 * i + 1] + dF1_12;
                F2_r[2 * i + 1] = F2_mid[2 * i + 1] + dF2_12;
                V_r[2 * i + 1] = F1_r[2 * i + 1] * mframes[2 * i + 1].XAxis + F2_r[2 * i + 1] * mframes[2 * i + 1].YAxis;

                // 2i + 2
                F1_l[2 * i + 2] = F1_mid[2 * i + 1] - dF1_12;
                F2_l[2 * i + 2] = F2_mid[2 * i + 1] - dF2_12;
                V_l[2 * i + 2] = F1_l[2 * i + 2] * mframes[2 * i + 2].XAxis + F2_l[2 * i + 2] * mframes[2 * i + 2].YAxis;

            }
        }
        private void UpdateInternalNodalForce()
        {
            // AXIAL FORCE : attention à ne pas répéter cette opération pour les variations sur θ

            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // ne prend pas en compte les efforts linéiques
            //Rint_x_axial[0] = ((N_mid[0] * t_mid[0] + V_mid[0]) * t[0]) * t[0];
            //Rint_x_axial[0] = N_mid[0] * t_mid[0];
            //Rint_x_axial[0] = N_r[0];

            //Rint_x_axial[1] = -N_mid[0] * t_mid[0];
            //for (int i = 1; i < ne - 1; i++)
            //{
            //    MVector force = N_mid[i] * t_mid[i];
            //    Rint_x_axial[i] += force;
            //    Rint_x_axial[i + 1] = -force;
            //}
            //Rint_x_axial[nv - 2] += N_mid[ne - 1] * t_mid[ne - 1];
            //Rint_x_axial[nv - 1] = -((N_mid[ne - 1] * t_mid[ne - 1] + V_M[ne - 1] + V_Q[ne - 1]) * t[nv - 1]) * t[nv - 1];

            //// SHEAR FORCE | bending contribution
            //// A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            //// ne prend pas en compte les efforts linéiques
            //Rint_x_shear_M[0] = (N_mid[0] * t_mid[0]) - ((N_mid[0] * t_mid[0] + V_M[0]) * t[0]) * t[0];
            //for (int i = 0; i < ne; i++)
            //{
            //    Rint_x_shear_M[i] += V_M[i];
            //    Rint_x_shear_M[i + 1] = -V_M[i];
            //}
            //Rint_x_shear_M[nv - 1] += -N_mid[ne - 1] * t_mid[ne - 1] + ((N_mid[ne - 1] * t_mid[ne - 1] + V_M[ne - 1]) * t[nv - 1]) * t[nv - 1];

            //// SHEAR FORCE | twisting contribution
            //Rint_x_shear_Q[0] = -(V_Q[0] * t[0]) * t[0];
            //for (int i = 0; i < ne; i++)
            //{
            //    Rint_x_shear_Q[i] += V_Q[i];
            //    Rint_x_shear_Q[i + 1] = -V_Q[i];
            //}
            //Rint_x_shear_Q[nv - 1] += (V_Q[nv_h - 1] * t[nv - 1]) * t[nv - 1];

            //// RESULTANT FORCE
            //for (int i = 0; i < nv; i++)
            //{
            //    Rint_x[i] = Rint_x_axial[i] + Rint_x_shear_M[i] + Rint_x_shear_Q[i];
            //}

            //RESULTANT FORCE
            Rint_x[0] = N_mid[0] * t[0] + V_mid[0];
            for (int i = 1; i < nv - 1; i++)
            {
                var N = -N_mid[i - 1] * t_mid[i - 1] + N_mid[i] * t_mid[i];
                Rint_x[i] = N - V_mid[i - 1] + V_mid[i];
            }
            Rint_x[nv - 1] = -N_mid[ne - 1] * t[nv - 1] - V_mid[ne - 1];

            //Rint_x[0] = MVector.Zero;
            //for (int i = 0; i < nv_g; i++)
            //{
            //    var l0 = l[2 * i];
            //    var l1 = l[2 * i + 1];

            //    var κV0 = MVector.CrossProduct(κb_h_r[i], V_r[2 * i]) * mframes[2 * i].ZAxis;
            //    var κV1 = MVector.CrossProduct(κb_g[i], 0.5 * (V_l[2 * i + 1] + V_r[2 * i + 1])) * mframes[2 * i + 1].ZAxis;
            //    var κV2 = MVector.CrossProduct(κb_h_l[i + 1], V_l[2 * i + 2]) * mframes[2 * i + 2].ZAxis;

            //    Rint_x[2 * i] += (N_mid[2 * i] + l0 / 4 * (κV0 + κV1)) * mframes[2 * i].ZAxis;
            //    Rint_x[2 * i] += V_mid[2 * i];

            //    Rint_x[2 * i + 1] = (-N_mid[2 * i] + N_mid[2 * i + 1] + l0 / 4 * (κV0 + κV1) + l1 / 4 * (κV1 + κV2)) * mframes[2 * i + 1].ZAxis;
            //    Rint_x[2 * i + 1] += -V_mid[2 * i] + V_mid[2 * i + 1];

            //    Rint_x[2 * i + 2] = (-N_mid[2 * i + 1] + l1 / 4 * (κV1 + κV2)) * mframes[2 * i].ZAxis;
            //    Rint_x[2 * i + 2] += -V_mid[2 * i + 1];
            //}

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
        }


        // GEOMETRY
        public MFrame[] Get_MaterialFrames(Configuration config)
        {
            switch (config)
            {
                case Configuration.Rest:
                    return mframes_0.ToArray();
                case Configuration.Initial:
                    return mframes_i.ToArray();
                default:
                    return mframes.ToArray();
            }
        }
        public MFrame[] Get_MaterialFramesAtMid()
        {
            return mframes_mid.ToArray();
        }
        public double Get_Length()
        {
            return l.Sum();
        }
        public double[] Get_l()
        {
            return l.ToArray();
        }
        public MVector[] Get_e()
        {
            return e.ToArray();
        }
        public MPoint[] Get_x()
        {
            return x.ToArray();
        }

        // ENERGIES
        public double Get_Ea(out double[] Ea)
        {
            double sum = 0;
            Ea = new double[ne];
            for (int i = 0; i < ne; i++)
            {
                var dE = 0.5 * ES[i / 2] * (ε[i] * ε[i]) * l[i];
                sum += dE;
                Ea[i] = dE;
            }
            return sum;
        }
        public double Get_Eb(out double[] Eb)
        {
            double κ1, κ2;
            double sum = 0;
            Eb = new double[ne];
            for (int i = 0; i < ne; i++)
            {
                κ1 = κb_mid[i] * mframes_mid[i].XAxis - κb_mid_0[i].X;
                κ2 = κb_mid[i] * mframes_mid[i].YAxis - κb_mid_0[i].Y;
                var dE = 0.5 * (EI1[i/2] * κ1 * κ1 + EI2[i/2] * κ2 * κ2) * l[i];
                sum += dE;
                Eb[i] = dE;
            }
            return sum;
        }
        public double Get_Et(out double[] Et)
        {
            double sum = 0;
            Et = new double[ne];
            for (int i = 0; i < ne; i++)
            {
                double τ = (this.τ[i] - τ_0[i]);
                var dE = 0.5 * GJ[i / 2] * (τ * τ) * l[i];
                sum += dE;
                Et[i] = dE;
            }
            return sum;
        }
        public void Diagram_Energy(double[] E, out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(E, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(E, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(E, mframes, scale, axis, out startPoints, out endPoints);
        }

        // DISPLAY
        public void Get_Q(out CMoment[] Ql, out CMoment[] Qr, out CMoment[] Qmid)
        {
            Ql = new CMoment[nv];
            Qr = new CMoment[nv];
            Qmid = new CMoment[ne];

            for (int i = 0; i < ne; i++)
            {
                var d3 = mframes[i].ZAxis;
                Qmid[i] = new CMoment(Q_mid[i] * t_mid[i], mframes_mid[i]);
            }
            for (int i = 0; i < nv; i++)
            {
                var d3 = mframes[i].ZAxis;
                Ql[i] = new CMoment(Q_l[i] * d3, mframes[i]);
                Qr[i] = new CMoment(Q_r[i] * d3, mframes[i]);
            }
        }
        public void Get_Q(out MVector[] Ql, out MVector[] Qr, out MVector[] Qmid)
        {
            Ql = new MVector[nv];
            Qr = new MVector[nv];
            Qmid = new MVector[ne];

            for (int i = 0; i < ne; i++)
            {
                var d3 = mframes[i].ZAxis;
                Qmid[i] = Q_mid[i] * t_mid[i];
            }
            for (int i = 0; i < nv; i++)
            {
                var d3 = mframes[i].ZAxis;
                Ql[i] = Q_l[i] * d3;
                Qr[i] = Q_r[i] * d3;
            }
        }
        public void Diagram_Q(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(Q_l, Q_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(Q_l, Q_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(Q_l, Q_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        public void Get_M(out CMoment[] Ml, out CMoment[] Mr)
        {
            Ml = new CMoment[nv];
            Mr = new CMoment[nv];

            for (int i = 0; i < nv_g; i++)
            {
                Ml[2 * i + 1] = new CMoment(M_g[i], mframes[2 * i + 1]);
                Mr[2 * i + 1] = new CMoment(M_g[i], mframes[2 * i + 1]);
            }
            for (int i = 0; i < nv_h; i++)
            {
                Ml[2 * i] = new CMoment(M_l[i], mframes[2 * i]);
                Mr[2 * i] = new CMoment(M_r[i], mframes[2 * i]);
            }
        }
        public void Get_M(out MVector[] Ml, out MVector[] Mr)
        {
            Ml = new MVector[nv];
            Mr = new MVector[nv];

            for (int i = 0; i < nv_g; i++)
            {
                Ml[2 * i + 1] = M_g[i];
                Mr[2 * i + 1] = M_g[i];
            }
            for (int i = 0; i < nv_h; i++)
            {
                Ml[2 * i] = M_l[i];
                Mr[2 * i] = M_r[i];
            }
        }
        public void Diagram_M(out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2, double scale, Configuration config)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_vector_12(M_l, M_g, M_r, mframes, mframes_0, scale, out startPoints, out endPoints_1, out endPoints_2);
            else if (config == Configuration.Initial)
                Diagram_vector_12(M_l, M_g, M_r, mframes, mframes_i, scale, out startPoints, out endPoints_1, out endPoints_2);
            else
                Diagram_vector_12(M_l, M_g, M_r, mframes, mframes, scale, out startPoints, out endPoints_1, out endPoints_2);
        }

        public void Get_V(out CForce[] Vl, out CForce[] Vr, out CForce[] Vmid)
        {
            Vl = new CForce[nv];
            Vr = new CForce[nv];
            Vmid = new CForce[ne];

            for (int i = 0; i < ne; i++)
            {
                Vmid[i] = new CForce(V_mid[i], mframes_mid[i]);
            }
            for (int i = 0; i < nv; i++)
            {
                Vl[i] = new CForce(V_l[i], mframes[i]);
                Vr[i] = new CForce(V_r[i], mframes[i]);
            }
        }
        public void Get_V(out MVector[] Vl, out MVector[] Vr, out MVector[] Vmid)
        {
            Vl = new MVector[nv];
            Vr = new MVector[nv];
            Vmid = new MVector[ne];

            for (int i = 0; i < ne; i++)
            {
                Vmid[i] = V_mid[i];
            }
            for (int i = 0; i < nv; i++)
            {
                Vl[i] = V_l[i];
                Vr[i] = V_r[i];
            }
        }
        public void Diagram_V(out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2, double scale, Configuration config)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_vector_12(V_l, V_r, mframes, mframes_0, scale, out startPoints, out endPoints_1, out endPoints_2);
            else if (config == Configuration.Initial)
                Diagram_vector_12(V_l, V_r, mframes, mframes_i, scale, out startPoints, out endPoints_1, out endPoints_2);
            else
                Diagram_vector_12(V_l, V_r, mframes, mframes, scale, out startPoints, out endPoints_1, out endPoints_2);
        }

        public void Get_N(out CForce[] Nl, out CForce[] Nr, out CForce[] Nmid)
        {
            Nl = new CForce[nv];
            Nr = new CForce[nv];
            Nmid = new CForce[ne];

            for (int i = 0; i < ne; i++)
            {
                var d3 = mframes[i].ZAxis;
                Nmid[i] = new CForce(N_mid[i] * t_mid[i], mframes_mid[i]);
            }
            for (int i = 0; i < nv; i++)
            {
                var d3 = mframes[i].ZAxis;
                Nl[i] = new CForce(N_l[i] * d3, mframes[i]);
                Nr[i] = new CForce(N_r[i] * d3, mframes[i]);
            }
        }
        public void Get_N(out MVector[] Nl, out MVector[] Nr, out MVector[] Nmid)
        {
            Nl = new MVector[nv];
            Nr = new MVector[nv];
            Nmid = new MVector[ne];

            for (int i = 0; i < ne; i++)
            {
                var d3 = mframes[i].ZAxis;
                Nmid[i] = N_mid[i] * t_mid[i];
            }
            for (int i = 0; i < nv; i++)
            {
                var d3 = mframes[i].ZAxis;
                Nl[i] = N_l[i] * d3;
                Nr[i] = N_r[i] * d3;
            }
        }
        public void Diagram_N(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(N_l, N_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(N_l, N_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(N_l, N_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        public void Get_κ(out MVector[] κbl, out MVector[] κbr, out MVector[] κbmid)
        {
            κbl = new MVector[nv];
            κbr = new MVector[nv];
            κbmid = κb_mid.DeepCopy();

            for (int i = 0; i < nv_g; i++)
            {
                κbl[2 * i + 1] = κb_g[i];
                κbr[2 * i + 1] = κb_g[i];
            }
            for (int i = 0; i < nv_h; i++)
            {
                κbl[2 * i] = κb_l[i];
                κbr[2 * i] = κb_r[i];
            }
        }
        public void Diagram_κ(out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2, double scale, Configuration config)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_vector_12(κb_l, κb_g, κb_r, mframes, mframes_0, scale, out startPoints, out endPoints_1, out endPoints_2);
            else if (config == Configuration.Initial)
                Diagram_vector_12(κb_l, κb_g, κb_r, mframes, mframes_i, scale, out startPoints, out endPoints_1, out endPoints_2);
            else
                Diagram_vector_12(κb_l, κb_g, κb_r, mframes, mframes, scale, out startPoints, out endPoints_1, out endPoints_2);
        }

        public void Get_τ(out double[] τl, out double[] τr, out double[] τmid)
        {
            τl = τ_l.ToArray();
            τr = τ_r.ToArray();
            τmid = τ.ToArray();
        }
        public void Diagram_τ(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(τ_l, τ_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(τ_l, τ_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(τ_l, τ_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        public void Get_ε(out double[] εl, out double[] εr, out double[] εmid)
        {
            εl = ε_l.ToArray();
            εr = ε_r.ToArray();
            εmid = ε.ToArray();
        }
        public void Diagram_ε(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(ε_l, ε_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(ε_l, ε_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(ε_l, ε_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        // Diagram Helper
        private void Diagram_scalar(double[] vmid,  MFrame[] displayFrames, double scale, Axis axis, out MPoint[] startPoints, out MPoint[] endPoints)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int ne = vmid.Length;

            startPoints = new MPoint[2 * ne];
            endPoints = new MPoint[2 * ne];

            var dir = displayFrames.GetAxis(axis);

            for (int i = 0; i < ne; i++)
            {
                var P = displayFrames[i].Origin;

                startPoints[2 * i] = P;
                endPoints[2 * i] = P + scale * vmid[i] * dir[i];

                startPoints[2 * i + 1] = P;
                endPoints[2 * i + 1] = P + scale * vmid[i] * dir[i + 1];
            }
        }
        private void Diagram_scalar(double[] vl, double[] vr, MFrame[] displayFrames, double scale, Axis axis, out MPoint[] startPoints, out MPoint[] endPoints)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int nv = vl.Length;

            startPoints = new MPoint[2 * nv];
            endPoints = new MPoint[2 * nv];

            var dir = displayFrames.GetAxis(axis);

            for (int i = 0; i < nv; i++)
            {
                var P = displayFrames[i].Origin;

                startPoints[2 * i] = P;
                endPoints[2 * i] = P + scale * vl[i] * dir[i];

                startPoints[2 * i + 1] = P;
                endPoints[2 * i + 1] = P + scale * vr[i] * dir[i];
            }
        }
        private void Diagram_scalar(double[] vl, double[] vg, double[] vr, MFrame[] displayFrames, double scale, Axis axis, out MPoint[] startPoints, out MPoint[] endPoints)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int nh = vl.Length;
            int ng = vg.Length;

            startPoints = new MPoint[2 * nv_h + nv_g];
            endPoints = new MPoint[2 * nv_h + nv_g];

            var dir = displayFrames.GetAxis(axis);

            MPoint P;

            for (int i = 0; i < nv_g; i++)
            {
                var ig = 3 * i + 2; // ghost index in startPoints & endPoints
                P = displayFrames[2 * i + 1].Origin;
                startPoints[ig] = P;
                endPoints[ig] = P + scale * vg[i] * dir[2 * i + 1];
            }
            for (int i = 0; i < nv_h; i++)
            {
                P = displayFrames[2 * i].Origin;

                var il = 3 * i;     // handle left index in startPoints & endPoints
                startPoints[il] = P;
                endPoints[il] = P + scale * vl[i] * dir[2 * i];

                var ir = il + 1;    // handle right index in startPoints & endPoints
                startPoints[ir] = P;
                endPoints[ir] = P + scale * vr[i] * dir[2 * i];
            }
        }

        private void Diagram_vector_12(MVector[] vl, MVector[] vr, MFrame[] valueFrames, MFrame[] displayFrames, double scale, out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int n = vl.Length;

            startPoints = new MPoint[2 * n];
            endPoints_1 = new MPoint[2 * n];
            endPoints_2 = new MPoint[2 * n];

            var d1 = displayFrames.GetAxis(Axis.d1);
            var d2 = displayFrames.GetAxis(Axis.d2);

            for (int i = 0; i < n; i++)
            {
                var P = displayFrames[i].Origin;

                startPoints[2 * i] = P;
                endPoints_1[2 * i] = P + scale * (vl[i] * valueFrames[i].XAxis) * d1[i];
                endPoints_2[2 * i] = P + scale * (vl[i] * valueFrames[i].YAxis) * d2[i];

                startPoints[2 * i + 1] = P;
                endPoints_1[2 * i + 1] = P + scale * (vr[i] * valueFrames[i].XAxis) * d1[i];
                endPoints_2[2 * i + 1] = P + scale * (vr[i] * valueFrames[i].YAxis) * d2[i];
            }
        }
        private void Diagram_vector_12(MVector[] vl, MVector[] vg, MVector[] vr, MFrame[] valueFrames, MFrame[] displayFrames, double scale, out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int nh = vl.Length;
            int ng = vg.Length;

            startPoints = new MPoint[2 * nv_h + nv_g];
            endPoints_1 = new MPoint[2 * nv_h + nv_g];
            endPoints_2 = new MPoint[2 * nv_h + nv_g];


            var d1 = displayFrames.GetAxis(Axis.d1);
            var d2 = displayFrames.GetAxis(Axis.d2);

            MPoint P;

            for (int i = 0; i < nv_g; i++)
            {
                var ig = 3 * i + 2; // ghost index in startPoints & endPoints
                P = displayFrames[2 * i + 1].Origin;
                startPoints[ig] = P;
                endPoints_1[ig] = P + scale * (vg[i] * valueFrames[2 * i + 1].XAxis) * d1[2 * i + 1];
                endPoints_2[ig] = P + scale * (vg[i] * valueFrames[2 * i + 1].YAxis) * d2[2 * i + 1];
            }
            for (int i = 0; i < nv_h; i++)
            {
                P = displayFrames[2 * i].Origin;

                var il = 3 * i;     // handle left index in startPoints & endPoints
                startPoints[il] = P;
                endPoints_1[il] = P + scale * (vl[i] * valueFrames[2 * i].XAxis) * d1[2 * i];
                endPoints_2[il] = P + scale * (vl[i] * valueFrames[2 * i].YAxis) * d2[2 * i];

                var ir = il + 1;    // handle right index in startPoints & endPoints
                startPoints[ir] = P;
                endPoints_1[ir] = P + scale * (vr[i] * valueFrames[2 * i].XAxis) * d1[2 * i];
                endPoints_2[ir] = P + scale * (vr[i] * valueFrames[2 * i].YAxis) * d2[2 * i];
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
