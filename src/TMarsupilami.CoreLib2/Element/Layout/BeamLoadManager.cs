using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    public class BeamLoadManager
    {
        protected IBeamLayout Layout { get; set; }
        private List<BeamVectorLoad> StaticLoads { get; set; }
        private List<BeamVectorLoad> DynamicLoads { get; set; }

        public bool[] HasDynamicLoads { get; protected set; }
        public bool[] HasStaticLoads { get; protected set; }

        // Fext_g : beam load buffer 0
        // fext_g : beam load buffer 1
        // Mext_g : beam load buffer 2
        // mext_g : beam load buffer 3
        // Fext_m : beam load buffer 4
        // fext_m : beam load buffer 5
        // Mext_m : beam load buffer 6
        // mext_m : beam load buffer 7
        internal MVector[][] LoadBuffers { get; set; }

        public BeamLoadManager(IBeamLayout layout)
        {
            Layout = layout;
            StaticLoads = new List<BeamVectorLoad>();
            DynamicLoads = new List<BeamVectorLoad>();

            LoadBuffers = new MVector[8][];
            for (int i = 0; i < 4; i++)
            {
                LoadBuffers[2 * i] = new MVector[layout.Nv];
                LoadBuffers[2 * i + 1] = new MVector[layout.Ne];
            }
        }
        public void Clear()
        {
            // Clear lists
            StaticLoads.Clear();
            DynamicLoads.Clear();

            // Clear LoadBuffers
            for (int i = 0; i < LoadBuffers.Length; i++)
            {
                MVector.Set(MVector.Zero, ref LoadBuffers[i]);
            }
        }
        public void Fill(IEnumerable<BeamVectorLoad> loads)
        {
            Clear();

            // dispatch loads in to lists
            foreach (var load in loads)
            {
                if (load.IsStatic)
                {
                    StaticLoads.Add(load);
                    AddToLoadBuffer(load);
                }
                else
                {
                    DynamicLoads.Add(load);
                }
            }

            // fill the LoadBuffers with static loads        
            for (int i = 0; i < StaticLoads.Count; i++)
            {
                var load = StaticLoads[i];

            }
        }

        // ici, on peut traiter pour chaque model comment on transmet la charge au buffer.
        // à transférer vers l'objet/class model mécanique.
        // par exemple un modèle qui ne prend pas en compte les charges linéique, ou qui peut linéariser des charges ponctuelles.
        protected void AddToLoadBuffer(BeamVectorLoad load)
        {
            var index = GetLoadBufferIndex(load);
            MVector.Add(load.Value, ref LoadBuffers[index]);
        }
        public static int GetLoadBufferIndex(BeamVectorLoad load)
        {
            return GetLoadBufferIndex(load.Type, load.IsDistributed, load.IsGlobal);
        }
        public static int GetLoadBufferIndex(VectorLoadType type, bool isDistributed, bool isGlobal)
        {
            var index = -1;

            if (isGlobal)
            {
                if (type == VectorLoadType.Force)
                {
                    if (!isDistributed)
                    {
                        index = 0;      // Fext_g
                    }
                    else
                    {
                        index = 1;      // fext_g
                    }
                }
                else
                {
                    if (!isDistributed)
                    {
                        index = 2;      // Mext_g
                    }
                    else
                    {
                        index = 3;      // mext_g
                    }
                }
            }
            else
            {
                if (type == VectorLoadType.Force)
                {
                    if (!isDistributed)
                    {
                        index = 4;      // Fext_m
                    }
                    else
                    {
                        index = 5;      // fext_m
                    }
                }
                else
                {
                    if (!isDistributed)
                    {
                        index = 6;      // Mext_m
                    }
                    else
                    {
                        index = 7;      // mext_m
                    }
                }
            }

            return index;
        }

        public static void ZipStaticLoads(IEnumerable<BeamStaticVectorLoad> loads)
        {
            //var layout = loads.First().Layout;
            //var isUniform = new bool[8] { true, true, true, true, true, true, true, true };
            //var dimension = new int[8] { 0, 0, 0, 0, 0, 0, 0, 0 };
            //var value = new MVector[8][];

            //for (int i = 0; i < 4; i++)
            //{
            //    value[2 * i] = new MVector[layout.Nv];
            //    value[2 * i + 1] = new MVector[layout.Ne];
            //}

            //foreach (var load in loads)
            //{
            //    if (load.Layout != layout)
            //        throw new ArgumentException("All loads must point to the same BeamLayout");

            //    int index = layout.LoadManager.GetLoadBufferIndex(load.Type, load.IsDistributed, load.IsGlobal);
            //    isUniform[index] &= load.IsUniform;
            //    dimension[index] = Math.Max(dimension[index], load.Dimension);
            //    load.AddToLoadBuffer(ref value[index]);
            //}

            //Fext_g = new BeamVectorLoad(VectorLoadType.Force, false, layout, dimension[0], isUniform[0], true);
            //fext_g = new BeamVectorLoad(VectorLoadType.Force, true, layout, dimension[1], isUniform[1], true);
            //Mext_g = new BeamVectorLoad(VectorLoadType.Moment, false, layout, dimension[2], isUniform[2], true);
            //mext_g = new BeamVectorLoad(VectorLoadType.Moment, true, layout, dimension[3], isUniform[3], true);

            //Fext_m = new BeamVectorLoad(VectorLoadType.Force, false, layout, dimension[4], isUniform[4], false);
            //fext_m = new BeamVectorLoad(VectorLoadType.Force, true, layout, dimension[5], isUniform[5], false);
            //Mext_m = new BeamVectorLoad(VectorLoadType.Moment, false, layout, dimension[6], isUniform[6], false);
            //mext_m = new BeamVectorLoad(VectorLoadType.Moment, true, layout, dimension[7], isUniform[7], false);

        }
    }
}
