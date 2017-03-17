using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Attributes.Jobs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib.Benchmark
{

    /// <summary>
    /// Compares different ways to modifiy an array of MVector structs and the cost of allocation.
    /// </summary>
    public class Bench_VectorAlloc
    {
        private const int N = 100000;
        private readonly MVector[] resVal, val;
        private double inf = 1 - 0.1;
        private double sup = 1 + 0.1;

        public Bench_VectorAlloc()
        {
            val = new MVector[N];
            resVal = new MVector[N];

            var delta = (sup - inf);

            var rdm = new Random();
            for (int i = 0; i < N; i++)
            {
                double t = rdm.NextDouble();
                val[i] = new MVector(inf + delta * t, inf + delta * t / 2, inf + delta * t / 4);
                resVal[i] = new MVector(0, 0, 0);
            }
        }

        public static MVector Neg1(MVector v) { return new MVector(-v.X, -v.Y, -v.Z); }
        public static void Neg2(MVector v, ref MVector r) { r.X = -v.X; r.Y = -v.Y; r.Z = -v.Z; }
        public static void Neg3(MVector v, out MVector r) { r = new MVector(-v.X, -v.Y, -v.Z); }

        public static MVector Mul1(MVector v) { return new MVector(v.X * v.X, v.Y * v.Y, v.Z * v.Z); }
        public static void Mul2(MVector v, ref MVector r) { r.X = v.X*v.X; r.Y = v.Y*v.Y; r.Z = v.Z * v.Z; }
        public static void Mul3(MVector v, out MVector r) { r = new MVector(v.X * v.X, v.Y * v.Y, v.Z * v.Z); }

        public static MVector Sqrt1(MVector v) { return new MVector(Math.Sqrt(v.X), Math.Sqrt(v.Y), Math.Sqrt(v.Z)); }
        public static void Sqrt2(MVector v, ref MVector r) { r.X = Math.Sqrt(v.X); r.Y = Math.Sqrt(v.Y); r.Z = Math.Sqrt(v.Z); }
        public static void Sqrt3(MVector v, out MVector r) { r = new MVector(Math.Sqrt(v.X), Math.Sqrt(v.Y), Math.Sqrt(v.Z)); }

        [Benchmark(Baseline = true)]
        public MVector[] Alloc_Neg()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                resVal[i] = Neg1(v);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Ref_Neg()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                Neg2(v, ref resVal[i]);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Out_Neg()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                Neg3(v, out resVal[i]);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Alloc_Mul()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                resVal[i] = Mul1(v);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Ref_Mul()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                Mul2(v, ref resVal[i]);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Out_Mul()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                Mul3(v, out resVal[i]);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Alloc_Sqrt()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                resVal[i] = Sqrt1(v);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Ref_Sqrt()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                Sqrt2(v, ref resVal[i]);
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Out_Sqrt()
        {
            for (int i = 0; i < N; i++)
            {
                var v = val[i];
                Sqrt3(v, out resVal[i]);
            }
            return resVal;
        }

    }
}
