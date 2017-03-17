using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Attributes.Jobs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib.Benchmark
{

    public class Bench_Sqrt
    {
        private const int N = 1000;
        private readonly double[] resVal, val;
        private  MVector[] resVec, vec;
        private double inf = 1 - 0.1;
        private double sup = 1 + 0.1;

        public Bench_Sqrt()
        {
            val = new double[N];
            resVal = new double[N];

            vec = new MVector[N];
            resVec = new MVector[N];

            var delta = (sup - inf);

            var rdm = new Random();
            for (int i = 0; i < N; i++)
            {
                double t = rdm.NextDouble();
                val[i] = inf + delta * t;
                vec[i] = new MVector(inf + delta * t, inf + delta * t, inf + delta * t);
            }
        }

        [Benchmark(Baseline = true)]
        public double[] Prod()
        {
            for (int i = 0; i < N; i++)
            {
                double v = val[i];
                resVal[i] = v * v;
            }
            return resVal;
        }

        [Benchmark(Baseline = false)]
        public double[] Sqrt()
        {
            for (int i = 0; i < N; i++)
            {
                resVal[i] = Math.Sqrt(val[i]);
            }
            return resVal;
        }

        [Benchmark]
        public double[] InvSqrt()
        {
            for (int i = 0; i < N; i++)
            {
                resVal[i] = 1/Math.Sqrt(val[i]);
            }
            return resVal;
        }

        [Benchmark]
        public double[] InvSqrt_R2()
        {
            for (int i = 0; i < N; i++)
            {
                resVal[i] = MathLib.Sqrt.InvSqrt_R2((val[i]));
            }
            return resVal;
        }

        [Benchmark]
        public double[] InvSqrt_R2_N1()
        {
            for (int i = 0; i < N; i++)
            {
                resVal[i] = MathLib.Sqrt.InvSqrt_R2_N1((val[i]));
            }
            return resVal;
        }

        [Benchmark]
        public double[] InvSqrt_R2_N2()
        {
            for (int i = 0; i < N; i++)
            {
                resVal[i] = MathLib.Sqrt.InvSqrt_R2_N2((val[i]));
            }
            return resVal;
        }

        [Benchmark]
        public MVector[] Normalize()
        {
            for (int i = 0; i < N; i++)
            {
                var v = vec[i];
                resVec[i] = MVector.Normalize(v);

            }
            return resVec;
        }

        [Benchmark]
        public MVector[] ReNormalize()
        {
            for (int i = 0; i < N; i++)
            {
                var v = vec[i];
                resVec[i] = MVector.ReNormalize(v);
            }
            return resVec;
        }

    }
}
