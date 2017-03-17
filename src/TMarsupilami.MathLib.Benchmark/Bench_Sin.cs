using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Attributes.Jobs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib.Benchmark
{

    public class Bench_Sin
    {
        private const int N = 1000;
        private readonly double[] angles, resAngles, val;
        private double inf = -Math.PI;
        private double sup = Math.PI;


        public Bench_Sin()
        {
            angles = new double[N];
            resAngles = new double[N];
            val = new double[N];

            var delta = (sup - inf);

            var rdm = new Random();
            for (int i = 0; i < N; i++)
            {
                double t = rdm.NextDouble();
                val[i] = t;
                double angle = inf + delta * t;
                var c = Math.Cos(angle);
                var s = Math.Sin(angle);
                angles[i] = angle;
            }
        }

        [Benchmark(Baseline = true)]
        public double[] Prod()
        {
            for (int i = 0; i < N; i++)
            {
                double angle = angles[i];
                resAngles[i] = angle * angle;
            }
            return resAngles;
        }

        [Benchmark(Baseline = false)]
        public double[] Sin()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Math.Sin(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Cos()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Math.Cos(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Acos()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Math.Acos(val[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Asin()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Math.Asin(val[i]);
            }
            return resAngles;
        }

        #region TAYLOR

        [Benchmark]
        public double[] Sin_T1()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_T1(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Sin_T3()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_T3(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Sin_T5()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_T5(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Sin_T7()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_T7(angles[i]);
            }
            return resAngles;
        }

        #endregion

        #region REMEZ
        [Benchmark]
        public double[] Sin_R3()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_R3(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Sin_R5()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_R5(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Sin_R7()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_R7(angles[i]);
            }
            return resAngles;
        }

        [Benchmark]
        public double[] Sin_R9()
        {
            for (int i = 0; i < N; i++)
            {
                resAngles[i] = Trigo.Sin_R9(angles[i]);
            }
            return resAngles;
        }

#endregion
    }
}
