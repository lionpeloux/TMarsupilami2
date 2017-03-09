using BenchmarkDotNet.Attributes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib.Benchmark
{

    public class Bench_Rotation
    {
        private const int N = 100000;
        private readonly MFrame[] frames, resFrames;
        private readonly double[] angles;


        public Bench_Rotation()
        {
            frames = new MFrame[N];
            resFrames = new MFrame[N];
            angles = new double[N];
            var rdm = new Random();
            for (int i = 0; i < N; i++)
            {
                double angle = rdm.NextDouble();
                var c = Math.Cos(angle);
                var s = Math.Sin(angle);
                frames[i] = new MFrame(new MPoint(0, 0, 0), new MVector(c, s, 0), new MVector(-s, c, 0));
                angles[i] = -angle;
            }
        }

        [Benchmark(Baseline = true)]
        public MFrame[] ZRotate()
        {
            for (int i = 0; i < N; i++)
            {
                Rotation.ZRotate(resFrames[i], angles[i], ref resFrames[i]);
            }
            return resFrames;
        }

        [Benchmark]
        public MFrame[] ZRotate_Diff()
        {
            for (int i = 0; i < N; i++)
            {
                Rotation.ZDiffRotate_Taylor_3(resFrames[i], angles[i], ref resFrames[i]);
            }
            return resFrames;
        }
    }
}
