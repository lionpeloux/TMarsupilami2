using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Attributes.Jobs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib.Benchmark
{
    //[LegacyJitX86Job, LegacyJitX64Job, RyuJitX64Job]
    public class Bench_ParallelTransport
    {
        private const int N = 100000;
        private readonly MFrame[] frames, resFrames;
        private readonly double[] angles;
        private readonly MPoint[] toPoints;
        private readonly MVector[] fromDirs, toDirs;
        private MVector fromDir;



        public Bench_ParallelTransport()
        {
            frames = new MFrame[N];
            resFrames = new MFrame[N];
            angles = new double[N];
            toPoints = new MPoint[N];
            toDirs = new MVector[N];
            fromDirs = new MVector[N];
            fromDir = new MVector(0, 0, 1);

            var rdm = new Random();
            for (int i = 0; i < N; i++)
            {
                double angle = rdm.NextDouble();
                var c = Math.Cos(angle);
                var s = Math.Sin(angle);
                frames[i] = new MFrame(new MPoint(0, 0, 0), new MVector(c, s, 0), new MVector(-s, c, 0));
                fromDirs[i] = frames[i].ZAxis;
                angles[i] = -angle;

                toPoints[i] = new MPoint(rdm.NextDouble());
                toDirs[i] = new MVector(c, s, 1) / Math.Sqrt(2);
            }
        }

        [Benchmark(Baseline = true)]
        public MFrame[] ZParallelTransport_Reflection()
        {
            for (int i = 0; i < N; i++)
            {
                ParallelTransport.ZPT_Reflection(frames[i], frames[i].Origin, frames[i].ZAxis, toPoints[i], toDirs[i], ref resFrames[i]);
            }
            return resFrames;
        }

        [Benchmark]
        public MFrame[] ZParallelTransport_Rotation()
        {
            for (int i = 0; i < N; i++)
            {
                ParallelTransport.ZPT_Rotation(frames[i], fromDirs[i], toPoints[i], toDirs[i], ref resFrames[i]);
            }
            return resFrames;
        }

        [Benchmark]
        public MFrame[] ParallelTransport_Reflection()
        {
            for (int i = 0; i < N; i++)
            {
                ParallelTransport.PT_Reflection(frames[i], frames[i].Origin, fromDir, toPoints[i], toDirs[i], ref resFrames[i]);
            }
            return resFrames;
        }

        [Benchmark]
        public MFrame[] ParallelTransport_Rotation()
        {
            for (int i = 0; i < N; i++)
            {
                ParallelTransport.PT_Rotation(frames[i], fromDir, toPoints[i], toDirs[i], ref resFrames[i]);
            }
            return resFrames;
        }

        //[Benchmark(Baseline = true)]
        //public MFrame[] ZRotate()
        //{
        //    for (int i = 0; i < N; i++)
        //    {
        //        Rotation.ZRotate(resFrames[i], angles[i], ref resFrames[i]);
        //    }
        //    return resFrames;
        //}

        //[Benchmark]
        //public MFrame[] ZRotate_Diff()
        //{
        //    for (int i = 0; i < N; i++)
        //    {
        //        Rotation.ZDiffRotate_Taylor_3(resFrames[i], angles[i], ref resFrames[i]);
        //    }
        //    return resFrames;
        //}
    }
}
