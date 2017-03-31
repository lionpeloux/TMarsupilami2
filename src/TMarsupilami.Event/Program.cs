using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Management;
using System.Threading;
using System.Threading.Tasks;

namespace TMarsupilami.Event
{
    class ConsoleTest
    {
        static void Main(string[] args)
        {
            int n = 20;
            int f = 1;
            TestEvent(n, f);
            TestCluster(n, f);
            TestCluster(n, f);
            TestCluster(n, f);

            Console.ReadKey();
        }

        public static void TestEvent(int n, int factor = 1)
        {
            var eventBuffer = new Event<object, EventArgs>();
            var handlers = new EventHandler<object, EventArgs>[n];
            var sleepTimes = new int[n];
            var rdm = new Random();

            for (int i = 0; i < handlers.Length; i++)
            {
                // assign to local variables to ensure proper closure
                int index = i;
                int sleepTime = factor * rdm.Next(1, 10);
                sleepTimes[i] = sleepTime;
                handlers[i] = (eventSender, eventArgs) =>
                {
                    Thread.Sleep(sleepTime);
                    Debug.WriteLine("handler[" + index + "] : slept " + sleepTime + " ms");
                };

                //eventBuffer.Subscribe(handlers[i]);
                eventBuffer.Subscribe(handlers[i]);

            }

            int numberOfPhysicalProcessors, numberOfCoresPerPrhysicalProcessor, numberOfLogicalProcessorsPerCore;
            GetProcessorCounts(out numberOfPhysicalProcessors, out numberOfCoresPerPrhysicalProcessor, out numberOfLogicalProcessorsPerCore);

            var watch = new Stopwatch();
            Console.WriteLine("=========");
            Console.WriteLine("Sync Call");
            Console.WriteLine("=========");
            eventBuffer.IsParallelModeEnabled = false;
            watch.Start();
            eventBuffer.Raise(null, new EventArgs());
            watch.Stop();
            double tsync = watch.ElapsedMilliseconds;
            Console.WriteLine("Elapsed = " + tsync);

            Console.WriteLine("=========");
            Console.WriteLine("Async Parallel Call");
            Console.WriteLine("Number of Physical Processors = {0}", numberOfPhysicalProcessors);
            Console.WriteLine("Number of Core per Processor = {0}", numberOfCoresPerPrhysicalProcessor);
            Console.WriteLine("Number of Logical Processor per Core = {0}", numberOfLogicalProcessorsPerCore);
            Console.WriteLine("Degree of Parallelism = {0}", numberOfPhysicalProcessors * numberOfCoresPerPrhysicalProcessor * numberOfLogicalProcessorsPerCore);
            Console.WriteLine("Parallel Option MaxDegreeOfParallelism = " + eventBuffer.ParallelOptions.MaxDegreeOfParallelism);
            Console.WriteLine("=========");
            eventBuffer.IsParallelModeEnabled = true;
            watch.Restart();
            eventBuffer.Raise(null, new EventArgs());
            watch.Stop();
            double tasync = watch.ElapsedMilliseconds;
            Console.WriteLine("Elapsed = " + tasync);
            Console.WriteLine("x times = " + Math.Round(tsync / tasync, 2) + " faster");
        }
        public static void TestCluster(int n, int factor = 1)
        {
            var stack = new Cluster();
            var methods = new Action[n];
            var sleepTimes = new int[n];
            var rdm = new Random();

            for (int i = 0; i < methods.Length; i++)
            {
                // assign to local variables to ensure proper closure
                int index = i;
                int sleepTime = factor * rdm.Next(1, 10);
                sleepTimes[i] = sleepTime;
                methods[i] = () =>
                {
                    Thread.Sleep(sleepTime);
                    Debug.WriteLine("handler[" + index + "] : slept " + sleepTime + " ms");
                };

                //stack.Subscribe(methods[i]);

            }

            stack.Subscribe(methods);

            int numberOfPhysicalProcessors, numberOfCoresPerPrhysicalProcessor, numberOfLogicalProcessorsPerCore;
            GetProcessorCounts(out numberOfPhysicalProcessors, out numberOfCoresPerPrhysicalProcessor, out numberOfLogicalProcessorsPerCore);

            var watch = new Stopwatch();
            Console.WriteLine("=========");
            Console.WriteLine("Sync Call");
            Console.WriteLine("=========");
            stack.IsParallelModeEnabled = false;
            watch.Start();
            stack.Call();
            watch.Stop();
            double tsync = watch.ElapsedMilliseconds;
            Console.WriteLine("Elapsed = " + tsync);

            Console.WriteLine("=========");
            Console.WriteLine("Async Parallel Call");
            Console.WriteLine("Number of Physical Processors = {0}", numberOfPhysicalProcessors);
            Console.WriteLine("Number of Core per Processor = {0}", numberOfCoresPerPrhysicalProcessor);
            Console.WriteLine("Number of Logical Processor per Core = {0}", numberOfLogicalProcessorsPerCore);
            Console.WriteLine("Degree of Parallelism = {0}", numberOfPhysicalProcessors * numberOfCoresPerPrhysicalProcessor * numberOfLogicalProcessorsPerCore);
            Console.WriteLine("Parallel Option MaxDegreeOfParallelism = " + stack.ParallelOptions.MaxDegreeOfParallelism);
            Console.WriteLine("=========");
            stack.IsParallelModeEnabled = true;
            watch.Restart();
            stack.Call();
            watch.Stop();
            double tasync = watch.ElapsedMilliseconds;
            Console.WriteLine("Elapsed = " + tasync);
            Console.WriteLine("x times = " + Math.Round(tsync / tasync, 2) + " faster");
        }

        public static void GetProcessorCounts(out int num_physical_processors, out int num_cores, out int num_logical_processors)
        {
            string query;
            ManagementObjectSearcher searcher;

            // Get the number of physical processors.
            num_physical_processors = 0;
            query = "SELECT * FROM Win32_ComputerSystem";
            searcher = new ManagementObjectSearcher(query);
            foreach (ManagementObject sys in searcher.Get())
                num_physical_processors =
                    int.Parse(sys["NumberOfProcessors"].ToString());

            // Get the number of cores.
            query = "SELECT * FROM Win32_Processor";
            num_cores = 0;
            searcher = new ManagementObjectSearcher(query);
            foreach (ManagementObject proc in searcher.Get())
                num_cores += int.Parse(proc["NumberOfCores"].ToString());

            num_logical_processors = Environment.ProcessorCount;
        }

    }

  

    

}


