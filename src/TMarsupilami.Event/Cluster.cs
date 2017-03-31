using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.Event
{
    /// <summary>
    /// This class is an Action Cluster.
    /// It acts similarly to an event with no sender/args informations.
    /// Actions are registered with the Subscribed/Unsubscribed methods.
    /// A collective call can be triggered with the Call() method.
    /// The call is either sync or async wether IsParallelModeEnabled is set to false or true.
    /// </summary>
    public class Cluster
    {
        private Action handler;
        private Delegate[] invocationList;
        private Action call;
        private bool isParallelModeEnabled;

        public bool IsParallelModeEnabled
        {
            get
            {
                return isParallelModeEnabled;
            }
            set
            {
                isParallelModeEnabled = value;

                if (isParallelModeEnabled)
                    call = AsyncCall;
                else
                    call = SyncCall;
            }
        }
        public ParallelOptions ParallelOptions { get; set; }

        public Cluster(bool isParallelModeEnabled = false)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = new ParallelOptions();
        }

        public void Subscribe(Action handler)
        {
            this.handler += handler;
            this.invocationList = this.handler.GetInvocationList();
        }
        public void Subscribe(IEnumerable<Action> handlers)
        {
            foreach (var handler in handlers)
            {
                this.handler += handler;
            }
            this.invocationList = this.handler.GetInvocationList();
        }

        public void UnSubscribe(Action handler)
        {
            this.handler -= handler;
            this.invocationList = this.handler.GetInvocationList();
        }
        public void UnSubscribe(IEnumerable<Action> handlers)
        {
            foreach (var handler in handlers)
            {
                this.handler -= handler;
            }
            this.invocationList = this.handler.GetInvocationList();
        }

        public void Call()
        {
            call();
        }
        private void SyncCall()
        {
            if (invocationList.Length > 0)
            {
                for (int i = 0; i < invocationList.Length; i++)
                {
                    invocationList[i].DynamicInvoke();
                }
            }
        }
        private void AsyncCall()
        {
            if (invocationList.Length > 0)
            {
                Parallel.For(0, invocationList.Length, ParallelOptions,
                    i => { invocationList[i].DynamicInvoke(); }
                );
            }
        }
    }
}
