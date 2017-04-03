using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.Event
{
    /// <summary>
    /// Generic delegate signature with strongly typed sender.
    /// </summary>
    /// <typeparam name="TSender">The type of the sender object to be passed to the handlers when the event is triggered.</typeparam>
    /// <typeparam name="TEventArgs">The type of the EventArgs object to be passed to the handlers when the event is triggered.</typeparam>
    /// <param name="sender">The sender object.</param>
    /// <param name="e">The event arguments object.</param>
    public delegate void EventHandler<TSender, TEventArgs>(TSender sender, TEventArgs e);

    /// <summary>
    /// Extended event object that acts like a standard .Net event where the sender object is strongly typed.
    /// EventHandlers are executed when the event is triggered by Raise().
    /// The call is either sync or async wether IsParallelModeEnabled is set to false or true. 
    /// </summary>
    /// <typeparam name="TSender">The type of the sender object.</typeparam>
    /// <typeparam name="TEventArgs">>The type of the EventArgs object.</typeparam>
    public class Event<TSender, TEventArgs> where TEventArgs : EventArgs
    {
        private event EventHandler<TSender, TEventArgs> handler;
        private Delegate[] invocationList;
        private Action<TSender, TEventArgs> raise;
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
                    raise = RaiseAsync;
                else
                    raise = RaiseSync;
            }
        }
        public ParallelOptions ParallelOptions { get; set; }

        public Event(bool isParallelModeEnabled = false)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = new ParallelOptions();
        }
        public Event(ParallelOptions options, bool isParallelModeEnabled = true)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = options;
        }


        public void Subscribe(EventHandler<TSender, TEventArgs> handler)
        {
            this.handler += handler;
            this.invocationList = this.handler.GetInvocationList();
        }
        public void Subscribe(IEnumerable<EventHandler<TSender, TEventArgs>> handlers)
        {
            foreach (var handler in handlers)
            {
                this.handler += handler;
            }
            this.invocationList = this.handler.GetInvocationList();
        }

        public void UnSubscribe(EventHandler<TSender, TEventArgs> handler)
        {
            this.handler -= handler;
            this.invocationList = this.handler.GetInvocationList();
        }
        public void UnSubscribe(IEnumerable<EventHandler<TSender, TEventArgs>> handlers)
        {
            foreach (var handler in handlers)
            {
                this.handler -= handler;
            }
            this.invocationList = this.handler.GetInvocationList();
        }

        public void Raise(TSender sender, TEventArgs e)
        {
            raise(sender, e);
        }
        private void RaiseSync(TSender sender, TEventArgs e)
        {
            if (invocationList.Length > 0)
            {
                object[] args = new object[2] { sender, e };
                for (int i = 0; i < invocationList.Length; i++)
                {
                    invocationList[i].DynamicInvoke(args);
                }
            }
        }
        private void RaiseAsync(TSender sender, TEventArgs e)
        {
            if (invocationList.Length > 0)
            {
                object[] args = new object[2] { sender, e };
                Parallel.For(0, invocationList.Length, ParallelOptions,
                    i => { invocationList[i].DynamicInvoke(args); }
                );
            }
        }
    }
}
