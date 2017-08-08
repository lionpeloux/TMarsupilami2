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
    public class ParallelEventHandler<TSender, TEventArgs> where TEventArgs : EventArgs
    {
        private event EventHandler<TSender, TEventArgs> handler;
        private Delegate[] invocationList;
        private bool isParallelModeEnabled;

        public bool IsParallelModeEnabled { get; set; }
        public ParallelOptions ParallelOptions { get; set; }

        public ParallelEventHandler(bool isParallelModeEnabled = false)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = new ParallelOptions();
        }
        public ParallelEventHandler(ParallelOptions options, bool isParallelModeEnabled = true)
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

        public void Raise(TSender sender, TEventArgs e, bool isParallelModeEnabled = true)
        {
            if (isParallelModeEnabled)
            {
                RaiseASync(sender, e);
            }
            else
            {
                RaiseSync(sender, e);
            }
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
        private void RaiseASync(TSender sender, TEventArgs e)
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

    public class ParallelActionHandler
    {
        private event Action handler;
        private Delegate[] invocationList;
        private bool isParallelModeEnabled;

        public bool IsParallelModeEnabled { get; set; }
        public ParallelOptions ParallelOptions { get; set; }

        public ParallelActionHandler(bool isParallelModeEnabled = false)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = new ParallelOptions();
        }
        public ParallelActionHandler(ParallelOptions options, bool isParallelModeEnabled = true)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = options;
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

        public void Raise(bool isParallelModeEnabled = true)
        {
            if (isParallelModeEnabled)
            {
                RaiseASync();
            }
            else
            {
                RaiseSync();
            }
        }
        private void RaiseSync()
        {
            if (invocationList == null)
                return;
            if (invocationList.Length > 0)
            {
                for (int i = 0; i < invocationList.Length; i++)
                {
                    invocationList[i].DynamicInvoke(null);
                }
            }
        }
        private void RaiseASync()
        {
            if (invocationList == null)
                return;
            if (invocationList.Length > 0)
            {
                Parallel.For(0, invocationList.Length, ParallelOptions,
                    i => { invocationList[i].DynamicInvoke(null); }
                );
            }
        }
    }
    public class ParallelActionHandler<T>
    {
        private event Action<T> handler;
        private Delegate[] invocationList;
        private bool isParallelModeEnabled;

        public bool IsParallelModeEnabled { get; set; }
        public ParallelOptions ParallelOptions { get; set; }

        public ParallelActionHandler(bool isParallelModeEnabled = false)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = new ParallelOptions();
        }
        public ParallelActionHandler(ParallelOptions options, bool isParallelModeEnabled = true)
        {
            IsParallelModeEnabled = isParallelModeEnabled;
            ParallelOptions = options;
        }

        public void Subscribe(Action<T> handler)
        {
            this.handler += handler;
            this.invocationList = this.handler.GetInvocationList();
        }
        public void Subscribe(IEnumerable<Action<T>> handlers)
        {
            foreach (var handler in handlers)
            {
                this.handler += handler;
            }
            this.invocationList = this.handler.GetInvocationList();
        }

        public void UnSubscribe(Action<T> handler)
        {
            this.handler -= handler;
            this.invocationList = this.handler.GetInvocationList();
        }
        public void UnSubscribe(IEnumerable<Action<T>> handlers)
        {
            foreach (var handler in handlers)
            {
                this.handler -= handler;
            }
            this.invocationList = this.handler.GetInvocationList();
        }

        public void Raise(T param, bool isParallelModeEnabled = true)
        {
            if (isParallelModeEnabled)
            {
                RaiseASync(param);
            }
            else
            {
                RaiseSync(param);
            }
        }
        private void RaiseSync(T param)
        {
            if (invocationList == null)
                return;
            if (invocationList.Length > 0)
            {
                for (int i = 0; i < invocationList.Length; i++)
                {
                    invocationList[i].DynamicInvoke(param);
                }
            }
        }
        private void RaiseASync(T param)
        {
            if (invocationList == null)
                return;
            if (invocationList.Length > 0)
            {
                Parallel.For(0, invocationList.Length, ParallelOptions,
                    i => { invocationList[i].DynamicInvoke(param); }
                );
            }
        }
    }
}   
    