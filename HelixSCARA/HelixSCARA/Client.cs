using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Threading;

namespace HelixSCARA
{
    class Client
    {
        public string ipString = "127.0.0.1";   // 服务器端ip
        public int port = 8888;                // 服务器端口
        public Socket socket;
        public Print print;                     // 运行时的信息输出方法,委托的函数名
        public bool connected = false;          // 标识当前是否连接到服务器
        public string localIpPort = "";         // 记录本地ip端口信息
        public delegate void Print(RobotData? info);// 运行时的信息输出方法,委托的函数名
        public Client(Print print = null, string ipString = null, int port = -1)
        {
            this.print = print;
            if (ipString != null) this.ipString = ipString;
            if (port >= 0) this.port = port;
        }

        public Client(Print print = null, string ipString = null, string port = "-1")
        {
            this.print = print;
            if (ipString != null) this.ipString = ipString;

            int port_int = Int32.Parse(port);
            if (port_int >= 0) this.port = port_int;
        }

        public string GetLocalIp()
        {
            ///获取本地的IP地址
            string AddressIP = string.Empty;
            foreach (IPAddress _IPAddress in Dns.GetHostEntry(Dns.GetHostName()).AddressList)
            {
                if (_IPAddress.AddressFamily.ToString() == "InterNetwork")
                {
                    AddressIP = _IPAddress.ToString();
                }
            }
            return AddressIP;
        }
        /// <summary>
        /// Print用于输出Server的输出信息
        /// </summary>

        /// <summary>
        /// 启动客户端，连接至服务器
        /// </summary>
        public void start()
        {
            if(ipString=="127.0.0.1")
            {
                ipString = GetLocalIp();
            }
            System.Diagnostics.Debug.WriteLine("ServeripPort:" + ipString);
            //设定服务器IP地址  
            IPAddress ip = IPAddress.Parse(ipString);
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            try
            {
                socket.Connect(new IPEndPoint(ip, port));   // 连接服务器
                //if (print != null) print("连接服务器【" + ipString + "】完成"); // 连接成功
                localIpPort = socket.LocalEndPoint.ToString();
                connected = true;
                System.Diagnostics.Debug.WriteLine("localipPort:"+localIpPort);
                Thread thread = new Thread(receiveData);
                thread.Start(socket);      // 在新的线程中接收服务器信息
            }
            catch (Exception ex)
            {
                //if (print != null) print("连接服务器失败 " + ex.ToString()); // 连接失败
                connected = false;
            }
        }

        /// <summary>
        /// 结束客户端
        /// </summary>
        public void stop()
        {
            connected = false;
        }

        /// <summary>
        /// 发送信息
        /// </summary>
        public void Send(string info)
        {
            try
            {
                Send(socket, info);
            }
            catch (Exception ex)
            {
                //if (print != null) print("服务器端已断开，【" + socket.RemoteEndPoint.ToString() + "】");
            }
        }

        /// <summary>
        /// 通过socket发送数据data
        /// </summary>
        private void Send(Socket socket, string data)
        {
            if (socket != null && data != null && !data.Equals(""))
            {
                byte[] bytes = Encoding.UTF8.GetBytes(data);   // 将data转化为byte数组
                socket.Send(bytes);                            // 
            }
        }

        /// <summary>
        /// 接收通过socket发送过来的数据
        /// </summary>
        private void receiveData(object socket)
        {
            Socket ortherSocket = (Socket)socket;

            while (true)
            {
                try
                {
                    RobotData? data = Receive(ortherSocket);       // 接收发送的信息
                    if (data!=null)
                    {
                        //if (print != null) print("服务器" + ortherSocket.RemoteEndPoint.ToString() + "信息：\r\n" + data);
                        if (print != null) print(data);
                        if (data.Equals("[.Shutdown]")) System.Environment.Exit(0);
                    }
                }
                catch (Exception ex)
                {
                    //if (print != null) print("连接已自动断开，" + ex.Message);
                    ortherSocket.Shutdown(SocketShutdown.Both);
                    ortherSocket.Close();
                    connected = false;
                    break;
                }
                if (!connected) break;
                Thread.Sleep(70);      // 延时70ms后处理接收到的信息
            }
        }
        /// <summary>
        /// 从socket接收数据
        /// </summary>
        private RobotData? Receive(Socket socket)
        {
            string data = "";

            byte[] bytes = null;
            int len = socket.Available;
            RobotData? robdata =null;   //支持空对象
            if (len > 0)
            {
                bytes = new byte[len];
                int receiveNumber = socket.Receive(bytes);
                robdata = MainWindow.ByteArrayToStructure<RobotData>(bytes);
                //data = Encoding.UTF8.GetString(bytes, 0, receiveNumber);
            }

            return robdata;
            //return data;
        }
    }
}
