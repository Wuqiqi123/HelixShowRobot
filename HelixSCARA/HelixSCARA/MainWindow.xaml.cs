using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Runtime.InteropServices;

namespace HelixSCARA
{
    class Joint
    {
        public Model3D model = null;
        public double angle = 0;
        public double angleMin = -180;
        public double angleMax = 180;
        public int rotPointX = 0;
        public int rotPointY = 0;
        public int rotPointZ = 0;
        public int rotAxisX = 0;
        public int rotAxisY = 0;
        public int rotAxisZ = 0;

        public Joint(Model3D pModel)
        {
            model = pModel;
        }
    }
    [Serializable] // 指示可序列化
    [StructLayout(LayoutKind.Sequential, Pack = 1)] // 按1字节对齐
    struct RobotData
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] JointsNow;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] JointsNext;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] JointsVelNow;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] JointsVelNext;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)] // 声明一个字符数组，大小为6*8
        public  double[] Origin6axisForce;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] JointsTorque;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] CartesianPositionNow;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] CartesianPositionNext;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public  double[] CartesianVelNow;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] // 声明一个字符数组，大小为4*8
        public double[] CartesianVelNext;
    };
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        Model3DGroup RA = new Model3DGroup(); //RoboticArm 3d group
        Model3D geom = null; //Debug sphere to check in which point the joint is rotatin
        ModelVisual3D visual;
        ModelVisual3D RoboticArm = new ModelVisual3D();
        GeometryModel3D oldSelectedModel = null;
        Color oldColor = Colors.White;

        Client client;    // 客户端实例
        public MainWindow()
        {
            InitializeComponent();



            var builder = new MeshBuilder(true, true);
            var position = new Point3D(0, 0, 0);
            builder.AddSphere(position, 50, 15, 15);
            geom = new GeometryModel3D(builder.ToMesh(), Materials.Brown);
            visual = new ModelVisual3D();
            visual.Content = geom;
            viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
            viewPort3d.Children.Add(visual);
            viewPort3d.Children.Add(RoboticArm);
            viewPort3d.Camera.LookDirection = new Vector3D(2038, -5200, -2930);
            viewPort3d.Camera.UpDirection = new Vector3D(-0.145, 0.372, 0.917);
            viewPort3d.Camera.Position = new Point3D(-1571, 4801, 3774);

            ConnetServer(); 
        }

        //连接服务器
        public void ConnetServer()
        {
            if (client == null) client = new Client(ClientPrint, "127.0.0.1", "8888");
            if (!client.connected) client.start();
           // if (client != null) thi = "客户端 " + client.localIpPort;
        }

        //调试的时候打印显示信息
        private void ClientPrint(string info)
        {
            System.Diagnostics.Debug.WriteLine(info);
            //if (textBox_showing.InvokeRequired)
            //{
            //    Client.Print F = new Client.Print(ClientPrint);
            //    this.Invoke(F, new object[] { info });
            //}
            //else
            //{
            //    if (info != null)
            //    {
            //        textBox_showing.SelectionColor = Color.Green;
            //        textBox_showing.AppendText(info);
            //        textBox_showing.AppendText(Environment.NewLine);
            //        textBox_showing.ScrollToCaret();
            //    }
            //}
        }

        //将sockct接受的字符转化成Robot对象
        T ByteArrayToStructure<T>(byte[] bytes) where T : struct    //where表示约束，只能为struct
        {
            T stuff;
            GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
            try
            {
                stuff = (T)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(T));
            }
            finally
            {
                handle.Free();
            }
            return stuff;
        }
        private void ViewPort3D_OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point mousePos = e.GetPosition(viewPort3d);
            PointHitTestParameters hitParams = new PointHitTestParameters(mousePos);
            VisualTreeHelper.HitTest(viewPort3d, null, ResultCallback, hitParams);
           // System.Diagnostics.Debug.WriteLine("测试打印信息");
        }

        private void ViewPort3D_OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            // Perform the hit test on the mouse's position relative to the viewport.
            HitTestResult result = VisualTreeHelper.HitTest(viewPort3d, e.GetPosition(viewPort3d));
            RayMeshGeometry3DHitTestResult mesh_result = result as RayMeshGeometry3DHitTestResult;

            if (oldSelectedModel != null)
                unselectModel();

            if (mesh_result != null)
            {
                selectModel(mesh_result.ModelHit);
            }
        }
        public HitTestResultBehavior ResultCallback(HitTestResult result)
        {
            // Did we hit 3D?
            RayHitTestResult rayResult = result as RayHitTestResult;
            if (rayResult != null)
            {
                // Did we hit a MeshGeometry3D?
                RayMeshGeometry3DHitTestResult rayMeshResult = rayResult as RayMeshGeometry3DHitTestResult;
                geom.Transform = new TranslateTransform3D(new Vector3D(rayResult.PointHit.X, rayResult.PointHit.Y, rayResult.PointHit.Z));

                if (rayMeshResult != null)
                {
                    // Yes we did!
                }
            }

            return HitTestResultBehavior.Continue;
        }
        private void unselectModel()
        {
            changeModelColor(oldSelectedModel, oldColor);
        }

        private Color changeModelColor(GeometryModel3D pModel, Color newColor)
        {
            if (pModel == null)
                return oldColor;

            Color previousColor = Colors.Black;

            MaterialGroup mg = (MaterialGroup)pModel.Material;
            if (mg.Children.Count > 0)
            {
                try
                {
                    previousColor = ((EmissiveMaterial)mg.Children[0]).Color;
                    ((EmissiveMaterial)mg.Children[0]).Color = newColor;
                    ((DiffuseMaterial)mg.Children[1]).Color = newColor;
                }
                catch (Exception exc)
                {
                    previousColor = oldColor;
                }
            }

            return previousColor;
        }

        private Color changeModelColor(Joint pJoint, Color newColor)
        {
            Model3DGroup models = ((Model3DGroup)pJoint.model);
            return changeModelColor(models.Children[0] as GeometryModel3D, newColor);
        }

        private void selectModel(Model3D pModel)
        {
            try
            {
                Model3DGroup models = ((Model3DGroup)pModel);
                oldSelectedModel = models.Children[0] as GeometryModel3D;
            }
            catch (Exception exc)
            {
                oldSelectedModel = (GeometryModel3D)pModel;
            }
            oldColor = changeModelColor(oldSelectedModel, ColorHelper.HexToColor("#ff3333"));
        }

    }
}
