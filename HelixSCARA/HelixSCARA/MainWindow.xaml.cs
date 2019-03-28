﻿using HelixToolkit.Wpf;
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
using System.IO;

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

        List<Joint> joints = null;

        ModelVisual3D visual;
        ModelVisual3D RoboticArm = new ModelVisual3D();
        GeometryModel3D oldSelectedModel = null;
        Color oldColor = Colors.White;
        string basePath = "";

        /// <summary>

        private const string MODEL_PATH1 = "SCARA_Robot - Base-1.STL";
        private const string MODEL_PATH2 = "SCARA_Robot - Link1-1.STL";
        private const string MODEL_PATH3 = "SCARA_Robot - Link2-1.STL";
        private const string MODEL_PATH4 = "SCARA_Robot - Move-1.STL";
        private const string MODEL_PATH5 = "SCARA_Robot - Rot-1.STL";
        /// </summary>
        Client client;    // 客户端实例
        public MainWindow()
        {
            InitializeComponent();
            basePath = Directory.GetParent(Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName) + "\\Models\\";
            List<string> modelsNames = new List<string>();
            modelsNames.Add(MODEL_PATH1);
            modelsNames.Add(MODEL_PATH2);
            modelsNames.Add(MODEL_PATH3);
            modelsNames.Add(MODEL_PATH4);
            modelsNames.Add(MODEL_PATH5);

            RoboticArm.Content = Initialize_Environment(modelsNames);

            var builder = new MeshBuilder(true, true);
            var position = new Point3D(0, 0, 0);
            builder.AddSphere(position, 20, 15, 15);
            geom = new GeometryModel3D(builder.ToMesh(), Materials.Brown);
            visual = new ModelVisual3D();
            visual.Content = geom;
            viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
            viewPort3d.Children.Add(visual);
            viewPort3d.Children.Add(RoboticArm);
            viewPort3d.Camera.LookDirection = new Vector3D(-1077, 1684, -877);
            viewPort3d.Camera.UpDirection = new Vector3D(0.248, -0.390, 0.887);
            viewPort3d.Camera.Position = new Point3D(1620, -1753, 1355);

            ConnetServer(); 
        }

       private Model3DGroup Initialize_Environment(List<string> modelsNames)
        {
            try
            {
                ModelImporter import = new ModelImporter();
                joints = new List<Joint>();

                foreach (string modelName in modelsNames)
                {
                    var materialGroup = new MaterialGroup();
                    Color mainColor = Colors.White;
                    EmissiveMaterial emissMat = new EmissiveMaterial(new SolidColorBrush(mainColor));
                    DiffuseMaterial diffMat = new DiffuseMaterial(new SolidColorBrush(mainColor));
                    SpecularMaterial specMat = new SpecularMaterial(new SolidColorBrush(mainColor), 200);
                    materialGroup.Children.Add(emissMat);
                    materialGroup.Children.Add(diffMat);
                    materialGroup.Children.Add(specMat);

                    var link = import.Load(basePath + modelName);
                    GeometryModel3D model = link.Children[0] as GeometryModel3D;
                    model.Material = materialGroup;
                    model.BackMaterial = materialGroup;
                    joints.Add(new Joint(link));
                }

                RA.Children.Add(joints[0].model);
                RA.Children.Add(joints[1].model);
                RA.Children.Add(joints[2].model);
                RA.Children.Add(joints[3].model);
                RA.Children.Add(joints[4].model);

                changeModelColor(joints[0], Colors.Red);
                changeModelColor(joints[1], Colors.Pink);
                changeModelColor(joints[2], Colors.Blue);
                changeModelColor(joints[3], Colors.Green);
                changeModelColor(joints[4], Colors.Yellow);


                joints[0].angleMin = -180;
                joints[0].angleMax = 180;
                joints[0].rotAxisX = 0;
                joints[0].rotAxisY = 0;
                joints[0].rotAxisZ = 1;
                joints[0].rotPointX = 0;
                joints[0].rotPointY = 0;
                joints[0].rotPointZ = 0;

                joints[1].angleMin = -100;
                joints[1].angleMax = 60;
                joints[1].rotAxisX = 0;
                joints[1].rotAxisY = 1;
                joints[1].rotAxisZ = 0;
                joints[1].rotPointX = 175;
                joints[1].rotPointY = -200;
                joints[1].rotPointZ = 500;

                joints[2].angleMin = -90;
                joints[2].angleMax = 90;
                joints[2].rotAxisX = 0;
                joints[2].rotAxisY = 1;
                joints[2].rotAxisZ = 0;
                joints[2].rotPointX = 190;
                joints[2].rotPointY = -700;
                joints[2].rotPointZ = 1595;

                joints[3].angleMin = -180;
                joints[3].angleMax = 180;
                joints[3].rotAxisX = 1;
                joints[3].rotAxisY = 0;
                joints[3].rotAxisZ = 0;
                joints[3].rotPointX = 400;
                joints[3].rotPointY = 0;
                joints[3].rotPointZ = 1765;

            }
            catch (Exception e)
            {
                MessageBox.Show("Exception Error:" + e.StackTrace);
            }
            return RA;
        }
        //连接服务器
        public void ConnetServer()
        {
            if (client == null) client = new Client(ClientPrint, "127.0.0.1", "8888");
            if (!client.connected) client.start();
           // if (client != null) thi = "客户端 " + client.localIpPort;
        }

        //调试的时候打印显示信息

        private void ClientPrint(RobotData? info)
        {
            RobotData robot = info.Value;
            double a = robot.JointsNow[0];
            double b = robot.CartesianPositionNow[1];
            double c = robot.JointsNow[2];
            double d = robot.JointsNow[3];
            double e = robot.JointsTorque[0];

            System.Diagnostics.Debug.WriteLine(c);
            System.Diagnostics.Debug.WriteLine('\n');
            System.Diagnostics.Debug.WriteLine(e);
            System.Diagnostics.Debug.WriteLine('\n');
            System.Diagnostics.Debug.WriteLine(b);
            System.Diagnostics.Debug.WriteLine('\n');
        }

        //将sockct接受的字符转化成Robot对象
        public static T ByteArrayToStructure<T>(byte[] bytes) where T : struct    //where表示约束，只能为struct
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
