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
using System.IO;
using System.Threading;
using System.Runtime.Serialization.Formatters.Binary;
using System.Runtime.Serialization;

namespace HelixSCARA
{
    public delegate void Entrust();
    public class Joint
    {
        public Model3D model = null;
        public double angle = 0;
        public double angleMin = -180;
        public double angleMax = 180;
        public double rotPointX = 0;
        public double rotPointY = 0;
        public double rotPointZ = 0;
        public int rotAxisX = 0;
        public int rotAxisY = 0;
        public int rotAxisZ = 0;

        public Joint(Model3D pModel)
        {
            model = pModel;
        }
    }

    public class ForceData
    {
        public double FX = 0;
        public double FY = 0;
        public double FZ = 0;
        public double MX = 0;
        public double MY = 0;
        public double MZ = 0;
        public ForceData(double Fx, double Fy, double Fz, double Mx, double My, double Mz)
        {
            FX = Fx;
            FY = Fy;
            FZ = Fz;
            MX = Mx;
            MY = My;
            MZ = Mz;
        }
    }

    [Serializable] // 指示可序列化
    [StructLayout(LayoutKind.Sequential, Pack = 1)] // 按1字节对齐
    public struct RobotData
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
        Model3DGroup FS = new Model3DGroup(); //力传感器系统
        Model3D geom = null; //Debug sphere to check in which point the joint is rotatin
        Model3D AxisX = null;
        Model3D AxisY = null;
        Model3D AxisZ = null;
        Model3D EndOrigin = null;
        Point3D OriPosition;
        Model3D ForceModel = null;
        Model3D TorqueModel = null;
        ForceData FD = null;

        List<Joint> joints = null;

        ModelVisual3D ForceSystem =new ModelVisual3D();
        ModelVisual3D RoboticArm = new ModelVisual3D();
        GeometryModel3D oldSelectedModel = null;
        Color oldColor = Colors.White;
        string basePath = "";

        Transform3DGroup F1;
        Transform3DGroup F2;
        Transform3DGroup F3;
        Transform3DGroup F4;
        Transform3DGroup F5;
        Transform3DGroup F6;
        RotateTransform3D R;
        TranslateTransform3D T;
        TranslateTransform3D Tran_2;

        //////////////
        System.Windows.Threading.DispatcherTimer dtimer;
        bool IsFirstFlag=true;
        /// <summary>

        private const string MODEL_PATH1 = "SCARA_Robot - Link1-1.STL";
        private const string MODEL_PATH2 = "SCARA_Robot - Link2-1.STL";
        private const string MODEL_PATH3 = "SCARA_Robot - Move-1.STL";
        private const string MODEL_PATH4 = "SCARA_Robot - Rot-1.STL";
        private const string MODEL_PATH5 = "SCARA_Robot - Base-1.STL";

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

            ForceSystem.Content = Initialize_ForceSystem();

            viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
            viewPort3d.Children.Add(ForceSystem);
            viewPort3d.Children.Add(RoboticArm);
            viewPort3d.Camera.LookDirection = new Vector3D(-862.674, 1144.325, -664.861);
            viewPort3d.Camera.UpDirection = new Vector3D(0.267, -0.355, 0.896);
            viewPort3d.Camera.Position = new Point3D(1147.73, -1353.113, 1086.891);
           

            ///////////////////////////////////

            ConnetServer();

            ////////////
            ///                dtimer = new System.Windows.Threading.DispatcherTimer();
            if (dtimer == null)
            {
                dtimer = new System.Windows.Threading.DispatcherTimer();
                dtimer.Interval = TimeSpan.FromSeconds(0.018);
                dtimer.Tick += dtimer_Tick;
                dtimer.Start();
            }

        }

        void dtimer_Tick(object sender, EventArgs e)
        {
            execute_fk();
        }


        public static object CloneObject(object obj)
        {
            using (MemoryStream memStream = new MemoryStream())
            {
                BinaryFormatter binaryFormatter = new BinaryFormatter(null, new StreamingContext(StreamingContextStates.Clone));
                binaryFormatter.Serialize(memStream, obj);
                memStream.Seek(0, SeekOrigin.Begin);
                return binaryFormatter.Deserialize(memStream);
            }
        }
        private Model3DGroup Initialize_ForceSystem()
        {
            List<MeshBuilder> builder = new List<MeshBuilder>();
             OriPosition = new Point3D(400, 0, 223.5);
            List<Point3D> ForceCoordinateSystem = new List<Point3D>();
            FD = new ForceData(OriPosition.X , OriPosition.Y , OriPosition.Z , OriPosition.X , OriPosition.Y , OriPosition.Z );

            var FAxisX = new Point3D(OriPosition.X + 0, OriPosition.Y + 100, OriPosition.Z + 0);   //力传感器坐标系的轴坐标系方向与机器人坐标系定义不同
            var FAxisY = new Point3D(OriPosition.X +100, OriPosition.Y + 0, OriPosition.Z + 0);
            var FAxisZ = new Point3D(OriPosition.X + 0, OriPosition.Y + 0, OriPosition.Z - 100);
            ForceCoordinateSystem.Add(FAxisX);
            ForceCoordinateSystem.Add(FAxisY);
            ForceCoordinateSystem.Add(FAxisZ);

            builder.Add(new MeshBuilder(true,true));
            builder[0].AddSphere(OriPosition, 0.001);
            EndOrigin = new GeometryModel3D(builder[0].ToMesh(), Materials.Brown);
            FS.Children.Add(EndOrigin);

            builder.Add(new MeshBuilder(true, true));
            builder[1].AddArrow(OriPosition, ForceCoordinateSystem[0], 8);
            AxisX = new GeometryModel3D(builder[1].ToMesh(), Materials.Red);
            FS.Children.Add(AxisX);

            builder.Add(new MeshBuilder(true, true));
            builder[2].AddArrow(OriPosition, ForceCoordinateSystem[1], 8);
            AxisY = new GeometryModel3D(builder[2].ToMesh(), Materials.Green);
            FS.Children.Add(AxisY);

            builder.Add(new MeshBuilder(true, true));
            builder[3].AddArrow(OriPosition, ForceCoordinateSystem[2], 8);
            AxisZ = new GeometryModel3D(builder[3].ToMesh(), Materials.Blue);
            FS.Children.Add(AxisZ);

            builder.Add(new MeshBuilder(true, true));
            builder[4].AddArrow(OriPosition, new Point3D(FD.FX, FD.FY, FD.FZ), 3);
            ForceModel = new GeometryModel3D(builder[4].ToMesh(), Materials.Gold);
            FS.Children.Add(ForceModel);

            builder.Add(new MeshBuilder(true, true));
            builder[5].AddArrow(OriPosition, new Point3D(FD.MX, FD.MY, FD.MZ), 3);
            TorqueModel = new GeometryModel3D(builder[5].ToMesh(), Materials.Indigo);
            FS.Children.Add(TorqueModel);

            return FS;
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


                joints[0].angleMin = -110;
                joints[0].angleMax = 110;
                joints[0].rotAxisX = 0;
                joints[0].rotAxisY = 0;
                joints[0].rotAxisZ = 1;
                joints[0].rotPointX = 0;
                joints[0].rotPointY = 0;
                joints[0].rotPointZ = 100;   //no use

                joints[1].angleMin = -120;
                joints[1].angleMax = 120;
                joints[1].rotAxisX = 0;
                joints[1].rotAxisY = 0;
                joints[1].rotAxisZ = 1;
                joints[1].rotPointX = 250;
                joints[1].rotPointY = 0;
                joints[1].rotPointZ = 100;   //no use

                joints[2].angleMin = -110;    //linear movement
                joints[2].angleMax = 110;
                joints[2].rotAxisX = 0;
                joints[2].rotAxisY = 0;
                joints[2].rotAxisZ = 1;
                joints[2].rotPointX = 400;
                joints[2].rotPointY = 0;
                joints[2].rotPointZ = 100;

                joints[3].angleMin = -360;
                joints[3].angleMax = 360;
                joints[3].rotAxisX = 0;
                joints[3].rotAxisY = 0;
                joints[3].rotAxisZ = 1;
                joints[3].rotPointX = 400;
                joints[3].rotPointY = 0;
                joints[3].rotPointZ = 100;    //no use

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
            if (client == null)
            {
                Entrust callback = new Entrust(CallBack); //把方法赋值给委托
                client = new Client(callback,"127.0.0.1", "8888");
                client.print += new myPrint(ClientPrint);
               
            }
            if (!client.connected) client.start();
           // if (client != null) thi = "客户端 " + client.localIpPort;
        }

        //调试的时候打印显示信息

        public void ClientPrint(RobotData? info)
        {
            RobotData robot = info.Value;
             //= CloneObject(object robot);
     
            joints[0].angle = robot.JointsNow[0];
            joints[1].angle = robot.JointsNow[1];
            joints[2].angle = robot.JointsNow[2];
            joints[3].angle = robot.JointsNow[3];
            FD.FX = robot.Origin6axisForce[0];
            FD.FY = robot.Origin6axisForce[1];
            FD.FZ = robot.Origin6axisForce[2];
            FD.MX = robot.Origin6axisForce[3];
            FD.MY = robot.Origin6axisForce[4];
            FD.MZ = robot.Origin6axisForce[5];
            IsFirstFlag = false;
            //double a = robot.JointsNow[0];
            ////double b = robot.CartesianPositionNow[1]
            //System.Diagnostics.Debug.WriteLine(a);
            //execute_fk();
        }

        public void CallBack()
        {
            //不能使用没有用
            execute_fk();
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

        public Vector3D ForwardKinematics(double[] angles,double[] tempF )
        {

            F1 = new Transform3DGroup();
            T = new TranslateTransform3D();
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[0].rotAxisX, joints[0].rotAxisY, joints[0].rotAxisZ), angles[0]), new Point3D(joints[0].rotPointX, joints[0].rotPointY, joints[0].rotPointZ));
            F1.Children.Add(R);
            F1.Children.Add(T);

            //This moves the first joint attached to the base, it may translate and rotate. Since the joint are already in the right position (the .stl model also store the joints position
            //in the virtual world when they were first created, so if you load all the .stl models of the joint they will be automatically positioned in the right locations)
            //so in all of these cases the first translation is always 0, I just left it for future purposes if something need to be moved
            //After that, the joint needs to rotate of a certain amount (given by the value in the slider), and the rotation must be executed on a specific point
            //After some testing it looks like the point 175, -200, 500 is the sweet spot to achieve the rotation intended for the joint
            //finally we also need to apply the transformation applied to the base 
            F2 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[1].rotAxisX, joints[1].rotAxisY, joints[1].rotAxisZ), angles[1]), new Point3D(joints[1].rotPointX, joints[1].rotPointY, joints[1].rotPointZ));
            F2.Children.Add(T);
            F2.Children.Add(R);
            F2.Children.Add(F1);

            //The second joint is attached to the first one. As before I found the sweet spot after testing, and looks like is rotating just fine. No pre-translation as before
            //and again the previous transformation needs to be applied
            F3 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            Tran_2 = new TranslateTransform3D(joints[2].rotAxisX * joints[2].angle, joints[2].rotAxisY * joints[2].angle,  joints[2].rotAxisZ * joints[2].angle);
            F3.Children.Add(T);
            F3.Children.Add(Tran_2);
            F3.Children.Add(F2);

            //as before
            F4 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0); //1500, 650, 1650
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[3].rotAxisX, joints[3].rotAxisY, joints[3].rotAxisZ), angles[3]), new Point3D(joints[3].rotPointX, joints[3].rotPointY, joints[3].rotPointZ));
            F4.Children.Add(T);
            F4.Children.Add(R);
            F4.Children.Add(F3);

            
            //NB: I was having a nightmare trying to understand why it was always rotating in a weird way... SO I realized that the order in which
            //you add the Children is actually VERY IMPORTANT in fact before I was applyting F and then T and R, but the previous transformation
            //Should always be applied as last (FORWARD Kinematics)   
            //////重新画力传感器坐标
            Point3D EndPosition = new Point3D(EndOrigin.Bounds.Location.X, EndOrigin.Bounds.Location.Y, EndOrigin.Bounds.Location.Z);
            ///////////////////////////////////////////////////////////
            FS.Children.Remove(ForceModel);
            MeshBuilder meshBuilderForce = new MeshBuilder(true, true);
            Point3D F = new Point3D(tempF[1]+ OriPosition.X, tempF[0]+ OriPosition.Y, -tempF[2]+ OriPosition.Z);
            meshBuilderForce.AddArrow(OriPosition, F, 5);
            ForceModel = new GeometryModel3D(meshBuilderForce.ToMesh(), Materials.Gold);
            FS.Children.Add(ForceModel);


            FS.Children.Remove(TorqueModel);
            MeshBuilder meshBuilderTorque = new MeshBuilder(true, true);
            Point3D M = new Point3D(tempF[4] + OriPosition.X, tempF[3] + OriPosition.Y, -tempF[5] + OriPosition.Z);
            meshBuilderTorque.AddArrow(OriPosition, M, 5);
            TorqueModel = new GeometryModel3D(meshBuilderTorque.ToMesh(), Materials.Indigo);
            FS.Children.Add(TorqueModel);

            ////////////////////////////////////////////////////////
            joints[0].model.Transform = F1; //First joint
            joints[1].model.Transform = F2; //Second joint (the "biceps")
            joints[2].model.Transform = F3; //movemet joint 
            joints[3].model.Transform = F4; //the rot
            EndOrigin.Transform = F4;
            AxisX.Transform = F4;
            AxisY.Transform = F4;
            AxisZ.Transform = F4;
            ForceModel.Transform = F4;
            TorqueModel.Transform = F4;

            if(IsFirstFlag)  //发现第一此运行的时候
            {
                Tx.Content = 400;
                Ty.Content = 0;
                Tz.Content = 223.5;
            }
            else
            {
                Tx.Content = EndPosition.X;
                Ty.Content = EndPosition.Y;
                Tz.Content = EndPosition.Z;
            }
            return new Vector3D(EndPosition.X,EndPosition.Y,EndPosition.Z);
        }

        private void joint_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {

            joints[0].angle = joint1.Value;
            joints[1].angle = joint2.Value;
            joints[2].angle = joint3.Value;
            joints[3].angle = joint4.Value;
            IsFirstFlag = false;
            execute_fk();
        }
        /**
 * This methodes execute the FK (Forward Kinematics). It starts from the first joint, the base.
 * */
        private void execute_fk()
        {
            /** Debug sphere, it takes the x,y,z of the textBoxes and update its position
             * This is useful when using x,y,z in the "new Point3D(x,y,z)* when defining a new RotateTransform3D() to check where the joints is actually  rotating */
            double[] angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle};
            double[] tempF = { FD.FX, FD.FY , FD.FZ , FD.MX ,FD.MY ,FD.MZ };
            ForwardKinematics(angles, tempF);
            //updateSpherePosition();
        }

    }
}
