using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;

public class FeetProjection : MonoBehaviour
{
    public Actor actor;
    public float size = 1f;
    public int resolution = 5;
    public float Width = 0.26f;
	public float Length = 0.1f;
	public int WidthResolution = 8;
	public int LengthResolution = 5;
    //private LayerMask GroundMask = LayerMask.GetMask("Ground");
    public bool ShowProjection = true;
    private HeightMap RightFootProjection;//new HeightMap(size, resolution, GroundMask, bool false);
    private HeightMap LeftFootProjection;

    private Vector3 RightFootCenter = new Vector3(0f, 0f, 0f);
    private Vector3 LeftFootCenter = new Vector3(0f, 0f, 0f);

    StreamWriter LeftFootData;
    StreamWriter RightFootData;

    private int CounterLeftPoints = 0;
    private int CounterLeftCorrectPoints = 0;
    private int CounterRightPoints = 0;
    private int CounterRightCorrectPoints = 0;
    private int CounterLeftAreaPoints = 0;
    private int CounterRightAreaPoints = 0;

    // Start is called before the first frame update
    void Start()
    {
        //RightFootProjection = new HeightMap(size, resolution, LayerMask.GetMask("Ground"), false);
        //LeftFootProjection = new HeightMap(size, resolution, LayerMask.GetMask("Ground"), false);
        RightFootProjection = new HeightMap(Width, Length, WidthResolution, LengthResolution, LayerMask.GetMask("Ground"));
        LeftFootProjection = new HeightMap(Width, Length, WidthResolution, LengthResolution, LayerMask.GetMask("Ground"));

        LeftFootData = CreateFile("LeftFootData");
        RightFootData = CreateFile("RightFootData");
    }

    // Update is called once per frame
    void Update()
    {

        Matrix4x4 LeftKnee = actor.GetBoneTransformation("LeftKnee");
        Matrix4x4 LeftAnkle = actor.GetBoneTransformation("LeftAnkle");
        Matrix4x4 LeftToe = actor.GetBoneTransformation("LeftToe");
        Matrix4x4 RightKnee = actor.GetBoneTransformation("RightKnee");
        Matrix4x4 RightAnkle = actor.GetBoneTransformation("RightAnkle");
        Matrix4x4 RightToe = actor.GetBoneTransformation("RightToe");

        //Vector3 LeftAnkleRight = LeftAnkle.GetRight();
        Vector3 LeftAnkleToe = LeftToe.GetPosition() - LeftAnkle.GetPosition();
        Vector3 LeftAnkleToeUnitNormal = Vector3.Cross(LeftAnkleToe, LeftAnkle.GetRight()).normalized;
        //DrawPlane(LeftAnkle.GetPosition(), LeftAnkleToeUnitNormal);
        Plane LeftAnkleToePlane = new Plane(LeftAnkleToeUnitNormal, LeftAnkle.GetPosition());
        //float distanceToPlane = LeftAnkleToePlane.GetDistanceToPoint()

        //Rotation of pivot = rotation of Toe
        Matrix4x4 LeftPivot = LeftToe;
        Matrix4x4 RightPivot = RightToe;

        //Used to translate projection a little behind the ankle
        Vector3 LeftAnkleOffset = -0.015f * LeftAnkle.GetForward();
        Vector3 RightAnkleOffset = -0.015f * RightAnkle.GetForward();

        //Pivot XZ position = Ankle XZ position + Offset ; Pivot Y (height) = Knee Y
        LeftPivot[0,3] = LeftAnkle[0,3] + LeftAnkleOffset[0];
        LeftPivot[1,3] = LeftKnee[1,3];
        LeftPivot[2,3] = LeftAnkle[2,3] + LeftAnkleOffset[2];

        RightPivot[0,3] = RightAnkle[0,3] + RightAnkleOffset[0];
        RightPivot[1,3] = RightKnee[1,3];
        RightPivot[2,3] = RightAnkle[2,3] + RightAnkleOffset[2];

        /*LeftFootCenter = new Vector3(LeftToe[0,3], LeftToe[1,3], LeftToe[2,3]) - 
        new Vector3(LeftAnkle[0,3], LeftAnkle[1,3], LeftAnkle[2,3]);//LeftToe[0,3] - LeftAnkle[0,3];
        RightFootCenter = new Vector3(RightToe[0,3], RightToe[1,3], RightToe[2,3]) - 
        new Vector3(RightAnkle[0,3], RightAnkle[1,3], RightAnkle[2,3]);

        LeftPivot[0,3] = LeftFootCenter[0] + LeftAnkle[0,3];
        LeftPivot[1,3] = LeftFootCenter[1] + LeftAnkle[1,3];
        LeftPivot[2,3] = LeftKnee[2,3];

        RightPivot[0,3] = RightFootCenter[0] + RightAnkle[0,3];
        RightPivot[1,3] = RightFootCenter[1] + RightAnkle[1,3];
        RightPivot[2,3] = RightKnee[2,3];*/

        /*for(int i=0; i<LeftFootProjection.Points.Length; i++) {
            LeftFootProjection.Sense(LeftKnee);
            RightFootProjection.Sense(RightKnee);
        }*/
        LeftFootProjection.Sense(LeftPivot);
        RightFootProjection.Sense(RightPivot);

        //LeftFootProjection.DrawFeet(UltiDraw.Blue.Transparent(1f));
        //RightFootProjection.DrawFeet(UltiDraw.Blue.Transparent(1f));

    }

    void OnRenderObject() {
        if(Application.isPlaying) {
            Vector3[] LeftPointsMap = LeftFootProjection.Map;
            Vector3[] RightPointsMap = RightFootProjection.Map;

            Vector3[] LeftPoints = LeftFootProjection.Points;
            Vector3[] RightPoints = RightFootProjection.Points;

            Matrix4x4 LeftKnee = actor.GetBoneTransformation("LeftKnee");
            Matrix4x4 LeftAnkle = actor.GetBoneTransformation("LeftAnkle");
            Matrix4x4 LeftToe = actor.GetBoneTransformation("LeftToe");
            Matrix4x4 RightKnee = actor.GetBoneTransformation("RightKnee");
            Matrix4x4 RightAnkle = actor.GetBoneTransformation("RightAnkle");
            Matrix4x4 RightToe = actor.GetBoneTransformation("RightToe");

            Vector3 LeftAnkleVelocity = actor.GetBoneVelocity("LeftAnkle");
            Vector3 RightAnkleVelocity = actor.GetBoneVelocity("RightAnkle");

            /*****************************************************************************************************
            Feet Projection
            ******************************************************************************************************/

            //Rotation of pivot = rotation of Toe
            Matrix4x4 LeftPivot = LeftToe;
            Matrix4x4 RightPivot = RightToe;

            //Used to translate projection a little behind the ankle
            Vector3 LeftAnkleOffsetted = -0.015f * LeftAnkle.GetForward();
            Vector3 RightAnkleOffsetted = -0.015f * RightAnkle.GetForward();

            //Pivot XZ position = Ankle XZ position + Offset ; Pivot Y (height) = Knee Y
            LeftPivot[0,3] = LeftAnkle[0,3] + LeftAnkleOffsetted.x;
            LeftPivot[1,3] = LeftKnee[1,3];
            LeftPivot[2,3] = LeftAnkle[2,3] + LeftAnkleOffsetted.z;

            RightPivot[0,3] = RightAnkle[0,3] + RightAnkleOffsetted.x;
            RightPivot[1,3] = RightKnee[1,3];
            RightPivot[2,3] = RightAnkle[2,3] + RightAnkleOffsetted.z;

            Vector3 LeftAnkleAtToeHeight = LeftAnkle.GetPosition();
            LeftAnkleAtToeHeight[1] = LeftToe.GetPosition().y;

            Vector3 RightAnkleAtToeHeight = RightAnkle.GetPosition();
            RightAnkleAtToeHeight[1] = RightToe.GetPosition().y;

            //Creating Plane with Normal (of Ankle right and Ankle to Toe vectors) and point (Ankle position)
            float AnkleOffset = 0.03f;
            Vector3 LeftAnkleOffsettedDown = LeftAnkle.GetPosition() + -AnkleOffset*LeftAnkle.GetUp();

            Vector3 LeftAnkleToe = LeftToe.GetPosition() - LeftAnkleOffsettedDown;//.GetPosition();
            Vector3 LeftAnkleToeUnitNormal = Vector3.Cross(LeftAnkleToe, LeftToe.GetRight()).normalized;
            //DrawPlane(LeftAnkleOffsettedDown, LeftAnkleToeUnitNormal);
            Plane LeftAnkleToePlane = new Plane(LeftAnkleToeUnitNormal, LeftAnkleOffsettedDown);

            Vector3 RightAnkleOffsettedDown = RightAnkle.GetPosition() + -AnkleOffset*RightAnkle.GetUp();
            
            // Using Ankle to toe
            /*Vector3 RightAnkleToe = RightToe.GetPosition() - RightAnkle.GetPosition();
            Vector3 RightAnkleToeUnitNormal = Vector3.Cross(RightAnkleToe, RightAnkle.GetRight()).normalized;
            //DrawPlane(RightAnkle.GetPosition(), RightAnkleToeUnitNormal);
            Plane RightAnkleToePlane = new Plane(RightAnkleToeUnitNormal, RightAnkle.GetPosition());*/

            Vector3 RightAnkleToe = RightToe.GetPosition() - RightAnkleOffsettedDown;
            Vector3 RightAnkleToeUnitNormal = Vector3.Cross(RightAnkleToe, RightToe.GetRight()).normalized;
            //DrawPlane(RightAnkleOffsettedDown, RightAnkleToeUnitNormal);
            Plane RightAnkleToePlane = new Plane(RightAnkleToeUnitNormal, RightAnkleOffsettedDown);

            //DrawPlane(LeftAnkle.GetPosition(), LeftAnkleToeUnitNormal);
            //Debug.DrawRay(LeftPointsMap[0].GetRelativePositionFrom(LeftPivot), Vector3.up, Color.blue);
            //float distanceToPlane = LeftAnkleToePlane.GetDistanceToPoint(LeftPointsMap[0].GetRelativePositionFrom(LeftPivot));
            //Debug.Log("Distance to plane = " + distanceToPlane);

            /*****************************************************************************************************
            Feet Center Point
            ******************************************************************************************************/
            //float AnkleOffset = 0.06f;
            //Vector3 LeftAnkleOffsettedDown = LeftAnkle.GetPosition() + AnkleOffset*LeftAnkle.GetUp();
            Vector3 LeftAnkleOffsetedDownToToe = LeftToe.GetPosition() - LeftAnkleOffsettedDown;
            float LeftDistance = Vector3.Distance(LeftToe.GetPosition(),LeftAnkleOffsettedDown);
            Vector3 LeftFootCenter = LeftAnkle.GetPosition() + (LeftDistance/2f)*LeftAnkleOffsetedDownToToe.normalized;
            //Debug.DrawRay(LeftFootCenter, Vector3.up, Color.blue);
            Vector3 LeftFootCenterToeNorm = Vector3.Cross(LeftToe.GetRight(), LeftAnkleOffsetedDownToToe);
            
            //DrawPlane(LeftFootCenter, LeftFootCenterToeNorm, Color.blue);
            
            Plane LeftFootCenterPlane = new Plane(LeftFootCenterToeNorm, LeftFootCenter);
            RaycastHit hit;
		    Physics.Raycast(new Vector3(LeftFootCenter.x, 100f, LeftFootCenter.z), Vector3.down, out hit, float.PositiveInfinity, LayerMask.GetMask("Ground"));
            float LeftCenterFootHeight = LeftFootCenterPlane.GetDistanceToPoint(hit.point); //Vector3.Distance(LeftFootCenter, hit.point);
            float LeftCenterFootGlobalHeight = hit.point.y;

            //Vector3 RightAnkleOffsettedDown = RightAnkle.GetPosition() + AnkleOffset*RightAnkle.GetUp();
            Vector3 RightAnkleOffsetedDownToToe = RightToe.GetPosition() - RightAnkleOffsettedDown;
            float RightDistance = Vector3.Distance(RightToe.GetPosition(),RightAnkleOffsettedDown);
            Vector3 RightFootCenter = RightAnkle.GetPosition() + (RightDistance/2f)*RightAnkleOffsetedDownToToe.normalized;
            //Debug.DrawRay(RightFootCenter, Vector3.up, Color.blue);
            Vector3 RightFootCenterToeNorm = Vector3.Cross(RightToe.GetRight(), RightAnkleOffsetedDownToToe);
            
            //DrawPlane(RightFootCenter, RightFootCenterToeNorm, Color.blue);
            
            Plane RightFootCenterPlane = new Plane(RightFootCenterToeNorm, RightFootCenter);
		    Physics.Raycast(new Vector3(RightFootCenter.x, 100f, RightFootCenter.z), Vector3.down, out hit, float.PositiveInfinity, LayerMask.GetMask("Ground"));
            float RightCenterFootHeight = RightFootCenterPlane.GetDistanceToPoint(hit.point); //Vector3.Distance(RightFootCenter, hit.point);
            float RightCenterFootGlobalHeight = hit.point.y;

            //*****************************************************************************************************

            float LeftTransparentValue = 0f;
            float RightTransparentValue = 0f;
            float ToeOffset = 0.0f;
            CounterLeftPoints = 0;
            CounterLeftCorrectPoints = 0;
            CounterRightPoints = 0;
            CounterRightCorrectPoints = 0;
            CounterLeftAreaPoints = 0;
            CounterRightAreaPoints = 0;

            for(int i = 0; i < LeftPointsMap.Length; i++){
                if(ShowProjection){
                    Vector3 LeftPointAtToeHeight = LeftPointsMap[i].GetRelativePositionFrom(LeftPivot);
                    float LeftPointGlobalHeight = Utility.GetHeight(LeftPointsMap[i].GetRelativePositionFrom(LeftPivot), LayerMask.GetMask("Ground","Interaction"));
                    LeftPointAtToeHeight[1] = LeftToe.GetPosition().y;
                    if(Vector3.Distance(LeftAnkleAtToeHeight, LeftToe.GetPosition()) > Vector3.Distance(LeftAnkleAtToeHeight, LeftPointAtToeHeight) - ToeOffset){
                        LeftTransparentValue = 1f;
                        CounterLeftPoints += 1;
                        if(LeftPointGlobalHeight == LeftCenterFootGlobalHeight){
                            CounterLeftAreaPoints += 1;
                        }
                    }else{
                        LeftTransparentValue = 0f;
                    }
                    //float height = LeftPointsMap[i].y;
                    if(LeftAnkleToePlane.GetDistanceToPoint(LeftPoints[i])<=0f){
                        LeftFootProjection.DrawFootPoint(UltiDraw.Blue.Transparent(LeftTransparentValue), i);
                        if(LeftTransparentValue > 0f){
                            CounterLeftCorrectPoints += 1;
                        }
                    }else{
                        //LeftFootProjection.DrawFootPoint(UltiDraw.Red.Transparent(LeftTransparentValue), i);
                        LeftFootProjection.DrawFootPoint(UltiDraw.Blue.Transparent(LeftTransparentValue), i);
                    }
                    

                    Vector3 RightPointAtToeHeight = RightPointsMap[i].GetRelativePositionFrom(RightPivot);
                    float RightPointGlobalHeight = Utility.GetHeight(RightPointsMap[i].GetRelativePositionFrom(RightPivot), LayerMask.GetMask("Ground","Interaction"));
                    RightPointAtToeHeight[1] = RightToe.GetPosition().y;
                    if(Vector3.Distance(RightAnkleAtToeHeight, RightToe.GetPosition()) > Vector3.Distance(RightAnkleAtToeHeight, RightPointAtToeHeight) - ToeOffset){
                        RightTransparentValue = 1f;
                        CounterRightPoints += 1;
                        if(RightPointGlobalHeight == RightCenterFootGlobalHeight){
                            CounterRightAreaPoints += 1;
                        }
                    }else{
                        RightTransparentValue = 0f;
                    }
                    //float height = LeftPointsMap[i].y;
                    if(RightAnkleToePlane.GetDistanceToPoint(RightPoints[i])<=0f){
                        RightFootProjection.DrawFootPoint(UltiDraw.Green.Transparent(RightTransparentValue), i);
                        if(RightTransparentValue > 0f){
                            CounterRightCorrectPoints += 1;
                        }
                    }else{
                        //RightFootProjection.DrawFootPoint(UltiDraw.Red.Transparent(RightTransparentValue), i);
                        RightFootProjection.DrawFootPoint(UltiDraw.Green.Transparent(RightTransparentValue), i);
                    }
                    //LeftFootProjection.DrawFootPoint(UltiDraw.Blue.Transparent(1f), i);
                    //RightFootProjection.DrawFootPoint(UltiDraw.Green.Transparent(1f), i);
                }
            }
            //Debug.Log("Left Area Points = " + CounterLeftAreaPoints);
            //Debug.Log("Right Area Points = " + CounterRightAreaPoints);
            //LeftFootProjection.DrawFootMap(UltiDraw.Blue.Transparent(1f));
            //RightFootProjection.DrawFootMap(UltiDraw.Green.Transparent(1f));


            //Write columns: Projection Points | Correctly projected points | Foot Center Distance to Ground | 3d Center Foot point | Foot Velocity magnitude | Correct area points
            LeftFootData.WriteLine(CounterLeftPoints + " " + CounterLeftCorrectPoints + " " + 
                                    LeftCenterFootHeight.ToString("F5",new CultureInfo("en-US")) + " " + 
                                    LeftFootCenter.x.ToString("F5",new CultureInfo("en-US")) + " " + 
                                    LeftFootCenter.y.ToString("F5",new CultureInfo("en-US")) + " " + 
                                    LeftFootCenter.z.ToString("F5",new CultureInfo("en-US")) + " " + 
                                    LeftAnkleVelocity.magnitude.ToString("F5",new CultureInfo("en-US")) + " " +
                                    CounterLeftAreaPoints);

            //Write columns: Projection Points | Correctly projected points | Foot Center Distance to Ground | 3d Center Foot point | Foot Velocity magnitude | Correct area points
            RightFootData.WriteLine(CounterRightPoints + " " + CounterRightCorrectPoints + " " + 
                                    RightCenterFootHeight.ToString("F5",new CultureInfo("en-US")) + " " + 
                                    RightFootCenter.x.ToString("F5",new CultureInfo("en-US")) + " " + 
                                    RightFootCenter.y.ToString("F5",new CultureInfo("en-US")) + " " +  
                                    RightFootCenter.z.ToString("F5",new CultureInfo("en-US")) + " " +
                                    RightAnkleVelocity.magnitude.ToString("F5",new CultureInfo("en-US")) + " " +
                                    CounterRightAreaPoints);

        }
		
	}

    void OnApplicationQuit(){
        LeftFootData.Close();
        Debug.Log("Left Foot Data saved.");
        RightFootData.Close();
        Debug.Log("Right Foot Data saved.");
    }

    // Source: https://answers.unity.com/questions/467458/how-to-debug-drawing-plane.html
    void DrawPlane(Vector3 position, Vector3 normal) {
    
        var v3 = new Vector3(0f,0f,0f);
        
        if (normal.normalized != Vector3.forward)
            v3 = Vector3.Cross(normal, Vector3.forward).normalized * normal.magnitude;
        else
            v3 = Vector3.Cross(normal, Vector3.up).normalized * normal.magnitude;
            
        var corner0 = position + v3;
        var corner2 = position - v3;
        var q = Quaternion.AngleAxis(90.0f, normal);
        v3 = q * v3;
        var corner1 = position + v3;
        var corner3 = position - v3;
        
        Debug.DrawLine(corner0, corner2, Color.green);
        Debug.DrawLine(corner1, corner3, Color.green);
        Debug.DrawLine(corner0, corner1, Color.green);
        Debug.DrawLine(corner1, corner2, Color.green);
        Debug.DrawLine(corner2, corner3, Color.green);
        Debug.DrawLine(corner3, corner0, Color.green);
        Debug.DrawRay(position, normal, Color.red);
    }

    void DrawPlane(Vector3 position, Vector3 normal, Color planeColor) {
    
        var v3 = new Vector3(0f,0f,0f);
        
        if (normal.normalized != Vector3.forward)
            v3 = Vector3.Cross(normal, Vector3.forward).normalized * normal.magnitude;
        else
            v3 = Vector3.Cross(normal, Vector3.up).normalized * normal.magnitude;
            
        var corner0 = position + v3;
        var corner2 = position - v3;
        var q = Quaternion.AngleAxis(90.0f, normal);
        v3 = q * v3;
        var corner1 = position + v3;
        var corner3 = position - v3;
        
        Debug.DrawLine(corner0, corner2, planeColor);
        Debug.DrawLine(corner1, corner3, planeColor);
        Debug.DrawLine(corner0, corner1, planeColor);
        Debug.DrawLine(corner1, corner2, planeColor);
        Debug.DrawLine(corner2, corner3, planeColor);
        Debug.DrawLine(corner3, corner0, planeColor);
        Debug.DrawRay(position, normal, Color.red);
    }

    // Copied from MotionExporter
    private StreamWriter CreateFile(string name) {
		string filename = string.Empty;
		string folder = Application.dataPath + "/../../FeetData/";
		if(!File.Exists(folder+name+".txt")) {
			filename = folder+name;
		} else {
			int i = 1;
			while(File.Exists(folder+name+" ("+i+").txt")) {
				i += 1;
			}
			filename = folder+name+" ("+i+")";
		}
		return File.CreateText(filename+".txt");
	}

}
