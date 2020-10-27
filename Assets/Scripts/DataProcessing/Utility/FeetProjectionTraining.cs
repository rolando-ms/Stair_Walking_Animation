#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class FeetProjectionTraining : MonoBehaviour
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

    // Start is called before the first frame update
    void Awake()
    {
        RightFootProjection = new HeightMap(Width, Length, WidthResolution, LengthResolution, LayerMask.GetMask("Ground"));
        LeftFootProjection = new HeightMap(Width, Length, WidthResolution, LengthResolution, LayerMask.GetMask("Ground"));
    }

    public static MotionEditor GetInstance() {
		return GameObject.FindObjectOfType<MotionEditor>();
	}

    // Update is called once per frame
    void Sense()
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

    void OnDrawGizmos() {
        if(!Application.isPlaying) {
            Sense();
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
            float AnkleOffset = 0.05f;
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
            float LeftTransparentValue = 0f;
            float RightTransparentValue = 0f;
            float ToeOffset = 0.0f;

            for(int i = 0; i < LeftPointsMap.Length; i++){
                if(ShowProjection){
                    Vector3 LeftPointAtToeHeight = LeftPointsMap[i].GetRelativePositionFrom(LeftPivot);
                    LeftPointAtToeHeight[1] = LeftToe.GetPosition().y;
                    if(Vector3.Distance(LeftAnkleAtToeHeight, LeftToe.GetPosition()) > Vector3.Distance(LeftAnkleAtToeHeight, LeftPointAtToeHeight) - ToeOffset){
                        LeftTransparentValue = 1f;
                    }else{
                        LeftTransparentValue = 0f;
                    }
                    //float height = LeftPointsMap[i].y;
                    if(LeftAnkleToePlane.GetDistanceToPoint(LeftPoints[i])<=0f){
                        LeftFootProjection.DrawFootPoint(UltiDraw.Blue.Transparent(LeftTransparentValue), i);
                    }else{
                        LeftFootProjection.DrawFootPoint(UltiDraw.Red.Transparent(LeftTransparentValue), i);
                    }
                    

                    Vector3 RightPointAtToeHeight = RightPointsMap[i].GetRelativePositionFrom(RightPivot);
                    RightPointAtToeHeight[1] = RightToe.GetPosition().y;
                    if(Vector3.Distance(RightAnkleAtToeHeight, RightToe.GetPosition()) > Vector3.Distance(RightAnkleAtToeHeight, RightPointAtToeHeight) - ToeOffset){
                        RightTransparentValue = 1f;
                    }else{
                        RightTransparentValue = 0f;
                    }
                    //float height = LeftPointsMap[i].y;
                    if(RightAnkleToePlane.GetDistanceToPoint(RightPoints[i])<=0f){
                        RightFootProjection.DrawFootPoint(UltiDraw.Green.Transparent(RightTransparentValue), i);
                    }else{
                        RightFootProjection.DrawFootPoint(UltiDraw.Red.Transparent(RightTransparentValue), i);
                    }
                    //LeftFootProjection.DrawFootPoint(UltiDraw.Blue.Transparent(1f), i);
                    //RightFootProjection.DrawFootPoint(UltiDraw.Green.Transparent(1f), i);
                }
            }
            //LeftFootProjection.DrawFootMap(UltiDraw.Blue.Transparent(1f));
            //RightFootProjection.DrawFootMap(UltiDraw.Green.Transparent(1f));

        }
		
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

}
#endif