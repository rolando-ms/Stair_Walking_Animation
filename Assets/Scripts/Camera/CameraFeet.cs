using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditorInternal;

public class CameraFeet : MonoBehaviour
{
    public bool SideView = false;
    public Transform Hips;
    public Transform LeftAnkle;
    public Transform RightAnkle;

    private float MeanHeight;
    private Vector3 Target = new Vector3(0f, 0f, 0f);
    private Vector3 nextPosition = new Vector3(0f, 0f, 0f);

    public float OffsetX = 0f;
    public float OffsetY = 0f;
    public float OffsetZ = -2f;

    //public int CameraView = 0;

    //[Range(0f, 1f)] public float damping = 0.05f;

    [Range(0, 2)] public int CameraView = 0;

    //private float aux = 0f;
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        MeanHeight = (LeftAnkle.position.y + RightAnkle.position.y) / 2f;
        Target = new Vector3(Hips.position.x, MeanHeight, Hips.position.z);
        Vector3 currentPosition = transform.position;
        Camera[] SceneCameras = new Camera[Camera.allCamerasCount];
        Camera.GetAllCameras(SceneCameras);
        //Debug.Log("Cameras = " + Camera.allCamerasCount);

        //EditorGUILayout.LabelField("0 = Global Z, 1 = Global X, 2 = Global Y, 3 = Follow Z, 4 = Follow X");
		//CameraView = EditorGUILayout.IntSlider(CameraView, 0, 4);

        switch(CameraView){
            /*case 4:
                // Follow X
                transform.rotation = Hips.rotation;
                SceneCameras[1].fieldOfView = 28f;
                OffsetX = -2f;
                OffsetY = 0f;
                OffsetZ = 0f;
                transform.position = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
                break;
            case 3:
                // Follow Z
                transform.rotation = Hips.rotation;
                SceneCameras[1].fieldOfView = 28f;
                OffsetX = 0f;
                OffsetY = 0f;
                OffsetZ = -2f;
                transform.position = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
                break;
                */
            case 2:
                // Global Y
                //Target.y = Hips.position.y;
                SceneCameras[1].fieldOfView = 70f;
                OffsetX = 0f;
                OffsetY = 0f;
                OffsetZ = 0.35f;
                transform.position = new Vector3(Target.x + OffsetX, Hips.position.y + OffsetY, Target.z + OffsetZ);
                break;
            case 1:
                // Global X
                SceneCameras[1].fieldOfView = 28f;
                OffsetX = -2f;
                OffsetY = 0.25f;
                OffsetZ = -0.5f;
                transform.position = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
                break;
            case 0:
                // Global Z
                SceneCameras[1].fieldOfView = 28f;
                OffsetX = 0f;
                OffsetY = 0f;
                OffsetZ = -2f;
                transform.position = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
                break;
            default:
                // Global Z
                SceneCameras[1].fieldOfView = 28f;
                OffsetX = 0f;
                OffsetY = 0f;
                OffsetZ = -2f;
                transform.position = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
                break;
        }

        /*if(SideView){
            //nextPosition = new Vector3(Target.x + OffsetZ, Target.y + OffsetY, Target.z + OffsetX);
            transform.position = new Vector3(Target.x + OffsetZ, Target.y + OffsetY, Target.z + OffsetX);
            //transform.position = Vector3.Lerp(currentPosition, Target, damping);
        }else{
            //nextPosition = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
            transform.position = new Vector3(Target.x + OffsetX, Target.y + OffsetY, Target.z + OffsetZ);
        }*/
        //transform.position = Vector3.Lerp(currentPosition, transform.position, damping);

        
        transform.LookAt(Target);
    }
}
