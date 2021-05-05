#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class RayTest : MonoBehaviour
{
    public GameObject targetCube;

    void Update()
    {
        Debug.DrawRay(transform.position, Vector3.down, Color.red, 1f);
        RaycastHit hitRay;
        Physics.Raycast(transform.position, Vector3.down, out hitRay , float.PositiveInfinity, LayerMask.GetMask("Ground"));
        Transform targetTransform = hitRay.transform;
        Vector4 PointV4 = new Vector4(1,1,1,1);
        PointV4[0] = hitRay.point[0];
        PointV4[1] = hitRay.point[1];
        PointV4[2] = hitRay.point[2];
        Matrix4x4 targetMatrix = targetTransform.GetWorldMatrix();
        Vector3 transformedPoint = targetMatrix.inverse.MultiplyPoint(hitRay.point);//targetTransform.GetWorldMatrix().MultiplyPoint(hitRay.point);

        Vector3 backTransformedPoint = targetMatrix.MultiplyPoint(transformedPoint);

        Debug.Log("Target Object position = " + targetTransform.position + " transformed point = " + transformedPoint + " back transformed point = " + backTransformedPoint);
    }
}
#endif