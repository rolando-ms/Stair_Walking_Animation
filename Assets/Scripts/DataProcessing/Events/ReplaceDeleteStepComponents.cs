using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class ReplaceDeleteStepComponents : MonoBehaviour
{
    void Update()
    {
        for(int i=0; i < transform.childCount; i++)
        {
            Transform childTransform = GetChildTransform(transform.GetChild(i));
            if(childTransform.GetComponent<StepData>() == null)
            {
                childTransform.gameObject.AddComponent<StepData>();
            }
        }
    }

    void LateUpdate()
    {
        for(int i=0; i < transform.childCount; i++)
        {
            Transform childTransform = GetChildTransform(transform.GetChild(i));
            if(childTransform.GetComponent<NoiseSteps>() != null && childTransform.GetComponent<StepData>().DefaultScale.x != 1f)
            {
                DestroyImmediate(childTransform.GetComponent<NoiseSteps>());
            }
        }
        for(int i=0; i < transform.childCount; i++)
        {
            Transform childTransform = GetChildTransform(transform.GetChild(i));
            if(childTransform.GetComponent<StepData>().MaxScaleNoise.x != 1.25f)
            {
                childTransform.GetComponent<StepData>().MaxScaleNoise.x = 1.25f;
            }
            if(childTransform.GetComponent<StepData>().DefaultScale.y == 4f)
            {
                childTransform.GetComponent<StepData>().DefaultScale.y = 8f;
                childTransform.GetComponent<StepData>().DefaultPosition.y -= 2f;
            }
            if(childTransform.tag != "Tread")
            {
                childTransform.tag = "Tread";
            }
        }
    }

    void OnRemoveComponent()
    {
        for(int i=0; i < transform.childCount; i++)
        {
            Transform childTransform = GetChildTransform(transform.GetChild(i));
            //childTransform.gameObject.AddComponent<StepData>();
            Destroy(childTransform.GetComponent<NoiseSteps>());
        }
    }

    private Transform GetChildTransform(Transform childTransform){
        if(childTransform.childCount == 0) return childTransform;
        else return GetChildTransform(childTransform.GetChild(0));
    }
}
