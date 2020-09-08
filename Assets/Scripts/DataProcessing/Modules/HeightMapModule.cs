#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditorInternal;

public class HeightMapModule : Module {

	public float Size = 1f;
	public int Resolution = 25;
	public LayerMask Mask = -1;
	private int Samples = 0;

	public int GridType = 0;
	public bool SquareGrid = true;
	public float Width = 1;
	public float Length = 1;
	public int WidthResolution = 10;
	public int LengthResolution = 10;

	public bool CircularGrid = false;

	public override ID GetID() {
		return ID.HeightMap;
	}

	public override Module Initialise(MotionData data) {
		Data = data;
		return this;
	}

	public override void Slice(Sequence sequence) {

	}

	public override void Callback(MotionEditor editor) {
		
	}

	public HeightMap GetHeightMap(Actor actor) {
		HeightMap sensor;
		switch(GridType){
			case 3:
				CircularGrid = true;
				sensor = new HeightMap(Size, Resolution, Mask);
				break;
			case 2:
				CircularGrid = true;
				sensor = new HeightMap(Size, Resolution, Mask, CircularGrid);
				break;
			case 1:
				sensor = new HeightMap(Width, Length, WidthResolution, LengthResolution, Mask);
				break;
			case 0:
				CircularGrid = false;
				sensor = new HeightMap(Size, Resolution, Mask, CircularGrid);
				break;
			default:
				CircularGrid = false;
				sensor = new HeightMap(Size, Resolution, Mask, CircularGrid);
				break;
		}
		
		sensor.Sense(actor.GetRoot().GetWorldMatrix());
		Samples = sensor.Points.Length;
		return sensor;
	}

	protected override void DerivedDraw(MotionEditor editor) {
		HeightMap sensor = GetHeightMap(editor.GetActor());
		sensor.Draw();
		//sensor.Render(new Vector2(0.1f, 0.25f), new Vector2(0.3f*Screen.height/Screen.width, 0.3f), Resolution, Resolution, 1f);
	}

	protected override void DerivedInspector(MotionEditor editor) {
		Size = EditorGUILayout.FloatField("Size", Size);
		Resolution = EditorGUILayout.IntField("Resolution", Resolution);
		Mask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(EditorGUILayout.MaskField("Mask", InternalEditorUtility.LayerMaskToConcatenatedLayersMask(Mask), InternalEditorUtility.layers));
		EditorGUILayout.LabelField("Samples: " + Samples);
		EditorGUILayout.LabelField("0 = Square, 1 = Rectangular, 2 = Circular, 3 = Half Circle");
		GridType = EditorGUILayout.IntSlider(GridType, 0, 3);
		EditorGUILayout.LabelField("Rectangular Grid Parameters");
		Width = EditorGUILayout.FloatField("Width", Width);
		Length = EditorGUILayout.FloatField("Length", Length);
		WidthResolution = EditorGUILayout.IntField("Width Resolution", WidthResolution);
		LengthResolution = EditorGUILayout.IntField("Length Resolution", LengthResolution);
	}

}
#endif
