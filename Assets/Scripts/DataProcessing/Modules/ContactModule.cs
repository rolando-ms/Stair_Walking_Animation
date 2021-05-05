#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine.SceneManagement;

public class ContactModule : Module {

	public float DrawScale = 1f;
	public bool ShowDebug = true;
	public bool ShowSensors = true;
	public bool ShowTolerances = true;
	public bool ShowDistances = false;
	public bool TrueMotionTrajectory = false;
	public bool CorrectedMotionTrajectory = false;
	public bool ShowContacts = false;
	public bool ContactTrajectories = false;
	public bool ShowFutureGoalPoints = false;
	public bool useSteps = true;
	public bool IKTest = false;

	//public bool ShowSkeletons = false;
	//public int SkeletonStep = 10;

	public bool EditMotion = true;
	public int Step = 10;
	public float CaptureFilter = 0.1f;
	public float EditFilter = 0.1f;
	public Sensor[] Sensors = new Sensor[0];
	public RootSensor rootSensor = new RootSensor(null);
	public BakedContacts BakedContacts = null;

	public float PastTrajectoryWindow = 1f;
	public float FutureTrajectoryWindow = 1f;

	private float Window = 1f;

	private UltimateIK.Model IK;

	public override ID GetID() {
		return ID.Contact;
	}

	public override Module Initialise(MotionData data) {
		Data = data;
		return this;
	}

	public override void Slice(Sequence sequence) {

	}

	public override void Callback(MotionEditor editor) {
		if(EditMotion) {
			//Debug.Log("Max scale noise = " + editor.MaxScaleNoiseZ);
			//Debug.Log("Random scale noise = " + editor.RandomScaleNoiseZ);
			float delta = 1f/editor.TargetFramerate;
			Actor actor = editor.GetActor();
			IK = UltimateIK.BuildModel(IK, actor.Bones[0].Transform, GetObjectives(actor));
			if(IK.Objectives.Length == 0) {
				return;
			}
			IK.Iterations = 50;
			IK.Activation = UltimateIK.ACTIVATION.Constant;
			IK.RootTranslationY = true;
			IK.SetWeights(GetWeights());
			bool[] solvePositions = GetSolvePositions();
			bool[] solveRotations = GetSolveRotations();
			for(int i=0; i<IK.Objectives.Length; i++) {
				IK.Objectives[i].SolvePosition = solvePositions[i];
				IK.Objectives[i].SolveRotation = solveRotations[i];
			}
			
			Frame frame = editor.GetCurrentFrame();
			Frame relative = (frame.Timestamp - delta) < 0f ? Data.GetFrame(frame.Timestamp + delta) : Data.GetFrame(frame.Timestamp - delta);
			actor.WriteTransforms(relative.GetBoneTransformations(editor.Mirror), Data.Source.GetBoneNames());
			//IK.Solve(GetTargets(relative, editor.Mirror));
			IK.Solve(GetTargets(relative, editor));
			Matrix4x4[] relativePosture = actor.GetBoneTransformations();
			actor.WriteTransforms(frame.GetBoneTransformations(editor.Mirror), Data.Source.GetBoneNames());
			//IK.Solve(GetTargets(frame, editor.Mirror));
			IK.Solve(GetTargets(frame, editor));
			Matrix4x4[] framePosture = actor.GetBoneTransformations();
			
			for(int i=0; i<actor.Bones.Length; i++) {
				actor.Bones[i].Velocity = (frame.Timestamp - delta) < 0f ? 
				(relativePosture[i].GetPosition() - framePosture[i].GetPosition()) / delta:
				(framePosture[i].GetPosition() - relativePosture[i].GetPosition()) / delta;
			}
		}
	}

	public Sensor AddSensor() {
		return AddSensor(Data.Source.Bones[0].Name);
	}

	public Sensor AddSensor(string bone) {
		return AddSensor(bone, Vector3.zero, 0.1f, 0f, 0f);
	}

	public Sensor AddSensor(string bone, Vector3 offset, float threshold, float tolerance, float velocity) {
		return AddSensor(bone, offset, threshold, tolerance, velocity, Sensor.ID.Closest, Sensor.ID.None);
	}

	public Sensor AddSensor(string bone, Vector3 offset, float threshold, float tolerance, float velocity, Sensor.ID capture, Sensor.ID edit) {
		Sensor sensor = new Sensor(this, Data.Source.FindBone(bone).Index, offset, threshold, tolerance, velocity, capture, edit);
		ArrayExtensions.Add(ref Sensors, sensor);
		return sensor;
	}

	public void AddRootSensor(){
		rootSensor = new RootSensor(this);
		//return rootSensor;
	}

	public void RemoveSensor(Sensor sensor) {
		if(!ArrayExtensions.Remove(ref Sensors, sensor)) {
			Debug.Log("Sensor could not be found in " + Data.GetName() + ".");
		}
	}

	public void Clear() {
		ArrayExtensions.Clear(ref Sensors);
	}

	public string[] GetNames() {
		string[] names = new string[Sensors.Length];
		for(int i=0; i<Sensors.Length; i++) {
			names[i] = Sensors[i].GetName();
		}
		return names;
	}

	public Sensor GetSensor(string bone) {
		return System.Array.Find(Sensors, x => x.GetName() == bone);
	}

	public float[] GetContacts(Frame frame, bool mirrored) {
		float[] contacts = new float[Sensors.Length];
		for(int i=0; i<Sensors.Length; i++) {
			contacts[i] = Sensors[i].GetContact(frame, mirrored);
		}
		return contacts;
	}

	/*public float[] GetFutureGoalPoints(Frame frame, bool mirrored) {
		float[] contacts = new float[Sensors.Length];
		for(int i=0; i<Sensors.Length; i++) {
			contacts[i] = Sensors[i].GetContact(frame, mirrored);
		}
		return contacts;
	}*/

	public float[] GetContacts(Frame frame, bool mirrored, params string[] bones) {
		float[] contacts = new float[bones.Length];
		for(int i=0; i<bones.Length; i++) {
			Sensor sensor = GetSensor(bones[i]);
			if(sensor == null) {
				Debug.Log("Sensor for bone " + bones[i] + " could not be found.");
				contacts[i] = 0f;
			} else {
				contacts[i] = sensor.GetContact(frame, mirrored);
			}
		}
		return contacts;
	}

	public Matrix4x4[] GetTargets(Frame frame, bool mirrored) {
		List<Matrix4x4> targets = new List<Matrix4x4>();
		for(int i=0; i<Sensors.Length; i++) {
			if(Sensors[i].Edit != Sensor.ID.None) {
				targets.Add(Sensors[i].GetCorrectedTransformation(frame, mirrored));
			}
		}
		return targets.ToArray();
	}

	public Matrix4x4[] GetTargets(Frame frame, MotionEditor editor) {
		UltiDraw.Begin();
		bool mirrored = editor.Mirror;
		/*GameObject tread = GameObject.Find("Cube");
		//Debug.Log("Object name = " + tread);
		Vector3 treadSize = Vector3.Scale(tread.GetComponent<Transform>().localScale, tread.GetComponent<MeshFilter>().mesh.bounds.size);
		//Debug.Log("Tread depth = " + treadSize[2].ToString("F2"));
		float treadDepthScaled = treadSize.z * editor.RandomScaleNoiseZ;
		//Debug.Log("Depth scaled = " + treadDepthScaled.ToString("F2"));
		*/

		Actor actor = editor.GetActor();
		Matrix4x4 rightFoot = actor.GetBoneTransformation("RightAnkle");
		Matrix4x4 leftFoot = actor.GetBoneTransformation("LeftAnkle");

		RaycastHit hitRightfoot;
		Physics.Raycast(new Vector3(rightFoot.GetPosition().x, rightFoot.GetPosition().y, rightFoot.GetPosition().z), 
		                Vector3.down, out hitRightfoot, float.PositiveInfinity); // 8 = Interaction mask
		UltiDraw.DrawSphere(new Vector3(rightFoot.GetPosition().x, rightFoot.GetPosition().y, rightFoot.GetPosition().z), new Quaternion(0,0,0,1), 0.5f, UltiDraw.Blue);
		float rightFootHitY = hitRightfoot.point.y;
		//Debug.Log("Hit point = " + hitRightfoot.point.ToString());
		//Debug.Log("Right Foot y position = " + rightFoot.GetPosition().y);
		//Debug.Log("Right Foot height = " + rightFootHitY);

		/*Vector3 sensorStartPos = new Vector3(rightFoot.GetPosition().x, rightFoot.GetPosition().y, rightFoot.GetPosition().z);
		if (Physics.Raycast(sensorStartPos, Vector3.down, out hitRightfoot, float.PositiveInfinity))
		{
			Debug.Log("Hit point!!!");
			//if it a surface, then Draw Red line to the hit point
			Debug.DrawLine(sensorStartPos, hitRightfoot.point, Color.red, 2f, false);
		} else
		{
			Debug.Log("Nothing hit!");
			//If don't hit, then draw Green line to the direction we are sensing,
			//Note hit.point will remain 0,0,0 at this point, because we don't hit anything
			//So you cannot use hit.point
			Debug.DrawLine(sensorStartPos, sensorStartPos + (Vector3.down * float.PositiveInfinity), Color.green, 2f, false);
		}*/

		RaycastHit hitLeftfoot;
		Physics.Raycast(new Vector3(leftFoot.GetPosition().x, leftFoot.GetPosition().y, leftFoot.GetPosition().z), 
		                Vector3.down, out hitLeftfoot, float.PositiveInfinity); // 8 = Interaction mask
		float leftFootHitY = hitLeftfoot.point.y;
		//Debug.Log("Local Scale = " + (hitLeftfoot.transform.localScale.x));
		//Debug.Log("Name of object = " + hitLeftfoot.collider.name);
		//Debug.Log("Default Z = " + hitLeftfoot.collider.gameObject.GetComponent<NoiseSteps>().DefaultScale.x);

		/*if(rightFootHitY == leftFootHitY){
			Debug.Log("Same Height!!!");
		}else{
			Debug.Log("Different height!");
		}*/
		//Matrix4x4.TRS(bone.GetPosition() + Utility.FilterGaussian(distances, contacts), bone.GetRotation(), Vector3.one);
		string colliderName = rootSensor.GetRootContactTreadName(frame, mirrored);
		//Debug.Log("Root Collider Name = " + colliderName);
		//Debug.Log("Root Local point = " + rootSensor.GetRootLocalXContact(frame, editor.Mirror));

		List<Matrix4x4> targets = new List<Matrix4x4>();
		for(int i=0; i<Sensors.Length; i++) {
			if(Sensors[i].Edit != Sensor.ID.None) {
				//Vector3 HorizontalOffset = Vector3.zero;
				Vector3 correctedPosition = Vector3.zero;
				Matrix4x4 rotationZ = Matrix4x4.TRS(new Vector3(0f,0f,0f), Quaternion.Euler(0f,0f,0f), Vector3.one);
				Matrix4x4 correctedMatrix = Matrix4x4.TRS(new Vector3(0f,0f,0f), Quaternion.Euler(0f,0f,0f), Vector3.one);
				Matrix4x4 correctedTransformation = Sensors[i].GetCorrectedTransformation(frame, mirrored);
				if(useSteps){
					//HorizontalOffset = Sensors[i].GetHorizontalStepOffsetVector(frame, mirrored); // Offset from horizontal step noise
					correctedPosition = GetCorrectedStraightStairWalkingStep(frame, editor.GetActor().GetRoot(), correctedTransformation.GetPosition(), Sensors[i].GetName(), mirrored);
					Vector3 offsetCorrectedPosition = correctedPosition;
					offsetCorrectedPosition.y += 1f;
					RaycastHit hitRay;
					Physics.Raycast(offsetCorrectedPosition, Vector3.down, out hitRay, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
					//Debug.DrawRay(offsetCorrectedPosition, Vector3.down, Color.cyan, 1f);
					//if(hitRay.collider.tag == "Tread"){
					//	if(Sensors[i].GetContact(frame, mirrored) > 0f){
							//Sensors[i].Get
							//correctedPosition += Vector3.up.normalized * 0.08f; // Ankle offset
					//	}
					//}
					
					//if(Sensors[i].GetName() == "LeftAnkle"){
					//	string colliderName = Sensors[i].GetRootContactTreadName(frame, mirrored);
					//	Debug.Log("Root Collider Name = " + colliderName);
					//}
					correctedPosition = GetCorrectedPointWithRelativeContacts(frame, Sensors[i].GetName(), mirrored);
					correctedMatrix = Matrix4x4.TRS(correctedPosition, correctedTransformation.GetRotation(), Vector3.one);
				}else{
					correctedMatrix = Matrix4x4.TRS(correctedTransformation.GetPosition(), correctedTransformation.GetRotation(), Vector3.one);	
				}
				if(IKTest){
					Vector3 position = Vector3.zero;
					if(Sensors[i].GetName() == "LeftAnkle"){
						GameObject leftStep = GameObject.Find("LeftStep");
						position = leftStep.transform.position;
						position.y += leftStep.transform.localScale.y / 2f;
					}else if(Sensors[i].GetName() == "RightAnkle"){
						GameObject rightStep = GameObject.Find("RightStep");
						position = rightStep.transform.position;
						position.y += rightStep.transform.localScale.y / 2f;
					}
					position.y = correctedTransformation.GetPosition().y;
					correctedMatrix = Matrix4x4.TRS(position, correctedTransformation.GetRotation(), Vector3.one);
				}
				//correctedMatrix = Matrix4x4.TRS(correctedTransformation.GetPosition() + HorizontalOffset, correctedTransformation.GetRotation(), Vector3.one);
				targets.Add(rotationZ * correctedMatrix);
			}
		}
		UltiDraw.End();
		return targets.ToArray();
	}

	public Transform[] GetObjectives(Actor actor) {
		List<Transform> objectives = new List<Transform>();
		for(int i=0; i<Sensors.Length; i++) {
			if(Sensors[i].Edit != Sensor.ID.None) {
				objectives.Add(actor.FindTransform(Data.Source.Bones[Sensors[i].Bone].Name));
			}
		}
		return objectives.ToArray();
	}

	public float[] GetWeights() {
		List<float> weights = new List<float>();
		for(int i=0; i<Sensors.Length; i++) {
			if(Sensors[i].Edit != Sensor.ID.None) {
				weights.Add(Sensors[i].Weight);
			}
		}
		return weights.ToArray();
	}

	public bool[] GetSolvePositions() {
		List<bool> values = new List<bool>();
		for(int i=0; i<Sensors.Length; i++) {
			if(Sensors[i].Edit != Sensor.ID.None) {
				values.Add(Sensors[i].SolvePosition);
			}
		}
		return values.ToArray();
	}

	public bool[] GetSolveRotations() {
		List<bool> values = new List<bool>();
		for(int i=0; i<Sensors.Length; i++) {
			if(Sensors[i].Edit != Sensor.ID.None) {
				values.Add(Sensors[i].SolveRotation);
			}
		}
		return values.ToArray();
	}

	public IEnumerator CaptureContacts(MotionEditor editor) {
		bool edit = EditMotion;
		EditMotion = false;
		Frame current = editor.GetCurrentFrame();
		for(int s=0; s<Sensors.Length; s++) {
			Sensors[s].RegularContacts = new float[Data.Frames.Length];
			Sensors[s].InverseContacts = new float[Data.Frames.Length];
			Sensors[s].RegularContactPoints = new Vector3[Data.Frames.Length];
			Sensors[s].InverseContactPoints = new Vector3[Data.Frames.Length];
			Sensors[s].RegularDistances = new Vector3[Data.Frames.Length];
			Sensors[s].InverseDistances = new Vector3[Data.Frames.Length];
			Sensors[s].RegularDirections = new Vector3[Data.Frames.Length];
			Sensors[s].InverseDirections = new Vector3[Data.Frames.Length];
			Sensors[s].RegularFutureGoalPoints = new Vector3[Data.Frames.Length];
			Sensors[s].InverseFutureGoalPoints = new Vector3[Data.Frames.Length];
			Sensors[s].RegularFutureGoalDirections = new Vector3[Data.Frames.Length];
			Sensors[s].InverseFutureGoalDirections = new Vector3[Data.Frames.Length];
			Sensors[s].RegularContactColliderNames = new string[Data.Frames.Length];
			Sensors[s].InverseContactColliderNames = new string[Data.Frames.Length];
			Sensors[s].RegularFutureGoalColliderNames = new string[Data.Frames.Length];
			Sensors[s].InverseFutureGoalColliderNames = new string[Data.Frames.Length];

			//Relative data
			Sensors[s].RegularRelativeContactPoints = new Vector3[Data.Frames.Length];
			Sensors[s].InverseRelativeContactPoints = new Vector3[Data.Frames.Length];
			Sensors[s].RegularRelativeFutureGoalPoints = new Vector3[Data.Frames.Length];
			Sensors[s].InverseRelativeFutureGoalPoints = new Vector3[Data.Frames.Length];

		}
		
		//Root Data
		//rootSensor = new RootSensor(this);
		rootSensor.RegularContactTreads = new string[Data.Frames.Length];
		rootSensor.InverseContactTreads = new string[Data.Frames.Length];
		rootSensor.RegularLocalXContacts = new Vector3[Data.Frames.Length];
		rootSensor.InverseLocalXContacts = new Vector3[Data.Frames.Length];
		System.DateTime time = Utility.GetTimestamp();
		//int count = 0;
		for(int i=0; i<Data.Frames.Length; i++) {
			//count += 1;
			Frame frame = Data.Frames[i];
			editor.LoadFrame(frame);
			for(int s=0; s<Sensors.Length; s++) {
				Sensors[s].CaptureContact(frame, editor);
				if(useSteps){
					Sensors[s].CaptureDirections(frame, editor);
					Vector3 point = editor.Mirror ? Sensors[s].InverseContactPoints[frame.Index-1] : Sensors[s].RegularContactPoints[frame.Index-1];
					Sensors[s].CaptureColliderName(frame, editor, point);
					Sensors[s].CaptureFutureColliderName(frame, editor, point);
					Sensors[s].CaptureRelativeToTreadContactPoints(frame, editor, point);
					Sensors[s].CaptureRelativeToTreadFutureGoalPoints(frame, editor, point);
				}
			}
			rootSensor.CaptureTreadName(frame, editor);
			rootSensor.CaptureLocalXContact(frame, editor);
			rootSensor.CaptureFirstTread(frame, editor);
			rootSensor.CaptureLastTread(frame, editor);
			//if(count > Step) {
			if(Utility.GetElapsedTime(time) > 0.2f) {
				time = Utility.GetTimestamp();
				//count = 0;
				yield return new WaitForSeconds(0f);
			}
			//}
		}
		if(useSteps){
			editor.Mirror = !editor.Mirror;
			for(int i=0; i<Data.Frames.Length; i++) {
				//count += 1;
				Frame frame = Data.Frames[i];
				editor.LoadFrame(frame);
				for(int s=0; s<Sensors.Length; s++) {
					Sensors[s].CaptureDirections(frame, editor);
					Vector3 point = editor.Mirror ? Sensors[s].InverseContactPoints[frame.Index-1] : Sensors[s].RegularContactPoints[frame.Index-1];
					Sensors[s].CaptureColliderName(frame, editor, point);
					Sensors[s].CaptureFutureColliderName(frame, editor, point);
					Sensors[s].CaptureRelativeToTreadContactPoints(frame, editor, point);
					Sensors[s].CaptureRelativeToTreadFutureGoalPoints(frame, editor, point);
				}
				rootSensor.CaptureTreadName(frame, editor);
				rootSensor.CaptureLocalXContact(frame, editor);
				//if(count > Step) {
				if(Utility.GetElapsedTime(time) > 0.2f) {
					time = Utility.GetTimestamp();
					//count = 0;
					yield return new WaitForSeconds(0f);
				}
				//}
			}
			editor.Mirror = !editor.Mirror;
			for(int i=0; i<Data.Frames.Length; i++){
				Frame frame = Data.Frames[i];
				for(int s=0; s<Sensors.Length; s++){
					Sensors[s].CaptureFutureGoalPoints(frame, FutureTrajectoryWindow, Mathf.RoundToInt(Data.Framerate));
				}
			}
			for(int s=0; s<Sensors.Length; s++){
				string sensorName = Sensors[s].GetName();
				if(sensorName == "LeftAnkle" || sensorName == "RightAnkle"){
					Sensors[s].FillInZeros(Data.Frames.Length);
				}
			}
		}
		
		Debug.Log("Future Goal Points Captured.");
		editor.LoadFrame(current);
		EditMotion = edit;
	}

	public void CaptureFeetVelocities(MotionEditor editor){
		StreamWriter FootVelocities;
		FootVelocities = CreateFile("DscFeetVelocities");
		for(int i=0; i < Data.Frames.Length; i++){
			Frame frame = Data.Frames[i];
			editor.LoadFrame(frame);
			FootVelocities.WriteLine(editor.GetActor().GetRoot().position.x.ToString("F5",new CultureInfo("en-US")) + " " +
									editor.GetActor().GetRoot().position.y.ToString("F5",new CultureInfo("en-US")) + " " +
									editor.GetActor().GetRoot().position.z.ToString("F5",new CultureInfo("en-US")) + " " +
									editor.GetActor().GetBoneVelocity("LeftAnkle").magnitude.ToString("F5",new CultureInfo("en-US")) + " " +
									editor.GetActor().GetBoneVelocity("RightAnkle").magnitude.ToString("F5",new CultureInfo("en-US")));
		}
		FootVelocities.Close();
		Debug.Log("Feet Velocities saved.");
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

	public void CaptureContactsNoCoroutine(MotionEditor editor) {
		bool edit = EditMotion;
		EditMotion = false;
		Frame current = editor.GetCurrentFrame();
		for(int s=0; s<Sensors.Length; s++) {
			Sensors[s].RegularContacts = new float[Data.Frames.Length];
			Sensors[s].InverseContacts = new float[Data.Frames.Length];
			Sensors[s].RegularContactPoints = new Vector3[Data.Frames.Length];
			Sensors[s].InverseContactPoints = new Vector3[Data.Frames.Length];
			Sensors[s].RegularDistances = new Vector3[Data.Frames.Length];
			Sensors[s].InverseDistances = new Vector3[Data.Frames.Length];
		}
		for(int i=0; i<Data.Frames.Length; i++) {
			Frame frame = Data.Frames[i];
			editor.LoadFrame(frame);
			for(int s=0; s<Sensors.Length; s++) {
				Sensors[s].CaptureContact(frame, editor);
			}
		}
		editor.LoadFrame(current);
		EditMotion = edit;
	}

	public void BakeContacts(MotionEditor editor) {
		if(BakedContacts == null) {
			return;
		}
		BakedContacts.Setup(GetNames(), Data.GetTotalFrames());
		for(int i=0; i<Data.Frames.Length; i++) {
			for(int s=0; s<Sensors.Length; s++) {
				if(Sensors[s].GetContact(Data.Frames[i], false) == 1f) {
					BakedContacts.BakeContact(s, Sensors[s].GetContactPoint(Data.Frames[i], false), Data.Frames[i], false);
				}
				if(Sensors[s].GetContact(Data.Frames[i], true) == 1f) {
					BakedContacts.BakeContact(s, Sensors[s].GetContactPoint(Data.Frames[i], true), Data.Frames[i], true);
				}
			}
		}
	}

	/* 
	****************************************************************
		Feet Data
	****************************************************************
	*/ 

	public Matrix4x4 GetLeftFootTransformation(Frame frame, bool mirrored) {
		return Matrix4x4.TRS(GetLeftFootPosition(frame, mirrored), GetLeftFootRotation(frame, mirrored), Vector3.one);
	}

	public Vector3 GetLeftFootPosition(Frame frame, bool mirrored) {
		return frame.GetBoneTransformation("LeftAnkle", mirrored).GetPosition();
	}

	public Quaternion GetLeftFootRotation(Frame frame, bool mirrored) {
		return frame.GetBoneTransformation("LeftAnkle", mirrored).GetRotation();
	}

	public Vector3 GetLeftFootVelocity(Frame frame, bool mirrored, float delta) {
		return frame.GetBoneVelocity("LeftAnkle", mirrored, delta);
	}

	public Matrix4x4 GetEstimatedLeftFootTransformation(Frame reference, float offset, bool mirrored) {
		return Matrix4x4.TRS(GetEstimatedLeftFootPosition(reference, offset, mirrored), GetEstimatedLeftFootRotation(reference, offset, mirrored), Vector3.one);
	}

	public Vector3 GetEstimatedLeftFootPosition(Frame reference, float offset, bool mirrored) {
		float t = reference.Timestamp + offset;
		if(t < 0f || t > Data.GetTotalTime()) {
			float boundary = Mathf.Clamp(t, 0f, Data.GetTotalTime());
			float pivot = 2f*boundary - t;
			float clamped = Mathf.Clamp(pivot, 0f, Data.GetTotalTime());
			return 2f*GetLeftFootPosition(Data.GetFrame(boundary), mirrored) - GetLeftFootPosition(Data.GetFrame(clamped), mirrored);
		} else {
			return GetLeftFootPosition(Data.GetFrame(t), mirrored);
		}
	}

	public Quaternion GetEstimatedLeftFootRotation(Frame reference, float offset, bool mirrored) {
		float t = reference.Timestamp + offset;
		if(t < 0f || t > Data.GetTotalTime()) {
			float boundary = Mathf.Clamp(t, 0f, Data.GetTotalTime());
			float pivot = 2f*boundary - t;
			float clamped = Mathf.Clamp(pivot, 0f, Data.GetTotalTime());
			return GetLeftFootRotation(Data.GetFrame(clamped), mirrored);
		} else {
			return GetLeftFootRotation(Data.GetFrame(t), mirrored);
		}
	}

	public Vector3 GetEstimatedLeftFootVelocity(Frame reference, float offset, bool mirrored, float delta) {
		return (GetEstimatedLeftFootPosition(reference, offset + delta, mirrored) - GetEstimatedLeftFootPosition(reference, offset, mirrored)) / delta;
	}

	public Matrix4x4 GetRightFootTransformation(Frame frame, bool mirrored) {
		return Matrix4x4.TRS(GetRightFootPosition(frame, mirrored), GetRightFootRotation(frame, mirrored), Vector3.one);
	}

	public Vector3 GetRightFootPosition(Frame frame, bool mirrored) {
		return frame.GetBoneTransformation("RightAnkle", mirrored).GetPosition();
	}

	public Quaternion GetRightFootRotation(Frame frame, bool mirrored) {
		return frame.GetBoneTransformation("RightAnkle", mirrored).GetRotation();
	}

	public Vector3 GetRightFootVelocity(Frame frame, bool mirrored, float delta) {
		return frame.GetBoneVelocity("RightAnkle", mirrored, delta);
	}

	public Matrix4x4 GetEstimatedRightFootTransformation(Frame reference, float offset, bool mirrored) {
		return Matrix4x4.TRS(GetEstimatedRightFootPosition(reference, offset, mirrored), GetEstimatedRightFootRotation(reference, offset, mirrored), Vector3.one);
	}

	public Vector3 GetEstimatedRightFootPosition(Frame reference, float offset, bool mirrored) {
		float t = reference.Timestamp + offset;
		if(t < 0f || t > Data.GetTotalTime()) {
			float boundary = Mathf.Clamp(t, 0f, Data.GetTotalTime());
			float pivot = 2f*boundary - t;
			float clamped = Mathf.Clamp(pivot, 0f, Data.GetTotalTime());
			return 2f*GetRightFootPosition(Data.GetFrame(boundary), mirrored) - GetRightFootPosition(Data.GetFrame(clamped), mirrored);
		} else {
			return GetRightFootPosition(Data.GetFrame(t), mirrored);
		}
	}

	public Quaternion GetEstimatedRightFootRotation(Frame reference, float offset, bool mirrored) {
		float t = reference.Timestamp + offset;
		if(t < 0f || t > Data.GetTotalTime()) {
			float boundary = Mathf.Clamp(t, 0f, Data.GetTotalTime());
			float pivot = 2f*boundary - t;
			float clamped = Mathf.Clamp(pivot, 0f, Data.GetTotalTime());
			return GetRightFootRotation(Data.GetFrame(clamped), mirrored);
		} else {
			return GetRightFootRotation(Data.GetFrame(t), mirrored);
		}
	}

	public Vector3 GetEstimatedRightFootVelocity(Frame reference, float offset, bool mirrored, float delta) {
		return (GetEstimatedRightFootPosition(reference, offset + delta, mirrored) - GetEstimatedRightFootPosition(reference, offset, mirrored)) / delta;
	}

	public Vector3 GetCorrectedStraightStairWalkingStep(Frame frame, Transform root, Vector3 point, string sensorName, bool mirrored){
		/*
		Sensor sensor = GetSensor(sensorName);
		Vector3 correctionOffset = Vector3.zero;
		RaycastHit contactRay;
		Vector3 pointMovedUp = point;
		pointMovedUp.y += 1f;
		Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		if(sensor.GetContact(frame, mirrored) > 0){
			if(Vector3.Distance(point, contactRay.point) < sensor.Threshold){
				//Vector3 correctionOffset = sensor.GetCorrectedContactPoint(frame, mirrored) - sensor.GetContactPoint(frame, mirrored);
				correctionOffset = GetCorrectionVector(frame, mirrored, sensorName);
				return point + correctionOffset;
			}else{
				//if(contactRay.transform.tag == "Tread"){
				//	if(GetHomogeneousNoiseStatus()) return GetCorrectedPointWithEvenNoise(frame, root, point, sensorName, mirrored);
				//	else return GetCorrectedPointWithStepNoise(root, point, mirrored);
				//}else return point;
				
				Sensor.ID originalID = sensor.Edit;
				float originalThreshold = sensor.Threshold;
				sensor.Edit = Sensor.ID.RayTopDown;
				sensor.Threshold = 1f;
				Vector3 newPoint = sensor.GetCorrectedContactPoint(frame, mirrored);
				sensor.Edit = originalID;
				sensor.Threshold = originalThreshold;
				correctionOffset = newPoint - sensor.GetContactPoint(frame, mirrored);
				return point + correctionOffset;
			}
		}else{
			return point;
		}
		*/
		//if(GetHomogeneousNoiseStatus()){
		//	point += GetStairWalkingCorrectionVector(frame, point, mirrored, sensorName);
		//	return point;
		//}else{
			RaycastHit contactRay;
			Vector3 pointMovedUp = point;
			pointMovedUp.y += 1f;
			Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			if(contactRay.transform.tag == "Tread"){
				return GetCorrectedPointWithStepNoise(root, point, mirrored);
			}else return point;
		//}
	}

	public Vector3 GetStairWalkingCorrectionVector(Frame frame, Vector3 point, bool mirrored, string sensorName){
		Sensor sensor = GetSensor(sensorName);
		Vector3 correctionOffset = Vector3.zero;
		RaycastHit contactRay;
		Vector3 pointMovedUp = point;
		pointMovedUp.y += 1f;
		Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		if(sensor.GetContact(frame, mirrored) > 0){
			if(Vector3.Distance(point, contactRay.point) < sensor.Threshold){
				//Vector3 correctionOffset = sensor.GetCorrectedContactPoint(frame, mirrored) - sensor.GetContactPoint(frame, mirrored);
				correctionOffset = GetContactCorrectionVector(frame, mirrored, sensorName);
				return correctionOffset;
			}else{
				/*if(contactRay.transform.tag == "Tread"){
					if(GetHomogeneousNoiseStatus()) return GetCorrectedPointWithEvenNoise(frame, root, point, sensorName, mirrored);
					else return GetCorrectedPointWithStepNoise(root, point, mirrored);
				}else return point;
				*/
				Sensor.ID originalID = sensor.Edit;
				float originalThreshold = sensor.Threshold;
				sensor.Edit = Sensor.ID.RayTopDown;
				sensor.Threshold = 1f;
				Vector3 newPoint = sensor.GetCorrectedContactPoint(frame, mirrored);
				sensor.Edit = originalID;
				sensor.Threshold = originalThreshold;
				correctionOffset = newPoint - sensor.GetContactPoint(frame, mirrored);
				return correctionOffset;
			}
		}else{
			return Vector3.zero;
		}
	}

	
	public Vector3 GetProjectedPoint(Vector3 point){
		RaycastHit hitRay;
		point.y += 1f;
		Physics.Raycast(point, Vector3.down, out hitRay, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
		return hitRay.point;
	}
	
	public Vector3 GetCorrectedFutureStairWalkingGoal(Frame frame, Transform root, Vector3 point, string sensorName, bool mirrored){
		if(GetHomogeneousNoiseStatus()) return GetEvenCorrectedFutureStairWalkingGoal(frame, point, mirrored, sensorName);
		else return GetCorrectedPointWithStepNoise(root, point, mirrored);
	}


	public Vector3 GetEvenCorrectedFutureStairWalkingGoal(Frame frame, Vector3 point, bool mirrored, string sensorName){
		Sensor sensor = GetSensor(sensorName);
		Vector3 correctionOffset = Vector3.zero;
		RaycastHit contactRay;
		Vector3 pointMovedUp = Vector3.zero;
		/*
		pointMovedUp.y += 1f;
		Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		return contactRay.point;
		*/
		if(sensor.GetContact(frame, mirrored) > 0){
			correctionOffset = GetContactCorrectionVector(frame, mirrored, sensorName);
			pointMovedUp = point + correctionOffset;
			pointMovedUp.y += 1f;
			Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			return contactRay.point;
		}else{
			pointMovedUp = point;
			pointMovedUp.y += 1f;
			Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			return contactRay.point;
		}
	}

	public Vector3 GetFrameCorrectionVector(Frame frame, Vector3 originalPoint, bool mirrored, string sensorName){
		Sensor sensor = GetSensor(sensorName);
		Debug.DrawLine(originalPoint, frame.GetBoneTransformation(sensorName, mirrored).GetPosition(), Color.cyan, 10f);
		return originalPoint - frame.GetBoneTransformation(sensorName, mirrored).GetPosition();
	}

	public Vector3 GetFrameYCorrectionVector(MotionEditor editor, Vector3 originalPoint, bool mirrored, string sensorName){
		Sensor sensor = GetSensor(sensorName);
		float frameYPosition = editor.GetActor().GetBoneTransformation(sensorName).GetPosition().y;
		float Ydifference = 0f;
		Vector3 YVector = Vector3.up;
		/*if(originalPoint.y > frameYPosition){
			Ydifference = originalPoint.y - frameYPosition;
			YVector *= -Ydifference;
		}else{
			Ydifference = frameYPosition - originalPoint.y;
			YVector *= Ydifference;
		}*/
		Debug.DrawLine(originalPoint, editor.GetActor().GetBoneTransformation(sensorName).GetPosition(), Color.magenta, 1f);
		Ydifference = originalPoint.y - frameYPosition;
		YVector *= -Ydifference;
		return YVector;
	}
	
	public Vector3 GetContactCorrectionVector(Frame frame, bool mirrored, string sensorName){
		Sensor sensor = GetSensor(sensorName);
		return sensor.GetCorrectedContactPoint(frame, mirrored) - sensor.GetContactPoint(frame, mirrored);
	}

	public bool GetHomogeneousNoiseStatus(Transform transform){
		return GetParentStaircase().GetComponent<NoiseStepsEvenDifferent>().UseEvenNoise;
	}

	public bool GetHomogeneousNoiseStatus(){
		return GameObject.Find("StairsNoise").GetComponent<NoiseMenu>().UseEvenNoise;
	}

	public Transform GetParentStaircase(){
		NoiseStepsEvenDifferent parentStaircase = GameObject.FindObjectOfType<NoiseStepsEvenDifferent>();
		//Debug.Log("Object = " + Stairs.name);
		//if(transform.GetComponent<NoiseStepsEvenDifferent>() != null) return transform;
		//else return GetParentStaircase(transform.parent.transform);
		return parentStaircase.transform;
	}

	public Vector3 GetCorrectedPointWithEvenNoise(Frame frame, Transform root, Vector3 point, string sensorName, bool mirrored){
		Sensor sensor = GetSensor(sensorName);
		//string treadName = sensor.GetFutureContactColliderName(frame, mirrored);
		string treadName = sensor.GetContactColliderName(frame, mirrored);
		if(treadName != "Ground"){
			GameObject tread = GameObject.Find(treadName);
			Vector3 treadRight = tread.transform.right;
			if(mirrored) treadRight *= -1f; // Adjust for mirroring
			float direction = Vector3.Dot(root.forward, treadRight);
			//Debug.Log("Direction = " + direction);
			//Debug.DrawRay(point, treadRight, Color.cyan, 1f);
			
			// Width noise
			Transform parentStaircase = GetParentStaircase();
			float randomNumber = parentStaircase.GetComponent<NoiseStepsEvenDifferent>().GetRandomNumberOfStairs();
			Vector3 scaleVector = tread.GetComponent<StepData>().DefaultScale;
			float XNoise = (tread.GetComponent<StepData>().MaxScaleNoise.x - 1f) * randomNumber;
			scaleVector.x = scaleVector.x * (1f + XNoise);
			
			//Debug.Log("direction = " + direction);
			if(direction > 0) treadRight *= -1f; // Adjust for descending
			RaycastHit[] TreadHits = Physics.RaycastAll(tread.transform.position, treadRight, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			float cummulativeDistanceOffset = 0f;
			foreach(RaycastHit hit in TreadHits){
				//Debug.Log("Hit = " + hit.collider.name);
				if(hit.collider.name != "Ground"){
					StepData hitData = hit.collider.GetComponent<StepData>();
					if(hitData.DefaultScale.x < 1f){ 
						cummulativeDistanceOffset += ((1f + XNoise) * hitData.DefaultScale.x - hitData.DefaultScale.x) / 8f;
					}
				}
			}

			float directionMultiplier = 1f;
			if(direction < 0) directionMultiplier = -1f;
			int NoiseType = parentStaircase.GetComponent<NoiseStepsEvenDifferent>().NoiseType;
			if(NoiseType == 2 || NoiseType == 3){
				cummulativeDistanceOffset += (((1f + XNoise) * tread.GetComponent<StepData>().DefaultScale.x - tread.GetComponent<StepData>().DefaultScale.x) / 2f);
				point += directionMultiplier * treadRight.normalized * cummulativeDistanceOffset;
			}

			// Height noise
			/*
			RaycastHit TreadHit;
			Vector3 pointOfsettedUp = point;
			pointOfsettedUp.y += 1f;
			Physics.Raycast(pointOfsettedUp, Vector3.down, out TreadHit, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			point = TreadHit.point;
			//point += sensor.GetPivot(frame, mirrored);
			//point.y += 0.08f;
			*/

			
			float scaledTreadHeight = 0f;
			float originalTreadHeight = 0f;

			scaledTreadHeight = tread.transform.localPosition.y;
			originalTreadHeight = tread.GetComponent<StepData>().DefaultPosition.y;
			float YTreadOffset = -Mathf.Abs(scaledTreadHeight - originalTreadHeight);
			point += YTreadOffset * root.up;
		}
		

		return point;
	}

	public Vector3 GetCorrectedPointWithRelativeContacts(Frame frame, string sensorName, bool mirrored){
		Sensor sensor = GetSensor(sensorName);
		Vector3 pointRelativeToTread = sensor.GetRelativeToTreadContactPoint(frame, mirrored);
		//if(sensorName == "LeftAnkle") Debug.Log("Relative Point = " + pointRelativeToTread);
		string treadName = sensor.GetContactColliderName(frame, mirrored);
		Matrix4x4 treadWorldMatrix = GameObject.Find(treadName).transform.GetWorldMatrix();
		Vector3 correctedPoint = pointRelativeToTread.GetRelativePositionFrom(treadWorldMatrix);
		correctedPoint.y = sensor.GetCorrectedTransformation(frame, mirrored).GetPosition().y;
		return correctedPoint;
	}

	public Vector3 GetCorrectionVectorWithRelativeContacts(Frame frame, Vector3 point, string sensorName, bool mirrored){
		Vector3 correctedPoint = GetCorrectedPointWithRelativeContacts(frame, sensorName, mirrored);
		return correctedPoint - point;
	}

	public Vector3 GetRelativeToTreadContactPoint(Frame frame, string sensorName, bool mirrored){
		Sensor sensor = GetSensor(sensorName);
		return mirrored ? sensor.InverseRelativeContactPoints[frame.Index-1] : sensor.RegularRelativeContactPoints[frame.Index-1];
	}

	public Vector3 GetRelativeToTreadFutureContactPoint(Frame frame, string sensorName, bool mirrored){
		Sensor sensor = GetSensor(sensorName);
		return mirrored ? sensor.InverseRelativeFutureGoalPoints[frame.Index-1] : sensor.RegularRelativeFutureGoalPoints[frame.Index-1];
	}

	public string GetContactColliderName(Frame frame, string sensorName, bool mirrored){
		Sensor sensor = GetSensor(sensorName);
		return mirrored ? sensor.InverseContactColliderNames[frame.Index-1] : sensor.RegularContactColliderNames[frame.Index-1];
	}

	public string GetFutureContactColliderName(Frame frame, string sensorName, bool mirrored){
		Sensor sensor = GetSensor(sensorName);
		return mirrored ? sensor.InverseFutureGoalColliderNames[frame.Index-1] : sensor.RegularFutureGoalColliderNames[frame.Index-1];
	}

	public Vector3 GetCorrectedPointWithStepNoise(Transform root, Vector3 point, bool mirrored){
		//Sensor sensor = GetSensor(SensorName);
		RaycastHit contactRay;
		float XTreadOffset = 0f;
		float scaledTreadSize = 0f;
		float originalTreadSize = 0f;
		float YTreadOffset = 0f;
		float scaledTreadHeight = 0f;
		float originalTreadHeight = 0f;
		Vector3 XDirection = Vector3.zero;
		Vector3 pointMovedUp = point;
		pointMovedUp.y += 1f;
		Physics.Raycast(pointMovedUp, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		//contactRay.transform.localScale.x < 1f
		if(contactRay.transform.tag == "Tread"){
			// Get Depth and Height Tread offsets according to Step variation
			if(contactRay.collider.gameObject.GetComponent<StepData>() == null){
				originalTreadSize = 0f;
				originalTreadHeight = 0f;
			}else{
				originalTreadSize = contactRay.collider.gameObject.GetComponent<StepData>().DefaultScale.x;
				originalTreadHeight = contactRay.collider.gameObject.GetComponent<StepData>().DefaultPosition.y;
			}
			scaledTreadSize = contactRay.transform.localScale.x;
			XTreadOffset = scaledTreadSize - originalTreadSize;
			scaledTreadHeight = contactRay.transform.localPosition.y;
			YTreadOffset = -Mathf.Abs(scaledTreadHeight - originalTreadHeight);
			
			// 
			XDirection = contactRay.collider.gameObject.transform.right;
			
			if(mirrored){
				XDirection *= -1f;	
			}
		}else{
			XTreadOffset = 0f;
			YTreadOffset = 0f;
		}

		if(contactRay.transform.localScale.x > 2f){
			XTreadOffset = 0f;
		}
		
		//Debug.DrawRay(contactRay.point, XDirection, Color.blue, 1f);
		//Debug.Log("Y offset = " + YTreadOffset.ToString("F5"));
		return point + XTreadOffset * XDirection + YTreadOffset * root.up;
	}

	protected override void DerivedDraw(MotionEditor editor) {
		UltiDraw.Begin();
		
		Frame frame = editor.GetCurrentFrame();

		Color[] colors = UltiDraw.GetRainbowColors(Sensors.Length);

		if(ShowDebug) {
			for(int i=0; i<Sensors.Length; i++) {
				if(Sensors[i].GetContact(frame, editor.Mirror) == 1f) {
					Vector3 contact = Sensors[i].GetContactPoint(frame, editor.Mirror);
					Vector3 corrected = Sensors[i].GetCorrectedContactPoint(frame, editor.Mirror);
					UltiDraw.DrawArrow(contact, corrected, 0.8f, 0.01f, DrawScale*0.025f, colors[i].Transparent(0.5f));
					UltiDraw.DrawSphere(contact, Quaternion.identity, DrawScale*0.025f, UltiDraw.Yellow);
					UltiDraw.DrawSphere(corrected, Quaternion.identity, DrawScale*0.05f, UltiDraw.Gold.Transparent(0.5f));
				}
			}
			for(int i=0; i<Sensors.Length; i++) {
				Matrix4x4 bone = frame.GetBoneTransformation(Sensors[i].Bone, editor.Mirror);
				Matrix4x4 corrected = Sensors[i].GetCorrectedTransformation(frame, editor.Mirror);
				UltiDraw.DrawCube(bone, DrawScale*0.025f, UltiDraw.DarkRed.Transparent(0.5f));
				UltiDraw.DrawLine(bone.GetPosition(), corrected.GetPosition(), colors[i].Transparent(0.5f));
				UltiDraw.DrawCube(corrected, DrawScale*0.025f, UltiDraw.DarkGreen.Transparent(0.5f));
			}
		}

		if(ShowSensors) {
			for(int i=0; i<Sensors.Length; i++) {
				Quaternion rot = editor.GetActor().GetBoneTransformation(Sensors[i].GetName()).GetRotation();
				Vector3 pos = editor.GetActor().GetBoneTransformation(Sensors[i].GetName()).GetPosition() + rot * Sensors[i].Offset;
				UltiDraw.DrawCube(pos, rot, DrawScale*0.025f, UltiDraw.Black);
				UltiDraw.DrawWireSphere(pos, rot, 2f*Sensors[i].Threshold, colors[i].Transparent(0.25f));
				if(Sensors[i].GetContact(frame, editor.Mirror) == 1f) {
					UltiDraw.DrawSphere(pos, rot, 2f*Sensors[i].Threshold, colors[i]);
				} else {
					UltiDraw.DrawSphere(pos, rot, 2f*Sensors[i].Threshold, colors[i].Transparent(0.125f));
				}
			}
		}

		if(ShowFutureGoalPoints) {
			for(int i=0; i<Sensors.Length; i++) {
				if(Sensors[i].Edit != Sensor.ID.None) {
					float currentTimeStamp = editor.GetCurrentFrame().Timestamp;
					for(float j=currentTimeStamp; j<=currentTimeStamp + 0; j+=Mathf.Max(Step, 1)/Data.Framerate) {
						Frame reference = Data.GetFrame(j);
						//Vector3 HorizontalOffset = Sensors[i].GetHorizontalStepOffsetVector(reference, editor.Mirror);
						Vector3 FutureGoalPoint = Sensors[i].GetFutureGoalPoint(reference, editor.Mirror);
						FutureGoalPoint = Sensors[i].GetRelativeToTreadFutureContactPoint(reference, editor.Mirror);
						string treadName = Sensors[i].GetFutureContactColliderName(frame, editor.Mirror);
						//if(Sensors[i].GetName() == "LeftAnkle") Debug.Log("Tread name = " + treadName);
						Matrix4x4 treadWorldMatrix = GameObject.Find(treadName).transform.GetWorldMatrix();
						FutureGoalPoint = FutureGoalPoint.GetRelativePositionFrom(treadWorldMatrix);
						//Vector3 CorrectionOffset = Sensors[i].GetCorrectedContactPoint(reference, editor.Mirror) - Sensors[i].GetContactPoint(reference, editor.Mirror);
						//FutureGoalPoint = GetCorrectedStraightStairWalkingStep(reference, editor.GetActor().GetRoot(), FutureGoalPoint, Sensors[i].GetName(), editor.Mirror);
						FutureGoalPoint = GetProjectedPoint(FutureGoalPoint);
						//FutureGoalPoint = GetCorrectedFutureStairWalkingGoal(reference, editor.GetActor().transform, FutureGoalPoint, Sensors[i].GetName(), editor.Mirror);
						//FutureGoalPoint = GetCorrectedPointWithStepNoise(editor.GetActor().GetRoot(), FutureGoalPoint, editor.Mirror);
						Debug.DrawRay(FutureGoalPoint, Sensors[i].GetFutureGoalDirection(reference, editor.Mirror), Color.red, 1f); // Printing Future direction
						//FutureGoalPoint.y = Utility.GetHeight(FutureGoalPoint, LayerMask.GetMask("Ground","Interaction")); // Adapting according to vertical noise
						//FutureGoalPoint = Sensors[i].ApplyHorizontalAndVerticalStepOffsets(reference, FutureGoalPoint, editor.Mirror);
						//UltiDraw.DrawSphere(FutureGoalPoint + HorizontalOffset, Quaternion.identity, DrawScale*0.5f, colors[i]);
						UltiDraw.DrawSphere(FutureGoalPoint, Quaternion.identity, DrawScale*0.5f, colors[i]);
						//Debug.DrawRay(FutureGoalPoint + HorizontalOffset, Sensors[i].GetRegularDirection(reference, editor.Mirror), Color.red, 1f);
						//Debug.Log("Current Goal surface = " + Sensors[i].GetContactColliderName(reference, editor.Mirror));
						//Debug.Log("Future Next Goal surface = " + Sensors[i].GetFutureContactColliderName(reference, editor.Mirror));
					}
				}
			}
		}

		if(ShowTolerances) {
			for(int i=0; i<Sensors.Length; i++) {
				Quaternion rot = editor.GetActor().GetBoneTransformation(Sensors[i].GetName()).GetRotation();
				Vector3 pos = editor.GetActor().GetBoneTransformation(Sensors[i].GetName()).GetPosition() + rot * Sensors[i].Offset;
				UltiDraw.DrawWireSphere(pos, rot, 2f*(Sensors[i].Tolerance+Sensors[i].Threshold), UltiDraw.DarkGrey.Transparent(0.25f));
			}
		}

		if(ShowContacts) {
			for(int i=0; i<Sensors.Length; i++) {
				if(Sensors[i].Edit != Sensor.ID.None) {
					for(float j=0f; j<=Data.GetTotalTime(); j+=Mathf.Max(Step, 1)/Data.Framerate) {
						Frame reference = Data.GetFrame(j);
						if(Sensors[i].GetContact(reference, editor.Mirror) == 1f) {
							UltiDraw.DrawSphere(Sensors[i].GetContactPoint(reference, editor.Mirror), Quaternion.identity, DrawScale*0.025f, colors[i]);
						}
					}
				}
			}
		}

		/*
		if(ShowSkeletons) {
			UltiDraw.End();
			float start = Mathf.Clamp(frame.Timestamp-Window, 0f, Data.GetTotalTime());
			float end = Mathf.Clamp(frame.Timestamp+Window, 0f, Data.GetTotalTime());
			float inc = Mathf.Max(SkeletonStep, 1)/Data.Framerate;
			for(float j=start; j<=end; j+=inc) {
				Frame reference = Data.GetFrame(j);
				float weight = (j-start+inc) / (end-start+inc);
				editor.GetActor().Sketch(reference.GetBoneTransformations(editor.GetActor().GetBoneNames(), editor.Mirror), Color.Lerp(UltiDraw.Cyan, UltiDraw.Magenta, weight).Transparent(weight));
			}
			UltiDraw.Begin();
		}
		*/

		if(TrueMotionTrajectory || CorrectedMotionTrajectory) {
			for(int i=0; i<Sensors.Length; i++) {
				if(Sensors[i].Edit != Sensor.ID.None) {
					Vector3 previousPos = Vector3.zero;
					Vector3 previousCorrected = Vector3.zero;
					float start = Mathf.Clamp(frame.Timestamp-PastTrajectoryWindow, 0f, Data.GetTotalTime());
					float end = Mathf.Clamp(frame.Timestamp+FutureTrajectoryWindow, 0f, Data.GetTotalTime());
					for(float j=start; j<=end; j+=Mathf.Max(Step, 1)/Data.Framerate) {
						Frame reference = Data.GetFrame(j);
						Matrix4x4 bone = reference.GetBoneTransformation(Sensors[i].Bone, editor.Mirror);
						Matrix4x4 corrected = Sensors[i].GetCorrectedTransformation(reference, editor.Mirror);
						if(j > start) {
							if(TrueMotionTrajectory) {
								UltiDraw.DrawArrow(previousPos, bone.GetPosition(), 0.8f, DrawScale*0.005f, DrawScale*0.025f, UltiDraw.DarkRed.Lighten(0.5f).Transparent(0.5f));
							}
							if(CorrectedMotionTrajectory) {
								UltiDraw.DrawArrow(previousCorrected, corrected.GetPosition(), 0.8f, DrawScale*0.005f, DrawScale*0.025f, UltiDraw.DarkGreen.Lighten(0.5f).Transparent(0.5f));
							}
							//UltiDraw.DrawLine(previousPos, bone.GetPosition(), UltiDraw.DarkRed.Transparent(0.5f));
							//UltiDraw.DrawLine(previousCorrected, corrected.GetPosition(), UltiDraw.DarkGreen.Transparent(0.5f));
						}
						previousPos = bone.GetPosition();
						previousCorrected = corrected.GetPosition();
						if(TrueMotionTrajectory) {
							UltiDraw.DrawCube(bone, DrawScale*0.025f, UltiDraw.DarkRed.Transparent(0.5f));
						}
						//UltiDraw.DrawLine(bone.GetPosition(), corrected.GetPosition(), colors[i].Transparent(0.5f));
						if(CorrectedMotionTrajectory) {
							UltiDraw.DrawCube(corrected, DrawScale*0.025f, UltiDraw.DarkGreen.Transparent(0.5f));
						}
					}
				}
			}
		}

		if(ContactTrajectories) {
			for(int i=0; i<Sensors.Length; i++) {
				if(Sensors[i].Edit != Sensor.ID.None) {
					float start = Mathf.Clamp(frame.Timestamp-Window, 0f, Data.GetTotalTime());
					float end = Mathf.Clamp(frame.Timestamp+Window, 0f, Data.GetTotalTime());
					for(float j=0f; j<=Data.GetTotalTime(); j+=Mathf.Max(Step, 1)/Data.Framerate) {
						Frame reference = Data.GetFrame(j);
						if(Sensors[i].GetContact(reference, editor.Mirror) == 1f) {
							Vector3 contact = Sensors[i].GetContactPoint(reference, editor.Mirror);
							Vector3 corrected = Sensors[i].GetCorrectedContactPoint(reference, editor.Mirror);
							UltiDraw.DrawArrow(contact, corrected, 0.8f, Vector3.Distance(contact, corrected)*DrawScale*0.025f, Vector3.Distance(contact, corrected)*DrawScale*0.1f, colors[i].Lighten(0.5f).Transparent(0.5f));
							UltiDraw.DrawSphere(contact, Quaternion.identity, DrawScale*0.0125f, colors[i].Transparent(0.5f));
							UltiDraw.DrawSphere(corrected, Quaternion.identity, DrawScale*0.05f, colors[i]);
						}
					}
				}
			}
		}

		if(ShowDistances) {
			for(int i=0; i<Sensors.Length; i++) {
				if(Sensors[i].Edit != Sensor.ID.None) {
					for(float j=frame.Timestamp-PastTrajectoryWindow; j<=frame.Timestamp+FutureTrajectoryWindow; j+=Mathf.Max(Step, 1)/Data.Framerate) {
						Frame reference = Data.GetFrame(j);
						if(Sensors[i].GetContact(reference, editor.Mirror) == 1f) {
							UltiDraw.DrawArrow(Sensors[i].GetContactPoint(reference, editor.Mirror), Sensors[i].GetContactPoint(reference, editor.Mirror) - Sensors[i].GetContactDistance(reference, editor.Mirror), 0.8f, DrawScale*0.0025f, DrawScale*0.01f, colors[i].Transparent(0.5f));
						}
					}
				}
			}
		}

		UltiDraw.End();
	}

	protected override void DerivedInspector(MotionEditor editor) {
		if(Utility.GUIButton("Capture Contacts", UltiDraw.DarkGrey, UltiDraw.White)) {
			EditorCoroutines.StartCoroutine(CaptureContacts(editor), this);
		}
		if(Utility.GUIButton("Save Velocity Values", UltiDraw.DarkGrey, UltiDraw.White)) {
			CaptureFeetVelocities(editor);
		}
		EditorGUILayout.BeginHorizontal();
		BakedContacts = (BakedContacts)EditorGUILayout.ObjectField(BakedContacts, typeof(BakedContacts), true);
		EditorGUI.BeginDisabledGroup(BakedContacts == null || editor.Mirror);
		if(Utility.GUIButton("Bake", UltiDraw.DarkGrey, UltiDraw.White)) {
			BakeContacts(editor);
		}
		EditorGUI.EndDisabledGroup();
		EditorGUILayout.EndHorizontal();
		DrawScale = EditorGUILayout.FloatField("Draw Scale", DrawScale);
		EditMotion = EditorGUILayout.Toggle("Edit Motion", EditMotion);
		ShowDebug = EditorGUILayout.Toggle("Show Debug", ShowDebug);
		ShowSensors = EditorGUILayout.Toggle("Show Sensors", ShowSensors);
		ShowTolerances = EditorGUILayout.Toggle("Show Tolerances", ShowTolerances);
		ShowDistances = EditorGUILayout.Toggle("Show Distances", ShowDistances);
		TrueMotionTrajectory = EditorGUILayout.Toggle("True Motion Trajectory", TrueMotionTrajectory);
		CorrectedMotionTrajectory = EditorGUILayout.Toggle("Corrected Motion Trajectory", CorrectedMotionTrajectory);
		ShowFutureGoalPoints = EditorGUILayout.Toggle("Show Future Goal Points", ShowFutureGoalPoints);
		useSteps = EditorGUILayout.Toggle("Use Steps Data", useSteps);
		IKTest = EditorGUILayout.Toggle("IK test", IKTest);
		PastTrajectoryWindow = EditorGUILayout.FloatField("Past Trajectory Window", PastTrajectoryWindow);
		FutureTrajectoryWindow = EditorGUILayout.FloatField("Future Trajectory Window" , FutureTrajectoryWindow);
		//ShowSkeletons = EditorGUILayout.Toggle("Show Skeletons", ShowSkeletons);
		//SkeletonStep = EditorGUILayout.IntField("Skeleton Step", SkeletonStep);
		ShowContacts = EditorGUILayout.Toggle("Show Contacts", ShowContacts);
		ContactTrajectories = EditorGUILayout.Toggle("Contact Trajectories", ContactTrajectories);
		Step = EditorGUILayout.IntField("Step", Step);
		CaptureFilter = EditorGUILayout.Slider("Capture Filter", CaptureFilter, 0f, 1f);
		EditFilter = EditorGUILayout.Slider("Edit Filter", EditFilter, 0f, 1f);
		for(int i=0; i<Sensors.Length; i++) {
			EditorGUILayout.BeginHorizontal();
			Sensors[i].Inspector(editor);
			EditorGUILayout.BeginVertical();
			if(Utility.GUIButton("-", UltiDraw.DarkRed, UltiDraw.White, 28f, 18f)) {
				RemoveSensor(Sensors[i]);
			}
			EditorGUILayout.EndVertical();
			EditorGUILayout.EndHorizontal();
		}
		if(Utility.GUIButton("+", UltiDraw.DarkGrey, UltiDraw.White)) {
			AddSensor();
		}
		if(Utility.GUIButton("Add Root Sensor", UltiDraw.DarkGrey, UltiDraw.White)) {
			AddRootSensor();
		}
	}

	
	[System.Serializable]
	public class RootSensor{

		public ContactModule Module = null;

		public string FirstTread, LastTread = " ";
		public string[] RegularContactTreads, InverseContactTreads = new string[0];
		public Vector3[] RegularLocalXContacts, InverseLocalXContacts = new Vector3[0];

		public RootSensor(ContactModule module){
			Module = module;
			if(module != null){
				FirstTread = " ";
				LastTread = " ";
				RegularContactTreads = new string[Module.Data.Frames.Length];
				InverseContactTreads = new string[Module.Data.Frames.Length];
				RegularLocalXContacts = new Vector3[Module.Data.Frames.Length];
				InverseLocalXContacts = new Vector3[Module.Data.Frames.Length];
			}
		}

		public string GetRootContactTreadName(Frame frame, bool mirrored){
			return mirrored ? InverseContactTreads[frame.Index-1] : RegularContactTreads[frame.Index-1];
		}

		public Vector3 GetRootLocalXContact(Frame frame, bool mirrored){
			return mirrored ? InverseLocalXContacts[frame.Index-1] : RegularLocalXContacts[frame.Index-1];
		}

		public string GetFirstTreadName(){
			return FirstTread;
		}

		public string GetLastTreadName(){
			return LastTread;
		}

		public void CaptureFirstTread(Frame frame, MotionEditor editor){
			Vector3 rootPos = editor.GetActor().GetRoot().position; 
			RaycastHit surfaceHit;
			rootPos.y += 1f;
			Physics.Raycast(rootPos, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(FirstTread == " " && surfaceHit.collider.name != "Ground") FirstTread = surfaceHit.collider.name;
		}

		public void CaptureLastTread(Frame frame, MotionEditor editor){
			Vector3 rootPos = editor.GetActor().GetRoot().position; 
			RaycastHit surfaceHit;
			rootPos.y += 1f;
			Physics.Raycast(rootPos, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(surfaceHit.collider.name != "Ground") LastTread = surfaceHit.collider.name;
		}

		public void CaptureTreadName(Frame frame, MotionEditor editor){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			Vector3 rootPos = editor.GetActor().GetRoot().position; 
			RaycastHit surfaceHit;
			rootPos.y += 1f;
			Physics.Raycast(rootPos, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(surfaceHit.collider == null){
				if(editor.Mirror) InverseContactTreads[frame.Index-1] = "NoContact";
				else RegularContactTreads[frame.Index-1] = "NoContact";
			}else{
				if(editor.Mirror) InverseContactTreads[frame.Index-1] = surfaceHit.collider.name;
				else RegularContactTreads[frame.Index-1] = surfaceHit.collider.name;
			}
		}

		public void CaptureLocalXContact(Frame frame, MotionEditor editor){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			Vector3 rootPos = editor.GetActor().GetRoot().position; 
			RaycastHit surfaceHit;
			rootPos.y += 1f;
			Physics.Raycast(rootPos, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(surfaceHit.collider == null){
				if(editor.Mirror) InverseLocalXContacts[frame.Index-1] = Vector3.zero;
				else RegularLocalXContacts[frame.Index-1] = Vector3.zero;
			}else{
				Matrix4x4 targetMatrix = surfaceHit.transform.GetWorldMatrix();
        		Vector3 transformedPoint = targetMatrix.inverse.MultiplyPoint(surfaceHit.point);
				if(editor.Mirror) InverseLocalXContacts[frame.Index-1] = transformedPoint;
				else RegularLocalXContacts[frame.Index-1] = transformedPoint;
			}
		}

	}

	[System.Serializable]
	public class Sensor {
		public enum ID {
			None, 
			Closest, 
			RayTopDown, RayCenterDown, RayBottomUp, RayCenterUp, 
			SphereTopDown, SphereCenterDown, SphereBottomUp, SphereCenterUp, 
			RayXPositive, RayXNegative, RayYPositive, RayYNegative, RayZPositive, RayZNegative,
			Identity
			};
		public ContactModule Module = null;
		public int Bone = 0;
		public Vector3 Offset = Vector3.zero;
		public float Threshold = 0.1f;
		public float Tolerance = 0f;
		public float Velocity = 0f;
		public bool SolvePosition = true;
		public bool SolveRotation = true;
		public bool SolveDistance = true;
		public LayerMask Mask = -1;
		public ID Capture = ID.Closest;
		public ID Edit = ID.None;
		public float Weight = 1f;

		public float[] RegularContacts, InverseContacts = new float[0];
		public Vector3[] RegularContactPoints, InverseContactPoints = new Vector3[0];
		public Vector3[] RegularDistances, InverseDistances = new Vector3[0];

		public Vector3[] RegularDirections, InverseDirections = new Vector3[0];
		public Vector3[] RegularFutureGoalPoints, InverseFutureGoalPoints = new Vector3[0];
		public Vector3[] RegularFutureGoalDirections, InverseFutureGoalDirections = new Vector3[0];

		public string[] RegularContactColliderNames, InverseContactColliderNames = new string[0];
		public string[] RegularFutureGoalColliderNames, InverseFutureGoalColliderNames = new string[0];

		public Vector3[] RegularRelativeContactPoints, InverseRelativeContactPoints = new Vector3[0];
		public Vector3[] RegularRelativeFutureGoalPoints, InverseRelativeFutureGoalPoints = new Vector3[0];

		public Sensor(ContactModule module, int bone, Vector3 offset, float threshold, float tolerance, float velocity, ID capture, ID edit) {
			Module = module;
			Bone = bone;
			Offset = offset;
			Threshold = threshold;
			Tolerance = tolerance;
			Velocity = velocity;
			Capture = capture;
			Edit = edit;
			RegularContacts = new float[Module.Data.Frames.Length];
			InverseContacts = new float[Module.Data.Frames.Length];
			RegularContactPoints = new Vector3[Module.Data.Frames.Length];
			InverseContactPoints = new Vector3[Module.Data.Frames.Length];
			RegularDistances = new Vector3[Module.Data.Frames.Length];
			InverseDistances = new Vector3[Module.Data.Frames.Length];
			RegularFutureGoalPoints = new Vector3[Module.Data.Frames.Length];
			InverseFutureGoalPoints = new Vector3[Module.Data.Frames.Length];
			RegularDirections = new Vector3[Module.Data.Frames.Length];
			InverseDirections = new Vector3[Module.Data.Frames.Length];
			RegularFutureGoalDirections = new Vector3[Module.Data.Frames.Length];
			InverseFutureGoalDirections = new Vector3[Module.Data.Frames.Length];
			RegularContactColliderNames = new string[Module.Data.Frames.Length];
			InverseContactColliderNames = new string[Module.Data.Frames.Length];
			RegularFutureGoalColliderNames = new string[Module.Data.Frames.Length];
			InverseFutureGoalColliderNames = new string[Module.Data.Frames.Length];

			//Relative Data
			RegularRelativeContactPoints = new Vector3[Module.Data.Frames.Length];
			InverseRelativeContactPoints = new Vector3[Module.Data.Frames.Length];
			RegularRelativeFutureGoalPoints = new Vector3[Module.Data.Frames.Length];
			InverseRelativeFutureGoalPoints = new Vector3[Module.Data.Frames.Length];
		}

		public string GetName() {
			return Module.Data.Source.Bones[Bone].Name;
		}

		public int GetIndex() {
			return System.Array.FindIndex(Module.Sensors, x => x==this);
		}

		public Vector3 GetPivot(Frame frame, bool mirrored) {
			return Offset.GetRelativePositionFrom(frame.GetBoneTransformation(Bone, mirrored));
		}

		public float GetContact(Frame frame, bool mirrored) {
			return mirrored ? InverseContacts[frame.Index-1] : RegularContacts[frame.Index-1];
		}

		public Vector3 GetContactDistance(Frame frame, bool mirrored) {
			return mirrored ? InverseDistances[frame.Index-1] : RegularDistances[frame.Index-1];
		}

		public Vector3 GetContactPoint(Frame frame, bool mirrored) {
			return mirrored ? InverseContactPoints[frame.Index-1] : RegularContactPoints[frame.Index-1];
		}

		public Vector3 GetFutureGoalPoint(Frame frame, bool mirrored){
			return mirrored ? InverseFutureGoalPoints[frame.Index-1] : RegularFutureGoalPoints[frame.Index-1];
		}

		public Vector3 GetRegularDirection(Frame frame, bool mirrored){
			return mirrored ? InverseDirections[frame.Index-1] : RegularDirections[frame.Index-1];
			//return Vector3.zero;
		}

		public Vector3 GetFutureGoalDirection(Frame frame, bool mirrored){
			return mirrored ? InverseFutureGoalDirections[frame.Index-1] : RegularFutureGoalDirections[frame.Index-1];
		}

		public string GetContactColliderName(Frame frame, bool mirrored){
			return mirrored ? InverseContactColliderNames[frame.Index-1] : RegularContactColliderNames[frame.Index-1];
		}

		public string GetFutureContactColliderName(Frame frame, bool mirrored){
			return mirrored ? InverseFutureGoalColliderNames[frame.Index-1] : RegularFutureGoalColliderNames[frame.Index-1];
		}

		public Vector3 GetRelativeToTreadContactPoint(Frame frame, bool mirrored){
			return mirrored ? InverseRelativeContactPoints[frame.Index-1] : RegularRelativeContactPoints[frame.Index-1];
		}

		public Vector3 GetRelativeToTreadFutureContactPoint(Frame frame, bool mirrored){
			return mirrored ? InverseRelativeFutureGoalPoints[frame.Index-1] : RegularRelativeFutureGoalPoints[frame.Index-1];
		}

		public Vector3 GetCorrectedPointWithStepNoise(Transform root, Vector3 point, bool mirrored){
		//Sensor sensor = GetSensor(SensorName);
		RaycastHit contactRay;
		float XTreadOffset = 0f;
		float scaledTreadSize = 0f;
		float originalTreadSize = 0f;
		float YTreadOffset = 0f;
		float scaledTreadHeight = 0f;
		float originalTreadHeight = 0f;
		Vector3 XDirection = Vector3.zero;
		point.y += 1f;
		Physics.Raycast(point, Vector3.down, out contactRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		if(contactRay.transform.localScale.x < 1f){
			originalTreadSize = contactRay.collider.gameObject.GetComponent<NoiseSteps>().DefaultScale.x;
			scaledTreadSize = contactRay.transform.localScale.x;
			XTreadOffset = scaledTreadSize - originalTreadSize;
			originalTreadSize = contactRay.collider.gameObject.GetComponent<NoiseSteps>().DefaultScale.y;
			scaledTreadHeight = contactRay.transform.localScale.y;
			YTreadOffset = scaledTreadHeight - originalTreadHeight;
			
			XDirection = contactRay.collider.gameObject.transform.right;
			
			/*if(Vector3.Dot(root.forward, contactRay.collider.transform.right) < 0)
				//XDirection = -1f*contactRay.collider.gameObject.transform.right;
				XDirection = contactRay.collider.gameObject.transform.right;
			else{
				XDirection = contactRay.collider.gameObject.transform.right;
			}*/
			if(mirrored){
				XDirection *= -1f;	
			}
		}else{
			XTreadOffset = 0f;
			YTreadOffset = 0f;
		}
		//Matrix4x4 correctedTransformation = GetCorrectedTransformation(frame, mirrored);
		//XDirection = contactRay.collider.gameObject.transform.right;
		Debug.DrawRay(contactRay.point, XDirection, Color.blue, 1f);
		return XTreadOffset * XDirection;
	}

		public Vector3 GetHorizontalStepOffsetVector(Frame frame, bool mirrored){
			// Offset for horizontal step noise
			// Shoot a ray and check if collided object is a tread by reading its width. If it is a tread, then we update offset accordingly.
			//Vector3 contactPoint = GetContactPoint(frame, mirrored);
			Vector3 contactPoint = GetFutureGoalPoint(frame, mirrored);
			contactPoint.y += 0.1f;
			RaycastHit contactRay;
			float XTreadOffset = 0f;
			float scaledTreadSize = 0f;
			float originalTreadSize = 0f;
			Vector3 XDirection = Vector3.zero;
			Physics.Raycast(contactPoint, Vector3.down, out contactRay, float.PositiveInfinity);
			if(contactRay.transform.localScale.x < 1f){
				if(contactRay.collider.gameObject.GetComponent<StepData>() == null){
					originalTreadSize = 0f;
				}else{
					originalTreadSize = contactRay.collider.gameObject.GetComponent<StepData>().DefaultScale.x;
				}
				scaledTreadSize = contactRay.transform.localScale.x;
				XTreadOffset = scaledTreadSize - originalTreadSize;
			}else{
				XTreadOffset = 0f;
			}
			Matrix4x4 correctedTransformation = GetCorrectedTransformation(frame, mirrored);
			if(Vector3.Dot(correctedTransformation.GetForward(), contactRay.collider.transform.right) < 0)
				XDirection = -1f*correctedTransformation.GetForward();
			else{
				XDirection = correctedTransformation.GetForward();
			}
			if(mirrored){
				XDirection *= -1f;	
			}
			
			return XTreadOffset * XDirection;
		}

		public float GetHorizontalStepOffset(Vector3 contactPoint, bool mirrored){
			// Offset for horizontal step noise
			// Shoot a ray and check if collided object is a tread by reading its width. If it is a tread, then we update offset accordingly.
			RaycastHit contactRay;
			float XTreadOffset = 0f;
			float scaledTreadSize = 0f;
			float originalTreadSize = 0f;
			//Vector3 XDirection = Vector3.zero;
			Physics.Raycast(contactPoint, Vector3.down, out contactRay, float.PositiveInfinity);
			if(contactRay.transform.localScale.x < 1f){
				originalTreadSize = contactRay.collider.gameObject.GetComponent<StepData>().DefaultScale.x;
				scaledTreadSize = contactRay.transform.localScale.x;
				XTreadOffset = scaledTreadSize - originalTreadSize;
			}else{
				XTreadOffset = 0f;
			}
			return XTreadOffset;
		}

		public Vector3 ApplyHorizontalAndVerticalStepOffsets(Frame frame, Vector3 FutureGoal, bool mirrored){
			// Offset for horizontal step noise
			// Shoot a ray and check if collided object is a tread by reading its width. If it is a tread, then we update offset accordingly.
			//contactPoint.y += 0.1f;
			//float XTreadOffset = 0f;
			//float scaledTreadSize = 0f;
			//float originalTreadSize = 0f;
			Vector3 XDirection = Vector3.zero;
			Vector3 contactPoint = GetContactPoint(frame, mirrored);
			float XTreadOffset = GetHorizontalStepOffset(contactPoint, mirrored);
			RaycastHit contactRay;
			Physics.Raycast(contactPoint, Vector3.down, out contactRay, float.PositiveInfinity);
			/*if(contactRay.transform.localScale.x < 1f){
				originalTreadSize = contactRay.collider.gameObject.GetComponent<NoiseSteps>().DefaultScale.x;
				scaledTreadSize = contactRay.transform.localScale.x;
				XTreadOffset = scaledTreadSize - originalTreadSize;
			}else{
				XTreadOffset = 0f;
			}*/
			Matrix4x4 correctedTransformation = GetCorrectedTransformation(frame, mirrored);
			if(Vector3.Dot(correctedTransformation.GetForward(), contactRay.collider.transform.right) < 0)
				XDirection = -1f*correctedTransformation.GetForward();
			else{
				XDirection = correctedTransformation.GetForward();
			}
			if(mirrored){
				XDirection *= -1f;	
			}
			FutureGoal.y = Utility.GetHeight(FutureGoal, LayerMask.GetMask("Ground","Interaction"));
			return FutureGoal + XTreadOffset * XDirection;
		}

		public Vector3 GetCorrectedContactDistance(Frame frame, bool mirrored) {
			Matrix4x4 bone = frame.GetBoneTransformation(Bone, mirrored);
			if(SolveDistance) {
				Vector3 offset = new Vector3(0f,0f,0.0f);
				return GetCorrectedContactPoint(frame, mirrored) - GetContactDistance(frame, mirrored) - bone.GetPosition();
			} else {
				return GetCorrectedContactPoint(frame, mirrored) - bone.GetRotation()*Offset - bone.GetPosition();
			}
		}

		public Vector3 GetCorrectedContactPoint(Frame frame, bool mirrored) {
			Collider collider = null;
			Vector3 point = DetectCollision(frame, mirrored, Edit, GetPivot(frame, mirrored), Tolerance+Threshold, out collider);
			if(collider != null) {
				Interaction annotated = collider.GetComponentInParent<Interaction>();
				if(annotated != null) {
					if(annotated.ContainsContact(GetName())) {
						point = annotated.GetContact(GetName(), frame, mirrored).GetPosition();
					}
					// Transform t = annotated.GetContactTransform(GetName());
					// if(t != null) {
					// 	if(mirrored) {
					// 		point = t.parent.position + t.parent.rotation * Vector3.Scale(t.parent.lossyScale.GetMirror(Module.Data.MirrorAxis), t.localPosition);
					// 	} else {
					// 		point = t.position;
					// 	}
					// }
				}
				BakedContacts baked = collider.GetComponentInParent<BakedContacts>();
				if(baked != null) {
					return baked.GetContactPoint(GetName(), frame, mirrored);
				}
			}
			return point;
		}

		public Matrix4x4 GetCorrectedTransformation(Frame frame, bool mirrored) {
			/*GameObject tread = GameObject.Find("Cube");
			//Debug.Log("Object name = " + tread);
			Vector3 treadSize = Vector3.Scale(tread.GetComponent<Transform>().localScale, tread.GetComponent<MeshFilter>().mesh.bounds.size);
			Debug.Log("Tread depth = " + treadSize[2].ToString("F2"));
			*/
			Matrix4x4 bone = frame.GetBoneTransformation(Bone, mirrored);
			if(Edit == ID.None) {
				return bone;
			}
			if(Edit == ID.Identity) {
				return bone;
			}
			if(GetContact(frame, mirrored) == 1f) {
				//Gaussian smoothing filter along contact points
				int width = Mathf.RoundToInt(Module.EditFilter * Module.Data.Framerate);
				bool[] contacts = new bool[2*width + 1];
				Vector3[] distances = new Vector3[2*width + 1];
				contacts[width] = true;
				distances[width] = GetCorrectedContactDistance(frame, mirrored);
				for(int i=1; i<=width; i++) {
					int left = frame.Index - i;
					int right = frame.Index + i;
					if(left > 1 && right <= Module.Data.GetTotalFrames()) {
						if(GetContact(Module.Data.GetFrame(left), mirrored) == 1f && GetContact(Module.Data.GetFrame(right), mirrored) == 1f) {
							contacts[width-i] = true;
							contacts[width+i] = true;
							distances[width-i] = GetCorrectedContactDistance(Module.Data.GetFrame(left), mirrored);
							distances[width+i] = GetCorrectedContactDistance(Module.Data.GetFrame(right), mirrored);
						} else {
							break;
						}
					} else {
						break;
					}
				}
				return Matrix4x4.TRS(bone.GetPosition() + Utility.FilterGaussian(distances, contacts), bone.GetRotation(), Vector3.one);
			} else {
				//Interpolation between ground truth and contact points
				float min = Mathf.Clamp(frame.Timestamp-Module.Window, 0f, Module.Data.GetTotalTime());
				float max = Mathf.Clamp(frame.Timestamp+Module.Window, 0f, Module.Data.GetTotalTime());
				Frame start = null;
				Frame end = null;
				for(float j=frame.Timestamp; j>=min; j-=1f/Module.Data.Framerate) {
					Frame reference = Module.Data.GetFrame(j);
					if(GetContact(reference, mirrored) == 1f) {
						start = reference;
						break;
					}
				}
				for(float j=frame.Timestamp; j<=max; j+=1f/Module.Data.Framerate) {
					Frame reference = Module.Data.GetFrame(j);
					if(GetContact(reference, mirrored) == 1f) {
						end = reference;
						break;
					}
				}
				if(start != null && end == null) {
					float weight = 1f - (frame.Timestamp - start.Timestamp) / (frame.Timestamp - min);
					return Matrix4x4.TRS(bone.GetPosition() + weight*GetCorrectedContactDistance(start, mirrored), bone.GetRotation(), Vector3.one);
				}
				if(start == null && end != null) {
					float weight = 1f - (end.Timestamp - frame.Timestamp) / (max - frame.Timestamp);
					return Matrix4x4.TRS(bone.GetPosition() + weight*GetCorrectedContactDistance(end, mirrored), bone.GetRotation(), Vector3.one);
				}
				if(start != null && end != null) {
					float weight = (frame.Timestamp - start.Timestamp) / (end.Timestamp - start.Timestamp);
					return Matrix4x4.TRS(
						bone.GetPosition() + Vector3.Lerp(GetCorrectedContactDistance(start, mirrored), GetCorrectedContactDistance(end, mirrored), weight), 
						bone.GetRotation(), 
						Vector3.one
					);
				}
				return bone;
			}
		}

		public Matrix4x4 GetCorrectedTransformation(Frame frame, bool mirrored, bool treadDepth) {
			GameObject tread = GameObject.Find("tread");
			//Debug.Log("Object name = " + tread);
			Vector3 treadSize = Vector3.Scale(tread.GetComponent<Transform>().localScale, tread.GetComponent<MeshFilter>().mesh.bounds.size);
			Debug.Log("Tread depth = " + treadSize[2].ToString("F2"));
			Matrix4x4 bone = frame.GetBoneTransformation(Bone, mirrored);
			if(Edit == ID.None) {
				return bone;
			}
			if(Edit == ID.Identity) {
				return bone;
			}
			if(GetContact(frame, mirrored) == 1f) {
				//Gaussian smoothing filter along contact points
				int width = Mathf.RoundToInt(Module.EditFilter * Module.Data.Framerate);
				bool[] contacts = new bool[2*width + 1];
				Vector3[] distances = new Vector3[2*width + 1];
				contacts[width] = true;
				distances[width] = GetCorrectedContactDistance(frame, mirrored);
				for(int i=1; i<=width; i++) {
					int left = frame.Index - i;
					int right = frame.Index + i;
					if(left > 1 && right <= Module.Data.GetTotalFrames()) {
						if(GetContact(Module.Data.GetFrame(left), mirrored) == 1f && GetContact(Module.Data.GetFrame(right), mirrored) == 1f) {
							contacts[width-i] = true;
							contacts[width+i] = true;
							distances[width-i] = GetCorrectedContactDistance(Module.Data.GetFrame(left), mirrored);
							distances[width+i] = GetCorrectedContactDistance(Module.Data.GetFrame(right), mirrored);
						} else {
							break;
						}
					} else {
						break;
					}
				}
				return Matrix4x4.TRS(bone.GetPosition() + Utility.FilterGaussian(distances, contacts), bone.GetRotation(), Vector3.one);
			} else {
				//Interpolation between ground truth and contact points
				float min = Mathf.Clamp(frame.Timestamp-Module.Window, 0f, Module.Data.GetTotalTime());
				float max = Mathf.Clamp(frame.Timestamp+Module.Window, 0f, Module.Data.GetTotalTime());
				Frame start = null;
				Frame end = null;
				for(float j=frame.Timestamp; j>=min; j-=1f/Module.Data.Framerate) {
					Frame reference = Module.Data.GetFrame(j);
					if(GetContact(reference, mirrored) == 1f) {
						start = reference;
						break;
					}
				}
				for(float j=frame.Timestamp; j<=max; j+=1f/Module.Data.Framerate) {
					Frame reference = Module.Data.GetFrame(j);
					if(GetContact(reference, mirrored) == 1f) {
						end = reference;
						break;
					}
				}
				if(start != null && end == null) {
					float weight = 1f - (frame.Timestamp - start.Timestamp) / (frame.Timestamp - min);
					return Matrix4x4.TRS(bone.GetPosition() + weight*GetCorrectedContactDistance(start, mirrored), bone.GetRotation(), Vector3.one);
				}
				if(start == null && end != null) {
					float weight = 1f - (end.Timestamp - frame.Timestamp) / (max - frame.Timestamp);
					return Matrix4x4.TRS(bone.GetPosition() + weight*GetCorrectedContactDistance(end, mirrored), bone.GetRotation(), Vector3.one);
				}
				if(start != null && end != null) {
					float weight = (frame.Timestamp - start.Timestamp) / (end.Timestamp - start.Timestamp);
					return Matrix4x4.TRS(
						bone.GetPosition() + Vector3.Lerp(GetCorrectedContactDistance(start, mirrored), GetCorrectedContactDistance(end, mirrored), weight), 
						bone.GetRotation(), 
						Vector3.one
					);
				}
				return bone;
			}
		}
		
		public Vector3 DetectCollision(Frame frame, bool mirrored, Sensor.ID mode, Vector3 pivot, float radius, out Collider collider) {
			if(mode == ID.Closest) {
				return Utility.GetClosestPointOverlapSphere(pivot, radius, Mask, out collider);
			}

			if(mode == ID.RayTopDown) {
				RaycastHit info;
				bool hit = Physics.Raycast(pivot + new Vector3(0f, radius, 0f), Vector3.down, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayCenterDown) {
				RaycastHit info;
				bool hit = Physics.Raycast(pivot, Vector3.down, out info, radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayBottomUp) {
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - new Vector3(0f, radius, 0f), Vector3.up, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayCenterUp) {
				RaycastHit info;
				bool hit = Physics.Raycast(pivot, Vector3.up, out info, radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.SphereTopDown) {
				RaycastHit info;
				bool hit = Physics.SphereCast(pivot + new Vector3(0f, radius+Threshold, 0f), Threshold, Vector3.down, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.SphereCenterDown) {
				RaycastHit info;
				bool hit = Physics.SphereCast(pivot + new Vector3(0f, radius, 0f), Threshold, Vector3.down, out info, radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.SphereBottomUp) {
				RaycastHit info;
				bool hit = Physics.SphereCast(pivot - new Vector3(0f, radius+Threshold, 0f), Threshold, Vector3.up, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.SphereCenterUp) {
				RaycastHit info;
				bool hit = Physics.SphereCast(pivot - new Vector3(0f, radius, 0f), Threshold, Vector3.up, out info, radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayXPositive) {
				Vector3 dir = frame.GetBoneTransformation(Bone, mirrored).GetRight();
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - radius*dir, dir, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayXNegative) {
				Vector3 dir = -frame.GetBoneTransformation(Bone, mirrored).GetRight();
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - radius*dir, dir, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayYPositive) {
				Vector3 dir = frame.GetBoneTransformation(Bone, mirrored).GetUp();
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - radius*dir, dir, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayYNegative) {
				Vector3 dir = -frame.GetBoneTransformation(Bone, mirrored).GetUp();
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - radius*dir, dir, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayZPositive) {
				Vector3 dir = frame.GetBoneTransformation(Bone, mirrored).GetForward();
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - radius*dir, dir, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			if(mode == ID.RayZNegative) {
				Vector3 dir = -frame.GetBoneTransformation(Bone, mirrored).GetForward();
				RaycastHit info;
				bool hit = Physics.Raycast(pivot - radius*dir, dir, out info, 2f*radius, Mask);
				if(hit) {
					collider = info.collider;
					return info.point;
				}
			}

			collider = null;
			return pivot;
		}

		//TODO: FilterGaussian here has problems at the boundary of the file since the pivot point is not centered.
		public void CaptureContact(Frame frame, MotionEditor editor) {
			int width = Mathf.RoundToInt(Module.CaptureFilter * Module.Data.Framerate);
			Frame[] frames = Module.Data.GetFrames(Mathf.Clamp(frame.Index-width, 1, Module.Data.GetTotalFrames()), Mathf.Clamp(frame.Index+width, 1, Module.Data.GetTotalFrames()));
			{
				bool[] contacts = new bool[frames.Length];
				Vector3[] contactPoints = new Vector3[frames.Length];
				Vector3[] distances = new Vector3[frames.Length];
				for(int i=0; i<frames.Length; i++) {
					Frame f = frames[i];
					Vector3 bone = editor.Mirror ? f.GetBoneTransformation(Bone, false).GetPosition().GetMirror(f.Data.MirrorAxis) : f.GetBoneTransformation(Bone, false).GetPosition();
					Vector3 pivot = editor.Mirror ? GetPivot(f, false).GetMirror(f.Data.MirrorAxis) : GetPivot(f, false);
					Collider collider;
					Vector3 collision = DetectCollision(frame, false, Capture, pivot, Threshold, out collider);
					contacts[i] = collider != null;
					if(collider != null) {
						Vector3 distance = collision - bone;
						contactPoints[i] = editor.Mirror ? collision.GetMirror(f.Data.MirrorAxis) : collision;
						distances[i] = editor.Mirror ? distance.GetMirror(f.Data.MirrorAxis) : distance;
					}
				}
				bool hit = Utility.GetMostCommonItem(contacts);

				if(hit) {
					RegularContacts[frame.Index-1] = 1f;
					RegularDistances[frame.Index-1] = Utility.GetMostCenteredVector(distances, contacts);
					RegularContactPoints[frame.Index-1] = Utility.GetMostCenteredVector(contactPoints, contacts);
					//Vector3 pivot = editor.Mirror ? GetPivot(frame, false).GetMirror(frame.Data.MirrorAxis) : GetPivot(frame, false);
					//CaptureColliderName(frame, editor, pivot);

				} else {
					RegularContacts[frame.Index-1] = 0f;
					RegularDistances[frame.Index-1] = Vector3.zero;
					RegularContactPoints[frame.Index-1] = Vector3.zero;
				}				
			}
			{
				bool[] contacts = new bool[frames.Length];
				Vector3[] distances = new Vector3[frames.Length];
				Vector3[] contactPoints = new Vector3[frames.Length];
				for(int i=0; i<frames.Length; i++) {
					Frame f = frames[i];
					Vector3 bone = editor.Mirror ? f.GetBoneTransformation(Bone, true).GetPosition() : f.GetBoneTransformation(Bone, true).GetPosition().GetMirror(f.Data.MirrorAxis);
					Vector3 pivot = editor.Mirror ? GetPivot(f, true) : GetPivot(f, true).GetMirror(f.Data.MirrorAxis);
					Collider collider;
					Vector3 collision = DetectCollision(frame, true, Capture, pivot, Threshold, out collider);
					contacts[i] = collider != null;
					if(collider != null) {
						Vector3 distance = collision - bone;
						distances[i] = editor.Mirror ? distance : distance.GetMirror(f.Data.MirrorAxis);
						contactPoints[i] = editor.Mirror ? collision : collision.GetMirror(f.Data.MirrorAxis);
					}
				}
				bool hit = Utility.GetMostCommonItem(contacts);
				if(hit) {
					InverseContacts[frame.Index-1] = 1f;
					InverseDistances[frame.Index-1] = Utility.GetMostCenteredVector(distances, contacts);
					InverseContactPoints[frame.Index-1] = Utility.GetMostCenteredVector(contactPoints, contacts);
					//Vector3 pivot = editor.Mirror ? GetPivot(frame, true) : GetPivot(frame, true).GetMirror(frame.Data.MirrorAxis);
					//CaptureColliderName(frame, editor, pivot);
				} else {
					InverseContacts[frame.Index-1] = 0f;
					InverseDistances[frame.Index-1] = Vector3.zero;
					InverseContactPoints[frame.Index-1] = Vector3.zero;
				}
			}
			if(Velocity > 0f) {
				if(GetContact(frame, false) == 1f) {
					if(frame.GetBoneVelocity(Bone, false, 1f/Module.Data.Framerate).magnitude > Velocity) {
						RegularContacts[frame.Index-1] = 0f;
						RegularContactPoints[frame.Index-1] = GetPivot(frame, false);
						RegularDistances[frame.Index-1] = Vector3.zero;
					}
				}
				if(GetContact(frame, true) == 1f) {
					if(frame.GetBoneVelocity(Bone, true, 1f/Module.Data.Framerate).magnitude > Velocity) {
						InverseContacts[frame.Index-1] = 0f;
						InverseContactPoints[frame.Index-1] = GetPivot(frame, true);
						InverseDistances[frame.Index-1] = Vector3.zero;
					}
				}
			}
		}

		public void CaptureColliderName(Frame frame, MotionEditor editor, Vector3 pivot){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			RaycastHit surfaceHit;
			pivot.y += 1f;
			Physics.Raycast(pivot, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(pivot ==  null || surfaceHit.collider == null){
				if(editor.Mirror) InverseContactColliderNames[frame.Index-1] = "NoContact";
				else RegularContactColliderNames[frame.Index-1] = "NoContact";
			}else{
				if(editor.Mirror) InverseContactColliderNames[frame.Index-1] = surfaceHit.collider.name;
				else RegularContactColliderNames[frame.Index-1] = surfaceHit.collider.name;
			}
		}

		public void CaptureFutureColliderName(Frame frame, MotionEditor editor, Vector3 pivot){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			RaycastHit surfaceHit;
			pivot.y += 1f;
			Physics.Raycast(pivot, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(pivot ==  null || surfaceHit.collider == null){
				if(editor.Mirror) InverseFutureGoalColliderNames[frame.Index-1] = "NoContact";
				else RegularFutureGoalColliderNames[frame.Index-1] = "NoContact";
			}else{
				if(editor.Mirror) InverseFutureGoalColliderNames[frame.Index-1] = surfaceHit.collider.name;
				else RegularFutureGoalColliderNames[frame.Index-1] = surfaceHit.collider.name;
			}
		}

		public void CaptureRelativeToTreadContactPoints(Frame frame, MotionEditor editor, Vector3 pivot){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			RaycastHit surfaceHit;
			pivot.y += 1f;
			Physics.Raycast(pivot, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(pivot ==  null || surfaceHit.collider == null){
				if(editor.Mirror) InverseRelativeContactPoints[frame.Index-1] = Vector3.zero;
				else RegularRelativeContactPoints[frame.Index-1] = Vector3.zero;
			}else{
				Matrix4x4 surfaceWorldMatrix = surfaceHit.transform.GetWorldMatrix();
				if(editor.Mirror) InverseRelativeContactPoints[frame.Index-1] = surfaceHit.point.GetRelativePositionTo(surfaceWorldMatrix);
				else RegularRelativeContactPoints[frame.Index-1] = surfaceHit.point.GetRelativePositionTo(surfaceWorldMatrix);
			}
		}

		public void CaptureRelativeToTreadFutureGoalPoints(Frame frame, MotionEditor editor, Vector3 pivot){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			RaycastHit surfaceHit;
			pivot.y += 1f;
			Physics.Raycast(pivot, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(pivot ==  null || surfaceHit.collider == null){
				if(editor.Mirror) InverseRelativeFutureGoalPoints[frame.Index-1] = Vector3.zero;
				else RegularRelativeFutureGoalPoints[frame.Index-1] = Vector3.zero;
			}else{
				Matrix4x4 surfaceWorldMatrix = surfaceHit.transform.GetWorldMatrix();
				if(editor.Mirror) InverseRelativeFutureGoalPoints[frame.Index-1] = surfaceHit.point.GetRelativePositionTo(surfaceWorldMatrix);
				else RegularRelativeFutureGoalPoints[frame.Index-1] = surfaceHit.point.GetRelativePositionTo(surfaceWorldMatrix);
			}
		}

		/*public void CaptureTreadName(Frame frame, MotionEditor editor){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			Vector3 rootPos = editor.GetActor().GetRoot().position; 
			RaycastHit surfaceHit;
			rootPos.y += 1f;
			Physics.Raycast(rootPos, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(surfaceHit.collider == null){
				if(editor.Mirror) RootInverseContactTreads[frame.Index-1] = "NoContact";
				else RootRegularContactTreads[frame.Index-1] = "NoContact";
			}else{
				if(editor.Mirror) RootInverseContactTreads[frame.Index-1] = surfaceHit.collider.name;
				else RootRegularContactTreads[frame.Index-1] = surfaceHit.collider.name;
			}
		}*/

		/*public void CaptureLocalXContact(Frame frame, MotionEditor editor){
			//Vector3 origin = editor.GetActor().GetBoneTransformation(GetName()).GetPosition();
			Vector3 rootPos = editor.GetActor().GetRoot().position; 
			RaycastHit surfaceHit;
			rootPos.y += 1f;
			Physics.Raycast(rootPos, Vector3.down, out surfaceHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
			if(surfaceHit.collider == null){
				if(editor.Mirror) RootInverseLocalXContacts[frame.Index-1] = 0f;
				else RootRegularLocalXContacts[frame.Index-1] = 0f;
			}else{
				Matrix4x4 targetMatrix = surfaceHit.transform.GetWorldMatrix();
        		Vector3 transformedPoint = targetMatrix.inverse.MultiplyPoint(surfaceHit.point);
				if(editor.Mirror) RootInverseLocalXContacts[frame.Index-1] = transformedPoint.x;
				else RootRegularLocalXContacts[frame.Index-1] = transformedPoint.x;
			}
		}*/

		public void CaptureDirections(Frame f, MotionEditor editor){
			//Using bone directions as contact point directions
			{
				Matrix4x4 Root = editor.GetActor().GetRoot().GetWorldMatrix(true);
				//Debug.Log("LeftAnkle, LelfToe, RightAnkle RightToe = " + editor.GetActor().GetBoneIndices("LeftAnkle","LeftToe","RightAnkle","RightToe"));
				//Debug.Log("Bone, Bone +1 = " + Bone + (Bone+1));
				//Debug.Log("Sensor name = " + this.GetName());
				//Vector3 direction = editor.Mirror ? f.GetBoneTransformation(Bone + 1, false).GetPosition().GetMirror(f.Data.MirrorAxis) - f.GetBoneTransformation(Bone, false).GetPosition().GetMirror(f.Data.MirrorAxis) : 
				//									f.GetBoneTransformation(Bone + 1, false).GetPosition() - f.GetBoneTransformation(Bone, false).GetPosition();
				//Vector3 direction = editor.Mirror ? f.GetBoneTransformation(Bone, false).GetForward() : 
				//									f.GetBoneTransformation(Bone, false).GetForward().GetMirror(Axis.ZPositive);
				Vector3 direction = Vector3.zero;
				//TimeSeries TimeSeries = ((TimeSeriesModule)editor.GetData().GetModule(Module.ID.TimeSeries).GetTimeSeries(f, editor.Mirror, 6, 6, 1f, 1f, 1, 1f/editor.TargetFramerate);
				if(editor.Mirror){
					direction = f.GetBoneTransformation(Bone, false).GetForward().GetMirror(f.Data.MirrorAxis);
					//direction = f.GetBoneVelocity(Bone, editor.Mirror, 1).normalized;
					//direction = f.GetBoneTransformation(Bone, false).GetForward();
					if(Bone == 25){
						direction = editor.GetActor().FindTransform("LeftAnkle").forward;
					}
					if(Bone == 20){
						direction = editor.GetActor().FindTransform("RightAnkle").forward;
					}
					direction.y = 0;
					InverseDirections[f.Index-1] = direction;
				}else{
					direction = f.GetBoneTransformation(Bone, false).GetForward();
					//direction = f.GetBoneVelocity(Bone, editor.Mirror, 1).normalized;
					if(Bone == 25){
						direction = editor.GetActor().FindTransform("LeftAnkle").forward;
					}
					if(Bone == 20){
						direction = editor.GetActor().FindTransform("RightAnkle").forward;
					}
					direction.y = 0;
					RegularDirections[f.Index-1] = direction;
				}
				//Vector3 position = editor.Mirror ? f.GetBoneTransformation(Bone, false).GetPosition().GetMirror(f.Data.MirrorAxis).GetRelativePositionTo(Root) : f.GetBoneTransformation(Bone, false).GetPosition().GetRelativePositionTo(Root);
				//Debug.DrawLine(position, direction, Color.red, 1f);
			}
			/*{
				Matrix4x4 Root = editor.GetActor().GetRoot().GetWorldMatrix(true);
				//Debug.Log("Bone, Bone +1 = " + Bone + (Bone+1));
				//Debug.Log("Sensor name = " + this.GetName());
				//Vector3 direction = editor.Mirror ? f.GetBoneTransformation(Bone + 1, false).GetPosition() - f.GetBoneTransformation(Bone, false).GetPosition() : 
				//									f.GetBoneTransformation(Bone + 1, false).GetPosition().GetMirror(f.Data.MirrorAxis) - f.GetBoneTransformation(Bone, false).GetPosition().GetMirror(f.Data.MirrorAxis);
				Vector3 direction = editor.Mirror ? f.GetBoneTransformation(Bone, false).GetForward().GetMirror(Axis.XPositive) : 
													f.GetBoneTransformation(Bone, false).GetForward();
				//direction = direction.GetMirror(Axis.XPositive);
				direction.y = 0;
				InverseDirections[f.Index-1] = direction;
				//Vector3 position = editor.Mirror ? f.GetBoneTransformation(Bone, false).GetPosition().GetRelativePositionTo(Root) : f.GetBoneTransformation(Bone, false).GetPosition().GetMirror(f.Data.MirrorAxis).GetRelativePositionTo(Root);
				//Debug.DrawLine(position, direction, Color.red, 1f);
			}*/

		}

		public void CaptureFutureGoalPoints(Frame frame, float FutureTrajectoryWindow, int Framerate){
			//Check for contact point in future window, if no contact then while backwards until find a contact
			int start = 0;
			FutureTrajectoryWindow /= 30f;
			int end = Mathf.RoundToInt(FutureTrajectoryWindow * Framerate);
			if(frame.Index + end >= RegularContacts.Length){
				end = RegularContacts.Length - frame.Index;
			}
			//Use ints in window to check if contact is done
			for(int j=end - 1; j>start; j-=1){
				if(RegularContacts[frame.Index+j] != 0f){
					RegularFutureGoalPoints[frame.Index-1] = RegularContactPoints[frame.Index+j];
					RegularFutureGoalDirections[frame.Index-1] = RegularDirections[frame.Index+j];
					RegularFutureGoalColliderNames[frame.Index-1] = RegularContactColliderNames[frame.Index+j];
					RegularRelativeFutureGoalPoints[frame.Index-1] = RegularRelativeFutureGoalPoints[frame.Index+j];
					break;
				}
				/*if(j==start+1){
					RegularFutureGoalPoints[frame.Index-1] = RegularFutureGoalPoints[frame.Index-1];
					RegularFutureGoalDirections[frame.Index-1] = RegularFutureGoalDirections[frame.Index-1];
				}*/
			}
			//}
			for(int j=end - 1; j>start; j-=1){
				if(InverseContacts[frame.Index+j] != 0f){
					InverseFutureGoalPoints[frame.Index-1] = InverseContactPoints[frame.Index+j];
					InverseFutureGoalDirections[frame.Index-1] = InverseDirections[frame.Index+j];
					InverseFutureGoalColliderNames[frame.Index-1] = InverseContactColliderNames[frame.Index+j];
					InverseRelativeFutureGoalPoints[frame.Index-1] = InverseRelativeFutureGoalPoints[frame.Index+j];
					break;
				}
				/*if(j==start+1){
					InverseFutureGoalPoints[frame.Index-1] = InverseFutureGoalPoints[frame.Index-1];
					InverseFutureGoalDirections[frame.Index-1] = InverseFutureGoalDirections[frame.Index-1];
				}*/
			}
		}

		public void FillInZeros(int framesLength){
			for(int i = framesLength - 1; i >= 0; i--){
				if(RegularFutureGoalPoints[i] == Vector3.zero && i != framesLength-1){
					RegularFutureGoalPoints[i] = RegularFutureGoalPoints[i+1];
					RegularFutureGoalDirections[i] = RegularFutureGoalDirections[i+1];
					//RegularContactColliderNames[i] = RegularContactColliderNames[i+1];
					RegularFutureGoalColliderNames[i] = RegularFutureGoalColliderNames[i+1];
					RegularRelativeFutureGoalPoints[i] = RegularRelativeFutureGoalPoints[i+1];
				}
				if(InverseFutureGoalPoints[i] == Vector3.zero && i != framesLength-1){
					InverseFutureGoalPoints[i] = InverseFutureGoalPoints[i+1];
					InverseFutureGoalDirections[i] = InverseFutureGoalDirections[i+1];
					//InverseContactColliderNames[i] = InverseContactColliderNames[i+1];
					InverseFutureGoalColliderNames[i] = InverseFutureGoalColliderNames[i+1];
					InverseRelativeFutureGoalPoints[i] = InverseRelativeFutureGoalPoints[i+1];
				}
			}
		}

		public void Inspector(MotionEditor editor) {
			UltiDraw.Begin();
			Utility.SetGUIColor(UltiDraw.Grey);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				Utility.ResetGUIColor();

				EditorGUILayout.BeginHorizontal();
				EditorGUILayout.LabelField("Bone", GUILayout.Width(40f));
				Bone = EditorGUILayout.Popup(Bone, editor.GetData().Source.GetBoneNames(), GUILayout.Width(80f));
				EditorGUILayout.LabelField("Mask", GUILayout.Width(30));
				Mask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(EditorGUILayout.MaskField(InternalEditorUtility.LayerMaskToConcatenatedLayersMask(Mask), InternalEditorUtility.layers, GUILayout.Width(75f)));
				EditorGUILayout.LabelField("Capture", GUILayout.Width(50));
				Capture = (ID)EditorGUILayout.EnumPopup(Capture, GUILayout.Width(75f));
				EditorGUILayout.LabelField("Edit", GUILayout.Width(30));
				Edit = (ID)EditorGUILayout.EnumPopup(Edit, GUILayout.Width(75f));
				EditorGUILayout.LabelField("Solve Position", GUILayout.Width(80f));
				SolvePosition = EditorGUILayout.Toggle(SolvePosition, GUILayout.Width(20f));
				EditorGUILayout.LabelField("Solve Rotation", GUILayout.Width(80f));
				SolveRotation = EditorGUILayout.Toggle(SolveRotation, GUILayout.Width(20f));
				EditorGUILayout.LabelField("Solve Distance", GUILayout.Width(80f));
				SolveDistance = EditorGUILayout.Toggle(SolveDistance, GUILayout.Width(20f));
				EditorGUILayout.EndHorizontal();

				EditorGUILayout.BeginHorizontal();
				EditorGUILayout.LabelField("Offset", GUILayout.Width(40f));
				Offset = EditorGUILayout.Vector3Field("", Offset, GUILayout.Width(180f));
				EditorGUILayout.LabelField("Threshold", GUILayout.Width(70f));
				Threshold = EditorGUILayout.FloatField(Threshold, GUILayout.Width(50f));
				EditorGUILayout.LabelField("Tolerance", GUILayout.Width(70f));
				Tolerance = EditorGUILayout.FloatField(Tolerance, GUILayout.Width(50f));
				EditorGUILayout.LabelField("Velocity", GUILayout.Width(70f));
				Velocity = EditorGUILayout.FloatField(Velocity, GUILayout.Width(50f));
				EditorGUILayout.LabelField("Weight", GUILayout.Width(60f));
				Weight = EditorGUILayout.FloatField(Weight, GUILayout.Width(50f));
				EditorGUILayout.EndHorizontal();

				Frame frame = editor.GetCurrentFrame();
				MotionData data = editor.GetData();

				EditorGUILayout.BeginVertical(GUILayout.Height(10f));
				Rect ctrl = EditorGUILayout.GetControlRect();
				Rect rect = new Rect(ctrl.x, ctrl.y, ctrl.width, 10f);
				EditorGUI.DrawRect(rect, UltiDraw.Black);

				float startTime = frame.Timestamp-editor.GetWindow()/2f;
				float endTime = frame.Timestamp+editor.GetWindow()/2f;
				if(startTime < 0f) {
					endTime -= startTime;
					startTime = 0f;
				}
				if(endTime > data.GetTotalTime()) {
					startTime -= endTime-data.GetTotalTime();
					endTime = data.GetTotalTime();
				}
				startTime = Mathf.Max(0f, startTime);
				endTime = Mathf.Min(data.GetTotalTime(), endTime);
				int start = data.GetFrame(startTime).Index;
				int end = data.GetFrame(endTime).Index;
				int elements = end-start;

				Vector3 bottom = new Vector3(0f, rect.yMax, 0f);
				Vector3 top = new Vector3(0f, rect.yMax - rect.height, 0f);

				start = Mathf.Clamp(start, 1, Module.Data.Frames.Length);
				end = Mathf.Clamp(end, 1, Module.Data.Frames.Length);

				//Contacts
				for(int i=start; i<=end; i++) {
					if((editor.Mirror ? InverseContacts[i-1] : RegularContacts[i-1]) == 1f) {
						float left = rect.xMin + (float)(i-start)/(float)elements * rect.width;
						float right = left;
						while(i<end && (editor.Mirror ? InverseContacts[i-1] : RegularContacts[i-1]) != 0f) {
							right = rect.xMin + (float)(i-start)/(float)elements * rect.width;
							i++;
						}
						if(left != right) {
							Vector3 a = new Vector3(left, rect.y, 0f);
							Vector3 b = new Vector3(right, rect.y, 0f);
							Vector3 c = new Vector3(left, rect.y+rect.height, 0f);
							Vector3 d = new Vector3(right, rect.y+rect.height, 0f);
							UltiDraw.DrawTriangle(a, c, b, UltiDraw.Green);
							UltiDraw.DrawTriangle(b, c, d, UltiDraw.Green);
						}
					}
				}

				//Current Pivot
				top.x = rect.xMin + (float)(frame.Index-start)/elements * rect.width;
				bottom.x = rect.xMin + (float)(frame.Index-start)/elements * rect.width;
				top.y = rect.yMax - rect.height;
				bottom.y = rect.yMax;
				UltiDraw.DrawLine(top, bottom, UltiDraw.Yellow);

				Handles.DrawLine(Vector3.zero, Vector3.zero); //Somehow needed to get it working...

				EditorGUILayout.EndVertical();
			}
			UltiDraw.End();
		}
	}

}
#endif