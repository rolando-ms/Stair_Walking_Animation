
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepLearning;
using BezierSolution;

public class StairWalking : NeuralAnimation {
	
	public bool activeSpline = true;
	public BezierSpline SplineTrajectory;
	public bool ShowBiDirectional = true;
	public bool ShowRoot = true;
	public bool ShowGoal = true;
	public bool ShowCurrent = true;
	public bool ShowPhase = false;
	public bool ShowContacts = false;
	public bool ShowEnvironment = true;
	public bool ShowInteraction = false;
	public bool ShowHeightMap = true;
	public bool ShowGUI = true;

	public float CylinderSize = 4f;
	public int CylinderResolution = 8;
	public int CylinderLayers = 8;
	public float HeightOffset = 0f;

	public bool UseHeightMap = false;
	public int HeightMapSize = 3;
	public int HeightMapResolution = 20;

	public bool UseIK = true;
	public bool ProjectRoot = true;
	public bool ProjectGoal = true;
	public bool UseSteps = false;
	public bool CorrectFootPlacement = false;
	public bool CorrectFootTrajectory = false;
	public bool CorrectAnklePosition = false;

	private Controller Controller;
	private TimeSeries TimeSeries;
	private TimeSeries.Root RootSeries;
	private TimeSeries.Feet FeetSeries;
	private TimeSeries.Style StyleSeries;
	private TimeSeries.Goal GoalSeries;
	private TimeSeries.Contact ContactSeries;
	private TimeSeries.Phase PhaseSeries;
	private CylinderMap Environment;
	private CuboidMap Geometry;
	private HeightMap EnvironmentHeight;

	/*private HeightMap RightFootProjection;//new HeightMap(size, resolution, GroundMask, bool false);
    private HeightMap LeftFootProjection;

	private Vector3 RightFootCenter = new Vector3(0f, 0f, 0f);
    private Vector3 LeftFootCenter = new Vector3(0f, 0f, 0f);*/

	private ContactModule contactModule;

	private Vector3[] PosePrediction;
	private Matrix4x4[] RootPrediction;
	private Matrix4x4[] GoalPrediction;

	private float[] Signals = new float[0];
    private float UserControl = 0f;
	private float NetworkControl = 0f;

	private float InteractionSmoothing = 0.9f;

	private bool IsInteracting = false;

	private Vector3 FutureLeftGoalStored = Vector3.zero;
	private Vector3 LeftGoalDirection = Vector3.zero;
	private Vector3 FutureRightGoalStored = Vector3.zero;
	private Vector3 RightGoalDirection = Vector3.zero;
	private bool FirstStep = true;
	private bool LeftGoalReached = true;
	private bool RightGoalReached = true;
	private bool LeftGoalClipped = false;
	private bool RightGoalClipped = false;
	private bool StepsCycle = false;
	
	private Vector3 pastLeftGoal = Vector3.zero;
	private Vector3 currentLeftGoal = Vector3.zero;
	private float leftGoalVelocity = 0f;

	private Vector3 pastRightGoal = Vector3.zero;
	private Vector3 currentRightGoal = Vector3.zero;
	private float rightGoalVelocity = 0f;

	private float walkVelocity = 0.8f;
	private float maxTreadDepth = 0.58f;

	private float stepWidthOffset = 0.1f;

	private UltimateIK.Model RightFootIK, LeftFootIK, ActorIK;

	public Controller GetController() {
		return Controller;
	}

	public TimeSeries GetTimeSeries() {
		return TimeSeries;
	}

	void StopWaiting()
    {
      Controller.waiting = false;
			var points = Controller.getCurrentPoints();
      points[0].statusMode = points[1].statusMode;
      Controller.Signal.type = points[0].statusMode;
    }

	protected override void Setup() {
		Controller = new Controller();
		Controller.spline = SplineTrajectory;
		
		Controller.Signal idle = Controller.AddSignal("Idle");
		idle.Default = true;
		idle.Velocity = 0f;
		idle.AddKey(KeyCode.W, false);
		idle.AddKey(KeyCode.A, false);
		idle.AddKey(KeyCode.S, false);
		idle.AddKey(KeyCode.D, false);
		idle.AddKey(KeyCode.Q, false);
		idle.AddKey(KeyCode.E, false);
		idle.AddKey(KeyCode.V, true);
		idle.UserControl = 0.25f;
		idle.NetworkControl = 0.1f;

		Controller.Signal walk = Controller.AddSignal("Walk");
		walk.AddKey(KeyCode.W, true);
		walk.AddKey(KeyCode.A, true);
		walk.AddKey(KeyCode.S, true);
		walk.AddKey(KeyCode.D, true);
		walk.AddKey(KeyCode.Q, true);
		walk.AddKey(KeyCode.E, true);
		walk.AddKey(KeyCode.LeftShift, false);
		walk.AddKey(KeyCode.C, false);
		walk.Velocity = walkVelocity;
		walk.UserControl = 0.25f;
		walk.NetworkControl = 0.25f;

		Environment = new CylinderMap(CylinderSize, CylinderResolution, CylinderLayers, true);
		Geometry = new CuboidMap(new Vector3Int(8, 8, 8));
		EnvironmentHeight = new HeightMap(HeightMapSize, HeightMapResolution, LayerMask.GetMask("Ground","Interaction","Default"),true);

		//RightFootProjection = new HeightMap(1f, 5, LayerMask.GetMask("Ground"), true);
        //LeftFootProjection = new HeightMap(1f, 5, LayerMask.GetMask("Ground"), true);

		TimeSeries = new TimeSeries(6, 6, 1f, 1f, 5);
		RootSeries = new TimeSeries.Root(TimeSeries);
		FeetSeries = new TimeSeries.Feet(TimeSeries);
		
		//StyleSeries = new TimeSeries.Style(TimeSeries, "Idle", "Walk", "Run", "Carry", "Open", "Sit", "Climb");
		StyleSeries = new TimeSeries.Style(TimeSeries, "Idle","Walk","Ascend","Descend");
		GoalSeries = new TimeSeries.Goal(TimeSeries, Controller.GetSignalNames());
		ContactSeries = new TimeSeries.Contact(TimeSeries, "RightAnkle", "LeftAnkle","RightWrist","LeftWrist");
		//ContactSeries = new TimeSeries.Contact(TimeSeries, "Hips", "RightWrist", "LeftWrist", "RightAnkle", "LeftAnkle");
		PhaseSeries = new TimeSeries.Phase(TimeSeries);
		for(int i=0; i<TimeSeries.Samples.Length; i++) {
			RootSeries.Transformations[i] = transform.GetWorldMatrix(true);
			
			Matrix4x4 currentRoot = RootSeries.Transformations[i];
			FeetSeries.LeftFootTransformations[i] = Actor.FindTransform("LeftAnkle").GetWorldMatrix();
			Vector3 LeftGoal = Actor.FindTransform("LeftAnkle").position;
			LeftGoal.y = Utility.GetHeight(LeftGoal, LayerMask.GetMask("Ground","Interaction","Default"));
			FeetSeries.FutureLeftFootGoalPoints[i] = LeftGoal;
			FeetSeries.FutureLeftFootGoalDirections[i] = Actor.FindTransform("LeftAnkle").forward;
			
			FeetSeries.RightFootTransformations[i] = Actor.FindTransform("RightAnkle").GetWorldMatrix();
			Vector3 RightGoal = Actor.FindTransform("RightAnkle").position;
			RightGoal.y = Utility.GetHeight(RightGoal, LayerMask.GetMask("Ground","Interaction","Default"));
			FeetSeries.FutureRightFootGoalPoints[i] = RightGoal;
			FeetSeries.FutureRightFootGoalDirections[i] = Actor.FindTransform("RightAnkle").forward;
			
			if(StyleSeries.Styles.Length > 0) {
				StyleSeries.Values[i][0] = 1f;
			}
			if(GoalSeries.Actions.Length > 0) {
				GoalSeries.Values[i][0] = 1f;
			}
			GoalSeries.Transformations[i] = transform.GetWorldMatrix(true);
			PhaseSeries.Values[i] = Mathf.Repeat((float)i/GetFramerate(), 1f);
			//Geometry.Pivot = transform.GetWorldMatrix(true);
			//Geometry.References[i] = transform.position;
		}

		PosePrediction = new Vector3[Actor.Bones.Length];
		RootPrediction = new Matrix4x4[7];
		GoalPrediction = new Matrix4x4[7];
		RightFootIK = UltimateIK.BuildModel(Actor.FindTransform("RightHip"), Actor.GetBoneTransforms(ContactSeries.Bones[0]));
		LeftFootIK = UltimateIK.BuildModel(Actor.FindTransform("LeftHip"), Actor.GetBoneTransforms(ContactSeries.Bones[1]));
		//ActorIK = UltimateIK.BuildModel(ActorIK, Actor.Bones[0].Transform, contactModule.GetObjectives(Actor));

		/*for(int i=0; i<Actor.Bones.Length; i++) {
			Actor.Bones[i].Velocity = Vector3.zero;
			Actor.Bones[i].ApplyLength();
		}*/
	}

    protected override void Feed() {
		Controller.Update();

		//Updating height w.r.t. ground
		if(ProjectRoot){
			for(int i=0; i<TimeSeries.Samples.Length; i++){
				RootSeries.Postprocess(i);
			}
			//RootSeries.Postprocess(TimeSeries.Pivot);
		}

		//Get Root
		Matrix4x4 root = RootSeries.Transformations[TimeSeries.Pivot];

		//Control Cycle
		Signals = Controller.PoolSignals();
		if (activeSpline){
			if(Controller.getStatusMode() == "Walk"){
				Signals[0] = 0;
				Signals[1] = 1;
			}else{
				Signals[0] = 1;
				Signals[1] = 0;
			}
		}
		UserControl = Controller.PoolUserControl(Signals);
		NetworkControl = Controller.PoolNetworkControl(Signals);

		if(IsInteracting) {
			//Do nothing because coroutines have control.
		} else if(Controller.QuerySignal("Sit")) {
			//StartCoroutine(Sit());
		} else if(Controller.QuerySignal("Carry")) {
			//StartCoroutine(Carry());
		} else if(Controller.QuerySignal("Open")) {
			//StartCoroutine(Open());
		} else {
			Default();
		}

		//Input Bone Positions / Velocities
		for(int i=0; i<Actor.Bones.Length; i++) {
			NeuralNetwork.Feed(Actor.Bones[i].Transform.position.GetRelativePositionTo(root));
			NeuralNetwork.Feed(Actor.Bones[i].Transform.forward.GetRelativeDirectionTo(root));
			NeuralNetwork.Feed(Actor.Bones[i].Transform.up.GetRelativeDirectionTo(root));
			NeuralNetwork.Feed(Actor.Bones[i].Velocity.GetRelativeDirectionTo(root));
		}

		//Input Trajectory Positions / Directions / Velocities / Styles
		for(int i=0; i<TimeSeries.KeyCount; i++) {
			TimeSeries.Sample sample = TimeSeries.GetKey(i);
			NeuralNetwork.FeedXZ(RootSeries.GetPosition(sample.Index).GetRelativePositionTo(root));
			NeuralNetwork.FeedXZ(RootSeries.GetDirection(sample.Index).GetRelativeDirectionTo(root));
			NeuralNetwork.Feed(StyleSeries.Values[sample.Index]);
		}

		if(UseSteps){
			//Input Feet Trajectory Positions / Directions
			for(int i=0; i<TimeSeries.KeyCount; i++) {
				TimeSeries.Sample sample = TimeSeries.GetKey(i);
				NeuralNetwork.Feed(FeetSeries.GetLeftFootPosition(sample.Index).GetRelativePositionTo(root));
				//NeuralNetwork.Feed(FeetSeries.GetLeftFootDirection(sample.Index).GetRelativeDirectionTo(root));
				NeuralNetwork.Feed(FeetSeries.GetRightFootPosition(sample.Index).GetRelativePositionTo(root));
				//NeuralNetwork.Feed(FeetSeries.GetRightFootDirection(sample.Index).GetRelativeDirectionTo(root));
			}
		}
		
		//Input Goals
		for(int i=0; i<TimeSeries.KeyCount; i++) {
			TimeSeries.Sample sample = TimeSeries.GetKey(i);
			if(ProjectGoal){
				GoalSeries.Postprocess(sample.Index); // Updating goal height w.r.t. ground
			}
			NeuralNetwork.Feed(GoalSeries.Transformations[sample.Index].GetPosition().GetRelativePositionTo(root));
			NeuralNetwork.Feed(GoalSeries.Transformations[sample.Index].GetForward().GetRelativeDirectionTo(root));
			NeuralNetwork.Feed(GoalSeries.Values[sample.Index]);
		}

		if(UseSteps){
			//Input Feet Goal Positions / Directions
			/*for(int i=0; i<TimeSeries.KeyCount; i++) {
				TimeSeries.Sample sample = TimeSeries.GetKey(i);
				NeuralNetwork.Feed(FeetSeries.GetFutureLeftFootPosition(sample.Index).GetRelativePositionTo(root));
				NeuralNetwork.FeedXZ(FeetSeries.GetFutureLeftFootDirection(sample.Index).GetRelativeDirectionTo(root));
				NeuralNetwork.Feed(FeetSeries.GetFutureRightFootPosition(sample.Index).GetRelativePositionTo(root));
				NeuralNetwork.FeedXZ(FeetSeries.GetFutureRightFootDirection(sample.Index).GetRelativeDirectionTo(root));
			}*/
			NeuralNetwork.Feed(FeetSeries.GetFutureLeftFootPosition(TimeSeries.Pivot).GetRelativePositionTo(root));
			NeuralNetwork.Feed(FeetSeries.GetFutureRightFootPosition(TimeSeries.Pivot).GetRelativePositionTo(root));
		}

		if(UseHeightMap){
			EnvironmentHeight.Sense(root);
			//LeftFootProjection.Sense(root);
        	//RightFootProjection.Sense(root);
			for(int i=0; i<EnvironmentHeight.Points.Length; i++) {
				//EnvironmentHeight.Sense(root);
				NeuralNetwork.Feed(EnvironmentHeight.Points[i].GetRelativePositionTo(root));
			}
		}else{
			//Input Environment
			//Environment.Sense(root, LayerMask.GetMask("Default", "Interaction"));
			Environment.Sense(Actor, root, LayerMask.GetMask("Default", "Interaction"), HeightOffset);
			NeuralNetwork.Feed(Environment.Occupancies);
		}

		//Input Geometry
		/*for(int i=0; i<Geometry.Points.Length; i++) {
			NeuralNetwork.Feed(Geometry.References[i].GetRelativePositionTo(root));
			NeuralNetwork.Feed(Geometry.Occupancies[i]);
		}*/

		//Setup Gating Features
		NeuralNetwork.Feed(GenerateGating());
	}

	protected override void Read() {
		//Update Past State
		for(int i=0; i<TimeSeries.Pivot; i++) {
			TimeSeries.Sample sample = TimeSeries.Samples[i];
			PhaseSeries.Values[i] = PhaseSeries.Values[i+1];
			RootSeries.SetPosition(i, RootSeries.GetPosition(i+1));
			RootSeries.SetDirection(i, RootSeries.GetDirection(i+1));
			FeetSeries.SetLeftFootPosition(i, FeetSeries.GetLeftFootPosition(i+1));
			FeetSeries.SetLeftFootDirection(i, FeetSeries.GetLeftFootDirection(i+1));
			FeetSeries.SetFutureLeftFootPosition(i, FeetSeries.GetFutureLeftFootPosition(i+1));
			FeetSeries.SetFutureLeftFootDirection(i, FeetSeries.GetFutureLeftFootDirection(i+1));
			FeetSeries.SetRightFootPosition(i, FeetSeries.GetRightFootPosition(i+1));
			FeetSeries.SetRightFootDirection(i, FeetSeries.GetRightFootDirection(i+1));
			FeetSeries.SetFutureRightFootPosition(i, FeetSeries.GetFutureRightFootPosition(i+1));
			FeetSeries.SetFutureRightFootDirection(i, FeetSeries.GetFutureRightFootDirection(i+1));
			for(int j=0; j<StyleSeries.Styles.Length; j++) {
				StyleSeries.Values[i][j] = StyleSeries.Values[i+1][j];
			}
			for(int j=0; j<ContactSeries.Bones.Length; j++) {
				ContactSeries.Values[i][j] = ContactSeries.Values[i+1][j];
			}
			GoalSeries.Transformations[i] = GoalSeries.Transformations[i+1];
			for(int j=0; j<GoalSeries.Actions.Length; j++) {
				GoalSeries.Values[i][j] = GoalSeries.Values[i+1][j];
			}
		}

		//RootSeries.Postprocess(TimeSeries.Pivot);
		//Get Root
		Matrix4x4 root = RootSeries.Transformations[TimeSeries.Pivot];

		//Read Posture
		Vector3[] positions = new Vector3[Actor.Bones.Length];
		Vector3[] forwards = new Vector3[Actor.Bones.Length];
		Vector3[] upwards = new Vector3[Actor.Bones.Length];
		Vector3[] velocities = new Vector3[Actor.Bones.Length];
		for(int i=0; i<Actor.Bones.Length; i++) {
			Vector3 position = NeuralNetwork.ReadVector3().GetRelativePositionFrom(root);
			Vector3 forward = NeuralNetwork.ReadVector3().normalized.GetRelativeDirectionFrom(root);
			Vector3 upward = NeuralNetwork.ReadVector3().normalized.GetRelativeDirectionFrom(root);
			Vector3 velocity = NeuralNetwork.ReadVector3().GetRelativeDirectionFrom(root);
			positions[i] = Vector3.Lerp(Actor.Bones[i].Transform.position + velocity / GetFramerate(), position, 0.5f);
			forwards[i] = forward;
			upwards[i] = upward;
			velocities[i] = velocity;
		}

		//Read Inverse Pose
		for(int i=0; i<Actor.Bones.Length; i++) {
			PosePrediction[i] = NeuralNetwork.ReadVector3().GetRelativePositionFrom(RootSeries.Transformations.Last());
			velocities[i] = Vector3.Lerp(velocities[i], GetFramerate() * (PosePrediction[i] - Actor.Bones[i].Transform.position), 1f/GetFramerate());
		}

		//Read Future Feet Trajectories
		if(UseSteps){
			for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++){
				TimeSeries.Sample sample = TimeSeries.GetKey(i);
				Vector3 LeftTrajectoryPosition = NeuralNetwork.ReadVector3().GetRelativePositionFrom(root);
				//Vector3 LeftTrajectoryDirection = NeuralNetwork.ReadVector3().GetRelativeDirectionFrom(root);
				FeetSeries.SetLeftFootPosition(sample.Index, LeftTrajectoryPosition);
				//FeetSeries.SetLeftFootDirection(sample.Index, LeftTrajectoryDirection);
				Vector3 RightTrajectoryPosition = NeuralNetwork.ReadVector3().GetRelativePositionFrom(root);
				//Vector3 RightTrajectoryDirections = NeuralNetwork.ReadVector3().GetRelativeDirectionFrom(root);
				FeetSeries.SetRightFootPosition(sample.Index, RightTrajectoryPosition);
				//FeetSeries.SetRightFootDirection(sample.Index, RightTrajectoryDirections);
			}
		}

		//Read Future Trajectory
		for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++) {
			TimeSeries.Sample sample = TimeSeries.GetKey(i);
			Vector3 pos = NeuralNetwork.ReadXZ().GetRelativePositionFrom(root);
			Vector3 dir = NeuralNetwork.ReadXZ().normalized.GetRelativeDirectionFrom(root);
			RootSeries.SetPosition(sample.Index, pos);
			RootSeries.SetDirection(sample.Index, dir);
			float[] styles = NeuralNetwork.Read(StyleSeries.Styles.Length);
			for(int j=0; j<styles.Length; j++) {
				styles[j] = Mathf.Clamp(styles[j], 0f, 1f);
			}
			StyleSeries.Values[sample.Index] = styles;

			RootPrediction[i-6] = Matrix4x4.TRS(pos, Quaternion.LookRotation(dir, Vector3.up), Vector3.one);
		}

		//Read Inverse Trajectory
		for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++) {
			TimeSeries.Sample sample = TimeSeries.GetKey(i);
			Matrix4x4 goal = GoalSeries.Transformations[TimeSeries.Pivot];
			//goal[1,3] = 0f;
			Vector3 pos = NeuralNetwork.ReadXZ().GetRelativePositionFrom(goal);
			Vector3 dir = NeuralNetwork.ReadXZ().normalized.GetRelativeDirectionFrom(goal);
			if(i > TimeSeries.PivotKey) {
				Matrix4x4 pivot = RootSeries.Transformations[sample.Index];
				//pivot[1,3] = 0f;
				Matrix4x4 reference = GoalSeries.Transformations[sample.Index];
				//reference[1,3] = 0f;
				float distance = Vector3.Distance(pivot.GetPosition(), reference.GetPosition());
				float weight = Mathf.Pow((float)(i-6)/7f, distance*distance);

				RootSeries.SetPosition(sample.Index, Vector3.Lerp(RootSeries.GetPosition(sample.Index), pos, weight));
				RootSeries.SetDirection(sample.Index, Vector3.Slerp(RootSeries.GetDirection(sample.Index), dir, weight));
			}
			
			GoalPrediction[i-6] = Matrix4x4.TRS(pos, Quaternion.LookRotation(dir, Vector3.up), Vector3.one);
		}
		
		//Read and Correct Goals
		for(int i=0; i<TimeSeries.KeyCount; i++) {
			float weight = TimeSeries.GetWeight1byN1(TimeSeries.GetKey(i).Index, 2f);
			TimeSeries.Sample sample = TimeSeries.GetKey(i);
			Vector3 pos = NeuralNetwork.ReadVector3().GetRelativePositionFrom(root);
			Vector3 dir = NeuralNetwork.ReadVector3().normalized.GetRelativeDirectionFrom(root);
			float[] actions = NeuralNetwork.Read(GoalSeries.Actions.Length);
			for(int j=0; j<actions.Length; j++) {
				actions[j] = Mathf.Clamp(actions[j], 0f, 1f);
			}
			GoalSeries.Transformations[sample.Index] = Utility.Interpolate(GoalSeries.Transformations[sample.Index], Matrix4x4.TRS(pos, Quaternion.LookRotation(dir, Vector3.up), Vector3.one), weight * NetworkControl);
			GoalSeries.Values[sample.Index] = Utility.Interpolate(GoalSeries.Values[sample.Index], actions, weight * NetworkControl);
		}

		RaycastHit RootPosHit;
		Vector3 RootPos = Actor.GetRoot().position;
		RootPos += Actor.GetRoot().forward.normalized * 0.3f;// * 1f;
		RootPos.y += 1f;
		//Debug.DrawRay(RootPos, -Vector3.up, Color.green);
		Physics.Raycast(RootPos, Vector3.down, out RootPosHit, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		float treadXScale = 100 * RootPosHit.transform.localScale.x;
		//Debug.Log("Dot product = " + directionWRTZFwd);
		if(RootPosHit.transform.tag == "Tread"){
			Vector3 HitForward = GetParentForwardVector(RootPosHit.transform);
			//Debug.DrawRay(Actor.GetRoot().position, HitForward, Color.cyan, 1f);
			float directionWRTTreadZFwd = Vector3.Dot(root.GetForward(), HitForward);
			//Debug.Log("direction value = " + directionWRTTreadZFwd);
			float VelocityOnStairs = 6.66f * treadXScale - 1.53f;
			//If going upstairs, reduce character's velocity
			if(directionWRTTreadZFwd > 0f){
				VelocityOnStairs -= (VelocityOnStairs * 0.25f * directionWRTTreadZFwd);
			}
			if(VelocityOnStairs > walkVelocity){
				VelocityOnStairs = walkVelocity;
			}
			//VelocityOnStairs = 0.8f;
			Controller.GetSignal("Walk").Velocity = VelocityOnStairs; //(treadXScale / maxTreadDepth) * walkVelocity;
		}else{
			Controller.GetSignal("Walk").Velocity = walkVelocity;
		}
		
		if(UseSteps){

			//DrawPlane(Actor.GetRoot().position + Actor.GetRoot().forward.normalized * treadXScale, -1 * Actor.GetRoot().forward, Color.blue);
			Plane limitStepPlane = new Plane(-1 * Actor.GetRoot().forward, Actor.GetRoot().position + Actor.GetRoot().forward.normalized * treadXScale);
			Vector3 rootOffsetLeft = Actor.GetRoot().right * stepWidthOffset;
			Vector3 rootOffsetRight = -Actor.GetRoot().right * stepWidthOffset;

			if(RootPosHit.transform.tag != "Tread"){
				treadXScale = 0.5f;
			}

			if(!CharacterMoving()){
				resetCycle();
				//FutureLeftGoalStored = Vector3.zero;
				//FutureRightGoalStored = Vector3.zero;
			}
			
			//Read Future Footstep
			float velocityThreshold = 0.0000f;
			//float stepWidthOffset = 0.1f;
			Vector3 LeftGoalPosition = NeuralNetwork.ReadVector3().GetRelativePositionFrom(root);
			LeftGoalPosition.y = Utility.GetHeight(LeftGoalPosition, LayerMask.GetMask("Ground", "Interaction"));
			LeftGoalDirection = Actor.GetRoot().forward.normalized;
			Vector3 RightGoalPosition = NeuralNetwork.ReadVector3().GetRelativePositionFrom(root);
			RightGoalPosition.y = Utility.GetHeight(RightGoalPosition, LayerMask.GetMask("Ground", "Interaction"));
			RightGoalDirection = Actor.GetRoot().forward.normalized;
			//Debug.Log("Left Goal pos = " + LeftGoalPosition);
			
			if(CorrectFootPlacement){
				currentLeftGoal = LeftGoalPosition;
				leftGoalVelocity = (currentLeftGoal - pastLeftGoal).magnitude / GetFramerate();
				//Debug.Log("Left Vel = " + leftGoalVelocity);

				float LeftGoalToPlaneDistance = limitStepPlane.GetDistanceToPoint(LeftGoalPosition);
				//Debug.Log("Left distance = " + LeftGoalToPlaneDistance);

				pastLeftGoal = currentLeftGoal;
				//if(leftGoalVelocity > velocityThreshold){
				Vector3 leftFootPos = Actor.GetBoneTransformation("LeftAnkle").GetPosition();
				/*LeftGoalPosition = ClipGoalLength(LeftGoalPosition, LeftGoalToPlaneDistance, ref LeftGoalClipped);
				// If goal not reached, future goal stays the same, update otherwise
				Debug.Log("Distance = " + Vector3.Distance(leftFootPos, LeftGoalPosition));
				if(Vector3.Distance(leftFootPos, LeftGoalPosition) < 0.15f){
					LeftGoalReached = true;
				}
				if(LeftGoalClipped){
					LeftGoalClipped = false;
					LeftGoalPosition = SetNewZValue(LeftGoalPosition);
					if(LeftGoalPosition.z == FutureRightGoalStored.z){
						LeftGoalPosition = CorrectingGoalPosition(LeftGoalPosition, treadXScale);
						LeftGoalPosition = SetNewZValue(LeftGoalPosition);
					}
				}else{
					FutureLeftGoalStored = LeftGoalPosition;
				}
				LeftGoalPosition = FutureLeftGoalStored;*/
				//LeftGoalPosition = SetValidStairStep(LeftGoalPosition, leftFootPos);
				//LeftGoalPosition = SetValidTreadStep(LeftGoalPosition, leftFootPos);
				//LeftGoalPosition = SetNewZValue(LeftGoalPosition);
				//LeftGoalPosition = SetNewGoalPosition(LeftGoalPosition); // Correcting footstep over tread
				//}

				LeftGoalPosition = ClipGoalLength(LeftGoalPosition, LeftGoalToPlaneDistance, ref LeftGoalClipped);

				// Start step cycle with left foot
				if(LeftGoalClipped && !StepsCycle){
					//LeftGoalClipped = false;
					FutureLeftGoalStored = LeftGoalPosition;//Actor.GetRoot().position + treadXScale * Actor.GetRoot().forward + stepWidthOffset * Actor.GetRoot().right;
					//FutureLeftGoalStored = SetNewZValue(FutureLeftGoalStored);
					FutureLeftGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetLeft, FutureLeftGoalStored);
					FutureRightGoalStored = Actor.GetRoot().position + treadXScale * Actor.GetRoot().forward + stepWidthOffset * -Actor.GetRoot().right;
					FutureRightGoalStored.y = Utility.GetHeight(FutureRightGoalStored, LayerMask.GetMask("Ground", "Interaction"));
					//FutureRightGoalStored = SetNewZValue(FutureRightGoalStored);
					FutureRightGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetRight, FutureRightGoalStored);
					//FutureLeftGoalStored.y == FutureRightGoalStored.y
					if(Mathf.Abs(FutureLeftGoalStored.y - FutureRightGoalStored.y) < 0.001f){
						FutureRightGoalStored = CorrectingGoalPosition(FutureRightGoalStored, treadXScale);
						//FutureRightGoalStored = SetNewZValue(FutureRightGoalStored);
						FutureRightGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetRight, FutureRightGoalStored);
					}
					LeftGoalDirection = Actor.GetRoot().forward.normalized;
					RightGoalDirection = Actor.GetRoot().forward.normalized;
					StepsCycle = true;
				}else{
					if(!StepsCycle){
						FutureLeftGoalStored = LeftGoalPosition;
					}
				}
				

				//Debug.Log("Right pos = " + RightGoalPosition);
				currentRightGoal = RightGoalPosition;
				rightGoalVelocity = (currentRightGoal - pastRightGoal).magnitude / GetFramerate();
				//Debug.Log("Left Vel = " + rightGoalVelocity);
				pastRightGoal = currentRightGoal;

				float RightGoalToPlaneDistance = limitStepPlane.GetDistanceToPoint(RightGoalPosition);
				//Debug.Log("Right distance = " + RightGoalToPlaneDistance);

				//if(rightGoalVelocity > velocityThreshold){
				Vector3 rightFootPos = Actor.GetBoneTransformation("RightAnkle").GetPosition();
				/*RightGoalPosition = ClipGoalLength(RightGoalPosition, RightGoalToPlaneDistance, ref RightGoalClipped);
				if(Vector3.Distance(rightFootPos, RightGoalPosition) < 0.15f){
					RightGoalReached = true;
				}
				if(RightGoalClipped){
					RightGoalClipped = false;
					RightGoalPosition = SetNewZValue(RightGoalPosition);
					if(RightGoalPosition.z == FutureLeftGoalStored.z){
						RightGoalPosition = CorrectingGoalPosition(RightGoalPosition, treadXScale);
						RightGoalPosition = SetNewZValue(RightGoalPosition);
					}
				}else{
					FutureRightGoalStored = RightGoalPosition;
				}
				RightGoalPosition = FutureRightGoalStored;*/
				//RightGoalPosition = SetValidStairStep(RightGoalPosition, rightFootPos);
				//RightGoalPosition = SetValidTreadStep(RightGoalPosition, rightFootPos);
				//RightGoalPosition = SetNewZValue(RightGoalPosition);
				//RightGoalPosition = SetNewGoalPosition(RightGoalPosition); // Correcting footstep over tread
				//}

				RightGoalPosition = ClipGoalLength(RightGoalPosition, RightGoalToPlaneDistance, ref RightGoalClipped);
				//if(Vector3.Distance(leftFootPos, LeftGoalPosition) < 0.15f){
				//	LeftGoalReached = true;
				//}

				// Start step cycle with Right foot
				if(RightGoalClipped && !StepsCycle){
					FutureRightGoalStored = RightGoalPosition;//Actor.GetRoot().position + treadXScale * Actor.GetRoot().forward + stepWidthOffset * -Actor.GetRoot().right;
					//FutureRightGoalStored = SetNewZValue(FutureRightGoalStored);
					FutureRightGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetRight, FutureRightGoalStored);
					FutureLeftGoalStored = Actor.GetRoot().position + treadXScale * Actor.GetRoot().forward + stepWidthOffset * Actor.GetRoot().right;
					FutureLeftGoalStored.y = Utility.GetHeight(FutureLeftGoalStored, LayerMask.GetMask("Ground", "Interaction"));
					//FutureLeftGoalStored = SetNewZValue(FutureLeftGoalStored);
					FutureLeftGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetLeft, FutureLeftGoalStored);
					if(Mathf.Abs(FutureLeftGoalStored.y - FutureRightGoalStored.y) < 0.001f){
						FutureLeftGoalStored = CorrectingGoalPosition(FutureLeftGoalStored, treadXScale);
						//FutureLeftGoalStored = SetNewZValue(FutureLeftGoalStored);
						FutureLeftGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetLeft, FutureLeftGoalStored);
					}
					StepsCycle = true;
					LeftGoalDirection = Actor.GetRoot().forward.normalized;
					RightGoalDirection = Actor.GetRoot().forward.normalized;
				}else{
					if(!StepsCycle){
						FutureRightGoalStored = RightGoalPosition;
					}
				}

				// Continue Step cycle. Sum 2 times the tread depth
				if((LeftGoalClipped || RightGoalClipped) && StepsCycle){
					if(Vector3.Distance(leftFootPos, FutureLeftGoalStored) < 0.2f){
						LeftGoalReached = true;
					}
					if(Vector3.Distance(rightFootPos, FutureRightGoalStored) < 0.2f){
						RightGoalReached = true;
					}
					if(LeftGoalReached){
						LeftGoalReached = false;
						FutureLeftGoalStored = Actor.GetRoot().position + 2 * treadXScale * Actor.GetRoot().forward + stepWidthOffset * Actor.GetRoot().right; //FutureLeftGoalStored + 2 * treadXScale * Actor.GetRoot().forward;
						FutureLeftGoalStored.y = Utility.GetHeight(FutureLeftGoalStored, LayerMask.GetMask("Ground", "Interaction"));
						//FutureLeftGoalStored = SetNewZValue(FutureLeftGoalStored);
						FutureLeftGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetLeft, FutureLeftGoalStored);
						if(Mathf.Abs(FutureLeftGoalStored.y - FutureRightGoalStored.y) < 0.001f){
							FutureLeftGoalStored = CorrectingGoalPosition(FutureLeftGoalStored, treadXScale);
							//FutureLeftGoalStored = SetNewZValue(FutureLeftGoalStored);
							FutureLeftGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetLeft, FutureLeftGoalStored);
						}
						LeftGoalDirection = Actor.GetRoot().forward.normalized;
					}
					if(RightGoalReached){
						RightGoalReached = false;	
						FutureRightGoalStored = Actor.GetRoot().position + 2 * treadXScale * Actor.GetRoot().forward + stepWidthOffset * -Actor.GetRoot().right; //FutureRightGoalStored + 2 * treadXScale * Actor.GetRoot().forward;
						FutureRightGoalStored.y = Utility.GetHeight(FutureRightGoalStored, LayerMask.GetMask("Ground", "Interaction"));
						//FutureRightGoalStored = SetNewZValue(FutureRightGoalStored);
						FutureRightGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetRight, FutureRightGoalStored);
						if(Mathf.Abs(FutureLeftGoalStored.y - FutureRightGoalStored.y) < 0.001f){
							FutureRightGoalStored = CorrectingGoalPosition(FutureRightGoalStored, treadXScale);
							//FutureRightGoalStored = SetNewZValue(FutureRightGoalStored);
							FutureRightGoalStored = SetPointToCenterOfTread(Actor.GetRoot().position + rootOffsetRight, FutureRightGoalStored);
						}
						RightGoalDirection = Actor.GetRoot().forward.normalized;
					}
				}
			}else{
				FutureLeftGoalStored = LeftGoalPosition;
				FutureRightGoalStored = RightGoalPosition;
			}

			LeftGoalPosition = FutureLeftGoalStored;
			RightGoalPosition = FutureRightGoalStored;


			for(int i=0; i<TimeSeries.KeyCount; i++){
				//Saving Future footstep in all time series values
				TimeSeries.Sample sample = TimeSeries.GetKey(i);
				FeetSeries.SetFutureLeftFootPosition(sample.Index, LeftGoalPosition);
				FeetSeries.SetFutureRightFootPosition(sample.Index, RightGoalPosition);
			}

			// If character direction changes, restart step cycle
			if((Vector3.Dot(Actor.GetBoneTransformation("LeftAnkle").GetForward().normalized, LeftGoalDirection) < 0.1f) ||
			(Vector3.Dot(Actor.GetBoneTransformation("RightAnkle").GetForward().normalized, RightGoalDirection) < 0.1f) ||
			(Vector3.Distance(Actor.GetBoneTransformation("LeftAnkle").GetPosition(), LeftGoalPosition) > 1f) ||
			(Vector3.Distance(Actor.GetBoneTransformation("RightAnkle").GetPosition(), RightGoalPosition) > 1f) ){
				resetCycle();
			}


			if(CorrectFootTrajectory){
				//Correcting Feet Trajectories according to foot step goal
				if(RootPosHit.transform.tag == "Tread"){
					float AnkleToFootCenterDistance = 0.05f;
					float AnkleHeight = 0.08f;
					float OffAnkleDistance = 0f;
					float OffRootDistance = 0f;
					float AnkleDistanceThreshold = 0.4f;
					float RootDistanceThreshold = 0.2f;
					int MinLeftDistancePointIndex = 0;
					float MinLeftDistance = float.PositiveInfinity;
					Vector3 DesiredLeftAnklePos = LeftGoalPosition + (-1f) * Actor.GetRoot().forward.normalized * AnkleToFootCenterDistance;
					DesiredLeftAnklePos.y += AnkleHeight;
					int MinRightDistancePointIndex = 0;
					float MinRightDistance = float.PositiveInfinity;
					Vector3 DesiredRightAnklePos = RightGoalPosition + (-1f) * Actor.GetRoot().forward.normalized * AnkleToFootCenterDistance;
					DesiredRightAnklePos.y += AnkleHeight;

					float Multiplier = 1f / (TimeSeries.KeyCount - TimeSeries.PivotKey + 1);

					for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++){
						TimeSeries.Sample sample2 = TimeSeries.GetKey(i);
						Vector3 LeftTrajectoryPos = FeetSeries.GetLeftFootPosition(sample2.Index);
						OffAnkleDistance = Vector3.Distance(LeftTrajectoryPos, DesiredLeftAnklePos);
						OffRootDistance = Vector3.Distance(DesiredLeftAnklePos, Actor.GetRoot().position);
						if(OffAnkleDistance < MinLeftDistance){
							MinLeftDistance = OffAnkleDistance;
							MinLeftDistancePointIndex = i;
						}
						Vector3 RightTrajectoryPos = FeetSeries.GetRightFootPosition(sample2.Index);
						OffAnkleDistance = Vector3.Distance(RightTrajectoryPos, DesiredRightAnklePos);
						OffRootDistance = Vector3.Distance(DesiredRightAnklePos, Actor.GetRoot().position);
						if(OffAnkleDistance < MinRightDistance){
							MinRightDistance = OffAnkleDistance;
							MinRightDistancePointIndex = i;
						}
					}

					TimeSeries.Sample sample3 = TimeSeries.GetKey(MinLeftDistancePointIndex);
					Vector3 ClosestLeftPoint = FeetSeries.GetLeftFootPosition(sample3.Index);
					Vector3 LeftOffsetVector = DesiredLeftAnklePos - ClosestLeftPoint;
					float RootDistance = Vector3.Distance(Actor.GetRoot().position, DesiredLeftAnklePos);
					Debug.DrawLine(ClosestLeftPoint, ClosestLeftPoint + LeftOffsetVector, Color.red, 1f);

					if(MinLeftDistance < AnkleDistanceThreshold && RootDistance > RootDistanceThreshold){
						//Vector3 OffsetLeftPosition = FeetSeries.GetLeftFootPosition(sample3.Index) + LeftOffsetVector;
						//Debug.DrawLine(FeetSeries.GetLeftFootPosition(sample3.Index), OffsetLeftPosition, Color.yellow, 1f);
						//FeetSeries.SetLeftFootPosition(sample3.Index, FeetSeries.GetLeftFootPosition(sample3.Index) + LeftOffsetVector);
						for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++){
							TimeSeries.Sample sample4 = TimeSeries.GetKey(i);
							Vector3 OffsetLeftPosition = FeetSeries.GetLeftFootPosition(sample4.Index) + LeftOffsetVector;
							Debug.DrawLine(FeetSeries.GetLeftFootPosition(sample4.Index), OffsetLeftPosition, Color.yellow, 1f);
							float Weight = Mathf.Abs(MinLeftDistancePointIndex - i);
							FeetSeries.SetLeftFootPosition(sample4.Index, FeetSeries.GetLeftFootPosition(sample4.Index) + (1f - Multiplier * Weight) * LeftOffsetVector); // 
						}
					}

					sample3 = TimeSeries.GetKey(MinRightDistancePointIndex);
					Vector3 ClosestRightPoint = FeetSeries.GetRightFootPosition(sample3.Index);
					Vector3 RightOffsetVector = DesiredRightAnklePos - ClosestRightPoint;
					RootDistance = Vector3.Distance(Actor.GetRoot().position, DesiredRightAnklePos);
					Debug.DrawLine(ClosestRightPoint, ClosestRightPoint + RightOffsetVector, Color.blue, 1f);

					if(MinRightDistance < AnkleDistanceThreshold && RootDistance > RootDistanceThreshold){
						//Vector3 OffsetRightPosition = FeetSeries.GetRightFootPosition(sample3.Index) + RightOffsetVector;
						//Debug.DrawLine(FeetSeries.GetRightFootPosition(sample3.Index), OffsetRightPosition, Color.yellow, 1f);
						//FeetSeries.SetRightFootPosition(sample3.Index, FeetSeries.GetRightFootPosition(sample3.Index) + RightOffsetVector);
						for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++){
							TimeSeries.Sample sample4 = TimeSeries.GetKey(i);
							Vector3 OffsetRightPosition = FeetSeries.GetRightFootPosition(sample4.Index) + RightOffsetVector;
							Debug.DrawLine(FeetSeries.GetRightFootPosition(sample4.Index), OffsetRightPosition, Color.yellow, 1f);
							float Weight = Mathf.Abs(MinRightDistancePointIndex - i);
							FeetSeries.SetRightFootPosition(sample4.Index, FeetSeries.GetRightFootPosition(sample4.Index) + (1f - Multiplier * Weight) * RightOffsetVector);// 
						}
					}

				}
			}
			
		}

		//Read Future Contacts
		float[] contacts = NeuralNetwork.Read(ContactSeries.Bones.Length);
		for(int i=0; i<contacts.Length; i++) {
			contacts[i] = Mathf.Clamp(contacts[i], 0f, 1f);
		}
		ContactSeries.Values[TimeSeries.Pivot] = contacts;

		//Read Phase Update
		float phase = PhaseSeries.Values[TimeSeries.Pivot];
		for(int i=TimeSeries.PivotKey; i<TimeSeries.KeyCount; i++) {
			PhaseSeries.Values[TimeSeries.GetKey(i).Index] = Mathf.Repeat(phase + NeuralNetwork.Read(), 1f);
		}

		//FeetSeries.SetLeftFootPosition(TimeSeries.Pivot, Actor.GetBoneTransformation("LeftAnkle").GetPosition());
		//FeetSeries.SetRightFootPosition(TimeSeries.Pivot, Actor.GetBoneTransformation("RightAnkle").GetPosition());

		//Interpolate Current to Future Trajectory
		for(int i=0; i<TimeSeries.Samples.Length; i++) {
			float weight = (float)(i % TimeSeries.Resolution) / TimeSeries.Resolution;
			TimeSeries.Sample sample = TimeSeries.Samples[i];
			TimeSeries.Sample prevSample = TimeSeries.GetPreviousKey(i);
			TimeSeries.Sample nextSample = TimeSeries.GetNextKey(i);
			//PhaseSeries.Values[sample.Index] = Mathf.Lerp(PhaseSeries.Values[prevSample.Index], PhaseSeries.Values[nextSample.Index], weight);
			RootSeries.SetPosition(sample.Index, Vector3.Lerp(RootSeries.GetPosition(prevSample.Index), RootSeries.GetPosition(nextSample.Index), weight));
			RootSeries.SetDirection(sample.Index, Vector3.Slerp(RootSeries.GetDirection(prevSample.Index), RootSeries.GetDirection(nextSample.Index), weight));
			GoalSeries.Transformations[sample.Index] = Utility.Interpolate(GoalSeries.Transformations[prevSample.Index], GoalSeries.Transformations[nextSample.Index], weight);
			for(int j=0; j<StyleSeries.Styles.Length; j++) {
				StyleSeries.Values[i][j] = Mathf.Lerp(StyleSeries.Values[prevSample.Index][j], StyleSeries.Values[nextSample.Index][j], weight);
			}
			for(int j=0; j<GoalSeries.Actions.Length; j++) {
				GoalSeries.Values[i][j] = Mathf.Lerp(GoalSeries.Values[prevSample.Index][j], GoalSeries.Values[nextSample.Index][j], weight);
			}

			FeetSeries.SetLeftFootPosition(sample.Index, Vector3.Lerp(FeetSeries.GetLeftFootPosition(sample.Index), FeetSeries.GetLeftFootPosition(nextSample.Index),weight));
			//FeetSeries.SetLeftFootDirection(sample.Index, Vector3.Slerp(FeetSeries.GetLeftFootPosition(prevSample.Index), FeetSeries.GetLeftFootDirection(nextSample.Index), weight));
			FeetSeries.SetRightFootPosition(sample.Index, Vector3.Lerp(FeetSeries.GetRightFootPosition(sample.Index), FeetSeries.GetRightFootPosition(nextSample.Index),weight));
			//FeetSeries.SetRightFootDirection(sample.Index, Vector3.Slerp(FeetSeries.GetRightFootPosition(prevSample.Index), FeetSeries.GetRightFootDirection(nextSample.Index), weight));
			
			//FeetSeries.SetLeftFootTransformation(i, Actor.GetBoneTransformation("LeftAnkle"));
			//FeetSeries.SetRightFootTransformation(i, Actor.GetBoneTransformation("RightAnkle"));
		}

		/*// Feet trajectories interpolation
		for(int i=0; i<TimeSeries.KeyCount; i++){
			float weight = (float)(i % TimeSeries.Resolution) / TimeSeries.Resolution;
			//float weight = 0.5f;
			TimeSeries.Sample sample = TimeSeries.Samples[i];//TimeSeries.GetKey(i);
			TimeSeries.Sample prevSample = TimeSeries.GetPreviousKey(i);
			TimeSeries.Sample nextSample = TimeSeries.GetNextKey(i);
			FeetSeries.SetLeftFootPosition(sample.Index, Vector3.Lerp(FeetSeries.GetLeftFootPosition(sample.Index),FeetSeries.GetLeftFootPosition(nextSample.Index),weight));
			//FeetSeries.SetFutureLeftFootDirection(sample.Index, move);
			FeetSeries.SetRightFootPosition(sample.Index, Vector3.Lerp(FeetSeries.GetRightFootPosition(sample.Index),FeetSeries.GetRightFootPosition(nextSample.Index),weight));
			//FeetSeries.SetFutureRightFootDirection(sample.Index, move);
		}*/

		//Assign Posture
		Vector3 LeftToePos = Vector3.zero;
		Vector3 RightToePos = Vector3.zero;
		transform.position = RootSeries.GetPosition(TimeSeries.Pivot);
		transform.rotation = RootSeries.GetRotation(TimeSeries.Pivot);
		for(int i=0; i<Actor.Bones.Length; i++) {
			/*if(Actor.Bones[i].GetName() == "LeftAnkle"){
				Actor.Bones[i].Transform.position = FeetSeries.LeftFootTransformations[TimeSeries.Pivot].GetPosition(); 
			}else if(Actor.Bones[i].GetName() == "RightAnkle"){
				Actor.Bones[i].Transform.position = FeetSeries.RightFootTransformations[TimeSeries.Pivot].GetPosition();
			}else{
				Actor.Bones[i].Transform.position = positions[i];
			}*/
			Actor.Bones[i].Velocity = velocities[i];
			Actor.Bones[i].Transform.position = positions[i];
			Actor.Bones[i].Transform.rotation = Quaternion.LookRotation(forwards[i], upwards[i]);
			Actor.Bones[i].ApplyLength();
		}

		// Setting Ankle position estimation to the actor. Also correcting Toe position by offset
		if(UseSteps){
			if(CorrectAnklePosition){
				float ankleVelThres = 0.8f;
				Actor.FindBone("LeftAnkle").Transform.position = FeetSeries.LeftFootTransformations[TimeSeries.Pivot].GetPosition();
				if(Actor.GetBoneVelocity("LeftAnkle").magnitude < ankleVelThres){
					Actor.FindBone("LeftToe").Transform.position = Actor.FindBone("LeftToe").Transform.position + (-1f) * Actor.FindBone("Hips").Transform.up * 0.06f;
				}
				//Debug.Log("Foot height = " + FeetSeries.LeftFootTransformations[TimeSeries.Pivot].GetPosition().y);
				//Actor.FindBone("LeftToe").Transform.position = LeftToePos;
				//Actor.FindBone("LeftAnkle").Transform.rotation = FeetSeries.LeftFootTransformations[TimeSeries.Pivot].GetRotation();
				Actor.FindBone("RightAnkle").Transform.position = FeetSeries.RightFootTransformations[TimeSeries.Pivot].GetPosition();
				if(Actor.GetBoneVelocity("RightAnkle").magnitude < ankleVelThres){
					Actor.FindBone("RightToe").Transform.position = Actor.FindBone("RightToe").Transform.position + (-1f) * Actor.FindBone("Hips").Transform.up * 0.06f;
				}
				//Actor.FindBone("RightAnkle").Transform.position = RightToePos; 
				//Actor.FindBone("RightAnkle").Transform.rotation = FeetSeries.RightFootTransformations[TimeSeries.Pivot].GetRotation();
			}
		}
		
		//FeetSeries.SetLeftFootTransformation(TimeSeries.Pivot, Actor.GetBoneTransformation("LeftAnkle"));
		//FeetSeries.SetRightFootTransformation(TimeSeries.Pivot, Actor.GetBoneTransformation("RightAnkle"));
		//Debug.Log("Read completed " + counter);
		//counter += 1;
	}

	
	//
	private void resetCycle(){
		LeftGoalReached = false;
		RightGoalReached = false;
		LeftGoalClipped = false;
		RightGoalClipped = false;
		StepsCycle = false;
	}

	//Used to get the direction of climbing a staircase (ascend/descend)
	private Vector3 GetParentForwardVector(Transform transform){
		if(transform.name.StartsWith("Stairs_")){
			return transform.forward.normalized;
		}
		else{
			//Debug.Log("Name = " + transform.name);
			return GetParentForwardVector(transform.parent.transform);
		}
	}

	
	private Vector3 ClipGoalLength(Vector3 goalPoint, float distanceToPlane, ref bool goalClipped){
		Vector3 rootPos = Actor.GetRoot().position;
		if(distanceToPlane < 0f){
			goalClipped = true;
			goalPoint += (distanceToPlane) * Actor.GetRoot().forward.normalized;
			goalPoint.y = Utility.GetHeight(goalPoint, LayerMask.GetMask("Ground","Interaction"));			
		}
		return goalPoint;
	}
	private Vector3 SetNewGoalPosition(Vector3 originalPos){
		float distanceOffset = 0.2f;
		float frontDistance = 0f;
		float backDistance = 0f;
		Vector3 ProjectedPoint = Vector3.zero;
		Vector3 CollisionPoint = Vector3.zero;
		Vector3 originalPosMovedUp = originalPos;
		originalPosMovedUp.y += 1;
		RaycastHit hitRay;
		Physics.Raycast(originalPosMovedUp, Vector3.down, out hitRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		float localScaleX = hitRay.transform.localScale.x * 100;
		if(localScaleX < 1f){
			Vector3 FrontPoint = hitRay.point + Actor.GetRoot().forward.normalized * distanceOffset;
			//Debug.DrawRay(hitRay.point, Actor.GetRoot().forward, Color.red, 1f);
			//Debug.DrawRay(FrontPoint, Actor.GetRoot().forward, Color.blue, 1f);
			frontDistance = GetCorrectionDistance(hitRay, FrontPoint, -1f * Actor.GetRoot().forward.normalized);
			Vector3 BackPoint = hitRay.point + (-1f) * Actor.GetRoot().forward.normalized * distanceOffset;
			if(frontDistance == 0f){
				backDistance = GetCorrectionDistance(hitRay, BackPoint, Actor.GetRoot().forward.normalized);
			}
			return originalPos 	+ (-1f * Actor.GetRoot().forward.normalized * frontDistance)
								+ (Actor.GetRoot().forward.normalized * backDistance);
		}
		return originalPos;
	}

	private float GetCorrectionDistance(RaycastHit hitRay, Vector3 OffsetPoint, Vector3 ShootingDirection){
		float OffsetPointHeight = Utility.GetHeight(OffsetPoint, LayerMask.GetMask("Ground","Interaction"));
		//Debug.Log("OffsetPoint height = " + OffsetPointHeight);
		OffsetPoint.y += 1;
		Vector3 hitRayPoint = hitRay.point;
		RaycastHit hitRayOffsetPoint;
		RaycastHit hitRayCollision;
		if(OffsetPointHeight != hitRayPoint.y){
			Physics.Raycast(OffsetPoint, Vector3.down, out hitRayOffsetPoint, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			OffsetPoint.y = hitRayOffsetPoint.point.y;
			if(OffsetPointHeight > hitRayPoint.y){
				OffsetPoint.y = hitRayPoint.y;
			}else{
				hitRayPoint.y = OffsetPoint.y;
			}
			//Debug.DrawRay(hitRayPoint, ShootingDirection, Color.red, 1f);
			Debug.DrawRay(OffsetPoint, ShootingDirection, Color.blue, 1f);
			Physics.Raycast(OffsetPoint, ShootingDirection, out hitRayCollision, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			//Debug.DrawRay(hitRayCollision.point, ShootingDirection, Color.blue, 1f);
			return Vector3.Distance(OffsetPoint, hitRayCollision.point);
		}
		return 0f;
	}

	//Clipping distance of step goal from character
	private Vector3 SetValidStairStep(Vector3 goalPoint, Vector3 footPos){
		RaycastHit footPosHit;
		RaycastHit validStairStep;
		Physics.Raycast(footPos, Vector3.down, out footPosHit, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		float localScaleX = footPosHit.transform.localScale.x * 100;
		if(localScaleX < 1f && (Vector3.Distance(goalPoint, footPosHit.point) > localScaleX * 0.8 ||
		Vector3.Distance(goalPoint, footPosHit.point) < 0.05) && 
		Controller.QueryMove(KeyCode.W, KeyCode.S, KeyCode.A, KeyCode.D, Signals) != Vector3.zero){
			//Debug.Log("Distance = " + Vector3.Distance(goalPoint, footPosHit.point));
			footPos.y = footPosHit.point.y;
			Vector3 ValidStepLengthPoint = footPos + Actor.GetRoot().forward.normalized * localScaleX;
			ValidStepLengthPoint.y += 1;
			Physics.Raycast(ValidStepLengthPoint, Vector3.down, out validStairStep, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			return validStairStep.point;
		}
		return goalPoint;
	}

	//Moving goal point in direction of root by 1 tread depth
	private Vector3 SetValidTreadStep(Vector3 goalPoint, Vector3 footPos){
		Vector3 rootPos = Actor.GetRoot().position;
		RaycastHit footPosHit;
		RaycastHit validStairStep;
		Physics.Raycast(footPos, Vector3.down, out footPosHit, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		float localScaleX = footPosHit.transform.localScale.x * 100;
		if(localScaleX < 1f && Vector3.Distance(goalPoint, rootPos) < localScaleX * 0.6 && 
		Controller.QueryMove(KeyCode.W, KeyCode.S, KeyCode.A, KeyCode.D, Signals) != Vector3.zero){
			//Debug.Log("Distance = " + Vector3.Distance(goalPoint, rootPos));
			footPos.y = footPosHit.point.y;
			Vector3 ValidStepLengthPoint = footPos + Actor.GetRoot().forward.normalized * localScaleX;
			ValidStepLengthPoint.y += 1;
			Physics.Raycast(ValidStepLengthPoint, Vector3.down, out validStairStep, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			return validStairStep.point;
		}
		return goalPoint;
	}

	//Setting point to center of Tread
	private Vector3 SetPointToCenterOfTread(Vector3 vectorOrigin, Vector3 goalPoint){
		Vector3 goalPointMovedUp = goalPoint;
		goalPointMovedUp.y += 1;
		RaycastHit hitRay;
		Physics.Raycast(goalPointMovedUp, Vector3.down, out hitRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		//Plane treadCenterPlane = new Plane(hitRay.transform.right, hitRay.transform.position);
		//DrawPlane(hitRay.transform.position, hitRay.transform.right, Color.green);
		if( hitRay.transform.tag == "Tread" && CharacterMoving()){
			Plane treadCenterPlane = new Plane(hitRay.transform.right, hitRay.transform.position);
			//DrawPlane(hitRay.transform.position, hitRay.transform.right, Color.green);
			//Computing distance as follows: d = (p0 - l0)*n / (l*n) ; Source: https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
			float d = Vector3.Dot(hitRay.transform.position - vectorOrigin, hitRay.transform.right) / 
			          Vector3.Dot(Actor.GetRoot().forward.normalized, hitRay.transform.right);
			if(d > 2f || d < 0f){
				resetCycle();
				return goalPoint;
			}
			RaycastHit treadHit;
			Vector3 vectorOnTreadPlaneMovedUp = vectorOrigin + Actor.GetRoot().forward.normalized * d;
			vectorOnTreadPlaneMovedUp.y += 1f;
			Physics.Raycast(vectorOnTreadPlaneMovedUp, Vector3.down, out treadHit, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
			//Debug.Log("d = " + d);
			//Vector3 intersectionPoint = treadCenterPlane.;
			//goalPoint.z = hitRay.transform.position.z;
			goalPoint = treadHit.point;
		}
		return goalPoint;
	}
	
	//Setting Z value to the tread Z coordinate (center of tread)
	private Vector3 SetNewZValue(Vector3 originalPos){
		Vector3 originalPosMovedUp = originalPos;
		originalPosMovedUp.y += 1;
		RaycastHit hitRay;
		Physics.Raycast(originalPosMovedUp, Vector3.down, out hitRay, float.PositiveInfinity, LayerMask.GetMask("Ground","Interaction"));
		float localScaleX = hitRay.transform.localScale.x * 100;
		if(localScaleX < 1f && CharacterMoving()){
			originalPos.z = hitRay.transform.position.z;
		}
		return originalPos;
	}

	private Vector3 CorrectingGoalPosition(Vector3 goalPos, float XScale){
		if(XScale < 1f){
			goalPos = goalPos + 1f * XScale * Actor.GetRoot().forward.normalized;
			goalPos.y = Utility.GetHeight(goalPos, LayerMask.GetMask("Ground","Interaction"));
		}
		return goalPos;
	}

	private void Default() {
		if(Controller.ProjectionActive) {
			ApplyStaticGoal(Controller.Projection.point, Vector3.ProjectOnPlane(Controller.Projection.point-transform.position, Vector3.up).normalized, Signals);
			/*
			Vector3 direction = (Controller.Projection.point - transform.position).GetRelativeDirectionTo(transform.GetWorldMatrix()).normalized;
			ApplyDynamicGoal(
				transform.GetWorldMatrix(),
				direction,
				Vector3.SignedAngle(transform.forward, Controller.Projection.point, Vector3.up) / 2f,
				Signals
			);
			*/
		} else {
			if(activeSpline){
				//Debug.Log("Quit = " + Controller.quit);
				if(Controller.quit){
					UnityEditor.EditorApplication.isPlaying = false;
				}
				Matrix4x4 root = RootSeries.Transformations[TimeSeries.Pivot];
				//Vector3 targetDir = Controller.getTransition(root).normalized.GetRelativeDirectionTo(root);
				Vector3 targetDir = Controller.getTransition(root).normalized;
				//Debug.DrawRay(root.GetPosition(), targetDir, Color.red);
				//Debug.Log("Target direction = " + targetDir);
				bool[] Turns = Controller.OnOffControllerWithHysterisis(root, 15f);
				ApplyDynamicGoal(
				RootSeries.Transformations[TimeSeries.Pivot],
				Controller.QueryMove(Signals),
				Controller.QueryTurn(Turns[0], Turns[1], 90f), 
				Signals,
				Quaternion.LookRotation(targetDir, Vector3.up) * Vector3.ProjectOnPlane(root.GetForward(), Vector3.up).normalized
			);	
			}else{
				ApplyDynamicGoal(
				RootSeries.Transformations[TimeSeries.Pivot],
				Controller.QueryMove(KeyCode.W, KeyCode.S, KeyCode.A, KeyCode.D, Signals),
				Controller.QueryTurn(KeyCode.Q, KeyCode.E, 90f), 
				Signals,
				Vector3.zero
			);
			}
		}
		/*Geometry.Setup(Geometry.Resolution);
		Geometry.Sense(RootSeries.Transformations[TimeSeries.Pivot], LayerMask.GetMask("Interaction"), Vector3.zero, InteractionSmoothing);
		*/
	}

    protected override void Postprocess() {
		

		if(UseIK){
			Matrix4x4 rightFoot = Actor.GetBoneTransformation(ContactSeries.Bones[0]);
			Matrix4x4 leftFoot = Actor.GetBoneTransformation(ContactSeries.Bones[1]);
			RightFootIK.Objectives[0].SetTarget(rightFoot.GetPosition(), 1f-ContactSeries.Values[TimeSeries.Pivot][0]);
			RightFootIK.Objectives[0].SetTarget(rightFoot.GetRotation());
			LeftFootIK.Objectives[0].SetTarget(leftFoot.GetPosition(), 1f-ContactSeries.Values[TimeSeries.Pivot][1]);
			LeftFootIK.Objectives[0].SetTarget(leftFoot.GetRotation());
			RightFootIK.Solve();
			LeftFootIK.Solve();

			Transform rootTransform = Actor.GetRoot();
			Vector3 rootTransformPos = rootTransform.transform.position;
			float rootPosHeight = Utility.GetHeight(rootTransformPos, LayerMask.GetMask("Ground"));
			//Debug.Log("Root Pos = " + rootPosHeight);
			
			Transform rightAnkle = Actor.FindBone("RightAnkle").Transform;
			Vector3 rightAnklePos = rightAnkle.transform.position;
			//rightPos.y = Mathf.Max(rightPos.y, 0.02f);
			//spine.position = new Vector3(spinePosition.x, spineHeight + (spinePosition.y - transform.position.y), spinePosition.z);
			if (rightAnklePos.y < rootPosHeight){
				rightAnklePos.y = rootPosHeight + 0.05f;
			}
			//rightAnklePos.y += rootPosHeight;
			rightAnkle.position = rightAnklePos;

			Transform leftAnkle = Actor.FindBone("LeftAnkle").Transform;
			Vector3 leftAnklePos = leftAnkle.transform.position;
			//rightPos.y = Mathf.Max(rightPos.y, 0.02f);
			//spine.position = new Vector3(spinePosition.x, spineHeight + (spinePosition.y - transform.position.y), spinePosition.z);
			if (leftAnklePos.y < rootPosHeight){
				leftAnklePos.y = rootPosHeight + 0.05f;
			}
			//leftAnklePos.y += rootPosHeight;
			leftAnkle.position = leftAnklePos;
		}

    }

	protected override void OnGUIDerived() {
		if(!ShowGUI) {
			return;
		}
		if(ShowGoal) {
			GoalSeries.GUI();
		}
		if(ShowCurrent) {
			StyleSeries.GUI();
		}
		if(ShowPhase) {
			PhaseSeries.GUI();
		}
		if(ShowContacts) {
			ContactSeries.GUI();
		}
	}

	protected override void OnRenderObjectDerived() {
		Controller.Draw();

		if(ShowRoot) {
			RootSeries.Draw();
		}
		if(ShowGoal) {
			GoalSeries.Draw();
		}
		if(ShowCurrent) {
			StyleSeries.Draw();
		}
		if(ShowPhase) {
			PhaseSeries.Draw();
		}
		if(ShowContacts) {
			ContactSeries.Draw();
		}
		if(ShowEnvironment) {
			Environment.Draw(UltiDraw.Mustard.Transparent(0.25f));
		}
		if(ShowInteraction) {
			Geometry.Draw(UltiDraw.Cyan.Transparent(0.25f));
		}
		if(ShowHeightMap){
			EnvironmentHeight.Draw();
			//LeftFootProjection.Draw();
			//RightFootProjection.Draw();
		}
		if(UseSteps){
			FeetSeries.Draw();
		}

		if(ShowBiDirectional) {
			UltiDraw.Begin();
			for(int i=0; i<PosePrediction.Length; i++) {
				UltiDraw.DrawSphere(PosePrediction[i], Quaternion.identity, 0.05f, UltiDraw.Magenta);
			}
			for(int i=0; i<RootPrediction.Length; i++) {
				UltiDraw.DrawCircle(RootPrediction[i].GetPosition(), 0.05f, UltiDraw.DarkRed.Darken(0.5f));
				UltiDraw.DrawArrow(RootPrediction[i].GetPosition(), RootPrediction[i].GetPosition() + 0.1f*RootPrediction[i].GetForward(), 0f, 0f, 0.025f, UltiDraw.DarkRed);
				if(i<RootPrediction.Length-1) {
					UltiDraw.DrawLine(RootPrediction[i].GetPosition(), RootPrediction[i+1].GetPosition(), UltiDraw.Black);
				}
			}
			for(int i=0; i<GoalPrediction.Length; i++) {
				UltiDraw.DrawCircle(GoalPrediction[i].GetPosition(), 0.05f, UltiDraw.DarkGreen.Darken(0.5f));
				UltiDraw.DrawArrow(GoalPrediction[i].GetPosition(), GoalPrediction[i].GetPosition() + 0.1f*GoalPrediction[i].GetForward(), 0f, 0f, 0.025f, UltiDraw.DarkGreen);
				if(i<GoalPrediction.Length-1) {
					UltiDraw.DrawLine(GoalPrediction[i].GetPosition(), GoalPrediction[i+1].GetPosition(), UltiDraw.Black);
				}
			}
			UltiDraw.End();
		}
    }

	private float[] GenerateGating() {
		List<float> values = new List<float>();

		for(int k=0; k<TimeSeries.KeyCount; k++) {
			int index = TimeSeries.GetKey(k).Index;
			Vector2 phase = Utility.PhaseVector(PhaseSeries.Values[index]);
			for(int i=0; i<StyleSeries.Styles.Length; i++) {
				float magnitude = StyleSeries.Values[index][i];
				magnitude = Utility.Normalise(magnitude, 0f, 1f, -1f ,1f);
				values.Add(magnitude * phase.x);
				values.Add(magnitude * phase.y);
			}
			for(int i=0; i<GoalSeries.Actions.Length; i++) {
				float magnitude = GoalSeries.Values[index][i];
				magnitude = Utility.Normalise(magnitude, 0f, 1f, -1f ,1f);
				Matrix4x4 root = RootSeries.Transformations[index];
				root[1,3] = 0f;
				Matrix4x4 goal = GoalSeries.Transformations[index];
				goal[1,3] = 0f;
				float distance = Vector3.Distance(root.GetPosition(), goal.GetPosition());
				float angle = Quaternion.Angle(root.GetRotation(), goal.GetRotation());
				values.Add(magnitude * phase.x);
				values.Add(magnitude * phase.y);
				values.Add(magnitude * distance * phase.x);
				values.Add(magnitude * distance * phase.y);
				values.Add(magnitude * angle * phase.x);
				values.Add(magnitude * angle * phase.y);
			}
		}
		return values.ToArray();
	}

	private void ApplyStaticGoal(Vector3 position, Vector3 direction, float[] actions) {
		//Transformations
		for(int i=0; i<TimeSeries.Samples.Length; i++) {
			float weight = TimeSeries.GetWeight1byN1(i, 2f);
			float positionBlending = weight * UserControl;
			float directionBlending = weight * UserControl;
			Matrix4x4Extensions.SetPosition(ref GoalSeries.Transformations[i], Vector3.Lerp(GoalSeries.Transformations[i].GetPosition(), position, positionBlending));
			Matrix4x4Extensions.SetRotation(ref GoalSeries.Transformations[i], Quaternion.LookRotation(Vector3.Slerp(GoalSeries.Transformations[i].GetForward(), direction, directionBlending), Vector3.up));
		}

		//Actions
		for(int i=TimeSeries.Pivot; i<TimeSeries.Samples.Length; i++) {
			float w = (float)(i-TimeSeries.Pivot) / (float)(TimeSeries.FutureSampleCount);
			w = Utility.Normalise(w, 0f, 1f, 1f/TimeSeries.FutureKeyCount, 1f);
			for(int j=0; j<GoalSeries.Actions.Length; j++) {
				float weight = GoalSeries.Values[i][j];
				weight = 2f * (0.5f - Mathf.Abs(weight - 0.5f));
				weight = Utility.Normalise(weight, 0f, 1f, UserControl, 1f-UserControl);
				if(actions[j] != GoalSeries.Values[i][j]) {
					GoalSeries.Values[i][j] = Mathf.Lerp(
						GoalSeries.Values[i][j], 
						Mathf.Clamp(GoalSeries.Values[i][j] + weight * UserControl * Mathf.Sign(actions[j] - GoalSeries.Values[i][j]), 0f, 1f),
						w);
				}
			}
		}
	}

	private void ApplyDynamicGoal(Matrix4x4 root, Vector3 move, float turn, float[] actions, Vector3 direction) {
		//Transformations
		Vector3[] positions_blend = new Vector3[TimeSeries.Samples.Length];
		Vector3[] directions_blend = new Vector3[TimeSeries.Samples.Length];
		float time = 2f;
		float TargetBlending = 0.25f;
		for(int i=0; i<TimeSeries.Samples.Length; i++) {
			float weight = TimeSeries.GetWeight1byN1(i, 0.5f);
			float bias_pos = 1.0f - Mathf.Pow(1.0f - weight, 0.75f);
			float bias_dir = 1.0f - Mathf.Pow(1.0f - weight, 0.75f);
			//Debug.Log("bias_dir = " + bias_dir);
			if(!activeSpline){
				//time = 2f;
				directions_blend[i] = Quaternion.AngleAxis(bias_dir * turn, Vector3.up) * Vector3.ProjectOnPlane(root.GetForward(), Vector3.up).normalized;
			}else{
				if (Controller.Signal.type == BezierSolution.BezierPoint.StatusMode.Wait && !Controller.waiting)
				{
					Controller.waiting = true;
					Invoke("StopWaiting", Controller.getCurrentPoints()[0].timeout);
				}else{
					//time = 2f;
					directions_blend[i] = Quaternion.AngleAxis(bias_dir * turn, Vector3.up) * Vector3.ProjectOnPlane(root.GetForward(), Vector3.up).normalized;
					//directions_blend[i] = root.GetForward();
					//directions_blend[i] = Vector3.Lerp(directions_blend[i], direction, 0.15f);
				}
			}

			if(i==0) {
				positions_blend[i] = root.GetPosition() + 
				Vector3.Lerp(
				GoalSeries.Transformations[i+1].GetPosition() - GoalSeries.Transformations[i].GetPosition(), 
				time / (TimeSeries.Samples.Length - 1f) * (Quaternion.LookRotation(directions_blend[i], Vector3.up) * move),
				bias_pos
				);
			} else {
				positions_blend[i] = positions_blend[i-1] + 
				Vector3.Lerp(
				GoalSeries.Transformations[i].GetPosition() - GoalSeries.Transformations[i-1].GetPosition(), 
				time / (TimeSeries.Samples.Length - 1f) * (Quaternion.LookRotation(directions_blend[i], Vector3.up) * move),
				bias_pos
				);
			}
			
		}
		for(int i=0; i<TimeSeries.Samples.Length; i++) {
			Matrix4x4Extensions.SetPosition(ref GoalSeries.Transformations[i], Vector3.Lerp(GoalSeries.Transformations[i].GetPosition(), positions_blend[i], UserControl));
			Matrix4x4Extensions.SetRotation(ref GoalSeries.Transformations[i], Quaternion.Slerp(GoalSeries.Transformations[i].GetRotation(), Quaternion.LookRotation(directions_blend[i], Vector3.up), UserControl));
		}
		
		//Actions
		for(int i=TimeSeries.Pivot; i<TimeSeries.Samples.Length; i++) {
			float w = (float)(i-TimeSeries.Pivot) / (float)(TimeSeries.FutureSampleCount);
			w = Utility.Normalise(w, 0f, 1f, 1f/TimeSeries.FutureKeyCount, 1f);
			for(int j=0; j<GoalSeries.Actions.Length; j++) {
				float weight = GoalSeries.Values[i][j];
				weight = 2f * (0.5f - Mathf.Abs(weight - 0.5f));
				weight = Utility.Normalise(weight, 0f, 1f, UserControl, 1f-UserControl);
				if(actions[j] != GoalSeries.Values[i][j]) {
					GoalSeries.Values[i][j] = Mathf.Lerp(
						GoalSeries.Values[i][j], 
						Mathf.Clamp(GoalSeries.Values[i][j] + weight * UserControl * Mathf.Sign(actions[j] - GoalSeries.Values[i][j]), 0f, 1f),
						w);
				}
			}
		}
	}

	private bool CharacterMoving(){
		if(Controller.QueryMove(KeyCode.W, KeyCode.S, KeyCode.A, KeyCode.D, Signals) != Vector3.zero || 
		(activeSpline && Controller.QueryMove(Signals) != Vector3.zero)){
			return true;
		}
		return false;
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