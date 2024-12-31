//
// This file was auto-generated using the following command:
//
// jai generate.jai
//

PxEMPTY :: enum s32 {
    PxEmpty :: 0;
}

PxZERO :: enum s32 {
    PxZero :: 0;
}

PxIDENTITY :: enum s32 {
    PxIdentity :: 0;
}

PxErrorCode :: enum_flags s32 {
    NoError :: 0;
    DebugInfo :: 1;
    DebugWarning :: 2;
    InvalidParameter :: 4;
    InvalidOperation :: 8;
    OutOfMemory :: 16;
    InternalError :: 32;
    Abort :: 64;
    PerfWarning :: 128;
    MaskAll :: -1;
}

PxThreadPriority :: enum u32 {
    High :: 0;
    AboveNormal :: 1;
    Normal :: 2;
    BelowNormal :: 3;
    Low :: 4;
    ForceDword :: 4294967295;
}

PxDebugColor :: enum u32 {
    ArgbBlack :: 4278190080;
    ArgbRed :: 4294901760;
    ArgbGreen :: 4278255360;
    ArgbBlue :: 4278190335;
    ArgbYellow :: 4294967040;
    ArgbMagenta :: 4294902015;
    ArgbCyan :: 4278255615;
    ArgbWhite :: 4294967295;
    ArgbGrey :: 4286611584;
    ArgbDarkred :: 4287102976;
    ArgbDarkgreen :: 4278224896;
    ArgbDarkblue :: 4278190216;
}

PxConcreteType :: enum s32 {
    Undefined :: 0;
    Heightfield :: 1;
    ConvexMesh :: 2;
    TriangleMeshBvh33 :: 3;
    TriangleMeshBvh34 :: 4;
    TetrahedronMesh :: 5;
    SoftbodyMesh :: 6;
    RigidDynamic :: 7;
    RigidStatic :: 8;
    Shape :: 9;
    Material :: 10;
    SoftbodyMaterial :: 11;
    ClothMaterial :: 12;
    PbdMaterial :: 13;
    FlipMaterial :: 14;
    MpmMaterial :: 15;
    CustomMaterial :: 16;
    Constraint :: 17;
    Aggregate :: 18;
    ArticulationReducedCoordinate :: 19;
    ArticulationLink :: 20;
    ArticulationJointReducedCoordinate :: 21;
    ArticulationSensor :: 22;
    ArticulationSpatialTendon :: 23;
    ArticulationFixedTendon :: 24;
    ArticulationAttachment :: 25;
    ArticulationTendonJoint :: 26;
    PruningStructure :: 27;
    Bvh :: 28;
    SoftBody :: 29;
    SoftBodyState :: 30;
    PbdParticlesystem :: 31;
    FlipParticlesystem :: 32;
    MpmParticlesystem :: 33;
    CustomParticlesystem :: 34;
    FemCloth :: 35;
    HairSystem :: 36;
    ParticleBuffer :: 37;
    ParticleDiffuseBuffer :: 38;
    ParticleClothBuffer :: 39;
    ParticleRigidBuffer :: 40;
    PhysxCoreCount :: 41;
    FirstPhysxExtension :: 256;
    FirstVehicleExtension :: 512;
    FirstUserExtension :: 1024;
}

PxBaseFlags :: enum_flags u16 {
    OwnsMemory :: 1 << 0;
    IsReleasable :: 1 << 1;
}

PxMetaDataFlag :: enum_flags s32 {
    Class :: 1;
    Virtual :: 2;
    Typedef :: 4;
    Ptr :: 8;
    Handle :: 16;
    ExtraData :: 32;
    ExtraItem :: 64;
    ExtraItems :: 128;
    ExtraName :: 256;
    Union :: 512;
    Padding :: 1024;
    Alignment :: 2048;
    CountMaskMsb :: 4096;
    CountSkipIfOne :: 8192;
    ControlFlip :: 16384;
    ControlMask :: 32768;
    ControlMaskRange :: 255;
    ForceDword :: 2147483647;
}

PxTaskType :: enum s32 {
    Cpu :: 0;
    NotPresent :: 1;
    Completed :: 2;
}

PxGeometryType :: enum s32 {
    Sphere :: 0;
    Plane :: 1;
    Capsule :: 2;
    Box :: 3;
    Convexmesh :: 4;
    Particlesystem :: 5;
    Tetrahedronmesh :: 6;
    Trianglemesh :: 7;
    Heightfield :: 8;
    Hairsystem :: 9;
    Custom :: 10;
    GeometryCount :: 11;
    Invalid :: -1;
}

PxGeometryQueryFlags :: enum_flags u32 {
    SimdGuard :: 1 << 0;
}

PxBVHBuildStrategy :: enum s32 {
    Fast :: 0;
    Default :: 1;
    Sah :: 2;
    Last :: 3;
}

PxConvexMeshGeometryFlags :: enum_flags u8 {
    TightBounds :: 1 << 0;
}

PxMeshGeometryFlags :: enum_flags u8 {
    TightBounds :: 1 << 0;
    DoubleSided :: 1 << 1;
}

PxParticleSolverType :: enum_flags s32 {
    Pbd :: 1;
    Flip :: 2;
    Mpm :: 4;
    Custom :: 8;
}

PxHitFlags :: enum_flags u16 {
    Position :: 1 << 0;
    Normal :: 1 << 1;
    Uv :: 1 << 3;
    AssumeNoInitialOverlap :: 1 << 4;
    AnyHit :: 1 << 5;
    MeshMultiple :: 1 << 6;
    MeshBothSides :: 1 << 7;
    PreciseSweep :: 1 << 8;
    Mtd :: 1 << 9;
    FaceIndex :: 1 << 10;
    Default :: PxHitFlags.Position | PxHitFlags.Normal | PxHitFlags.FaceIndex;
    ModifiableFlags :: PxHitFlags.AssumeNoInitialOverlap | PxHitFlags.MeshMultiple | PxHitFlags.MeshBothSides | PxHitFlags.PreciseSweep;
}

PxHeightFieldFormat :: enum s32 {
    S16Tm :: 1;
}

PxHeightFieldTessFlag :: enum_flags s32 {
    E0ThVertexShared :: 1;
}

PxHeightFieldFlags :: enum_flags u16 {
    NoBoundaryEdges :: 1 << 0;
}

PxHeightFieldMaterial :: enum s32 {
    Hole :: 127;
}

PxMeshMeshQueryFlags :: enum_flags u32 {
    Default :: 0;
    DiscardCoplanar :: 1 << 0;
}

PxMeshFlags :: enum_flags u16 {
    Flipnormals :: 1 << 0;
    E16BitIndices :: 1 << 1;
}

PxMeshMidPhase :: enum s32 {
    Bvh33 :: 0;
    Bvh34 :: 1;
    Last :: 2;
}

PxTriangleMeshFlags :: enum_flags u8 {
    E16BitIndices :: 1 << 1;
    AdjacencyInfo :: 1 << 2;
    PreferNoSdfProj :: 1 << 3;
}

PxTetrahedronMeshFlags :: enum_flags u8 {
    E16BitIndices :: 1 << 1;
}

PxActorFlags :: enum_flags u8 {
    Visualization :: 1 << 0;
    DisableGravity :: 1 << 1;
    SendSleepNotifies :: 1 << 2;
    DisableSimulation :: 1 << 3;
}

PxActorType :: enum s32 {
    RigidStatic :: 0;
    RigidDynamic :: 1;
    ArticulationLink :: 2;
}

PxAggregateType :: enum s32 {
    Generic :: 0;
    Static :: 1;
    Kinematic :: 2;
}

Px1DConstraintFlags :: enum_flags u16 {
    Spring :: 1 << 0;
    AccelerationSpring :: 1 << 1;
    Restitution :: 1 << 2;
    Keepbias :: 1 << 3;
    OutputForce :: 1 << 4;
    HasDriveLimit :: 1 << 5;
    AngularConstraint :: 1 << 6;
    DriveRow :: 1 << 7;
}

PxConstraintSolveHint :: enum s32 {
    None :: 0;
    Acceleration1 :: 256;
    SlerpSpring :: 258;
    Acceleration2 :: 512;
    Acceleration3 :: 768;
    RotationalEquality :: 1024;
    RotationalInequality :: 1025;
    Equality :: 2048;
    Inequality :: 2049;
}

PxConstraintVisualizationFlag :: enum_flags s32 {
    LocalFrames :: 1;
    Limits :: 2;
}

PxPvdUpdateType :: enum s32 {
    CreateInstance :: 0;
    ReleaseInstance :: 1;
    UpdateAllProperties :: 2;
    UpdateSimProperties :: 3;
}

ConstraintType :: enum s32 {
    ContactConstraint :: 0;
    JointConstraint :: 1;
}

BodyState :: enum_flags s32 {
    DynamicBody :: 1;
    StaticBody :: 2;
    KinematicBody :: 4;
    Articulation :: 8;
}

PxArticulationAxis :: enum s32 {
    Twist :: 0;
    Swing1 :: 1;
    Swing2 :: 2;
    X :: 3;
    Y :: 4;
    Z :: 5;
    Count :: 6;
}

PxArticulationMotions :: enum_flags u8 {
    Locked :: 0;
    Limited :: 1 << 0;
    Free :: 1 << 1;
}

PxArticulationJointType :: enum s32 {
    Fix :: 0;
    Prismatic :: 1;
    Revolute :: 2;
    RevoluteUnwrapped :: 3;
    Spherical :: 4;
    Undefined :: 5;
}

PxArticulationFlags :: enum_flags u8 {
    FixBase :: 1 << 0;
    DriveLimitsAreForces :: 1 << 1;
    DisableSelfCollision :: 1 << 2;
    ComputeJointForces :: 1 << 3;
}

PxArticulationDriveType :: enum s32 {
    Force :: 0;
    Acceleration :: 1;
    Target :: 2;
    Velocity :: 3;
    None :: 4;
}

PxArticulationGpuDataType :: enum s32 {
    JointPosition :: 0;
    JointVelocity :: 1;
    JointAcceleration :: 2;
    JointForce :: 3;
    JointSolverForce :: 4;
    JointTargetVelocity :: 5;
    JointTargetPosition :: 6;
    SensorForce :: 7;
    RootTransform :: 8;
    RootVelocity :: 9;
    LinkTransform :: 10;
    LinkVelocity :: 11;
    LinkForce :: 12;
    LinkTorque :: 13;
    FixedTendon :: 14;
    FixedTendonJoint :: 15;
    SpatialTendon :: 16;
    SpatialTendonAttachment :: 17;
}

PxArticulationCacheFlags :: enum_flags u32 {
    Velocity :: 1 << 0;
    Acceleration :: 1 << 1;
    Position :: 1 << 2;
    Force :: 1 << 3;
    LinkVelocity :: 1 << 4;
    LinkAcceleration :: 1 << 5;
    RootTransform :: 1 << 6;
    RootVelocities :: 1 << 7;
    SensorForces :: 1 << 8;
    JointSolverForces :: 1 << 9;
    All :: PxArticulationCacheFlags.Velocity | PxArticulationCacheFlags.Acceleration | PxArticulationCacheFlags.Position | PxArticulationCacheFlags.LinkVelocity | PxArticulationCacheFlags.LinkAcceleration | PxArticulationCacheFlags.RootTransform | PxArticulationCacheFlags.RootVelocities;
}

PxArticulationSensorFlags :: enum_flags u8 {
    ForwardDynamicsForces :: 1 << 0;
    ConstraintSolverForces :: 1 << 1;
    WorldFrame :: 1 << 2;
}

PxArticulationKinematicFlags :: enum_flags u8 {
    Position :: 1 << 0;
    Velocity :: 1 << 1;
}

PxShapeFlags :: enum_flags u8 {
    SimulationShape :: 1 << 0;
    SceneQueryShape :: 1 << 1;
    TriggerShape :: 1 << 2;
    Visualization :: 1 << 3;
}

PxForceMode :: enum s32 {
    Force :: 0;
    Impulse :: 1;
    VelocityChange :: 2;
    Acceleration :: 3;
}

PxRigidBodyFlags :: enum_flags u16 {
    Kinematic :: 1 << 0;
    UseKinematicTargetForSceneQueries :: 1 << 1;
    EnableCcd :: 1 << 2;
    EnableCcdFriction :: 1 << 3;
    EnableSpeculativeCcd :: 1 << 4;
    EnablePoseIntegrationPreview :: 1 << 5;
    EnableCcdMaxContactImpulse :: 1 << 6;
    RetainAccelerations :: 1 << 7;
    ForceKineKineNotifications :: 1 << 8;
    ForceStaticKineNotifications :: 1 << 9;
    EnableGyroscopicForces :: 1 << 10;
}

PxConstraintFlags :: enum_flags u16 {
    Broken :: 1 << 0;
    ProjectToActor0 :: 1 << 1;
    ProjectToActor1 :: 1 << 2;
    Projection :: PxConstraintFlags.ProjectToActor0 | PxConstraintFlags.ProjectToActor1;
    CollisionEnabled :: 1 << 3;
    Visualization :: 1 << 4;
    DriveLimitsAreForces :: 1 << 5;
    ImprovedSlerp :: 1 << 7;
    DisablePreprocessing :: 1 << 8;
    EnableExtendedLimits :: 1 << 9;
    GpuCompatible :: 1 << 10;
    AlwaysUpdate :: 1 << 11;
    DisableConstraint :: 1 << 12;
}

PxContactPatchFlags :: enum_flags s32 {
    HasFaceIndices :: 1;
    Modifiable :: 2;
    ForceNoResponse :: 4;
    HasModifiedMassRatios :: 8;
    HasTargetVelocity :: 16;
    HasMaxImpulse :: 32;
    RegeneratePatches :: 64;
    CompressedModifiedContact :: 128;
}

StreamFormat :: enum s32 {
    SimpleStream :: 0;
    ModifiableStream :: 1;
    CompressedModifiableStream :: 2;
}

PxDeletionEventFlags :: enum_flags u8 {
    UserRelease :: 1 << 0;
    MemoryRelease :: 1 << 1;
}

PxPairFlags :: enum_flags u16 {
    SolveContact :: 1 << 0;
    ModifyContacts :: 1 << 1;
    NotifyTouchFound :: 1 << 2;
    NotifyTouchPersists :: 1 << 3;
    NotifyTouchLost :: 1 << 4;
    NotifyTouchCcd :: 1 << 5;
    NotifyThresholdForceFound :: 1 << 6;
    NotifyThresholdForcePersists :: 1 << 7;
    NotifyThresholdForceLost :: 1 << 8;
    NotifyContactPoints :: 1 << 9;
    DetectDiscreteContact :: 1 << 10;
    DetectCcdContact :: 1 << 11;
    PreSolverVelocity :: 1 << 12;
    PostSolverVelocity :: 1 << 13;
    ContactEventPose :: 1 << 14;
    NextFree :: 1 << 15;
    ContactDefault :: PxPairFlags.SolveContact | PxPairFlags.DetectDiscreteContact;
    TriggerDefault :: PxPairFlags.NotifyTouchFound | PxPairFlags.NotifyTouchLost | PxPairFlags.DetectDiscreteContact;
}

PxFilterFlags :: enum_flags u16 {
    Default :: 0;
    Kill :: 1 << 0;
    Suppress :: 1 << 1;
    Callback :: 1 << 2;
    Notify :: PxFilterFlags.Callback;
}

PxFilterObjectType :: enum s32 {
    RigidStatic :: 0;
    RigidDynamic :: 1;
    Articulation :: 2;
    Particlesystem :: 3;
    Softbody :: 4;
    Femcloth :: 5;
    Hairsystem :: 6;
    MaxTypeCount :: 16;
    Undefined :: 15;
}

PxFilterObjectFlag :: enum_flags s32 {
    Kinematic :: 16;
    Trigger :: 32;
}

PxPairFilteringMode :: enum s32 {
    Keep :: 0;
    Suppress :: 1;
    Kill :: 2;
}

PxDataAccessFlags :: enum_flags u8 {
    Readable :: 1 << 0;
    Writable :: 1 << 1;
    Device :: 1 << 2;
}

PxMaterialFlags :: enum_flags u16 {
    DisableFriction :: 1 << 0;
    DisableStrongFriction :: 1 << 1;
    ImprovedPatchFriction :: 1 << 2;
    CompliantContact :: 1 << 3;
}

PxRigidDynamicLockFlags :: enum_flags u8 {
    LockLinearX :: 1 << 0;
    LockLinearY :: 1 << 1;
    LockLinearZ :: 1 << 2;
    LockAngularX :: 1 << 3;
    LockAngularY :: 1 << 4;
    LockAngularZ :: 1 << 5;
}

PxSceneFlags :: enum_flags u32 {
    EnableActiveActors :: 1 << 0;
    EnableCcd :: 1 << 1;
    DisableCcdResweep :: 1 << 2;
    EnablePcm :: 1 << 6;
    DisableContactReportBufferResize :: 1 << 7;
    DisableContactCache :: 1 << 8;
    RequireRwLock :: 1 << 9;
    EnableStabilization :: 1 << 10;
    EnableAveragePoint :: 1 << 11;
    ExcludeKinematicsFromActiveActors :: 1 << 12;
    EnableGpuDynamics :: 1 << 13;
    EnableEnhancedDeterminism :: 1 << 14;
    EnableFrictionEveryIteration :: 1 << 15;
    SuppressReadback :: 1 << 16;
    ForceReadback :: 1 << 17;
    MutableFlags :: EnableActiveActors | ExcludeKinematicsFromActiveActors | SuppressReadback;
}

PxConvexFlags :: enum_flags u16 {
    E16BitIndices :: 1 << 0;
    ComputeConvex :: 1 << 1;
    CheckZeroAreaTriangles :: 1 << 2;
    QuantizeInput :: 1 << 3;
    DisableMeshValidation :: 1 << 4;
    PlaneShifting :: 1 << 5;
    FastInertiaComputation :: 1 << 6;
    GpuCompatible :: 1 << 7;
    ShiftVertices :: 1 << 8;
}

PxConvexMeshCookingResult :: enum s32 {
    /// Convex mesh cooking succeeded.
    Success :: 0;
    /// Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.
    ZeroAreaTestFailed :: 1;
    /// Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
    /// The produced hull does not contain all input vertices. Try to simplify the input vertices
    /// or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.
    PolygonsLimitReached :: 2;
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure :: 3;
}

PxDefaultCpuDispatcherWaitForWorkMode :: enum s32 {
    WaitForWork :: 0;
    YieldThread :: 1;
    YieldProcessor :: 2;
}

PxControllerShapeType :: enum s32 {
    /// A box controller.
    Box :: 0;
    /// A capsule controller
    Capsule :: 1;
    /// A capsule controller
    ForceDword :: 2147483647;
}

PxControllerNonWalkableMode :: enum s32 {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing :: 0;
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding :: 1;
}

PxControllerCollisionFlags :: enum_flags u8 {
    CollisionSides :: 1 << 0;
    CollisionUp :: 1 << 1;
    CollisionDown :: 1 << 2;
}

PxCapsuleClimbingMode :: enum s32 {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy :: 0;
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained :: 1;
    Last :: 2;
}

PxControllerBehaviorFlags :: enum_flags u8 {
    /// Controller can ride on touched object (i.e. when this touched object is moving horizontally).
    ///
    /// The CCT vs. CCT case is not supported.
    CctCanRideOnObject :: 1;
    /// Controller should slide on touched object
    CctSlide :: 2;
    /// Disable all code dealing with controllers riding on objects, let users define it outside of the SDK.
    CctUserDefinedRide :: 4;
}

PxQueryHitType :: enum s32 {
    /// the query should ignore this shape
    None :: 0;
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch :: 1;
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block :: 2;
}

PxQueryFlags :: enum_flags u16 {
    Static :: 1 << 0;
    Dynamic :: 1 << 1;
    Prefilter :: 1 << 2;
    Postfilter :: 1 << 3;
    AnyHit :: 1 << 4;
    NoBlock :: 1 << 5;
    DisableHardcodedFilter :: 1 << 6;
    Reserved :: 1 << 15;
}

PxBroadPhaseType :: enum s32 {
    /// 3-axes sweep-and-prune
    Sap :: 0;
    /// Multi box pruning
    Mbp :: 1;
    /// Automatic box pruning
    Abp :: 2;
    /// Parallel automatic box pruning
    Pabp :: 3;
    /// GPU broad phase
    Gpu :: 4;
    Last :: 5;
}

PxCombineMode :: enum s32 {
    /// Average: (a + b)/2
    Average :: 0;
    /// Minimum: minimum(a,b)
    Min :: 1;
    /// Multiply: a*b
    Multiply :: 2;
    /// Maximum: maximum(a,b)
    Max :: 3;
}

PxD6Axis :: enum s32 {
    /// motion along the X axis
    X :: 0;
    /// motion along the Y axis
    Y :: 1;
    /// motion along the Z axis
    Z :: 2;
    /// motion around the X axis
    Twist :: 3;
    /// motion around the Y axis
    Swing1 :: 4;
    /// motion around the Z axis
    Swing2 :: 5;
    Count :: 6;
}

PxD6Motion :: enum s32 {
    /// The DOF is locked, it does not allow relative motion.
    Locked :: 0;
    /// The DOF is limited, it only allows motion within a specific range.
    Limited :: 1;
    /// The DOF is free and has its full range of motion.
    Free :: 2;
}

PxRevoluteJointFlags :: enum_flags u16 {
    LimitEnabled :: 1 << 0;
    DriveEnabled :: 1 << 1;
    DriveFreespin :: 1 << 2;
}

PxDistanceJointFlags :: enum_flags u16 {
    MaxDistanceEnabled :: 1 << 1;
    MinDistanceEnabled :: 1 << 2;
    SpringEnabled :: 1 << 3;
}



__clang__ :: 1;
NDEBUG :: 1;
PxMat34 :: struct {}

PxAllocatorCallback :: struct {
    vtable_: *void;
}

PxAssertHandler :: struct {
    vtable_: *void;
}

PxFoundation :: struct {
    vtable_: *void;
}

PxAllocator :: struct {
    structgen_pad0: [1] u8;
}

PxRawAllocator :: struct {
    structgen_pad0: [1] u8;
}

PxVirtualAllocatorCallback :: struct {
    vtable_: *void;
}

PxVirtualAllocator :: struct {
    structgen_pad0: [16] u8;
}

PxUserAllocated :: struct {
    structgen_pad0: [1] u8;
}

PxTempAllocatorChunk :: union {
    mNext:  *PxTempAllocatorChunk;
    mIndex: u32;
    mPad:   [16] u8;
}

PxTempAllocator :: struct {
    structgen_pad0: [1] u8;
}

PxLogTwo :: struct {}
PxUnConst :: struct {}
PxBitAndByte :: struct {
    structgen_pad0: [1] u8;
}

PxBitMap :: struct {
    structgen_pad0: [16] u8;
}

PxVec3Padded :: struct {
    x:       float;
    y:       float;
    z:       float;
    padding: u32;
}

PxTransform :: struct {
    q: Quaternion;
    p: Vector3;
}

PxTransformPadded :: struct {
    transform: PxTransform;
    padding:   u32;
}

PxMat33 :: struct {
    column0: Vector3;
    column1: Vector3;
    column2: Vector3;
}

PxBounds3 :: struct {
    minimum: Vector3;
    maximum: Vector3;
}

PxErrorCallback :: struct {
    vtable_: *void;
}

PxAllocationListener :: struct {
    vtable_: *void;
}

PxBroadcastingAllocator :: struct {
    structgen_pad0: [176] u8;
}

PxBroadcastingErrorCallback :: struct {
    structgen_pad0: [160] u8;
}

PxHash :: struct {}
PxInputStream :: struct {
    vtable_: *void;
}

PxInputData :: struct {
    vtable_: *void;
}

PxOutputStream :: struct {
    vtable_: *void;
}

PxMat44 :: struct {
    column0: Vector4;
    column1: Vector4;
    column2: Vector4;
    column3: Vector4;
}

Interpolation :: struct {
    structgen_pad0: [1] u8;
}

PxMutexImpl :: struct {
    structgen_pad0: [1] u8;
}

PxReadWriteLock :: struct {
    structgen_pad0: [8] u8;
}

PxProfilerCallback :: struct {
    vtable_: *void;
}

PxProfileScoped :: struct {
    mCallback:      *PxProfilerCallback;
    mEventName:     *u8;
    mProfilerData:  *void;
    mContextId:     u64;
    mDetached:      bool;
    structgen_pad0: [7] u8;
}

PxSListEntry :: struct {
    structgen_pad0: [16] u8;
}

PxSListImpl :: struct {
    structgen_pad0: [1] u8;
}

PxSyncImpl :: struct {
    structgen_pad0: [1] u8;
}

PxRunnable :: struct {
    vtable_: *void;
}

PxCounterFrequencyToTensOfNanos :: struct {
    mNumerator:   u64;
    mDenominator: u64;
}

PxTime :: struct {
    structgen_pad0: [8] u8;
}

PxStridedData :: struct {
    stride:         u32;
    structgen_pad0: [4] u8;
    data:           *void;
}

PxBoundedData :: struct {
    stride:         u32;
    structgen_pad0: [4] u8;
    data:           *void;
    count:          u32;
    structgen_pad1: [4] u8;
}

PxDebugPoint :: struct {
    pos:   Vector3;
    color: u32;
}

PxDebugLine :: struct {
    pos0:   Vector3;
    color0: u32;
    pos1:   Vector3;
    color1: u32;
}

PxDebugTriangle :: struct {
    pos0:   Vector3;
    color0: u32;
    pos1:   Vector3;
    color1: u32;
    pos2:   Vector3;
    color2: u32;
}

PxDebugText :: struct {
    position:       Vector3;
    size:           float;
    color:          u32;
    structgen_pad0: [4] u8;
    _string:        *u8;
}

PxRenderBuffer :: struct {
    vtable_: *void;
}

PxProcessPxBaseCallback :: struct {
    vtable_: *void;
}

PxSerializationContext :: struct {
    vtable_: *void;
}

PxDeserializationContext :: struct {
    structgen_pad0: [16] u8;
}

PxSerializationRegistry :: struct {
    vtable_: *void;
}

PxCollection :: struct {
    vtable_: *void;
}

PxTypeInfo :: struct {}

PxFEMSoftBodyMaterial :: struct {}
PxFEMClothMaterial :: struct {}
PxPBDMaterial :: struct {}
PxFLIPMaterial :: struct {}
PxMPMMaterial :: struct {}
PxCustomMaterial :: struct {}

PxBVH33TriangleMesh :: struct {}

PxParticleSystem :: struct {}
PxPBDParticleSystem :: struct {}
PxFLIPParticleSystem :: struct {}
PxMPMParticleSystem :: struct {}
PxCustomParticleSystem :: struct {}
PxSoftBody :: struct {}
PxFEMCloth :: struct {}
PxHairSystem :: struct {}
PxParticleBuffer :: struct {}
PxParticleAndDiffuseBuffer :: struct {}
PxParticleClothBuffer :: struct {}
PxParticleRigidBuffer :: struct {}
PxBase :: struct {
    structgen_pad0: [16] u8;
}

PxRefCounted :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as base: PxBase = ---;
}

PxTolerancesScale :: struct {
    length: float;
    speed:  float;
}

PxStringTable :: struct {
    vtable_: *void;
}

PxSerializer :: struct {
    vtable_: *void;
}

PxMetaDataEntry :: struct {
    type:       *u8;
    name:       *u8;
    offset:     u32;
    size:       u32;
    count:      u32;
    offsetSize: u32;
    flags:      u32;
    alignment:  u32;
}

PxInsertionCallback :: struct {
    vtable_: *void;
}

PxTaskManager :: struct {
    vtable_: *void;
}

PxCpuDispatcher :: struct {
    vtable_: *void;
}

PxBaseTask :: struct {
    structgen_pad0: [24] u8;
}

PxTask :: struct {
    structgen_pad0: [32] u8;
}

PxLightCpuTask :: struct {
    structgen_pad0: [40] u8;
}

PxGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

PxBoxGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    halfExtents:    Vector3;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxBVHRaycastCallback :: struct {
    vtable_: *void;
}

PxBVHOverlapCallback :: struct {
    vtable_: *void;
}

PxBVHTraversalCallback :: struct {
    vtable_: *void;
}

PxBVH :: struct {
    structgen_pad0: [16] u8;
}

PxCapsuleGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    radius:         float;
    halfHeight:     float;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxHullPolygon :: struct {
    mPlane:     [4] float;
    mNbVerts:   u16;
    mIndexBase: u16;
}

PxConvexMesh :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxMeshScale :: struct {
    scale:    Vector3;
    rotation: Quaternion;
}

PxConvexMeshGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    scale:          PxMeshScale;
    structgen_pad1: [4] u8;
    convexMesh:     *PxConvexMesh;
    meshFlags:      u8;
    structgen_pad2: [7] u8;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxSphereGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    radius:         float;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxPlaneGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxTriangleMeshGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    scale:          PxMeshScale;
    meshFlags:      u8;
    structgen_pad1: [3] u8;
    triangleMesh:   *PxTriangleMesh;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxHeightFieldGeometry :: struct {
    structgen_pad0:   [4] u8;
    mTypePadding:     float;
    heightField:      *PxHeightField;
    heightScale:      float;
    rowScale:         float;
    columnScale:      float;
    heightFieldFlags: u8;
    structgen_pad1:   [3] u8;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxParticleSystemGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    mSolverType:    s32;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxHairSystemGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxTetrahedronMeshGeometry :: struct {
    structgen_pad0:  [4] u8;
    mTypePadding:    float;
    tetrahedronMesh: *PxTetrahedronMesh;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxQueryHit :: struct {
    faceIndex: u32;
}

PxLocationHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
}

PxGeomRaycastHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
    u:              float;
    v:              float;
}

PxGeomOverlapHit :: struct {
    faceIndex: u32;
}

PxGeomSweepHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
}

PxGeomIndexPair :: struct {
    id0: u32;
    id1: u32;
}

PxQueryThreadContext :: struct {
    structgen_pad0: [1] u8;
}

PxContactBuffer :: struct {}
PxRenderOutput :: struct {}

PxCustomGeometryType :: struct {
    structgen_pad0: [4] u8;
}

PxCustomGeometryCallbacks :: struct {
    vtable_: *void;
}

PxCustomGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    callbacks:      *PxCustomGeometryCallbacks;
#place structgen_pad0; #as geometry: PxGeometry = ---;
}

PxGeometryHolder :: struct {
    structgen_pad0: [56] u8;
}

PxGeometryQuery :: struct {
    structgen_pad0: [1] u8;
}

PxHeightFieldSample :: struct {
    height:         s16;
    materialIndex0: PxBitAndByte;
    materialIndex1: PxBitAndByte;
}

PxHeightField :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxHeightFieldDesc :: struct {
    nbRows:              u32;
    nbColumns:           u32;
    format:              s32;
    structgen_pad0:      [4] u8;
    samples:             PxStridedData;
    convexEdgeThreshold: float;
    flags:               u16;
    structgen_pad1:      [2] u8;
}

PxMeshQuery :: struct {
    structgen_pad0: [1] u8;
}

PxSimpleTriangleMesh :: struct {
    points:         PxBoundedData;
    triangles:      PxBoundedData;
    flags:          u16;
    structgen_pad0: [6] u8;
}

PxTriangle :: struct {
    verts: [3] Vector3;
}

PxTrianglePadded :: struct {
    verts:   [3] Vector3;
    padding: u32;
}

PxTriangleMesh :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxBVH34TriangleMesh :: struct {
    structgen_pad0: [16] u8;
}

PxTetrahedron :: struct {
    verts: [4] Vector3;
}

PxSoftBodyAuxData :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxTetrahedronMesh :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxSoftBodyMesh :: struct {
    structgen_pad0: [16] u8;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxCollisionMeshMappingData :: struct {
    structgen_pad0: [8] u8;
}

PxSoftBodyCollisionData :: struct {
    structgen_pad0: [1] u8;
}

PxTetrahedronMeshData :: struct {
    structgen_pad0: [1] u8;
}

PxSoftBodySimulationData :: struct {
    structgen_pad0: [1] u8;
}

PxCollisionTetrahedronMeshData :: struct {
    structgen_pad0: [8] u8;
}

PxSimulationTetrahedronMeshData :: struct {
    structgen_pad0: [8] u8;
}

PxActor :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxAggregate :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxSpringModifiers :: struct {
    stiffness:      float;
    damping:        float;
    structgen_pad0: [8] u8;
}

PxRestitutionModifiers :: struct {
    restitution:       float;
    velocityThreshold: float;
    structgen_pad0:    [8] u8;
}

Px1DConstraintMods :: union {
    spring: PxSpringModifiers;
    bounce: PxRestitutionModifiers;
}

Px1DConstraint :: struct {
    linear0:        Vector3;
    geometricError: float;
    angular0:       Vector3;
    velocityTarget: float;
    linear1:        Vector3;
    minImpulse:     float;
    angular1:       Vector3;
    maxImpulse:     float;
    mods:           Px1DConstraintMods;
    forInternalUse: float;
    flags:          u16;
    solveHint:      u16;
    structgen_pad0: [8] u8;
}

PxConstraintInvMassScale :: struct {
    linear0:  float;
    angular0: float;
    linear1:  float;
    angular1: float;
}

PxConstraintVisualizer :: struct {
    vtable_: *void;
}

PxConstraintConnector :: struct {
    vtable_: *void;
}

PxContactPoint :: struct {
    normal:             Vector3;
    separation:         float;
    point:              Vector3;
    maxImpulse:         float;
    targetVel:          Vector3;
    staticFriction:     float;
    materialFlags:      u8;
    structgen_pad0:     [3] u8;
    internalFaceIndex1: u32;
    dynamicFriction:    float;
    restitution:        float;
    damping:            float;
    structgen_pad1:     [12] u8;
}

PxSolverBody :: struct {
    linearVelocity:            Vector3;
    maxSolverNormalProgress:   u16;
    maxSolverFrictionProgress: u16;
    angularState:              Vector3;
    solverProgress:            u32;
}

PxSolverBodyData :: struct {
    linearVelocity:    Vector3;
    invMass:           float;
    angularVelocity:   Vector3;
    reportThreshold:   float;
    sqrtInvInertia:    PxMat33;
    penBiasClamp:      float;
    nodeIndex:         u32;
    maxContactImpulse: float;
    body2World:        PxTransform;
    pad:               u16;
    structgen_pad0:    [2] u8;
}

PxConstraintBatchHeader :: struct {
    startIndex:     u32;
    stride:         u16;
    constraintType: u16;
}

PxSolverConstraintDesc :: struct {
    structgen_pad0:         [16] u8;
    bodyADataIndex:         u32;
    bodyBDataIndex:         u32;
    linkIndexA:             u32;
    linkIndexB:             u32;
    constraint:             *u8;
    writeBack:              *void;
    progressA:              u16;
    progressB:              u16;
    constraintLengthOver16: u16;
    padding:                [10] u8;
}

PxSolverConstraintPrepDescBase :: struct {
    invMassScales:  PxConstraintInvMassScale;
    desc:           *PxSolverConstraintDesc;
    body0:          *PxSolverBody;
    body1:          *PxSolverBody;
    data0:          *PxSolverBodyData;
    data1:          *PxSolverBodyData;
    bodyFrame0:     PxTransform;
    bodyFrame1:     PxTransform;
    bodyState0:     s32;
    bodyState1:     s32;
    structgen_pad0: [8] u8;
}

PxSolverConstraintPrepDesc :: struct {
    invMassScales:        PxConstraintInvMassScale;
    desc:                 *PxSolverConstraintDesc;
    body0:                *PxSolverBody;
    body1:                *PxSolverBody;
    data0:                *PxSolverBodyData;
    data1:                *PxSolverBodyData;
    bodyFrame0:           PxTransform;
    bodyFrame1:           PxTransform;
    bodyState0:           s32;
    bodyState1:           s32;
    structgen_pad0:       [8] u8;
    rows:                 *Px1DConstraint;
    numRows:              u32;
    linBreakForce:        float;
    angBreakForce:        float;
    minResponseThreshold: float;
    writeback:            *void;
    disablePreprocessing: bool;
    improvedSlerp:        bool;
    driveLimitsAreForces: bool;
    extendedLimits:       bool;
    disableConstraint:    bool;
    structgen_pad1:       [3] u8;
    body0WorldOffset:     PxVec3Padded;
    structgen_pad2:       [8] u8;
}

PxSolverContactDesc :: struct {
    invMassScales:           PxConstraintInvMassScale;
    desc:                    *PxSolverConstraintDesc;
    body0:                   *PxSolverBody;
    body1:                   *PxSolverBody;
    data0:                   *PxSolverBodyData;
    data1:                   *PxSolverBodyData;
    bodyFrame0:              PxTransform;
    bodyFrame1:              PxTransform;
    bodyState0:              s32;
    bodyState1:              s32;
    shapeInteraction:        *void;
    contacts:                *PxContactPoint;
    numContacts:             u32;
    hasMaxImpulse:           bool;
    disableStrongFriction:   bool;
    hasForceThresholds:      bool;
    structgen_pad0:          [1] u8;
    restDistance:            float;
    maxCCDSeparation:        float;
    frictionPtr:             *u8;
    frictionCount:           u8;
    structgen_pad1:          [7] u8;
    contactForces:           *float;
    startFrictionPatchIndex: u32;
    numFrictionPatches:      u32;
    startContactPatchIndex:  u32;
    numContactPatches:       u16;
    axisConstraintCount:     u16;
    offsetSlop:              float;
    structgen_pad2:          [12] u8;
}

PxConstraintAllocator :: struct {
    vtable_: *void;
}

PxArticulationLimit :: struct {
    low:  float;
    high: float;
}

PxArticulationDrive :: struct {
    stiffness: float;
    damping:   float;
    maxForce:  float;
    driveType: s32;
}

PxTGSSolverBodyVel :: struct {
    linearVelocity:       Vector3;
    nbStaticInteractions: u16;
    maxDynamicPartition:  u16;
    angularVelocity:      Vector3;
    partitionMask:        u32;
    deltaAngDt:           Vector3;
    maxAngVel:            float;
    deltaLinDt:           Vector3;
    lockFlags:            u16;
    isKinematic:          bool;
    pad:                  u8;
}

PxTGSSolverBodyTxInertia :: struct {
    deltaBody2World: PxTransform;
    sqrtInvInertia:  PxMat33;
}

PxTGSSolverBodyData :: struct {
    originalLinearVelocity:  Vector3;
    maxContactImpulse:       float;
    originalAngularVelocity: Vector3;
    penBiasClamp:            float;
    invMass:                 float;
    nodeIndex:               u32;
    reportThreshold:         float;
    pad:                     u32;
}

PxTGSSolverConstraintPrepDescBase :: struct {
    invMassScales:  PxConstraintInvMassScale;
    desc:           *PxSolverConstraintDesc;
    body0:          *PxTGSSolverBodyVel;
    body1:          *PxTGSSolverBodyVel;
    body0TxI:       *PxTGSSolverBodyTxInertia;
    body1TxI:       *PxTGSSolverBodyTxInertia;
    bodyData0:      *PxTGSSolverBodyData;
    bodyData1:      *PxTGSSolverBodyData;
    bodyFrame0:     PxTransform;
    bodyFrame1:     PxTransform;
    bodyState0:     s32;
    bodyState1:     s32;
    structgen_pad0: [8] u8;
}

PxTGSSolverConstraintPrepDesc :: struct {
    invMassScales:        PxConstraintInvMassScale;
    desc:                 *PxSolverConstraintDesc;
    body0:                *PxTGSSolverBodyVel;
    body1:                *PxTGSSolverBodyVel;
    body0TxI:             *PxTGSSolverBodyTxInertia;
    body1TxI:             *PxTGSSolverBodyTxInertia;
    bodyData0:            *PxTGSSolverBodyData;
    bodyData1:            *PxTGSSolverBodyData;
    bodyFrame0:           PxTransform;
    bodyFrame1:           PxTransform;
    bodyState0:           s32;
    bodyState1:           s32;
    rows:                 *Px1DConstraint;
    numRows:              u32;
    linBreakForce:        float;
    angBreakForce:        float;
    minResponseThreshold: float;
    writeback:            *void;
    disablePreprocessing: bool;
    improvedSlerp:        bool;
    driveLimitsAreForces: bool;
    extendedLimits:       bool;
    disableConstraint:    bool;
    structgen_pad0:       [3] u8;
    body0WorldOffset:     PxVec3Padded;
    cA2w:                 PxVec3Padded;
    cB2w:                 PxVec3Padded;
}

PxTGSSolverContactDesc :: struct {
    invMassScales:           PxConstraintInvMassScale;
    desc:                    *PxSolverConstraintDesc;
    body0:                   *PxTGSSolverBodyVel;
    body1:                   *PxTGSSolverBodyVel;
    body0TxI:                *PxTGSSolverBodyTxInertia;
    body1TxI:                *PxTGSSolverBodyTxInertia;
    bodyData0:               *PxTGSSolverBodyData;
    bodyData1:               *PxTGSSolverBodyData;
    bodyFrame0:              PxTransform;
    bodyFrame1:              PxTransform;
    bodyState0:              s32;
    bodyState1:              s32;
    shapeInteraction:        *void;
    contacts:                *PxContactPoint;
    numContacts:             u32;
    hasMaxImpulse:           bool;
    disableStrongFriction:   bool;
    hasForceThresholds:      bool;
    structgen_pad0:          [1] u8;
    restDistance:            float;
    maxCCDSeparation:        float;
    frictionPtr:             *u8;
    frictionCount:           u8;
    structgen_pad1:          [7] u8;
    contactForces:           *float;
    startFrictionPatchIndex: u32;
    numFrictionPatches:      u32;
    startContactPatchIndex:  u32;
    numContactPatches:       u16;
    axisConstraintCount:     u16;
    maxImpulse:              float;
    torsionalPatchRadius:    float;
    minTorsionalPatchRadius: float;
    offsetSlop:              float;
}

PxArticulationTendonLimit :: struct {
    lowLimit:  float;
    highLimit: float;
}

PxArticulationAttachment :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxArticulationTendonJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxArticulationTendon :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxArticulationSpatialTendon :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxArticulationFixedTendon :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxSpatialForce :: struct {
    force:  Vector3;
    pad0:   float;
    torque: Vector3;
    pad1:   float;
}

PxSpatialVelocity :: struct {
    linear:  Vector3;
    pad0:    float;
    angular: Vector3;
    pad1:    float;
}

PxArticulationRootLinkData :: struct {
    transform:     PxTransform;
    worldLinVel:   Vector3;
    worldAngVel:   Vector3;
    worldLinAccel: Vector3;
    worldAngAccel: Vector3;
}

PxArticulationCache :: struct {
    externalForces:    *PxSpatialForce;
    denseJacobian:     *float;
    massMatrix:        *float;
    jointVelocity:     *float;
    jointAcceleration: *float;
    jointPosition:     *float;
    jointForce:        *float;
    jointSolverForces: *float;
    linkVelocity:      *PxSpatialVelocity;
    linkAcceleration:  *PxSpatialVelocity;
    rootLinkData:      *PxArticulationRootLinkData;
    sensorForces:      *PxSpatialForce;
    coefficientMatrix: *float;
    lambda:            *float;
    scratchMemory:     *void;
    scratchAllocator:  *void;
    version:           u32;
    structgen_pad0:    [4] u8;
}

PxArticulationSensor :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxArticulationReducedCoordinate :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxArticulationJointReducedCoordinate :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxShape :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxRigidActor :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as actor: PxActor = ---;
}

PxNodeIndex :: struct {
    structgen_pad0: [8] u8;
}

PxRigidBody :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as rigid_actor: PxRigidActor = ---;
}

PxArticulationLink :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as rigid_body: PxRigidBody = ---;
}

PxConeLimitedConstraint :: struct {
    mAxis:      Vector3;
    mAngle:     float;
    mLowLimit:  float;
    mHighLimit: float;
}

PxConeLimitParams :: struct {
    lowHighLimits: Vector4;
    axisAngle:     Vector4;
}

PxConstraintShaderTable :: struct {
    solverPrep:     *void;
    structgen_pad0: [8] u8;
    visualize:      *void;
    flag:           s32;
    structgen_pad1: [4] u8;
}

PxConstraint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxMassModificationProps :: struct {
    mInvMassScale0:    float;
    mInvInertiaScale0: float;
    mInvMassScale1:    float;
    mInvInertiaScale1: float;
}

PxContactPatch :: struct {
    mMassModification: PxMassModificationProps;
    normal:            Vector3;
    restitution:       float;
    dynamicFriction:   float;
    staticFriction:    float;
    damping:           float;
    startContactIndex: u16;
    nbContacts:        u8;
    materialFlags:     u8;
    internalFlags:     u16;
    materialIndex0:    u16;
    materialIndex1:    u16;
    pad:               [5] u16;
}

PxContact :: struct {
    contact:    Vector3;
    separation: float;
}

PxExtendedContact :: struct {
    contact:        Vector3;
    separation:     float;
    targetVelocity: Vector3;
    maxImpulse:     float;
}

PxModifiableContact :: struct {
    contact:         Vector3;
    separation:      float;
    targetVelocity:  Vector3;
    maxImpulse:      float;
    normal:          Vector3;
    restitution:     float;
    materialFlags:   u32;
    materialIndex0:  u16;
    materialIndex1:  u16;
    staticFriction:  float;
    dynamicFriction: float;
}

PxContactStreamIterator :: struct {
    zero:                   Vector3;
    structgen_pad0:         [4] u8;
    patch:                  *PxContactPatch;
    contact:                *PxContact;
    faceIndice:             *u32;
    totalPatches:           u32;
    totalContacts:          u32;
    nextContactIndex:       u32;
    nextPatchIndex:         u32;
    contactPatchHeaderSize: u32;
    contactPointSize:       u32;
    mStreamFormat:          s32;
    forceNoResponse:        u32;
    pointStepped:           bool;
    structgen_pad1:         [3] u8;
    hasFaceIndices:         u32;
}

PxGpuContactPair :: struct {
    contactPatches:     *u8;
    contactPoints:      *u8;
    contactForces:      *float;
    transformCacheRef0: u32;
    transformCacheRef1: u32;
    nodeIndex0:         PxNodeIndex;
    nodeIndex1:         PxNodeIndex;
    actor0:             *PxActor;
    actor1:             *PxActor;
    nbContacts:         u16;
    nbPatches:          u16;
    structgen_pad0:     [4] u8;
}

PxContactSet :: struct {
    structgen_pad0: [16] u8;
}

PxContactModifyPair :: struct {
    actor:     [2] *PxRigidActor;
    shape:     [2] *PxShape;
    transform: [2] PxTransform;
    contacts:  PxContactSet;
}

PxContactModifyCallback :: struct {
    vtable_: *void;
}

PxCCDContactModifyCallback :: struct {
    vtable_: *void;
}

PxDeletionListener :: struct {
    vtable_: *void;
}

PxBaseMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as ref_counted: PxRefCounted = ---;
}

PxFEMMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxFilterData :: struct {
    word0: u32;
    word1: u32;
    word2: u32;
    word3: u32;
}

PxSimulationFilterCallback :: struct {
    vtable_: *void;
}

PxParticleRigidFilterPair :: struct {
    mID0: u64;
    mID1: u64;
}

PxLockedData :: struct {
    vtable_: *void;
}

PxMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as base_material: PxBaseMaterial = ---;
}

PxGpuParticleBufferIndexPair :: struct {
    systemIndex: u32;
    bufferIndex: u32;
}

PxCudaContextManager :: struct {}
PxParticleRigidAttachment :: struct {}
PxParticleVolume :: struct {
    bound:                 PxBounds3;
    particleIndicesOffset: u32;
    numParticles:          u32;
}

PxDiffuseParticleParams :: struct {
    threshold:           float;
    lifetime:            float;
    airDrag:             float;
    bubbleDrag:          float;
    buoyancy:            float;
    kineticEnergyWeight: float;
    pressureWeight:      float;
    divergenceWeight:    float;
    collisionDecay:      float;
    useAccurateVelocity: bool;
    structgen_pad0:      [3] u8;
}

PxParticleSpring :: struct {
    ind0:      u32;
    ind1:      u32;
    length:    float;
    stiffness: float;
    damping:   float;
    pad:       float;
}

PxParticleMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxOmniPvd :: struct {}
PxPhysics :: struct {
    vtable_: *void;
}

PxActorShape :: struct {
    actor: *PxRigidActor;
    shape: *PxShape;
}

PxRaycastHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
    u:              float;
    v:              float;
    structgen_pad1: [4] u8;
    actor:          *PxRigidActor;
    shape:          *PxShape;
}

PxOverlapHit :: struct {
    faceIndex:      u32;
    structgen_pad0: [4] u8;
    actor:          *PxRigidActor;
    shape:          *PxShape;
}

PxSweepHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
    structgen_pad1: [4] u8;
    actor:          *PxRigidActor;
    shape:          *PxShape;
}

PxRaycastCallback :: struct {
    structgen_pad0: [8] u8;
    block:          PxRaycastHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *PxRaycastHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

PxOverlapCallback :: struct {
    structgen_pad0: [8] u8;
    block:          PxOverlapHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *PxOverlapHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

PxSweepCallback :: struct {
    structgen_pad0: [8] u8;
    block:          PxSweepHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *PxSweepHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

PxRaycastBuffer :: struct {
    structgen_pad0: [8] u8;
    block:          PxRaycastHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *PxRaycastHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

PxOverlapBuffer :: struct {
    structgen_pad0: [8] u8;
    block:          PxOverlapHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *PxOverlapHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

PxSweepBuffer :: struct {
    structgen_pad0: [8] u8;
    block:          PxSweepHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *PxSweepHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

PxQueryCache :: struct {
    shape:          *PxShape;
    actor:          *PxRigidActor;
    faceIndex:      u32;
    structgen_pad0: [4] u8;
}

PxQueryFilterData :: struct {
    data:           PxFilterData;
    flags:          u16;
    structgen_pad0: [2] u8;
}

PxQueryFilterCallback :: struct {
    vtable_: *void;
}

PxRigidDynamic :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as rigid_body: PxRigidBody = ---;
}

PxRigidStatic :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as rigid_actor: PxRigidActor = ---;
}

PxSceneQueryDesc :: struct {
    staticStructure:            s32;
    dynamicStructure:           s32;
    dynamicTreeRebuildRateHint: u32;
    dynamicTreeSecondaryPruner: s32;
    staticBVHBuildStrategy:     s32;
    dynamicBVHBuildStrategy:    s32;
    staticNbObjectsPerNode:     u32;
    dynamicNbObjectsPerNode:    u32;
    sceneQueryUpdateMode:       s32;
}

PxSceneQuerySystemBase :: struct {
    vtable_: *void;
}

PxSceneSQSystem :: struct {
    vtable_: *void;
}

PxSceneQuerySystem :: struct {
    vtable_: *void;
}

PxBroadPhaseRegion :: struct {
    mBounds:   PxBounds3;
    mUserData: *void;
}

PxBroadPhaseRegionInfo :: struct {
    mRegion:           PxBroadPhaseRegion;
    mNbStaticObjects:  u32;
    mNbDynamicObjects: u32;
    mActive:           bool;
    mOverlap:          bool;
    structgen_pad0:    [6] u8;
}

PxBroadPhaseCaps :: struct {
    mMaxNbRegions: u32;
}

PxBroadPhaseDesc :: struct {
    mType:                        s32;
    structgen_pad0:               [4] u8;
    mContextID:                   u64;
    structgen_pad1:               [8] u8;
    mFoundLostPairsCapacity:      u32;
    mDiscardStaticVsKinematic:    bool;
    mDiscardKinematicVsKinematic: bool;
    structgen_pad2:               [2] u8;
}

PxBroadPhaseUpdateData :: struct {
    mCreated:       *u32;
    mNbCreated:     u32;
    structgen_pad0: [4] u8;
    mUpdated:       *u32;
    mNbUpdated:     u32;
    structgen_pad1: [4] u8;
    mRemoved:       *u32;
    mNbRemoved:     u32;
    structgen_pad2: [4] u8;
    mBounds:        *PxBounds3;
    mGroups:        *u32;
    mDistances:     *float;
    mCapacity:      u32;
    structgen_pad3: [4] u8;
}

PxBroadPhasePair :: struct {
    mID0: u32;
    mID1: u32;
}

PxBroadPhaseResults :: struct {
    mNbCreatedPairs: u32;
    structgen_pad0:  [4] u8;
    mCreatedPairs:   *PxBroadPhasePair;
    mNbDeletedPairs: u32;
    structgen_pad1:  [4] u8;
    mDeletedPairs:   *PxBroadPhasePair;
}

PxBroadPhaseRegions :: struct {
    vtable_: *void;
}

PxBroadPhase :: struct {
    vtable_: *void;
}

PxAABBManager :: struct {
    vtable_: *void;
}

PxSceneLimits :: struct {
    maxNbActors:             u32;
    maxNbBodies:             u32;
    maxNbStaticShapes:       u32;
    maxNbDynamicShapes:      u32;
    maxNbAggregates:         u32;
    maxNbConstraints:        u32;
    maxNbRegions:            u32;
    maxNbBroadPhaseOverlaps: u32;
}

PxgDynamicsMemoryConfig :: struct {
    tempBufferCapacity:              u32;
    maxRigidContactCount:            u32;
    maxRigidPatchCount:              u32;
    heapCapacity:                    u32;
    foundLostPairsCapacity:          u32;
    foundLostAggregatePairsCapacity: u32;
    totalAggregatePairsCapacity:     u32;
    maxSoftBodyContacts:             u32;
    maxFemClothContacts:             u32;
    maxParticleContacts:             u32;
    collisionStackSize:              u32;
    maxHairContacts:                 u32;
}

PxSceneDesc :: struct {
    staticStructure:               s32;
    dynamicStructure:              s32;
    dynamicTreeRebuildRateHint:    u32;
    dynamicTreeSecondaryPruner:    s32;
    staticBVHBuildStrategy:        s32;
    dynamicBVHBuildStrategy:       s32;
    staticNbObjectsPerNode:        u32;
    dynamicNbObjectsPerNode:       u32;
    sceneQueryUpdateMode:          s32;
    gravity:                       Vector3;
    simulationEventCallback:       *PxSimulationEventCallback;
    contactModifyCallback:         *PxContactModifyCallback;
    ccdContactModifyCallback:      *PxCCDContactModifyCallback;
    filterShaderData:              *void;
    filterShaderDataSize:          u32;
    structgen_pad0:                [4] u8;
    filterShader:                  *void;
    filterCallback:                *PxSimulationFilterCallback;
    kineKineFilteringMode:         s32;
    staticKineFilteringMode:       s32;
    broadPhaseType:                s32;
    structgen_pad1:                [4] u8;
    broadPhaseCallback:            *PxBroadPhaseCallback;
    limits:                        PxSceneLimits;
    frictionType:                  s32;
    solverType:                    s32;
    bounceThresholdVelocity:       float;
    frictionOffsetThreshold:       float;
    frictionCorrelationDistance:   float;
    flags:                         u32;
    cpuDispatcher:                 *PxCpuDispatcher;
    structgen_pad2:                [8] u8;
    userData:                      *void;
    solverBatchSize:               u32;
    solverArticulationBatchSize:   u32;
    nbContactDataBlocks:           u32;
    maxNbContactDataBlocks:        u32;
    maxBiasCoefficient:            float;
    contactReportStreamBufferSize: u32;
    ccdMaxPasses:                  u32;
    ccdThreshold:                  float;
    ccdMaxSeparation:              float;
    wakeCounterResetValue:         float;
    sanityBounds:                  PxBounds3;
    gpuDynamicsConfig:             PxgDynamicsMemoryConfig;
    gpuMaxNumPartitions:           u32;
    gpuMaxNumStaticPartitions:     u32;
    gpuComputeVersion:             u32;
    contactPairSlabSize:           u32;
    sceneQuerySystem:              *PxSceneQuerySystem;
    structgen_pad3:                [8] u8;
}

PxSimulationStatistics :: struct {
    nbActiveConstraints:                 u32;
    nbActiveDynamicBodies:               u32;
    nbActiveKinematicBodies:             u32;
    nbStaticBodies:                      u32;
    nbDynamicBodies:                     u32;
    nbKinematicBodies:                   u32;
    nbShapes:                            [11] u32;
    nbAggregates:                        u32;
    nbArticulations:                     u32;
    nbAxisSolverConstraints:             u32;
    compressedContactSize:               u32;
    requiredContactConstraintMemory:     u32;
    peakConstraintMemory:                u32;
    nbDiscreteContactPairsTotal:         u32;
    nbDiscreteContactPairsWithCacheHits: u32;
    nbDiscreteContactPairsWithContacts:  u32;
    nbNewPairs:                          u32;
    nbLostPairs:                         u32;
    nbNewTouches:                        u32;
    nbLostTouches:                       u32;
    nbPartitions:                        u32;
    structgen_pad0:                      [4] u8;
    gpuMemParticles:                     u64;
    gpuMemSoftBodies:                    u64;
    gpuMemFEMCloths:                     u64;
    gpuMemHairSystems:                   u64;
    gpuMemHeap:                          u64;
    gpuMemHeapBroadPhase:                u64;
    gpuMemHeapNarrowPhase:               u64;
    gpuMemHeapSolver:                    u64;
    gpuMemHeapArticulation:              u64;
    gpuMemHeapSimulation:                u64;
    gpuMemHeapSimulationArticulation:    u64;
    gpuMemHeapSimulationParticles:       u64;
    gpuMemHeapSimulationSoftBody:        u64;
    gpuMemHeapSimulationFEMCloth:        u64;
    gpuMemHeapSimulationHairSystem:      u64;
    gpuMemHeapParticles:                 u64;
    gpuMemHeapSoftBodies:                u64;
    gpuMemHeapFEMCloths:                 u64;
    gpuMemHeapHairSystems:               u64;
    gpuMemHeapOther:                     u64;
    nbBroadPhaseAdds:                    u32;
    nbBroadPhaseRemoves:                 u32;
    nbDiscreteContactPairs:              [11] [11] u32;
    nbCCDPairs:                          [11] [11] u32;
    nbModifiedContactPairs:              [11] [11] u32;
    nbTriggerPairs:                      [11] [11] u32;
}

PxGpuBodyData :: struct {
    quat:   Quaternion;
    pos:    Vector4;
    linVel: Vector4;
    angVel: Vector4;
}

PxGpuActorPair :: struct {
    srcIndex:       u32;
    structgen_pad0: [4] u8;
    nodeIndex:      PxNodeIndex;
}

PxIndexDataPair :: struct {
    index:          u32;
    structgen_pad0: [4] u8;
    data:           *void;
}

PxPvdSceneClient :: struct {
    vtable_: *void;
}

PxDominanceGroupPair :: struct {
    dominance0: u8;
    dominance1: u8;
}

PxBroadPhaseCallback :: struct {
    vtable_: *void;
}

PxScene :: struct {
    structgen_pad0: [8] u8;
    userData:       *void;
}

PxSceneReadLock :: struct {
    structgen_pad0: [8] u8;
}

PxSceneWriteLock :: struct {
    structgen_pad0: [8] u8;
}

PxContactPairExtraDataItem :: struct {
    type: u8;
}

PxContactPairVelocity :: struct {
    type:            u8;
    structgen_pad0:  [3] u8;
    linearVelocity:  [2] Vector3;
    angularVelocity: [2] Vector3;
}

PxContactPairPose :: struct {
    type:           u8;
    structgen_pad0: [3] u8;
    globalPose:     [2] PxTransform;
}

PxContactPairIndex :: struct {
    type:           u8;
    structgen_pad0: [1] u8;
    index:          u16;
}

PxContactPairExtraDataIterator :: struct {
    currPtr:            *u8;
    endPtr:             *u8;
    preSolverVelocity:  *PxContactPairVelocity;
    postSolverVelocity: *PxContactPairVelocity;
    eventPose:          *PxContactPairPose;
    contactPairIndex:   u32;
    structgen_pad0:     [4] u8;
}

PxContactPairHeader :: struct {
    actors:              [2] *PxActor;
    extraDataStream:     *u8;
    extraDataStreamSize: u16;
    flags:               u16;
    structgen_pad0:      [4] u8;
    pairs:               *PxContactPair;
    nbPairs:             u32;
    structgen_pad1:      [4] u8;
}

PxContactPairPoint :: struct {
    position:           Vector3;
    separation:         float;
    normal:             Vector3;
    internalFaceIndex0: u32;
    impulse:            Vector3;
    internalFaceIndex1: u32;
}

PxContactPair :: struct {
    shapes:             [2] *PxShape;
    contactPatches:     *u8;
    contactPoints:      *u8;
    contactImpulses:    *float;
    requiredBufferSize: u32;
    contactCount:       u8;
    patchCount:         u8;
    contactStreamSize:  u16;
    flags:              u16;
    events:             u16;
    internalData:       [2] u32;
    structgen_pad0:     [4] u8;
}

PxTriggerPair :: struct {
    triggerShape:   *PxShape;
    triggerActor:   *PxActor;
    otherShape:     *PxShape;
    otherActor:     *PxActor;
    status:         s32;
    flags:          u8;
    structgen_pad0: [3] u8;
}

PxConstraintInfo :: struct {
    constraint:        *PxConstraint;
    externalReference: *void;
    type:              u32;
    structgen_pad0:    [4] u8;
}

PxSimulationEventCallback :: struct {
    vtable_: *void;
}

PxFEMParameters :: struct {
    velocityDamping:              float;
    settlingThreshold:            float;
    sleepThreshold:               float;
    sleepDamping:                 float;
    selfCollisionFilterDistance:  float;
    selfCollisionStressTolerance: float;
}

PxPruningStructure :: struct {
    structgen_pad0: [16] u8;
}

PxExtendedVec3 :: struct {
    x: float64;
    y: float64;
    z: float64;
}

PxObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           PxExtendedVec3;
    mRot:           Quaternion;
}

PxBoxObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           PxExtendedVec3;
    mRot:           Quaternion;
    mHalfExtents:   Vector3;
    structgen_pad1: [4] u8;
}

PxCapsuleObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           PxExtendedVec3;
    mRot:           Quaternion;
    mHalfHeight:    float;
    mRadius:        float;
}

PxObstacleContext :: struct {
    vtable_: *void;
}

PxControllerState :: struct {
    deltaXP:               Vector3;
    structgen_pad0:        [4] u8;
    touchedShape:          *PxShape;
    touchedActor:          *PxRigidActor;
    touchedObstacleHandle: u32;
    collisionFlags:        u32;
    standOnAnotherCCT:     bool;
    standOnObstacle:       bool;
    isMovingUp:            bool;
    structgen_pad1:        [5] u8;
}

PxControllerStats :: struct {
    nbIterations:     u16;
    nbFullUpdates:    u16;
    nbPartialUpdates: u16;
    nbTessellation:   u16;
}

PxControllerHit :: struct {
    controller:     *PxController;
    worldPos:       PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
}

PxControllerShapeHit :: struct {
    controller:     *PxController;
    worldPos:       PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    shape:          *PxShape;
    actor:          *PxRigidActor;
    triangleIndex:  u32;
    structgen_pad1: [4] u8;
}

PxControllersHit :: struct {
    controller:     *PxController;
    worldPos:       PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    other:          *PxController;
}

PxControllerObstacleHit :: struct {
    controller:     *PxController;
    worldPos:       PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    userData:       *void;
}

PxUserControllerHitReport :: struct {
    vtable_: *void;
}

PxControllerFilterCallback :: struct {
    vtable_: *void;
}

PxControllerFilters :: struct {
    mFilterData:        *PxFilterData;
    mFilterCallback:    *PxQueryFilterCallback;
    mFilterFlags:       u16;
    structgen_pad0:     [6] u8;
    mCCTFilterCallback: *PxControllerFilterCallback;
}

PxControllerDesc :: struct {
    structgen_pad0:           [8] u8;
    position:                 PxExtendedVec3;
    upDirection:              Vector3;
    slopeLimit:               float;
    invisibleWallHeight:      float;
    maxJumpHeight:            float;
    contactOffset:            float;
    stepOffset:               float;
    density:                  float;
    scaleCoeff:               float;
    volumeGrowth:             float;
    structgen_pad1:           [4] u8;
    reportCallback:           *PxUserControllerHitReport;
    behaviorCallback:         *PxControllerBehaviorCallback;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *PxMaterial;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [8] u8;
}

PxController :: struct {
    vtable_: *void;
}

PxBoxControllerDesc :: struct {
    structgen_pad0:           [8] u8;
    position:                 PxExtendedVec3;
    upDirection:              Vector3;
    slopeLimit:               float;
    invisibleWallHeight:      float;
    maxJumpHeight:            float;
    contactOffset:            float;
    stepOffset:               float;
    density:                  float;
    scaleCoeff:               float;
    volumeGrowth:             float;
    structgen_pad1:           [4] u8;
    reportCallback:           *PxUserControllerHitReport;
    behaviorCallback:         *PxControllerBehaviorCallback;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *PxMaterial;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [4] u8;
    halfHeight:               float;
    halfSideExtent:           float;
    halfForwardExtent:        float;
#place structgen_pad0; #as controller_desc: PxControllerDesc = ---;
}

PxBoxController :: struct {
    vtable_: *void;
#place vtable_; #as controller: PxController = ---;
}

PxCapsuleControllerDesc :: struct {
    structgen_pad0:           [8] u8;
    position:                 PxExtendedVec3;
    upDirection:              Vector3;
    slopeLimit:               float;
    invisibleWallHeight:      float;
    maxJumpHeight:            float;
    contactOffset:            float;
    stepOffset:               float;
    density:                  float;
    scaleCoeff:               float;
    volumeGrowth:             float;
    structgen_pad1:           [4] u8;
    reportCallback:           *PxUserControllerHitReport;
    behaviorCallback:         *PxControllerBehaviorCallback;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *PxMaterial;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [4] u8;
    radius:                   float;
    height:                   float;
    climbingMode:             s32;
#place structgen_pad0; #as controller_desc: PxControllerDesc = ---;
}

PxCapsuleController :: struct {
    vtable_: *void;
#place vtable_; #as controller: PxController = ---;
}

PxControllerBehaviorCallback :: struct {
    vtable_: *void;
}

PxControllerManager :: struct {
    vtable_: *void;
}

PxDim3 :: struct {
    x: u32;
    y: u32;
    z: u32;
}

PxSDFDesc :: struct {
    sdf:                                            PxBoundedData;
    dims:                                           PxDim3;
    meshLower:                                      Vector3;
    spacing:                                        float;
    subgridSize:                                    u32;
    bitsPerSubgridPixel:                            s32;
    sdfSubgrids3DTexBlockDim:                       PxDim3;
    sdfSubgrids:                                    PxBoundedData;
    sdfStartSlots:                                  PxBoundedData;
    subgridsMinSdfValue:                            float;
    subgridsMaxSdfValue:                            float;
    sdfBounds:                                      PxBounds3;
    narrowBandThicknessRelativeToSdfBoundsDiagonal: float;
    numThreadsForSdfConstruction:                   u32;
}

PxConvexMeshDesc :: struct {
    points:         PxBoundedData;
    polygons:       PxBoundedData;
    indices:        PxBoundedData;
    flags:          u16;
    vertexLimit:    u16;
    polygonLimit:   u16;
    quantizedCount: u16;
    sdfDesc:        *PxSDFDesc;
}

PxTriangleMeshDesc :: struct {
    points:         PxBoundedData;
    triangles:      PxBoundedData;
    flags:          u16;
    structgen_pad0: [22] u8;
    sdfDesc:        *PxSDFDesc;
}

PxTetrahedronMeshDesc :: struct {
    structgen_pad0: [16] u8;
    points:         PxBoundedData;
    tetrahedrons:   PxBoundedData;
    flags:          u16;
    tetsPerElement: u16;
    structgen_pad1: [4] u8;
}

PxSoftBodySimulationDataDesc :: struct {
    vertexToTet: PxBoundedData;
}

PxBVH34MidphaseDesc :: struct {
    numPrimsPerLeaf: u32;
    buildStrategy:   s32;
    quantized:       bool;
    structgen_pad0:  [3] u8;
}

PxMidphaseDesc :: struct {
    structgen_pad0: [16] u8;
}

PxBVHDesc :: struct {
    bounds:          PxBoundedData;
    enlargement:     float;
    numPrimsPerLeaf: u32;
    buildStrategy:   s32;
    structgen_pad0:  [4] u8;
}

PxCookingParams :: struct {
    areaTestEpsilon:                float;
    planeTolerance:                 float;
    convexMeshCookingType:          s32;
    suppressTriangleMeshRemapTable: bool;
    buildTriangleAdjacencies:       bool;
    buildGPUData:                   bool;
    structgen_pad0:                 [1] u8;
    scale:                          PxTolerancesScale;
    meshPreprocessParams:           u32;
    meshWeldTolerance:              float;
    midphaseDesc:                   PxMidphaseDesc;
    gaussMapLimit:                  u32;
    maxWeightRatioInTet:            float;
}

PxDefaultMemoryOutputStream :: struct {
    structgen_pad0: [32] u8;
#place structgen_pad0; #as output_stream: PxOutputStream = ---;
}

PxDefaultMemoryInputData :: struct {
    structgen_pad0: [32] u8;
#place structgen_pad0; #as input_stream: PxInputStream = ---;
}

PxDefaultFileOutputStream :: struct {
    structgen_pad0: [16] u8;
}

PxDefaultFileInputData :: struct {
    structgen_pad0: [24] u8;
}

PxDefaultAllocator :: struct {
    vtable_: *void;
}

PxJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as base: PxBase = ---;
}

PxSpring :: struct {
    stiffness: float;
    damping:   float;
}

PxDistanceJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxJacobianRow :: struct {
    linear0:  Vector3;
    linear1:  Vector3;
    angular0: Vector3;
    angular1: Vector3;
}

PxContactJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxFixedJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxJointLimitParameters :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
}

PxJointLinearLimit :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    value:                      float;
}

PxJointLinearLimitPair :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

PxJointAngularLimitPair :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

PxJointLimitCone :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    yAngle:                     float;
    zAngle:                     float;
}

PxJointLimitPyramid :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    yAngleMin:                  float;
    yAngleMax:                  float;
    zAngleMin:                  float;
    zAngleMax:                  float;
}

PxPrismaticJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxRevoluteJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxSphericalJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxD6JointDrive :: struct {
    stiffness:  float;
    damping:    float;
    forceLimit: float;
    flags:      u32;
}

PxD6Joint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxGearJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
#place structgen_pad0; #as joint: PxJoint = ---;
}

PxRackAndPinionJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

PxGroupsMask :: struct {
    bits0: u16;
    bits1: u16;
    bits2: u16;
    bits3: u16;
}

PxDefaultErrorCallback :: struct {
    vtable_: *void;
}

PxRigidActorExt :: struct {
    structgen_pad0: [1] u8;
}

PxMassProperties :: struct {
    inertiaTensor: PxMat33;
    centerOfMass:  Vector3;
    mass:          float;
}

PxRigidBodyExt :: struct {
    structgen_pad0: [1] u8;
}

PxShapeExt :: struct {
    structgen_pad0: [1] u8;
}

PxMeshOverlapUtil :: struct {
    structgen_pad0: [1040] u8;
}

PxBinaryConverter :: struct {}
PxXmlMiscParameter :: struct {
    upVector: Vector3;
    scale:    PxTolerancesScale;
}

PxSerialization :: struct {
    structgen_pad0: [1] u8;
}

PxDefaultCpuDispatcher :: struct {
    vtable_: *void;
}

PxStringTableExt :: struct {
    structgen_pad0: [1] u8;
}

PxBroadPhaseExt :: struct {
    structgen_pad0: [1] u8;
}

PxSceneQueryExt :: struct {
    structgen_pad0: [1] u8;
}

PxBatchQueryExt :: struct {
    vtable_: *void;
}

PxCustomSceneQuerySystem :: struct {
    vtable_: *void;
}

PxCustomSceneQuerySystemAdapter :: struct {
    vtable_: *void;
}

PxSamplingExt :: struct {
    structgen_pad0: [1] u8;
}

PxPoissonSampler :: struct {
    structgen_pad0: [8] u8;
}

PxTriangleMeshPoissonSampler :: struct {
    structgen_pad0: [8] u8;
}

PxTetrahedronMeshExt :: struct {
    structgen_pad0: [1] u8;
}

PxRepXObject :: struct {
    typeName:     *u8;
    serializable: *void;
    id:           u64;
}

PxCooking :: struct {}
PxRepXInstantiationArgs :: struct {
    structgen_pad0: [8] u8;
    cooker:         *PxCooking;
    stringTable:    *PxStringTable;
}

XmlMemoryAllocator :: struct {}
XmlWriter :: struct {}
XmlReader :: struct {}
MemoryBuffer :: struct {}
PxRepXSerializer :: struct {
    vtable_: *void;
}

PxVehicleWheels4SimData :: struct {}
PxVehicleWheels4DynData :: struct {}
PxVehicleTireForceCalculator :: struct {}
PxVehicleDrivableSurfaceToTireFrictionPairs :: struct {}
PxVehicleTelemetryData :: struct {}

PxPvd :: struct {
    vtable_: *void;
}

PxPvdTransport :: struct {
    vtable_: *void;
}

PxAllocatorCallback_delete :: (self: *PxAllocatorCallback) -> void #foreign libphysx;

PxAllocatorCallback_allocate :: (self: *PxAllocatorCallback, size: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign libphysx "PxAllocatorCallback_allocate_mut";

PxAllocatorCallback_deallocate :: (self: *PxAllocatorCallback, ptr: *void) -> void #foreign libphysx "PxAllocatorCallback_deallocate_mut";

PxAssertHandler_delete :: (self: *PxAssertHandler) -> void #foreign libphysx;

PxGetAssertHandler :: () -> *PxAssertHandler #foreign libphysx "phys_PxGetAssertHandler";

PxSetAssertHandler :: (handler: *PxAssertHandler) -> void #foreign libphysx "phys_PxSetAssertHandler";

PxFoundation_release :: (self: *PxFoundation) -> void #foreign libphysx "PxFoundation_release_mut";

PxFoundation_getErrorCallback :: (self: *PxFoundation) -> *PxErrorCallback #foreign libphysx "PxFoundation_getErrorCallback_mut";

PxFoundation_setErrorLevel :: (self: *PxFoundation, mask: u32) -> void #foreign libphysx "PxFoundation_setErrorLevel_mut";

PxFoundation_getErrorLevel :: (self: *PxFoundation) -> u32 #foreign libphysx;

PxFoundation_getAllocatorCallback :: (self: *PxFoundation) -> *PxAllocatorCallback #foreign libphysx "PxFoundation_getAllocatorCallback_mut";

PxFoundation_getReportAllocationNames :: (self: *PxFoundation) -> bool #foreign libphysx;

PxFoundation_setReportAllocationNames :: (self: *PxFoundation, value: bool) -> void #foreign libphysx "PxFoundation_setReportAllocationNames_mut";

PxFoundation_registerAllocationListener :: (self: *PxFoundation, listener: *PxAllocationListener) -> void #foreign libphysx "PxFoundation_registerAllocationListener_mut";

PxFoundation_deregisterAllocationListener :: (self: *PxFoundation, listener: *PxAllocationListener) -> void #foreign libphysx "PxFoundation_deregisterAllocationListener_mut";

PxFoundation_registerErrorCallback :: (self: *PxFoundation, callback: *PxErrorCallback) -> void #foreign libphysx "PxFoundation_registerErrorCallback_mut";

PxFoundation_deregisterErrorCallback :: (self: *PxFoundation, callback: *PxErrorCallback) -> void #foreign libphysx "PxFoundation_deregisterErrorCallback_mut";

PxCreateFoundation :: (version: u32, allocator: *PxAllocatorCallback, errorCallback: *PxErrorCallback) -> *PxFoundation #foreign libphysx "phys_PxCreateFoundation";

PxSetFoundationInstance :: (foundation: *PxFoundation) -> void #foreign libphysx "phys_PxSetFoundationInstance";

PxGetFoundation :: () -> *PxFoundation #foreign libphysx "phys_PxGetFoundation";

PxGetProfilerCallback :: () -> *PxProfilerCallback #foreign libphysx "phys_PxGetProfilerCallback";

PxSetProfilerCallback :: (profiler: *PxProfilerCallback) -> void #foreign libphysx "phys_PxSetProfilerCallback";

PxGetAllocatorCallback :: () -> *PxAllocatorCallback #foreign libphysx "phys_PxGetAllocatorCallback";

PxGetBroadcastAllocator :: () -> *PxAllocatorCallback #foreign libphysx "phys_PxGetBroadcastAllocator";

PxGetErrorCallback :: () -> *PxErrorCallback #foreign libphysx "phys_PxGetErrorCallback";

PxGetBroadcastError :: () -> *PxErrorCallback #foreign libphysx "phys_PxGetBroadcastError";

PxGetWarnOnceTimeStamp :: () -> u32 #foreign libphysx "phys_PxGetWarnOnceTimeStamp";

PxDecFoundationRefCount :: () -> void #foreign libphysx "phys_PxDecFoundationRefCount";

PxIncFoundationRefCount :: () -> void #foreign libphysx "phys_PxIncFoundationRefCount";

PxAllocator_new :: (anon_param0: *u8) -> PxAllocator #foreign libphysx;

PxAllocator_allocate :: (self: *PxAllocator, size: u64, file: *u8, line: s32) -> *void #foreign libphysx "PxAllocator_allocate_mut";

PxAllocator_deallocate :: (self: *PxAllocator, ptr: *void) -> void #foreign libphysx "PxAllocator_deallocate_mut";

PxRawAllocator_new :: (anon_param0: *u8) -> PxRawAllocator #foreign libphysx;

PxRawAllocator_allocate :: (self: *PxRawAllocator, size: u64, anon_param1: *u8, anon_param2: s32) -> *void #foreign libphysx "PxRawAllocator_allocate_mut";

PxRawAllocator_deallocate :: (self: *PxRawAllocator, ptr: *void) -> void #foreign libphysx "PxRawAllocator_deallocate_mut";

PxVirtualAllocatorCallback_delete :: (self: *PxVirtualAllocatorCallback) -> void #foreign libphysx;

PxVirtualAllocatorCallback_allocate :: (self: *PxVirtualAllocatorCallback, size: u64, group: s32, file: *u8, line: s32) -> *void #foreign libphysx "PxVirtualAllocatorCallback_allocate_mut";

PxVirtualAllocatorCallback_deallocate :: (self: *PxVirtualAllocatorCallback, ptr: *void) -> void #foreign libphysx "PxVirtualAllocatorCallback_deallocate_mut";

PxVirtualAllocator_new :: (callback: *PxVirtualAllocatorCallback, group: s32) -> PxVirtualAllocator #foreign libphysx;

PxVirtualAllocator_allocate :: (self: *PxVirtualAllocator, size: u64, file: *u8, line: s32) -> *void #foreign libphysx "PxVirtualAllocator_allocate_mut";

PxVirtualAllocator_deallocate :: (self: *PxVirtualAllocator, ptr: *void) -> void #foreign libphysx "PxVirtualAllocator_deallocate_mut";

PxTempAllocatorChunk_new :: () -> PxTempAllocatorChunk #foreign libphysx;

PxTempAllocator_new :: (anon_param0: *u8) -> PxTempAllocator #foreign libphysx;

PxTempAllocator_allocate :: (self: *PxTempAllocator, size: u64, file: *u8, line: s32) -> *void #foreign libphysx "PxTempAllocator_allocate_mut";

PxTempAllocator_deallocate :: (self: *PxTempAllocator, ptr: *void) -> void #foreign libphysx "PxTempAllocator_deallocate_mut";

PxMemZero :: (dest: *void, count: u32) -> *void #foreign libphysx "phys_PxMemZero";

PxMemSet :: (dest: *void, c: s32, count: u32) -> *void #foreign libphysx "phys_PxMemSet";

PxMemCopy :: (dest: *void, src: *void, count: u32) -> *void #foreign libphysx "phys_PxMemCopy";

PxMemMove :: (dest: *void, src: *void, count: u32) -> *void #foreign libphysx "phys_PxMemMove";

PxMarkSerializedMemory :: (ptr: *void, byteSize: u32) -> void #foreign libphysx "phys_PxMarkSerializedMemory";

PxMemoryBarrier :: () -> void #foreign libphysx "phys_PxMemoryBarrier";

PxHighestSetBitUnsafe :: (v: u32) -> u32 #foreign libphysx "phys_PxHighestSetBitUnsafe";

PxLowestSetBitUnsafe :: (v: u32) -> u32 #foreign libphysx "phys_PxLowestSetBitUnsafe";

PxCountLeadingZeros :: (v: u32) -> u32 #foreign libphysx "phys_PxCountLeadingZeros";

PxPrefetchLine :: (ptr: *void, offset: u32) -> void #foreign libphysx "phys_PxPrefetchLine";

PxPrefetch :: (ptr: *void, count: u32) -> void #foreign libphysx "phys_PxPrefetch";

PxBitCount :: (v: u32) -> u32 #foreign libphysx "phys_PxBitCount";

PxIsPowerOfTwo :: (x: u32) -> bool #foreign libphysx "phys_PxIsPowerOfTwo";

PxNextPowerOfTwo :: (x: u32) -> u32 #foreign libphysx "phys_PxNextPowerOfTwo";

PxLowestSetBit :: (x: u32) -> u32 #foreign libphysx "phys_PxLowestSetBit";

PxHighestSetBit :: (x: u32) -> u32 #foreign libphysx "phys_PxHighestSetBit";

PxILog2 :: (num: u32) -> u32 #foreign libphysx "phys_PxILog2";

PxVec3_new :: () -> Vector3 #foreign libphysx;

PxVec3_new :: (anon_param0: s32) -> Vector3 #foreign libphysx "PxVec3_new_1";

PxVec3_new :: (a: float) -> Vector3 #foreign libphysx "PxVec3_new_2";

PxVec3_new :: (nx: float, ny: float, nz: float) -> Vector3 #foreign libphysx "PxVec3_new_3";

PxVec3_isZero :: (self: *Vector3) -> bool #foreign libphysx;

PxVec3_isFinite :: (self: *Vector3) -> bool #foreign libphysx;

PxVec3_isNormalized :: (self: *Vector3) -> bool #foreign libphysx;

PxVec3_magnitudeSquared :: (self: *Vector3) -> float #foreign libphysx;

PxVec3_magnitude :: (self: *Vector3) -> float #foreign libphysx;

PxVec3_dot :: (self: *Vector3, v: *Vector3) -> float #foreign libphysx;

PxVec3_cross :: (self: *Vector3, v: *Vector3) -> Vector3 #foreign libphysx;

PxVec3_getNormalized :: (self: *Vector3) -> Vector3 #foreign libphysx;

PxVec3_normalize :: (self: *Vector3) -> float #foreign libphysx "PxVec3_normalize_mut";

PxVec3_normalizeSafe :: (self: *Vector3) -> float #foreign libphysx "PxVec3_normalizeSafe_mut";

PxVec3_normalizeFast :: (self: *Vector3) -> float #foreign libphysx "PxVec3_normalizeFast_mut";

PxVec3_multiply :: (self: *Vector3, a: *Vector3) -> Vector3 #foreign libphysx;

PxVec3_minimum :: (self: *Vector3, v: *Vector3) -> Vector3 #foreign libphysx;

PxVec3_minElement :: (self: *Vector3) -> float #foreign libphysx;

PxVec3_maximum :: (self: *Vector3, v: *Vector3) -> Vector3 #foreign libphysx;

PxVec3_maxElement :: (self: *Vector3) -> float #foreign libphysx;

PxVec3_abs :: (self: *Vector3) -> Vector3 #foreign libphysx;

PxVec3Padded_new_alloc :: () -> *PxVec3Padded #foreign libphysx;

PxVec3Padded_delete :: (self: *PxVec3Padded) -> void #foreign libphysx;

PxVec3Padded_new_alloc :: (p: *Vector3) -> *PxVec3Padded #foreign libphysx "PxVec3Padded_new_alloc_1";

PxVec3Padded_new_alloc :: (f: float) -> *PxVec3Padded #foreign libphysx "PxVec3Padded_new_alloc_2";

PxQuat_new :: (anon_param0: s32) -> Quaternion #foreign libphysx "PxQuat_new_1";

PxQuat_new :: (r: float) -> Quaternion #foreign libphysx "PxQuat_new_2";

PxQuat_new :: (nx: float, ny: float, nz: float, nw: float) -> Quaternion #foreign libphysx "PxQuat_new_3";

PxQuat_new :: (angleRadians: float, unitAxis: *Vector3) -> Quaternion #foreign libphysx "PxQuat_new_4";

PxQuat_new :: (m: *PxMat33) -> Quaternion #foreign libphysx "PxQuat_new_5";

PxQuat_isIdentity :: (self: *Quaternion) -> bool #foreign libphysx;

PxQuat_isFinite :: (self: *Quaternion) -> bool #foreign libphysx;

PxQuat_isUnit :: (self: *Quaternion) -> bool #foreign libphysx;

PxQuat_isSane :: (self: *Quaternion) -> bool #foreign libphysx;

PxQuat_toRadiansAndUnitAxis :: (self: *Quaternion, angle: *float, axis: *Vector3) -> void #foreign libphysx;

PxQuat_getAngle :: (self: *Quaternion) -> float #foreign libphysx;

PxQuat_getAngle :: (self: *Quaternion, q: *Quaternion) -> float #foreign libphysx "PxQuat_getAngle_1";

PxQuat_magnitudeSquared :: (self: *Quaternion) -> float #foreign libphysx;

PxQuat_dot :: (self: *Quaternion, v: *Quaternion) -> float #foreign libphysx;

PxQuat_getNormalized :: (self: *Quaternion) -> Quaternion #foreign libphysx;

PxQuat_magnitude :: (self: *Quaternion) -> float #foreign libphysx;

PxQuat_normalize :: (self: *Quaternion) -> float #foreign libphysx "PxQuat_normalize_mut";

PxQuat_getConjugate :: (self: *Quaternion) -> Quaternion #foreign libphysx;

PxQuat_getImaginaryPart :: (self: *Quaternion) -> Vector3 #foreign libphysx;

PxQuat_getBasisVector0 :: (self: *Quaternion) -> Vector3 #foreign libphysx;

PxQuat_getBasisVector1 :: (self: *Quaternion) -> Vector3 #foreign libphysx;

PxQuat_getBasisVector2 :: (self: *Quaternion) -> Vector3 #foreign libphysx;

PxQuat_rotate :: (self: *Quaternion, v: *Vector3) -> Vector3 #foreign libphysx;

PxQuat_rotateInv :: (self: *Quaternion, v: *Vector3) -> Vector3 #foreign libphysx;

PxTransform_new :: (position: *Vector3) -> PxTransform #foreign libphysx "PxTransform_new_1";

PxTransform_new :: (anon_param0: s32) -> PxTransform #foreign libphysx "PxTransform_new_2";

PxTransform_new :: (orientation: *Quaternion) -> PxTransform #foreign libphysx "PxTransform_new_3";

PxTransform_new :: (x: float, y: float, z: float, aQ: Quaternion) -> PxTransform #foreign libphysx "PxTransform_new_4";

PxTransform_new :: (p0: *Vector3, q0: *Quaternion) -> PxTransform #foreign libphysx "PxTransform_new_5";

PxTransform_new :: (m: *PxMat44) -> PxTransform #foreign libphysx "PxTransform_new_6";

PxTransform_getInverse :: (self: *PxTransform) -> PxTransform #foreign libphysx;

PxTransform_transform :: (self: *PxTransform, input: *Vector3) -> Vector3 #foreign libphysx;

PxTransform_transformInv :: (self: *PxTransform, input: *Vector3) -> Vector3 #foreign libphysx;

PxTransform_rotate :: (self: *PxTransform, input: *Vector3) -> Vector3 #foreign libphysx;

PxTransform_rotateInv :: (self: *PxTransform, input: *Vector3) -> Vector3 #foreign libphysx;

PxTransform_transform :: (self: *PxTransform, src: *PxTransform) -> PxTransform #foreign libphysx "PxTransform_transform_1";

PxTransform_isValid :: (self: *PxTransform) -> bool #foreign libphysx;

PxTransform_isSane :: (self: *PxTransform) -> bool #foreign libphysx;

PxTransform_isFinite :: (self: *PxTransform) -> bool #foreign libphysx;

PxTransform_transformInv :: (self: *PxTransform, src: *PxTransform) -> PxTransform #foreign libphysx "PxTransform_transformInv_1";

PxTransform_getNormalized :: (self: *PxTransform) -> PxTransform #foreign libphysx;

PxMat33_new :: () -> PxMat33 #foreign libphysx;

PxMat33_new :: (col0: *Vector3, col1: *Vector3, col2: *Vector3) -> PxMat33 #foreign libphysx "PxMat33_new_3";

PxMat33_new :: (r: float) -> PxMat33 #foreign libphysx "PxMat33_new_4";

PxMat33_new :: (values: *float) -> PxMat33 #foreign libphysx "PxMat33_new_5";

PxMat33_new :: (q: *Quaternion) -> PxMat33 #foreign libphysx "PxMat33_new_6";

PxMat33_createDiagonal :: (d: *Vector3) -> PxMat33 #foreign libphysx;

PxMat33_outer :: (a: *Vector3, b: *Vector3) -> PxMat33 #foreign libphysx;

PxMat33_getTranspose :: (self: *PxMat33) -> PxMat33 #foreign libphysx;

PxMat33_getInverse :: (self: *PxMat33) -> PxMat33 #foreign libphysx;

PxMat33_getDeterminant :: (self: *PxMat33) -> float #foreign libphysx;

PxMat33_transform :: (self: *PxMat33, other: *Vector3) -> Vector3 #foreign libphysx;

PxMat33_transformTranspose :: (self: *PxMat33, other: *Vector3) -> Vector3 #foreign libphysx;

PxMat33_front :: (self: *PxMat33) -> *float #foreign libphysx;

PxBounds3_new :: () -> PxBounds3 #foreign libphysx;

PxBounds3_new :: (minimum: *Vector3, maximum: *Vector3) -> PxBounds3 #foreign libphysx "PxBounds3_new_1";

PxBounds3_empty :: () -> PxBounds3 #foreign libphysx;

PxBounds3_boundsOfPoints :: (v0: *Vector3, v1: *Vector3) -> PxBounds3 #foreign libphysx;

PxBounds3_centerExtents :: (center: *Vector3, extent: *Vector3) -> PxBounds3 #foreign libphysx;

PxBounds3_basisExtent :: (center: *Vector3, basis: *PxMat33, extent: *Vector3) -> PxBounds3 #foreign libphysx;

PxBounds3_poseExtent :: (pose: *PxTransform, extent: *Vector3) -> PxBounds3 #foreign libphysx;

PxBounds3_transformSafe :: (matrix: *PxMat33, bounds: *PxBounds3) -> PxBounds3 #foreign libphysx;

PxBounds3_transformFast :: (matrix: *PxMat33, bounds: *PxBounds3) -> PxBounds3 #foreign libphysx;

PxBounds3_transformSafe :: (transform: *PxTransform, bounds: *PxBounds3) -> PxBounds3 #foreign libphysx "PxBounds3_transformSafe_1";

PxBounds3_transformFast :: (transform: *PxTransform, bounds: *PxBounds3) -> PxBounds3 #foreign libphysx "PxBounds3_transformFast_1";

PxBounds3_setEmpty :: (self: *PxBounds3) -> void #foreign libphysx "PxBounds3_setEmpty_mut";

PxBounds3_setMaximal :: (self: *PxBounds3) -> void #foreign libphysx "PxBounds3_setMaximal_mut";

PxBounds3_include :: (self: *PxBounds3, v: *Vector3) -> void #foreign libphysx "PxBounds3_include_mut";

PxBounds3_include_mut :: (self: *PxBounds3, b: *PxBounds3) -> void #foreign libphysx "PxBounds3_include_mut_1";

PxBounds3_isEmpty :: (self: *PxBounds3) -> bool #foreign libphysx;

PxBounds3_intersects :: (self: *PxBounds3, b: *PxBounds3) -> bool #foreign libphysx;

PxBounds3_intersects1D :: (self: *PxBounds3, a: *PxBounds3, axis: u32) -> bool #foreign libphysx;

PxBounds3_contains :: (self: *PxBounds3, v: *Vector3) -> bool #foreign libphysx;

PxBounds3_isInside :: (self: *PxBounds3, box: *PxBounds3) -> bool #foreign libphysx;

PxBounds3_getCenter :: (self: *PxBounds3) -> Vector3 #foreign libphysx;

PxBounds3_getCenter :: (self: *PxBounds3, axis: u32) -> float #foreign libphysx "PxBounds3_getCenter_1";

PxBounds3_getExtents :: (self: *PxBounds3, axis: u32) -> float #foreign libphysx;

PxBounds3_getDimensions :: (self: *PxBounds3) -> Vector3 #foreign libphysx;

PxBounds3_getExtents :: (self: *PxBounds3) -> Vector3 #foreign libphysx "PxBounds3_getExtents_1";

PxBounds3_scaleSafe :: (self: *PxBounds3, scale: float) -> void #foreign libphysx "PxBounds3_scaleSafe_mut";

PxBounds3_scaleFast :: (self: *PxBounds3, scale: float) -> void #foreign libphysx "PxBounds3_scaleFast_mut";

PxBounds3_fattenSafe :: (self: *PxBounds3, distance: float) -> void #foreign libphysx "PxBounds3_fattenSafe_mut";

PxBounds3_fattenFast :: (self: *PxBounds3, distance: float) -> void #foreign libphysx "PxBounds3_fattenFast_mut";

PxBounds3_isFinite :: (self: *PxBounds3) -> bool #foreign libphysx;

PxBounds3_isValid :: (self: *PxBounds3) -> bool #foreign libphysx;

PxBounds3_closestPoint :: (self: *PxBounds3, p: *Vector3) -> Vector3 #foreign libphysx;

PxErrorCallback_delete :: (self: *PxErrorCallback) -> void #foreign libphysx;

PxErrorCallback_reportError :: (self: *PxErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign libphysx "PxErrorCallback_reportError_mut";

PxAllocationListener_onAllocation :: (self: *PxAllocationListener, size: u64, typeName: *u8, filename: *u8, line: s32, allocatedMemory: *void) -> void #foreign libphysx "PxAllocationListener_onAllocation_mut";

PxAllocationListener_onDeallocation :: (self: *PxAllocationListener, allocatedMemory: *void) -> void #foreign libphysx "PxAllocationListener_onDeallocation_mut";

PxBroadcastingAllocator_new_alloc :: (allocator: *PxAllocatorCallback, error: *PxErrorCallback) -> *PxBroadcastingAllocator #foreign libphysx;

PxBroadcastingAllocator_delete :: (self: *PxBroadcastingAllocator) -> void #foreign libphysx;

PxBroadcastingAllocator_allocate :: (self: *PxBroadcastingAllocator, size: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign libphysx "PxBroadcastingAllocator_allocate_mut";

PxBroadcastingAllocator_deallocate :: (self: *PxBroadcastingAllocator, ptr: *void) -> void #foreign libphysx "PxBroadcastingAllocator_deallocate_mut";

PxBroadcastingErrorCallback_new_alloc :: (errorCallback: *PxErrorCallback) -> *PxBroadcastingErrorCallback #foreign libphysx;

PxBroadcastingErrorCallback_delete :: (self: *PxBroadcastingErrorCallback) -> void #foreign libphysx;

PxBroadcastingErrorCallback_reportError :: (self: *PxBroadcastingErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign libphysx "PxBroadcastingErrorCallback_reportError_mut";

PxEnableFPExceptions :: () -> void #foreign libphysx "phys_PxEnableFPExceptions";

PxDisableFPExceptions :: () -> void #foreign libphysx "phys_PxDisableFPExceptions";

PxInputStream_read :: (self: *PxInputStream, dest: *void, count: u32) -> u32 #foreign libphysx "PxInputStream_read_mut";

PxInputStream_delete :: (self: *PxInputStream) -> void #foreign libphysx;

PxInputData_getLength :: (self: *PxInputData) -> u32 #foreign libphysx;

PxInputData_seek :: (self: *PxInputData, offset: u32) -> void #foreign libphysx "PxInputData_seek_mut";

PxInputData_tell :: (self: *PxInputData) -> u32 #foreign libphysx;

PxInputData_delete :: (self: *PxInputData) -> void #foreign libphysx;

PxOutputStream_write :: (self: *PxOutputStream, src: *void, count: u32) -> u32 #foreign libphysx "PxOutputStream_write_mut";

PxOutputStream_delete :: (self: *PxOutputStream) -> void #foreign libphysx;

PxVec4_new :: () -> Vector4 #foreign libphysx;

PxVec4_new :: (anon_param0: s32) -> Vector4 #foreign libphysx "PxVec4_new_1";

PxVec4_new :: (a: float) -> Vector4 #foreign libphysx "PxVec4_new_2";

PxVec4_new :: (nx: float, ny: float, nz: float, nw: float) -> Vector4 #foreign libphysx "PxVec4_new_3";

PxVec4_new :: (v: *Vector3, nw: float) -> Vector4 #foreign libphysx "PxVec4_new_4";

PxVec4_new :: (v: *float) -> Vector4 #foreign libphysx "PxVec4_new_5";

PxVec4_isZero :: (self: *Vector4) -> bool #foreign libphysx;

PxVec4_isFinite :: (self: *Vector4) -> bool #foreign libphysx;

PxVec4_isNormalized :: (self: *Vector4) -> bool #foreign libphysx;

PxVec4_magnitudeSquared :: (self: *Vector4) -> float #foreign libphysx;

PxVec4_magnitude :: (self: *Vector4) -> float #foreign libphysx;

PxVec4_dot :: (self: *Vector4, v: *Vector4) -> float #foreign libphysx;

PxVec4_getNormalized :: (self: *Vector4) -> Vector4 #foreign libphysx;

PxVec4_normalize :: (self: *Vector4) -> float #foreign libphysx "PxVec4_normalize_mut";

PxVec4_multiply :: (self: *Vector4, a: *Vector4) -> Vector4 #foreign libphysx;

PxVec4_minimum :: (self: *Vector4, v: *Vector4) -> Vector4 #foreign libphysx;

PxVec4_maximum :: (self: *Vector4, v: *Vector4) -> Vector4 #foreign libphysx;

PxVec4_getXYZ :: (self: *Vector4) -> Vector3 #foreign libphysx;

PxMat44_new :: () -> PxMat44 #foreign libphysx;

PxMat44_new :: (col0: *Vector4, col1: *Vector4, col2: *Vector4, col3: *Vector4) -> PxMat44 #foreign libphysx "PxMat44_new_3";

PxMat44_new :: (r: float) -> PxMat44 #foreign libphysx "PxMat44_new_4";

PxMat44_new :: (col0: *Vector3, col1: *Vector3, col2: *Vector3, col3: *Vector3) -> PxMat44 #foreign libphysx "PxMat44_new_5";

PxMat44_new :: (values: *float) -> PxMat44 #foreign libphysx "PxMat44_new_6";

PxMat44_new :: (q: *Quaternion) -> PxMat44 #foreign libphysx "PxMat44_new_7";

PxMat44_new :: (diagonal: *Vector4) -> PxMat44 #foreign libphysx "PxMat44_new_8";

PxMat44_new :: (axes: *PxMat33, position: *Vector3) -> PxMat44 #foreign libphysx "PxMat44_new_9";

PxMat44_new :: (t: *PxTransform) -> PxMat44 #foreign libphysx "PxMat44_new_10";

PxMat44_getTranspose :: (self: *PxMat44) -> PxMat44 #foreign libphysx;

PxMat44_transform :: (self: *PxMat44, other: *Vector4) -> Vector4 #foreign libphysx;

PxMat44_transform :: (self: *PxMat44, other: *Vector3) -> Vector3 #foreign libphysx "PxMat44_transform_1";

PxMat44_rotate :: (self: *PxMat44, other: *Vector4) -> Vector4 #foreign libphysx;

PxMat44_rotate :: (self: *PxMat44, other: *Vector3) -> Vector3 #foreign libphysx "PxMat44_rotate_1";

PxMat44_getBasis :: (self: *PxMat44, num: u32) -> Vector3 #foreign libphysx;

PxMat44_getPosition :: (self: *PxMat44) -> Vector3 #foreign libphysx;

PxMat44_setPosition :: (self: *PxMat44, position: *Vector3) -> void #foreign libphysx "PxMat44_setPosition_mut";

PxMat44_front :: (self: *PxMat44) -> *float #foreign libphysx;

PxMat44_scale :: (self: *PxMat44, p: *Vector4) -> void #foreign libphysx "PxMat44_scale_mut";

PxMat44_inverseRT :: (self: *PxMat44) -> PxMat44 #foreign libphysx;

PxMat44_isFinite :: (self: *PxMat44) -> bool #foreign libphysx;

PxPlane_new :: () -> Plane3 #foreign libphysx;

PxPlane_new :: (nx: float, ny: float, nz: float, distance: float) -> Plane3 #foreign libphysx "PxPlane_new_1";

PxPlane_new :: (normal: *Vector3, distance: float) -> Plane3 #foreign libphysx "PxPlane_new_2";

PxPlane_new :: (point: *Vector3, normal: *Vector3) -> Plane3 #foreign libphysx "PxPlane_new_3";

PxPlane_new :: (p0: *Vector3, p1: *Vector3, p2: *Vector3) -> Plane3 #foreign libphysx "PxPlane_new_4";

PxPlane_distance :: (self: *Plane3, p: *Vector3) -> float #foreign libphysx;

PxPlane_contains :: (self: *Plane3, p: *Vector3) -> bool #foreign libphysx;

PxPlane_project :: (self: *Plane3, p: *Vector3) -> Vector3 #foreign libphysx;

PxPlane_pointInPlane :: (self: *Plane3) -> Vector3 #foreign libphysx;

PxPlane_normalize :: (self: *Plane3) -> void #foreign libphysx "PxPlane_normalize_mut";

PxPlane_transform :: (self: *Plane3, pose: *PxTransform) -> Plane3 #foreign libphysx;

PxPlane_inverseTransform :: (self: *Plane3, pose: *PxTransform) -> Plane3 #foreign libphysx;

PxShortestRotation :: (from: *Vector3, target: *Vector3) -> Quaternion #foreign libphysx "phys_PxShortestRotation";

PxDiagonalize :: (m: *PxMat33, axes: *Quaternion) -> Vector3 #foreign libphysx "phys_PxDiagonalize";

PxTransformFromSegment :: (p0: *Vector3, p1: *Vector3, halfHeight: *float) -> PxTransform #foreign libphysx "phys_PxTransformFromSegment";

PxTransformFromPlaneEquation :: (plane: *Plane3) -> PxTransform #foreign libphysx "phys_PxTransformFromPlaneEquation";

PxPlaneEquationFromTransform :: (pose: *PxTransform) -> Plane3 #foreign libphysx "phys_PxPlaneEquationFromTransform";

PxSlerp :: (t: float, left: *Quaternion, right: *Quaternion) -> Quaternion #foreign libphysx "phys_PxSlerp";

PxIntegrateTransform :: (curTrans: *PxTransform, linvel: *Vector3, angvel: *Vector3, timeStep: float, result: *PxTransform) -> void #foreign libphysx "phys_PxIntegrateTransform";

PxExp :: (v: *Vector3) -> Quaternion #foreign libphysx "phys_PxExp";

PxOptimizeBoundingBox :: (basis: *PxMat33) -> Vector3 #foreign libphysx "phys_PxOptimizeBoundingBox";

PxLog :: (q: *Quaternion) -> Vector3 #foreign libphysx "phys_PxLog";

PxLargestAxis :: (v: *Vector3) -> u32 #foreign libphysx "phys_PxLargestAxis";

PxTanHalf :: (sin: float, cos: float) -> float #foreign libphysx "phys_PxTanHalf";

PxEllipseClamp :: (point: *Vector3, radii: *Vector3) -> Vector3 #foreign libphysx "phys_PxEllipseClamp";

PxSeparateSwingTwist :: (q: *Quaternion, swing: *Quaternion, twist: *Quaternion) -> void #foreign libphysx "phys_PxSeparateSwingTwist";

PxComputeAngle :: (v0: *Vector3, v1: *Vector3) -> float #foreign libphysx "phys_PxComputeAngle";

PxComputeBasisVectors :: (dir: *Vector3, right: *Vector3, up: *Vector3) -> void #foreign libphysx "phys_PxComputeBasisVectors";

PxComputeBasisVectors_1 :: (p0: *Vector3, p1: *Vector3, dir: *Vector3, right: *Vector3, up: *Vector3) -> void #foreign libphysx "phys_PxComputeBasisVectors_1";

PxGetNextIndex3 :: (i: u32) -> u32 #foreign libphysx "phys_PxGetNextIndex3";

computeBarycentric :: (a: *Vector3, b: *Vector3, c: *Vector3, d: *Vector3, p: *Vector3, bary: *Vector4) -> void #foreign libphysx "phys_computeBarycentric";

computeBarycentric_1 :: (a: *Vector3, b: *Vector3, c: *Vector3, p: *Vector3, bary: *Vector4) -> void #foreign libphysx "phys_computeBarycentric_1";

Interpolation_PxLerp :: (a: float, b: float, t: float) -> float #foreign libphysx;

Interpolation_PxBiLerp :: (f00: float, f10: float, f01: float, f11: float, tx: float, ty: float) -> float #foreign libphysx;

Interpolation_PxTriLerp :: (f000: float, f100: float, f010: float, f110: float, f001: float, f101: float, f011: float, f111: float, tx: float, ty: float, tz: float) -> float #foreign libphysx;

Interpolation_PxSDFIdx :: (i: u32, j: u32, k: u32, nbX: u32, nbY: u32) -> u32 #foreign libphysx;

Interpolation_PxSDFSampleImpl :: (sdf: *float, localPos: *Vector3, sdfBoxLower: *Vector3, sdfBoxHigher: *Vector3, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, tolerance: float) -> float #foreign libphysx;

PxSdfSample :: (sdf: *float, localPos: *Vector3, sdfBoxLower: *Vector3, sdfBoxHigher: *Vector3, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, gradient: *Vector3, tolerance: float) -> float #foreign libphysx "phys_PxSdfSample";

PxMutexImpl_new_alloc :: () -> *PxMutexImpl #foreign libphysx;

PxMutexImpl_delete :: (self: *PxMutexImpl) -> void #foreign libphysx;

PxMutexImpl_lock :: (self: *PxMutexImpl) -> void #foreign libphysx "PxMutexImpl_lock_mut";

PxMutexImpl_trylock :: (self: *PxMutexImpl) -> bool #foreign libphysx "PxMutexImpl_trylock_mut";

PxMutexImpl_unlock :: (self: *PxMutexImpl) -> void #foreign libphysx "PxMutexImpl_unlock_mut";

PxMutexImpl_getSize :: () -> u32 #foreign libphysx;

PxReadWriteLock_new_alloc :: () -> *PxReadWriteLock #foreign libphysx;

PxReadWriteLock_delete :: (self: *PxReadWriteLock) -> void #foreign libphysx;

PxReadWriteLock_lockReader :: (self: *PxReadWriteLock, takeLock: bool) -> void #foreign libphysx "PxReadWriteLock_lockReader_mut";

PxReadWriteLock_lockWriter :: (self: *PxReadWriteLock) -> void #foreign libphysx "PxReadWriteLock_lockWriter_mut";

PxReadWriteLock_unlockReader :: (self: *PxReadWriteLock) -> void #foreign libphysx "PxReadWriteLock_unlockReader_mut";

PxReadWriteLock_unlockWriter :: (self: *PxReadWriteLock) -> void #foreign libphysx "PxReadWriteLock_unlockWriter_mut";

PxProfilerCallback_zoneStart :: (self: *PxProfilerCallback, eventName: *u8, detached: bool, contextId: u64) -> *void #foreign libphysx "PxProfilerCallback_zoneStart_mut";

PxProfilerCallback_zoneEnd :: (self: *PxProfilerCallback, profilerData: *void, eventName: *u8, detached: bool, contextId: u64) -> void #foreign libphysx "PxProfilerCallback_zoneEnd_mut";

PxProfileScoped_new_alloc :: (callback: *PxProfilerCallback, eventName: *u8, detached: bool, contextId: u64) -> *PxProfileScoped #foreign libphysx;

PxProfileScoped_delete :: (self: *PxProfileScoped) -> void #foreign libphysx;

PxSListEntry_new :: () -> PxSListEntry #foreign libphysx;

PxSListEntry_next :: (self: *PxSListEntry) -> *PxSListEntry #foreign libphysx "PxSListEntry_next_mut";

PxSListImpl_new_alloc :: () -> *PxSListImpl #foreign libphysx;

PxSListImpl_delete :: (self: *PxSListImpl) -> void #foreign libphysx;

PxSListImpl_push :: (self: *PxSListImpl, entry: *PxSListEntry) -> void #foreign libphysx "PxSListImpl_push_mut";

PxSListImpl_pop :: (self: *PxSListImpl) -> *PxSListEntry #foreign libphysx "PxSListImpl_pop_mut";

PxSListImpl_flush :: (self: *PxSListImpl) -> *PxSListEntry #foreign libphysx "PxSListImpl_flush_mut";

PxSListImpl_getSize :: () -> u32 #foreign libphysx;

PxSyncImpl_new_alloc :: () -> *PxSyncImpl #foreign libphysx;

PxSyncImpl_delete :: (self: *PxSyncImpl) -> void #foreign libphysx;

PxSyncImpl_wait :: (self: *PxSyncImpl, milliseconds: u32) -> bool #foreign libphysx "PxSyncImpl_wait_mut";

PxSyncImpl_set :: (self: *PxSyncImpl) -> void #foreign libphysx "PxSyncImpl_set_mut";

PxSyncImpl_reset :: (self: *PxSyncImpl) -> void #foreign libphysx "PxSyncImpl_reset_mut";

PxSyncImpl_getSize :: () -> u32 #foreign libphysx;

PxRunnable_new_alloc :: () -> *PxRunnable #foreign libphysx;

PxRunnable_delete :: (self: *PxRunnable) -> void #foreign libphysx;

PxRunnable_execute :: (self: *PxRunnable) -> void #foreign libphysx "PxRunnable_execute_mut";

PxTlsAlloc :: () -> u32 #foreign libphysx "phys_PxTlsAlloc";

PxTlsFree :: (index: u32) -> void #foreign libphysx "phys_PxTlsFree";

PxTlsGet :: (index: u32) -> *void #foreign libphysx "phys_PxTlsGet";

PxTlsGetValue :: (index: u32) -> u64 #foreign libphysx "phys_PxTlsGetValue";

PxTlsSet :: (index: u32, value: *void) -> u32 #foreign libphysx "phys_PxTlsSet";

PxTlsSetValue :: (index: u32, value: u64) -> u32 #foreign libphysx "phys_PxTlsSetValue";

PxCounterFrequencyToTensOfNanos_new :: (inNum: u64, inDenom: u64) -> PxCounterFrequencyToTensOfNanos #foreign libphysx;

PxCounterFrequencyToTensOfNanos_toTensOfNanos :: (self: *PxCounterFrequencyToTensOfNanos, inCounter: u64) -> u64 #foreign libphysx;

PxTime_getBootCounterFrequency :: () -> *PxCounterFrequencyToTensOfNanos #foreign libphysx;

PxTime_getCounterFrequency :: () -> PxCounterFrequencyToTensOfNanos #foreign libphysx;

PxTime_getCurrentCounterValue :: () -> u64 #foreign libphysx;

PxTime_getCurrentTimeInTensOfNanoSeconds :: () -> u64 #foreign libphysx;

PxTime_new :: () -> PxTime #foreign libphysx;

PxTime_getElapsedSeconds :: (self: *PxTime) -> float64 #foreign libphysx "PxTime_getElapsedSeconds_mut";

PxTime_peekElapsedSeconds :: (self: *PxTime) -> float64 #foreign libphysx "PxTime_peekElapsedSeconds_mut";

PxTime_getLastTime :: (self: *PxTime) -> float64 #foreign libphysx;

PxVec2_new :: () -> Vector2 #foreign libphysx;

PxVec2_new :: (anon_param0: s32) -> Vector2 #foreign libphysx "PxVec2_new_1";

PxVec2_new :: (a: float) -> Vector2 #foreign libphysx "PxVec2_new_2";

PxVec2_new :: (nx: float, ny: float) -> Vector2 #foreign libphysx "PxVec2_new_3";

PxVec2_isZero :: (self: *Vector2) -> bool #foreign libphysx;

PxVec2_isFinite :: (self: *Vector2) -> bool #foreign libphysx;

PxVec2_isNormalized :: (self: *Vector2) -> bool #foreign libphysx;

PxVec2_magnitudeSquared :: (self: *Vector2) -> float #foreign libphysx;

PxVec2_magnitude :: (self: *Vector2) -> float #foreign libphysx;

PxVec2_dot :: (self: *Vector2, v: *Vector2) -> float #foreign libphysx;

PxVec2_getNormalized :: (self: *Vector2) -> Vector2 #foreign libphysx;

PxVec2_normalize :: (self: *Vector2) -> float #foreign libphysx "PxVec2_normalize_mut";

PxVec2_multiply :: (self: *Vector2, a: *Vector2) -> Vector2 #foreign libphysx;

PxVec2_minimum :: (self: *Vector2, v: *Vector2) -> Vector2 #foreign libphysx;

PxVec2_minElement :: (self: *Vector2) -> float #foreign libphysx;

PxVec2_maximum :: (self: *Vector2, v: *Vector2) -> Vector2 #foreign libphysx;

PxVec2_maxElement :: (self: *Vector2) -> float #foreign libphysx;

PxStridedData_new :: () -> PxStridedData #foreign libphysx;

PxBoundedData_new :: () -> PxBoundedData #foreign libphysx;

PxDebugPoint_new :: (p: *Vector3, c: *u32) -> PxDebugPoint #foreign libphysx;

PxDebugLine_new :: (p0: *Vector3, p1: *Vector3, c: *u32) -> PxDebugLine #foreign libphysx;

PxDebugTriangle_new :: (p0: *Vector3, p1: *Vector3, p2: *Vector3, c: *u32) -> PxDebugTriangle #foreign libphysx;

PxDebugText_new :: () -> PxDebugText #foreign libphysx;

PxDebugText_new :: (pos: *Vector3, sz: *float, clr: *u32, str: *u8) -> PxDebugText #foreign libphysx "PxDebugText_new_1";

PxRenderBuffer_delete :: (self: *PxRenderBuffer) -> void #foreign libphysx;

PxRenderBuffer_getNbPoints :: (self: *PxRenderBuffer) -> u32 #foreign libphysx;

PxRenderBuffer_getPoints :: (self: *PxRenderBuffer) -> *PxDebugPoint #foreign libphysx;

PxRenderBuffer_addPoint :: (self: *PxRenderBuffer, point: *PxDebugPoint) -> void #foreign libphysx "PxRenderBuffer_addPoint_mut";

PxRenderBuffer_getNbLines :: (self: *PxRenderBuffer) -> u32 #foreign libphysx;

PxRenderBuffer_getLines :: (self: *PxRenderBuffer) -> *PxDebugLine #foreign libphysx;

PxRenderBuffer_addLine :: (self: *PxRenderBuffer, line: *PxDebugLine) -> void #foreign libphysx "PxRenderBuffer_addLine_mut";

PxRenderBuffer_reserveLines :: (self: *PxRenderBuffer, nbLines: u32) -> *PxDebugLine #foreign libphysx "PxRenderBuffer_reserveLines_mut";

PxRenderBuffer_reservePoints :: (self: *PxRenderBuffer, nbLines: u32) -> *PxDebugPoint #foreign libphysx "PxRenderBuffer_reservePoints_mut";

PxRenderBuffer_getNbTriangles :: (self: *PxRenderBuffer) -> u32 #foreign libphysx;

PxRenderBuffer_getTriangles :: (self: *PxRenderBuffer) -> *PxDebugTriangle #foreign libphysx;

PxRenderBuffer_addTriangle :: (self: *PxRenderBuffer, triangle: *PxDebugTriangle) -> void #foreign libphysx "PxRenderBuffer_addTriangle_mut";

PxRenderBuffer_append :: (self: *PxRenderBuffer, other: *PxRenderBuffer) -> void #foreign libphysx "PxRenderBuffer_append_mut";

PxRenderBuffer_clear :: (self: *PxRenderBuffer) -> void #foreign libphysx "PxRenderBuffer_clear_mut";

PxRenderBuffer_shift :: (self: *PxRenderBuffer, delta: *Vector3) -> void #foreign libphysx "PxRenderBuffer_shift_mut";

PxRenderBuffer_empty :: (self: *PxRenderBuffer) -> bool #foreign libphysx;

PxProcessPxBaseCallback_delete :: (self: *PxProcessPxBaseCallback) -> void #foreign libphysx;

PxProcessPxBaseCallback_process :: (self: *PxProcessPxBaseCallback, anon_param0: *PxBase) -> void #foreign libphysx "PxProcessPxBaseCallback_process_mut";

PxSerializationContext_registerReference :: (self: *PxSerializationContext, base: *PxBase, kind: u32, reference: u64) -> void #foreign libphysx "PxSerializationContext_registerReference_mut";

PxSerializationContext_getCollection :: (self: *PxSerializationContext) -> *PxCollection #foreign libphysx;

PxSerializationContext_writeData :: (self: *PxSerializationContext, data: *void, size: u32) -> void #foreign libphysx "PxSerializationContext_writeData_mut";

PxSerializationContext_alignData :: (self: *PxSerializationContext, alignment: u32) -> void #foreign libphysx "PxSerializationContext_alignData_mut";

PxSerializationContext_writeName :: (self: *PxSerializationContext, name: *u8) -> void #foreign libphysx "PxSerializationContext_writeName_mut";

PxDeserializationContext_resolveReference :: (self: *PxDeserializationContext, kind: u32, reference: u64) -> *PxBase #foreign libphysx;

PxDeserializationContext_readName :: (self: *PxDeserializationContext, name: **u8) -> void #foreign libphysx "PxDeserializationContext_readName_mut";

PxDeserializationContext_alignExtraData :: (self: *PxDeserializationContext, alignment: u32) -> void #foreign libphysx "PxDeserializationContext_alignExtraData_mut";

PxSerializationRegistry_registerSerializer :: (self: *PxSerializationRegistry, type: u16, serializer: *PxSerializer) -> void #foreign libphysx "PxSerializationRegistry_registerSerializer_mut";

PxSerializationRegistry_unregisterSerializer :: (self: *PxSerializationRegistry, type: u16) -> *PxSerializer #foreign libphysx "PxSerializationRegistry_unregisterSerializer_mut";

PxSerializationRegistry_getSerializer :: (self: *PxSerializationRegistry, type: u16) -> *PxSerializer #foreign libphysx;

PxSerializationRegistry_registerRepXSerializer :: (self: *PxSerializationRegistry, type: u16, serializer: *PxRepXSerializer) -> void #foreign libphysx "PxSerializationRegistry_registerRepXSerializer_mut";

PxSerializationRegistry_unregisterRepXSerializer :: (self: *PxSerializationRegistry, type: u16) -> *PxRepXSerializer #foreign libphysx "PxSerializationRegistry_unregisterRepXSerializer_mut";

PxSerializationRegistry_getRepXSerializer :: (self: *PxSerializationRegistry, typeName: *u8) -> *PxRepXSerializer #foreign libphysx;

PxSerializationRegistry_release :: (self: *PxSerializationRegistry) -> void #foreign libphysx "PxSerializationRegistry_release_mut";

PxCollection_add :: (self: *PxCollection, object: *PxBase, id: u64) -> void #foreign libphysx "PxCollection_add_mut";

PxCollection_remove :: (self: *PxCollection, object: *PxBase) -> void #foreign libphysx "PxCollection_remove_mut";

PxCollection_contains :: (self: *PxCollection, object: *PxBase) -> bool #foreign libphysx;

PxCollection_addId :: (self: *PxCollection, object: *PxBase, id: u64) -> void #foreign libphysx "PxCollection_addId_mut";

PxCollection_removeId :: (self: *PxCollection, id: u64) -> void #foreign libphysx "PxCollection_removeId_mut";

PxCollection_add_mut :: (self: *PxCollection, collection: *PxCollection) -> void #foreign libphysx "PxCollection_add_mut_1";

PxCollection_remove_mut :: (self: *PxCollection, collection: *PxCollection) -> void #foreign libphysx "PxCollection_remove_mut_1";

PxCollection_getNbObjects :: (self: *PxCollection) -> u32 #foreign libphysx;

PxCollection_getObject :: (self: *PxCollection, index: u32) -> *PxBase #foreign libphysx;

PxCollection_getObjects :: (self: *PxCollection, userBuffer: **PxBase, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxCollection_find :: (self: *PxCollection, id: u64) -> *PxBase #foreign libphysx;

PxCollection_getNbIds :: (self: *PxCollection) -> u32 #foreign libphysx;

PxCollection_getIds :: (self: *PxCollection, userBuffer: *u64, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxCollection_getId :: (self: *PxCollection, object: *PxBase) -> u64 #foreign libphysx;

PxCollection_release :: (self: *PxCollection) -> void #foreign libphysx "PxCollection_release_mut";

PxCreateCollection :: () -> *PxCollection #foreign libphysx "phys_PxCreateCollection";

PxBase_release :: (self: *PxBase) -> void #foreign libphysx "PxBase_release_mut";

PxBase_getConcreteTypeName :: (self: *PxBase) -> *u8 #foreign libphysx;

PxBase_getConcreteType :: (self: *PxBase) -> u16 #foreign libphysx;

PxBase_setBaseFlag :: (self: *PxBase, flag: s32, value: bool) -> void #foreign libphysx "PxBase_setBaseFlag_mut";

PxBase_setBaseFlags :: (self: *PxBase, inFlags: u16) -> void #foreign libphysx "PxBase_setBaseFlags_mut";

PxBase_getBaseFlags :: (self: *PxBase) -> u16 #foreign libphysx;

PxBase_isReleasable :: (self: *PxBase) -> bool #foreign libphysx;

PxRefCounted_release :: (self: *PxRefCounted) -> void #foreign libphysx "PxRefCounted_release_mut";

PxRefCounted_getReferenceCount :: (self: *PxRefCounted) -> u32 #foreign libphysx;

PxRefCounted_acquireReference :: (self: *PxRefCounted) -> void #foreign libphysx "PxRefCounted_acquireReference_mut";

PxTolerancesScale_new :: (defaultLength: float, defaultSpeed: float) -> PxTolerancesScale #foreign libphysx;

PxTolerancesScale_isValid :: (self: *PxTolerancesScale) -> bool #foreign libphysx;

PxStringTable_allocateStr :: (self: *PxStringTable, inSrc: *u8) -> *u8 #foreign libphysx "PxStringTable_allocateStr_mut";

PxStringTable_release :: (self: *PxStringTable) -> void #foreign libphysx "PxStringTable_release_mut";

PxSerializer_getConcreteTypeName :: (self: *PxSerializer) -> *u8 #foreign libphysx;

PxSerializer_requiresObjects :: (self: *PxSerializer, anon_param0: *PxBase, anon_param1: *PxProcessPxBaseCallback) -> void #foreign libphysx;

PxSerializer_isSubordinate :: (self: *PxSerializer) -> bool #foreign libphysx;

PxSerializer_exportExtraData :: (self: *PxSerializer, anon_param0: *PxBase, anon_param1: *PxSerializationContext) -> void #foreign libphysx;

PxSerializer_exportData :: (self: *PxSerializer, anon_param0: *PxBase, anon_param1: *PxSerializationContext) -> void #foreign libphysx;

PxSerializer_registerReferences :: (self: *PxSerializer, obj: *PxBase, s: *PxSerializationContext) -> void #foreign libphysx;

PxSerializer_getClassSize :: (self: *PxSerializer) -> u64 #foreign libphysx;

PxSerializer_createObject :: (self: *PxSerializer, address: **u8, context_pod: *PxDeserializationContext) -> *PxBase #foreign libphysx;

PxSerializer_delete :: (self: *PxSerializer) -> void #foreign libphysx;

PxInsertionCallback_buildObjectFromData :: (self: *PxInsertionCallback, type: s32, data: *void) -> *PxBase #foreign libphysx "PxInsertionCallback_buildObjectFromData_mut";

PxTaskManager_setCpuDispatcher :: (self: *PxTaskManager, ref: *PxCpuDispatcher) -> void #foreign libphysx "PxTaskManager_setCpuDispatcher_mut";

PxTaskManager_getCpuDispatcher :: (self: *PxTaskManager) -> *PxCpuDispatcher #foreign libphysx;

PxTaskManager_resetDependencies :: (self: *PxTaskManager) -> void #foreign libphysx "PxTaskManager_resetDependencies_mut";

PxTaskManager_startSimulation :: (self: *PxTaskManager) -> void #foreign libphysx "PxTaskManager_startSimulation_mut";

PxTaskManager_stopSimulation :: (self: *PxTaskManager) -> void #foreign libphysx "PxTaskManager_stopSimulation_mut";

PxTaskManager_taskCompleted :: (self: *PxTaskManager, task: *PxTask) -> void #foreign libphysx "PxTaskManager_taskCompleted_mut";

PxTaskManager_getNamedTask :: (self: *PxTaskManager, name: *u8) -> u32 #foreign libphysx "PxTaskManager_getNamedTask_mut";

PxTaskManager_submitNamedTask :: (self: *PxTaskManager, task: *PxTask, name: *u8, type: s32) -> u32 #foreign libphysx "PxTaskManager_submitNamedTask_mut";

PxTaskManager_submitUnnamedTask :: (self: *PxTaskManager, task: *PxTask, type: s32) -> u32 #foreign libphysx "PxTaskManager_submitUnnamedTask_mut";

PxTaskManager_getTaskFromID :: (self: *PxTaskManager, id: u32) -> *PxTask #foreign libphysx "PxTaskManager_getTaskFromID_mut";

PxTaskManager_release :: (self: *PxTaskManager) -> void #foreign libphysx "PxTaskManager_release_mut";

PxTaskManager_createTaskManager :: (errorCallback: *PxErrorCallback, anon_param1: *PxCpuDispatcher) -> *PxTaskManager #foreign libphysx;

PxCpuDispatcher_submitTask :: (self: *PxCpuDispatcher, task: *PxBaseTask) -> void #foreign libphysx "PxCpuDispatcher_submitTask_mut";

PxCpuDispatcher_getWorkerCount :: (self: *PxCpuDispatcher) -> u32 #foreign libphysx;

PxCpuDispatcher_delete :: (self: *PxCpuDispatcher) -> void #foreign libphysx;

PxBaseTask_run :: (self: *PxBaseTask) -> void #foreign libphysx "PxBaseTask_run_mut";

PxBaseTask_getName :: (self: *PxBaseTask) -> *u8 #foreign libphysx;

PxBaseTask_addReference :: (self: *PxBaseTask) -> void #foreign libphysx "PxBaseTask_addReference_mut";

PxBaseTask_removeReference :: (self: *PxBaseTask) -> void #foreign libphysx "PxBaseTask_removeReference_mut";

PxBaseTask_getReference :: (self: *PxBaseTask) -> s32 #foreign libphysx;

PxBaseTask_release :: (self: *PxBaseTask) -> void #foreign libphysx "PxBaseTask_release_mut";

PxBaseTask_getTaskManager :: (self: *PxBaseTask) -> *PxTaskManager #foreign libphysx;

PxBaseTask_setContextId :: (self: *PxBaseTask, id: u64) -> void #foreign libphysx "PxBaseTask_setContextId_mut";

PxBaseTask_getContextId :: (self: *PxBaseTask) -> u64 #foreign libphysx;

PxTask_release :: (self: *PxTask) -> void #foreign libphysx "PxTask_release_mut";

PxTask_finishBefore :: (self: *PxTask, taskID: u32) -> void #foreign libphysx "PxTask_finishBefore_mut";

PxTask_startAfter :: (self: *PxTask, taskID: u32) -> void #foreign libphysx "PxTask_startAfter_mut";

PxTask_addReference :: (self: *PxTask) -> void #foreign libphysx "PxTask_addReference_mut";

PxTask_removeReference :: (self: *PxTask) -> void #foreign libphysx "PxTask_removeReference_mut";

PxTask_getReference :: (self: *PxTask) -> s32 #foreign libphysx;

PxTask_getTaskID :: (self: *PxTask) -> u32 #foreign libphysx;

PxTask_submitted :: (self: *PxTask) -> void #foreign libphysx "PxTask_submitted_mut";

PxLightCpuTask_setContinuation :: (self: *PxLightCpuTask, tm: *PxTaskManager, c: *PxBaseTask) -> void #foreign libphysx "PxLightCpuTask_setContinuation_mut";

PxLightCpuTask_setContinuation_mut :: (self: *PxLightCpuTask, c: *PxBaseTask) -> void #foreign libphysx "PxLightCpuTask_setContinuation_mut_1";

PxLightCpuTask_getContinuation :: (self: *PxLightCpuTask) -> *PxBaseTask #foreign libphysx;

PxLightCpuTask_removeReference :: (self: *PxLightCpuTask) -> void #foreign libphysx "PxLightCpuTask_removeReference_mut";

PxLightCpuTask_getReference :: (self: *PxLightCpuTask) -> s32 #foreign libphysx;

PxLightCpuTask_addReference :: (self: *PxLightCpuTask) -> void #foreign libphysx "PxLightCpuTask_addReference_mut";

PxLightCpuTask_release :: (self: *PxLightCpuTask) -> void #foreign libphysx "PxLightCpuTask_release_mut";

PxGeometry_getType :: (self: *PxGeometry) -> s32 #foreign libphysx;

PxBoxGeometry_new :: (hx: float, hy: float, hz: float) -> PxBoxGeometry #foreign libphysx;

PxBoxGeometry_new :: (halfExtents: Vector3) -> PxBoxGeometry #foreign libphysx "PxBoxGeometry_new_1";

PxBoxGeometry_isValid :: (self: *PxBoxGeometry) -> bool #foreign libphysx;

PxBVHRaycastCallback_delete :: (self: *PxBVHRaycastCallback) -> void #foreign libphysx;

PxBVHRaycastCallback_reportHit :: (self: *PxBVHRaycastCallback, boundsIndex: u32, distance: *float) -> bool #foreign libphysx "PxBVHRaycastCallback_reportHit_mut";

PxBVHOverlapCallback_delete :: (self: *PxBVHOverlapCallback) -> void #foreign libphysx;

PxBVHOverlapCallback_reportHit :: (self: *PxBVHOverlapCallback, boundsIndex: u32) -> bool #foreign libphysx "PxBVHOverlapCallback_reportHit_mut";

PxBVHTraversalCallback_delete :: (self: *PxBVHTraversalCallback) -> void #foreign libphysx;

PxBVHTraversalCallback_visitNode :: (self: *PxBVHTraversalCallback, bounds: *PxBounds3) -> bool #foreign libphysx "PxBVHTraversalCallback_visitNode_mut";

PxBVHTraversalCallback_reportLeaf :: (self: *PxBVHTraversalCallback, nbPrims: u32, prims: *u32) -> bool #foreign libphysx "PxBVHTraversalCallback_reportLeaf_mut";

PxBVH_raycast :: (self: *PxBVH, origin: *Vector3, unitDir: *Vector3, maxDist: float, cb: *PxBVHRaycastCallback, queryFlags: u32) -> bool #foreign libphysx;

PxBVH_sweep :: (self: *PxBVH, geom: *PxGeometry, pose: *PxTransform, unitDir: *Vector3, maxDist: float, cb: *PxBVHRaycastCallback, queryFlags: u32) -> bool #foreign libphysx;

PxBVH_overlap :: (self: *PxBVH, geom: *PxGeometry, pose: *PxTransform, cb: *PxBVHOverlapCallback, queryFlags: u32) -> bool #foreign libphysx;

PxBVH_cull :: (self: *PxBVH, nbPlanes: u32, planes: *Plane3, cb: *PxBVHOverlapCallback, queryFlags: u32) -> bool #foreign libphysx;

PxBVH_getNbBounds :: (self: *PxBVH) -> u32 #foreign libphysx;

PxBVH_getBounds :: (self: *PxBVH) -> *PxBounds3 #foreign libphysx;

PxBVH_getBoundsForModification :: (self: *PxBVH) -> *PxBounds3 #foreign libphysx "PxBVH_getBoundsForModification_mut";

PxBVH_refit :: (self: *PxBVH) -> void #foreign libphysx "PxBVH_refit_mut";

PxBVH_updateBounds :: (self: *PxBVH, boundsIndex: u32, newBounds: *PxBounds3) -> bool #foreign libphysx "PxBVH_updateBounds_mut";

PxBVH_partialRefit :: (self: *PxBVH) -> void #foreign libphysx "PxBVH_partialRefit_mut";

PxBVH_traverse :: (self: *PxBVH, cb: *PxBVHTraversalCallback) -> bool #foreign libphysx;

PxBVH_getConcreteTypeName :: (self: *PxBVH) -> *u8 #foreign libphysx;

PxCapsuleGeometry_new :: (radius_: float, halfHeight_: float) -> PxCapsuleGeometry #foreign libphysx;

PxCapsuleGeometry_isValid :: (self: *PxCapsuleGeometry) -> bool #foreign libphysx;

PxConvexMesh_getNbVertices :: (self: *PxConvexMesh) -> u32 #foreign libphysx;

PxConvexMesh_getVertices :: (self: *PxConvexMesh) -> *Vector3 #foreign libphysx;

PxConvexMesh_getIndexBuffer :: (self: *PxConvexMesh) -> *u8 #foreign libphysx;

PxConvexMesh_getNbPolygons :: (self: *PxConvexMesh) -> u32 #foreign libphysx;

PxConvexMesh_getPolygonData :: (self: *PxConvexMesh, index: u32, data: *PxHullPolygon) -> bool #foreign libphysx;

PxConvexMesh_release :: (self: *PxConvexMesh) -> void #foreign libphysx "PxConvexMesh_release_mut";

PxConvexMesh_getMassInformation :: (self: *PxConvexMesh, mass: *float, localInertia: *PxMat33, localCenterOfMass: *Vector3) -> void #foreign libphysx;

PxConvexMesh_getLocalBounds :: (self: *PxConvexMesh) -> PxBounds3 #foreign libphysx;

PxConvexMesh_getSDF :: (self: *PxConvexMesh) -> *float #foreign libphysx;

PxConvexMesh_getConcreteTypeName :: (self: *PxConvexMesh) -> *u8 #foreign libphysx;

PxConvexMesh_isGpuCompatible :: (self: *PxConvexMesh) -> bool #foreign libphysx;

PxMeshScale_new :: () -> PxMeshScale #foreign libphysx;

PxMeshScale_new :: (r: float) -> PxMeshScale #foreign libphysx "PxMeshScale_new_1";

PxMeshScale_new :: (s: *Vector3) -> PxMeshScale #foreign libphysx "PxMeshScale_new_2";

PxMeshScale_new :: (s: *Vector3, r: *Quaternion) -> PxMeshScale #foreign libphysx "PxMeshScale_new_3";

PxMeshScale_isIdentity :: (self: *PxMeshScale) -> bool #foreign libphysx;

PxMeshScale_getInverse :: (self: *PxMeshScale) -> PxMeshScale #foreign libphysx;

PxMeshScale_toMat33 :: (self: *PxMeshScale) -> PxMat33 #foreign libphysx;

PxMeshScale_hasNegativeDeterminant :: (self: *PxMeshScale) -> bool #foreign libphysx;

PxMeshScale_transform :: (self: *PxMeshScale, v: *Vector3) -> Vector3 #foreign libphysx;

PxMeshScale_isValidForTriangleMesh :: (self: *PxMeshScale) -> bool #foreign libphysx;

PxMeshScale_isValidForConvexMesh :: (self: *PxMeshScale) -> bool #foreign libphysx;

PxConvexMeshGeometry_new :: (mesh: *PxConvexMesh, scaling: *PxMeshScale, flags: u8) -> PxConvexMeshGeometry #foreign libphysx;

PxConvexMeshGeometry_isValid :: (self: *PxConvexMeshGeometry) -> bool #foreign libphysx;

PxSphereGeometry_new :: (ir: float) -> PxSphereGeometry #foreign libphysx;

PxSphereGeometry_isValid :: (self: *PxSphereGeometry) -> bool #foreign libphysx;

PxPlaneGeometry_new :: () -> PxPlaneGeometry #foreign libphysx;

PxPlaneGeometry_isValid :: (self: *PxPlaneGeometry) -> bool #foreign libphysx;

PxTriangleMeshGeometry_new :: (mesh: *PxTriangleMesh, scaling: *PxMeshScale, flags: u8) -> PxTriangleMeshGeometry #foreign libphysx;

PxTriangleMeshGeometry_isValid :: (self: *PxTriangleMeshGeometry) -> bool #foreign libphysx;

PxHeightFieldGeometry_new :: (hf: *PxHeightField, flags: u8, heightScale_: float, rowScale_: float, columnScale_: float) -> PxHeightFieldGeometry #foreign libphysx;

PxHeightFieldGeometry_isValid :: (self: *PxHeightFieldGeometry) -> bool #foreign libphysx;

PxParticleSystemGeometry_new :: () -> PxParticleSystemGeometry #foreign libphysx;

PxParticleSystemGeometry_isValid :: (self: *PxParticleSystemGeometry) -> bool #foreign libphysx;

PxHairSystemGeometry_new :: () -> PxHairSystemGeometry #foreign libphysx;

PxHairSystemGeometry_isValid :: (self: *PxHairSystemGeometry) -> bool #foreign libphysx;

PxTetrahedronMeshGeometry_new :: (mesh: *PxTetrahedronMesh) -> PxTetrahedronMeshGeometry #foreign libphysx;

PxTetrahedronMeshGeometry_isValid :: (self: *PxTetrahedronMeshGeometry) -> bool #foreign libphysx;

PxQueryHit_new :: () -> PxQueryHit #foreign libphysx;

PxLocationHit_new :: () -> PxLocationHit #foreign libphysx;

PxLocationHit_hadInitialOverlap :: (self: *PxLocationHit) -> bool #foreign libphysx;

PxGeomRaycastHit_new :: () -> PxGeomRaycastHit #foreign libphysx;

PxGeomOverlapHit_new :: () -> PxGeomOverlapHit #foreign libphysx;

PxGeomSweepHit_new :: () -> PxGeomSweepHit #foreign libphysx;

PxGeomIndexPair_new :: () -> PxGeomIndexPair #foreign libphysx;

PxGeomIndexPair_new :: (_id0: u32, _id1: u32) -> PxGeomIndexPair #foreign libphysx "PxGeomIndexPair_new_1";

PxCustomGeometry_getUniqueID :: () -> u32 #foreign libphysx "phys_PxCustomGeometry_getUniqueID";

PxCustomGeometryType_new :: () -> PxCustomGeometryType #foreign libphysx;

PxCustomGeometryType_INVALID :: () -> PxCustomGeometryType #foreign libphysx;

PxCustomGeometryCallbacks_getCustomType :: (self: *PxCustomGeometryCallbacks) -> PxCustomGeometryType #foreign libphysx;

PxCustomGeometryCallbacks_getLocalBounds :: (self: *PxCustomGeometryCallbacks, geometry: *PxGeometry) -> PxBounds3 #foreign libphysx;

PxCustomGeometryCallbacks_raycast :: (self: *PxCustomGeometryCallbacks, origin: *Vector3, unitDir: *Vector3, geom: *PxGeometry, pose: *PxTransform, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *PxGeomRaycastHit, stride: u32, threadContext: *PxQueryThreadContext) -> u32 #foreign libphysx;

PxCustomGeometryCallbacks_overlap :: (self: *PxCustomGeometryCallbacks, geom0: *PxGeometry, pose0: *PxTransform, geom1: *PxGeometry, pose1: *PxTransform, threadContext: *PxQueryThreadContext) -> bool #foreign libphysx;

PxCustomGeometryCallbacks_sweep :: (self: *PxCustomGeometryCallbacks, unitDir: *Vector3, maxDist: float, geom0: *PxGeometry, pose0: *PxTransform, geom1: *PxGeometry, pose1: *PxTransform, sweepHit: *PxGeomSweepHit, hitFlags: u16, inflation: float, threadContext: *PxQueryThreadContext) -> bool #foreign libphysx;

PxCustomGeometryCallbacks_computeMassProperties :: (self: *PxCustomGeometryCallbacks, geometry: *PxGeometry, massProperties: *PxMassProperties) -> void #foreign libphysx;

PxCustomGeometryCallbacks_usePersistentContactManifold :: (self: *PxCustomGeometryCallbacks, geometry: *PxGeometry, breakingThreshold: *float) -> bool #foreign libphysx;

PxCustomGeometryCallbacks_delete :: (self: *PxCustomGeometryCallbacks) -> void #foreign libphysx;

PxCustomGeometry_new :: () -> PxCustomGeometry #foreign libphysx;

PxCustomGeometry_new :: (_callbacks: *PxCustomGeometryCallbacks) -> PxCustomGeometry #foreign libphysx "PxCustomGeometry_new_1";

PxCustomGeometry_isValid :: (self: *PxCustomGeometry) -> bool #foreign libphysx;

PxCustomGeometry_getCustomType :: (self: *PxCustomGeometry) -> PxCustomGeometryType #foreign libphysx;

PxGeometryHolder_getType :: (self: *PxGeometryHolder) -> s32 #foreign libphysx;

PxGeometryHolder_any :: (self: *PxGeometryHolder) -> *PxGeometry #foreign libphysx "PxGeometryHolder_any_mut";

PxGeometryHolder_sphere :: (self: *PxGeometryHolder) -> *PxSphereGeometry #foreign libphysx "PxGeometryHolder_sphere_mut";

PxGeometryHolder_plane :: (self: *PxGeometryHolder) -> *PxPlaneGeometry #foreign libphysx "PxGeometryHolder_plane_mut";

PxGeometryHolder_capsule :: (self: *PxGeometryHolder) -> *PxCapsuleGeometry #foreign libphysx "PxGeometryHolder_capsule_mut";

PxGeometryHolder_box :: (self: *PxGeometryHolder) -> *PxBoxGeometry #foreign libphysx "PxGeometryHolder_box_mut";

PxGeometryHolder_convexMesh :: (self: *PxGeometryHolder) -> *PxConvexMeshGeometry #foreign libphysx "PxGeometryHolder_convexMesh_mut";

PxGeometryHolder_tetMesh :: (self: *PxGeometryHolder) -> *PxTetrahedronMeshGeometry #foreign libphysx "PxGeometryHolder_tetMesh_mut";

PxGeometryHolder_triangleMesh :: (self: *PxGeometryHolder) -> *PxTriangleMeshGeometry #foreign libphysx "PxGeometryHolder_triangleMesh_mut";

PxGeometryHolder_heightField :: (self: *PxGeometryHolder) -> *PxHeightFieldGeometry #foreign libphysx "PxGeometryHolder_heightField_mut";

PxGeometryHolder_particleSystem :: (self: *PxGeometryHolder) -> *PxParticleSystemGeometry #foreign libphysx "PxGeometryHolder_particleSystem_mut";

PxGeometryHolder_hairSystem :: (self: *PxGeometryHolder) -> *PxHairSystemGeometry #foreign libphysx "PxGeometryHolder_hairSystem_mut";

PxGeometryHolder_custom :: (self: *PxGeometryHolder) -> *PxCustomGeometry #foreign libphysx "PxGeometryHolder_custom_mut";

PxGeometryHolder_storeAny :: (self: *PxGeometryHolder, geometry: *PxGeometry) -> void #foreign libphysx "PxGeometryHolder_storeAny_mut";

PxGeometryHolder_new :: () -> PxGeometryHolder #foreign libphysx;

PxGeometryHolder_new :: (geometry: *PxGeometry) -> PxGeometryHolder #foreign libphysx "PxGeometryHolder_new_1";

PxGeometryQuery_raycast :: (origin: *Vector3, unitDir: *Vector3, geom: *PxGeometry, pose: *PxTransform, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *PxGeomRaycastHit, stride: u32, queryFlags: u32, threadContext: *PxQueryThreadContext) -> u32 #foreign libphysx;

PxGeometryQuery_overlap :: (geom0: *PxGeometry, pose0: *PxTransform, geom1: *PxGeometry, pose1: *PxTransform, queryFlags: u32, threadContext: *PxQueryThreadContext) -> bool #foreign libphysx;

PxGeometryQuery_sweep :: (unitDir: *Vector3, maxDist: float, geom0: *PxGeometry, pose0: *PxTransform, geom1: *PxGeometry, pose1: *PxTransform, sweepHit: *PxGeomSweepHit, hitFlags: u16, inflation: float, queryFlags: u32, threadContext: *PxQueryThreadContext) -> bool #foreign libphysx;

PxGeometryQuery_computePenetration :: (direction: *Vector3, depth: *float, geom0: *PxGeometry, pose0: *PxTransform, geom1: *PxGeometry, pose1: *PxTransform, queryFlags: u32) -> bool #foreign libphysx;

PxGeometryQuery_pointDistance :: (point: *Vector3, geom: *PxGeometry, pose: *PxTransform, closestPoint: *Vector3, closestIndex: *u32, queryFlags: u32) -> float #foreign libphysx;

PxGeometryQuery_computeGeomBounds :: (bounds: *PxBounds3, geom: *PxGeometry, pose: *PxTransform, offset: float, inflation: float, queryFlags: u32) -> void #foreign libphysx;

PxGeometryQuery_isValid :: (geom: *PxGeometry) -> bool #foreign libphysx;

PxHeightFieldSample_tessFlag :: (self: *PxHeightFieldSample) -> u8 #foreign libphysx;

PxHeightFieldSample_setTessFlag :: (self: *PxHeightFieldSample) -> void #foreign libphysx "PxHeightFieldSample_setTessFlag_mut";

PxHeightFieldSample_clearTessFlag :: (self: *PxHeightFieldSample) -> void #foreign libphysx "PxHeightFieldSample_clearTessFlag_mut";

PxHeightField_release :: (self: *PxHeightField) -> void #foreign libphysx "PxHeightField_release_mut";

PxHeightField_saveCells :: (self: *PxHeightField, destBuffer: *void, destBufferSize: u32) -> u32 #foreign libphysx;

PxHeightField_modifySamples :: (self: *PxHeightField, startCol: s32, startRow: s32, subfieldDesc: *PxHeightFieldDesc, shrinkBounds: bool) -> bool #foreign libphysx "PxHeightField_modifySamples_mut";

PxHeightField_getNbRows :: (self: *PxHeightField) -> u32 #foreign libphysx;

PxHeightField_getNbColumns :: (self: *PxHeightField) -> u32 #foreign libphysx;

PxHeightField_getFormat :: (self: *PxHeightField) -> s32 #foreign libphysx;

PxHeightField_getSampleStride :: (self: *PxHeightField) -> u32 #foreign libphysx;

PxHeightField_getConvexEdgeThreshold :: (self: *PxHeightField) -> float #foreign libphysx;

PxHeightField_getFlags :: (self: *PxHeightField) -> u16 #foreign libphysx;

PxHeightField_getHeight :: (self: *PxHeightField, x: float, z: float) -> float #foreign libphysx;

PxHeightField_getTriangleMaterialIndex :: (self: *PxHeightField, triangleIndex: u32) -> u16 #foreign libphysx;

PxHeightField_getTriangleNormal :: (self: *PxHeightField, triangleIndex: u32) -> Vector3 #foreign libphysx;

PxHeightField_getSample :: (self: *PxHeightField, row: u32, column: u32) -> *PxHeightFieldSample #foreign libphysx;

PxHeightField_getTimestamp :: (self: *PxHeightField) -> u32 #foreign libphysx;

PxHeightField_getConcreteTypeName :: (self: *PxHeightField) -> *u8 #foreign libphysx;

PxHeightFieldDesc_new :: () -> PxHeightFieldDesc #foreign libphysx;

PxHeightFieldDesc_setToDefault :: (self: *PxHeightFieldDesc) -> void #foreign libphysx "PxHeightFieldDesc_setToDefault_mut";

PxHeightFieldDesc_isValid :: (self: *PxHeightFieldDesc) -> bool #foreign libphysx;

PxMeshQuery_getTriangle :: (triGeom: *PxTriangleMeshGeometry, transform: *PxTransform, triangleIndex: u32, triangle: *PxTriangle, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign libphysx;

PxMeshQuery_getTriangle :: (hfGeom: *PxHeightFieldGeometry, transform: *PxTransform, triangleIndex: u32, triangle: *PxTriangle, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign libphysx "PxMeshQuery_getTriangle_1";

PxMeshQuery_findOverlapTriangleMesh :: (geom: *PxGeometry, geomPose: *PxTransform, meshGeom: *PxTriangleMeshGeometry, meshPose: *PxTransform, results: *u32, maxResults: u32, startIndex: u32, overflow: *bool, queryFlags: u32) -> u32 #foreign libphysx;

PxMeshQuery_findOverlapHeightField :: (geom: *PxGeometry, geomPose: *PxTransform, hfGeom: *PxHeightFieldGeometry, hfPose: *PxTransform, results: *u32, maxResults: u32, startIndex: u32, overflow: *bool, queryFlags: u32) -> u32 #foreign libphysx;

PxMeshQuery_sweep :: (unitDir: *Vector3, distance: float, geom: *PxGeometry, pose: *PxTransform, triangleCount: u32, triangles: *PxTriangle, sweepHit: *PxGeomSweepHit, hitFlags: u16, cachedIndex: *u32, inflation: float, doubleSided: bool, queryFlags: u32) -> bool #foreign libphysx;

PxSimpleTriangleMesh_new :: () -> PxSimpleTriangleMesh #foreign libphysx;

PxSimpleTriangleMesh_setToDefault :: (self: *PxSimpleTriangleMesh) -> void #foreign libphysx "PxSimpleTriangleMesh_setToDefault_mut";

PxSimpleTriangleMesh_isValid :: (self: *PxSimpleTriangleMesh) -> bool #foreign libphysx;

PxTriangle_new_alloc :: () -> *PxTriangle #foreign libphysx;

PxTriangle_new_alloc :: (p0: *Vector3, p1: *Vector3, p2: *Vector3) -> *PxTriangle #foreign libphysx "PxTriangle_new_alloc_1";

PxTriangle_delete :: (self: *PxTriangle) -> void #foreign libphysx;

PxTriangle_normal :: (self: *PxTriangle, _normal: *Vector3) -> void #foreign libphysx;

PxTriangle_denormalizedNormal :: (self: *PxTriangle, _normal: *Vector3) -> void #foreign libphysx;

PxTriangle_area :: (self: *PxTriangle) -> float #foreign libphysx;

PxTriangle_pointFromUV :: (self: *PxTriangle, u: float, v: float) -> Vector3 #foreign libphysx;

PxTrianglePadded_new_alloc :: () -> *PxTrianglePadded #foreign libphysx;

PxTrianglePadded_delete :: (self: *PxTrianglePadded) -> void #foreign libphysx;

PxTriangleMesh_getNbVertices :: (self: *PxTriangleMesh) -> u32 #foreign libphysx;

PxTriangleMesh_getVertices :: (self: *PxTriangleMesh) -> *Vector3 #foreign libphysx;

PxTriangleMesh_getVerticesForModification :: (self: *PxTriangleMesh) -> *Vector3 #foreign libphysx "PxTriangleMesh_getVerticesForModification_mut";

PxTriangleMesh_refitBVH :: (self: *PxTriangleMesh) -> PxBounds3 #foreign libphysx "PxTriangleMesh_refitBVH_mut";

PxTriangleMesh_getNbTriangles :: (self: *PxTriangleMesh) -> u32 #foreign libphysx;

PxTriangleMesh_getTriangles :: (self: *PxTriangleMesh) -> *void #foreign libphysx;

PxTriangleMesh_getTriangleMeshFlags :: (self: *PxTriangleMesh) -> u8 #foreign libphysx;

PxTriangleMesh_getTrianglesRemap :: (self: *PxTriangleMesh) -> *u32 #foreign libphysx;

PxTriangleMesh_release :: (self: *PxTriangleMesh) -> void #foreign libphysx "PxTriangleMesh_release_mut";

PxTriangleMesh_getTriangleMaterialIndex :: (self: *PxTriangleMesh, triangleIndex: u32) -> u16 #foreign libphysx;

PxTriangleMesh_getLocalBounds :: (self: *PxTriangleMesh) -> PxBounds3 #foreign libphysx;

PxTriangleMesh_getSDF :: (self: *PxTriangleMesh) -> *float #foreign libphysx;

PxTriangleMesh_getSDFDimensions :: (self: *PxTriangleMesh, numX: *u32, numY: *u32, numZ: *u32) -> void #foreign libphysx;

PxTriangleMesh_setPreferSDFProjection :: (self: *PxTriangleMesh, preferProjection: bool) -> void #foreign libphysx "PxTriangleMesh_setPreferSDFProjection_mut";

PxTriangleMesh_getPreferSDFProjection :: (self: *PxTriangleMesh) -> bool #foreign libphysx;

PxTriangleMesh_getMassInformation :: (self: *PxTriangleMesh, mass: *float, localInertia: *PxMat33, localCenterOfMass: *Vector3) -> void #foreign libphysx;

PxTetrahedron_new_alloc :: () -> *PxTetrahedron #foreign libphysx;

PxTetrahedron_new_alloc :: (p0: *Vector3, p1: *Vector3, p2: *Vector3, p3: *Vector3) -> *PxTetrahedron #foreign libphysx "PxTetrahedron_new_alloc_1";

PxTetrahedron_delete :: (self: *PxTetrahedron) -> void #foreign libphysx;

PxSoftBodyAuxData_release :: (self: *PxSoftBodyAuxData) -> void #foreign libphysx "PxSoftBodyAuxData_release_mut";

PxTetrahedronMesh_getNbVertices :: (self: *PxTetrahedronMesh) -> u32 #foreign libphysx;

PxTetrahedronMesh_getVertices :: (self: *PxTetrahedronMesh) -> *Vector3 #foreign libphysx;

PxTetrahedronMesh_getNbTetrahedrons :: (self: *PxTetrahedronMesh) -> u32 #foreign libphysx;

PxTetrahedronMesh_getTetrahedrons :: (self: *PxTetrahedronMesh) -> *void #foreign libphysx;

PxTetrahedronMesh_getTetrahedronMeshFlags :: (self: *PxTetrahedronMesh) -> u8 #foreign libphysx;

PxTetrahedronMesh_getTetrahedraRemap :: (self: *PxTetrahedronMesh) -> *u32 #foreign libphysx;

PxTetrahedronMesh_getLocalBounds :: (self: *PxTetrahedronMesh) -> PxBounds3 #foreign libphysx;

PxTetrahedronMesh_release :: (self: *PxTetrahedronMesh) -> void #foreign libphysx "PxTetrahedronMesh_release_mut";

PxSoftBodyMesh_getCollisionMesh :: (self: *PxSoftBodyMesh) -> *PxTetrahedronMesh #foreign libphysx "PxSoftBodyMesh_getCollisionMesh_mut";

PxSoftBodyMesh_getSimulationMesh :: (self: *PxSoftBodyMesh) -> *PxTetrahedronMesh #foreign libphysx "PxSoftBodyMesh_getSimulationMesh_mut";

PxSoftBodyMesh_getSoftBodyAuxData :: (self: *PxSoftBodyMesh) -> *PxSoftBodyAuxData #foreign libphysx "PxSoftBodyMesh_getSoftBodyAuxData_mut";

PxSoftBodyMesh_release :: (self: *PxSoftBodyMesh) -> void #foreign libphysx "PxSoftBodyMesh_release_mut";

PxCollisionMeshMappingData_release :: (self: *PxCollisionMeshMappingData) -> void #foreign libphysx "PxCollisionMeshMappingData_release_mut";

PxCollisionTetrahedronMeshData_getMesh :: (self: *PxCollisionTetrahedronMeshData) -> *PxTetrahedronMeshData #foreign libphysx "PxCollisionTetrahedronMeshData_getMesh_mut";

PxCollisionTetrahedronMeshData_getData :: (self: *PxCollisionTetrahedronMeshData) -> *PxSoftBodyCollisionData #foreign libphysx "PxCollisionTetrahedronMeshData_getData_mut";

PxCollisionTetrahedronMeshData_release :: (self: *PxCollisionTetrahedronMeshData) -> void #foreign libphysx "PxCollisionTetrahedronMeshData_release_mut";

PxSimulationTetrahedronMeshData_getMesh :: (self: *PxSimulationTetrahedronMeshData) -> *PxTetrahedronMeshData #foreign libphysx "PxSimulationTetrahedronMeshData_getMesh_mut";

PxSimulationTetrahedronMeshData_getData :: (self: *PxSimulationTetrahedronMeshData) -> *PxSoftBodySimulationData #foreign libphysx "PxSimulationTetrahedronMeshData_getData_mut";

PxSimulationTetrahedronMeshData_release :: (self: *PxSimulationTetrahedronMeshData) -> void #foreign libphysx "PxSimulationTetrahedronMeshData_release_mut";

PxActor_release :: (self: *PxActor) -> void #foreign libphysx "PxActor_release_mut";

PxActor_getType :: (self: *PxActor) -> s32 #foreign libphysx;

PxActor_getScene :: (self: *PxActor) -> *PxScene #foreign libphysx;

PxActor_setName :: (self: *PxActor, name: *u8) -> void #foreign libphysx "PxActor_setName_mut";

PxActor_getName :: (self: *PxActor) -> *u8 #foreign libphysx;

PxActor_getWorldBounds :: (self: *PxActor, inflation: float) -> PxBounds3 #foreign libphysx;

PxActor_setActorFlag :: (self: *PxActor, flag: s32, value: bool) -> void #foreign libphysx "PxActor_setActorFlag_mut";

PxActor_setActorFlags :: (self: *PxActor, inFlags: u8) -> void #foreign libphysx "PxActor_setActorFlags_mut";

PxActor_getActorFlags :: (self: *PxActor) -> u8 #foreign libphysx;

PxActor_setDominanceGroup :: (self: *PxActor, dominanceGroup: u8) -> void #foreign libphysx "PxActor_setDominanceGroup_mut";

PxActor_getDominanceGroup :: (self: *PxActor) -> u8 #foreign libphysx;

PxActor_setOwnerClient :: (self: *PxActor, inClient: u8) -> void #foreign libphysx "PxActor_setOwnerClient_mut";

PxActor_getOwnerClient :: (self: *PxActor) -> u8 #foreign libphysx;

PxActor_getAggregate :: (self: *PxActor) -> *PxAggregate #foreign libphysx;

PxGetAggregateFilterHint :: (type: s32, enableSelfCollision: bool) -> u32 #foreign libphysx "phys_PxGetAggregateFilterHint";

PxGetAggregateSelfCollisionBit :: (hint: u32) -> u32 #foreign libphysx "phys_PxGetAggregateSelfCollisionBit";

PxGetAggregateType :: (hint: u32) -> s32 #foreign libphysx "phys_PxGetAggregateType";

PxAggregate_release :: (self: *PxAggregate) -> void #foreign libphysx "PxAggregate_release_mut";

PxAggregate_addActor :: (self: *PxAggregate, actor: *PxActor, bvh: *PxBVH) -> bool #foreign libphysx "PxAggregate_addActor_mut";

PxAggregate_removeActor :: (self: *PxAggregate, actor: *PxActor) -> bool #foreign libphysx "PxAggregate_removeActor_mut";

PxAggregate_addArticulation :: (self: *PxAggregate, articulation: *PxArticulationReducedCoordinate) -> bool #foreign libphysx "PxAggregate_addArticulation_mut";

PxAggregate_removeArticulation :: (self: *PxAggregate, articulation: *PxArticulationReducedCoordinate) -> bool #foreign libphysx "PxAggregate_removeArticulation_mut";

PxAggregate_getNbActors :: (self: *PxAggregate) -> u32 #foreign libphysx;

PxAggregate_getMaxNbShapes :: (self: *PxAggregate) -> u32 #foreign libphysx;

PxAggregate_getActors :: (self: *PxAggregate, userBuffer: **PxActor, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxAggregate_getScene :: (self: *PxAggregate) -> *PxScene #foreign libphysx "PxAggregate_getScene_mut";

PxAggregate_getSelfCollision :: (self: *PxAggregate) -> bool #foreign libphysx;

PxAggregate_getConcreteTypeName :: (self: *PxAggregate) -> *u8 #foreign libphysx;

PxConstraintInvMassScale_new :: () -> PxConstraintInvMassScale #foreign libphysx;

PxConstraintInvMassScale_new :: (lin0: float, ang0: float, lin1: float, ang1: float) -> PxConstraintInvMassScale #foreign libphysx "PxConstraintInvMassScale_new_1";

PxConstraintVisualizer_visualizeJointFrames :: (self: *PxConstraintVisualizer, parent: *PxTransform, child: *PxTransform) -> void #foreign libphysx "PxConstraintVisualizer_visualizeJointFrames_mut";

PxConstraintVisualizer_visualizeLinearLimit :: (self: *PxConstraintVisualizer, t0: *PxTransform, t1: *PxTransform, value: float, active: bool) -> void #foreign libphysx "PxConstraintVisualizer_visualizeLinearLimit_mut";

PxConstraintVisualizer_visualizeAngularLimit :: (self: *PxConstraintVisualizer, t0: *PxTransform, lower: float, upper: float, active: bool) -> void #foreign libphysx "PxConstraintVisualizer_visualizeAngularLimit_mut";

PxConstraintVisualizer_visualizeLimitCone :: (self: *PxConstraintVisualizer, t: *PxTransform, tanQSwingY: float, tanQSwingZ: float, active: bool) -> void #foreign libphysx "PxConstraintVisualizer_visualizeLimitCone_mut";

PxConstraintVisualizer_visualizeDoubleCone :: (self: *PxConstraintVisualizer, t: *PxTransform, angle: float, active: bool) -> void #foreign libphysx "PxConstraintVisualizer_visualizeDoubleCone_mut";

PxConstraintVisualizer_visualizeLine :: (self: *PxConstraintVisualizer, p0: *Vector3, p1: *Vector3, color: u32) -> void #foreign libphysx "PxConstraintVisualizer_visualizeLine_mut";

PxConstraintConnector_prepareData :: (self: *PxConstraintConnector) -> *void #foreign libphysx "PxConstraintConnector_prepareData_mut";

PxConstraintConnector_onConstraintRelease :: (self: *PxConstraintConnector) -> void #foreign libphysx "PxConstraintConnector_onConstraintRelease_mut";

PxConstraintConnector_onComShift :: (self: *PxConstraintConnector, actor: u32) -> void #foreign libphysx "PxConstraintConnector_onComShift_mut";

PxConstraintConnector_onOriginShift :: (self: *PxConstraintConnector, shift: *Vector3) -> void #foreign libphysx "PxConstraintConnector_onOriginShift_mut";

PxConstraintConnector_getSerializable :: (self: *PxConstraintConnector) -> *PxBase #foreign libphysx "PxConstraintConnector_getSerializable_mut";

PxConstraintConnector_getConstantBlock :: (self: *PxConstraintConnector) -> *void #foreign libphysx;

PxConstraintConnector_connectToConstraint :: (self: *PxConstraintConnector, anon_param0: *PxConstraint) -> void #foreign libphysx "PxConstraintConnector_connectToConstraint_mut";

PxConstraintConnector_delete :: (self: *PxConstraintConnector) -> void #foreign libphysx;

PxSolverBody_new :: () -> PxSolverBody #foreign libphysx;

PxSolverBodyData_projectVelocity :: (self: *PxSolverBodyData, lin: *Vector3, ang: *Vector3) -> float #foreign libphysx;

PxSolverConstraintPrepDesc_delete :: (self: *PxSolverConstraintPrepDesc) -> void #foreign libphysx;

PxConstraintAllocator_reserveConstraintData :: (self: *PxConstraintAllocator, byteSize: u32) -> *u8 #foreign libphysx "PxConstraintAllocator_reserveConstraintData_mut";

PxConstraintAllocator_reserveFrictionData :: (self: *PxConstraintAllocator, byteSize: u32) -> *u8 #foreign libphysx "PxConstraintAllocator_reserveFrictionData_mut";

PxConstraintAllocator_delete :: (self: *PxConstraintAllocator) -> void #foreign libphysx;

PxArticulationLimit_new :: () -> PxArticulationLimit #foreign libphysx;

PxArticulationLimit_new :: (low_: float, high_: float) -> PxArticulationLimit #foreign libphysx "PxArticulationLimit_new_1";

PxArticulationDrive_new :: () -> PxArticulationDrive #foreign libphysx;

PxArticulationDrive_new :: (stiffness_: float, damping_: float, maxForce_: float, driveType: s32) -> PxArticulationDrive #foreign libphysx "PxArticulationDrive_new_1";

PxTGSSolverBodyVel_projectVelocity :: (self: *PxTGSSolverBodyVel, lin: *Vector3, ang: *Vector3) -> float #foreign libphysx;

PxTGSSolverBodyData_projectVelocity :: (self: *PxTGSSolverBodyData, linear: *Vector3, angular: *Vector3) -> float #foreign libphysx;

PxTGSSolverConstraintPrepDesc_delete :: (self: *PxTGSSolverConstraintPrepDesc) -> void #foreign libphysx;

PxArticulationAttachment_setRestLength :: (self: *PxArticulationAttachment, restLength: float) -> void #foreign libphysx "PxArticulationAttachment_setRestLength_mut";

PxArticulationAttachment_getRestLength :: (self: *PxArticulationAttachment) -> float #foreign libphysx;

PxArticulationAttachment_setLimitParameters :: (self: *PxArticulationAttachment, parameters: *PxArticulationTendonLimit) -> void #foreign libphysx "PxArticulationAttachment_setLimitParameters_mut";

PxArticulationAttachment_getLimitParameters :: (self: *PxArticulationAttachment) -> PxArticulationTendonLimit #foreign libphysx;

PxArticulationAttachment_setRelativeOffset :: (self: *PxArticulationAttachment, offset: *Vector3) -> void #foreign libphysx "PxArticulationAttachment_setRelativeOffset_mut";

PxArticulationAttachment_getRelativeOffset :: (self: *PxArticulationAttachment) -> Vector3 #foreign libphysx;

PxArticulationAttachment_setCoefficient :: (self: *PxArticulationAttachment, coefficient: float) -> void #foreign libphysx "PxArticulationAttachment_setCoefficient_mut";

PxArticulationAttachment_getCoefficient :: (self: *PxArticulationAttachment) -> float #foreign libphysx;

PxArticulationAttachment_getLink :: (self: *PxArticulationAttachment) -> *PxArticulationLink #foreign libphysx;

PxArticulationAttachment_getParent :: (self: *PxArticulationAttachment) -> *PxArticulationAttachment #foreign libphysx;

PxArticulationAttachment_isLeaf :: (self: *PxArticulationAttachment) -> bool #foreign libphysx;

PxArticulationAttachment_getTendon :: (self: *PxArticulationAttachment) -> *PxArticulationSpatialTendon #foreign libphysx;

PxArticulationAttachment_release :: (self: *PxArticulationAttachment) -> void #foreign libphysx "PxArticulationAttachment_release_mut";

PxArticulationAttachment_getConcreteTypeName :: (self: *PxArticulationAttachment) -> *u8 #foreign libphysx;

PxArticulationTendonJoint_setCoefficient :: (self: *PxArticulationTendonJoint, axis: s32, coefficient: float, recipCoefficient: float) -> void #foreign libphysx "PxArticulationTendonJoint_setCoefficient_mut";

PxArticulationTendonJoint_getCoefficient :: (self: *PxArticulationTendonJoint, axis: *s32, coefficient: *float, recipCoefficient: *float) -> void #foreign libphysx;

PxArticulationTendonJoint_getLink :: (self: *PxArticulationTendonJoint) -> *PxArticulationLink #foreign libphysx;

PxArticulationTendonJoint_getParent :: (self: *PxArticulationTendonJoint) -> *PxArticulationTendonJoint #foreign libphysx;

PxArticulationTendonJoint_getTendon :: (self: *PxArticulationTendonJoint) -> *PxArticulationFixedTendon #foreign libphysx;

PxArticulationTendonJoint_release :: (self: *PxArticulationTendonJoint) -> void #foreign libphysx "PxArticulationTendonJoint_release_mut";

PxArticulationTendonJoint_getConcreteTypeName :: (self: *PxArticulationTendonJoint) -> *u8 #foreign libphysx;

PxArticulationTendon_setStiffness :: (self: *PxArticulationTendon, stiffness: float) -> void #foreign libphysx "PxArticulationTendon_setStiffness_mut";

PxArticulationTendon_getStiffness :: (self: *PxArticulationTendon) -> float #foreign libphysx;

PxArticulationTendon_setDamping :: (self: *PxArticulationTendon, damping: float) -> void #foreign libphysx "PxArticulationTendon_setDamping_mut";

PxArticulationTendon_getDamping :: (self: *PxArticulationTendon) -> float #foreign libphysx;

PxArticulationTendon_setLimitStiffness :: (self: *PxArticulationTendon, stiffness: float) -> void #foreign libphysx "PxArticulationTendon_setLimitStiffness_mut";

PxArticulationTendon_getLimitStiffness :: (self: *PxArticulationTendon) -> float #foreign libphysx;

PxArticulationTendon_setOffset :: (self: *PxArticulationTendon, offset: float, autowake: bool) -> void #foreign libphysx "PxArticulationTendon_setOffset_mut";

PxArticulationTendon_getOffset :: (self: *PxArticulationTendon) -> float #foreign libphysx;

PxArticulationTendon_getArticulation :: (self: *PxArticulationTendon) -> *PxArticulationReducedCoordinate #foreign libphysx;

PxArticulationTendon_release :: (self: *PxArticulationTendon) -> void #foreign libphysx "PxArticulationTendon_release_mut";

PxArticulationSpatialTendon_createAttachment :: (self: *PxArticulationSpatialTendon, parent: *PxArticulationAttachment, coefficient: float, relativeOffset: Vector3, link: *PxArticulationLink) -> *PxArticulationAttachment #foreign libphysx "PxArticulationSpatialTendon_createAttachment_mut";

PxArticulationSpatialTendon_getAttachments :: (self: *PxArticulationSpatialTendon, userBuffer: **PxArticulationAttachment, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationSpatialTendon_getNbAttachments :: (self: *PxArticulationSpatialTendon) -> u32 #foreign libphysx;

PxArticulationSpatialTendon_getConcreteTypeName :: (self: *PxArticulationSpatialTendon) -> *u8 #foreign libphysx;

PxArticulationFixedTendon_createTendonJoint :: (self: *PxArticulationFixedTendon, parent: *PxArticulationTendonJoint, axis: s32, coefficient: float, recipCoefficient: float, link: *PxArticulationLink) -> *PxArticulationTendonJoint #foreign libphysx "PxArticulationFixedTendon_createTendonJoint_mut";

PxArticulationFixedTendon_getTendonJoints :: (self: *PxArticulationFixedTendon, userBuffer: **PxArticulationTendonJoint, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationFixedTendon_getNbTendonJoints :: (self: *PxArticulationFixedTendon) -> u32 #foreign libphysx;

PxArticulationFixedTendon_setRestLength :: (self: *PxArticulationFixedTendon, restLength: float) -> void #foreign libphysx "PxArticulationFixedTendon_setRestLength_mut";

PxArticulationFixedTendon_getRestLength :: (self: *PxArticulationFixedTendon) -> float #foreign libphysx;

PxArticulationFixedTendon_setLimitParameters :: (self: *PxArticulationFixedTendon, parameter: *PxArticulationTendonLimit) -> void #foreign libphysx "PxArticulationFixedTendon_setLimitParameters_mut";

PxArticulationFixedTendon_getLimitParameters :: (self: *PxArticulationFixedTendon) -> PxArticulationTendonLimit #foreign libphysx;

PxArticulationFixedTendon_getConcreteTypeName :: (self: *PxArticulationFixedTendon) -> *u8 #foreign libphysx;

PxArticulationCache_new :: () -> PxArticulationCache #foreign libphysx;

PxArticulationCache_release :: (self: *PxArticulationCache) -> void #foreign libphysx "PxArticulationCache_release_mut";

PxArticulationSensor_release :: (self: *PxArticulationSensor) -> void #foreign libphysx "PxArticulationSensor_release_mut";

PxArticulationSensor_getForces :: (self: *PxArticulationSensor) -> PxSpatialForce #foreign libphysx;

PxArticulationSensor_getRelativePose :: (self: *PxArticulationSensor) -> PxTransform #foreign libphysx;

PxArticulationSensor_setRelativePose :: (self: *PxArticulationSensor, pose: *PxTransform) -> void #foreign libphysx "PxArticulationSensor_setRelativePose_mut";

PxArticulationSensor_getLink :: (self: *PxArticulationSensor) -> *PxArticulationLink #foreign libphysx;

PxArticulationSensor_getIndex :: (self: *PxArticulationSensor) -> u32 #foreign libphysx;

PxArticulationSensor_getArticulation :: (self: *PxArticulationSensor) -> *PxArticulationReducedCoordinate #foreign libphysx;

PxArticulationSensor_getFlags :: (self: *PxArticulationSensor) -> u8 #foreign libphysx;

PxArticulationSensor_setFlag :: (self: *PxArticulationSensor, flag: s32, enabled: bool) -> void #foreign libphysx "PxArticulationSensor_setFlag_mut";

PxArticulationSensor_getConcreteTypeName :: (self: *PxArticulationSensor) -> *u8 #foreign libphysx;

PxArticulationReducedCoordinate_getScene :: (self: *PxArticulationReducedCoordinate) -> *PxScene #foreign libphysx;

PxArticulationReducedCoordinate_setSolverIterationCounts :: (self: *PxArticulationReducedCoordinate, minPositionIters: u32, minVelocityIters: u32) -> void #foreign libphysx "PxArticulationReducedCoordinate_setSolverIterationCounts_mut";

PxArticulationReducedCoordinate_getSolverIterationCounts :: (self: *PxArticulationReducedCoordinate, minPositionIters: *u32, minVelocityIters: *u32) -> void #foreign libphysx;

PxArticulationReducedCoordinate_isSleeping :: (self: *PxArticulationReducedCoordinate) -> bool #foreign libphysx;

PxArticulationReducedCoordinate_setSleepThreshold :: (self: *PxArticulationReducedCoordinate, threshold: float) -> void #foreign libphysx "PxArticulationReducedCoordinate_setSleepThreshold_mut";

PxArticulationReducedCoordinate_getSleepThreshold :: (self: *PxArticulationReducedCoordinate) -> float #foreign libphysx;

PxArticulationReducedCoordinate_setStabilizationThreshold :: (self: *PxArticulationReducedCoordinate, threshold: float) -> void #foreign libphysx "PxArticulationReducedCoordinate_setStabilizationThreshold_mut";

PxArticulationReducedCoordinate_getStabilizationThreshold :: (self: *PxArticulationReducedCoordinate) -> float #foreign libphysx;

PxArticulationReducedCoordinate_setWakeCounter :: (self: *PxArticulationReducedCoordinate, wakeCounterValue: float) -> void #foreign libphysx "PxArticulationReducedCoordinate_setWakeCounter_mut";

PxArticulationReducedCoordinate_getWakeCounter :: (self: *PxArticulationReducedCoordinate) -> float #foreign libphysx;

PxArticulationReducedCoordinate_wakeUp :: (self: *PxArticulationReducedCoordinate) -> void #foreign libphysx "PxArticulationReducedCoordinate_wakeUp_mut";

PxArticulationReducedCoordinate_putToSleep :: (self: *PxArticulationReducedCoordinate) -> void #foreign libphysx "PxArticulationReducedCoordinate_putToSleep_mut";

PxArticulationReducedCoordinate_setMaxCOMLinearVelocity :: (self: *PxArticulationReducedCoordinate, maxLinearVelocity: float) -> void #foreign libphysx "PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut";

PxArticulationReducedCoordinate_getMaxCOMLinearVelocity :: (self: *PxArticulationReducedCoordinate) -> float #foreign libphysx;

PxArticulationReducedCoordinate_setMaxCOMAngularVelocity :: (self: *PxArticulationReducedCoordinate, maxAngularVelocity: float) -> void #foreign libphysx "PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut";

PxArticulationReducedCoordinate_getMaxCOMAngularVelocity :: (self: *PxArticulationReducedCoordinate) -> float #foreign libphysx;

PxArticulationReducedCoordinate_createLink :: (self: *PxArticulationReducedCoordinate, parent: *PxArticulationLink, pose: *PxTransform) -> *PxArticulationLink #foreign libphysx "PxArticulationReducedCoordinate_createLink_mut";

PxArticulationReducedCoordinate_release :: (self: *PxArticulationReducedCoordinate) -> void #foreign libphysx "PxArticulationReducedCoordinate_release_mut";

PxArticulationReducedCoordinate_getNbLinks :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getLinks :: (self: *PxArticulationReducedCoordinate, userBuffer: **PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getNbShapes :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_setName :: (self: *PxArticulationReducedCoordinate, name: *u8) -> void #foreign libphysx "PxArticulationReducedCoordinate_setName_mut";

PxArticulationReducedCoordinate_getName :: (self: *PxArticulationReducedCoordinate) -> *u8 #foreign libphysx;

PxArticulationReducedCoordinate_getWorldBounds :: (self: *PxArticulationReducedCoordinate, inflation: float) -> PxBounds3 #foreign libphysx;

PxArticulationReducedCoordinate_getAggregate :: (self: *PxArticulationReducedCoordinate) -> *PxAggregate #foreign libphysx;

PxArticulationReducedCoordinate_setArticulationFlags :: (self: *PxArticulationReducedCoordinate, flags: u8) -> void #foreign libphysx "PxArticulationReducedCoordinate_setArticulationFlags_mut";

PxArticulationReducedCoordinate_setArticulationFlag :: (self: *PxArticulationReducedCoordinate, flag: s32, value: bool) -> void #foreign libphysx "PxArticulationReducedCoordinate_setArticulationFlag_mut";

PxArticulationReducedCoordinate_getArticulationFlags :: (self: *PxArticulationReducedCoordinate) -> u8 #foreign libphysx;

PxArticulationReducedCoordinate_getDofs :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_createCache :: (self: *PxArticulationReducedCoordinate) -> *PxArticulationCache #foreign libphysx;

PxArticulationReducedCoordinate_getCacheDataSize :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_zeroCache :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_applyCache :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache, flags: u32, autowake: bool) -> void #foreign libphysx "PxArticulationReducedCoordinate_applyCache_mut";

PxArticulationReducedCoordinate_copyInternalStateToCache :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache, flags: u32) -> void #foreign libphysx;

PxArticulationReducedCoordinate_packJointData :: (self: *PxArticulationReducedCoordinate, maximum: *float, reduced: *float) -> void #foreign libphysx;

PxArticulationReducedCoordinate_unpackJointData :: (self: *PxArticulationReducedCoordinate, reduced: *float, maximum: *float) -> void #foreign libphysx;

PxArticulationReducedCoordinate_commonInit :: (self: *PxArticulationReducedCoordinate) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeGeneralizedGravityForce :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeGeneralizedExternalForce :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeJointAcceleration :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeJointForce :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeDenseJacobian :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache, nRows: *u32, nCols: *u32) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeCoefficientMatrix :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_computeLambda :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache, initialState: *PxArticulationCache, jointTorque: *float, maxIter: u32) -> bool #foreign libphysx;

PxArticulationReducedCoordinate_computeGeneralizedMassMatrix :: (self: *PxArticulationReducedCoordinate, cache: *PxArticulationCache) -> void #foreign libphysx;

PxArticulationReducedCoordinate_addLoopJoint :: (self: *PxArticulationReducedCoordinate, joint: *PxConstraint) -> void #foreign libphysx "PxArticulationReducedCoordinate_addLoopJoint_mut";

PxArticulationReducedCoordinate_removeLoopJoint :: (self: *PxArticulationReducedCoordinate, joint: *PxConstraint) -> void #foreign libphysx "PxArticulationReducedCoordinate_removeLoopJoint_mut";

PxArticulationReducedCoordinate_getNbLoopJoints :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getLoopJoints :: (self: *PxArticulationReducedCoordinate, userBuffer: **PxConstraint, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getCoefficientMatrixSize :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_setRootGlobalPose :: (self: *PxArticulationReducedCoordinate, pose: *PxTransform, autowake: bool) -> void #foreign libphysx "PxArticulationReducedCoordinate_setRootGlobalPose_mut";

PxArticulationReducedCoordinate_getRootGlobalPose :: (self: *PxArticulationReducedCoordinate) -> PxTransform #foreign libphysx;

PxArticulationReducedCoordinate_setRootLinearVelocity :: (self: *PxArticulationReducedCoordinate, linearVelocity: *Vector3, autowake: bool) -> void #foreign libphysx "PxArticulationReducedCoordinate_setRootLinearVelocity_mut";

PxArticulationReducedCoordinate_getRootLinearVelocity :: (self: *PxArticulationReducedCoordinate) -> Vector3 #foreign libphysx;

PxArticulationReducedCoordinate_setRootAngularVelocity :: (self: *PxArticulationReducedCoordinate, angularVelocity: *Vector3, autowake: bool) -> void #foreign libphysx "PxArticulationReducedCoordinate_setRootAngularVelocity_mut";

PxArticulationReducedCoordinate_getRootAngularVelocity :: (self: *PxArticulationReducedCoordinate) -> Vector3 #foreign libphysx;

PxArticulationReducedCoordinate_getLinkAcceleration :: (self: *PxArticulationReducedCoordinate, linkId: u32) -> PxSpatialVelocity #foreign libphysx "PxArticulationReducedCoordinate_getLinkAcceleration_mut";

PxArticulationReducedCoordinate_getGpuArticulationIndex :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx "PxArticulationReducedCoordinate_getGpuArticulationIndex_mut";

PxArticulationReducedCoordinate_createSpatialTendon :: (self: *PxArticulationReducedCoordinate) -> *PxArticulationSpatialTendon #foreign libphysx "PxArticulationReducedCoordinate_createSpatialTendon_mut";

PxArticulationReducedCoordinate_createFixedTendon :: (self: *PxArticulationReducedCoordinate) -> *PxArticulationFixedTendon #foreign libphysx "PxArticulationReducedCoordinate_createFixedTendon_mut";

PxArticulationReducedCoordinate_createSensor :: (self: *PxArticulationReducedCoordinate, link: *PxArticulationLink, relativePose: *PxTransform) -> *PxArticulationSensor #foreign libphysx "PxArticulationReducedCoordinate_createSensor_mut";

PxArticulationReducedCoordinate_getSpatialTendons :: (self: *PxArticulationReducedCoordinate, userBuffer: **PxArticulationSpatialTendon, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getNbSpatialTendons :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx "PxArticulationReducedCoordinate_getNbSpatialTendons_mut";

PxArticulationReducedCoordinate_getFixedTendons :: (self: *PxArticulationReducedCoordinate, userBuffer: **PxArticulationFixedTendon, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getNbFixedTendons :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx "PxArticulationReducedCoordinate_getNbFixedTendons_mut";

PxArticulationReducedCoordinate_getSensors :: (self: *PxArticulationReducedCoordinate, userBuffer: **PxArticulationSensor, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationReducedCoordinate_getNbSensors :: (self: *PxArticulationReducedCoordinate) -> u32 #foreign libphysx "PxArticulationReducedCoordinate_getNbSensors_mut";

PxArticulationReducedCoordinate_updateKinematic :: (self: *PxArticulationReducedCoordinate, flags: u8) -> void #foreign libphysx "PxArticulationReducedCoordinate_updateKinematic_mut";

PxArticulationJointReducedCoordinate_getParentArticulationLink :: (self: *PxArticulationJointReducedCoordinate) -> *PxArticulationLink #foreign libphysx;

PxArticulationJointReducedCoordinate_setParentPose :: (self: *PxArticulationJointReducedCoordinate, pose: *PxTransform) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setParentPose_mut";

PxArticulationJointReducedCoordinate_getParentPose :: (self: *PxArticulationJointReducedCoordinate) -> PxTransform #foreign libphysx;

PxArticulationJointReducedCoordinate_getChildArticulationLink :: (self: *PxArticulationJointReducedCoordinate) -> *PxArticulationLink #foreign libphysx;

PxArticulationJointReducedCoordinate_setChildPose :: (self: *PxArticulationJointReducedCoordinate, pose: *PxTransform) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setChildPose_mut";

PxArticulationJointReducedCoordinate_getChildPose :: (self: *PxArticulationJointReducedCoordinate) -> PxTransform #foreign libphysx;

PxArticulationJointReducedCoordinate_setJointType :: (self: *PxArticulationJointReducedCoordinate, jointType: s32) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setJointType_mut";

PxArticulationJointReducedCoordinate_getJointType :: (self: *PxArticulationJointReducedCoordinate) -> s32 #foreign libphysx;

PxArticulationJointReducedCoordinate_setMotion :: (self: *PxArticulationJointReducedCoordinate, axis: s32, motion: s32) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setMotion_mut";

PxArticulationJointReducedCoordinate_getMotion :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> s32 #foreign libphysx;

PxArticulationJointReducedCoordinate_setLimitParams :: (self: *PxArticulationJointReducedCoordinate, axis: s32, limit: *PxArticulationLimit) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setLimitParams_mut";

PxArticulationJointReducedCoordinate_getLimitParams :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> PxArticulationLimit #foreign libphysx;

PxArticulationJointReducedCoordinate_setDriveParams :: (self: *PxArticulationJointReducedCoordinate, axis: s32, drive: *PxArticulationDrive) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setDriveParams_mut";

PxArticulationJointReducedCoordinate_getDriveParams :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> PxArticulationDrive #foreign libphysx;

PxArticulationJointReducedCoordinate_setDriveTarget :: (self: *PxArticulationJointReducedCoordinate, axis: s32, target: float, autowake: bool) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setDriveTarget_mut";

PxArticulationJointReducedCoordinate_getDriveTarget :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_setDriveVelocity :: (self: *PxArticulationJointReducedCoordinate, axis: s32, targetVel: float, autowake: bool) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setDriveVelocity_mut";

PxArticulationJointReducedCoordinate_getDriveVelocity :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_setArmature :: (self: *PxArticulationJointReducedCoordinate, axis: s32, armature: float) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setArmature_mut";

PxArticulationJointReducedCoordinate_getArmature :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_setFrictionCoefficient :: (self: *PxArticulationJointReducedCoordinate, coefficient: float) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut";

PxArticulationJointReducedCoordinate_getFrictionCoefficient :: (self: *PxArticulationJointReducedCoordinate) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_setMaxJointVelocity :: (self: *PxArticulationJointReducedCoordinate, maxJointV: float) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut";

PxArticulationJointReducedCoordinate_getMaxJointVelocity :: (self: *PxArticulationJointReducedCoordinate) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_setJointPosition :: (self: *PxArticulationJointReducedCoordinate, axis: s32, jointPos: float) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setJointPosition_mut";

PxArticulationJointReducedCoordinate_getJointPosition :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_setJointVelocity :: (self: *PxArticulationJointReducedCoordinate, axis: s32, jointVel: float) -> void #foreign libphysx "PxArticulationJointReducedCoordinate_setJointVelocity_mut";

PxArticulationJointReducedCoordinate_getJointVelocity :: (self: *PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign libphysx;

PxArticulationJointReducedCoordinate_getConcreteTypeName :: (self: *PxArticulationJointReducedCoordinate) -> *u8 #foreign libphysx;

PxShape_release :: (self: *PxShape) -> void #foreign libphysx "PxShape_release_mut";

PxShape_setGeometry :: (self: *PxShape, geometry: *PxGeometry) -> void #foreign libphysx "PxShape_setGeometry_mut";

PxShape_getGeometry :: (self: *PxShape) -> *PxGeometry #foreign libphysx;

PxShape_getActor :: (self: *PxShape) -> *PxRigidActor #foreign libphysx;

PxShape_setLocalPose :: (self: *PxShape, pose: *PxTransform) -> void #foreign libphysx "PxShape_setLocalPose_mut";

PxShape_getLocalPose :: (self: *PxShape) -> PxTransform #foreign libphysx;

PxShape_setSimulationFilterData :: (self: *PxShape, data: *PxFilterData) -> void #foreign libphysx "PxShape_setSimulationFilterData_mut";

PxShape_getSimulationFilterData :: (self: *PxShape) -> PxFilterData #foreign libphysx;

PxShape_setQueryFilterData :: (self: *PxShape, data: *PxFilterData) -> void #foreign libphysx "PxShape_setQueryFilterData_mut";

PxShape_getQueryFilterData :: (self: *PxShape) -> PxFilterData #foreign libphysx;

PxShape_setMaterials :: (self: *PxShape, materials: **PxMaterial, materialCount: u16) -> void #foreign libphysx "PxShape_setMaterials_mut";

PxShape_getNbMaterials :: (self: *PxShape) -> u16 #foreign libphysx;

PxShape_getMaterials :: (self: *PxShape, userBuffer: **PxMaterial, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxShape_getMaterialFromInternalFaceIndex :: (self: *PxShape, faceIndex: u32) -> *PxBaseMaterial #foreign libphysx;

PxShape_setContactOffset :: (self: *PxShape, contactOffset: float) -> void #foreign libphysx "PxShape_setContactOffset_mut";

PxShape_getContactOffset :: (self: *PxShape) -> float #foreign libphysx;

PxShape_setRestOffset :: (self: *PxShape, restOffset: float) -> void #foreign libphysx "PxShape_setRestOffset_mut";

PxShape_getRestOffset :: (self: *PxShape) -> float #foreign libphysx;

PxShape_setDensityForFluid :: (self: *PxShape, densityForFluid: float) -> void #foreign libphysx "PxShape_setDensityForFluid_mut";

PxShape_getDensityForFluid :: (self: *PxShape) -> float #foreign libphysx;

PxShape_setTorsionalPatchRadius :: (self: *PxShape, radius: float) -> void #foreign libphysx "PxShape_setTorsionalPatchRadius_mut";

PxShape_getTorsionalPatchRadius :: (self: *PxShape) -> float #foreign libphysx;

PxShape_setMinTorsionalPatchRadius :: (self: *PxShape, radius: float) -> void #foreign libphysx "PxShape_setMinTorsionalPatchRadius_mut";

PxShape_getMinTorsionalPatchRadius :: (self: *PxShape) -> float #foreign libphysx;

PxShape_setFlag :: (self: *PxShape, flag: s32, value: bool) -> void #foreign libphysx "PxShape_setFlag_mut";

PxShape_setFlags :: (self: *PxShape, inFlags: u8) -> void #foreign libphysx "PxShape_setFlags_mut";

PxShape_getFlags :: (self: *PxShape) -> u8 #foreign libphysx;

PxShape_isExclusive :: (self: *PxShape) -> bool #foreign libphysx;

PxShape_setName :: (self: *PxShape, name: *u8) -> void #foreign libphysx "PxShape_setName_mut";

PxShape_getName :: (self: *PxShape) -> *u8 #foreign libphysx;

PxShape_getConcreteTypeName :: (self: *PxShape) -> *u8 #foreign libphysx;

PxRigidActor_release :: (self: *PxRigidActor) -> void #foreign libphysx "PxRigidActor_release_mut";

PxRigidActor_getInternalActorIndex :: (self: *PxRigidActor) -> u32 #foreign libphysx;

PxRigidActor_getGlobalPose :: (self: *PxRigidActor) -> PxTransform #foreign libphysx;

PxRigidActor_setGlobalPose :: (self: *PxRigidActor, pose: *PxTransform, autowake: bool) -> void #foreign libphysx "PxRigidActor_setGlobalPose_mut";

PxRigidActor_attachShape :: (self: *PxRigidActor, shape: *PxShape) -> bool #foreign libphysx "PxRigidActor_attachShape_mut";

PxRigidActor_detachShape :: (self: *PxRigidActor, shape: *PxShape, wakeOnLostTouch: bool) -> void #foreign libphysx "PxRigidActor_detachShape_mut";

PxRigidActor_getNbShapes :: (self: *PxRigidActor) -> u32 #foreign libphysx;

PxRigidActor_getShapes :: (self: *PxRigidActor, userBuffer: **PxShape, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxRigidActor_getNbConstraints :: (self: *PxRigidActor) -> u32 #foreign libphysx;

PxRigidActor_getConstraints :: (self: *PxRigidActor, userBuffer: **PxConstraint, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxNodeIndex_new :: (id: u32, articLinkId: u32) -> PxNodeIndex #foreign libphysx;

PxNodeIndex_new :: (id: u32) -> PxNodeIndex #foreign libphysx "PxNodeIndex_new_1";

PxNodeIndex_index :: (self: *PxNodeIndex) -> u32 #foreign libphysx;

PxNodeIndex_articulationLinkId :: (self: *PxNodeIndex) -> u32 #foreign libphysx;

PxNodeIndex_isArticulation :: (self: *PxNodeIndex) -> u32 #foreign libphysx;

PxNodeIndex_isStaticBody :: (self: *PxNodeIndex) -> bool #foreign libphysx;

PxNodeIndex_isValid :: (self: *PxNodeIndex) -> bool #foreign libphysx;

PxNodeIndex_setIndices :: (self: *PxNodeIndex, index: u32, articLinkId: u32) -> void #foreign libphysx "PxNodeIndex_setIndices_mut";

PxNodeIndex_setIndices_mut :: (self: *PxNodeIndex, index: u32) -> void #foreign libphysx "PxNodeIndex_setIndices_mut_1";

PxNodeIndex_getInd :: (self: *PxNodeIndex) -> u64 #foreign libphysx;

PxRigidBody_setCMassLocalPose :: (self: *PxRigidBody, pose: *PxTransform) -> void #foreign libphysx "PxRigidBody_setCMassLocalPose_mut";

PxRigidBody_getCMassLocalPose :: (self: *PxRigidBody) -> PxTransform #foreign libphysx;

PxRigidBody_setMass :: (self: *PxRigidBody, mass: float) -> void #foreign libphysx "PxRigidBody_setMass_mut";

PxRigidBody_getMass :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_getInvMass :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_setMassSpaceInertiaTensor :: (self: *PxRigidBody, m: *Vector3) -> void #foreign libphysx "PxRigidBody_setMassSpaceInertiaTensor_mut";

PxRigidBody_getMassSpaceInertiaTensor :: (self: *PxRigidBody) -> Vector3 #foreign libphysx;

PxRigidBody_getMassSpaceInvInertiaTensor :: (self: *PxRigidBody) -> Vector3 #foreign libphysx;

PxRigidBody_setLinearDamping :: (self: *PxRigidBody, linDamp: float) -> void #foreign libphysx "PxRigidBody_setLinearDamping_mut";

PxRigidBody_getLinearDamping :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_setAngularDamping :: (self: *PxRigidBody, angDamp: float) -> void #foreign libphysx "PxRigidBody_setAngularDamping_mut";

PxRigidBody_getAngularDamping :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_getLinearVelocity :: (self: *PxRigidBody) -> Vector3 #foreign libphysx;

PxRigidBody_getAngularVelocity :: (self: *PxRigidBody) -> Vector3 #foreign libphysx;

PxRigidBody_setMaxLinearVelocity :: (self: *PxRigidBody, maxLinVel: float) -> void #foreign libphysx "PxRigidBody_setMaxLinearVelocity_mut";

PxRigidBody_getMaxLinearVelocity :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_setMaxAngularVelocity :: (self: *PxRigidBody, maxAngVel: float) -> void #foreign libphysx "PxRigidBody_setMaxAngularVelocity_mut";

PxRigidBody_getMaxAngularVelocity :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_addForce :: (self: *PxRigidBody, force: *Vector3, mode: s32, autowake: bool) -> void #foreign libphysx "PxRigidBody_addForce_mut";

PxRigidBody_addTorque :: (self: *PxRigidBody, torque: *Vector3, mode: s32, autowake: bool) -> void #foreign libphysx "PxRigidBody_addTorque_mut";

PxRigidBody_clearForce :: (self: *PxRigidBody, mode: s32) -> void #foreign libphysx "PxRigidBody_clearForce_mut";

PxRigidBody_clearTorque :: (self: *PxRigidBody, mode: s32) -> void #foreign libphysx "PxRigidBody_clearTorque_mut";

PxRigidBody_setForceAndTorque :: (self: *PxRigidBody, force: *Vector3, torque: *Vector3, mode: s32) -> void #foreign libphysx "PxRigidBody_setForceAndTorque_mut";

PxRigidBody_setRigidBodyFlag :: (self: *PxRigidBody, flag: s32, value: bool) -> void #foreign libphysx "PxRigidBody_setRigidBodyFlag_mut";

PxRigidBody_setRigidBodyFlags :: (self: *PxRigidBody, inFlags: u16) -> void #foreign libphysx "PxRigidBody_setRigidBodyFlags_mut";

PxRigidBody_getRigidBodyFlags :: (self: *PxRigidBody) -> u16 #foreign libphysx;

PxRigidBody_setMinCCDAdvanceCoefficient :: (self: *PxRigidBody, advanceCoefficient: float) -> void #foreign libphysx "PxRigidBody_setMinCCDAdvanceCoefficient_mut";

PxRigidBody_getMinCCDAdvanceCoefficient :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_setMaxDepenetrationVelocity :: (self: *PxRigidBody, biasClamp: float) -> void #foreign libphysx "PxRigidBody_setMaxDepenetrationVelocity_mut";

PxRigidBody_getMaxDepenetrationVelocity :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_setMaxContactImpulse :: (self: *PxRigidBody, maxImpulse: float) -> void #foreign libphysx "PxRigidBody_setMaxContactImpulse_mut";

PxRigidBody_getMaxContactImpulse :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_setContactSlopCoefficient :: (self: *PxRigidBody, slopCoefficient: float) -> void #foreign libphysx "PxRigidBody_setContactSlopCoefficient_mut";

PxRigidBody_getContactSlopCoefficient :: (self: *PxRigidBody) -> float #foreign libphysx;

PxRigidBody_getInternalIslandNodeIndex :: (self: *PxRigidBody) -> PxNodeIndex #foreign libphysx;

PxArticulationLink_release :: (self: *PxArticulationLink) -> void #foreign libphysx "PxArticulationLink_release_mut";

PxArticulationLink_getArticulation :: (self: *PxArticulationLink) -> *PxArticulationReducedCoordinate #foreign libphysx;

PxArticulationLink_getInboundJoint :: (self: *PxArticulationLink) -> *PxArticulationJointReducedCoordinate #foreign libphysx;

PxArticulationLink_getInboundJointDof :: (self: *PxArticulationLink) -> u32 #foreign libphysx;

PxArticulationLink_getNbChildren :: (self: *PxArticulationLink) -> u32 #foreign libphysx;

PxArticulationLink_getLinkIndex :: (self: *PxArticulationLink) -> u32 #foreign libphysx;

PxArticulationLink_getChildren :: (self: *PxArticulationLink, userBuffer: **PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxArticulationLink_setCfmScale :: (self: *PxArticulationLink, cfm: float) -> void #foreign libphysx "PxArticulationLink_setCfmScale_mut";

PxArticulationLink_getCfmScale :: (self: *PxArticulationLink) -> float #foreign libphysx;

PxArticulationLink_getLinearVelocity :: (self: *PxArticulationLink) -> Vector3 #foreign libphysx;

PxArticulationLink_getAngularVelocity :: (self: *PxArticulationLink) -> Vector3 #foreign libphysx;

PxArticulationLink_getConcreteTypeName :: (self: *PxArticulationLink) -> *u8 #foreign libphysx;

PxConeLimitedConstraint_new :: () -> PxConeLimitedConstraint #foreign libphysx;

PxConstraint_release :: (self: *PxConstraint) -> void #foreign libphysx "PxConstraint_release_mut";

PxConstraint_getScene :: (self: *PxConstraint) -> *PxScene #foreign libphysx;

PxConstraint_getActors :: (self: *PxConstraint, actor0: **PxRigidActor, actor1: **PxRigidActor) -> void #foreign libphysx;

PxConstraint_setActors :: (self: *PxConstraint, actor0: *PxRigidActor, actor1: *PxRigidActor) -> void #foreign libphysx "PxConstraint_setActors_mut";

PxConstraint_markDirty :: (self: *PxConstraint) -> void #foreign libphysx "PxConstraint_markDirty_mut";

PxConstraint_getFlags :: (self: *PxConstraint) -> u16 #foreign libphysx;

PxConstraint_setFlags :: (self: *PxConstraint, flags: u16) -> void #foreign libphysx "PxConstraint_setFlags_mut";

PxConstraint_setFlag :: (self: *PxConstraint, flag: s32, value: bool) -> void #foreign libphysx "PxConstraint_setFlag_mut";

PxConstraint_getForce :: (self: *PxConstraint, linear: *Vector3, angular: *Vector3) -> void #foreign libphysx;

PxConstraint_isValid :: (self: *PxConstraint) -> bool #foreign libphysx;

PxConstraint_setBreakForce :: (self: *PxConstraint, linear: float, angular: float) -> void #foreign libphysx "PxConstraint_setBreakForce_mut";

PxConstraint_getBreakForce :: (self: *PxConstraint, linear: *float, angular: *float) -> void #foreign libphysx;

PxConstraint_setMinResponseThreshold :: (self: *PxConstraint, threshold: float) -> void #foreign libphysx "PxConstraint_setMinResponseThreshold_mut";

PxConstraint_getMinResponseThreshold :: (self: *PxConstraint) -> float #foreign libphysx;

PxConstraint_getExternalReference :: (self: *PxConstraint, typeID: *u32) -> *void #foreign libphysx "PxConstraint_getExternalReference_mut";

PxConstraint_setConstraintFunctions :: (self: *PxConstraint, connector: *PxConstraintConnector, shaders: *PxConstraintShaderTable) -> void #foreign libphysx "PxConstraint_setConstraintFunctions_mut";

PxConstraint_getConcreteTypeName :: (self: *PxConstraint) -> *u8 #foreign libphysx;

PxContactStreamIterator_new :: (contactPatches: *u8, contactPoints: *u8, contactFaceIndices: *u32, nbPatches: u32, nbContacts: u32) -> PxContactStreamIterator #foreign libphysx;

PxContactStreamIterator_hasNextPatch :: (self: *PxContactStreamIterator) -> bool #foreign libphysx;

PxContactStreamIterator_getTotalContactCount :: (self: *PxContactStreamIterator) -> u32 #foreign libphysx;

PxContactStreamIterator_getTotalPatchCount :: (self: *PxContactStreamIterator) -> u32 #foreign libphysx;

PxContactStreamIterator_nextPatch :: (self: *PxContactStreamIterator) -> void #foreign libphysx "PxContactStreamIterator_nextPatch_mut";

PxContactStreamIterator_hasNextContact :: (self: *PxContactStreamIterator) -> bool #foreign libphysx;

PxContactStreamIterator_nextContact :: (self: *PxContactStreamIterator) -> void #foreign libphysx "PxContactStreamIterator_nextContact_mut";

PxContactStreamIterator_getContactNormal :: (self: *PxContactStreamIterator) -> *Vector3 #foreign libphysx;

PxContactStreamIterator_getInvMassScale0 :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getInvMassScale1 :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getInvInertiaScale0 :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getInvInertiaScale1 :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getMaxImpulse :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getTargetVel :: (self: *PxContactStreamIterator) -> *Vector3 #foreign libphysx;

PxContactStreamIterator_getContactPoint :: (self: *PxContactStreamIterator) -> *Vector3 #foreign libphysx;

PxContactStreamIterator_getSeparation :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getFaceIndex0 :: (self: *PxContactStreamIterator) -> u32 #foreign libphysx;

PxContactStreamIterator_getFaceIndex1 :: (self: *PxContactStreamIterator) -> u32 #foreign libphysx;

PxContactStreamIterator_getStaticFriction :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getDynamicFriction :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getRestitution :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getDamping :: (self: *PxContactStreamIterator) -> float #foreign libphysx;

PxContactStreamIterator_getMaterialFlags :: (self: *PxContactStreamIterator) -> u32 #foreign libphysx;

PxContactStreamIterator_getMaterialIndex0 :: (self: *PxContactStreamIterator) -> u16 #foreign libphysx;

PxContactStreamIterator_getMaterialIndex1 :: (self: *PxContactStreamIterator) -> u16 #foreign libphysx;

PxContactStreamIterator_advanceToIndex :: (self: *PxContactStreamIterator, initialIndex: u32) -> bool #foreign libphysx "PxContactStreamIterator_advanceToIndex_mut";

PxContactSet_getPoint :: (self: *PxContactSet, i: u32) -> *Vector3 #foreign libphysx;

PxContactSet_setPoint :: (self: *PxContactSet, i: u32, p: *Vector3) -> void #foreign libphysx "PxContactSet_setPoint_mut";

PxContactSet_getNormal :: (self: *PxContactSet, i: u32) -> *Vector3 #foreign libphysx;

PxContactSet_setNormal :: (self: *PxContactSet, i: u32, n: *Vector3) -> void #foreign libphysx "PxContactSet_setNormal_mut";

PxContactSet_getSeparation :: (self: *PxContactSet, i: u32) -> float #foreign libphysx;

PxContactSet_setSeparation :: (self: *PxContactSet, i: u32, s: float) -> void #foreign libphysx "PxContactSet_setSeparation_mut";

PxContactSet_getTargetVelocity :: (self: *PxContactSet, i: u32) -> *Vector3 #foreign libphysx;

PxContactSet_setTargetVelocity :: (self: *PxContactSet, i: u32, v: *Vector3) -> void #foreign libphysx "PxContactSet_setTargetVelocity_mut";

PxContactSet_getInternalFaceIndex0 :: (self: *PxContactSet, i: u32) -> u32 #foreign libphysx;

PxContactSet_getInternalFaceIndex1 :: (self: *PxContactSet, i: u32) -> u32 #foreign libphysx;

PxContactSet_getMaxImpulse :: (self: *PxContactSet, i: u32) -> float #foreign libphysx;

PxContactSet_setMaxImpulse :: (self: *PxContactSet, i: u32, s: float) -> void #foreign libphysx "PxContactSet_setMaxImpulse_mut";

PxContactSet_getRestitution :: (self: *PxContactSet, i: u32) -> float #foreign libphysx;

PxContactSet_setRestitution :: (self: *PxContactSet, i: u32, r: float) -> void #foreign libphysx "PxContactSet_setRestitution_mut";

PxContactSet_getStaticFriction :: (self: *PxContactSet, i: u32) -> float #foreign libphysx;

PxContactSet_setStaticFriction :: (self: *PxContactSet, i: u32, f: float) -> void #foreign libphysx "PxContactSet_setStaticFriction_mut";

PxContactSet_getDynamicFriction :: (self: *PxContactSet, i: u32) -> float #foreign libphysx;

PxContactSet_setDynamicFriction :: (self: *PxContactSet, i: u32, f: float) -> void #foreign libphysx "PxContactSet_setDynamicFriction_mut";

PxContactSet_ignore :: (self: *PxContactSet, i: u32) -> void #foreign libphysx "PxContactSet_ignore_mut";

PxContactSet_size :: (self: *PxContactSet) -> u32 #foreign libphysx;

PxContactSet_getInvMassScale0 :: (self: *PxContactSet) -> float #foreign libphysx;

PxContactSet_getInvMassScale1 :: (self: *PxContactSet) -> float #foreign libphysx;

PxContactSet_getInvInertiaScale0 :: (self: *PxContactSet) -> float #foreign libphysx;

PxContactSet_getInvInertiaScale1 :: (self: *PxContactSet) -> float #foreign libphysx;

PxContactSet_setInvMassScale0 :: (self: *PxContactSet, scale: float) -> void #foreign libphysx "PxContactSet_setInvMassScale0_mut";

PxContactSet_setInvMassScale1 :: (self: *PxContactSet, scale: float) -> void #foreign libphysx "PxContactSet_setInvMassScale1_mut";

PxContactSet_setInvInertiaScale0 :: (self: *PxContactSet, scale: float) -> void #foreign libphysx "PxContactSet_setInvInertiaScale0_mut";

PxContactSet_setInvInertiaScale1 :: (self: *PxContactSet, scale: float) -> void #foreign libphysx "PxContactSet_setInvInertiaScale1_mut";

PxContactModifyCallback_onContactModify :: (self: *PxContactModifyCallback, pairs: *PxContactModifyPair, count: u32) -> void #foreign libphysx "PxContactModifyCallback_onContactModify_mut";

PxCCDContactModifyCallback_onCCDContactModify :: (self: *PxCCDContactModifyCallback, pairs: *PxContactModifyPair, count: u32) -> void #foreign libphysx "PxCCDContactModifyCallback_onCCDContactModify_mut";

PxDeletionListener_onRelease :: (self: *PxDeletionListener, observed: *PxBase, userData: *void, deletionEvent: s32) -> void #foreign libphysx "PxDeletionListener_onRelease_mut";

PxBaseMaterial_isKindOf :: (self: *PxBaseMaterial, name: *u8) -> bool #foreign libphysx;

PxFEMMaterial_setYoungsModulus :: (self: *PxFEMMaterial, young: float) -> void #foreign libphysx "PxFEMMaterial_setYoungsModulus_mut";

PxFEMMaterial_getYoungsModulus :: (self: *PxFEMMaterial) -> float #foreign libphysx;

PxFEMMaterial_setPoissons :: (self: *PxFEMMaterial, poisson: float) -> void #foreign libphysx "PxFEMMaterial_setPoissons_mut";

PxFEMMaterial_getPoissons :: (self: *PxFEMMaterial) -> float #foreign libphysx;

PxFEMMaterial_setDynamicFriction :: (self: *PxFEMMaterial, dynamicFriction: float) -> void #foreign libphysx "PxFEMMaterial_setDynamicFriction_mut";

PxFEMMaterial_getDynamicFriction :: (self: *PxFEMMaterial) -> float #foreign libphysx;

PxFilterData_new :: (anon_param0: s32) -> PxFilterData #foreign libphysx;

PxFilterData_new :: () -> PxFilterData #foreign libphysx "PxFilterData_new_1";

PxFilterData_new :: (w0: u32, w1: u32, w2: u32, w3: u32) -> PxFilterData #foreign libphysx "PxFilterData_new_2";

PxFilterData_setToDefault :: (self: *PxFilterData) -> void #foreign libphysx "PxFilterData_setToDefault_mut";

PxGetFilterObjectType :: (attr: u32) -> s32 #foreign libphysx "phys_PxGetFilterObjectType";

PxFilterObjectIsKinematic :: (attr: u32) -> bool #foreign libphysx "phys_PxFilterObjectIsKinematic";

PxFilterObjectIsTrigger :: (attr: u32) -> bool #foreign libphysx "phys_PxFilterObjectIsTrigger";

PxSimulationFilterCallback_pairFound :: (self: *PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: PxFilterData, a0: *PxActor, s0: *PxShape, attributes1: u32, filterData1: PxFilterData, a1: *PxActor, s1: *PxShape, pairFlags: *u16) -> u16 #foreign libphysx "PxSimulationFilterCallback_pairFound_mut";

PxSimulationFilterCallback_pairLost :: (self: *PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: PxFilterData, attributes1: u32, filterData1: PxFilterData, objectRemoved: bool) -> void #foreign libphysx "PxSimulationFilterCallback_pairLost_mut";

PxSimulationFilterCallback_statusChange :: (self: *PxSimulationFilterCallback, pairID: *u32, pairFlags: *u16, filterFlags: *u16) -> bool #foreign libphysx "PxSimulationFilterCallback_statusChange_mut";

PxLockedData_getDataAccessFlags :: (self: *PxLockedData) -> u8 #foreign libphysx "PxLockedData_getDataAccessFlags_mut";

PxLockedData_unlock :: (self: *PxLockedData) -> void #foreign libphysx "PxLockedData_unlock_mut";

PxLockedData_delete :: (self: *PxLockedData) -> void #foreign libphysx;

PxMaterial_setDynamicFriction :: (self: *PxMaterial, coef: float) -> void #foreign libphysx "PxMaterial_setDynamicFriction_mut";

PxMaterial_getDynamicFriction :: (self: *PxMaterial) -> float #foreign libphysx;

PxMaterial_setStaticFriction :: (self: *PxMaterial, coef: float) -> void #foreign libphysx "PxMaterial_setStaticFriction_mut";

PxMaterial_getStaticFriction :: (self: *PxMaterial) -> float #foreign libphysx;

PxMaterial_setRestitution :: (self: *PxMaterial, rest: float) -> void #foreign libphysx "PxMaterial_setRestitution_mut";

PxMaterial_getRestitution :: (self: *PxMaterial) -> float #foreign libphysx;

PxMaterial_setDamping :: (self: *PxMaterial, damping: float) -> void #foreign libphysx "PxMaterial_setDamping_mut";

PxMaterial_getDamping :: (self: *PxMaterial) -> float #foreign libphysx;

PxMaterial_setFlag :: (self: *PxMaterial, flag: s32, b: bool) -> void #foreign libphysx "PxMaterial_setFlag_mut";

PxMaterial_setFlags :: (self: *PxMaterial, flags: u16) -> void #foreign libphysx "PxMaterial_setFlags_mut";

PxMaterial_getFlags :: (self: *PxMaterial) -> u16 #foreign libphysx;

PxMaterial_setFrictionCombineMode :: (self: *PxMaterial, combMode: s32) -> void #foreign libphysx "PxMaterial_setFrictionCombineMode_mut";

PxMaterial_getFrictionCombineMode :: (self: *PxMaterial) -> s32 #foreign libphysx;

PxMaterial_setRestitutionCombineMode :: (self: *PxMaterial, combMode: s32) -> void #foreign libphysx "PxMaterial_setRestitutionCombineMode_mut";

PxMaterial_getRestitutionCombineMode :: (self: *PxMaterial) -> s32 #foreign libphysx;

PxMaterial_getConcreteTypeName :: (self: *PxMaterial) -> *u8 #foreign libphysx;

PxDiffuseParticleParams_new :: () -> PxDiffuseParticleParams #foreign libphysx;

PxDiffuseParticleParams_setToDefault :: (self: *PxDiffuseParticleParams) -> void #foreign libphysx "PxDiffuseParticleParams_setToDefault_mut";

PxParticleMaterial_setFriction :: (self: *PxParticleMaterial, friction: float) -> void #foreign libphysx "PxParticleMaterial_setFriction_mut";

PxParticleMaterial_getFriction :: (self: *PxParticleMaterial) -> float #foreign libphysx;

PxParticleMaterial_setDamping :: (self: *PxParticleMaterial, damping: float) -> void #foreign libphysx "PxParticleMaterial_setDamping_mut";

PxParticleMaterial_getDamping :: (self: *PxParticleMaterial) -> float #foreign libphysx;

PxParticleMaterial_setAdhesion :: (self: *PxParticleMaterial, adhesion: float) -> void #foreign libphysx "PxParticleMaterial_setAdhesion_mut";

PxParticleMaterial_getAdhesion :: (self: *PxParticleMaterial) -> float #foreign libphysx;

PxParticleMaterial_setGravityScale :: (self: *PxParticleMaterial, scale: float) -> void #foreign libphysx "PxParticleMaterial_setGravityScale_mut";

PxParticleMaterial_getGravityScale :: (self: *PxParticleMaterial) -> float #foreign libphysx;

PxParticleMaterial_setAdhesionRadiusScale :: (self: *PxParticleMaterial, scale: float) -> void #foreign libphysx "PxParticleMaterial_setAdhesionRadiusScale_mut";

PxParticleMaterial_getAdhesionRadiusScale :: (self: *PxParticleMaterial) -> float #foreign libphysx;

PxPhysics_release :: (self: *PxPhysics) -> void #foreign libphysx "PxPhysics_release_mut";

PxPhysics_getFoundation :: (self: *PxPhysics) -> *PxFoundation #foreign libphysx "PxPhysics_getFoundation_mut";

PxPhysics_createAggregate :: (self: *PxPhysics, maxActor: u32, maxShape: u32, filterHint: u32) -> *PxAggregate #foreign libphysx "PxPhysics_createAggregate_mut";

PxPhysics_getTolerancesScale :: (self: *PxPhysics) -> *PxTolerancesScale #foreign libphysx;

PxPhysics_createTriangleMesh :: (self: *PxPhysics, stream: *PxInputStream) -> *PxTriangleMesh #foreign libphysx "PxPhysics_createTriangleMesh_mut";

PxPhysics_getNbTriangleMeshes :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getTriangleMeshes :: (self: *PxPhysics, userBuffer: **PxTriangleMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createTetrahedronMesh :: (self: *PxPhysics, stream: *PxInputStream) -> *PxTetrahedronMesh #foreign libphysx "PxPhysics_createTetrahedronMesh_mut";

PxPhysics_createSoftBodyMesh :: (self: *PxPhysics, stream: *PxInputStream) -> *PxSoftBodyMesh #foreign libphysx "PxPhysics_createSoftBodyMesh_mut";

PxPhysics_getNbTetrahedronMeshes :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getTetrahedronMeshes :: (self: *PxPhysics, userBuffer: **PxTetrahedronMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createHeightField :: (self: *PxPhysics, stream: *PxInputStream) -> *PxHeightField #foreign libphysx "PxPhysics_createHeightField_mut";

PxPhysics_getNbHeightFields :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getHeightFields :: (self: *PxPhysics, userBuffer: **PxHeightField, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createConvexMesh :: (self: *PxPhysics, stream: *PxInputStream) -> *PxConvexMesh #foreign libphysx "PxPhysics_createConvexMesh_mut";

PxPhysics_getNbConvexMeshes :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getConvexMeshes :: (self: *PxPhysics, userBuffer: **PxConvexMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createBVH :: (self: *PxPhysics, stream: *PxInputStream) -> *PxBVH #foreign libphysx "PxPhysics_createBVH_mut";

PxPhysics_getNbBVHs :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getBVHs :: (self: *PxPhysics, userBuffer: **PxBVH, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createScene :: (self: *PxPhysics, sceneDesc: *PxSceneDesc) -> *PxScene #foreign libphysx "PxPhysics_createScene_mut";

PxPhysics_getNbScenes :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getScenes :: (self: *PxPhysics, userBuffer: **PxScene, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createRigidStatic :: (self: *PxPhysics, pose: *PxTransform) -> *PxRigidStatic #foreign libphysx "PxPhysics_createRigidStatic_mut";

PxPhysics_createRigidDynamic :: (self: *PxPhysics, pose: *PxTransform) -> *PxRigidDynamic #foreign libphysx "PxPhysics_createRigidDynamic_mut";

PxPhysics_createPruningStructure :: (self: *PxPhysics, actors: **PxRigidActor, nbActors: u32) -> *PxPruningStructure #foreign libphysx "PxPhysics_createPruningStructure_mut";

PxPhysics_createShape :: (self: *PxPhysics, geometry: *PxGeometry, material: *PxMaterial, isExclusive: bool, shapeFlags: u8) -> *PxShape #foreign libphysx "PxPhysics_createShape_mut";

PxPhysics_createShape_mut :: (self: *PxPhysics, geometry: *PxGeometry, materials: **PxMaterial, materialCount: u16, isExclusive: bool, shapeFlags: u8) -> *PxShape #foreign libphysx "PxPhysics_createShape_mut_1";

PxPhysics_getNbShapes :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getShapes :: (self: *PxPhysics, userBuffer: **PxShape, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_createConstraint :: (self: *PxPhysics, actor0: *PxRigidActor, actor1: *PxRigidActor, connector: *PxConstraintConnector, shaders: *PxConstraintShaderTable, dataSize: u32) -> *PxConstraint #foreign libphysx "PxPhysics_createConstraint_mut";

PxPhysics_createArticulationReducedCoordinate :: (self: *PxPhysics) -> *PxArticulationReducedCoordinate #foreign libphysx "PxPhysics_createArticulationReducedCoordinate_mut";

PxPhysics_createMaterial :: (self: *PxPhysics, staticFriction: float, dynamicFriction: float, restitution: float) -> *PxMaterial #foreign libphysx "PxPhysics_createMaterial_mut";

PxPhysics_getNbMaterials :: (self: *PxPhysics) -> u32 #foreign libphysx;

PxPhysics_getMaterials :: (self: *PxPhysics, userBuffer: **PxMaterial, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPhysics_registerDeletionListener :: (self: *PxPhysics, observer: *PxDeletionListener, deletionEvents: *u8, restrictedObjectSet: bool) -> void #foreign libphysx "PxPhysics_registerDeletionListener_mut";

PxPhysics_unregisterDeletionListener :: (self: *PxPhysics, observer: *PxDeletionListener) -> void #foreign libphysx "PxPhysics_unregisterDeletionListener_mut";

PxPhysics_registerDeletionListenerObjects :: (self: *PxPhysics, observer: *PxDeletionListener, observables: **PxBase, observableCount: u32) -> void #foreign libphysx "PxPhysics_registerDeletionListenerObjects_mut";

PxPhysics_unregisterDeletionListenerObjects :: (self: *PxPhysics, observer: *PxDeletionListener, observables: **PxBase, observableCount: u32) -> void #foreign libphysx "PxPhysics_unregisterDeletionListenerObjects_mut";

PxPhysics_getPhysicsInsertionCallback :: (self: *PxPhysics) -> *PxInsertionCallback #foreign libphysx "PxPhysics_getPhysicsInsertionCallback_mut";

PxCreatePhysics :: (version: u32, foundation: *PxFoundation, scale: *PxTolerancesScale, trackOutstandingAllocations: bool, pvd: *PxPvd, omniPvd: *PxOmniPvd) -> *PxPhysics #foreign libphysx "phys_PxCreatePhysics";

PxGetPhysics :: () -> *PxPhysics #foreign libphysx "phys_PxGetPhysics";

PxActorShape_new :: () -> PxActorShape #foreign libphysx;

PxActorShape_new :: (a: *PxRigidActor, s: *PxShape) -> PxActorShape #foreign libphysx "PxActorShape_new_1";

PxQueryCache_new :: () -> PxQueryCache #foreign libphysx;

PxQueryCache_new :: (s: *PxShape, findex: u32) -> PxQueryCache #foreign libphysx "PxQueryCache_new_1";

PxQueryFilterData_new :: () -> PxQueryFilterData #foreign libphysx;

PxQueryFilterData_new :: (fd: *PxFilterData, f: u16) -> PxQueryFilterData #foreign libphysx "PxQueryFilterData_new_1";

PxQueryFilterData_new :: (f: u16) -> PxQueryFilterData #foreign libphysx "PxQueryFilterData_new_2";

PxQueryFilterCallback_preFilter :: (self: *PxQueryFilterCallback, filterData: *PxFilterData, shape: *PxShape, actor: *PxRigidActor, queryFlags: *u16) -> s32 #foreign libphysx "PxQueryFilterCallback_preFilter_mut";

PxQueryFilterCallback_postFilter :: (self: *PxQueryFilterCallback, filterData: *PxFilterData, hit: *PxQueryHit, shape: *PxShape, actor: *PxRigidActor) -> s32 #foreign libphysx "PxQueryFilterCallback_postFilter_mut";

PxQueryFilterCallback_delete :: (self: *PxQueryFilterCallback) -> void #foreign libphysx;

PxRigidDynamic_setKinematicTarget :: (self: *PxRigidDynamic, destination: *PxTransform) -> void #foreign libphysx "PxRigidDynamic_setKinematicTarget_mut";

PxRigidDynamic_getKinematicTarget :: (self: *PxRigidDynamic, target: *PxTransform) -> bool #foreign libphysx;

PxRigidDynamic_isSleeping :: (self: *PxRigidDynamic) -> bool #foreign libphysx;

PxRigidDynamic_setSleepThreshold :: (self: *PxRigidDynamic, threshold: float) -> void #foreign libphysx "PxRigidDynamic_setSleepThreshold_mut";

PxRigidDynamic_getSleepThreshold :: (self: *PxRigidDynamic) -> float #foreign libphysx;

PxRigidDynamic_setStabilizationThreshold :: (self: *PxRigidDynamic, threshold: float) -> void #foreign libphysx "PxRigidDynamic_setStabilizationThreshold_mut";

PxRigidDynamic_getStabilizationThreshold :: (self: *PxRigidDynamic) -> float #foreign libphysx;

PxRigidDynamic_getRigidDynamicLockFlags :: (self: *PxRigidDynamic) -> u8 #foreign libphysx;

PxRigidDynamic_setRigidDynamicLockFlag :: (self: *PxRigidDynamic, flag: s32, value: bool) -> void #foreign libphysx "PxRigidDynamic_setRigidDynamicLockFlag_mut";

PxRigidDynamic_setRigidDynamicLockFlags :: (self: *PxRigidDynamic, flags: u8) -> void #foreign libphysx "PxRigidDynamic_setRigidDynamicLockFlags_mut";

PxRigidDynamic_getLinearVelocity :: (self: *PxRigidDynamic) -> Vector3 #foreign libphysx;

PxRigidDynamic_setLinearVelocity :: (self: *PxRigidDynamic, linVel: *Vector3, autowake: bool) -> void #foreign libphysx "PxRigidDynamic_setLinearVelocity_mut";

PxRigidDynamic_getAngularVelocity :: (self: *PxRigidDynamic) -> Vector3 #foreign libphysx;

PxRigidDynamic_setAngularVelocity :: (self: *PxRigidDynamic, angVel: *Vector3, autowake: bool) -> void #foreign libphysx "PxRigidDynamic_setAngularVelocity_mut";

PxRigidDynamic_setWakeCounter :: (self: *PxRigidDynamic, wakeCounterValue: float) -> void #foreign libphysx "PxRigidDynamic_setWakeCounter_mut";

PxRigidDynamic_getWakeCounter :: (self: *PxRigidDynamic) -> float #foreign libphysx;

PxRigidDynamic_wakeUp :: (self: *PxRigidDynamic) -> void #foreign libphysx "PxRigidDynamic_wakeUp_mut";

PxRigidDynamic_putToSleep :: (self: *PxRigidDynamic) -> void #foreign libphysx "PxRigidDynamic_putToSleep_mut";

PxRigidDynamic_setSolverIterationCounts :: (self: *PxRigidDynamic, minPositionIters: u32, minVelocityIters: u32) -> void #foreign libphysx "PxRigidDynamic_setSolverIterationCounts_mut";

PxRigidDynamic_getSolverIterationCounts :: (self: *PxRigidDynamic, minPositionIters: *u32, minVelocityIters: *u32) -> void #foreign libphysx;

PxRigidDynamic_getContactReportThreshold :: (self: *PxRigidDynamic) -> float #foreign libphysx;

PxRigidDynamic_setContactReportThreshold :: (self: *PxRigidDynamic, threshold: float) -> void #foreign libphysx "PxRigidDynamic_setContactReportThreshold_mut";

PxRigidDynamic_getConcreteTypeName :: (self: *PxRigidDynamic) -> *u8 #foreign libphysx;

PxRigidStatic_getConcreteTypeName :: (self: *PxRigidStatic) -> *u8 #foreign libphysx;

PxSceneQueryDesc_new :: () -> PxSceneQueryDesc #foreign libphysx;

PxSceneQueryDesc_setToDefault :: (self: *PxSceneQueryDesc) -> void #foreign libphysx "PxSceneQueryDesc_setToDefault_mut";

PxSceneQueryDesc_isValid :: (self: *PxSceneQueryDesc) -> bool #foreign libphysx;

PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint :: (self: *PxSceneQuerySystemBase, dynamicTreeRebuildRateHint: u32) -> void #foreign libphysx "PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut";

PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint :: (self: *PxSceneQuerySystemBase) -> u32 #foreign libphysx;

PxSceneQuerySystemBase_forceRebuildDynamicTree :: (self: *PxSceneQuerySystemBase, prunerIndex: u32) -> void #foreign libphysx "PxSceneQuerySystemBase_forceRebuildDynamicTree_mut";

PxSceneQuerySystemBase_setUpdateMode :: (self: *PxSceneQuerySystemBase, updateMode: s32) -> void #foreign libphysx "PxSceneQuerySystemBase_setUpdateMode_mut";

PxSceneQuerySystemBase_getUpdateMode :: (self: *PxSceneQuerySystemBase) -> s32 #foreign libphysx;

PxSceneQuerySystemBase_getStaticTimestamp :: (self: *PxSceneQuerySystemBase) -> u32 #foreign libphysx;

PxSceneQuerySystemBase_flushUpdates :: (self: *PxSceneQuerySystemBase) -> void #foreign libphysx "PxSceneQuerySystemBase_flushUpdates_mut";

PxSceneQuerySystemBase_raycast :: (self: *PxSceneQuerySystemBase, origin: *Vector3, unitDir: *Vector3, distance: float, hitCall: *PxRaycastCallback, hitFlags: u16, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, queryFlags: u32) -> bool #foreign libphysx;

PxSceneQuerySystemBase_sweep :: (self: *PxSceneQuerySystemBase, geometry: *PxGeometry, pose: *PxTransform, unitDir: *Vector3, distance: float, hitCall: *PxSweepCallback, hitFlags: u16, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, inflation: float, queryFlags: u32) -> bool #foreign libphysx;

PxSceneQuerySystemBase_overlap :: (self: *PxSceneQuerySystemBase, geometry: *PxGeometry, pose: *PxTransform, hitCall: *PxOverlapCallback, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, queryFlags: u32) -> bool #foreign libphysx;

PxSceneSQSystem_setSceneQueryUpdateMode :: (self: *PxSceneSQSystem, updateMode: s32) -> void #foreign libphysx "PxSceneSQSystem_setSceneQueryUpdateMode_mut";

PxSceneSQSystem_getSceneQueryUpdateMode :: (self: *PxSceneSQSystem) -> s32 #foreign libphysx;

PxSceneSQSystem_getSceneQueryStaticTimestamp :: (self: *PxSceneSQSystem) -> u32 #foreign libphysx;

PxSceneSQSystem_flushQueryUpdates :: (self: *PxSceneSQSystem) -> void #foreign libphysx "PxSceneSQSystem_flushQueryUpdates_mut";

PxSceneSQSystem_forceDynamicTreeRebuild :: (self: *PxSceneSQSystem, rebuildStaticStructure: bool, rebuildDynamicStructure: bool) -> void #foreign libphysx "PxSceneSQSystem_forceDynamicTreeRebuild_mut";

PxSceneSQSystem_getStaticStructure :: (self: *PxSceneSQSystem) -> s32 #foreign libphysx;

PxSceneSQSystem_getDynamicStructure :: (self: *PxSceneSQSystem) -> s32 #foreign libphysx;

PxSceneSQSystem_sceneQueriesUpdate :: (self: *PxSceneSQSystem, completionTask: *PxBaseTask, controlSimulation: bool) -> void #foreign libphysx "PxSceneSQSystem_sceneQueriesUpdate_mut";

PxSceneSQSystem_checkQueries :: (self: *PxSceneSQSystem, block: bool) -> bool #foreign libphysx "PxSceneSQSystem_checkQueries_mut";

PxSceneSQSystem_fetchQueries :: (self: *PxSceneSQSystem, block: bool) -> bool #foreign libphysx "PxSceneSQSystem_fetchQueries_mut";

PxSceneQuerySystem_release :: (self: *PxSceneQuerySystem) -> void #foreign libphysx "PxSceneQuerySystem_release_mut";

PxSceneQuerySystem_acquireReference :: (self: *PxSceneQuerySystem) -> void #foreign libphysx "PxSceneQuerySystem_acquireReference_mut";

PxSceneQuerySystem_preallocate :: (self: *PxSceneQuerySystem, prunerIndex: u32, nbShapes: u32) -> void #foreign libphysx "PxSceneQuerySystem_preallocate_mut";

PxSceneQuerySystem_flushMemory :: (self: *PxSceneQuerySystem) -> void #foreign libphysx "PxSceneQuerySystem_flushMemory_mut";

PxSceneQuerySystem_addSQShape :: (self: *PxSceneQuerySystem, actor: *PxRigidActor, shape: *PxShape, bounds: *PxBounds3, transform: *PxTransform, compoundHandle: *u32, hasPruningStructure: bool) -> void #foreign libphysx "PxSceneQuerySystem_addSQShape_mut";

PxSceneQuerySystem_removeSQShape :: (self: *PxSceneQuerySystem, actor: *PxRigidActor, shape: *PxShape) -> void #foreign libphysx "PxSceneQuerySystem_removeSQShape_mut";

PxSceneQuerySystem_updateSQShape :: (self: *PxSceneQuerySystem, actor: *PxRigidActor, shape: *PxShape, transform: *PxTransform) -> void #foreign libphysx "PxSceneQuerySystem_updateSQShape_mut";

PxSceneQuerySystem_addSQCompound :: (self: *PxSceneQuerySystem, actor: *PxRigidActor, shapes: **PxShape, bvh: *PxBVH, transforms: *PxTransform) -> u32 #foreign libphysx "PxSceneQuerySystem_addSQCompound_mut";

PxSceneQuerySystem_removeSQCompound :: (self: *PxSceneQuerySystem, compoundHandle: u32) -> void #foreign libphysx "PxSceneQuerySystem_removeSQCompound_mut";

PxSceneQuerySystem_updateSQCompound :: (self: *PxSceneQuerySystem, compoundHandle: u32, compoundTransform: *PxTransform) -> void #foreign libphysx "PxSceneQuerySystem_updateSQCompound_mut";

PxSceneQuerySystem_shiftOrigin :: (self: *PxSceneQuerySystem, shift: *Vector3) -> void #foreign libphysx "PxSceneQuerySystem_shiftOrigin_mut";

PxSceneQuerySystem_merge :: (self: *PxSceneQuerySystem, pruningStructure: *PxPruningStructure) -> void #foreign libphysx "PxSceneQuerySystem_merge_mut";

PxSceneQuerySystem_getHandle :: (self: *PxSceneQuerySystem, actor: *PxRigidActor, shape: *PxShape, prunerIndex: *u32) -> u32 #foreign libphysx;

PxSceneQuerySystem_sync :: (self: *PxSceneQuerySystem, prunerIndex: u32, handles: *u32, indices: *u32, bounds: *PxBounds3, transforms: *PxTransformPadded, count: u32, ignoredIndices: *PxBitMap) -> void #foreign libphysx "PxSceneQuerySystem_sync_mut";

PxSceneQuerySystem_finalizeUpdates :: (self: *PxSceneQuerySystem) -> void #foreign libphysx "PxSceneQuerySystem_finalizeUpdates_mut";

PxSceneQuerySystem_prepareSceneQueryBuildStep :: (self: *PxSceneQuerySystem, prunerIndex: u32) -> *void #foreign libphysx "PxSceneQuerySystem_prepareSceneQueryBuildStep_mut";

PxSceneQuerySystem_sceneQueryBuildStep :: (self: *PxSceneQuerySystem, handle: *void) -> void #foreign libphysx "PxSceneQuerySystem_sceneQueryBuildStep_mut";

PxBroadPhaseDesc_new :: (type: s32) -> PxBroadPhaseDesc #foreign libphysx;

PxBroadPhaseDesc_isValid :: (self: *PxBroadPhaseDesc) -> bool #foreign libphysx;

PxGetBroadPhaseStaticFilterGroup :: () -> u32 #foreign libphysx "phys_PxGetBroadPhaseStaticFilterGroup";

PxGetBroadPhaseDynamicFilterGroup :: (id: u32) -> u32 #foreign libphysx "phys_PxGetBroadPhaseDynamicFilterGroup";

PxGetBroadPhaseKinematicFilterGroup :: (id: u32) -> u32 #foreign libphysx "phys_PxGetBroadPhaseKinematicFilterGroup";

PxBroadPhaseUpdateData_new :: (created: *u32, nbCreated: u32, updated: *u32, nbUpdated: u32, removed: *u32, nbRemoved: u32, bounds: *PxBounds3, groups: *u32, distances: *float, capacity: u32) -> PxBroadPhaseUpdateData #foreign libphysx;

PxBroadPhaseResults_new :: () -> PxBroadPhaseResults #foreign libphysx;

PxBroadPhaseRegions_getNbRegions :: (self: *PxBroadPhaseRegions) -> u32 #foreign libphysx;

PxBroadPhaseRegions_getRegions :: (self: *PxBroadPhaseRegions, userBuffer: *PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxBroadPhaseRegions_addRegion :: (self: *PxBroadPhaseRegions, region: *PxBroadPhaseRegion, populateRegion: bool, bounds: *PxBounds3, distances: *float) -> u32 #foreign libphysx "PxBroadPhaseRegions_addRegion_mut";

PxBroadPhaseRegions_removeRegion :: (self: *PxBroadPhaseRegions, handle: u32) -> bool #foreign libphysx "PxBroadPhaseRegions_removeRegion_mut";

PxBroadPhaseRegions_getNbOutOfBoundsObjects :: (self: *PxBroadPhaseRegions) -> u32 #foreign libphysx;

PxBroadPhaseRegions_getOutOfBoundsObjects :: (self: *PxBroadPhaseRegions) -> *u32 #foreign libphysx;

PxBroadPhase_release :: (self: *PxBroadPhase) -> void #foreign libphysx "PxBroadPhase_release_mut";

PxBroadPhase_getType :: (self: *PxBroadPhase) -> s32 #foreign libphysx;

PxBroadPhase_getCaps :: (self: *PxBroadPhase, caps: *PxBroadPhaseCaps) -> void #foreign libphysx;

PxBroadPhase_getRegions :: (self: *PxBroadPhase) -> *PxBroadPhaseRegions #foreign libphysx "PxBroadPhase_getRegions_mut";

PxBroadPhase_getAllocator :: (self: *PxBroadPhase) -> *PxAllocatorCallback #foreign libphysx "PxBroadPhase_getAllocator_mut";

PxBroadPhase_getContextID :: (self: *PxBroadPhase) -> u64 #foreign libphysx;

PxBroadPhase_setScratchBlock :: (self: *PxBroadPhase, scratchBlock: *void, size: u32) -> void #foreign libphysx "PxBroadPhase_setScratchBlock_mut";

PxBroadPhase_update :: (self: *PxBroadPhase, updateData: *PxBroadPhaseUpdateData, continuation: *PxBaseTask) -> void #foreign libphysx "PxBroadPhase_update_mut";

PxBroadPhase_fetchResults :: (self: *PxBroadPhase, results: *PxBroadPhaseResults) -> void #foreign libphysx "PxBroadPhase_fetchResults_mut";

PxBroadPhase_update_mut :: (self: *PxBroadPhase, results: *PxBroadPhaseResults, updateData: *PxBroadPhaseUpdateData) -> void #foreign libphysx "PxBroadPhase_update_mut_1";

PxCreateBroadPhase :: (desc: *PxBroadPhaseDesc) -> *PxBroadPhase #foreign libphysx "phys_PxCreateBroadPhase";

PxAABBManager_release :: (self: *PxAABBManager) -> void #foreign libphysx "PxAABBManager_release_mut";

PxAABBManager_getBroadPhase :: (self: *PxAABBManager) -> *PxBroadPhase #foreign libphysx "PxAABBManager_getBroadPhase_mut";

PxAABBManager_getBounds :: (self: *PxAABBManager) -> *PxBounds3 #foreign libphysx;

PxAABBManager_getDistances :: (self: *PxAABBManager) -> *float #foreign libphysx;

PxAABBManager_getGroups :: (self: *PxAABBManager) -> *u32 #foreign libphysx;

PxAABBManager_getCapacity :: (self: *PxAABBManager) -> u32 #foreign libphysx;

PxAABBManager_addObject :: (self: *PxAABBManager, index: u32, bounds: *PxBounds3, group: u32, distance: float) -> void #foreign libphysx "PxAABBManager_addObject_mut";

PxAABBManager_removeObject :: (self: *PxAABBManager, index: u32) -> void #foreign libphysx "PxAABBManager_removeObject_mut";

PxAABBManager_updateObject :: (self: *PxAABBManager, index: u32, bounds: *PxBounds3, distance: *float) -> void #foreign libphysx "PxAABBManager_updateObject_mut";

PxAABBManager_update :: (self: *PxAABBManager, continuation: *PxBaseTask) -> void #foreign libphysx "PxAABBManager_update_mut";

PxAABBManager_fetchResults :: (self: *PxAABBManager, results: *PxBroadPhaseResults) -> void #foreign libphysx "PxAABBManager_fetchResults_mut";

PxAABBManager_update_mut :: (self: *PxAABBManager, results: *PxBroadPhaseResults) -> void #foreign libphysx "PxAABBManager_update_mut_1";

PxCreateAABBManager :: (broadphase: *PxBroadPhase) -> *PxAABBManager #foreign libphysx "phys_PxCreateAABBManager";

PxSceneLimits_new :: () -> PxSceneLimits #foreign libphysx;

PxSceneLimits_setToDefault :: (self: *PxSceneLimits) -> void #foreign libphysx "PxSceneLimits_setToDefault_mut";

PxSceneLimits_isValid :: (self: *PxSceneLimits) -> bool #foreign libphysx;

PxgDynamicsMemoryConfig_new :: () -> PxgDynamicsMemoryConfig #foreign libphysx;

PxgDynamicsMemoryConfig_isValid :: (self: *PxgDynamicsMemoryConfig) -> bool #foreign libphysx;

PxSceneDesc_new :: (scale: *PxTolerancesScale) -> PxSceneDesc #foreign libphysx;

PxSceneDesc_setToDefault :: (self: *PxSceneDesc, scale: *PxTolerancesScale) -> void #foreign libphysx "PxSceneDesc_setToDefault_mut";

PxSceneDesc_isValid :: (self: *PxSceneDesc) -> bool #foreign libphysx;

PxSceneDesc_getTolerancesScale :: (self: *PxSceneDesc) -> *PxTolerancesScale #foreign libphysx;

PxSimulationStatistics_getNbBroadPhaseAdds :: (self: *PxSimulationStatistics) -> u32 #foreign libphysx;

PxSimulationStatistics_getNbBroadPhaseRemoves :: (self: *PxSimulationStatistics) -> u32 #foreign libphysx;

PxSimulationStatistics_getRbPairStats :: (self: *PxSimulationStatistics, pairType: s32, g0: s32, g1: s32) -> u32 #foreign libphysx;

PxSimulationStatistics_new :: () -> PxSimulationStatistics #foreign libphysx;

PxPvdSceneClient_setScenePvdFlag :: (self: *PxPvdSceneClient, flag: s32, value: bool) -> void #foreign libphysx "PxPvdSceneClient_setScenePvdFlag_mut";

PxPvdSceneClient_setScenePvdFlags :: (self: *PxPvdSceneClient, flags: u8) -> void #foreign libphysx "PxPvdSceneClient_setScenePvdFlags_mut";

PxPvdSceneClient_getScenePvdFlags :: (self: *PxPvdSceneClient) -> u8 #foreign libphysx;

PxPvdSceneClient_updateCamera :: (self: *PxPvdSceneClient, name: *u8, origin: *Vector3, up: *Vector3, target: *Vector3) -> void #foreign libphysx "PxPvdSceneClient_updateCamera_mut";

PxPvdSceneClient_drawPoints :: (self: *PxPvdSceneClient, points: *PxDebugPoint, count: u32) -> void #foreign libphysx "PxPvdSceneClient_drawPoints_mut";

PxPvdSceneClient_drawLines :: (self: *PxPvdSceneClient, lines: *PxDebugLine, count: u32) -> void #foreign libphysx "PxPvdSceneClient_drawLines_mut";

PxPvdSceneClient_drawTriangles :: (self: *PxPvdSceneClient, triangles: *PxDebugTriangle, count: u32) -> void #foreign libphysx "PxPvdSceneClient_drawTriangles_mut";

PxPvdSceneClient_drawText :: (self: *PxPvdSceneClient, text: *PxDebugText) -> void #foreign libphysx "PxPvdSceneClient_drawText_mut";

PxDominanceGroupPair_new :: (a: u8, b: u8) -> PxDominanceGroupPair #foreign libphysx;

PxBroadPhaseCallback_delete :: (self: *PxBroadPhaseCallback) -> void #foreign libphysx;

PxBroadPhaseCallback_onObjectOutOfBounds :: (self: *PxBroadPhaseCallback, shape: *PxShape, actor: *PxActor) -> void #foreign libphysx "PxBroadPhaseCallback_onObjectOutOfBounds_mut";

PxBroadPhaseCallback_onObjectOutOfBounds_mut :: (self: *PxBroadPhaseCallback, aggregate: *PxAggregate) -> void #foreign libphysx "PxBroadPhaseCallback_onObjectOutOfBounds_mut_1";

PxScene_release :: (self: *PxScene) -> void #foreign libphysx "PxScene_release_mut";

PxScene_setFlag :: (self: *PxScene, flag: s32, value: bool) -> void #foreign libphysx "PxScene_setFlag_mut";

PxScene_getFlags :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_setLimits :: (self: *PxScene, limits: *PxSceneLimits) -> void #foreign libphysx "PxScene_setLimits_mut";

PxScene_getLimits :: (self: *PxScene) -> PxSceneLimits #foreign libphysx;

PxScene_getPhysics :: (self: *PxScene) -> *PxPhysics #foreign libphysx "PxScene_getPhysics_mut";

PxScene_getTimestamp :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_addArticulation :: (self: *PxScene, articulation: *PxArticulationReducedCoordinate) -> bool #foreign libphysx "PxScene_addArticulation_mut";

PxScene_removeArticulation :: (self: *PxScene, articulation: *PxArticulationReducedCoordinate, wakeOnLostTouch: bool) -> void #foreign libphysx "PxScene_removeArticulation_mut";

PxScene_addActor :: (self: *PxScene, actor: *PxActor, bvh: *PxBVH) -> bool #foreign libphysx "PxScene_addActor_mut";

PxScene_addActors :: (self: *PxScene, actors: **PxActor, nbActors: u32) -> bool #foreign libphysx "PxScene_addActors_mut";

PxScene_addActors_mut :: (self: *PxScene, pruningStructure: *PxPruningStructure) -> bool #foreign libphysx "PxScene_addActors_mut_1";

PxScene_removeActor :: (self: *PxScene, actor: *PxActor, wakeOnLostTouch: bool) -> void #foreign libphysx "PxScene_removeActor_mut";

PxScene_removeActors :: (self: *PxScene, actors: **PxActor, nbActors: u32, wakeOnLostTouch: bool) -> void #foreign libphysx "PxScene_removeActors_mut";

PxScene_addAggregate :: (self: *PxScene, aggregate: *PxAggregate) -> bool #foreign libphysx "PxScene_addAggregate_mut";

PxScene_removeAggregate :: (self: *PxScene, aggregate: *PxAggregate, wakeOnLostTouch: bool) -> void #foreign libphysx "PxScene_removeAggregate_mut";

PxScene_addCollection :: (self: *PxScene, collection: *PxCollection) -> bool #foreign libphysx "PxScene_addCollection_mut";

PxScene_getNbActors :: (self: *PxScene, types: u16) -> u32 #foreign libphysx;

PxScene_getActors :: (self: *PxScene, types: u16, userBuffer: **PxActor, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxScene_getActiveActors :: (self: *PxScene, nbActorsOut: *u32) -> **PxActor #foreign libphysx "PxScene_getActiveActors_mut";

PxScene_getNbArticulations :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getArticulations :: (self: *PxScene, userBuffer: **PxArticulationReducedCoordinate, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxScene_getNbConstraints :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getConstraints :: (self: *PxScene, userBuffer: **PxConstraint, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxScene_getNbAggregates :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getAggregates :: (self: *PxScene, userBuffer: **PxAggregate, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxScene_setDominanceGroupPair :: (self: *PxScene, group1: u8, group2: u8, dominance: *PxDominanceGroupPair) -> void #foreign libphysx "PxScene_setDominanceGroupPair_mut";

PxScene_getDominanceGroupPair :: (self: *PxScene, group1: u8, group2: u8) -> PxDominanceGroupPair #foreign libphysx;

PxScene_getCpuDispatcher :: (self: *PxScene) -> *PxCpuDispatcher #foreign libphysx;

PxScene_createClient :: (self: *PxScene) -> u8 #foreign libphysx "PxScene_createClient_mut";

PxScene_setSimulationEventCallback :: (self: *PxScene, callback: *PxSimulationEventCallback) -> void #foreign libphysx "PxScene_setSimulationEventCallback_mut";

PxScene_getSimulationEventCallback :: (self: *PxScene) -> *PxSimulationEventCallback #foreign libphysx;

PxScene_setContactModifyCallback :: (self: *PxScene, callback: *PxContactModifyCallback) -> void #foreign libphysx "PxScene_setContactModifyCallback_mut";

PxScene_setCCDContactModifyCallback :: (self: *PxScene, callback: *PxCCDContactModifyCallback) -> void #foreign libphysx "PxScene_setCCDContactModifyCallback_mut";

PxScene_getContactModifyCallback :: (self: *PxScene) -> *PxContactModifyCallback #foreign libphysx;

PxScene_getCCDContactModifyCallback :: (self: *PxScene) -> *PxCCDContactModifyCallback #foreign libphysx;

PxScene_setBroadPhaseCallback :: (self: *PxScene, callback: *PxBroadPhaseCallback) -> void #foreign libphysx "PxScene_setBroadPhaseCallback_mut";

PxScene_getBroadPhaseCallback :: (self: *PxScene) -> *PxBroadPhaseCallback #foreign libphysx;

PxScene_setFilterShaderData :: (self: *PxScene, data: *void, dataSize: u32) -> void #foreign libphysx "PxScene_setFilterShaderData_mut";

PxScene_getFilterShaderData :: (self: *PxScene) -> *void #foreign libphysx;

PxScene_getFilterShaderDataSize :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_resetFiltering :: (self: *PxScene, actor: *PxActor) -> bool #foreign libphysx "PxScene_resetFiltering_mut";

PxScene_resetFiltering_mut :: (self: *PxScene, actor: *PxRigidActor, shapes: **PxShape, shapeCount: u32) -> bool #foreign libphysx "PxScene_resetFiltering_mut_1";

PxScene_getKinematicKinematicFilteringMode :: (self: *PxScene) -> s32 #foreign libphysx;

PxScene_getStaticKinematicFilteringMode :: (self: *PxScene) -> s32 #foreign libphysx;

PxScene_simulate :: (self: *PxScene, elapsedTime: float, completionTask: *PxBaseTask, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign libphysx "PxScene_simulate_mut";

PxScene_advance :: (self: *PxScene, completionTask: *PxBaseTask) -> bool #foreign libphysx "PxScene_advance_mut";

PxScene_collide :: (self: *PxScene, elapsedTime: float, completionTask: *PxBaseTask, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign libphysx "PxScene_collide_mut";

PxScene_checkResults :: (self: *PxScene, block: bool) -> bool #foreign libphysx "PxScene_checkResults_mut";

PxScene_fetchCollision :: (self: *PxScene, block: bool) -> bool #foreign libphysx "PxScene_fetchCollision_mut";

PxScene_fetchResults :: (self: *PxScene, block: bool, errorState: *u32) -> bool #foreign libphysx "PxScene_fetchResults_mut";

PxScene_fetchResultsStart :: (self: *PxScene, contactPairs: **PxContactPairHeader, nbContactPairs: *u32, block: bool) -> bool #foreign libphysx "PxScene_fetchResultsStart_mut";

PxScene_processCallbacks :: (self: *PxScene, continuation: *PxBaseTask) -> void #foreign libphysx "PxScene_processCallbacks_mut";

PxScene_fetchResultsFinish :: (self: *PxScene, errorState: *u32) -> void #foreign libphysx "PxScene_fetchResultsFinish_mut";

PxScene_fetchResultsParticleSystem :: (self: *PxScene) -> void #foreign libphysx "PxScene_fetchResultsParticleSystem_mut";

PxScene_flushSimulation :: (self: *PxScene, sendPendingReports: bool) -> void #foreign libphysx "PxScene_flushSimulation_mut";

PxScene_setGravity :: (self: *PxScene, vec: *Vector3) -> void #foreign libphysx "PxScene_setGravity_mut";

PxScene_getGravity :: (self: *PxScene) -> Vector3 #foreign libphysx;

PxScene_setBounceThresholdVelocity :: (self: *PxScene, t: float) -> void #foreign libphysx "PxScene_setBounceThresholdVelocity_mut";

PxScene_getBounceThresholdVelocity :: (self: *PxScene) -> float #foreign libphysx;

PxScene_setCCDMaxPasses :: (self: *PxScene, ccdMaxPasses: u32) -> void #foreign libphysx "PxScene_setCCDMaxPasses_mut";

PxScene_getCCDMaxPasses :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_setCCDMaxSeparation :: (self: *PxScene, t: float) -> void #foreign libphysx "PxScene_setCCDMaxSeparation_mut";

PxScene_getCCDMaxSeparation :: (self: *PxScene) -> float #foreign libphysx;

PxScene_setCCDThreshold :: (self: *PxScene, t: float) -> void #foreign libphysx "PxScene_setCCDThreshold_mut";

PxScene_getCCDThreshold :: (self: *PxScene) -> float #foreign libphysx;

PxScene_setMaxBiasCoefficient :: (self: *PxScene, t: float) -> void #foreign libphysx "PxScene_setMaxBiasCoefficient_mut";

PxScene_getMaxBiasCoefficient :: (self: *PxScene) -> float #foreign libphysx;

PxScene_setFrictionOffsetThreshold :: (self: *PxScene, t: float) -> void #foreign libphysx "PxScene_setFrictionOffsetThreshold_mut";

PxScene_getFrictionOffsetThreshold :: (self: *PxScene) -> float #foreign libphysx;

PxScene_setFrictionCorrelationDistance :: (self: *PxScene, t: float) -> void #foreign libphysx "PxScene_setFrictionCorrelationDistance_mut";

PxScene_getFrictionCorrelationDistance :: (self: *PxScene) -> float #foreign libphysx;

PxScene_getFrictionType :: (self: *PxScene) -> s32 #foreign libphysx;

PxScene_getSolverType :: (self: *PxScene) -> s32 #foreign libphysx;

PxScene_setVisualizationParameter :: (self: *PxScene, param: s32, value: float) -> bool #foreign libphysx "PxScene_setVisualizationParameter_mut";

PxScene_getVisualizationParameter :: (self: *PxScene, paramEnum: s32) -> float #foreign libphysx;

PxScene_setVisualizationCullingBox :: (self: *PxScene, box: *PxBounds3) -> void #foreign libphysx "PxScene_setVisualizationCullingBox_mut";

PxScene_getVisualizationCullingBox :: (self: *PxScene) -> PxBounds3 #foreign libphysx;

PxScene_getRenderBuffer :: (self: *PxScene) -> *PxRenderBuffer #foreign libphysx "PxScene_getRenderBuffer_mut";

PxScene_getSimulationStatistics :: (self: *PxScene, stats: *PxSimulationStatistics) -> void #foreign libphysx;

PxScene_getBroadPhaseType :: (self: *PxScene) -> s32 #foreign libphysx;

PxScene_getBroadPhaseCaps :: (self: *PxScene, caps: *PxBroadPhaseCaps) -> bool #foreign libphysx;

PxScene_getNbBroadPhaseRegions :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getBroadPhaseRegions :: (self: *PxScene, userBuffer: *PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxScene_addBroadPhaseRegion :: (self: *PxScene, region: *PxBroadPhaseRegion, populateRegion: bool) -> u32 #foreign libphysx "PxScene_addBroadPhaseRegion_mut";

PxScene_removeBroadPhaseRegion :: (self: *PxScene, handle: u32) -> bool #foreign libphysx "PxScene_removeBroadPhaseRegion_mut";

PxScene_getTaskManager :: (self: *PxScene) -> *PxTaskManager #foreign libphysx;

PxScene_lockRead :: (self: *PxScene, file: *u8, line: u32) -> void #foreign libphysx "PxScene_lockRead_mut";

PxScene_unlockRead :: (self: *PxScene) -> void #foreign libphysx "PxScene_unlockRead_mut";

PxScene_lockWrite :: (self: *PxScene, file: *u8, line: u32) -> void #foreign libphysx "PxScene_lockWrite_mut";

PxScene_unlockWrite :: (self: *PxScene) -> void #foreign libphysx "PxScene_unlockWrite_mut";

PxScene_setNbContactDataBlocks :: (self: *PxScene, numBlocks: u32) -> void #foreign libphysx "PxScene_setNbContactDataBlocks_mut";

PxScene_getNbContactDataBlocksUsed :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getMaxNbContactDataBlocksUsed :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getContactReportStreamBufferSize :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_setSolverBatchSize :: (self: *PxScene, solverBatchSize: u32) -> void #foreign libphysx "PxScene_setSolverBatchSize_mut";

PxScene_getSolverBatchSize :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_setSolverArticulationBatchSize :: (self: *PxScene, solverBatchSize: u32) -> void #foreign libphysx "PxScene_setSolverArticulationBatchSize_mut";

PxScene_getSolverArticulationBatchSize :: (self: *PxScene) -> u32 #foreign libphysx;

PxScene_getWakeCounterResetValue :: (self: *PxScene) -> float #foreign libphysx;

PxScene_shiftOrigin :: (self: *PxScene, shift: *Vector3) -> void #foreign libphysx "PxScene_shiftOrigin_mut";

PxScene_getScenePvdClient :: (self: *PxScene) -> *PxPvdSceneClient #foreign libphysx "PxScene_getScenePvdClient_mut";

PxScene_copyArticulationData :: (self: *PxScene, data: *void, index: *void, dataType: s32, nbCopyArticulations: u32, copyEvent: *void) -> void #foreign libphysx "PxScene_copyArticulationData_mut";

PxScene_applyArticulationData :: (self: *PxScene, data: *void, index: *void, dataType: s32, nbUpdatedArticulations: u32, waitEvent: *void, signalEvent: *void) -> void #foreign libphysx "PxScene_applyArticulationData_mut";

PxScene_copySoftBodyData :: (self: *PxScene, data: **void, dataSizes: *void, softBodyIndices: *void, flag: s32, nbCopySoftBodies: u32, maxSize: u32, copyEvent: *void) -> void #foreign libphysx "PxScene_copySoftBodyData_mut";

PxScene_applySoftBodyData :: (self: *PxScene, data: **void, dataSizes: *void, softBodyIndices: *void, flag: s32, nbUpdatedSoftBodies: u32, maxSize: u32, applyEvent: *void) -> void #foreign libphysx "PxScene_applySoftBodyData_mut";

PxScene_copyContactData :: (self: *PxScene, data: *void, maxContactPairs: u32, numContactPairs: *void, copyEvent: *void) -> void #foreign libphysx "PxScene_copyContactData_mut";

PxScene_copyBodyData :: (self: *PxScene, data: *PxGpuBodyData, index: *PxGpuActorPair, nbCopyActors: u32, copyEvent: *void) -> void #foreign libphysx "PxScene_copyBodyData_mut";

PxScene_applyActorData :: (self: *PxScene, data: *void, index: *PxGpuActorPair, flag: s32, nbUpdatedActors: u32, waitEvent: *void, signalEvent: *void) -> void #foreign libphysx "PxScene_applyActorData_mut";

PxScene_computeDenseJacobians :: (self: *PxScene, indices: *PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign libphysx "PxScene_computeDenseJacobians_mut";

PxScene_computeGeneralizedMassMatrices :: (self: *PxScene, indices: *PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign libphysx "PxScene_computeGeneralizedMassMatrices_mut";

PxScene_computeGeneralizedGravityForces :: (self: *PxScene, indices: *PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign libphysx "PxScene_computeGeneralizedGravityForces_mut";

PxScene_computeCoriolisAndCentrifugalForces :: (self: *PxScene, indices: *PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign libphysx "PxScene_computeCoriolisAndCentrifugalForces_mut";

PxScene_getGpuDynamicsConfig :: (self: *PxScene) -> PxgDynamicsMemoryConfig #foreign libphysx;

PxScene_applyParticleBufferData :: (self: *PxScene, indices: *u32, bufferIndexPair: *PxGpuParticleBufferIndexPair, flags: *u32, nbUpdatedBuffers: u32, waitEvent: *void, signalEvent: *void) -> void #foreign libphysx "PxScene_applyParticleBufferData_mut";

PxSceneReadLock_new_alloc :: (scene: *PxScene, file: *u8, line: u32) -> *PxSceneReadLock #foreign libphysx;

PxSceneReadLock_delete :: (self: *PxSceneReadLock) -> void #foreign libphysx;

PxSceneWriteLock_new_alloc :: (scene: *PxScene, file: *u8, line: u32) -> *PxSceneWriteLock #foreign libphysx;

PxSceneWriteLock_delete :: (self: *PxSceneWriteLock) -> void #foreign libphysx;

PxContactPairExtraDataItem_new :: () -> PxContactPairExtraDataItem #foreign libphysx;

PxContactPairVelocity_new :: () -> PxContactPairVelocity #foreign libphysx;

PxContactPairPose_new :: () -> PxContactPairPose #foreign libphysx;

PxContactPairIndex_new :: () -> PxContactPairIndex #foreign libphysx;

PxContactPairExtraDataIterator_new :: (stream: *u8, size: u32) -> PxContactPairExtraDataIterator #foreign libphysx;

PxContactPairExtraDataIterator_nextItemSet :: (self: *PxContactPairExtraDataIterator) -> bool #foreign libphysx "PxContactPairExtraDataIterator_nextItemSet_mut";

PxContactPairHeader_new :: () -> PxContactPairHeader #foreign libphysx;

PxContactPair_new :: () -> PxContactPair #foreign libphysx;

PxContactPair_extractContacts :: (self: *PxContactPair, userBuffer: *PxContactPairPoint, bufferSize: u32) -> u32 #foreign libphysx;

PxContactPair_bufferContacts :: (self: *PxContactPair, newPair: *PxContactPair, bufferMemory: *u8) -> void #foreign libphysx;

PxContactPair_getInternalFaceIndices :: (self: *PxContactPair) -> *u32 #foreign libphysx;

PxTriggerPair_new :: () -> PxTriggerPair #foreign libphysx;

PxConstraintInfo_new :: () -> PxConstraintInfo #foreign libphysx;

PxConstraintInfo_new :: (c: *PxConstraint, extRef: *void, t: u32) -> PxConstraintInfo #foreign libphysx "PxConstraintInfo_new_1";

PxSimulationEventCallback_onConstraintBreak :: (self: *PxSimulationEventCallback, constraints: *PxConstraintInfo, count: u32) -> void #foreign libphysx "PxSimulationEventCallback_onConstraintBreak_mut";

PxSimulationEventCallback_onWake :: (self: *PxSimulationEventCallback, actors: **PxActor, count: u32) -> void #foreign libphysx "PxSimulationEventCallback_onWake_mut";

PxSimulationEventCallback_onSleep :: (self: *PxSimulationEventCallback, actors: **PxActor, count: u32) -> void #foreign libphysx "PxSimulationEventCallback_onSleep_mut";

PxSimulationEventCallback_onContact :: (self: *PxSimulationEventCallback, pairHeader: *PxContactPairHeader, pairs: *PxContactPair, nbPairs: u32) -> void #foreign libphysx "PxSimulationEventCallback_onContact_mut";

PxSimulationEventCallback_onTrigger :: (self: *PxSimulationEventCallback, pairs: *PxTriggerPair, count: u32) -> void #foreign libphysx "PxSimulationEventCallback_onTrigger_mut";

PxSimulationEventCallback_onAdvance :: (self: *PxSimulationEventCallback, bodyBuffer: **PxRigidBody, poseBuffer: *PxTransform, count: u32) -> void #foreign libphysx "PxSimulationEventCallback_onAdvance_mut";

PxSimulationEventCallback_delete :: (self: *PxSimulationEventCallback) -> void #foreign libphysx;

PxFEMParameters_new :: () -> PxFEMParameters #foreign libphysx;

PxPruningStructure_release :: (self: *PxPruningStructure) -> void #foreign libphysx "PxPruningStructure_release_mut";

PxPruningStructure_getRigidActors :: (self: *PxPruningStructure, userBuffer: **PxRigidActor, bufferSize: u32, startIndex: u32) -> u32 #foreign libphysx;

PxPruningStructure_getNbRigidActors :: (self: *PxPruningStructure) -> u32 #foreign libphysx;

PxPruningStructure_getStaticMergeData :: (self: *PxPruningStructure) -> *void #foreign libphysx;

PxPruningStructure_getDynamicMergeData :: (self: *PxPruningStructure) -> *void #foreign libphysx;

PxPruningStructure_getConcreteTypeName :: (self: *PxPruningStructure) -> *u8 #foreign libphysx;

PxExtendedVec3_new :: () -> PxExtendedVec3 #foreign libphysx;

PxExtendedVec3_new :: (_x: float64, _y: float64, _z: float64) -> PxExtendedVec3 #foreign libphysx "PxExtendedVec3_new_1";

PxExtendedVec3_isZero :: (self: *PxExtendedVec3) -> bool #foreign libphysx;

PxExtendedVec3_dot :: (self: *PxExtendedVec3, v: *Vector3) -> float64 #foreign libphysx;

PxExtendedVec3_distanceSquared :: (self: *PxExtendedVec3, v: *PxExtendedVec3) -> float64 #foreign libphysx;

PxExtendedVec3_magnitudeSquared :: (self: *PxExtendedVec3) -> float64 #foreign libphysx;

PxExtendedVec3_magnitude :: (self: *PxExtendedVec3) -> float64 #foreign libphysx;

PxExtendedVec3_normalize :: (self: *PxExtendedVec3) -> float64 #foreign libphysx "PxExtendedVec3_normalize_mut";

PxExtendedVec3_isFinite :: (self: *PxExtendedVec3) -> bool #foreign libphysx;

PxExtendedVec3_maximum :: (self: *PxExtendedVec3, v: *PxExtendedVec3) -> void #foreign libphysx "PxExtendedVec3_maximum_mut";

PxExtendedVec3_minimum :: (self: *PxExtendedVec3, v: *PxExtendedVec3) -> void #foreign libphysx "PxExtendedVec3_minimum_mut";

PxExtendedVec3_set :: (self: *PxExtendedVec3, x_: float64, y_: float64, z_: float64) -> void #foreign libphysx "PxExtendedVec3_set_mut";

PxExtendedVec3_setPlusInfinity :: (self: *PxExtendedVec3) -> void #foreign libphysx "PxExtendedVec3_setPlusInfinity_mut";

PxExtendedVec3_setMinusInfinity :: (self: *PxExtendedVec3) -> void #foreign libphysx "PxExtendedVec3_setMinusInfinity_mut";

PxExtendedVec3_cross :: (self: *PxExtendedVec3, left: *PxExtendedVec3, right: *Vector3) -> void #foreign libphysx "PxExtendedVec3_cross_mut";

PxExtendedVec3_cross_mut :: (self: *PxExtendedVec3, left: *PxExtendedVec3, right: *PxExtendedVec3) -> void #foreign libphysx "PxExtendedVec3_cross_mut_1";

PxExtendedVec3_cross :: (self: *PxExtendedVec3, v: *PxExtendedVec3) -> PxExtendedVec3 #foreign libphysx;

PxExtendedVec3_cross_mut :: (self: *PxExtendedVec3, left: *Vector3, right: *PxExtendedVec3) -> void #foreign libphysx "PxExtendedVec3_cross_mut_2";

toVec3 :: (v: *PxExtendedVec3) -> Vector3 #foreign libphysx "phys_toVec3";

PxObstacle_getType :: (self: *PxObstacle) -> s32 #foreign libphysx;

PxBoxObstacle_new :: () -> PxBoxObstacle #foreign libphysx;

PxCapsuleObstacle_new :: () -> PxCapsuleObstacle #foreign libphysx;

PxObstacleContext_release :: (self: *PxObstacleContext) -> void #foreign libphysx "PxObstacleContext_release_mut";

PxObstacleContext_getControllerManager :: (self: *PxObstacleContext) -> *PxControllerManager #foreign libphysx;

PxObstacleContext_addObstacle :: (self: *PxObstacleContext, obstacle: *PxObstacle) -> u32 #foreign libphysx "PxObstacleContext_addObstacle_mut";

PxObstacleContext_removeObstacle :: (self: *PxObstacleContext, handle: u32) -> bool #foreign libphysx "PxObstacleContext_removeObstacle_mut";

PxObstacleContext_updateObstacle :: (self: *PxObstacleContext, handle: u32, obstacle: *PxObstacle) -> bool #foreign libphysx "PxObstacleContext_updateObstacle_mut";

PxObstacleContext_getNbObstacles :: (self: *PxObstacleContext) -> u32 #foreign libphysx;

PxObstacleContext_getObstacle :: (self: *PxObstacleContext, i: u32) -> *PxObstacle #foreign libphysx;

PxObstacleContext_getObstacleByHandle :: (self: *PxObstacleContext, handle: u32) -> *PxObstacle #foreign libphysx;

PxUserControllerHitReport_onShapeHit :: (self: *PxUserControllerHitReport, hit: *PxControllerShapeHit) -> void #foreign libphysx "PxUserControllerHitReport_onShapeHit_mut";

PxUserControllerHitReport_onControllerHit :: (self: *PxUserControllerHitReport, hit: *PxControllersHit) -> void #foreign libphysx "PxUserControllerHitReport_onControllerHit_mut";

PxUserControllerHitReport_onObstacleHit :: (self: *PxUserControllerHitReport, hit: *PxControllerObstacleHit) -> void #foreign libphysx "PxUserControllerHitReport_onObstacleHit_mut";

PxControllerFilterCallback_delete :: (self: *PxControllerFilterCallback) -> void #foreign libphysx;

PxControllerFilterCallback_filter :: (self: *PxControllerFilterCallback, a: *PxController, b: *PxController) -> bool #foreign libphysx "PxControllerFilterCallback_filter_mut";

PxControllerFilters_new :: (filterData: *PxFilterData, cb: *PxQueryFilterCallback, cctFilterCb: *PxControllerFilterCallback) -> PxControllerFilters #foreign libphysx;

PxControllerDesc_isValid :: (self: *PxControllerDesc) -> bool #foreign libphysx;

PxControllerDesc_getType :: (self: *PxControllerDesc) -> s32 #foreign libphysx;

PxController_getType :: (self: *PxController) -> s32 #foreign libphysx;

PxController_release :: (self: *PxController) -> void #foreign libphysx "PxController_release_mut";

PxController_move :: (self: *PxController, disp: *Vector3, minDist: float, elapsedTime: float, filters: *PxControllerFilters, obstacles: *PxObstacleContext) -> u8 #foreign libphysx "PxController_move_mut";

PxController_setPosition :: (self: *PxController, position: *PxExtendedVec3) -> bool #foreign libphysx "PxController_setPosition_mut";

PxController_getPosition :: (self: *PxController) -> *PxExtendedVec3 #foreign libphysx;

PxController_setFootPosition :: (self: *PxController, position: *PxExtendedVec3) -> bool #foreign libphysx "PxController_setFootPosition_mut";

PxController_getFootPosition :: (self: *PxController) -> PxExtendedVec3 #foreign libphysx;

PxController_getActor :: (self: *PxController) -> *PxRigidDynamic #foreign libphysx;

PxController_setStepOffset :: (self: *PxController, offset: float) -> void #foreign libphysx "PxController_setStepOffset_mut";

PxController_getStepOffset :: (self: *PxController) -> float #foreign libphysx;

PxController_setNonWalkableMode :: (self: *PxController, flag: s32) -> void #foreign libphysx "PxController_setNonWalkableMode_mut";

PxController_getNonWalkableMode :: (self: *PxController) -> s32 #foreign libphysx;

PxController_getContactOffset :: (self: *PxController) -> float #foreign libphysx;

PxController_setContactOffset :: (self: *PxController, offset: float) -> void #foreign libphysx "PxController_setContactOffset_mut";

PxController_getUpDirection :: (self: *PxController) -> Vector3 #foreign libphysx;

PxController_setUpDirection :: (self: *PxController, up: *Vector3) -> void #foreign libphysx "PxController_setUpDirection_mut";

PxController_getSlopeLimit :: (self: *PxController) -> float #foreign libphysx;

PxController_setSlopeLimit :: (self: *PxController, slopeLimit: float) -> void #foreign libphysx "PxController_setSlopeLimit_mut";

PxController_invalidateCache :: (self: *PxController) -> void #foreign libphysx "PxController_invalidateCache_mut";

PxController_getScene :: (self: *PxController) -> *PxScene #foreign libphysx "PxController_getScene_mut";

PxController_getUserData :: (self: *PxController) -> *void #foreign libphysx;

PxController_setUserData :: (self: *PxController, userData: *void) -> void #foreign libphysx "PxController_setUserData_mut";

PxController_getState :: (self: *PxController, state: *PxControllerState) -> void #foreign libphysx;

PxController_getStats :: (self: *PxController, stats: *PxControllerStats) -> void #foreign libphysx;

PxController_resize :: (self: *PxController, height: float) -> void #foreign libphysx "PxController_resize_mut";

PxBoxControllerDesc_new_alloc :: () -> *PxBoxControllerDesc #foreign libphysx;

PxBoxControllerDesc_delete :: (self: *PxBoxControllerDesc) -> void #foreign libphysx;

PxBoxControllerDesc_setToDefault :: (self: *PxBoxControllerDesc) -> void #foreign libphysx "PxBoxControllerDesc_setToDefault_mut";

PxBoxControllerDesc_isValid :: (self: *PxBoxControllerDesc) -> bool #foreign libphysx;

PxBoxController_getHalfHeight :: (self: *PxBoxController) -> float #foreign libphysx;

PxBoxController_getHalfSideExtent :: (self: *PxBoxController) -> float #foreign libphysx;

PxBoxController_getHalfForwardExtent :: (self: *PxBoxController) -> float #foreign libphysx;

PxBoxController_setHalfHeight :: (self: *PxBoxController, halfHeight: float) -> bool #foreign libphysx "PxBoxController_setHalfHeight_mut";

PxBoxController_setHalfSideExtent :: (self: *PxBoxController, halfSideExtent: float) -> bool #foreign libphysx "PxBoxController_setHalfSideExtent_mut";

PxBoxController_setHalfForwardExtent :: (self: *PxBoxController, halfForwardExtent: float) -> bool #foreign libphysx "PxBoxController_setHalfForwardExtent_mut";

PxCapsuleControllerDesc_new_alloc :: () -> *PxCapsuleControllerDesc #foreign libphysx;

PxCapsuleControllerDesc_delete :: (self: *PxCapsuleControllerDesc) -> void #foreign libphysx;

PxCapsuleControllerDesc_setToDefault :: (self: *PxCapsuleControllerDesc) -> void #foreign libphysx "PxCapsuleControllerDesc_setToDefault_mut";

PxCapsuleControllerDesc_isValid :: (self: *PxCapsuleControllerDesc) -> bool #foreign libphysx;

PxCapsuleController_getRadius :: (self: *PxCapsuleController) -> float #foreign libphysx;

PxCapsuleController_setRadius :: (self: *PxCapsuleController, radius: float) -> bool #foreign libphysx "PxCapsuleController_setRadius_mut";

PxCapsuleController_getHeight :: (self: *PxCapsuleController) -> float #foreign libphysx;

PxCapsuleController_setHeight :: (self: *PxCapsuleController, height: float) -> bool #foreign libphysx "PxCapsuleController_setHeight_mut";

PxCapsuleController_getClimbingMode :: (self: *PxCapsuleController) -> s32 #foreign libphysx;

PxCapsuleController_setClimbingMode :: (self: *PxCapsuleController, mode: s32) -> bool #foreign libphysx "PxCapsuleController_setClimbingMode_mut";

PxControllerBehaviorCallback_getBehaviorFlags :: (self: *PxControllerBehaviorCallback, shape: *PxShape, actor: *PxActor) -> u8 #foreign libphysx "PxControllerBehaviorCallback_getBehaviorFlags_mut";

PxControllerBehaviorCallback_getBehaviorFlags_mut :: (self: *PxControllerBehaviorCallback, controller: *PxController) -> u8 #foreign libphysx "PxControllerBehaviorCallback_getBehaviorFlags_mut_1";

PxControllerBehaviorCallback_getBehaviorFlags_mut :: (self: *PxControllerBehaviorCallback, obstacle: *PxObstacle) -> u8 #foreign libphysx "PxControllerBehaviorCallback_getBehaviorFlags_mut_2";

PxControllerManager_release :: (self: *PxControllerManager) -> void #foreign libphysx "PxControllerManager_release_mut";

PxControllerManager_getScene :: (self: *PxControllerManager) -> *PxScene #foreign libphysx;

PxControllerManager_getNbControllers :: (self: *PxControllerManager) -> u32 #foreign libphysx;

PxControllerManager_getController :: (self: *PxControllerManager, index: u32) -> *PxController #foreign libphysx "PxControllerManager_getController_mut";

PxControllerManager_createController :: (self: *PxControllerManager, desc: *PxControllerDesc) -> *PxController #foreign libphysx "PxControllerManager_createController_mut";

PxControllerManager_purgeControllers :: (self: *PxControllerManager) -> void #foreign libphysx "PxControllerManager_purgeControllers_mut";

PxControllerManager_getRenderBuffer :: (self: *PxControllerManager) -> *PxRenderBuffer #foreign libphysx "PxControllerManager_getRenderBuffer_mut";

PxControllerManager_setDebugRenderingFlags :: (self: *PxControllerManager, flags: u32) -> void #foreign libphysx "PxControllerManager_setDebugRenderingFlags_mut";

PxControllerManager_getNbObstacleContexts :: (self: *PxControllerManager) -> u32 #foreign libphysx;

PxControllerManager_getObstacleContext :: (self: *PxControllerManager, index: u32) -> *PxObstacleContext #foreign libphysx "PxControllerManager_getObstacleContext_mut";

PxControllerManager_createObstacleContext :: (self: *PxControllerManager) -> *PxObstacleContext #foreign libphysx "PxControllerManager_createObstacleContext_mut";

PxControllerManager_computeInteractions :: (self: *PxControllerManager, elapsedTime: float, cctFilterCb: *PxControllerFilterCallback) -> void #foreign libphysx "PxControllerManager_computeInteractions_mut";

PxControllerManager_setTessellation :: (self: *PxControllerManager, flag: bool, maxEdgeLength: float) -> void #foreign libphysx "PxControllerManager_setTessellation_mut";

PxControllerManager_setOverlapRecoveryModule :: (self: *PxControllerManager, flag: bool) -> void #foreign libphysx "PxControllerManager_setOverlapRecoveryModule_mut";

PxControllerManager_setPreciseSweeps :: (self: *PxControllerManager, flag: bool) -> void #foreign libphysx "PxControllerManager_setPreciseSweeps_mut";

PxControllerManager_setPreventVerticalSlidingAgainstCeiling :: (self: *PxControllerManager, flag: bool) -> void #foreign libphysx "PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut";

PxControllerManager_shiftOrigin :: (self: *PxControllerManager, shift: *Vector3) -> void #foreign libphysx "PxControllerManager_shiftOrigin_mut";

PxCreateControllerManager :: (scene: *PxScene, lockingEnabled: bool) -> *PxControllerManager #foreign libphysx "phys_PxCreateControllerManager";

PxDim3_new :: () -> PxDim3 #foreign libphysx;

PxSDFDesc_new :: () -> PxSDFDesc #foreign libphysx;

PxSDFDesc_isValid :: (self: *PxSDFDesc) -> bool #foreign libphysx;

PxConvexMeshDesc_new :: () -> PxConvexMeshDesc #foreign libphysx;

PxConvexMeshDesc_setToDefault :: (self: *PxConvexMeshDesc) -> void #foreign libphysx "PxConvexMeshDesc_setToDefault_mut";

PxConvexMeshDesc_isValid :: (self: *PxConvexMeshDesc) -> bool #foreign libphysx;

PxTriangleMeshDesc_new :: () -> PxTriangleMeshDesc #foreign libphysx;

PxTriangleMeshDesc_setToDefault :: (self: *PxTriangleMeshDesc) -> void #foreign libphysx "PxTriangleMeshDesc_setToDefault_mut";

PxTriangleMeshDesc_isValid :: (self: *PxTriangleMeshDesc) -> bool #foreign libphysx;

PxTetrahedronMeshDesc_new :: () -> PxTetrahedronMeshDesc #foreign libphysx;

PxTetrahedronMeshDesc_isValid :: (self: *PxTetrahedronMeshDesc) -> bool #foreign libphysx;

PxSoftBodySimulationDataDesc_new :: () -> PxSoftBodySimulationDataDesc #foreign libphysx;

PxSoftBodySimulationDataDesc_isValid :: (self: *PxSoftBodySimulationDataDesc) -> bool #foreign libphysx;

PxBVH34MidphaseDesc_setToDefault :: (self: *PxBVH34MidphaseDesc) -> void #foreign libphysx "PxBVH34MidphaseDesc_setToDefault_mut";

PxBVH34MidphaseDesc_isValid :: (self: *PxBVH34MidphaseDesc) -> bool #foreign libphysx;

PxMidphaseDesc_new :: () -> PxMidphaseDesc #foreign libphysx;

PxMidphaseDesc_getType :: (self: *PxMidphaseDesc) -> s32 #foreign libphysx;

PxMidphaseDesc_setToDefault :: (self: *PxMidphaseDesc, type: s32) -> void #foreign libphysx "PxMidphaseDesc_setToDefault_mut";

PxMidphaseDesc_isValid :: (self: *PxMidphaseDesc) -> bool #foreign libphysx;

PxBVHDesc_new :: () -> PxBVHDesc #foreign libphysx;

PxBVHDesc_setToDefault :: (self: *PxBVHDesc) -> void #foreign libphysx "PxBVHDesc_setToDefault_mut";

PxBVHDesc_isValid :: (self: *PxBVHDesc) -> bool #foreign libphysx;

PxCookingParams_new :: (sc: *PxTolerancesScale) -> PxCookingParams #foreign libphysx;

PxGetStandaloneInsertionCallback :: () -> *PxInsertionCallback #foreign libphysx "phys_PxGetStandaloneInsertionCallback";

PxCookBVH :: (desc: *PxBVHDesc, stream: *PxOutputStream) -> bool #foreign libphysx "phys_PxCookBVH";

PxCreateBVH :: (desc: *PxBVHDesc, insertionCallback: *PxInsertionCallback) -> *PxBVH #foreign libphysx "phys_PxCreateBVH";

PxCookHeightField :: (desc: *PxHeightFieldDesc, stream: *PxOutputStream) -> bool #foreign libphysx "phys_PxCookHeightField";

PxCreateHeightField :: (desc: *PxHeightFieldDesc, insertionCallback: *PxInsertionCallback) -> *PxHeightField #foreign libphysx "phys_PxCreateHeightField";

PxCookConvexMesh :: (params: *PxCookingParams, desc: *PxConvexMeshDesc, stream: *PxOutputStream, condition: *s32) -> bool #foreign libphysx "phys_PxCookConvexMesh";

PxCreateConvexMesh :: (params: *PxCookingParams, desc: *PxConvexMeshDesc, insertionCallback: *PxInsertionCallback, condition: *s32) -> *PxConvexMesh #foreign libphysx "phys_PxCreateConvexMesh";

PxValidateConvexMesh :: (params: *PxCookingParams, desc: *PxConvexMeshDesc) -> bool #foreign libphysx "phys_PxValidateConvexMesh";

PxComputeHullPolygons :: (params: *PxCookingParams, mesh: *PxSimpleTriangleMesh, inCallback: *PxAllocatorCallback, nbVerts: *u32, vertices: **Vector3, nbIndices: *u32, indices: **u32, nbPolygons: *u32, hullPolygons: **PxHullPolygon) -> bool #foreign libphysx "phys_PxComputeHullPolygons";

PxValidateTriangleMesh :: (params: *PxCookingParams, desc: *PxTriangleMeshDesc) -> bool #foreign libphysx "phys_PxValidateTriangleMesh";

PxCreateTriangleMesh :: (params: *PxCookingParams, desc: *PxTriangleMeshDesc, insertionCallback: *PxInsertionCallback, condition: *s32) -> *PxTriangleMesh #foreign libphysx "phys_PxCreateTriangleMesh";

PxCookTriangleMesh :: (params: *PxCookingParams, desc: *PxTriangleMeshDesc, stream: *PxOutputStream, condition: *s32) -> bool #foreign libphysx "phys_PxCookTriangleMesh";

PxDefaultMemoryOutputStream_new_alloc :: (allocator: *PxAllocatorCallback) -> *PxDefaultMemoryOutputStream #foreign libphysx;

PxDefaultMemoryOutputStream_delete :: (self: *PxDefaultMemoryOutputStream) -> void #foreign libphysx;

PxDefaultMemoryOutputStream_write :: (self: *PxDefaultMemoryOutputStream, src: *void, count: u32) -> u32 #foreign libphysx "PxDefaultMemoryOutputStream_write_mut";

PxDefaultMemoryOutputStream_getSize :: (self: *PxDefaultMemoryOutputStream) -> u32 #foreign libphysx;

PxDefaultMemoryOutputStream_getData :: (self: *PxDefaultMemoryOutputStream) -> *u8 #foreign libphysx;

PxDefaultMemoryInputData_new_alloc :: (data: *u8, length: u32) -> *PxDefaultMemoryInputData #foreign libphysx;

PxDefaultMemoryInputData_read :: (self: *PxDefaultMemoryInputData, dest: *void, count: u32) -> u32 #foreign libphysx "PxDefaultMemoryInputData_read_mut";

PxDefaultMemoryInputData_getLength :: (self: *PxDefaultMemoryInputData) -> u32 #foreign libphysx;

PxDefaultMemoryInputData_seek :: (self: *PxDefaultMemoryInputData, pos: u32) -> void #foreign libphysx "PxDefaultMemoryInputData_seek_mut";

PxDefaultMemoryInputData_tell :: (self: *PxDefaultMemoryInputData) -> u32 #foreign libphysx;

PxDefaultFileOutputStream_new_alloc :: (name: *u8) -> *PxDefaultFileOutputStream #foreign libphysx;

PxDefaultFileOutputStream_delete :: (self: *PxDefaultFileOutputStream) -> void #foreign libphysx;

PxDefaultFileOutputStream_write :: (self: *PxDefaultFileOutputStream, src: *void, count: u32) -> u32 #foreign libphysx "PxDefaultFileOutputStream_write_mut";

PxDefaultFileOutputStream_isValid :: (self: *PxDefaultFileOutputStream) -> bool #foreign libphysx "PxDefaultFileOutputStream_isValid_mut";

PxDefaultFileInputData_new_alloc :: (name: *u8) -> *PxDefaultFileInputData #foreign libphysx;

PxDefaultFileInputData_delete :: (self: *PxDefaultFileInputData) -> void #foreign libphysx;

PxDefaultFileInputData_read :: (self: *PxDefaultFileInputData, dest: *void, count: u32) -> u32 #foreign libphysx "PxDefaultFileInputData_read_mut";

PxDefaultFileInputData_seek :: (self: *PxDefaultFileInputData, pos: u32) -> void #foreign libphysx "PxDefaultFileInputData_seek_mut";

PxDefaultFileInputData_tell :: (self: *PxDefaultFileInputData) -> u32 #foreign libphysx;

PxDefaultFileInputData_getLength :: (self: *PxDefaultFileInputData) -> u32 #foreign libphysx;

PxDefaultFileInputData_isValid :: (self: *PxDefaultFileInputData) -> bool #foreign libphysx;

platformAlignedAlloc :: (size: u64) -> *void #foreign libphysx "phys_platformAlignedAlloc";

platformAlignedFree :: (ptr: *void) -> void #foreign libphysx "phys_platformAlignedFree";

PxDefaultAllocator_allocate :: (self: *PxDefaultAllocator, size: u64, anon_param1: *u8, anon_param2: *u8, anon_param3: s32) -> *void #foreign libphysx "PxDefaultAllocator_allocate_mut";

PxDefaultAllocator_deallocate :: (self: *PxDefaultAllocator, ptr: *void) -> void #foreign libphysx "PxDefaultAllocator_deallocate_mut";

PxDefaultAllocator_delete :: (self: *PxDefaultAllocator) -> void #foreign libphysx;

PxJoint_setActors :: (self: *PxJoint, actor0: *PxRigidActor, actor1: *PxRigidActor) -> void #foreign libphysx "PxJoint_setActors_mut";

PxJoint_getActors :: (self: *PxJoint, actor0: **PxRigidActor, actor1: **PxRigidActor) -> void #foreign libphysx;

PxJoint_setLocalPose :: (self: *PxJoint, actor: s32, localPose: *PxTransform) -> void #foreign libphysx "PxJoint_setLocalPose_mut";

PxJoint_getLocalPose :: (self: *PxJoint, actor: s32) -> PxTransform #foreign libphysx;

PxJoint_getRelativeTransform :: (self: *PxJoint) -> PxTransform #foreign libphysx;

PxJoint_getRelativeLinearVelocity :: (self: *PxJoint) -> Vector3 #foreign libphysx;

PxJoint_getRelativeAngularVelocity :: (self: *PxJoint) -> Vector3 #foreign libphysx;

PxJoint_setBreakForce :: (self: *PxJoint, force: float, torque: float) -> void #foreign libphysx "PxJoint_setBreakForce_mut";

PxJoint_getBreakForce :: (self: *PxJoint, force: *float, torque: *float) -> void #foreign libphysx;

PxJoint_setConstraintFlags :: (self: *PxJoint, flags: u16) -> void #foreign libphysx "PxJoint_setConstraintFlags_mut";

PxJoint_setConstraintFlag :: (self: *PxJoint, flag: s32, value: bool) -> void #foreign libphysx "PxJoint_setConstraintFlag_mut";

PxJoint_getConstraintFlags :: (self: *PxJoint) -> u16 #foreign libphysx;

PxJoint_setInvMassScale0 :: (self: *PxJoint, invMassScale: float) -> void #foreign libphysx "PxJoint_setInvMassScale0_mut";

PxJoint_getInvMassScale0 :: (self: *PxJoint) -> float #foreign libphysx;

PxJoint_setInvInertiaScale0 :: (self: *PxJoint, invInertiaScale: float) -> void #foreign libphysx "PxJoint_setInvInertiaScale0_mut";

PxJoint_getInvInertiaScale0 :: (self: *PxJoint) -> float #foreign libphysx;

PxJoint_setInvMassScale1 :: (self: *PxJoint, invMassScale: float) -> void #foreign libphysx "PxJoint_setInvMassScale1_mut";

PxJoint_getInvMassScale1 :: (self: *PxJoint) -> float #foreign libphysx;

PxJoint_setInvInertiaScale1 :: (self: *PxJoint, invInertiaScale: float) -> void #foreign libphysx "PxJoint_setInvInertiaScale1_mut";

PxJoint_getInvInertiaScale1 :: (self: *PxJoint) -> float #foreign libphysx;

PxJoint_getConstraint :: (self: *PxJoint) -> *PxConstraint #foreign libphysx;

PxJoint_setName :: (self: *PxJoint, name: *u8) -> void #foreign libphysx "PxJoint_setName_mut";

PxJoint_getName :: (self: *PxJoint) -> *u8 #foreign libphysx;

PxJoint_release :: (self: *PxJoint) -> void #foreign libphysx "PxJoint_release_mut";

PxJoint_getScene :: (self: *PxJoint) -> *PxScene #foreign libphysx;

PxJoint_getBinaryMetaData :: (stream: *PxOutputStream) -> void #foreign libphysx;

PxSpring_new :: (stiffness_: float, damping_: float) -> PxSpring #foreign libphysx;

PxSetJointGlobalFrame :: (joint: *PxJoint, wsAnchor: *Vector3, wsAxis: *Vector3) -> void #foreign libphysx "phys_PxSetJointGlobalFrame";

PxDistanceJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxDistanceJoint #foreign libphysx "phys_PxDistanceJointCreate";

PxDistanceJoint_getDistance :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setMinDistance :: (self: *PxDistanceJoint, distance: float) -> void #foreign libphysx "PxDistanceJoint_setMinDistance_mut";

PxDistanceJoint_getMinDistance :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setMaxDistance :: (self: *PxDistanceJoint, distance: float) -> void #foreign libphysx "PxDistanceJoint_setMaxDistance_mut";

PxDistanceJoint_getMaxDistance :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setTolerance :: (self: *PxDistanceJoint, tolerance: float) -> void #foreign libphysx "PxDistanceJoint_setTolerance_mut";

PxDistanceJoint_getTolerance :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setStiffness :: (self: *PxDistanceJoint, stiffness: float) -> void #foreign libphysx "PxDistanceJoint_setStiffness_mut";

PxDistanceJoint_getStiffness :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setDamping :: (self: *PxDistanceJoint, damping: float) -> void #foreign libphysx "PxDistanceJoint_setDamping_mut";

PxDistanceJoint_getDamping :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setContactDistance :: (self: *PxDistanceJoint, contactDistance: float) -> void #foreign libphysx "PxDistanceJoint_setContactDistance_mut";

PxDistanceJoint_getContactDistance :: (self: *PxDistanceJoint) -> float #foreign libphysx;

PxDistanceJoint_setDistanceJointFlags :: (self: *PxDistanceJoint, flags: u16) -> void #foreign libphysx "PxDistanceJoint_setDistanceJointFlags_mut";

PxDistanceJoint_setDistanceJointFlag :: (self: *PxDistanceJoint, flag: s32, value: bool) -> void #foreign libphysx "PxDistanceJoint_setDistanceJointFlag_mut";

PxDistanceJoint_getDistanceJointFlags :: (self: *PxDistanceJoint) -> u16 #foreign libphysx;

PxDistanceJoint_getConcreteTypeName :: (self: *PxDistanceJoint) -> *u8 #foreign libphysx;

PxContactJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxContactJoint #foreign libphysx "phys_PxContactJointCreate";

PxJacobianRow_new :: () -> PxJacobianRow #foreign libphysx;

PxJacobianRow_new :: (lin0: *Vector3, lin1: *Vector3, ang0: *Vector3, ang1: *Vector3) -> PxJacobianRow #foreign libphysx "PxJacobianRow_new_1";

PxContactJoint_setContact :: (self: *PxContactJoint, contact: *Vector3) -> void #foreign libphysx "PxContactJoint_setContact_mut";

PxContactJoint_setContactNormal :: (self: *PxContactJoint, contactNormal: *Vector3) -> void #foreign libphysx "PxContactJoint_setContactNormal_mut";

PxContactJoint_setPenetration :: (self: *PxContactJoint, penetration: float) -> void #foreign libphysx "PxContactJoint_setPenetration_mut";

PxContactJoint_getContact :: (self: *PxContactJoint) -> Vector3 #foreign libphysx;

PxContactJoint_getContactNormal :: (self: *PxContactJoint) -> Vector3 #foreign libphysx;

PxContactJoint_getPenetration :: (self: *PxContactJoint) -> float #foreign libphysx;

PxContactJoint_getRestitution :: (self: *PxContactJoint) -> float #foreign libphysx;

PxContactJoint_setRestitution :: (self: *PxContactJoint, restitution: float) -> void #foreign libphysx "PxContactJoint_setRestitution_mut";

PxContactJoint_getBounceThreshold :: (self: *PxContactJoint) -> float #foreign libphysx;

PxContactJoint_setBounceThreshold :: (self: *PxContactJoint, bounceThreshold: float) -> void #foreign libphysx "PxContactJoint_setBounceThreshold_mut";

PxContactJoint_getConcreteTypeName :: (self: *PxContactJoint) -> *u8 #foreign libphysx;

PxContactJoint_computeJacobians :: (self: *PxContactJoint, jacobian: *PxJacobianRow) -> void #foreign libphysx;

PxContactJoint_getNbJacobianRows :: (self: *PxContactJoint) -> u32 #foreign libphysx;

PxFixedJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxFixedJoint #foreign libphysx "phys_PxFixedJointCreate";

PxFixedJoint_getConcreteTypeName :: (self: *PxFixedJoint) -> *u8 #foreign libphysx;

PxJointLimitParameters_new_alloc :: () -> *PxJointLimitParameters #foreign libphysx;

PxJointLimitParameters_isValid :: (self: *PxJointLimitParameters) -> bool #foreign libphysx;

PxJointLimitParameters_isSoft :: (self: *PxJointLimitParameters) -> bool #foreign libphysx;

PxJointLinearLimit_new :: (scale: *PxTolerancesScale, extent: float, contactDist_deprecated: float) -> PxJointLinearLimit #foreign libphysx;

PxJointLinearLimit_new :: (extent: float, spring: *PxSpring) -> PxJointLinearLimit #foreign libphysx "PxJointLinearLimit_new_1";

PxJointLinearLimit_isValid :: (self: *PxJointLinearLimit) -> bool #foreign libphysx;

PxJointLinearLimit_delete :: (self: *PxJointLinearLimit) -> void #foreign libphysx;

PxJointLinearLimitPair_new :: (scale: *PxTolerancesScale, lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> PxJointLinearLimitPair #foreign libphysx;

PxJointLinearLimitPair_new :: (lowerLimit: float, upperLimit: float, spring: *PxSpring) -> PxJointLinearLimitPair #foreign libphysx "PxJointLinearLimitPair_new_1";

PxJointLinearLimitPair_isValid :: (self: *PxJointLinearLimitPair) -> bool #foreign libphysx;

PxJointLinearLimitPair_delete :: (self: *PxJointLinearLimitPair) -> void #foreign libphysx;

PxJointAngularLimitPair_new :: (lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> PxJointAngularLimitPair #foreign libphysx;

PxJointAngularLimitPair_new :: (lowerLimit: float, upperLimit: float, spring: *PxSpring) -> PxJointAngularLimitPair #foreign libphysx "PxJointAngularLimitPair_new_1";

PxJointAngularLimitPair_isValid :: (self: *PxJointAngularLimitPair) -> bool #foreign libphysx;

PxJointAngularLimitPair_delete :: (self: *PxJointAngularLimitPair) -> void #foreign libphysx;

PxJointLimitCone_new :: (yLimitAngle: float, zLimitAngle: float, contactDist_deprecated: float) -> PxJointLimitCone #foreign libphysx;

PxJointLimitCone_new :: (yLimitAngle: float, zLimitAngle: float, spring: *PxSpring) -> PxJointLimitCone #foreign libphysx "PxJointLimitCone_new_1";

PxJointLimitCone_isValid :: (self: *PxJointLimitCone) -> bool #foreign libphysx;

PxJointLimitCone_delete :: (self: *PxJointLimitCone) -> void #foreign libphysx;

PxJointLimitPyramid_new :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, contactDist_deprecated: float) -> PxJointLimitPyramid #foreign libphysx;

PxJointLimitPyramid_new :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, spring: *PxSpring) -> PxJointLimitPyramid #foreign libphysx "PxJointLimitPyramid_new_1";

PxJointLimitPyramid_isValid :: (self: *PxJointLimitPyramid) -> bool #foreign libphysx;

PxJointLimitPyramid_delete :: (self: *PxJointLimitPyramid) -> void #foreign libphysx;

PxPrismaticJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxPrismaticJoint #foreign libphysx "phys_PxPrismaticJointCreate";

PxPrismaticJoint_getPosition :: (self: *PxPrismaticJoint) -> float #foreign libphysx;

PxPrismaticJoint_getVelocity :: (self: *PxPrismaticJoint) -> float #foreign libphysx;

PxPrismaticJoint_setLimit :: (self: *PxPrismaticJoint, anon_param0: *PxJointLinearLimitPair) -> void #foreign libphysx "PxPrismaticJoint_setLimit_mut";

PxPrismaticJoint_getLimit :: (self: *PxPrismaticJoint) -> PxJointLinearLimitPair #foreign libphysx;

PxPrismaticJoint_setPrismaticJointFlags :: (self: *PxPrismaticJoint, flags: u16) -> void #foreign libphysx "PxPrismaticJoint_setPrismaticJointFlags_mut";

PxPrismaticJoint_setPrismaticJointFlag :: (self: *PxPrismaticJoint, flag: s32, value: bool) -> void #foreign libphysx "PxPrismaticJoint_setPrismaticJointFlag_mut";

PxPrismaticJoint_getPrismaticJointFlags :: (self: *PxPrismaticJoint) -> u16 #foreign libphysx;

PxPrismaticJoint_getConcreteTypeName :: (self: *PxPrismaticJoint) -> *u8 #foreign libphysx;

PxRevoluteJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxRevoluteJoint #foreign libphysx "phys_PxRevoluteJointCreate";

PxRevoluteJoint_getAngle :: (self: *PxRevoluteJoint) -> float #foreign libphysx;

PxRevoluteJoint_getVelocity :: (self: *PxRevoluteJoint) -> float #foreign libphysx;

PxRevoluteJoint_setLimit :: (self: *PxRevoluteJoint, limits: *PxJointAngularLimitPair) -> void #foreign libphysx "PxRevoluteJoint_setLimit_mut";

PxRevoluteJoint_getLimit :: (self: *PxRevoluteJoint) -> PxJointAngularLimitPair #foreign libphysx;

PxRevoluteJoint_setDriveVelocity :: (self: *PxRevoluteJoint, velocity: float, autowake: bool) -> void #foreign libphysx "PxRevoluteJoint_setDriveVelocity_mut";

PxRevoluteJoint_getDriveVelocity :: (self: *PxRevoluteJoint) -> float #foreign libphysx;

PxRevoluteJoint_setDriveForceLimit :: (self: *PxRevoluteJoint, limit: float) -> void #foreign libphysx "PxRevoluteJoint_setDriveForceLimit_mut";

PxRevoluteJoint_getDriveForceLimit :: (self: *PxRevoluteJoint) -> float #foreign libphysx;

PxRevoluteJoint_setDriveGearRatio :: (self: *PxRevoluteJoint, ratio: float) -> void #foreign libphysx "PxRevoluteJoint_setDriveGearRatio_mut";

PxRevoluteJoint_getDriveGearRatio :: (self: *PxRevoluteJoint) -> float #foreign libphysx;

PxRevoluteJoint_setRevoluteJointFlags :: (self: *PxRevoluteJoint, flags: u16) -> void #foreign libphysx "PxRevoluteJoint_setRevoluteJointFlags_mut";

PxRevoluteJoint_setRevoluteJointFlag :: (self: *PxRevoluteJoint, flag: s32, value: bool) -> void #foreign libphysx "PxRevoluteJoint_setRevoluteJointFlag_mut";

PxRevoluteJoint_getRevoluteJointFlags :: (self: *PxRevoluteJoint) -> u16 #foreign libphysx;

PxRevoluteJoint_getConcreteTypeName :: (self: *PxRevoluteJoint) -> *u8 #foreign libphysx;

PxSphericalJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxSphericalJoint #foreign libphysx "phys_PxSphericalJointCreate";

PxSphericalJoint_getLimitCone :: (self: *PxSphericalJoint) -> PxJointLimitCone #foreign libphysx;

PxSphericalJoint_setLimitCone :: (self: *PxSphericalJoint, limit: *PxJointLimitCone) -> void #foreign libphysx "PxSphericalJoint_setLimitCone_mut";

PxSphericalJoint_getSwingYAngle :: (self: *PxSphericalJoint) -> float #foreign libphysx;

PxSphericalJoint_getSwingZAngle :: (self: *PxSphericalJoint) -> float #foreign libphysx;

PxSphericalJoint_setSphericalJointFlags :: (self: *PxSphericalJoint, flags: u16) -> void #foreign libphysx "PxSphericalJoint_setSphericalJointFlags_mut";

PxSphericalJoint_setSphericalJointFlag :: (self: *PxSphericalJoint, flag: s32, value: bool) -> void #foreign libphysx "PxSphericalJoint_setSphericalJointFlag_mut";

PxSphericalJoint_getSphericalJointFlags :: (self: *PxSphericalJoint) -> u16 #foreign libphysx;

PxSphericalJoint_getConcreteTypeName :: (self: *PxSphericalJoint) -> *u8 #foreign libphysx;

PxD6JointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxD6Joint #foreign libphysx "phys_PxD6JointCreate";

PxD6JointDrive_new :: () -> PxD6JointDrive #foreign libphysx;

PxD6JointDrive_new :: (driveStiffness: float, driveDamping: float, driveForceLimit: float, isAcceleration: bool) -> PxD6JointDrive #foreign libphysx "PxD6JointDrive_new_1";

PxD6JointDrive_isValid :: (self: *PxD6JointDrive) -> bool #foreign libphysx;

PxD6Joint_setMotion :: (self: *PxD6Joint, axis: s32, type: s32) -> void #foreign libphysx "PxD6Joint_setMotion_mut";

PxD6Joint_getMotion :: (self: *PxD6Joint, axis: s32) -> s32 #foreign libphysx;

PxD6Joint_getTwistAngle :: (self: *PxD6Joint) -> float #foreign libphysx;

PxD6Joint_getSwingYAngle :: (self: *PxD6Joint) -> float #foreign libphysx;

PxD6Joint_getSwingZAngle :: (self: *PxD6Joint) -> float #foreign libphysx;

PxD6Joint_setDistanceLimit :: (self: *PxD6Joint, limit: *PxJointLinearLimit) -> void #foreign libphysx "PxD6Joint_setDistanceLimit_mut";

PxD6Joint_getDistanceLimit :: (self: *PxD6Joint) -> PxJointLinearLimit #foreign libphysx;

PxD6Joint_setLinearLimit :: (self: *PxD6Joint, axis: s32, limit: *PxJointLinearLimitPair) -> void #foreign libphysx "PxD6Joint_setLinearLimit_mut";

PxD6Joint_getLinearLimit :: (self: *PxD6Joint, axis: s32) -> PxJointLinearLimitPair #foreign libphysx;

PxD6Joint_setTwistLimit :: (self: *PxD6Joint, limit: *PxJointAngularLimitPair) -> void #foreign libphysx "PxD6Joint_setTwistLimit_mut";

PxD6Joint_getTwistLimit :: (self: *PxD6Joint) -> PxJointAngularLimitPair #foreign libphysx;

PxD6Joint_setSwingLimit :: (self: *PxD6Joint, limit: *PxJointLimitCone) -> void #foreign libphysx "PxD6Joint_setSwingLimit_mut";

PxD6Joint_getSwingLimit :: (self: *PxD6Joint) -> PxJointLimitCone #foreign libphysx;

PxD6Joint_setPyramidSwingLimit :: (self: *PxD6Joint, limit: *PxJointLimitPyramid) -> void #foreign libphysx "PxD6Joint_setPyramidSwingLimit_mut";

PxD6Joint_getPyramidSwingLimit :: (self: *PxD6Joint) -> PxJointLimitPyramid #foreign libphysx;

PxD6Joint_setDrive :: (self: *PxD6Joint, index: s32, drive: *PxD6JointDrive) -> void #foreign libphysx "PxD6Joint_setDrive_mut";

PxD6Joint_getDrive :: (self: *PxD6Joint, index: s32) -> PxD6JointDrive #foreign libphysx;

PxD6Joint_setDrivePosition :: (self: *PxD6Joint, pose: *PxTransform, autowake: bool) -> void #foreign libphysx "PxD6Joint_setDrivePosition_mut";

PxD6Joint_getDrivePosition :: (self: *PxD6Joint) -> PxTransform #foreign libphysx;

PxD6Joint_setDriveVelocity :: (self: *PxD6Joint, linear: *Vector3, angular: *Vector3, autowake: bool) -> void #foreign libphysx "PxD6Joint_setDriveVelocity_mut";

PxD6Joint_getDriveVelocity :: (self: *PxD6Joint, linear: *Vector3, angular: *Vector3) -> void #foreign libphysx;

PxD6Joint_setProjectionLinearTolerance :: (self: *PxD6Joint, tolerance: float) -> void #foreign libphysx "PxD6Joint_setProjectionLinearTolerance_mut";

PxD6Joint_getProjectionLinearTolerance :: (self: *PxD6Joint) -> float #foreign libphysx;

PxD6Joint_setProjectionAngularTolerance :: (self: *PxD6Joint, tolerance: float) -> void #foreign libphysx "PxD6Joint_setProjectionAngularTolerance_mut";

PxD6Joint_getProjectionAngularTolerance :: (self: *PxD6Joint) -> float #foreign libphysx;

PxD6Joint_getConcreteTypeName :: (self: *PxD6Joint) -> *u8 #foreign libphysx;

PxGearJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxGearJoint #foreign libphysx "phys_PxGearJointCreate";

PxGearJoint_setHinges :: (self: *PxGearJoint, hinge0: *PxBase, hinge1: *PxBase) -> bool #foreign libphysx "PxGearJoint_setHinges_mut";

PxGearJoint_setGearRatio :: (self: *PxGearJoint, ratio: float) -> void #foreign libphysx "PxGearJoint_setGearRatio_mut";

PxGearJoint_getGearRatio :: (self: *PxGearJoint) -> float #foreign libphysx;

PxGearJoint_getConcreteTypeName :: (self: *PxGearJoint) -> *u8 #foreign libphysx;

PxRackAndPinionJointCreate :: (physics: *PxPhysics, actor0: *PxRigidActor, localFrame0: *PxTransform, actor1: *PxRigidActor, localFrame1: *PxTransform) -> *PxRackAndPinionJoint #foreign libphysx "phys_PxRackAndPinionJointCreate";

PxRackAndPinionJoint_setJoints :: (self: *PxRackAndPinionJoint, hinge: *PxBase, prismatic: *PxBase) -> bool #foreign libphysx "PxRackAndPinionJoint_setJoints_mut";

PxRackAndPinionJoint_setRatio :: (self: *PxRackAndPinionJoint, ratio: float) -> void #foreign libphysx "PxRackAndPinionJoint_setRatio_mut";

PxRackAndPinionJoint_getRatio :: (self: *PxRackAndPinionJoint) -> float #foreign libphysx;

PxRackAndPinionJoint_setData :: (self: *PxRackAndPinionJoint, nbRackTeeth: u32, nbPinionTeeth: u32, rackLength: float) -> bool #foreign libphysx "PxRackAndPinionJoint_setData_mut";

PxRackAndPinionJoint_getConcreteTypeName :: (self: *PxRackAndPinionJoint) -> *u8 #foreign libphysx;

PxGroupsMask_new_alloc :: () -> *PxGroupsMask #foreign libphysx;

PxGroupsMask_delete :: (self: *PxGroupsMask) -> void #foreign libphysx;

PxDefaultSimulationFilterShader :: (attributes0: u32, filterData0: PxFilterData, attributes1: u32, filterData1: PxFilterData, pairFlags: *u16, constantBlock: *void, constantBlockSize: u32) -> u16 #foreign libphysx "phys_PxDefaultSimulationFilterShader";

PxGetGroupCollisionFlag :: (group1: u16, group2: u16) -> bool #foreign libphysx "phys_PxGetGroupCollisionFlag";

PxSetGroupCollisionFlag :: (group1: u16, group2: u16, enable: bool) -> void #foreign libphysx "phys_PxSetGroupCollisionFlag";

PxGetGroup :: (actor: *PxActor) -> u16 #foreign libphysx "phys_PxGetGroup";

PxSetGroup :: (actor: *PxActor, collisionGroup: u16) -> void #foreign libphysx "phys_PxSetGroup";

PxGetFilterOps :: (op0: *s32, op1: *s32, op2: *s32) -> void #foreign libphysx "phys_PxGetFilterOps";

PxSetFilterOps :: (op0: *s32, op1: *s32, op2: *s32) -> void #foreign libphysx "phys_PxSetFilterOps";

PxGetFilterBool :: () -> bool #foreign libphysx "phys_PxGetFilterBool";

PxSetFilterBool :: (enable: bool) -> void #foreign libphysx "phys_PxSetFilterBool";

PxGetFilterConstants :: (c0: *PxGroupsMask, c1: *PxGroupsMask) -> void #foreign libphysx "phys_PxGetFilterConstants";

PxSetFilterConstants :: (c0: *PxGroupsMask, c1: *PxGroupsMask) -> void #foreign libphysx "phys_PxSetFilterConstants";

PxGetGroupsMask :: (actor: *PxActor) -> PxGroupsMask #foreign libphysx "phys_PxGetGroupsMask";

PxSetGroupsMask :: (actor: *PxActor, mask: *PxGroupsMask) -> void #foreign libphysx "phys_PxSetGroupsMask";

PxDefaultErrorCallback_new_alloc :: () -> *PxDefaultErrorCallback #foreign libphysx;

PxDefaultErrorCallback_delete :: (self: *PxDefaultErrorCallback) -> void #foreign libphysx;

PxDefaultErrorCallback_reportError :: (self: *PxDefaultErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign libphysx "PxDefaultErrorCallback_reportError_mut";

PxRigidActorExt_createExclusiveShape :: (actor: *PxRigidActor, geometry: *PxGeometry, materials: **PxMaterial, materialCount: u16, shapeFlags: u8) -> *PxShape #foreign libphysx;

PxRigidActorExt_createExclusiveShape :: (actor: *PxRigidActor, geometry: *PxGeometry, material: *PxMaterial, shapeFlags: u8) -> *PxShape #foreign libphysx "PxRigidActorExt_createExclusiveShape_1";

PxRigidActorExt_getRigidActorShapeLocalBoundsList :: (actor: *PxRigidActor, numBounds: *u32) -> *PxBounds3 #foreign libphysx;

PxRigidActorExt_createBVHFromActor :: (physics: *PxPhysics, actor: *PxRigidActor) -> *PxBVH #foreign libphysx;

PxMassProperties_new :: () -> PxMassProperties #foreign libphysx;

PxMassProperties_new :: (m: float, inertiaT: *PxMat33, com: *Vector3) -> PxMassProperties #foreign libphysx "PxMassProperties_new_1";

PxMassProperties_new :: (geometry: *PxGeometry) -> PxMassProperties #foreign libphysx "PxMassProperties_new_2";

PxMassProperties_translate :: (self: *PxMassProperties, t: *Vector3) -> void #foreign libphysx "PxMassProperties_translate_mut";

PxMassProperties_getMassSpaceInertia :: (inertia: *PxMat33, massFrame: *Quaternion) -> Vector3 #foreign libphysx;

PxMassProperties_translateInertia :: (inertia: *PxMat33, mass: float, t: *Vector3) -> PxMat33 #foreign libphysx;

PxMassProperties_rotateInertia :: (inertia: *PxMat33, q: *Quaternion) -> PxMat33 #foreign libphysx;

PxMassProperties_scaleInertia :: (inertia: *PxMat33, scaleRotation: *Quaternion, scale: *Vector3) -> PxMat33 #foreign libphysx;

PxMassProperties_sum :: (props: *PxMassProperties, transforms: *PxTransform, count: u32) -> PxMassProperties #foreign libphysx;

PxRigidBodyExt_updateMassAndInertia :: (body: *PxRigidBody, shapeDensities: *float, shapeDensityCount: u32, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign libphysx;

PxRigidBodyExt_updateMassAndInertia :: (body: *PxRigidBody, density: float, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign libphysx "PxRigidBodyExt_updateMassAndInertia_1";

PxRigidBodyExt_setMassAndUpdateInertia :: (body: *PxRigidBody, shapeMasses: *float, shapeMassCount: u32, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign libphysx;

PxRigidBodyExt_setMassAndUpdateInertia :: (body: *PxRigidBody, mass: float, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign libphysx "PxRigidBodyExt_setMassAndUpdateInertia_1";

PxRigidBodyExt_computeMassPropertiesFromShapes :: (shapes: **PxShape, shapeCount: u32) -> PxMassProperties #foreign libphysx;

PxRigidBodyExt_addForceAtPos :: (body: *PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign libphysx;

PxRigidBodyExt_addForceAtLocalPos :: (body: *PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign libphysx;

PxRigidBodyExt_addLocalForceAtPos :: (body: *PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign libphysx;

PxRigidBodyExt_addLocalForceAtLocalPos :: (body: *PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign libphysx;

PxRigidBodyExt_getVelocityAtPos :: (body: *PxRigidBody, pos: *Vector3) -> Vector3 #foreign libphysx;

PxRigidBodyExt_getLocalVelocityAtLocalPos :: (body: *PxRigidBody, pos: *Vector3) -> Vector3 #foreign libphysx;

PxRigidBodyExt_getVelocityAtOffset :: (body: *PxRigidBody, pos: *Vector3) -> Vector3 #foreign libphysx;

PxRigidBodyExt_computeVelocityDeltaFromImpulse :: (body: *PxRigidBody, impulsiveForce: *Vector3, impulsiveTorque: *Vector3, deltaLinearVelocity: *Vector3, deltaAngularVelocity: *Vector3) -> void #foreign libphysx;

PxRigidBodyExt_computeVelocityDeltaFromImpulse :: (body: *PxRigidBody, globalPose: *PxTransform, point: *Vector3, impulse: *Vector3, invMassScale: float, invInertiaScale: float, deltaLinearVelocity: *Vector3, deltaAngularVelocity: *Vector3) -> void #foreign libphysx "PxRigidBodyExt_computeVelocityDeltaFromImpulse_1";

PxRigidBodyExt_computeLinearAngularImpulse :: (body: *PxRigidBody, globalPose: *PxTransform, point: *Vector3, impulse: *Vector3, invMassScale: float, invInertiaScale: float, linearImpulse: *Vector3, angularImpulse: *Vector3) -> void #foreign libphysx;

PxRigidBodyExt_linearSweepSingle :: (body: *PxRigidBody, scene: *PxScene, unitDir: *Vector3, distance: float, outputFlags: u16, closestHit: *PxSweepHit, shapeIndex: *u32, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, inflation: float) -> bool #foreign libphysx;

PxRigidBodyExt_linearSweepMultiple :: (body: *PxRigidBody, scene: *PxScene, unitDir: *Vector3, distance: float, outputFlags: u16, touchHitBuffer: *PxSweepHit, touchHitShapeIndices: *u32, touchHitBufferSize: u32, block: *PxSweepHit, blockingShapeIndex: *s32, overflow: *bool, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, inflation: float) -> u32 #foreign libphysx;

PxShapeExt_getGlobalPose :: (shape: *PxShape, actor: *PxRigidActor) -> PxTransform #foreign libphysx;

PxShapeExt_raycast :: (shape: *PxShape, actor: *PxRigidActor, rayOrigin: *Vector3, rayDir: *Vector3, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *PxRaycastHit) -> u32 #foreign libphysx;

PxShapeExt_overlap :: (shape: *PxShape, actor: *PxRigidActor, otherGeom: *PxGeometry, otherGeomPose: *PxTransform) -> bool #foreign libphysx;

PxShapeExt_sweep :: (shape: *PxShape, actor: *PxRigidActor, unitDir: *Vector3, distance: float, otherGeom: *PxGeometry, otherGeomPose: *PxTransform, sweepHit: *PxSweepHit, hitFlags: u16) -> bool #foreign libphysx;

PxShapeExt_getWorldBounds :: (shape: *PxShape, actor: *PxRigidActor, inflation: float) -> PxBounds3 #foreign libphysx;

PxMeshOverlapUtil_new_alloc :: () -> *PxMeshOverlapUtil #foreign libphysx;

PxMeshOverlapUtil_delete :: (self: *PxMeshOverlapUtil) -> void #foreign libphysx;

PxMeshOverlapUtil_findOverlap :: (self: *PxMeshOverlapUtil, geom: *PxGeometry, geomPose: *PxTransform, meshGeom: *PxTriangleMeshGeometry, meshPose: *PxTransform) -> u32 #foreign libphysx "PxMeshOverlapUtil_findOverlap_mut";

PxMeshOverlapUtil_findOverlap_mut :: (self: *PxMeshOverlapUtil, geom: *PxGeometry, geomPose: *PxTransform, hfGeom: *PxHeightFieldGeometry, hfPose: *PxTransform) -> u32 #foreign libphysx "PxMeshOverlapUtil_findOverlap_mut_1";

PxMeshOverlapUtil_getResults :: (self: *PxMeshOverlapUtil) -> *u32 #foreign libphysx;

PxMeshOverlapUtil_getNbResults :: (self: *PxMeshOverlapUtil) -> u32 #foreign libphysx;

PxComputeTriangleMeshPenetration :: (direction: *Vector3, depth: *float, geom: *PxGeometry, geomPose: *PxTransform, meshGeom: *PxTriangleMeshGeometry, meshPose: *PxTransform, maxIter: u32, usedIter: *u32) -> bool #foreign libphysx "phys_PxComputeTriangleMeshPenetration";

PxComputeHeightFieldPenetration :: (direction: *Vector3, depth: *float, geom: *PxGeometry, geomPose: *PxTransform, heightFieldGeom: *PxHeightFieldGeometry, heightFieldPose: *PxTransform, maxIter: u32, usedIter: *u32) -> bool #foreign libphysx "phys_PxComputeHeightFieldPenetration";

PxXmlMiscParameter_new :: () -> PxXmlMiscParameter #foreign libphysx;

PxXmlMiscParameter_new :: (inUpVector: *Vector3, inScale: PxTolerancesScale) -> PxXmlMiscParameter #foreign libphysx "PxXmlMiscParameter_new_1";

PxSerialization_isSerializable :: (collection: *PxCollection, sr: *PxSerializationRegistry, externalReferences: *PxCollection) -> bool #foreign libphysx;

PxSerialization_complete :: (collection: *PxCollection, sr: *PxSerializationRegistry, exceptFor: *PxCollection, followJoints: bool) -> void #foreign libphysx;

PxSerialization_createSerialObjectIds :: (collection: *PxCollection, base: u64) -> void #foreign libphysx;

PxSerialization_createCollectionFromXml :: (inputData: *PxInputData, cooking: *PxCooking, sr: *PxSerializationRegistry, externalRefs: *PxCollection, stringTable: *PxStringTable, outArgs: *PxXmlMiscParameter) -> *PxCollection #foreign libphysx;

PxSerialization_createCollectionFromBinary :: (memBlock: *void, sr: *PxSerializationRegistry, externalRefs: *PxCollection) -> *PxCollection #foreign libphysx;

PxSerialization_serializeCollectionToXml :: (outputStream: *PxOutputStream, collection: *PxCollection, sr: *PxSerializationRegistry, cooking: *PxCooking, externalRefs: *PxCollection, inArgs: *PxXmlMiscParameter) -> bool #foreign libphysx;

PxSerialization_serializeCollectionToBinary :: (outputStream: *PxOutputStream, collection: *PxCollection, sr: *PxSerializationRegistry, externalRefs: *PxCollection, exportNames: bool) -> bool #foreign libphysx;

PxSerialization_createSerializationRegistry :: (physics: *PxPhysics) -> *PxSerializationRegistry #foreign libphysx;

PxDefaultCpuDispatcher_release :: (self: *PxDefaultCpuDispatcher) -> void #foreign libphysx "PxDefaultCpuDispatcher_release_mut";

PxDefaultCpuDispatcher_setRunProfiled :: (self: *PxDefaultCpuDispatcher, runProfiled: bool) -> void #foreign libphysx "PxDefaultCpuDispatcher_setRunProfiled_mut";

PxDefaultCpuDispatcher_getRunProfiled :: (self: *PxDefaultCpuDispatcher) -> bool #foreign libphysx;

PxDefaultCpuDispatcherCreate :: (numThreads: u32, affinityMasks: *u32, mode: s32, yieldProcessorCount: u32) -> *PxDefaultCpuDispatcher #foreign libphysx "phys_PxDefaultCpuDispatcherCreate";

PxBuildSmoothNormals :: (nbTris: u32, nbVerts: u32, verts: *Vector3, dFaces: *u32, wFaces: *u16, normals: *Vector3, flip: bool) -> bool #foreign libphysx "phys_PxBuildSmoothNormals";

PxCreateDynamic :: (sdk: *PxPhysics, transform: *PxTransform, geometry: *PxGeometry, material: *PxMaterial, density: float, shapeOffset: *PxTransform) -> *PxRigidDynamic #foreign libphysx "phys_PxCreateDynamic";

PxCreateDynamic_1 :: (sdk: *PxPhysics, transform: *PxTransform, shape: *PxShape, density: float) -> *PxRigidDynamic #foreign libphysx "phys_PxCreateDynamic_1";

PxCreateKinematic :: (sdk: *PxPhysics, transform: *PxTransform, geometry: *PxGeometry, material: *PxMaterial, density: float, shapeOffset: *PxTransform) -> *PxRigidDynamic #foreign libphysx "phys_PxCreateKinematic";

PxCreateKinematic_1 :: (sdk: *PxPhysics, transform: *PxTransform, shape: *PxShape, density: float) -> *PxRigidDynamic #foreign libphysx "phys_PxCreateKinematic_1";

PxCreateStatic :: (sdk: *PxPhysics, transform: *PxTransform, geometry: *PxGeometry, material: *PxMaterial, shapeOffset: *PxTransform) -> *PxRigidStatic #foreign libphysx "phys_PxCreateStatic";

PxCreateStatic_1 :: (sdk: *PxPhysics, transform: *PxTransform, shape: *PxShape) -> *PxRigidStatic #foreign libphysx "phys_PxCreateStatic_1";

PxCloneShape :: (physicsSDK: *PxPhysics, shape: *PxShape, isExclusive: bool) -> *PxShape #foreign libphysx "phys_PxCloneShape";

PxCloneStatic :: (physicsSDK: *PxPhysics, transform: *PxTransform, actor: *PxRigidActor) -> *PxRigidStatic #foreign libphysx "phys_PxCloneStatic";

PxCloneDynamic :: (physicsSDK: *PxPhysics, transform: *PxTransform, body: *PxRigidDynamic) -> *PxRigidDynamic #foreign libphysx "phys_PxCloneDynamic";

PxCreatePlane :: (sdk: *PxPhysics, plane: *Plane3, material: *PxMaterial) -> *PxRigidStatic #foreign libphysx "phys_PxCreatePlane";

PxScaleRigidActor :: (actor: *PxRigidActor, scale: float, scaleMassProps: bool) -> void #foreign libphysx "phys_PxScaleRigidActor";

PxStringTableExt_createStringTable :: (inAllocator: *PxAllocatorCallback) -> *PxStringTable #foreign libphysx;

PxBroadPhaseExt_createRegionsFromWorldBounds :: (regions: *PxBounds3, globalBounds: *PxBounds3, nbSubdiv: u32, upAxis: u32) -> u32 #foreign libphysx;

PxSceneQueryExt_raycastAny :: (scene: *PxScene, origin: *Vector3, unitDir: *Vector3, distance: float, hit: *PxQueryHit, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache) -> bool #foreign libphysx;

PxSceneQueryExt_raycastSingle :: (scene: *PxScene, origin: *Vector3, unitDir: *Vector3, distance: float, outputFlags: u16, hit: *PxRaycastHit, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache) -> bool #foreign libphysx;

PxSceneQueryExt_raycastMultiple :: (scene: *PxScene, origin: *Vector3, unitDir: *Vector3, distance: float, outputFlags: u16, hitBuffer: *PxRaycastHit, hitBufferSize: u32, blockingHit: *bool, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache) -> s32 #foreign libphysx;

PxSceneQueryExt_sweepAny :: (scene: *PxScene, geometry: *PxGeometry, pose: *PxTransform, unitDir: *Vector3, distance: float, queryFlags: u16, hit: *PxQueryHit, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, inflation: float) -> bool #foreign libphysx;

PxSceneQueryExt_sweepSingle :: (scene: *PxScene, geometry: *PxGeometry, pose: *PxTransform, unitDir: *Vector3, distance: float, outputFlags: u16, hit: *PxSweepHit, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, inflation: float) -> bool #foreign libphysx;

PxSceneQueryExt_sweepMultiple :: (scene: *PxScene, geometry: *PxGeometry, pose: *PxTransform, unitDir: *Vector3, distance: float, outputFlags: u16, hitBuffer: *PxSweepHit, hitBufferSize: u32, blockingHit: *bool, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback, cache: *PxQueryCache, inflation: float) -> s32 #foreign libphysx;

PxSceneQueryExt_overlapMultiple :: (scene: *PxScene, geometry: *PxGeometry, pose: *PxTransform, hitBuffer: *PxOverlapHit, hitBufferSize: u32, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback) -> s32 #foreign libphysx;

PxSceneQueryExt_overlapAny :: (scene: *PxScene, geometry: *PxGeometry, pose: *PxTransform, hit: *PxOverlapHit, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback) -> bool #foreign libphysx;

PxBatchQueryExt_release :: (self: *PxBatchQueryExt) -> void #foreign libphysx "PxBatchQueryExt_release_mut";

PxBatchQueryExt_raycast :: (self: *PxBatchQueryExt, origin: *Vector3, unitDir: *Vector3, distance: float, maxNbTouches: u16, hitFlags: u16, filterData: *PxQueryFilterData, cache: *PxQueryCache) -> *PxRaycastBuffer #foreign libphysx "PxBatchQueryExt_raycast_mut";

PxBatchQueryExt_sweep :: (self: *PxBatchQueryExt, geometry: *PxGeometry, pose: *PxTransform, unitDir: *Vector3, distance: float, maxNbTouches: u16, hitFlags: u16, filterData: *PxQueryFilterData, cache: *PxQueryCache, inflation: float) -> *PxSweepBuffer #foreign libphysx "PxBatchQueryExt_sweep_mut";

PxBatchQueryExt_overlap :: (self: *PxBatchQueryExt, geometry: *PxGeometry, pose: *PxTransform, maxNbTouches: u16, filterData: *PxQueryFilterData, cache: *PxQueryCache) -> *PxOverlapBuffer #foreign libphysx "PxBatchQueryExt_overlap_mut";

PxBatchQueryExt_execute :: (self: *PxBatchQueryExt) -> void #foreign libphysx "PxBatchQueryExt_execute_mut";

PxCreateBatchQueryExt :: (scene: *PxScene, queryFilterCallback: *PxQueryFilterCallback, maxNbRaycasts: u32, maxNbRaycastTouches: u32, maxNbSweeps: u32, maxNbSweepTouches: u32, maxNbOverlaps: u32, maxNbOverlapTouches: u32) -> *PxBatchQueryExt #foreign libphysx "phys_PxCreateBatchQueryExt";

PxCreateBatchQueryExt_1 :: (scene: *PxScene, queryFilterCallback: *PxQueryFilterCallback, raycastBuffers: *PxRaycastBuffer, maxNbRaycasts: u32, raycastTouches: *PxRaycastHit, maxNbRaycastTouches: u32, sweepBuffers: *PxSweepBuffer, maxNbSweeps: u32, sweepTouches: *PxSweepHit, maxNbSweepTouches: u32, overlapBuffers: *PxOverlapBuffer, maxNbOverlaps: u32, overlapTouches: *PxOverlapHit, maxNbOverlapTouches: u32) -> *PxBatchQueryExt #foreign libphysx "phys_PxCreateBatchQueryExt_1";

PxCreateExternalSceneQuerySystem :: (desc: *PxSceneQueryDesc, contextID: u64) -> *PxSceneQuerySystem #foreign libphysx "phys_PxCreateExternalSceneQuerySystem";

PxCustomSceneQuerySystem_addPruner :: (self: *PxCustomSceneQuerySystem, primaryType: s32, secondaryType: s32, preallocated: u32) -> u32 #foreign libphysx "PxCustomSceneQuerySystem_addPruner_mut";

PxCustomSceneQuerySystem_startCustomBuildstep :: (self: *PxCustomSceneQuerySystem) -> u32 #foreign libphysx "PxCustomSceneQuerySystem_startCustomBuildstep_mut";

PxCustomSceneQuerySystem_customBuildstep :: (self: *PxCustomSceneQuerySystem, index: u32) -> void #foreign libphysx "PxCustomSceneQuerySystem_customBuildstep_mut";

PxCustomSceneQuerySystem_finishCustomBuildstep :: (self: *PxCustomSceneQuerySystem) -> void #foreign libphysx "PxCustomSceneQuerySystem_finishCustomBuildstep_mut";

PxCustomSceneQuerySystemAdapter_delete :: (self: *PxCustomSceneQuerySystemAdapter) -> void #foreign libphysx;

PxCustomSceneQuerySystemAdapter_getPrunerIndex :: (self: *PxCustomSceneQuerySystemAdapter, actor: *PxRigidActor, shape: *PxShape) -> u32 #foreign libphysx;

PxCustomSceneQuerySystemAdapter_processPruner :: (self: *PxCustomSceneQuerySystemAdapter, prunerIndex: u32, context_pod: *PxQueryThreadContext, filterData: *PxQueryFilterData, filterCall: *PxQueryFilterCallback) -> bool #foreign libphysx;

PxCreateCustomSceneQuerySystem :: (sceneQueryUpdateMode: s32, contextID: u64, adapter: *PxCustomSceneQuerySystemAdapter, usesTreeOfPruners: bool) -> *PxCustomSceneQuerySystem #foreign libphysx "phys_PxCreateCustomSceneQuerySystem";

PxFindFaceIndex :: (convexGeom: *PxConvexMeshGeometry, geomPose: *PxTransform, impactPos: *Vector3, unitDir: *Vector3) -> u32 #foreign libphysx "phys_PxFindFaceIndex";

PxPoissonSampler_setSamplingRadius :: (self: *PxPoissonSampler, samplingRadius: float) -> bool #foreign libphysx "PxPoissonSampler_setSamplingRadius_mut";

PxPoissonSampler_addSamplesInSphere :: (self: *PxPoissonSampler, sphereCenter: *Vector3, sphereRadius: float, createVolumeSamples: bool) -> void #foreign libphysx "PxPoissonSampler_addSamplesInSphere_mut";

PxPoissonSampler_addSamplesInBox :: (self: *PxPoissonSampler, axisAlignedBox: *PxBounds3, boxOrientation: *Quaternion, createVolumeSamples: bool) -> void #foreign libphysx "PxPoissonSampler_addSamplesInBox_mut";

PxPoissonSampler_delete :: (self: *PxPoissonSampler) -> void #foreign libphysx;

PxCreateShapeSampler :: (geometry: *PxGeometry, transform: *PxTransform, worldBounds: *PxBounds3, initialSamplingRadius: float, numSampleAttemptsAroundPoint: s32) -> *PxPoissonSampler #foreign libphysx "phys_PxCreateShapeSampler";

PxTriangleMeshPoissonSampler_isPointInTriangleMesh :: (self: *PxTriangleMeshPoissonSampler, p: *Vector3) -> bool #foreign libphysx "PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut";

PxTriangleMeshPoissonSampler_delete :: (self: *PxTriangleMeshPoissonSampler) -> void #foreign libphysx;

PxCreateTriangleMeshSampler :: (triangles: *u32, numTriangles: u32, vertices: *Vector3, numVertices: u32, initialSamplingRadius: float, numSampleAttemptsAroundPoint: s32) -> *PxTriangleMeshPoissonSampler #foreign libphysx "phys_PxCreateTriangleMeshSampler";

PxTetrahedronMeshExt_findTetrahedronContainingPoint :: (mesh: *PxTetrahedronMesh, point: *Vector3, bary: *Vector4, tolerance: float) -> s32 #foreign libphysx;

PxTetrahedronMeshExt_findTetrahedronClosestToPoint :: (mesh: *PxTetrahedronMesh, point: *Vector3, bary: *Vector4) -> s32 #foreign libphysx;

PxInitExtensions :: (physics: *PxPhysics, pvd: *PxPvd) -> bool #foreign libphysx "phys_PxInitExtensions";

PxCloseExtensions :: () -> void #foreign libphysx "phys_PxCloseExtensions";

PxRepXObject_new :: (inTypeName: *u8, inSerializable: *void, inId: u64) -> PxRepXObject #foreign libphysx;

PxRepXObject_isValid :: (self: *PxRepXObject) -> bool #foreign libphysx;

PxRepXInstantiationArgs_new :: (inPhysics: *PxPhysics, inCooking: *PxCooking, inStringTable: *PxStringTable) -> PxRepXInstantiationArgs #foreign libphysx;

PxRepXSerializer_getTypeName :: (self: *PxRepXSerializer) -> *u8 #foreign libphysx "PxRepXSerializer_getTypeName_mut";

PxRepXSerializer_objectToFile :: (self: *PxRepXSerializer, inLiveObject: *PxRepXObject, inCollection: *PxCollection, inWriter: *XmlWriter, inTempBuffer: *MemoryBuffer, inArgs: *PxRepXInstantiationArgs) -> void #foreign libphysx "PxRepXSerializer_objectToFile_mut";

PxRepXSerializer_fileToObject :: (self: *PxRepXSerializer, inReader: *XmlReader, inAllocator: *XmlMemoryAllocator, inArgs: *PxRepXInstantiationArgs, inCollection: *PxCollection) -> PxRepXObject #foreign libphysx "PxRepXSerializer_fileToObject_mut";

PxPvd_connect :: (self: *PxPvd, transport: *PxPvdTransport, flags: u8) -> bool #foreign libphysx "PxPvd_connect_mut";

PxPvd_disconnect :: (self: *PxPvd) -> void #foreign libphysx "PxPvd_disconnect_mut";

PxPvd_isConnected :: (self: *PxPvd, useCachedStatus: bool) -> bool #foreign libphysx "PxPvd_isConnected_mut";

PxPvd_getTransport :: (self: *PxPvd) -> *PxPvdTransport #foreign libphysx "PxPvd_getTransport_mut";

PxPvd_getInstrumentationFlags :: (self: *PxPvd) -> u8 #foreign libphysx "PxPvd_getInstrumentationFlags_mut";

PxPvd_release :: (self: *PxPvd) -> void #foreign libphysx "PxPvd_release_mut";

PxCreatePvd :: (foundation: *PxFoundation) -> *PxPvd #foreign libphysx "phys_PxCreatePvd";

PxPvdTransport_connect :: (self: *PxPvdTransport) -> bool #foreign libphysx "PxPvdTransport_connect_mut";

PxPvdTransport_disconnect :: (self: *PxPvdTransport) -> void #foreign libphysx "PxPvdTransport_disconnect_mut";

PxPvdTransport_isConnected :: (self: *PxPvdTransport) -> bool #foreign libphysx "PxPvdTransport_isConnected_mut";

PxPvdTransport_write :: (self: *PxPvdTransport, inBytes: *u8, inLength: u32) -> bool #foreign libphysx "PxPvdTransport_write_mut";

PxPvdTransport_lock :: (self: *PxPvdTransport) -> *PxPvdTransport #foreign libphysx "PxPvdTransport_lock_mut";

PxPvdTransport_unlock :: (self: *PxPvdTransport) -> void #foreign libphysx "PxPvdTransport_unlock_mut";

PxPvdTransport_flush :: (self: *PxPvdTransport) -> void #foreign libphysx "PxPvdTransport_flush_mut";

PxPvdTransport_getWrittenDataSize :: (self: *PxPvdTransport) -> u64 #foreign libphysx "PxPvdTransport_getWrittenDataSize_mut";

PxPvdTransport_release :: (self: *PxPvdTransport) -> void #foreign libphysx "PxPvdTransport_release_mut";

PxDefaultPvdSocketTransportCreate :: (host: *u8, port: s32, timeoutInMilliseconds: u32) -> *PxPvdTransport #foreign libphysx "phys_PxDefaultPvdSocketTransportCreate";

PxDefaultPvdFileTransportCreate :: (name: *u8) -> *PxPvdTransport #foreign libphysx "phys_PxDefaultPvdFileTransportCreate";

gAllocator: PxDefaultAllocator #elsewhere libphysx;
gErrorCallback: PxDefaultErrorCallback #elsewhere libphysx;

CollisionCallback :: #type (unknown0: *void, unknown1: *PxContactPairHeader, unknown2: *PxContactPair, unknown3: u32) -> void #c_call;

SimulationFilterShader :: #type (attributes0: *u32, filterData0: *PxFilterData, attributes1: *u32, filterData1: *PxFilterData, pairFlags: *PxPairFlags) -> u16 #c_call;

FilterShaderHandle :: struct {
    shader: SimulationFilterShader;
}

TriggerCallback :: #type (unknown0: *void, unknown1: *PxTriggerPair, unknown2: u32) -> void #c_call;
ConstraintBreakCallback :: #type (unknown0: *void, unknown1: *PxConstraintInfo, unknown2: u32) -> void #c_call;
WakeSleepCallback :: #type (unknown0: *void, unknown1: **PxActor, unknown2: u32, unknown3: bool) -> void #c_call;
AdvanceCallback :: #type (unknown0: *void, unknown1: **PxRigidBody, unknown2: *PxTransform, unknown3: u32) -> void #c_call;

SimulationEventCallbackInfo :: struct {
    // Callback for collision events.
    collisionCallback:       CollisionCallback = null;
    collisionUserData:       *void = null;

    // Callback for trigger shape events (an object entered or left a trigger shape).
    triggerCallback:         TriggerCallback = null;
    triggerUserData:         *void = null;

    // Callback for when a constraint breaks (such as a joint with a force limit)
    constraintBreakCallback: ConstraintBreakCallback = null;
    constraintBreakUserData: *void = null;

    // Callback for when an object falls asleep or is awoken.
    wakeSleepCallback:       WakeSleepCallback = null;
    wakeSleepUserData:       *void = null;

    // Callback to get the next pose early for objects (if flagged with eENABLE_POSE_INTEGRATION_PREVIEW).
    advanceCallback:         AdvanceCallback = null;
    advanceUserData:         *void = null;
}

RaycastHitCallback :: #type (actor: *PxRigidActor, filterData: *PxFilterData, shape: *PxShape, hitFlags: u32, userData: *void) -> s32 #c_call;
PostFilterCallback :: #type (filterData: *PxFilterData, hit: *PxQueryHit, userData: *void) -> s32 #c_call;

RaycastHitProcessTouchesCallback :: #type (buffer: *PxRaycastHit, nbHits: u32, userdata: *void) -> bool #c_call;
SweepHitProcessTouchesCallback :: #type (buffer: *PxSweepHit, nbHits: u32, userdata: *void) -> bool #c_call;
OverlapHitProcessTouchesCallback :: #type (buffer: *PxOverlapHit, nbHits: u32, userdata: *void) -> bool #c_call;
HitFinalizeQueryCallback :: #type (userdata: *void) -> void #c_call;

AllocCallback :: #type (size: u64, typeName: *u8, filename: *u8, line: s32, userdata: *void) -> *void #c_call;
DeallocCallback :: #type (ptr: *void, userdata: *void) -> void #c_call;

ZoneStartCallback :: #type (typeName: *u8, detached: bool, _context_pod: u64, userdata: *void) -> *void #c_call;
ZoneEndCallback :: #type (profilerData: *void, typeName: *u8, detached: bool, _context_pod: u64, userdata: *void) -> void #c_call;

ErrorCallback :: #type (code: s32, message: *u8, file: *u8, line: s32, userdata: *void) -> void #c_call;

AssertHandler :: #type (expr: *u8, file: *u8, line: s32, should_ignore: *bool, userdata: *void) -> void #c_call;

ShapeHitCallback :: #type (unknown0: *void, unknown1: *PxControllerShapeHit) -> void #c_call;
ControllerHitCallback :: #type (unknown0: *void, unknown1: *PxControllersHit) -> void #c_call;
ObstacleHitCallback :: #type (unknown0: *void, unknown1: *PxControllerObstacleHit) -> void #c_call;

UserControllerHitReportInfo :: struct {
    shapeHitCallback:      ShapeHitCallback = null;
    shapeHitUserData:      *void = null;
    controllerHitCallback: ControllerHitCallback = null;
    controllerHitUserData: *void = null;
    obstacleHitCallback:   ObstacleHitCallback = null;
    obstacleHitUserData:   *void = null;
}

ShapeBehaviorCallback :: #type (data: *void, shape: *PxShape, actor: *PxActor) -> PxControllerBehaviorFlags #c_call;
CharacterBehaviorCallback :: #type (data: *void, controller: *PxController) -> PxControllerBehaviorFlags #c_call;
ObstacleBehaviorCallback :: #type (data: *void, obstacle: *PxObstacle) -> PxControllerBehaviorFlags #c_call;

ControllerBehaviorCallbackInfo :: struct {
    shapeBehaviorCallback:      ShapeBehaviorCallback = null;
    shapeBehaviorUserData:      *void = null;
    controllerBehaviorCallback: CharacterBehaviorCallback = null;
    controllerBehaviorUserData: *void = null;
    obstacleBehaviorCallback:   ObstacleBehaviorCallback = null;
    obstacleBehaviorUserData:   *void = null;
}

OnContactModify :: #type (data: *void, pairs: *PxContactModifyPair, count: u32) -> void #c_call;

ContactModifyCallbackInfo :: struct {
    onContactModify:         OnContactModify = null;
    onContactModifyUserData: *void = null;
}

create_foundation :: () -> *PxFoundation #foreign libphysx "physx_create_foundation";

create_foundation_with_alloc :: (allocator: *PxAllocatorCallback) -> *PxFoundation #foreign libphysx "physx_create_foundation_with_alloc";

// fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
// linking against the raw interface
get_default_allocator :: () -> *PxAllocatorCallback #foreign libphysx;

// fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
// linking against the raw interface
get_default_error_callback :: () -> *PxErrorCallback #foreign libphysx;

create_physics :: (foundation: *PxFoundation) -> *PxPhysics #foreign libphysx "physx_create_physics";

create_raycast_filter_callback :: (actor_to_ignore: *PxRigidActor) -> *PxQueryFilterCallback #foreign libphysx;

destroy_raycast_filter_callback :: (callback: *PxQueryFilterCallback) -> void #foreign libphysx;

create_raycast_filter_callback_func :: (callback: RaycastHitCallback, userData: *void) -> *PxQueryFilterCallback #foreign libphysx;

destroy_raycast_filter_callback_func :: (callback: *PxQueryFilterCallback) -> void #foreign libphysx;

create_pre_and_post_raycast_filter_callback_func :: (preFilter: RaycastHitCallback, postFilter: PostFilterCallback, userData: *void) -> *PxQueryFilterCallback #foreign libphysx;

destroy_pre_and_post_raycast_filter_callback_func :: (callback: *PxQueryFilterCallback) -> void #foreign libphysx;

create_raycast_buffer :: () -> *PxRaycastCallback #foreign libphysx;

create_sweep_buffer :: () -> *PxSweepCallback #foreign libphysx;

create_overlap_buffer :: () -> *PxOverlapCallback #foreign libphysx;

create_raycast_callback :: (process_touches_callback: RaycastHitProcessTouchesCallback, finalize_query_callback: HitFinalizeQueryCallback, touchesBuffer: *PxRaycastHit, numTouches: u32, userdata: *void) -> *PxRaycastCallback #foreign libphysx;

delete_raycast_callback :: (callback: *PxRaycastCallback) -> void #foreign libphysx;

delete_sweep_callback :: (callback: *PxSweepCallback) -> void #foreign libphysx;

delete_overlap_callback :: (callback: *PxOverlapCallback) -> void #foreign libphysx;

create_sweep_callback :: (process_touches_callback: SweepHitProcessTouchesCallback, finalize_query_callback: HitFinalizeQueryCallback, touchesBuffer: *PxSweepHit, numTouches: u32, userdata: *void) -> *PxSweepCallback #foreign libphysx;

create_overlap_callback :: (process_touches_callback: OverlapHitProcessTouchesCallback, finalize_query_callback: HitFinalizeQueryCallback, touchesBuffer: *PxOverlapHit, numTouches: u32, userdata: *void) -> *PxOverlapCallback #foreign libphysx;

create_alloc_callback :: (alloc_callback: AllocCallback, dealloc_callback: DeallocCallback, userdata: *void) -> *PxAllocatorCallback #foreign libphysx;

get_alloc_callback_user_data :: (allocator: *PxAllocatorCallback) -> *void #foreign libphysx;

create_profiler_callback :: (zone_start_callback: ZoneStartCallback, zone_end_callback: ZoneEndCallback, userdata: *void) -> *PxProfilerCallback #foreign libphysx;

create_error_callback :: (error_callback: ErrorCallback, userdata: *void) -> *PxErrorCallback #foreign libphysx;

create_assert_handler :: (on_assert: AssertHandler, userdata: *void) -> *PxAssertHandler #foreign libphysx;

// simulation event
create_simulation_event_callbacks :: (callbacks: *SimulationEventCallbackInfo) -> *PxSimulationEventCallback #foreign libphysx;

get_simulation_event_info :: (callback: *PxSimulationEventCallback) -> *SimulationEventCallbackInfo #foreign libphysx;

destroy_simulation_event_callbacks :: (callback: *PxSimulationEventCallback) -> void #foreign libphysx;

// hit report
create_user_controller_hit_report :: (callbacks: *UserControllerHitReportInfo) -> *PxUserControllerHitReport #foreign libphysx;

get_user_controller_hit_info :: (callback: *PxUserControllerHitReport) -> *UserControllerHitReportInfo #foreign libphysx;

destroy_user_controller_hit_report :: (callback: *PxUserControllerHitReport) -> void #foreign libphysx;

// controller behavior
create_controller_behavior_callbacks :: (callbacks: *ControllerBehaviorCallbackInfo) -> *PxControllerBehaviorCallback #foreign libphysx;

get_controller_behavior_info :: (callback: *PxControllerBehaviorCallback) -> *ControllerBehaviorCallbackInfo #foreign libphysx;

destroy_controller_behavior_callbacks :: (callback: *PxControllerBehaviorCallback) -> void #foreign libphysx;

// contact modify
create_contact_modify_callbacks :: (callbacks: *ContactModifyCallbackInfo) -> *PxContactModifyCallback #foreign libphysx;

get_contact_modify_info :: (callback: *PxContactModifyCallback) -> *ContactModifyCallbackInfo #foreign libphysx;

destroy_contact_modify_callbacks :: (callback: *PxContactModifyCallback) -> void #foreign libphysx;

// ccd contact modify
create_ccd_contact_modify_callbacks :: (callbacks: *ContactModifyCallbackInfo) -> *PxCCDContactModifyCallback #foreign libphysx;

get_ccd_contact_modify_info :: (callback: *PxCCDContactModifyCallback) -> *ContactModifyCallbackInfo #foreign libphysx;

destroy_ccd_contact_modify_callbacks :: (callback: *PxCCDContactModifyCallback) -> void #foreign libphysx;

// filter shader
get_default_simulation_filter_shader :: () -> SimulationFilterShader #foreign libphysx;

set_default_filter_shader :: (desc: *PxSceneDesc) -> void #foreign libphysx;

create_custom_filter_shader :: (filter: SimulationFilterShader) -> *FilterShaderHandle #foreign libphysx;

destroy_custom_filter_shader :: (filterHandle: *FilterShaderHandle) -> void #foreign libphysx;

set_custom_filter_shader :: (desc: *PxSceneDesc, filterHandle: *FilterShaderHandle) -> void #foreign libphysx;

set_custom_filter_shader_with_default :: (desc: *PxSceneDesc, filterHandle: *FilterShaderHandle) -> void #foreign libphysx;

// Not generated, used only for testing and examples!
PxAssertHandler_opCall :: (self: *PxErrorCallback, expr: *u8, file: *u8, line: s32, ignore: *bool) -> void #foreign libphysx "PxAssertHandler_opCall_mut";

#scope_file

libphysx :: #library "linux/libphysx";
#import "Math";
#import "Socket";
