
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
