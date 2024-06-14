//
// This file was auto-generated using the following command:
//
// jai ../ generate.jai
//
physx_PxPairFlags :: enum_flags u16 {
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
    ContactDefault :: SolveContact | DetectDiscreteContact;
    TriggerDefault :: NotifyTouchFound | NotifyTouchLost | DetectDiscreteContact;
}

physx_PxDefaultCpuDispatcherWaitForWorkMode :: enum s32 {
    WaitForWork :: 0;
    YieldThread :: 1;
    YieldProcessor :: 2;
}

physx_PxConvexMeshCookingType :: enum s32 {
    Quickhull :: 0;
}

physx_PxConvexFlag :: enum_flags s32 {
    /// Denotes the use of 16-bit vertex indices in PxConvexMeshDesc::triangles or PxConvexMeshDesc::polygons.
    /// (otherwise, 32-bit indices are assumed)
    E16BitIndices :: 1;
    /// Automatically recomputes the hull from the vertices. If this flag is not set, you must provide the entire geometry manually.
    ///
    /// There are two different algorithms for hull computation, please see PxConvexMeshCookingType.
    ComputeConvex :: 2;
    /// Checks and removes almost zero-area triangles during convex hull computation.
    /// The rejected area size is specified in PxCookingParams::areaTestEpsilon
    ///
    /// This flag is only used in combination with eCOMPUTE_CONVEX.
    CheckZeroAreaTriangles :: 4;
    /// Quantizes the input vertices using the k-means clustering
    ///
    /// The input vertices are quantized to PxConvexMeshDesc::quantizedCount
    /// see http://en.wikipedia.org/wiki/K-means_clustering
    QuantizeInput :: 8;
    /// Disables the convex mesh validation to speed-up hull creation. Please use separate validation
    /// function in checked/debug builds. Creating a convex mesh with invalid input data without prior validation
    /// may result in undefined behavior.
    DisableMeshValidation :: 16;
    /// Enables plane shifting vertex limit algorithm.
    ///
    /// Plane shifting is an alternative algorithm for the case when the computed hull has more vertices
    /// than the specified vertex limit.
    ///
    /// The default algorithm computes the full hull, and an OBB around the input vertices. This OBB is then sliced
    /// with the hull planes until the vertex limit is reached.The default algorithm requires the vertex limit
    /// to be set to at least 8, and typically produces results that are much better quality than are produced
    /// by plane shifting.
    ///
    /// When plane shifting is enabled, the hull computation stops when vertex limit is reached. The hull planes
    /// are then shifted to contain all input vertices, and the new plane intersection points are then used to
    /// generate the final hull with the given vertex limit.Plane shifting may produce sharp edges to vertices
    /// very far away from the input cloud, and does not guarantee that all input vertices are inside the resulting
    /// hull.However, it can be used with a vertex limit as low as 4.
    PlaneShifting :: 32;
    /// Inertia tensor computation is faster using SIMD code, but the precision is lower, which may result
    /// in incorrect inertia for very thin hulls.
    FastInertiaComputation :: 64;
    /// Convex hulls are created with respect to GPU simulation limitations. Vertex limit and polygon limit
    /// is set to 64 and vertex limit per face is internally set to 32.
    ///
    /// Can be used only with eCOMPUTE_CONVEX flag.
    GpuCompatible :: 128;
    /// Convex hull input vertices are shifted to be around origin to provide better computation stability.
    /// It is recommended to provide input vertices around the origin, otherwise use this flag to improve
    /// numerical stability.
    ///
    /// Is used only with eCOMPUTE_CONVEX flag.
    ShiftVertices :: 256;
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



NDEBUG :: 1;
_MSC_VER :: 1939;
physx_PxMat34 :: struct {}

physx_PxAllocatorCallback :: struct {
    vtable_: *void;
}

physx_PxAssertHandler :: struct {
    vtable_: *void;
}

physx_PxFoundation :: struct {
    vtable_: *void;
}

physx_PxAllocator :: struct {
    structgen_pad0: [1] u8;
}

physx_PxRawAllocator :: struct {
    structgen_pad0: [1] u8;
}

physx_PxVirtualAllocatorCallback :: struct {
    vtable_: *void;
}

physx_PxVirtualAllocator :: struct {
    structgen_pad0: [16] u8;
}

physx_PxUserAllocated :: struct {
    structgen_pad0: [1] u8;
}

physx_PxTempAllocatorChunk :: union {
    mNext:  *physx_PxTempAllocatorChunk;
    mIndex: u32;
    mPad:   [16] u8;
}

physx_PxTempAllocator :: struct {
    structgen_pad0: [1] u8;
}

physx_PxLogTwo :: struct {}
physx_PxUnConst :: struct {}
physx_PxBitAndByte :: struct {
    structgen_pad0: [1] u8;
}

physx_PxBitMap :: struct {
    structgen_pad0: [16] u8;
}

physx_PxVec3Padded :: struct {
    x:       float;
    y:       float;
    z:       float;
    padding: u32;
}

physx_PxTransform :: struct {
    q: Quaternion;
    p: Vector3;
}

physx_PxTransformPadded :: struct {
    transform: physx_PxTransform;
    padding:   u32;
}

physx_PxMat33 :: struct {
    column0: Vector3;
    column1: Vector3;
    column2: Vector3;
}

physx_PxBounds3 :: struct {
    minimum: Vector3;
    maximum: Vector3;
}

physx_PxErrorCallback :: struct {
    vtable_: *void;
}

physx_PxAllocationListener :: struct {
    vtable_: *void;
}

physx_PxBroadcastingAllocator :: struct {
    structgen_pad0: [176] u8;
}

physx_PxBroadcastingErrorCallback :: struct {
    structgen_pad0: [160] u8;
}

physx_PxHash :: struct {}
physx_PxInputStream :: struct {
    vtable_: *void;
}

physx_PxInputData :: struct {
    vtable_: *void;
}

physx_PxOutputStream :: struct {
    vtable_: *void;
}

physx_PxMat44 :: struct {
    column0: Vector4;
    column1: Vector4;
    column2: Vector4;
    column3: Vector4;
}

physx_Interpolation :: struct {
    structgen_pad0: [1] u8;
}

physx_PxMutexImpl :: struct {
    structgen_pad0: [1] u8;
}

physx_PxReadWriteLock :: struct {
    structgen_pad0: [8] u8;
}

physx_PxProfilerCallback :: struct {
    vtable_: *void;
}

physx_PxProfileScoped :: struct {
    mCallback:      *physx_PxProfilerCallback;
    mEventName:     *u8;
    mProfilerData:  *void;
    mContextId:     u64;
    mDetached:      bool;
    structgen_pad0: [7] u8;
}

physx_PxSListEntry :: struct {
    structgen_pad0: [16] u8;
}

physx_PxSListImpl :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSyncImpl :: struct {
    structgen_pad0: [1] u8;
}

physx_PxRunnable :: struct {
    vtable_: *void;
}

physx_PxCounterFrequencyToTensOfNanos :: struct {
    mNumerator:   u64;
    mDenominator: u64;
}

physx_PxTime :: struct {
    structgen_pad0: [8] u8;
}

physx_PxStridedData :: struct {
    stride:         u32;
    structgen_pad0: [4] u8;
    data:           *void;
}

physx_PxBoundedData :: struct {
    stride:         u32;
    structgen_pad0: [4] u8;
    data:           *void;
    count:          u32;
    structgen_pad1: [4] u8;
}

physx_PxDebugPoint :: struct {
    pos:   Vector3;
    color: u32;
}

physx_PxDebugLine :: struct {
    pos0:   Vector3;
    color0: u32;
    pos1:   Vector3;
    color1: u32;
}

physx_PxDebugTriangle :: struct {
    pos0:   Vector3;
    color0: u32;
    pos1:   Vector3;
    color1: u32;
    pos2:   Vector3;
    color2: u32;
}

physx_PxDebugText :: struct {
    position:       Vector3;
    size:           float;
    color:          u32;
    structgen_pad0: [4] u8;
    _string:        *u8;
}

physx_PxRenderBuffer :: struct {
    vtable_: *void;
}

physx_PxProcessPxBaseCallback :: struct {
    vtable_: *void;
}

physx_PxSerializationContext :: struct {
    vtable_: *void;
}

physx_PxDeserializationContext :: struct {
    structgen_pad0: [16] u8;
}

physx_PxSerializationRegistry :: struct {
    vtable_: *void;
}

physx_PxCollection :: struct {
    vtable_: *void;
}

physx_PxTypeInfo :: struct {}

physx_PxFEMSoftBodyMaterial :: struct {}
physx_PxFEMClothMaterial :: struct {}
physx_PxPBDMaterial :: struct {}
physx_PxFLIPMaterial :: struct {}
physx_PxMPMMaterial :: struct {}
physx_PxCustomMaterial :: struct {}

physx_PxBVH33TriangleMesh :: struct {}

physx_PxParticleSystem :: struct {}
physx_PxPBDParticleSystem :: struct {}
physx_PxFLIPParticleSystem :: struct {}
physx_PxMPMParticleSystem :: struct {}
physx_PxCustomParticleSystem :: struct {}
physx_PxSoftBody :: struct {}
physx_PxFEMCloth :: struct {}
physx_PxHairSystem :: struct {}
physx_PxParticleBuffer :: struct {}
physx_PxParticleAndDiffuseBuffer :: struct {}
physx_PxParticleClothBuffer :: struct {}
physx_PxParticleRigidBuffer :: struct {}
physx_PxBase :: struct {
    structgen_pad0: [16] u8;
}

physx_PxRefCounted :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTolerancesScale :: struct {
    length: float;
    speed:  float;
}

physx_PxStringTable :: struct {
    vtable_: *void;
}

physx_PxSerializer :: struct {
    vtable_: *void;
}

physx_PxMetaDataEntry :: struct {
    type:       *u8;
    name:       *u8;
    offset:     u32;
    size:       u32;
    count:      u32;
    offsetSize: u32;
    flags:      u32;
    alignment:  u32;
}

physx_PxInsertionCallback :: struct {
    vtable_: *void;
}

physx_PxTaskManager :: struct {
    vtable_: *void;
}

physx_PxCpuDispatcher :: struct {
    vtable_: *void;
}

physx_PxBaseTask :: struct {
    structgen_pad0: [24] u8;
}

physx_PxTask :: struct {
    structgen_pad0: [32] u8;
}

physx_PxLightCpuTask :: struct {
    structgen_pad0: [40] u8;
}

physx_PxGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

physx_PxBoxGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    halfExtents:    Vector3;
}

physx_PxBVHRaycastCallback :: struct {
    vtable_: *void;
}

physx_PxBVHOverlapCallback :: struct {
    vtable_: *void;
}

physx_PxBVHTraversalCallback :: struct {
    vtable_: *void;
}

physx_PxBVH :: struct {
    structgen_pad0: [16] u8;
}

physx_PxCapsuleGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    radius:         float;
    halfHeight:     float;
}

physx_PxHullPolygon :: struct {
    mPlane:     [4] float;
    mNbVerts:   u16;
    mIndexBase: u16;
}

physx_PxConvexMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxMeshScale :: struct {
    scale:    Vector3;
    rotation: Quaternion;
}

physx_PxConvexMeshGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    scale:          physx_PxMeshScale;
    structgen_pad1: [4] u8;
    convexMesh:     *physx_PxConvexMesh;
    meshFlags:      u8;
    structgen_pad2: [7] u8;
}

physx_PxSphereGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    radius:         float;
}

physx_PxPlaneGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

physx_PxTriangleMeshGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    scale:          physx_PxMeshScale;
    meshFlags:      u8;
    structgen_pad1: [3] u8;
    triangleMesh:   *physx_PxTriangleMesh;
}

physx_PxHeightFieldGeometry :: struct {
    structgen_pad0:   [4] u8;
    mTypePadding:     float;
    heightField:      *physx_PxHeightField;
    heightScale:      float;
    rowScale:         float;
    columnScale:      float;
    heightFieldFlags: u8;
    structgen_pad1:   [3] u8;
}

physx_PxParticleSystemGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    mSolverType:    s32;
}

physx_PxHairSystemGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

physx_PxTetrahedronMeshGeometry :: struct {
    structgen_pad0:  [4] u8;
    mTypePadding:    float;
    tetrahedronMesh: *physx_PxTetrahedronMesh;
}

physx_PxQueryHit :: struct {
    faceIndex: u32;
}

physx_PxLocationHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
}

physx_PxGeomRaycastHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
    u:              float;
    v:              float;
}

physx_PxGeomOverlapHit :: struct {
    faceIndex: u32;
}

physx_PxGeomSweepHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
}

physx_PxGeomIndexPair :: struct {
    id0: u32;
    id1: u32;
}

physx_PxQueryThreadContext :: struct {
    structgen_pad0: [1] u8;
}

physx_PxContactBuffer :: struct {}
physx_PxRenderOutput :: struct {}

physx_PxCustomGeometryType :: struct {
    structgen_pad0: [4] u8;
}

physx_PxCustomGeometryCallbacks :: struct {
    vtable_: *void;
}

physx_PxCustomGeometry :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    callbacks:      *physx_PxCustomGeometryCallbacks;
}

physx_PxGeometryHolder :: struct {
    structgen_pad0: [56] u8;
}

physx_PxGeometryQuery :: struct {
    structgen_pad0: [1] u8;
}

physx_PxHeightFieldSample :: struct {
    height:         s16;
    materialIndex0: physx_PxBitAndByte;
    materialIndex1: physx_PxBitAndByte;
}

physx_PxHeightField :: struct {
    structgen_pad0: [16] u8;
}

physx_PxHeightFieldDesc :: struct {
    nbRows:              u32;
    nbColumns:           u32;
    format:              s32;
    structgen_pad0:      [4] u8;
    samples:             physx_PxStridedData;
    convexEdgeThreshold: float;
    flags:               u16;
    structgen_pad1:      [2] u8;
}

physx_PxMeshQuery :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSimpleTriangleMesh :: struct {
    points:         physx_PxBoundedData;
    triangles:      physx_PxBoundedData;
    flags:          u16;
    structgen_pad0: [6] u8;
}

physx_PxTriangle :: struct {
    verts: [3] Vector3;
}

physx_PxTrianglePadded :: struct {
    verts:   [3] Vector3;
    padding: u32;
}

physx_PxTriangleMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxBVH34TriangleMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTetrahedron :: struct {
    verts: [4] Vector3;
}

physx_PxSoftBodyAuxData :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTetrahedronMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxSoftBodyMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxCollisionMeshMappingData :: struct {
    structgen_pad0: [8] u8;
}

physx_PxSoftBodyCollisionData :: struct {
    structgen_pad0: [1] u8;
}

physx_PxTetrahedronMeshData :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSoftBodySimulationData :: struct {
    structgen_pad0: [1] u8;
}

physx_PxCollisionTetrahedronMeshData :: struct {
    structgen_pad0: [8] u8;
}

physx_PxSimulationTetrahedronMeshData :: struct {
    structgen_pad0: [8] u8;
}

physx_PxActor :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxAggregate :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpringModifiers :: struct {
    stiffness:      float;
    damping:        float;
    structgen_pad0: [8] u8;
}

physx_PxRestitutionModifiers :: struct {
    restitution:       float;
    velocityThreshold: float;
    structgen_pad0:    [8] u8;
}

physx_Px1DConstraintMods :: union {
    spring: physx_PxSpringModifiers;
    bounce: physx_PxRestitutionModifiers;
}

physx_Px1DConstraint :: struct {
    linear0:        Vector3;
    geometricError: float;
    angular0:       Vector3;
    velocityTarget: float;
    linear1:        Vector3;
    minImpulse:     float;
    angular1:       Vector3;
    maxImpulse:     float;
    mods:           physx_Px1DConstraintMods;
    forInternalUse: float;
    flags:          u16;
    solveHint:      u16;
    structgen_pad0: [8] u8;
}

physx_PxConstraintInvMassScale :: struct {
    linear0:  float;
    angular0: float;
    linear1:  float;
    angular1: float;
}

physx_PxConstraintVisualizer :: struct {
    vtable_: *void;
}

physx_PxConstraintConnector :: struct {
    vtable_: *void;
}

physx_PxContactPoint :: struct {
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

physx_PxSolverBody :: struct {
    linearVelocity:            Vector3;
    maxSolverNormalProgress:   u16;
    maxSolverFrictionProgress: u16;
    angularState:              Vector3;
    solverProgress:            u32;
}

physx_PxSolverBodyData :: struct {
    linearVelocity:    Vector3;
    invMass:           float;
    angularVelocity:   Vector3;
    reportThreshold:   float;
    sqrtInvInertia:    physx_PxMat33;
    penBiasClamp:      float;
    nodeIndex:         u32;
    maxContactImpulse: float;
    body2World:        physx_PxTransform;
    pad:               u16;
    structgen_pad0:    [2] u8;
}

physx_PxConstraintBatchHeader :: struct {
    startIndex:     u32;
    stride:         u16;
    constraintType: u16;
}

physx_PxSolverConstraintDesc :: struct {
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

physx_PxSolverConstraintPrepDescBase :: struct {
    invMassScales:  physx_PxConstraintInvMassScale;
    desc:           *physx_PxSolverConstraintDesc;
    body0:          *physx_PxSolverBody;
    body1:          *physx_PxSolverBody;
    data0:          *physx_PxSolverBodyData;
    data1:          *physx_PxSolverBodyData;
    bodyFrame0:     physx_PxTransform;
    bodyFrame1:     physx_PxTransform;
    bodyState0:     s32;
    bodyState1:     s32;
    structgen_pad0: [8] u8;
}

physx_PxSolverConstraintPrepDesc :: struct {
    invMassScales:        physx_PxConstraintInvMassScale;
    desc:                 *physx_PxSolverConstraintDesc;
    body0:                *physx_PxSolverBody;
    body1:                *physx_PxSolverBody;
    data0:                *physx_PxSolverBodyData;
    data1:                *physx_PxSolverBodyData;
    bodyFrame0:           physx_PxTransform;
    bodyFrame1:           physx_PxTransform;
    bodyState0:           s32;
    bodyState1:           s32;
    structgen_pad0:       [8] u8;
    rows:                 *physx_Px1DConstraint;
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
    body0WorldOffset:     physx_PxVec3Padded;
    structgen_pad2:       [8] u8;
}

physx_PxSolverContactDesc :: struct {
    invMassScales:           physx_PxConstraintInvMassScale;
    desc:                    *physx_PxSolverConstraintDesc;
    body0:                   *physx_PxSolverBody;
    body1:                   *physx_PxSolverBody;
    data0:                   *physx_PxSolverBodyData;
    data1:                   *physx_PxSolverBodyData;
    bodyFrame0:              physx_PxTransform;
    bodyFrame1:              physx_PxTransform;
    bodyState0:              s32;
    bodyState1:              s32;
    structgen_pad0:          [8] u8;
    shapeInteraction:        *void;
    contacts:                *physx_PxContactPoint;
    numContacts:             u32;
    hasMaxImpulse:           bool;
    disableStrongFriction:   bool;
    hasForceThresholds:      bool;
    structgen_pad1:          [1] u8;
    restDistance:            float;
    maxCCDSeparation:        float;
    frictionPtr:             *u8;
    frictionCount:           u8;
    structgen_pad2:          [7] u8;
    contactForces:           *float;
    startFrictionPatchIndex: u32;
    numFrictionPatches:      u32;
    startContactPatchIndex:  u32;
    numContactPatches:       u16;
    axisConstraintCount:     u16;
    offsetSlop:              float;
    structgen_pad3:          [4] u8;
}

physx_PxConstraintAllocator :: struct {
    vtable_: *void;
}

physx_PxArticulationLimit :: struct {
    low:  float;
    high: float;
}

physx_PxArticulationDrive :: struct {
    stiffness: float;
    damping:   float;
    maxForce:  float;
    driveType: s32;
}

physx_PxTGSSolverBodyVel :: struct {
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

physx_PxTGSSolverBodyTxInertia :: struct {
    deltaBody2World: physx_PxTransform;
    sqrtInvInertia:  physx_PxMat33;
}

physx_PxTGSSolverBodyData :: struct {
    originalLinearVelocity:  Vector3;
    maxContactImpulse:       float;
    originalAngularVelocity: Vector3;
    penBiasClamp:            float;
    invMass:                 float;
    nodeIndex:               u32;
    reportThreshold:         float;
    pad:                     u32;
}

physx_PxTGSSolverConstraintPrepDescBase :: struct {
    invMassScales:  physx_PxConstraintInvMassScale;
    desc:           *physx_PxSolverConstraintDesc;
    body0:          *physx_PxTGSSolverBodyVel;
    body1:          *physx_PxTGSSolverBodyVel;
    body0TxI:       *physx_PxTGSSolverBodyTxInertia;
    body1TxI:       *physx_PxTGSSolverBodyTxInertia;
    bodyData0:      *physx_PxTGSSolverBodyData;
    bodyData1:      *physx_PxTGSSolverBodyData;
    bodyFrame0:     physx_PxTransform;
    bodyFrame1:     physx_PxTransform;
    bodyState0:     s32;
    bodyState1:     s32;
    structgen_pad0: [8] u8;
}

physx_PxTGSSolverConstraintPrepDesc :: struct {
    invMassScales:        physx_PxConstraintInvMassScale;
    desc:                 *physx_PxSolverConstraintDesc;
    body0:                *physx_PxTGSSolverBodyVel;
    body1:                *physx_PxTGSSolverBodyVel;
    body0TxI:             *physx_PxTGSSolverBodyTxInertia;
    body1TxI:             *physx_PxTGSSolverBodyTxInertia;
    bodyData0:            *physx_PxTGSSolverBodyData;
    bodyData1:            *physx_PxTGSSolverBodyData;
    bodyFrame0:           physx_PxTransform;
    bodyFrame1:           physx_PxTransform;
    bodyState0:           s32;
    bodyState1:           s32;
    structgen_pad0:       [8] u8;
    rows:                 *physx_Px1DConstraint;
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
    body0WorldOffset:     physx_PxVec3Padded;
    cA2w:                 physx_PxVec3Padded;
    cB2w:                 physx_PxVec3Padded;
    structgen_pad2:       [8] u8;
}

physx_PxTGSSolverContactDesc :: struct {
    invMassScales:           physx_PxConstraintInvMassScale;
    desc:                    *physx_PxSolverConstraintDesc;
    body0:                   *physx_PxTGSSolverBodyVel;
    body1:                   *physx_PxTGSSolverBodyVel;
    body0TxI:                *physx_PxTGSSolverBodyTxInertia;
    body1TxI:                *physx_PxTGSSolverBodyTxInertia;
    bodyData0:               *physx_PxTGSSolverBodyData;
    bodyData1:               *physx_PxTGSSolverBodyData;
    bodyFrame0:              physx_PxTransform;
    bodyFrame1:              physx_PxTransform;
    bodyState0:              s32;
    bodyState1:              s32;
    structgen_pad0:          [8] u8;
    shapeInteraction:        *void;
    contacts:                *physx_PxContactPoint;
    numContacts:             u32;
    hasMaxImpulse:           bool;
    disableStrongFriction:   bool;
    hasForceThresholds:      bool;
    structgen_pad1:          [1] u8;
    restDistance:            float;
    maxCCDSeparation:        float;
    frictionPtr:             *u8;
    frictionCount:           u8;
    structgen_pad2:          [7] u8;
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
    structgen_pad3:          [8] u8;
}

physx_PxArticulationTendonLimit :: struct {
    lowLimit:  float;
    highLimit: float;
}

physx_PxArticulationAttachment :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationTendonJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationTendon :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationSpatialTendon :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationFixedTendon :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpatialForce :: struct {
    force:  Vector3;
    pad0:   float;
    torque: Vector3;
    pad1:   float;
}

physx_PxSpatialVelocity :: struct {
    linear:  Vector3;
    pad0:    float;
    angular: Vector3;
    pad1:    float;
}

physx_PxArticulationRootLinkData :: struct {
    transform:     physx_PxTransform;
    worldLinVel:   Vector3;
    worldAngVel:   Vector3;
    worldLinAccel: Vector3;
    worldAngAccel: Vector3;
}

physx_PxArticulationCache :: struct {
    externalForces:    *physx_PxSpatialForce;
    denseJacobian:     *float;
    massMatrix:        *float;
    jointVelocity:     *float;
    jointAcceleration: *float;
    jointPosition:     *float;
    jointForce:        *float;
    jointSolverForces: *float;
    linkVelocity:      *physx_PxSpatialVelocity;
    linkAcceleration:  *physx_PxSpatialVelocity;
    rootLinkData:      *physx_PxArticulationRootLinkData;
    sensorForces:      *physx_PxSpatialForce;
    coefficientMatrix: *float;
    lambda:            *float;
    scratchMemory:     *void;
    scratchAllocator:  *void;
    version:           u32;
    structgen_pad0:    [4] u8;
}

physx_PxArticulationSensor :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationReducedCoordinate :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationJointReducedCoordinate :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxShape :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRigidActor :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxNodeIndex :: struct {
    structgen_pad0: [8] u8;
}

physx_PxRigidBody :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationLink :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxConeLimitedConstraint :: struct {
    mAxis:      Vector3;
    mAngle:     float;
    mLowLimit:  float;
    mHighLimit: float;
}

physx_PxConeLimitParams :: struct {
    lowHighLimits: Vector4;
    axisAngle:     Vector4;
}

physx_PxConstraintShaderTable :: struct {
    solverPrep:     *void;
    structgen_pad0: [8] u8;
    visualize:      *void;
    flag:           s32;
    structgen_pad1: [4] u8;
}

physx_PxConstraint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxMassModificationProps :: struct {
    mInvMassScale0:    float;
    mInvInertiaScale0: float;
    mInvMassScale1:    float;
    mInvInertiaScale1: float;
}

physx_PxContactPatch :: struct {
    mMassModification: physx_PxMassModificationProps;
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

physx_PxContact :: struct {
    contact:    Vector3;
    separation: float;
}

physx_PxExtendedContact :: struct {
    contact:        Vector3;
    separation:     float;
    targetVelocity: Vector3;
    maxImpulse:     float;
}

physx_PxModifiableContact :: struct {
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

physx_PxContactStreamIterator :: struct {
    zero:                   Vector3;
    structgen_pad0:         [4] u8;
    patch:                  *physx_PxContactPatch;
    contact:                *physx_PxContact;
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

physx_PxGpuContactPair :: struct {
    contactPatches:     *u8;
    contactPoints:      *u8;
    contactForces:      *float;
    transformCacheRef0: u32;
    transformCacheRef1: u32;
    nodeIndex0:         physx_PxNodeIndex;
    nodeIndex1:         physx_PxNodeIndex;
    actor0:             *physx_PxActor;
    actor1:             *physx_PxActor;
    nbContacts:         u16;
    nbPatches:          u16;
    structgen_pad0:     [4] u8;
}

physx_PxContactSet :: struct {
    structgen_pad0: [16] u8;
}

physx_PxContactModifyPair :: struct {
    actor:     [2] *physx_PxRigidActor;
    shape:     [2] *physx_PxShape;
    transform: [2] physx_PxTransform;
    contacts:  physx_PxContactSet;
}

physx_PxContactModifyCallback :: struct {
    vtable_: *void;
}

physx_PxCCDContactModifyCallback :: struct {
    vtable_: *void;
}

physx_PxDeletionListener :: struct {
    vtable_: *void;
}

physx_PxBaseMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFEMMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFilterData :: struct {
    word0: u32;
    word1: u32;
    word2: u32;
    word3: u32;
}

physx_PxSimulationFilterCallback :: struct {
    vtable_: *void;
}

physx_PxParticleRigidFilterPair :: struct {
    mID0: u64;
    mID1: u64;
}

physx_PxLockedData :: struct {
    vtable_: *void;
}

physx_PxMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGpuParticleBufferIndexPair :: struct {
    systemIndex: u32;
    bufferIndex: u32;
}

physx_PxCudaContextManager :: struct {}
physx_PxParticleRigidAttachment :: struct {}
physx_PxParticleVolume :: struct {
    bound:                 physx_PxBounds3;
    particleIndicesOffset: u32;
    numParticles:          u32;
}

physx_PxDiffuseParticleParams :: struct {
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

physx_PxParticleSpring :: struct {
    ind0:      u32;
    ind1:      u32;
    length:    float;
    stiffness: float;
    damping:   float;
    pad:       float;
}

physx_PxParticleMaterial :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxOmniPvd :: struct {}
physx_PxPhysics :: struct {
    vtable_: *void;
}

physx_PxActorShape :: struct {
    actor: *physx_PxRigidActor;
    shape: *physx_PxShape;
}

physx_PxRaycastHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
    u:              float;
    v:              float;
    structgen_pad1: [4] u8;
    actor:          *physx_PxRigidActor;
    shape:          *physx_PxShape;
}

physx_PxOverlapHit :: struct {
    faceIndex:      u32;
    structgen_pad0: [4] u8;
    actor:          *physx_PxRigidActor;
    shape:          *physx_PxShape;
}

physx_PxSweepHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       Vector3;
    normal:         Vector3;
    distance:       float;
    structgen_pad1: [4] u8;
    actor:          *physx_PxRigidActor;
    shape:          *physx_PxShape;
}

physx_PxRaycastCallback :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxRaycastHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxRaycastHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxOverlapCallback :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxOverlapHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxOverlapHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxSweepCallback :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxSweepHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxSweepHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxRaycastBuffer :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxRaycastHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxRaycastHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxOverlapBuffer :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxOverlapHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxOverlapHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxSweepBuffer :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxSweepHit;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxSweepHit;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxQueryCache :: struct {
    shape:          *physx_PxShape;
    actor:          *physx_PxRigidActor;
    faceIndex:      u32;
    structgen_pad0: [4] u8;
}

physx_PxQueryFilterData :: struct {
    data:           physx_PxFilterData;
    flags:          u16;
    structgen_pad0: [2] u8;
}

physx_PxQueryFilterCallback :: struct {
    vtable_: *void;
}

physx_PxRigidDynamic :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRigidStatic :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSceneQueryDesc :: struct {
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

physx_PxSceneQuerySystemBase :: struct {
    vtable_: *void;
}

physx_PxSceneSQSystem :: struct {
    vtable_: *void;
}

physx_PxSceneQuerySystem :: struct {
    vtable_: *void;
}

physx_PxBroadPhaseRegion :: struct {
    mBounds:   physx_PxBounds3;
    mUserData: *void;
}

physx_PxBroadPhaseRegionInfo :: struct {
    mRegion:           physx_PxBroadPhaseRegion;
    mNbStaticObjects:  u32;
    mNbDynamicObjects: u32;
    mActive:           bool;
    mOverlap:          bool;
    structgen_pad0:    [6] u8;
}

physx_PxBroadPhaseCaps :: struct {
    mMaxNbRegions: u32;
}

physx_PxBroadPhaseDesc :: struct {
    mType:                        s32;
    structgen_pad0:               [4] u8;
    mContextID:                   u64;
    structgen_pad1:               [8] u8;
    mFoundLostPairsCapacity:      u32;
    mDiscardStaticVsKinematic:    bool;
    mDiscardKinematicVsKinematic: bool;
    structgen_pad2:               [2] u8;
}

physx_PxBroadPhaseUpdateData :: struct {
    mCreated:       *u32;
    mNbCreated:     u32;
    structgen_pad0: [4] u8;
    mUpdated:       *u32;
    mNbUpdated:     u32;
    structgen_pad1: [4] u8;
    mRemoved:       *u32;
    mNbRemoved:     u32;
    structgen_pad2: [4] u8;
    mBounds:        *physx_PxBounds3;
    mGroups:        *u32;
    mDistances:     *float;
    mCapacity:      u32;
    structgen_pad3: [4] u8;
}

physx_PxBroadPhasePair :: struct {
    mID0: u32;
    mID1: u32;
}

physx_PxBroadPhaseResults :: struct {
    mNbCreatedPairs: u32;
    structgen_pad0:  [4] u8;
    mCreatedPairs:   *physx_PxBroadPhasePair;
    mNbDeletedPairs: u32;
    structgen_pad1:  [4] u8;
    mDeletedPairs:   *physx_PxBroadPhasePair;
}

physx_PxBroadPhaseRegions :: struct {
    vtable_: *void;
}

physx_PxBroadPhase :: struct {
    vtable_: *void;
}

physx_PxAABBManager :: struct {
    vtable_: *void;
}

physx_PxSceneLimits :: struct {
    maxNbActors:             u32;
    maxNbBodies:             u32;
    maxNbStaticShapes:       u32;
    maxNbDynamicShapes:      u32;
    maxNbAggregates:         u32;
    maxNbConstraints:        u32;
    maxNbRegions:            u32;
    maxNbBroadPhaseOverlaps: u32;
}

physx_PxgDynamicsMemoryConfig :: struct {
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

physx_PxSceneDesc :: struct {
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
    simulationEventCallback:       *physx_PxSimulationEventCallback;
    contactModifyCallback:         *physx_PxContactModifyCallback;
    ccdContactModifyCallback:      *physx_PxCCDContactModifyCallback;
    filterShaderData:              *void;
    filterShaderDataSize:          u32;
    structgen_pad0:                [4] u8;
    filterShader:                  *void;
    filterCallback:                *physx_PxSimulationFilterCallback;
    kineKineFilteringMode:         s32;
    staticKineFilteringMode:       s32;
    broadPhaseType:                s32;
    structgen_pad1:                [4] u8;
    broadPhaseCallback:            *physx_PxBroadPhaseCallback;
    limits:                        physx_PxSceneLimits;
    frictionType:                  s32;
    solverType:                    s32;
    bounceThresholdVelocity:       float;
    frictionOffsetThreshold:       float;
    frictionCorrelationDistance:   float;
    flags:                         u32;
    cpuDispatcher:                 *physx_PxCpuDispatcher;
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
    sanityBounds:                  physx_PxBounds3;
    gpuDynamicsConfig:             physx_PxgDynamicsMemoryConfig;
    gpuMaxNumPartitions:           u32;
    gpuMaxNumStaticPartitions:     u32;
    gpuComputeVersion:             u32;
    contactPairSlabSize:           u32;
    sceneQuerySystem:              *physx_PxSceneQuerySystem;
    structgen_pad3:                [8] u8;
}

physx_PxSimulationStatistics :: struct {
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

physx_PxGpuBodyData :: struct {
    quat:   Quaternion;
    pos:    Vector4;
    linVel: Vector4;
    angVel: Vector4;
}

physx_PxGpuActorPair :: struct {
    srcIndex:       u32;
    structgen_pad0: [4] u8;
    nodeIndex:      physx_PxNodeIndex;
}

physx_PxIndexDataPair :: struct {
    index:          u32;
    structgen_pad0: [4] u8;
    data:           *void;
}

physx_PxPvdSceneClient :: struct {
    vtable_: *void;
}

physx_PxDominanceGroupPair :: struct {
    dominance0: u8;
    dominance1: u8;
}

physx_PxBroadPhaseCallback :: struct {
    vtable_: *void;
}

physx_PxScene :: struct {
    structgen_pad0: [8] u8;
    userData:       *void;
}

physx_PxSceneReadLock :: struct {
    structgen_pad0: [8] u8;
}

physx_PxSceneWriteLock :: struct {
    structgen_pad0: [8] u8;
}

physx_PxContactPairExtraDataItem :: struct {
    type: u8;
}

physx_PxContactPairVelocity :: struct {
    type:            u8;
    structgen_pad0:  [3] u8;
    linearVelocity:  [2] Vector3;
    angularVelocity: [2] Vector3;
}

physx_PxContactPairPose :: struct {
    type:           u8;
    structgen_pad0: [3] u8;
    globalPose:     [2] physx_PxTransform;
}

physx_PxContactPairIndex :: struct {
    type:           u8;
    structgen_pad0: [1] u8;
    index:          u16;
}

physx_PxContactPairExtraDataIterator :: struct {
    currPtr:            *u8;
    endPtr:             *u8;
    preSolverVelocity:  *physx_PxContactPairVelocity;
    postSolverVelocity: *physx_PxContactPairVelocity;
    eventPose:          *physx_PxContactPairPose;
    contactPairIndex:   u32;
    structgen_pad0:     [4] u8;
}

physx_PxContactPairHeader :: struct {
    actors:              [2] *physx_PxActor;
    extraDataStream:     *u8;
    extraDataStreamSize: u16;
    flags:               u16;
    structgen_pad0:      [4] u8;
    pairs:               *physx_PxContactPair;
    nbPairs:             u32;
    structgen_pad1:      [4] u8;
}

physx_PxContactPairPoint :: struct {
    position:           Vector3;
    separation:         float;
    normal:             Vector3;
    internalFaceIndex0: u32;
    impulse:            Vector3;
    internalFaceIndex1: u32;
}

physx_PxContactPair :: struct {
    shapes:             [2] *physx_PxShape;
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

physx_PxTriggerPair :: struct {
    triggerShape:   *physx_PxShape;
    triggerActor:   *physx_PxActor;
    otherShape:     *physx_PxShape;
    otherActor:     *physx_PxActor;
    status:         s32;
    flags:          u8;
    structgen_pad0: [3] u8;
}

physx_PxConstraintInfo :: struct {
    constraint:        *physx_PxConstraint;
    externalReference: *void;
    type:              u32;
    structgen_pad0:    [4] u8;
}

physx_PxSimulationEventCallback :: struct {
    vtable_: *void;
}

physx_PxFEMParameters :: struct {
    velocityDamping:              float;
    settlingThreshold:            float;
    sleepThreshold:               float;
    sleepDamping:                 float;
    selfCollisionFilterDistance:  float;
    selfCollisionStressTolerance: float;
}

physx_PxPruningStructure :: struct {
    structgen_pad0: [16] u8;
}

physx_PxExtendedVec3 :: struct {
    x: float64;
    y: float64;
    z: float64;
}

physx_PxObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3;
    mRot:           Quaternion;
}

physx_PxBoxObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3;
    mRot:           Quaternion;
    mHalfExtents:   Vector3;
    structgen_pad1: [4] u8;
}

physx_PxCapsuleObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3;
    mRot:           Quaternion;
    mHalfHeight:    float;
    mRadius:        float;
}

physx_PxObstacleContext :: struct {
    vtable_: *void;
}

physx_PxControllerState :: struct {
    deltaXP:               Vector3;
    structgen_pad0:        [4] u8;
    touchedShape:          *physx_PxShape;
    touchedActor:          *physx_PxRigidActor;
    touchedObstacleHandle: u32;
    collisionFlags:        u32;
    standOnAnotherCCT:     bool;
    standOnObstacle:       bool;
    isMovingUp:            bool;
    structgen_pad1:        [5] u8;
}

physx_PxControllerStats :: struct {
    nbIterations:     u16;
    nbFullUpdates:    u16;
    nbPartialUpdates: u16;
    nbTessellation:   u16;
}

physx_PxControllerHit :: struct {
    controller:     *physx_PxController;
    worldPos:       physx_PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
}

physx_PxControllerShapeHit :: struct {
    controller:     *physx_PxController;
    worldPos:       physx_PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    shape:          *physx_PxShape;
    actor:          *physx_PxRigidActor;
    triangleIndex:  u32;
    structgen_pad1: [4] u8;
}

physx_PxControllersHit :: struct {
    controller:     *physx_PxController;
    worldPos:       physx_PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    other:          *physx_PxController;
}

physx_PxControllerObstacleHit :: struct {
    controller:     *physx_PxController;
    worldPos:       physx_PxExtendedVec3;
    worldNormal:    Vector3;
    dir:            Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    userData:       *void;
}

physx_PxUserControllerHitReport :: struct {
    vtable_: *void;
}

physx_PxControllerFilterCallback :: struct {
    vtable_: *void;
}

physx_PxControllerFilters :: struct {
    mFilterData:        *physx_PxFilterData;
    mFilterCallback:    *physx_PxQueryFilterCallback;
    mFilterFlags:       u16;
    structgen_pad0:     [6] u8;
    mCCTFilterCallback: *physx_PxControllerFilterCallback;
}

physx_PxControllerDesc :: struct {
    structgen_pad0:           [8] u8;
    position:                 physx_PxExtendedVec3;
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
    reportCallback:           *physx_PxUserControllerHitReport;
    behaviorCallback:         *physx_PxControllerBehaviorCallback;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *physx_PxMaterial;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [8] u8;
}

physx_PxController :: struct {
    vtable_: *void;
}

physx_PxBoxControllerDesc :: struct {
    structgen_pad0:           [8] u8;
    position:                 physx_PxExtendedVec3;
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
    reportCallback:           *physx_PxUserControllerHitReport;
    behaviorCallback:         *physx_PxControllerBehaviorCallback;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *physx_PxMaterial;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [8] u8;
    halfHeight:               float;
    halfSideExtent:           float;
    halfForwardExtent:        float;
    structgen_pad5:           [4] u8;
}

physx_PxBoxController :: struct {
    vtable_: *void;
}

physx_PxCapsuleControllerDesc :: struct {
    structgen_pad0:           [8] u8;
    position:                 physx_PxExtendedVec3;
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
    reportCallback:           *physx_PxUserControllerHitReport;
    behaviorCallback:         *physx_PxControllerBehaviorCallback;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *physx_PxMaterial;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [8] u8;
    radius:                   float;
    height:                   float;
    climbingMode:             s32;
    structgen_pad5:           [4] u8;
}

physx_PxCapsuleController :: struct {
    vtable_: *void;
}

physx_PxControllerBehaviorCallback :: struct {
    vtable_: *void;
}

physx_PxControllerManager :: struct {
    vtable_: *void;
}

physx_PxDim3 :: struct {
    x: u32;
    y: u32;
    z: u32;
}

physx_PxSDFDesc :: struct {
    sdf:                                            physx_PxBoundedData;
    dims:                                           physx_PxDim3;
    meshLower:                                      Vector3;
    spacing:                                        float;
    subgridSize:                                    u32;
    bitsPerSubgridPixel:                            s32;
    sdfSubgrids3DTexBlockDim:                       physx_PxDim3;
    sdfSubgrids:                                    physx_PxBoundedData;
    sdfStartSlots:                                  physx_PxBoundedData;
    subgridsMinSdfValue:                            float;
    subgridsMaxSdfValue:                            float;
    sdfBounds:                                      physx_PxBounds3;
    narrowBandThicknessRelativeToSdfBoundsDiagonal: float;
    numThreadsForSdfConstruction:                   u32;
}

physx_PxConvexMeshDesc :: struct {
    points:         physx_PxBoundedData;
    polygons:       physx_PxBoundedData;
    indices:        physx_PxBoundedData;
    flags:          u16;
    vertexLimit:    u16;
    polygonLimit:   u16;
    quantizedCount: u16;
    sdfDesc:        *physx_PxSDFDesc;
}

physx_PxTriangleMeshDesc :: struct {
    points:         physx_PxBoundedData;
    triangles:      physx_PxBoundedData;
    flags:          u16;
    structgen_pad0: [22] u8;
    sdfDesc:        *physx_PxSDFDesc;
}

physx_PxTetrahedronMeshDesc :: struct {
    structgen_pad0: [16] u8;
    points:         physx_PxBoundedData;
    tetrahedrons:   physx_PxBoundedData;
    flags:          u16;
    tetsPerElement: u16;
    structgen_pad1: [4] u8;
}

physx_PxSoftBodySimulationDataDesc :: struct {
    vertexToTet: physx_PxBoundedData;
}

physx_PxBVH34MidphaseDesc :: struct {
    numPrimsPerLeaf: u32;
    buildStrategy:   s32;
    quantized:       bool;
    structgen_pad0:  [3] u8;
}

physx_PxMidphaseDesc :: struct {
    structgen_pad0: [16] u8;
}

physx_PxBVHDesc :: struct {
    bounds:          physx_PxBoundedData;
    enlargement:     float;
    numPrimsPerLeaf: u32;
    buildStrategy:   s32;
    structgen_pad0:  [4] u8;
}

physx_PxCookingParams :: struct {
    areaTestEpsilon:                float;
    planeTolerance:                 float;
    convexMeshCookingType:          s32;
    suppressTriangleMeshRemapTable: bool;
    buildTriangleAdjacencies:       bool;
    buildGPUData:                   bool;
    structgen_pad0:                 [1] u8;
    scale:                          physx_PxTolerancesScale;
    meshPreprocessParams:           u32;
    meshWeldTolerance:              float;
    midphaseDesc:                   physx_PxMidphaseDesc;
    gaussMapLimit:                  u32;
    maxWeightRatioInTet:            float;
}

physx_PxDefaultMemoryOutputStream :: struct {
    structgen_pad0: [32] u8;
}

physx_PxDefaultMemoryInputData :: struct {
    structgen_pad0: [32] u8;
}

physx_PxDefaultFileOutputStream :: struct {
    structgen_pad0: [16] u8;
}

physx_PxDefaultFileInputData :: struct {
    structgen_pad0: [24] u8;
}

physx_PxDefaultAllocator :: struct {
    vtable_: *void;
}

physx_PxJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpring :: struct {
    stiffness: float;
    damping:   float;
}

physx_PxDistanceJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxJacobianRow :: struct {
    linear0:  Vector3;
    linear1:  Vector3;
    angular0: Vector3;
    angular1: Vector3;
}

physx_PxContactJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFixedJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxJointLimitParameters :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
}

physx_PxJointLinearLimit :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    value:                      float;
}

physx_PxJointLinearLimitPair :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

physx_PxJointAngularLimitPair :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

physx_PxJointLimitCone :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    yAngle:                     float;
    zAngle:                     float;
}

physx_PxJointLimitPyramid :: struct {
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

physx_PxPrismaticJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRevoluteJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSphericalJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxD6JointDrive :: struct {
    stiffness:  float;
    damping:    float;
    forceLimit: float;
    flags:      u32;
}

physx_PxD6Joint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGearJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRackAndPinionJoint :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGroupsMask :: struct {
    bits0: u16;
    bits1: u16;
    bits2: u16;
    bits3: u16;
}

physx_PxDefaultErrorCallback :: struct {
    vtable_: *void;
}

physx_PxRigidActorExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxMassProperties :: struct {
    inertiaTensor: physx_PxMat33;
    centerOfMass:  Vector3;
    mass:          float;
}

physx_PxRigidBodyExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxShapeExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxMeshOverlapUtil :: struct {
    structgen_pad0: [1040] u8;
}

physx_PxBinaryConverter :: struct {}
physx_PxXmlMiscParameter :: struct {
    upVector: Vector3;
    scale:    physx_PxTolerancesScale;
}

physx_PxSerialization :: struct {
    structgen_pad0: [1] u8;
}

physx_PxDefaultCpuDispatcher :: struct {
    vtable_: *void;
}

physx_PxStringTableExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxBroadPhaseExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSceneQueryExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxBatchQueryExt :: struct {
    vtable_: *void;
}

physx_PxCustomSceneQuerySystem :: struct {
    vtable_: *void;
}

physx_PxCustomSceneQuerySystemAdapter :: struct {
    vtable_: *void;
}

physx_PxSamplingExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxPoissonSampler :: struct {
    structgen_pad0: [8] u8;
}

physx_PxTriangleMeshPoissonSampler :: struct {
    structgen_pad0: [24] u8;
}

physx_PxTetrahedronMeshExt :: struct {
    structgen_pad0: [1] u8;
}

physx_PxRepXObject :: struct {
    typeName:     *u8;
    serializable: *void;
    id:           u64;
}

physx_PxCooking :: struct {}
physx_PxRepXInstantiationArgs :: struct {
    structgen_pad0: [8] u8;
    cooker:         *physx_PxCooking;
    stringTable:    *physx_PxStringTable;
}

physx_XmlMemoryAllocator :: struct {}
physx_XmlWriter :: struct {}
physx_XmlReader :: struct {}
physx_MemoryBuffer :: struct {}
physx_PxRepXSerializer :: struct {
    vtable_: *void;
}

physx_PxVehicleWheels4SimData :: struct {}
physx_PxVehicleWheels4DynData :: struct {}
physx_PxVehicleTireForceCalculator :: struct {}
physx_PxVehicleDrivableSurfaceToTireFrictionPairs :: struct {}
physx_PxVehicleTelemetryData :: struct {}

physx_PxPvd :: struct {
    vtable_: *void;
}

physx_PxPvdTransport :: struct {
    vtable_: *void;
}

PxAllocatorCallback_delete :: (self: *physx_PxAllocatorCallback) -> void #foreign physx_sys;

PxAllocatorCallback_allocate :: (self: *physx_PxAllocatorCallback, size: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign physx_sys "PxAllocatorCallback_allocate_mut";

PxAllocatorCallback_deallocate :: (self: *physx_PxAllocatorCallback, ptr: *void) -> void #foreign physx_sys "PxAllocatorCallback_deallocate_mut";

PxAssertHandler_delete :: (self: *physx_PxAssertHandler) -> void #foreign physx_sys;

phys_PxGetAssertHandler :: () -> *physx_PxAssertHandler #foreign physx_sys;

phys_PxSetAssertHandler :: (handler: *physx_PxAssertHandler) -> void #foreign physx_sys;

PxFoundation_release :: (self: *physx_PxFoundation) -> void #foreign physx_sys "PxFoundation_release_mut";

PxFoundation_getErrorCallback :: (self: *physx_PxFoundation) -> *physx_PxErrorCallback #foreign physx_sys "PxFoundation_getErrorCallback_mut";

PxFoundation_setErrorLevel :: (self: *physx_PxFoundation, mask: u32) -> void #foreign physx_sys "PxFoundation_setErrorLevel_mut";

PxFoundation_getErrorLevel :: (self: *physx_PxFoundation) -> u32 #foreign physx_sys;

PxFoundation_getAllocatorCallback :: (self: *physx_PxFoundation) -> *physx_PxAllocatorCallback #foreign physx_sys "PxFoundation_getAllocatorCallback_mut";

PxFoundation_getReportAllocationNames :: (self: *physx_PxFoundation) -> bool #foreign physx_sys;

PxFoundation_setReportAllocationNames :: (self: *physx_PxFoundation, value: bool) -> void #foreign physx_sys "PxFoundation_setReportAllocationNames_mut";

PxFoundation_registerAllocationListener :: (self: *physx_PxFoundation, listener: *physx_PxAllocationListener) -> void #foreign physx_sys "PxFoundation_registerAllocationListener_mut";

PxFoundation_deregisterAllocationListener :: (self: *physx_PxFoundation, listener: *physx_PxAllocationListener) -> void #foreign physx_sys "PxFoundation_deregisterAllocationListener_mut";

PxFoundation_registerErrorCallback :: (self: *physx_PxFoundation, callback: *physx_PxErrorCallback) -> void #foreign physx_sys "PxFoundation_registerErrorCallback_mut";

PxFoundation_deregisterErrorCallback :: (self: *physx_PxFoundation, callback: *physx_PxErrorCallback) -> void #foreign physx_sys "PxFoundation_deregisterErrorCallback_mut";

phys_PxCreateFoundation :: (version: u32, allocator: *physx_PxAllocatorCallback, errorCallback: *physx_PxErrorCallback) -> *physx_PxFoundation #foreign physx_sys;

phys_PxSetFoundationInstance :: (foundation: *physx_PxFoundation) -> void #foreign physx_sys;

phys_PxGetFoundation :: () -> *physx_PxFoundation #foreign physx_sys;

phys_PxGetProfilerCallback :: () -> *physx_PxProfilerCallback #foreign physx_sys;

phys_PxSetProfilerCallback :: (profiler: *physx_PxProfilerCallback) -> void #foreign physx_sys;

phys_PxGetAllocatorCallback :: () -> *physx_PxAllocatorCallback #foreign physx_sys;

phys_PxGetBroadcastAllocator :: () -> *physx_PxAllocatorCallback #foreign physx_sys;

phys_PxGetErrorCallback :: () -> *physx_PxErrorCallback #foreign physx_sys;

phys_PxGetBroadcastError :: () -> *physx_PxErrorCallback #foreign physx_sys;

phys_PxGetWarnOnceTimeStamp :: () -> u32 #foreign physx_sys;

phys_PxDecFoundationRefCount :: () -> void #foreign physx_sys;

phys_PxIncFoundationRefCount :: () -> void #foreign physx_sys;

PxAllocator_new :: (anon_param0: *u8) -> physx_PxAllocator #foreign physx_sys;

PxAllocator_allocate :: (self: *physx_PxAllocator, size: u64, file: *u8, line: s32) -> *void #foreign physx_sys "PxAllocator_allocate_mut";

PxAllocator_deallocate :: (self: *physx_PxAllocator, ptr: *void) -> void #foreign physx_sys "PxAllocator_deallocate_mut";

PxRawAllocator_new :: (anon_param0: *u8) -> physx_PxRawAllocator #foreign physx_sys;

PxRawAllocator_allocate :: (self: *physx_PxRawAllocator, size: u64, anon_param1: *u8, anon_param2: s32) -> *void #foreign physx_sys "PxRawAllocator_allocate_mut";

PxRawAllocator_deallocate :: (self: *physx_PxRawAllocator, ptr: *void) -> void #foreign physx_sys "PxRawAllocator_deallocate_mut";

PxVirtualAllocatorCallback_delete :: (self: *physx_PxVirtualAllocatorCallback) -> void #foreign physx_sys;

PxVirtualAllocatorCallback_allocate :: (self: *physx_PxVirtualAllocatorCallback, size: u64, group: s32, file: *u8, line: s32) -> *void #foreign physx_sys "PxVirtualAllocatorCallback_allocate_mut";

PxVirtualAllocatorCallback_deallocate :: (self: *physx_PxVirtualAllocatorCallback, ptr: *void) -> void #foreign physx_sys "PxVirtualAllocatorCallback_deallocate_mut";

PxVirtualAllocator_new :: (callback: *physx_PxVirtualAllocatorCallback, group: s32) -> physx_PxVirtualAllocator #foreign physx_sys;

PxVirtualAllocator_allocate :: (self: *physx_PxVirtualAllocator, size: u64, file: *u8, line: s32) -> *void #foreign physx_sys "PxVirtualAllocator_allocate_mut";

PxVirtualAllocator_deallocate :: (self: *physx_PxVirtualAllocator, ptr: *void) -> void #foreign physx_sys "PxVirtualAllocator_deallocate_mut";

PxTempAllocatorChunk_new :: () -> physx_PxTempAllocatorChunk #foreign physx_sys;

PxTempAllocator_new :: (anon_param0: *u8) -> physx_PxTempAllocator #foreign physx_sys;

PxTempAllocator_allocate :: (self: *physx_PxTempAllocator, size: u64, file: *u8, line: s32) -> *void #foreign physx_sys "PxTempAllocator_allocate_mut";

PxTempAllocator_deallocate :: (self: *physx_PxTempAllocator, ptr: *void) -> void #foreign physx_sys "PxTempAllocator_deallocate_mut";

phys_PxMemZero :: (dest: *void, count: u32) -> *void #foreign physx_sys;

phys_PxMemSet :: (dest: *void, c: s32, count: u32) -> *void #foreign physx_sys;

phys_PxMemCopy :: (dest: *void, src: *void, count: u32) -> *void #foreign physx_sys;

phys_PxMemMove :: (dest: *void, src: *void, count: u32) -> *void #foreign physx_sys;

phys_PxMarkSerializedMemory :: (ptr: *void, byteSize: u32) -> void #foreign physx_sys;

phys_PxMemoryBarrier :: () -> void #foreign physx_sys;

phys_PxHighestSetBitUnsafe :: (v: u32) -> u32 #foreign physx_sys;

phys_PxLowestSetBitUnsafe :: (v: u32) -> u32 #foreign physx_sys;

phys_PxCountLeadingZeros :: (v: u32) -> u32 #foreign physx_sys;

phys_PxPrefetchLine :: (ptr: *void, offset: u32) -> void #foreign physx_sys;

phys_PxPrefetch :: (ptr: *void, count: u32) -> void #foreign physx_sys;

phys_PxBitCount :: (v: u32) -> u32 #foreign physx_sys;

phys_PxIsPowerOfTwo :: (x: u32) -> bool #foreign physx_sys;

phys_PxNextPowerOfTwo :: (x: u32) -> u32 #foreign physx_sys;

phys_PxLowestSetBit :: (x: u32) -> u32 #foreign physx_sys;

phys_PxHighestSetBit :: (x: u32) -> u32 #foreign physx_sys;

phys_PxILog2 :: (num: u32) -> u32 #foreign physx_sys;

PxVec3_new :: () -> Vector3 #foreign physx_sys;

PxVec3_new_1 :: (anon_param0: s32) -> Vector3 #foreign physx_sys;

PxVec3_new_2 :: (a: float) -> Vector3 #foreign physx_sys;

PxVec3_new_3 :: (nx: float, ny: float, nz: float) -> Vector3 #foreign physx_sys;

PxVec3_isZero :: (self: *Vector3) -> bool #foreign physx_sys;

PxVec3_isFinite :: (self: *Vector3) -> bool #foreign physx_sys;

PxVec3_isNormalized :: (self: *Vector3) -> bool #foreign physx_sys;

PxVec3_magnitudeSquared :: (self: *Vector3) -> float #foreign physx_sys;

PxVec3_magnitude :: (self: *Vector3) -> float #foreign physx_sys;

PxVec3_dot :: (self: *Vector3, v: *Vector3) -> float #foreign physx_sys;

PxVec3_cross :: (self: *Vector3, v: *Vector3) -> Vector3 #foreign physx_sys;

PxVec3_getNormalized :: (self: *Vector3) -> Vector3 #foreign physx_sys;

PxVec3_normalize :: (self: *Vector3) -> float #foreign physx_sys "PxVec3_normalize_mut";

PxVec3_normalizeSafe :: (self: *Vector3) -> float #foreign physx_sys "PxVec3_normalizeSafe_mut";

PxVec3_normalizeFast :: (self: *Vector3) -> float #foreign physx_sys "PxVec3_normalizeFast_mut";

PxVec3_multiply :: (self: *Vector3, a: *Vector3) -> Vector3 #foreign physx_sys;

PxVec3_minimum :: (self: *Vector3, v: *Vector3) -> Vector3 #foreign physx_sys;

PxVec3_minElement :: (self: *Vector3) -> float #foreign physx_sys;

PxVec3_maximum :: (self: *Vector3, v: *Vector3) -> Vector3 #foreign physx_sys;

PxVec3_maxElement :: (self: *Vector3) -> float #foreign physx_sys;

PxVec3_abs :: (self: *Vector3) -> Vector3 #foreign physx_sys;

PxVec3Padded_new_alloc :: () -> *physx_PxVec3Padded #foreign physx_sys;

PxVec3Padded_delete :: (self: *physx_PxVec3Padded) -> void #foreign physx_sys;

PxVec3Padded_new_alloc_1 :: (p: *Vector3) -> *physx_PxVec3Padded #foreign physx_sys;

PxVec3Padded_new_alloc_2 :: (f: float) -> *physx_PxVec3Padded #foreign physx_sys;

PxQuat_new :: () -> Quaternion #foreign physx_sys;

PxQuat_new_1 :: (anon_param0: s32) -> Quaternion #foreign physx_sys;

PxQuat_new_2 :: (r: float) -> Quaternion #foreign physx_sys;

PxQuat_new_3 :: (nx: float, ny: float, nz: float, nw: float) -> Quaternion #foreign physx_sys;

PxQuat_new_4 :: (angleRadians: float, unitAxis: *Vector3) -> Quaternion #foreign physx_sys;

PxQuat_new_5 :: (m: *physx_PxMat33) -> Quaternion #foreign physx_sys;

PxQuat_isIdentity :: (self: *Quaternion) -> bool #foreign physx_sys;

PxQuat_isFinite :: (self: *Quaternion) -> bool #foreign physx_sys;

PxQuat_isUnit :: (self: *Quaternion) -> bool #foreign physx_sys;

PxQuat_isSane :: (self: *Quaternion) -> bool #foreign physx_sys;

PxQuat_toRadiansAndUnitAxis :: (self: *Quaternion, angle: *float, axis: *Vector3) -> void #foreign physx_sys;

PxQuat_getAngle :: (self: *Quaternion) -> float #foreign physx_sys;

PxQuat_getAngle_1 :: (self: *Quaternion, q: *Quaternion) -> float #foreign physx_sys;

PxQuat_magnitudeSquared :: (self: *Quaternion) -> float #foreign physx_sys;

PxQuat_dot :: (self: *Quaternion, v: *Quaternion) -> float #foreign physx_sys;

PxQuat_getNormalized :: (self: *Quaternion) -> Quaternion #foreign physx_sys;

PxQuat_magnitude :: (self: *Quaternion) -> float #foreign physx_sys;

PxQuat_normalize :: (self: *Quaternion) -> float #foreign physx_sys "PxQuat_normalize_mut";

PxQuat_getConjugate :: (self: *Quaternion) -> Quaternion #foreign physx_sys;

PxQuat_getImaginaryPart :: (self: *Quaternion) -> Vector3 #foreign physx_sys;

PxQuat_getBasisVector0 :: (self: *Quaternion) -> Vector3 #foreign physx_sys;

PxQuat_getBasisVector1 :: (self: *Quaternion) -> Vector3 #foreign physx_sys;

PxQuat_getBasisVector2 :: (self: *Quaternion) -> Vector3 #foreign physx_sys;

PxQuat_rotate :: (self: *Quaternion, v: *Vector3) -> Vector3 #foreign physx_sys;

PxQuat_rotateInv :: (self: *Quaternion, v: *Vector3) -> Vector3 #foreign physx_sys;

PxTransform_new :: () -> physx_PxTransform #foreign physx_sys;

PxTransform_new_1 :: (position: *Vector3) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_2 :: (anon_param0: s32) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_3 :: (orientation: *Quaternion) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_4 :: (x: float, y: float, z: float, aQ: Quaternion) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_5 :: (p0: *Vector3, q0: *Quaternion) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_6 :: (m: *physx_PxMat44) -> physx_PxTransform #foreign physx_sys;

PxTransform_getInverse :: (self: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxTransform_transform :: (self: *physx_PxTransform, input: *Vector3) -> Vector3 #foreign physx_sys;

PxTransform_transformInv :: (self: *physx_PxTransform, input: *Vector3) -> Vector3 #foreign physx_sys;

PxTransform_rotate :: (self: *physx_PxTransform, input: *Vector3) -> Vector3 #foreign physx_sys;

PxTransform_rotateInv :: (self: *physx_PxTransform, input: *Vector3) -> Vector3 #foreign physx_sys;

PxTransform_transform_1 :: (self: *physx_PxTransform, src: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxTransform_isValid :: (self: *physx_PxTransform) -> bool #foreign physx_sys;

PxTransform_isSane :: (self: *physx_PxTransform) -> bool #foreign physx_sys;

PxTransform_isFinite :: (self: *physx_PxTransform) -> bool #foreign physx_sys;

PxTransform_transformInv_1 :: (self: *physx_PxTransform, src: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxTransform_getNormalized :: (self: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxMat33_new :: () -> physx_PxMat33 #foreign physx_sys;

PxMat33_new_1 :: (anon_param0: s32) -> physx_PxMat33 #foreign physx_sys;

PxMat33_new_2 :: (anon_param0: s32) -> physx_PxMat33 #foreign physx_sys;

PxMat33_new_3 :: (col0: *Vector3, col1: *Vector3, col2: *Vector3) -> physx_PxMat33 #foreign physx_sys;

PxMat33_new_4 :: (r: float) -> physx_PxMat33 #foreign physx_sys;

PxMat33_new_5 :: (values: *float) -> physx_PxMat33 #foreign physx_sys;

PxMat33_new_6 :: (q: *Quaternion) -> physx_PxMat33 #foreign physx_sys;

PxMat33_createDiagonal :: (d: *Vector3) -> physx_PxMat33 #foreign physx_sys;

PxMat33_outer :: (a: *Vector3, b: *Vector3) -> physx_PxMat33 #foreign physx_sys;

PxMat33_getTranspose :: (self: *physx_PxMat33) -> physx_PxMat33 #foreign physx_sys;

PxMat33_getInverse :: (self: *physx_PxMat33) -> physx_PxMat33 #foreign physx_sys;

PxMat33_getDeterminant :: (self: *physx_PxMat33) -> float #foreign physx_sys;

PxMat33_transform :: (self: *physx_PxMat33, other: *Vector3) -> Vector3 #foreign physx_sys;

PxMat33_transformTranspose :: (self: *physx_PxMat33, other: *Vector3) -> Vector3 #foreign physx_sys;

PxMat33_front :: (self: *physx_PxMat33) -> *float #foreign physx_sys;

PxBounds3_new :: () -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_new_1 :: (minimum: *Vector3, maximum: *Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_empty :: () -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_boundsOfPoints :: (v0: *Vector3, v1: *Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_centerExtents :: (center: *Vector3, extent: *Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_basisExtent :: (center: *Vector3, basis: *physx_PxMat33, extent: *Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_poseExtent :: (pose: *physx_PxTransform, extent: *Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformSafe :: (matrix: *physx_PxMat33, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformFast :: (matrix: *physx_PxMat33, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformSafe_1 :: (transform: *physx_PxTransform, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformFast_1 :: (transform: *physx_PxTransform, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_setEmpty :: (self: *physx_PxBounds3) -> void #foreign physx_sys "PxBounds3_setEmpty_mut";

PxBounds3_setMaximal :: (self: *physx_PxBounds3) -> void #foreign physx_sys "PxBounds3_setMaximal_mut";

PxBounds3_include :: (self: *physx_PxBounds3, v: *Vector3) -> void #foreign physx_sys "PxBounds3_include_mut";

PxBounds3_include :: (self: *physx_PxBounds3, b: *physx_PxBounds3) -> void #foreign physx_sys "PxBounds3_include_mut_1";

PxBounds3_isEmpty :: (self: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_intersects :: (self: *physx_PxBounds3, b: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_intersects1D :: (self: *physx_PxBounds3, a: *physx_PxBounds3, axis: u32) -> bool #foreign physx_sys;

PxBounds3_contains :: (self: *physx_PxBounds3, v: *Vector3) -> bool #foreign physx_sys;

PxBounds3_isInside :: (self: *physx_PxBounds3, box: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_getCenter :: (self: *physx_PxBounds3) -> Vector3 #foreign physx_sys;

PxBounds3_getCenter_1 :: (self: *physx_PxBounds3, axis: u32) -> float #foreign physx_sys;

PxBounds3_getExtents :: (self: *physx_PxBounds3, axis: u32) -> float #foreign physx_sys;

PxBounds3_getDimensions :: (self: *physx_PxBounds3) -> Vector3 #foreign physx_sys;

PxBounds3_getExtents_1 :: (self: *physx_PxBounds3) -> Vector3 #foreign physx_sys;

PxBounds3_scaleSafe :: (self: *physx_PxBounds3, scale: float) -> void #foreign physx_sys "PxBounds3_scaleSafe_mut";

PxBounds3_scaleFast :: (self: *physx_PxBounds3, scale: float) -> void #foreign physx_sys "PxBounds3_scaleFast_mut";

PxBounds3_fattenSafe :: (self: *physx_PxBounds3, distance: float) -> void #foreign physx_sys "PxBounds3_fattenSafe_mut";

PxBounds3_fattenFast :: (self: *physx_PxBounds3, distance: float) -> void #foreign physx_sys "PxBounds3_fattenFast_mut";

PxBounds3_isFinite :: (self: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_isValid :: (self: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_closestPoint :: (self: *physx_PxBounds3, p: *Vector3) -> Vector3 #foreign physx_sys;

PxErrorCallback_delete :: (self: *physx_PxErrorCallback) -> void #foreign physx_sys;

PxErrorCallback_reportError :: (self: *physx_PxErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys "PxErrorCallback_reportError_mut";

PxAllocationListener_onAllocation :: (self: *physx_PxAllocationListener, size: u64, typeName: *u8, filename: *u8, line: s32, allocatedMemory: *void) -> void #foreign physx_sys "PxAllocationListener_onAllocation_mut";

PxAllocationListener_onDeallocation :: (self: *physx_PxAllocationListener, allocatedMemory: *void) -> void #foreign physx_sys "PxAllocationListener_onDeallocation_mut";

PxBroadcastingAllocator_new_alloc :: (allocator: *physx_PxAllocatorCallback, error: *physx_PxErrorCallback) -> *physx_PxBroadcastingAllocator #foreign physx_sys;

PxBroadcastingAllocator_delete :: (self: *physx_PxBroadcastingAllocator) -> void #foreign physx_sys;

PxBroadcastingAllocator_allocate :: (self: *physx_PxBroadcastingAllocator, size: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign physx_sys "PxBroadcastingAllocator_allocate_mut";

PxBroadcastingAllocator_deallocate :: (self: *physx_PxBroadcastingAllocator, ptr: *void) -> void #foreign physx_sys "PxBroadcastingAllocator_deallocate_mut";

PxBroadcastingErrorCallback_new_alloc :: (errorCallback: *physx_PxErrorCallback) -> *physx_PxBroadcastingErrorCallback #foreign physx_sys;

PxBroadcastingErrorCallback_delete :: (self: *physx_PxBroadcastingErrorCallback) -> void #foreign physx_sys;

PxBroadcastingErrorCallback_reportError :: (self: *physx_PxBroadcastingErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys "PxBroadcastingErrorCallback_reportError_mut";

phys_PxEnableFPExceptions :: () -> void #foreign physx_sys;

phys_PxDisableFPExceptions :: () -> void #foreign physx_sys;

PxInputStream_read :: (self: *physx_PxInputStream, dest: *void, count: u32) -> u32 #foreign physx_sys "PxInputStream_read_mut";

PxInputStream_delete :: (self: *physx_PxInputStream) -> void #foreign physx_sys;

PxInputData_getLength :: (self: *physx_PxInputData) -> u32 #foreign physx_sys;

PxInputData_seek :: (self: *physx_PxInputData, offset: u32) -> void #foreign physx_sys "PxInputData_seek_mut";

PxInputData_tell :: (self: *physx_PxInputData) -> u32 #foreign physx_sys;

PxInputData_delete :: (self: *physx_PxInputData) -> void #foreign physx_sys;

PxOutputStream_write :: (self: *physx_PxOutputStream, src: *void, count: u32) -> u32 #foreign physx_sys "PxOutputStream_write_mut";

PxOutputStream_delete :: (self: *physx_PxOutputStream) -> void #foreign physx_sys;

PxVec4_new :: () -> Vector4 #foreign physx_sys;

PxVec4_new_1 :: (anon_param0: s32) -> Vector4 #foreign physx_sys;

PxVec4_new_2 :: (a: float) -> Vector4 #foreign physx_sys;

PxVec4_new_3 :: (nx: float, ny: float, nz: float, nw: float) -> Vector4 #foreign physx_sys;

PxVec4_new_4 :: (v: *Vector3, nw: float) -> Vector4 #foreign physx_sys;

PxVec4_new_5 :: (v: *float) -> Vector4 #foreign physx_sys;

PxVec4_isZero :: (self: *Vector4) -> bool #foreign physx_sys;

PxVec4_isFinite :: (self: *Vector4) -> bool #foreign physx_sys;

PxVec4_isNormalized :: (self: *Vector4) -> bool #foreign physx_sys;

PxVec4_magnitudeSquared :: (self: *Vector4) -> float #foreign physx_sys;

PxVec4_magnitude :: (self: *Vector4) -> float #foreign physx_sys;

PxVec4_dot :: (self: *Vector4, v: *Vector4) -> float #foreign physx_sys;

PxVec4_getNormalized :: (self: *Vector4) -> Vector4 #foreign physx_sys;

PxVec4_normalize :: (self: *Vector4) -> float #foreign physx_sys "PxVec4_normalize_mut";

PxVec4_multiply :: (self: *Vector4, a: *Vector4) -> Vector4 #foreign physx_sys;

PxVec4_minimum :: (self: *Vector4, v: *Vector4) -> Vector4 #foreign physx_sys;

PxVec4_maximum :: (self: *Vector4, v: *Vector4) -> Vector4 #foreign physx_sys;

PxVec4_getXYZ :: (self: *Vector4) -> Vector3 #foreign physx_sys;

PxMat44_new :: () -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_1 :: (anon_param0: s32) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_2 :: (anon_param0: s32) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_3 :: (col0: *Vector4, col1: *Vector4, col2: *Vector4, col3: *Vector4) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_4 :: (r: float) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_5 :: (col0: *Vector3, col1: *Vector3, col2: *Vector3, col3: *Vector3) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_6 :: (values: *float) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_7 :: (q: *Quaternion) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_8 :: (diagonal: *Vector4) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_9 :: (axes: *physx_PxMat33, position: *Vector3) -> physx_PxMat44 #foreign physx_sys;

PxMat44_new_10 :: (t: *physx_PxTransform) -> physx_PxMat44 #foreign physx_sys;

PxMat44_getTranspose :: (self: *physx_PxMat44) -> physx_PxMat44 #foreign physx_sys;

PxMat44_transform :: (self: *physx_PxMat44, other: *Vector4) -> Vector4 #foreign physx_sys;

PxMat44_transform_1 :: (self: *physx_PxMat44, other: *Vector3) -> Vector3 #foreign physx_sys;

PxMat44_rotate :: (self: *physx_PxMat44, other: *Vector4) -> Vector4 #foreign physx_sys;

PxMat44_rotate_1 :: (self: *physx_PxMat44, other: *Vector3) -> Vector3 #foreign physx_sys;

PxMat44_getBasis :: (self: *physx_PxMat44, num: u32) -> Vector3 #foreign physx_sys;

PxMat44_getPosition :: (self: *physx_PxMat44) -> Vector3 #foreign physx_sys;

PxMat44_setPosition :: (self: *physx_PxMat44, position: *Vector3) -> void #foreign physx_sys "PxMat44_setPosition_mut";

PxMat44_front :: (self: *physx_PxMat44) -> *float #foreign physx_sys;

PxMat44_scale :: (self: *physx_PxMat44, p: *Vector4) -> void #foreign physx_sys "PxMat44_scale_mut";

PxMat44_inverseRT :: (self: *physx_PxMat44) -> physx_PxMat44 #foreign physx_sys;

PxMat44_isFinite :: (self: *physx_PxMat44) -> bool #foreign physx_sys;

PxPlane_new :: () -> Plane3 #foreign physx_sys;

PxPlane_new_1 :: (nx: float, ny: float, nz: float, distance: float) -> Plane3 #foreign physx_sys;

PxPlane_new_2 :: (normal: *Vector3, distance: float) -> Plane3 #foreign physx_sys;

PxPlane_new_3 :: (point: *Vector3, normal: *Vector3) -> Plane3 #foreign physx_sys;

PxPlane_new_4 :: (p0: *Vector3, p1: *Vector3, p2: *Vector3) -> Plane3 #foreign physx_sys;

PxPlane_distance :: (self: *Plane3, p: *Vector3) -> float #foreign physx_sys;

PxPlane_contains :: (self: *Plane3, p: *Vector3) -> bool #foreign physx_sys;

PxPlane_project :: (self: *Plane3, p: *Vector3) -> Vector3 #foreign physx_sys;

PxPlane_pointInPlane :: (self: *Plane3) -> Vector3 #foreign physx_sys;

PxPlane_normalize :: (self: *Plane3) -> void #foreign physx_sys "PxPlane_normalize_mut";

PxPlane_transform :: (self: *Plane3, pose: *physx_PxTransform) -> Plane3 #foreign physx_sys;

PxPlane_inverseTransform :: (self: *Plane3, pose: *physx_PxTransform) -> Plane3 #foreign physx_sys;

phys_PxShortestRotation :: (from: *Vector3, target: *Vector3) -> Quaternion #foreign physx_sys;

phys_PxDiagonalize :: (m: *physx_PxMat33, axes: *Quaternion) -> Vector3 #foreign physx_sys;

phys_PxTransformFromSegment :: (p0: *Vector3, p1: *Vector3, halfHeight: *float) -> physx_PxTransform #foreign physx_sys;

phys_PxTransformFromPlaneEquation :: (plane: *Plane3) -> physx_PxTransform #foreign physx_sys;

phys_PxPlaneEquationFromTransform :: (pose: *physx_PxTransform) -> Plane3 #foreign physx_sys;

phys_PxSlerp :: (t: float, left: *Quaternion, right: *Quaternion) -> Quaternion #foreign physx_sys;

phys_PxIntegrateTransform :: (curTrans: *physx_PxTransform, linvel: *Vector3, angvel: *Vector3, timeStep: float, result: *physx_PxTransform) -> void #foreign physx_sys;

phys_PxExp :: (v: *Vector3) -> Quaternion #foreign physx_sys;

phys_PxOptimizeBoundingBox :: (basis: *physx_PxMat33) -> Vector3 #foreign physx_sys;

phys_PxLog :: (q: *Quaternion) -> Vector3 #foreign physx_sys;

phys_PxLargestAxis :: (v: *Vector3) -> u32 #foreign physx_sys;

phys_PxTanHalf :: (sin: float, cos: float) -> float #foreign physx_sys;

phys_PxEllipseClamp :: (point: *Vector3, radii: *Vector3) -> Vector3 #foreign physx_sys;

phys_PxSeparateSwingTwist :: (q: *Quaternion, swing: *Quaternion, twist: *Quaternion) -> void #foreign physx_sys;

phys_PxComputeAngle :: (v0: *Vector3, v1: *Vector3) -> float #foreign physx_sys;

phys_PxComputeBasisVectors :: (dir: *Vector3, right: *Vector3, up: *Vector3) -> void #foreign physx_sys;

phys_PxComputeBasisVectors_1 :: (p0: *Vector3, p1: *Vector3, dir: *Vector3, right: *Vector3, up: *Vector3) -> void #foreign physx_sys;

phys_PxGetNextIndex3 :: (i: u32) -> u32 #foreign physx_sys;

phys_computeBarycentric :: (a: *Vector3, b: *Vector3, c: *Vector3, d: *Vector3, p: *Vector3, bary: *Vector4) -> void #foreign physx_sys;

phys_computeBarycentric_1 :: (a: *Vector3, b: *Vector3, c: *Vector3, p: *Vector3, bary: *Vector4) -> void #foreign physx_sys;

Interpolation_PxLerp :: (a: float, b: float, t: float) -> float #foreign physx_sys;

Interpolation_PxBiLerp :: (f00: float, f10: float, f01: float, f11: float, tx: float, ty: float) -> float #foreign physx_sys;

Interpolation_PxTriLerp :: (f000: float, f100: float, f010: float, f110: float, f001: float, f101: float, f011: float, f111: float, tx: float, ty: float, tz: float) -> float #foreign physx_sys;

Interpolation_PxSDFIdx :: (i: u32, j: u32, k: u32, nbX: u32, nbY: u32) -> u32 #foreign physx_sys;

Interpolation_PxSDFSampleImpl :: (sdf: *float, localPos: *Vector3, sdfBoxLower: *Vector3, sdfBoxHigher: *Vector3, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, tolerance: float) -> float #foreign physx_sys;

phys_PxSdfSample :: (sdf: *float, localPos: *Vector3, sdfBoxLower: *Vector3, sdfBoxHigher: *Vector3, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, gradient: *Vector3, tolerance: float) -> float #foreign physx_sys;

PxMutexImpl_new_alloc :: () -> *physx_PxMutexImpl #foreign physx_sys;

PxMutexImpl_delete :: (self: *physx_PxMutexImpl) -> void #foreign physx_sys;

PxMutexImpl_lock :: (self: *physx_PxMutexImpl) -> void #foreign physx_sys "PxMutexImpl_lock_mut";

PxMutexImpl_trylock :: (self: *physx_PxMutexImpl) -> bool #foreign physx_sys "PxMutexImpl_trylock_mut";

PxMutexImpl_unlock :: (self: *physx_PxMutexImpl) -> void #foreign physx_sys "PxMutexImpl_unlock_mut";

PxMutexImpl_getSize :: () -> u32 #foreign physx_sys;

PxReadWriteLock_new_alloc :: () -> *physx_PxReadWriteLock #foreign physx_sys;

PxReadWriteLock_delete :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys;

PxReadWriteLock_lockReader :: (self: *physx_PxReadWriteLock, takeLock: bool) -> void #foreign physx_sys "PxReadWriteLock_lockReader_mut";

PxReadWriteLock_lockWriter :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys "PxReadWriteLock_lockWriter_mut";

PxReadWriteLock_unlockReader :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys "PxReadWriteLock_unlockReader_mut";

PxReadWriteLock_unlockWriter :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys "PxReadWriteLock_unlockWriter_mut";

PxProfilerCallback_zoneStart :: (self: *physx_PxProfilerCallback, eventName: *u8, detached: bool, contextId: u64) -> *void #foreign physx_sys "PxProfilerCallback_zoneStart_mut";

PxProfilerCallback_zoneEnd :: (self: *physx_PxProfilerCallback, profilerData: *void, eventName: *u8, detached: bool, contextId: u64) -> void #foreign physx_sys "PxProfilerCallback_zoneEnd_mut";

PxProfileScoped_new_alloc :: (callback: *physx_PxProfilerCallback, eventName: *u8, detached: bool, contextId: u64) -> *physx_PxProfileScoped #foreign physx_sys;

PxProfileScoped_delete :: (self: *physx_PxProfileScoped) -> void #foreign physx_sys;

PxSListEntry_new :: () -> physx_PxSListEntry #foreign physx_sys;

PxSListEntry_next :: (self: *physx_PxSListEntry) -> *physx_PxSListEntry #foreign physx_sys "PxSListEntry_next_mut";

PxSListImpl_new_alloc :: () -> *physx_PxSListImpl #foreign physx_sys;

PxSListImpl_delete :: (self: *physx_PxSListImpl) -> void #foreign physx_sys;

PxSListImpl_push :: (self: *physx_PxSListImpl, entry: *physx_PxSListEntry) -> void #foreign physx_sys "PxSListImpl_push_mut";

PxSListImpl_pop :: (self: *physx_PxSListImpl) -> *physx_PxSListEntry #foreign physx_sys "PxSListImpl_pop_mut";

PxSListImpl_flush :: (self: *physx_PxSListImpl) -> *physx_PxSListEntry #foreign physx_sys "PxSListImpl_flush_mut";

PxSListImpl_getSize :: () -> u32 #foreign physx_sys;

PxSyncImpl_new_alloc :: () -> *physx_PxSyncImpl #foreign physx_sys;

PxSyncImpl_delete :: (self: *physx_PxSyncImpl) -> void #foreign physx_sys;

PxSyncImpl_wait :: (self: *physx_PxSyncImpl, milliseconds: u32) -> bool #foreign physx_sys "PxSyncImpl_wait_mut";

PxSyncImpl_set :: (self: *physx_PxSyncImpl) -> void #foreign physx_sys "PxSyncImpl_set_mut";

PxSyncImpl_reset :: (self: *physx_PxSyncImpl) -> void #foreign physx_sys "PxSyncImpl_reset_mut";

PxSyncImpl_getSize :: () -> u32 #foreign physx_sys;

PxRunnable_new_alloc :: () -> *physx_PxRunnable #foreign physx_sys;

PxRunnable_delete :: (self: *physx_PxRunnable) -> void #foreign physx_sys;

PxRunnable_execute :: (self: *physx_PxRunnable) -> void #foreign physx_sys "PxRunnable_execute_mut";

phys_PxTlsAlloc :: () -> u32 #foreign physx_sys;

phys_PxTlsFree :: (index: u32) -> void #foreign physx_sys;

phys_PxTlsGet :: (index: u32) -> *void #foreign physx_sys;

phys_PxTlsGetValue :: (index: u32) -> u64 #foreign physx_sys;

phys_PxTlsSet :: (index: u32, value: *void) -> u32 #foreign physx_sys;

phys_PxTlsSetValue :: (index: u32, value: u64) -> u32 #foreign physx_sys;

PxCounterFrequencyToTensOfNanos_new :: (inNum: u64, inDenom: u64) -> physx_PxCounterFrequencyToTensOfNanos #foreign physx_sys;

PxCounterFrequencyToTensOfNanos_toTensOfNanos :: (self: *physx_PxCounterFrequencyToTensOfNanos, inCounter: u64) -> u64 #foreign physx_sys;

PxTime_getBootCounterFrequency :: () -> *physx_PxCounterFrequencyToTensOfNanos #foreign physx_sys;

PxTime_getCounterFrequency :: () -> physx_PxCounterFrequencyToTensOfNanos #foreign physx_sys;

PxTime_getCurrentCounterValue :: () -> u64 #foreign physx_sys;

PxTime_getCurrentTimeInTensOfNanoSeconds :: () -> u64 #foreign physx_sys;

PxTime_new :: () -> physx_PxTime #foreign physx_sys;

PxTime_getElapsedSeconds :: (self: *physx_PxTime) -> float64 #foreign physx_sys "PxTime_getElapsedSeconds_mut";

PxTime_peekElapsedSeconds :: (self: *physx_PxTime) -> float64 #foreign physx_sys "PxTime_peekElapsedSeconds_mut";

PxTime_getLastTime :: (self: *physx_PxTime) -> float64 #foreign physx_sys;

PxVec2_new :: () -> Vector2 #foreign physx_sys;

PxVec2_new_1 :: (anon_param0: s32) -> Vector2 #foreign physx_sys;

PxVec2_new_2 :: (a: float) -> Vector2 #foreign physx_sys;

PxVec2_new_3 :: (nx: float, ny: float) -> Vector2 #foreign physx_sys;

PxVec2_isZero :: (self: *Vector2) -> bool #foreign physx_sys;

PxVec2_isFinite :: (self: *Vector2) -> bool #foreign physx_sys;

PxVec2_isNormalized :: (self: *Vector2) -> bool #foreign physx_sys;

PxVec2_magnitudeSquared :: (self: *Vector2) -> float #foreign physx_sys;

PxVec2_magnitude :: (self: *Vector2) -> float #foreign physx_sys;

PxVec2_dot :: (self: *Vector2, v: *Vector2) -> float #foreign physx_sys;

PxVec2_getNormalized :: (self: *Vector2) -> Vector2 #foreign physx_sys;

PxVec2_normalize :: (self: *Vector2) -> float #foreign physx_sys "PxVec2_normalize_mut";

PxVec2_multiply :: (self: *Vector2, a: *Vector2) -> Vector2 #foreign physx_sys;

PxVec2_minimum :: (self: *Vector2, v: *Vector2) -> Vector2 #foreign physx_sys;

PxVec2_minElement :: (self: *Vector2) -> float #foreign physx_sys;

PxVec2_maximum :: (self: *Vector2, v: *Vector2) -> Vector2 #foreign physx_sys;

PxVec2_maxElement :: (self: *Vector2) -> float #foreign physx_sys;

PxStridedData_new :: () -> physx_PxStridedData #foreign physx_sys;

PxBoundedData_new :: () -> physx_PxBoundedData #foreign physx_sys;

PxDebugPoint_new :: (p: *Vector3, c: *u32) -> physx_PxDebugPoint #foreign physx_sys;

PxDebugLine_new :: (p0: *Vector3, p1: *Vector3, c: *u32) -> physx_PxDebugLine #foreign physx_sys;

PxDebugTriangle_new :: (p0: *Vector3, p1: *Vector3, p2: *Vector3, c: *u32) -> physx_PxDebugTriangle #foreign physx_sys;

PxDebugText_new :: () -> physx_PxDebugText #foreign physx_sys;

PxDebugText_new_1 :: (pos: *Vector3, sz: *float, clr: *u32, str: *u8) -> physx_PxDebugText #foreign physx_sys;

PxRenderBuffer_delete :: (self: *physx_PxRenderBuffer) -> void #foreign physx_sys;

PxRenderBuffer_getNbPoints :: (self: *physx_PxRenderBuffer) -> u32 #foreign physx_sys;

PxRenderBuffer_getPoints :: (self: *physx_PxRenderBuffer) -> *physx_PxDebugPoint #foreign physx_sys;

PxRenderBuffer_addPoint :: (self: *physx_PxRenderBuffer, point: *physx_PxDebugPoint) -> void #foreign physx_sys "PxRenderBuffer_addPoint_mut";

PxRenderBuffer_getNbLines :: (self: *physx_PxRenderBuffer) -> u32 #foreign physx_sys;

PxRenderBuffer_getLines :: (self: *physx_PxRenderBuffer) -> *physx_PxDebugLine #foreign physx_sys;

PxRenderBuffer_addLine :: (self: *physx_PxRenderBuffer, line: *physx_PxDebugLine) -> void #foreign physx_sys "PxRenderBuffer_addLine_mut";

PxRenderBuffer_reserveLines :: (self: *physx_PxRenderBuffer, nbLines: u32) -> *physx_PxDebugLine #foreign physx_sys "PxRenderBuffer_reserveLines_mut";

PxRenderBuffer_reservePoints :: (self: *physx_PxRenderBuffer, nbLines: u32) -> *physx_PxDebugPoint #foreign physx_sys "PxRenderBuffer_reservePoints_mut";

PxRenderBuffer_getNbTriangles :: (self: *physx_PxRenderBuffer) -> u32 #foreign physx_sys;

PxRenderBuffer_getTriangles :: (self: *physx_PxRenderBuffer) -> *physx_PxDebugTriangle #foreign physx_sys;

PxRenderBuffer_addTriangle :: (self: *physx_PxRenderBuffer, triangle: *physx_PxDebugTriangle) -> void #foreign physx_sys "PxRenderBuffer_addTriangle_mut";

PxRenderBuffer_append :: (self: *physx_PxRenderBuffer, other: *physx_PxRenderBuffer) -> void #foreign physx_sys "PxRenderBuffer_append_mut";

PxRenderBuffer_clear :: (self: *physx_PxRenderBuffer) -> void #foreign physx_sys "PxRenderBuffer_clear_mut";

PxRenderBuffer_shift :: (self: *physx_PxRenderBuffer, delta: *Vector3) -> void #foreign physx_sys "PxRenderBuffer_shift_mut";

PxRenderBuffer_empty :: (self: *physx_PxRenderBuffer) -> bool #foreign physx_sys;

PxProcessPxBaseCallback_delete :: (self: *physx_PxProcessPxBaseCallback) -> void #foreign physx_sys;

PxProcessPxBaseCallback_process :: (self: *physx_PxProcessPxBaseCallback, anon_param0: *physx_PxBase) -> void #foreign physx_sys "PxProcessPxBaseCallback_process_mut";

PxSerializationContext_registerReference :: (self: *physx_PxSerializationContext, base: *physx_PxBase, kind: u32, reference: u64) -> void #foreign physx_sys "PxSerializationContext_registerReference_mut";

PxSerializationContext_getCollection :: (self: *physx_PxSerializationContext) -> *physx_PxCollection #foreign physx_sys;

PxSerializationContext_writeData :: (self: *physx_PxSerializationContext, data: *void, size: u32) -> void #foreign physx_sys "PxSerializationContext_writeData_mut";

PxSerializationContext_alignData :: (self: *physx_PxSerializationContext, alignment: u32) -> void #foreign physx_sys "PxSerializationContext_alignData_mut";

PxSerializationContext_writeName :: (self: *physx_PxSerializationContext, name: *u8) -> void #foreign physx_sys "PxSerializationContext_writeName_mut";

PxDeserializationContext_resolveReference :: (self: *physx_PxDeserializationContext, kind: u32, reference: u64) -> *physx_PxBase #foreign physx_sys;

PxDeserializationContext_readName :: (self: *physx_PxDeserializationContext, name: **u8) -> void #foreign physx_sys "PxDeserializationContext_readName_mut";

PxDeserializationContext_alignExtraData :: (self: *physx_PxDeserializationContext, alignment: u32) -> void #foreign physx_sys "PxDeserializationContext_alignExtraData_mut";

PxSerializationRegistry_registerSerializer :: (self: *physx_PxSerializationRegistry, type: u16, serializer: *physx_PxSerializer) -> void #foreign physx_sys "PxSerializationRegistry_registerSerializer_mut";

PxSerializationRegistry_unregisterSerializer :: (self: *physx_PxSerializationRegistry, type: u16) -> *physx_PxSerializer #foreign physx_sys "PxSerializationRegistry_unregisterSerializer_mut";

PxSerializationRegistry_getSerializer :: (self: *physx_PxSerializationRegistry, type: u16) -> *physx_PxSerializer #foreign physx_sys;

PxSerializationRegistry_registerRepXSerializer :: (self: *physx_PxSerializationRegistry, type: u16, serializer: *physx_PxRepXSerializer) -> void #foreign physx_sys "PxSerializationRegistry_registerRepXSerializer_mut";

PxSerializationRegistry_unregisterRepXSerializer :: (self: *physx_PxSerializationRegistry, type: u16) -> *physx_PxRepXSerializer #foreign physx_sys "PxSerializationRegistry_unregisterRepXSerializer_mut";

PxSerializationRegistry_getRepXSerializer :: (self: *physx_PxSerializationRegistry, typeName: *u8) -> *physx_PxRepXSerializer #foreign physx_sys;

PxSerializationRegistry_release :: (self: *physx_PxSerializationRegistry) -> void #foreign physx_sys "PxSerializationRegistry_release_mut";

PxCollection_add :: (self: *physx_PxCollection, object: *physx_PxBase, id: u64) -> void #foreign physx_sys "PxCollection_add_mut";

PxCollection_remove :: (self: *physx_PxCollection, object: *physx_PxBase) -> void #foreign physx_sys "PxCollection_remove_mut";

PxCollection_contains :: (self: *physx_PxCollection, object: *physx_PxBase) -> bool #foreign physx_sys;

PxCollection_addId :: (self: *physx_PxCollection, object: *physx_PxBase, id: u64) -> void #foreign physx_sys "PxCollection_addId_mut";

PxCollection_removeId :: (self: *physx_PxCollection, id: u64) -> void #foreign physx_sys "PxCollection_removeId_mut";

PxCollection_add :: (self: *physx_PxCollection, collection: *physx_PxCollection) -> void #foreign physx_sys "PxCollection_add_mut_1";

PxCollection_remove :: (self: *physx_PxCollection, collection: *physx_PxCollection) -> void #foreign physx_sys "PxCollection_remove_mut_1";

PxCollection_getNbObjects :: (self: *physx_PxCollection) -> u32 #foreign physx_sys;

PxCollection_getObject :: (self: *physx_PxCollection, index: u32) -> *physx_PxBase #foreign physx_sys;

PxCollection_getObjects :: (self: *physx_PxCollection, userBuffer: **physx_PxBase, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxCollection_find :: (self: *physx_PxCollection, id: u64) -> *physx_PxBase #foreign physx_sys;

PxCollection_getNbIds :: (self: *physx_PxCollection) -> u32 #foreign physx_sys;

PxCollection_getIds :: (self: *physx_PxCollection, userBuffer: *u64, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxCollection_getId :: (self: *physx_PxCollection, object: *physx_PxBase) -> u64 #foreign physx_sys;

PxCollection_release :: (self: *physx_PxCollection) -> void #foreign physx_sys "PxCollection_release_mut";

phys_PxCreateCollection :: () -> *physx_PxCollection #foreign physx_sys;

PxBase_release :: (self: *physx_PxBase) -> void #foreign physx_sys "PxBase_release_mut";

PxBase_getConcreteTypeName :: (self: *physx_PxBase) -> *u8 #foreign physx_sys;

PxBase_getConcreteType :: (self: *physx_PxBase) -> u16 #foreign physx_sys;

PxBase_setBaseFlag :: (self: *physx_PxBase, flag: s32, value: bool) -> void #foreign physx_sys "PxBase_setBaseFlag_mut";

PxBase_setBaseFlags :: (self: *physx_PxBase, inFlags: u16) -> void #foreign physx_sys "PxBase_setBaseFlags_mut";

PxBase_getBaseFlags :: (self: *physx_PxBase) -> u16 #foreign physx_sys;

PxBase_isReleasable :: (self: *physx_PxBase) -> bool #foreign physx_sys;

PxRefCounted_release :: (self: *physx_PxRefCounted) -> void #foreign physx_sys "PxRefCounted_release_mut";

PxRefCounted_getReferenceCount :: (self: *physx_PxRefCounted) -> u32 #foreign physx_sys;

PxRefCounted_acquireReference :: (self: *physx_PxRefCounted) -> void #foreign physx_sys "PxRefCounted_acquireReference_mut";

PxTolerancesScale_new :: (defaultLength: float, defaultSpeed: float) -> physx_PxTolerancesScale #foreign physx_sys;

PxTolerancesScale_isValid :: (self: *physx_PxTolerancesScale) -> bool #foreign physx_sys;

PxStringTable_allocateStr :: (self: *physx_PxStringTable, inSrc: *u8) -> *u8 #foreign physx_sys "PxStringTable_allocateStr_mut";

PxStringTable_release :: (self: *physx_PxStringTable) -> void #foreign physx_sys "PxStringTable_release_mut";

PxSerializer_getConcreteTypeName :: (self: *physx_PxSerializer) -> *u8 #foreign physx_sys;

PxSerializer_requiresObjects :: (self: *physx_PxSerializer, anon_param0: *physx_PxBase, anon_param1: *physx_PxProcessPxBaseCallback) -> void #foreign physx_sys;

PxSerializer_isSubordinate :: (self: *physx_PxSerializer) -> bool #foreign physx_sys;

PxSerializer_exportExtraData :: (self: *physx_PxSerializer, anon_param0: *physx_PxBase, anon_param1: *physx_PxSerializationContext) -> void #foreign physx_sys;

PxSerializer_exportData :: (self: *physx_PxSerializer, anon_param0: *physx_PxBase, anon_param1: *physx_PxSerializationContext) -> void #foreign physx_sys;

PxSerializer_registerReferences :: (self: *physx_PxSerializer, obj: *physx_PxBase, s: *physx_PxSerializationContext) -> void #foreign physx_sys;

PxSerializer_getClassSize :: (self: *physx_PxSerializer) -> u64 #foreign physx_sys;

PxSerializer_createObject :: (self: *physx_PxSerializer, address: **u8, context_pod: *physx_PxDeserializationContext) -> *physx_PxBase #foreign physx_sys;

PxSerializer_delete :: (self: *physx_PxSerializer) -> void #foreign physx_sys;

PxInsertionCallback_buildObjectFromData :: (self: *physx_PxInsertionCallback, type: s32, data: *void) -> *physx_PxBase #foreign physx_sys "PxInsertionCallback_buildObjectFromData_mut";

PxTaskManager_setCpuDispatcher :: (self: *physx_PxTaskManager, ref: *physx_PxCpuDispatcher) -> void #foreign physx_sys "PxTaskManager_setCpuDispatcher_mut";

PxTaskManager_getCpuDispatcher :: (self: *physx_PxTaskManager) -> *physx_PxCpuDispatcher #foreign physx_sys;

PxTaskManager_resetDependencies :: (self: *physx_PxTaskManager) -> void #foreign physx_sys "PxTaskManager_resetDependencies_mut";

PxTaskManager_startSimulation :: (self: *physx_PxTaskManager) -> void #foreign physx_sys "PxTaskManager_startSimulation_mut";

PxTaskManager_stopSimulation :: (self: *physx_PxTaskManager) -> void #foreign physx_sys "PxTaskManager_stopSimulation_mut";

PxTaskManager_taskCompleted :: (self: *physx_PxTaskManager, task: *physx_PxTask) -> void #foreign physx_sys "PxTaskManager_taskCompleted_mut";

PxTaskManager_getNamedTask :: (self: *physx_PxTaskManager, name: *u8) -> u32 #foreign physx_sys "PxTaskManager_getNamedTask_mut";

PxTaskManager_submitNamedTask :: (self: *physx_PxTaskManager, task: *physx_PxTask, name: *u8, type: s32) -> u32 #foreign physx_sys "PxTaskManager_submitNamedTask_mut";

PxTaskManager_submitUnnamedTask :: (self: *physx_PxTaskManager, task: *physx_PxTask, type: s32) -> u32 #foreign physx_sys "PxTaskManager_submitUnnamedTask_mut";

PxTaskManager_getTaskFromID :: (self: *physx_PxTaskManager, id: u32) -> *physx_PxTask #foreign physx_sys "PxTaskManager_getTaskFromID_mut";

PxTaskManager_release :: (self: *physx_PxTaskManager) -> void #foreign physx_sys "PxTaskManager_release_mut";

PxTaskManager_createTaskManager :: (errorCallback: *physx_PxErrorCallback, anon_param1: *physx_PxCpuDispatcher) -> *physx_PxTaskManager #foreign physx_sys;

PxCpuDispatcher_submitTask :: (self: *physx_PxCpuDispatcher, task: *physx_PxBaseTask) -> void #foreign physx_sys "PxCpuDispatcher_submitTask_mut";

PxCpuDispatcher_getWorkerCount :: (self: *physx_PxCpuDispatcher) -> u32 #foreign physx_sys;

PxCpuDispatcher_delete :: (self: *physx_PxCpuDispatcher) -> void #foreign physx_sys;

PxBaseTask_run :: (self: *physx_PxBaseTask) -> void #foreign physx_sys "PxBaseTask_run_mut";

PxBaseTask_getName :: (self: *physx_PxBaseTask) -> *u8 #foreign physx_sys;

PxBaseTask_addReference :: (self: *physx_PxBaseTask) -> void #foreign physx_sys "PxBaseTask_addReference_mut";

PxBaseTask_removeReference :: (self: *physx_PxBaseTask) -> void #foreign physx_sys "PxBaseTask_removeReference_mut";

PxBaseTask_getReference :: (self: *physx_PxBaseTask) -> s32 #foreign physx_sys;

PxBaseTask_release :: (self: *physx_PxBaseTask) -> void #foreign physx_sys "PxBaseTask_release_mut";

PxBaseTask_getTaskManager :: (self: *physx_PxBaseTask) -> *physx_PxTaskManager #foreign physx_sys;

PxBaseTask_setContextId :: (self: *physx_PxBaseTask, id: u64) -> void #foreign physx_sys "PxBaseTask_setContextId_mut";

PxBaseTask_getContextId :: (self: *physx_PxBaseTask) -> u64 #foreign physx_sys;

PxTask_release :: (self: *physx_PxTask) -> void #foreign physx_sys "PxTask_release_mut";

PxTask_finishBefore :: (self: *physx_PxTask, taskID: u32) -> void #foreign physx_sys "PxTask_finishBefore_mut";

PxTask_startAfter :: (self: *physx_PxTask, taskID: u32) -> void #foreign physx_sys "PxTask_startAfter_mut";

PxTask_addReference :: (self: *physx_PxTask) -> void #foreign physx_sys "PxTask_addReference_mut";

PxTask_removeReference :: (self: *physx_PxTask) -> void #foreign physx_sys "PxTask_removeReference_mut";

PxTask_getReference :: (self: *physx_PxTask) -> s32 #foreign physx_sys;

PxTask_getTaskID :: (self: *physx_PxTask) -> u32 #foreign physx_sys;

PxTask_submitted :: (self: *physx_PxTask) -> void #foreign physx_sys "PxTask_submitted_mut";

PxLightCpuTask_setContinuation :: (self: *physx_PxLightCpuTask, tm: *physx_PxTaskManager, c: *physx_PxBaseTask) -> void #foreign physx_sys "PxLightCpuTask_setContinuation_mut";

PxLightCpuTask_setContinuation :: (self: *physx_PxLightCpuTask, c: *physx_PxBaseTask) -> void #foreign physx_sys "PxLightCpuTask_setContinuation_mut_1";

PxLightCpuTask_getContinuation :: (self: *physx_PxLightCpuTask) -> *physx_PxBaseTask #foreign physx_sys;

PxLightCpuTask_removeReference :: (self: *physx_PxLightCpuTask) -> void #foreign physx_sys "PxLightCpuTask_removeReference_mut";

PxLightCpuTask_getReference :: (self: *physx_PxLightCpuTask) -> s32 #foreign physx_sys;

PxLightCpuTask_addReference :: (self: *physx_PxLightCpuTask) -> void #foreign physx_sys "PxLightCpuTask_addReference_mut";

PxLightCpuTask_release :: (self: *physx_PxLightCpuTask) -> void #foreign physx_sys "PxLightCpuTask_release_mut";

PxGeometry_getType :: (self: *physx_PxGeometry) -> s32 #foreign physx_sys;

PxBoxGeometry_new :: (hx: float, hy: float, hz: float) -> physx_PxBoxGeometry #foreign physx_sys;

PxBoxGeometry_new_1 :: (halfExtents: Vector3) -> physx_PxBoxGeometry #foreign physx_sys;

PxBoxGeometry_isValid :: (self: *physx_PxBoxGeometry) -> bool #foreign physx_sys;

PxBVHRaycastCallback_delete :: (self: *physx_PxBVHRaycastCallback) -> void #foreign physx_sys;

PxBVHRaycastCallback_reportHit :: (self: *physx_PxBVHRaycastCallback, boundsIndex: u32, distance: *float) -> bool #foreign physx_sys "PxBVHRaycastCallback_reportHit_mut";

PxBVHOverlapCallback_delete :: (self: *physx_PxBVHOverlapCallback) -> void #foreign physx_sys;

PxBVHOverlapCallback_reportHit :: (self: *physx_PxBVHOverlapCallback, boundsIndex: u32) -> bool #foreign physx_sys "PxBVHOverlapCallback_reportHit_mut";

PxBVHTraversalCallback_delete :: (self: *physx_PxBVHTraversalCallback) -> void #foreign physx_sys;

PxBVHTraversalCallback_visitNode :: (self: *physx_PxBVHTraversalCallback, bounds: *physx_PxBounds3) -> bool #foreign physx_sys "PxBVHTraversalCallback_visitNode_mut";

PxBVHTraversalCallback_reportLeaf :: (self: *physx_PxBVHTraversalCallback, nbPrims: u32, prims: *u32) -> bool #foreign physx_sys "PxBVHTraversalCallback_reportLeaf_mut";

PxBVH_raycast :: (self: *physx_PxBVH, origin: *Vector3, unitDir: *Vector3, maxDist: float, cb: *physx_PxBVHRaycastCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_sweep :: (self: *physx_PxBVH, geom: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *Vector3, maxDist: float, cb: *physx_PxBVHRaycastCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_overlap :: (self: *physx_PxBVH, geom: *physx_PxGeometry, pose: *physx_PxTransform, cb: *physx_PxBVHOverlapCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_cull :: (self: *physx_PxBVH, nbPlanes: u32, planes: *Plane3, cb: *physx_PxBVHOverlapCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_getNbBounds :: (self: *physx_PxBVH) -> u32 #foreign physx_sys;

PxBVH_getBounds :: (self: *physx_PxBVH) -> *physx_PxBounds3 #foreign physx_sys;

PxBVH_getBoundsForModification :: (self: *physx_PxBVH) -> *physx_PxBounds3 #foreign physx_sys "PxBVH_getBoundsForModification_mut";

PxBVH_refit :: (self: *physx_PxBVH) -> void #foreign physx_sys "PxBVH_refit_mut";

PxBVH_updateBounds :: (self: *physx_PxBVH, boundsIndex: u32, newBounds: *physx_PxBounds3) -> bool #foreign physx_sys "PxBVH_updateBounds_mut";

PxBVH_partialRefit :: (self: *physx_PxBVH) -> void #foreign physx_sys "PxBVH_partialRefit_mut";

PxBVH_traverse :: (self: *physx_PxBVH, cb: *physx_PxBVHTraversalCallback) -> bool #foreign physx_sys;

PxBVH_getConcreteTypeName :: (self: *physx_PxBVH) -> *u8 #foreign physx_sys;

PxCapsuleGeometry_new :: (radius_: float, halfHeight_: float) -> physx_PxCapsuleGeometry #foreign physx_sys;

PxCapsuleGeometry_isValid :: (self: *physx_PxCapsuleGeometry) -> bool #foreign physx_sys;

PxConvexMesh_getNbVertices :: (self: *physx_PxConvexMesh) -> u32 #foreign physx_sys;

PxConvexMesh_getVertices :: (self: *physx_PxConvexMesh) -> *Vector3 #foreign physx_sys;

PxConvexMesh_getIndexBuffer :: (self: *physx_PxConvexMesh) -> *u8 #foreign physx_sys;

PxConvexMesh_getNbPolygons :: (self: *physx_PxConvexMesh) -> u32 #foreign physx_sys;

PxConvexMesh_getPolygonData :: (self: *physx_PxConvexMesh, index: u32, data: *physx_PxHullPolygon) -> bool #foreign physx_sys;

PxConvexMesh_release :: (self: *physx_PxConvexMesh) -> void #foreign physx_sys "PxConvexMesh_release_mut";

PxConvexMesh_getMassInformation :: (self: *physx_PxConvexMesh, mass: *float, localInertia: *physx_PxMat33, localCenterOfMass: *Vector3) -> void #foreign physx_sys;

PxConvexMesh_getLocalBounds :: (self: *physx_PxConvexMesh) -> physx_PxBounds3 #foreign physx_sys;

PxConvexMesh_getSDF :: (self: *physx_PxConvexMesh) -> *float #foreign physx_sys;

PxConvexMesh_getConcreteTypeName :: (self: *physx_PxConvexMesh) -> *u8 #foreign physx_sys;

PxConvexMesh_isGpuCompatible :: (self: *physx_PxConvexMesh) -> bool #foreign physx_sys;

PxMeshScale_new :: () -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_new_1 :: (r: float) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_new_2 :: (s: *Vector3) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_new_3 :: (s: *Vector3, r: *Quaternion) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_isIdentity :: (self: *physx_PxMeshScale) -> bool #foreign physx_sys;

PxMeshScale_getInverse :: (self: *physx_PxMeshScale) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_toMat33 :: (self: *physx_PxMeshScale) -> physx_PxMat33 #foreign physx_sys;

PxMeshScale_hasNegativeDeterminant :: (self: *physx_PxMeshScale) -> bool #foreign physx_sys;

PxMeshScale_transform :: (self: *physx_PxMeshScale, v: *Vector3) -> Vector3 #foreign physx_sys;

PxMeshScale_isValidForTriangleMesh :: (self: *physx_PxMeshScale) -> bool #foreign physx_sys;

PxMeshScale_isValidForConvexMesh :: (self: *physx_PxMeshScale) -> bool #foreign physx_sys;

PxConvexMeshGeometry_new :: (mesh: *physx_PxConvexMesh, scaling: *physx_PxMeshScale, flags: u8) -> physx_PxConvexMeshGeometry #foreign physx_sys;

PxConvexMeshGeometry_isValid :: (self: *physx_PxConvexMeshGeometry) -> bool #foreign physx_sys;

PxSphereGeometry_new :: (ir: float) -> physx_PxSphereGeometry #foreign physx_sys;

PxSphereGeometry_isValid :: (self: *physx_PxSphereGeometry) -> bool #foreign physx_sys;

PxPlaneGeometry_new :: () -> physx_PxPlaneGeometry #foreign physx_sys;

PxPlaneGeometry_isValid :: (self: *physx_PxPlaneGeometry) -> bool #foreign physx_sys;

PxTriangleMeshGeometry_new :: (mesh: *physx_PxTriangleMesh, scaling: *physx_PxMeshScale, flags: u8) -> physx_PxTriangleMeshGeometry #foreign physx_sys;

PxTriangleMeshGeometry_isValid :: (self: *physx_PxTriangleMeshGeometry) -> bool #foreign physx_sys;

PxHeightFieldGeometry_new :: (hf: *physx_PxHeightField, flags: u8, heightScale_: float, rowScale_: float, columnScale_: float) -> physx_PxHeightFieldGeometry #foreign physx_sys;

PxHeightFieldGeometry_isValid :: (self: *physx_PxHeightFieldGeometry) -> bool #foreign physx_sys;

PxParticleSystemGeometry_new :: () -> physx_PxParticleSystemGeometry #foreign physx_sys;

PxParticleSystemGeometry_isValid :: (self: *physx_PxParticleSystemGeometry) -> bool #foreign physx_sys;

PxHairSystemGeometry_new :: () -> physx_PxHairSystemGeometry #foreign physx_sys;

PxHairSystemGeometry_isValid :: (self: *physx_PxHairSystemGeometry) -> bool #foreign physx_sys;

PxTetrahedronMeshGeometry_new :: (mesh: *physx_PxTetrahedronMesh) -> physx_PxTetrahedronMeshGeometry #foreign physx_sys;

PxTetrahedronMeshGeometry_isValid :: (self: *physx_PxTetrahedronMeshGeometry) -> bool #foreign physx_sys;

PxQueryHit_new :: () -> physx_PxQueryHit #foreign physx_sys;

PxLocationHit_new :: () -> physx_PxLocationHit #foreign physx_sys;

PxLocationHit_hadInitialOverlap :: (self: *physx_PxLocationHit) -> bool #foreign physx_sys;

PxGeomRaycastHit_new :: () -> physx_PxGeomRaycastHit #foreign physx_sys;

PxGeomOverlapHit_new :: () -> physx_PxGeomOverlapHit #foreign physx_sys;

PxGeomSweepHit_new :: () -> physx_PxGeomSweepHit #foreign physx_sys;

PxGeomIndexPair_new :: () -> physx_PxGeomIndexPair #foreign physx_sys;

PxGeomIndexPair_new_1 :: (_id0: u32, _id1: u32) -> physx_PxGeomIndexPair #foreign physx_sys;

phys_PxCustomGeometry_getUniqueID :: () -> u32 #foreign physx_sys;

PxCustomGeometryType_new :: () -> physx_PxCustomGeometryType #foreign physx_sys;

PxCustomGeometryType_INVALID :: () -> physx_PxCustomGeometryType #foreign physx_sys;

PxCustomGeometryCallbacks_getCustomType :: (self: *physx_PxCustomGeometryCallbacks) -> physx_PxCustomGeometryType #foreign physx_sys;

PxCustomGeometryCallbacks_getLocalBounds :: (self: *physx_PxCustomGeometryCallbacks, geometry: *physx_PxGeometry) -> physx_PxBounds3 #foreign physx_sys;

PxCustomGeometryCallbacks_raycast :: (self: *physx_PxCustomGeometryCallbacks, origin: *Vector3, unitDir: *Vector3, geom: *physx_PxGeometry, pose: *physx_PxTransform, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *physx_PxGeomRaycastHit, stride: u32, threadContext: *physx_PxQueryThreadContext) -> u32 #foreign physx_sys;

PxCustomGeometryCallbacks_overlap :: (self: *physx_PxCustomGeometryCallbacks, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_sweep :: (self: *physx_PxCustomGeometryCallbacks, unitDir: *Vector3, maxDist: float, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, sweepHit: *physx_PxGeomSweepHit, hitFlags: u16, inflation: float, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_computeMassProperties :: (self: *physx_PxCustomGeometryCallbacks, geometry: *physx_PxGeometry, massProperties: *physx_PxMassProperties) -> void #foreign physx_sys;

PxCustomGeometryCallbacks_usePersistentContactManifold :: (self: *physx_PxCustomGeometryCallbacks, geometry: *physx_PxGeometry, breakingThreshold: *float) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_delete :: (self: *physx_PxCustomGeometryCallbacks) -> void #foreign physx_sys;

PxCustomGeometry_new :: () -> physx_PxCustomGeometry #foreign physx_sys;

PxCustomGeometry_new_1 :: (_callbacks: *physx_PxCustomGeometryCallbacks) -> physx_PxCustomGeometry #foreign physx_sys;

PxCustomGeometry_isValid :: (self: *physx_PxCustomGeometry) -> bool #foreign physx_sys;

PxCustomGeometry_getCustomType :: (self: *physx_PxCustomGeometry) -> physx_PxCustomGeometryType #foreign physx_sys;

PxGeometryHolder_getType :: (self: *physx_PxGeometryHolder) -> s32 #foreign physx_sys;

PxGeometryHolder_any :: (self: *physx_PxGeometryHolder) -> *physx_PxGeometry #foreign physx_sys "PxGeometryHolder_any_mut";

PxGeometryHolder_any :: (self: *physx_PxGeometryHolder) -> *physx_PxGeometry #foreign physx_sys;

PxGeometryHolder_sphere :: (self: *physx_PxGeometryHolder) -> *physx_PxSphereGeometry #foreign physx_sys "PxGeometryHolder_sphere_mut";

PxGeometryHolder_sphere :: (self: *physx_PxGeometryHolder) -> *physx_PxSphereGeometry #foreign physx_sys;

PxGeometryHolder_plane :: (self: *physx_PxGeometryHolder) -> *physx_PxPlaneGeometry #foreign physx_sys "PxGeometryHolder_plane_mut";

PxGeometryHolder_plane :: (self: *physx_PxGeometryHolder) -> *physx_PxPlaneGeometry #foreign physx_sys;

PxGeometryHolder_capsule :: (self: *physx_PxGeometryHolder) -> *physx_PxCapsuleGeometry #foreign physx_sys "PxGeometryHolder_capsule_mut";

PxGeometryHolder_capsule :: (self: *physx_PxGeometryHolder) -> *physx_PxCapsuleGeometry #foreign physx_sys;

PxGeometryHolder_box :: (self: *physx_PxGeometryHolder) -> *physx_PxBoxGeometry #foreign physx_sys "PxGeometryHolder_box_mut";

PxGeometryHolder_box :: (self: *physx_PxGeometryHolder) -> *physx_PxBoxGeometry #foreign physx_sys;

PxGeometryHolder_convexMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxConvexMeshGeometry #foreign physx_sys "PxGeometryHolder_convexMesh_mut";

PxGeometryHolder_convexMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxConvexMeshGeometry #foreign physx_sys;

PxGeometryHolder_tetMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxTetrahedronMeshGeometry #foreign physx_sys "PxGeometryHolder_tetMesh_mut";

PxGeometryHolder_tetMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxTetrahedronMeshGeometry #foreign physx_sys;

PxGeometryHolder_triangleMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxTriangleMeshGeometry #foreign physx_sys "PxGeometryHolder_triangleMesh_mut";

PxGeometryHolder_triangleMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxTriangleMeshGeometry #foreign physx_sys;

PxGeometryHolder_heightField :: (self: *physx_PxGeometryHolder) -> *physx_PxHeightFieldGeometry #foreign physx_sys "PxGeometryHolder_heightField_mut";

PxGeometryHolder_heightField :: (self: *physx_PxGeometryHolder) -> *physx_PxHeightFieldGeometry #foreign physx_sys;

PxGeometryHolder_particleSystem :: (self: *physx_PxGeometryHolder) -> *physx_PxParticleSystemGeometry #foreign physx_sys "PxGeometryHolder_particleSystem_mut";

PxGeometryHolder_particleSystem :: (self: *physx_PxGeometryHolder) -> *physx_PxParticleSystemGeometry #foreign physx_sys;

PxGeometryHolder_hairSystem :: (self: *physx_PxGeometryHolder) -> *physx_PxHairSystemGeometry #foreign physx_sys "PxGeometryHolder_hairSystem_mut";

PxGeometryHolder_hairSystem :: (self: *physx_PxGeometryHolder) -> *physx_PxHairSystemGeometry #foreign physx_sys;

PxGeometryHolder_custom :: (self: *physx_PxGeometryHolder) -> *physx_PxCustomGeometry #foreign physx_sys "PxGeometryHolder_custom_mut";

PxGeometryHolder_custom :: (self: *physx_PxGeometryHolder) -> *physx_PxCustomGeometry #foreign physx_sys;

PxGeometryHolder_storeAny :: (self: *physx_PxGeometryHolder, geometry: *physx_PxGeometry) -> void #foreign physx_sys "PxGeometryHolder_storeAny_mut";

PxGeometryHolder_new :: () -> physx_PxGeometryHolder #foreign physx_sys;

PxGeometryHolder_new_1 :: (geometry: *physx_PxGeometry) -> physx_PxGeometryHolder #foreign physx_sys;

PxGeometryQuery_raycast :: (origin: *Vector3, unitDir: *Vector3, geom: *physx_PxGeometry, pose: *physx_PxTransform, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *physx_PxGeomRaycastHit, stride: u32, queryFlags: u32, threadContext: *physx_PxQueryThreadContext) -> u32 #foreign physx_sys;

PxGeometryQuery_overlap :: (geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, queryFlags: u32, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxGeometryQuery_sweep :: (unitDir: *Vector3, maxDist: float, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, sweepHit: *physx_PxGeomSweepHit, hitFlags: u16, inflation: float, queryFlags: u32, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxGeometryQuery_computePenetration :: (direction: *Vector3, depth: *float, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, queryFlags: u32) -> bool #foreign physx_sys;

PxGeometryQuery_pointDistance :: (point: *Vector3, geom: *physx_PxGeometry, pose: *physx_PxTransform, closestPoint: *Vector3, closestIndex: *u32, queryFlags: u32) -> float #foreign physx_sys;

PxGeometryQuery_computeGeomBounds :: (bounds: *physx_PxBounds3, geom: *physx_PxGeometry, pose: *physx_PxTransform, offset: float, inflation: float, queryFlags: u32) -> void #foreign physx_sys;

PxGeometryQuery_isValid :: (geom: *physx_PxGeometry) -> bool #foreign physx_sys;

PxHeightFieldSample_tessFlag :: (self: *physx_PxHeightFieldSample) -> u8 #foreign physx_sys;

PxHeightFieldSample_setTessFlag :: (self: *physx_PxHeightFieldSample) -> void #foreign physx_sys "PxHeightFieldSample_setTessFlag_mut";

PxHeightFieldSample_clearTessFlag :: (self: *physx_PxHeightFieldSample) -> void #foreign physx_sys "PxHeightFieldSample_clearTessFlag_mut";

PxHeightField_release :: (self: *physx_PxHeightField) -> void #foreign physx_sys "PxHeightField_release_mut";

PxHeightField_saveCells :: (self: *physx_PxHeightField, destBuffer: *void, destBufferSize: u32) -> u32 #foreign physx_sys;

PxHeightField_modifySamples :: (self: *physx_PxHeightField, startCol: s32, startRow: s32, subfieldDesc: *physx_PxHeightFieldDesc, shrinkBounds: bool) -> bool #foreign physx_sys "PxHeightField_modifySamples_mut";

PxHeightField_getNbRows :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getNbColumns :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getFormat :: (self: *physx_PxHeightField) -> s32 #foreign physx_sys;

PxHeightField_getSampleStride :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getConvexEdgeThreshold :: (self: *physx_PxHeightField) -> float #foreign physx_sys;

PxHeightField_getFlags :: (self: *physx_PxHeightField) -> u16 #foreign physx_sys;

PxHeightField_getHeight :: (self: *physx_PxHeightField, x: float, z: float) -> float #foreign physx_sys;

PxHeightField_getTriangleMaterialIndex :: (self: *physx_PxHeightField, triangleIndex: u32) -> u16 #foreign physx_sys;

PxHeightField_getTriangleNormal :: (self: *physx_PxHeightField, triangleIndex: u32) -> Vector3 #foreign physx_sys;

PxHeightField_getSample :: (self: *physx_PxHeightField, row: u32, column: u32) -> *physx_PxHeightFieldSample #foreign physx_sys;

PxHeightField_getTimestamp :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getConcreteTypeName :: (self: *physx_PxHeightField) -> *u8 #foreign physx_sys;

PxHeightFieldDesc_new :: () -> physx_PxHeightFieldDesc #foreign physx_sys;

PxHeightFieldDesc_setToDefault :: (self: *physx_PxHeightFieldDesc) -> void #foreign physx_sys "PxHeightFieldDesc_setToDefault_mut";

PxHeightFieldDesc_isValid :: (self: *physx_PxHeightFieldDesc) -> bool #foreign physx_sys;

PxMeshQuery_getTriangle :: (triGeom: *physx_PxTriangleMeshGeometry, transform: *physx_PxTransform, triangleIndex: u32, triangle: *physx_PxTriangle, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign physx_sys;

PxMeshQuery_getTriangle_1 :: (hfGeom: *physx_PxHeightFieldGeometry, transform: *physx_PxTransform, triangleIndex: u32, triangle: *physx_PxTriangle, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign physx_sys;

PxMeshQuery_findOverlapTriangleMesh :: (geom: *physx_PxGeometry, geomPose: *physx_PxTransform, meshGeom: *physx_PxTriangleMeshGeometry, meshPose: *physx_PxTransform, results: *u32, maxResults: u32, startIndex: u32, overflow: *bool, queryFlags: u32) -> u32 #foreign physx_sys;

PxMeshQuery_findOverlapHeightField :: (geom: *physx_PxGeometry, geomPose: *physx_PxTransform, hfGeom: *physx_PxHeightFieldGeometry, hfPose: *physx_PxTransform, results: *u32, maxResults: u32, startIndex: u32, overflow: *bool, queryFlags: u32) -> u32 #foreign physx_sys;

PxMeshQuery_sweep :: (unitDir: *Vector3, distance: float, geom: *physx_PxGeometry, pose: *physx_PxTransform, triangleCount: u32, triangles: *physx_PxTriangle, sweepHit: *physx_PxGeomSweepHit, hitFlags: u16, cachedIndex: *u32, inflation: float, doubleSided: bool, queryFlags: u32) -> bool #foreign physx_sys;

PxSimpleTriangleMesh_new :: () -> physx_PxSimpleTriangleMesh #foreign physx_sys;

PxSimpleTriangleMesh_setToDefault :: (self: *physx_PxSimpleTriangleMesh) -> void #foreign physx_sys "PxSimpleTriangleMesh_setToDefault_mut";

PxSimpleTriangleMesh_isValid :: (self: *physx_PxSimpleTriangleMesh) -> bool #foreign physx_sys;

PxTriangle_new_alloc :: () -> *physx_PxTriangle #foreign physx_sys;

PxTriangle_new_alloc_1 :: (p0: *Vector3, p1: *Vector3, p2: *Vector3) -> *physx_PxTriangle #foreign physx_sys;

PxTriangle_delete :: (self: *physx_PxTriangle) -> void #foreign physx_sys;

PxTriangle_normal :: (self: *physx_PxTriangle, _normal: *Vector3) -> void #foreign physx_sys;

PxTriangle_denormalizedNormal :: (self: *physx_PxTriangle, _normal: *Vector3) -> void #foreign physx_sys;

PxTriangle_area :: (self: *physx_PxTriangle) -> float #foreign physx_sys;

PxTriangle_pointFromUV :: (self: *physx_PxTriangle, u: float, v: float) -> Vector3 #foreign physx_sys;

PxTrianglePadded_new_alloc :: () -> *physx_PxTrianglePadded #foreign physx_sys;

PxTrianglePadded_delete :: (self: *physx_PxTrianglePadded) -> void #foreign physx_sys;

PxTriangleMesh_getNbVertices :: (self: *physx_PxTriangleMesh) -> u32 #foreign physx_sys;

PxTriangleMesh_getVertices :: (self: *physx_PxTriangleMesh) -> *Vector3 #foreign physx_sys;

PxTriangleMesh_getVerticesForModification :: (self: *physx_PxTriangleMesh) -> *Vector3 #foreign physx_sys "PxTriangleMesh_getVerticesForModification_mut";

PxTriangleMesh_refitBVH :: (self: *physx_PxTriangleMesh) -> physx_PxBounds3 #foreign physx_sys "PxTriangleMesh_refitBVH_mut";

PxTriangleMesh_getNbTriangles :: (self: *physx_PxTriangleMesh) -> u32 #foreign physx_sys;

PxTriangleMesh_getTriangles :: (self: *physx_PxTriangleMesh) -> *void #foreign physx_sys;

PxTriangleMesh_getTriangleMeshFlags :: (self: *physx_PxTriangleMesh) -> u8 #foreign physx_sys;

PxTriangleMesh_getTrianglesRemap :: (self: *physx_PxTriangleMesh) -> *u32 #foreign physx_sys;

PxTriangleMesh_release :: (self: *physx_PxTriangleMesh) -> void #foreign physx_sys "PxTriangleMesh_release_mut";

PxTriangleMesh_getTriangleMaterialIndex :: (self: *physx_PxTriangleMesh, triangleIndex: u32) -> u16 #foreign physx_sys;

PxTriangleMesh_getLocalBounds :: (self: *physx_PxTriangleMesh) -> physx_PxBounds3 #foreign physx_sys;

PxTriangleMesh_getSDF :: (self: *physx_PxTriangleMesh) -> *float #foreign physx_sys;

PxTriangleMesh_getSDFDimensions :: (self: *physx_PxTriangleMesh, numX: *u32, numY: *u32, numZ: *u32) -> void #foreign physx_sys;

PxTriangleMesh_setPreferSDFProjection :: (self: *physx_PxTriangleMesh, preferProjection: bool) -> void #foreign physx_sys "PxTriangleMesh_setPreferSDFProjection_mut";

PxTriangleMesh_getPreferSDFProjection :: (self: *physx_PxTriangleMesh) -> bool #foreign physx_sys;

PxTriangleMesh_getMassInformation :: (self: *physx_PxTriangleMesh, mass: *float, localInertia: *physx_PxMat33, localCenterOfMass: *Vector3) -> void #foreign physx_sys;

PxTetrahedron_new_alloc :: () -> *physx_PxTetrahedron #foreign physx_sys;

PxTetrahedron_new_alloc_1 :: (p0: *Vector3, p1: *Vector3, p2: *Vector3, p3: *Vector3) -> *physx_PxTetrahedron #foreign physx_sys;

PxTetrahedron_delete :: (self: *physx_PxTetrahedron) -> void #foreign physx_sys;

PxSoftBodyAuxData_release :: (self: *physx_PxSoftBodyAuxData) -> void #foreign physx_sys "PxSoftBodyAuxData_release_mut";

PxTetrahedronMesh_getNbVertices :: (self: *physx_PxTetrahedronMesh) -> u32 #foreign physx_sys;

PxTetrahedronMesh_getVertices :: (self: *physx_PxTetrahedronMesh) -> *Vector3 #foreign physx_sys;

PxTetrahedronMesh_getNbTetrahedrons :: (self: *physx_PxTetrahedronMesh) -> u32 #foreign physx_sys;

PxTetrahedronMesh_getTetrahedrons :: (self: *physx_PxTetrahedronMesh) -> *void #foreign physx_sys;

PxTetrahedronMesh_getTetrahedronMeshFlags :: (self: *physx_PxTetrahedronMesh) -> u8 #foreign physx_sys;

PxTetrahedronMesh_getTetrahedraRemap :: (self: *physx_PxTetrahedronMesh) -> *u32 #foreign physx_sys;

PxTetrahedronMesh_getLocalBounds :: (self: *physx_PxTetrahedronMesh) -> physx_PxBounds3 #foreign physx_sys;

PxTetrahedronMesh_release :: (self: *physx_PxTetrahedronMesh) -> void #foreign physx_sys "PxTetrahedronMesh_release_mut";

PxSoftBodyMesh_getCollisionMesh :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxSoftBodyMesh_getCollisionMesh :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys "PxSoftBodyMesh_getCollisionMesh_mut";

PxSoftBodyMesh_getSimulationMesh :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxSoftBodyMesh_getSimulationMesh :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys "PxSoftBodyMesh_getSimulationMesh_mut";

PxSoftBodyMesh_getSoftBodyAuxData :: (self: *physx_PxSoftBodyMesh) -> *physx_PxSoftBodyAuxData #foreign physx_sys;

PxSoftBodyMesh_getSoftBodyAuxData :: (self: *physx_PxSoftBodyMesh) -> *physx_PxSoftBodyAuxData #foreign physx_sys "PxSoftBodyMesh_getSoftBodyAuxData_mut";

PxSoftBodyMesh_release :: (self: *physx_PxSoftBodyMesh) -> void #foreign physx_sys "PxSoftBodyMesh_release_mut";

PxCollisionMeshMappingData_release :: (self: *physx_PxCollisionMeshMappingData) -> void #foreign physx_sys "PxCollisionMeshMappingData_release_mut";

PxCollisionTetrahedronMeshData_getMesh :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxTetrahedronMeshData #foreign physx_sys;

PxCollisionTetrahedronMeshData_getMesh :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxTetrahedronMeshData #foreign physx_sys "PxCollisionTetrahedronMeshData_getMesh_mut";

PxCollisionTetrahedronMeshData_getData :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxSoftBodyCollisionData #foreign physx_sys;

PxCollisionTetrahedronMeshData_getData :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxSoftBodyCollisionData #foreign physx_sys "PxCollisionTetrahedronMeshData_getData_mut";

PxCollisionTetrahedronMeshData_release :: (self: *physx_PxCollisionTetrahedronMeshData) -> void #foreign physx_sys "PxCollisionTetrahedronMeshData_release_mut";

PxSimulationTetrahedronMeshData_getMesh :: (self: *physx_PxSimulationTetrahedronMeshData) -> *physx_PxTetrahedronMeshData #foreign physx_sys "PxSimulationTetrahedronMeshData_getMesh_mut";

PxSimulationTetrahedronMeshData_getData :: (self: *physx_PxSimulationTetrahedronMeshData) -> *physx_PxSoftBodySimulationData #foreign physx_sys "PxSimulationTetrahedronMeshData_getData_mut";

PxSimulationTetrahedronMeshData_release :: (self: *physx_PxSimulationTetrahedronMeshData) -> void #foreign physx_sys "PxSimulationTetrahedronMeshData_release_mut";

PxActor_release :: (self: *physx_PxActor) -> void #foreign physx_sys "PxActor_release_mut";

PxActor_getType :: (self: *physx_PxActor) -> s32 #foreign physx_sys;

PxActor_getScene :: (self: *physx_PxActor) -> *physx_PxScene #foreign physx_sys;

PxActor_setName :: (self: *physx_PxActor, name: *u8) -> void #foreign physx_sys "PxActor_setName_mut";

PxActor_getName :: (self: *physx_PxActor) -> *u8 #foreign physx_sys;

PxActor_getWorldBounds :: (self: *physx_PxActor, inflation: float) -> physx_PxBounds3 #foreign physx_sys;

PxActor_setActorFlag :: (self: *physx_PxActor, flag: s32, value: bool) -> void #foreign physx_sys "PxActor_setActorFlag_mut";

PxActor_setActorFlags :: (self: *physx_PxActor, inFlags: u8) -> void #foreign physx_sys "PxActor_setActorFlags_mut";

PxActor_getActorFlags :: (self: *physx_PxActor) -> u8 #foreign physx_sys;

PxActor_setDominanceGroup :: (self: *physx_PxActor, dominanceGroup: u8) -> void #foreign physx_sys "PxActor_setDominanceGroup_mut";

PxActor_getDominanceGroup :: (self: *physx_PxActor) -> u8 #foreign physx_sys;

PxActor_setOwnerClient :: (self: *physx_PxActor, inClient: u8) -> void #foreign physx_sys "PxActor_setOwnerClient_mut";

PxActor_getOwnerClient :: (self: *physx_PxActor) -> u8 #foreign physx_sys;

PxActor_getAggregate :: (self: *physx_PxActor) -> *physx_PxAggregate #foreign physx_sys;

phys_PxGetAggregateFilterHint :: (type: s32, enableSelfCollision: bool) -> u32 #foreign physx_sys;

phys_PxGetAggregateSelfCollisionBit :: (hint: u32) -> u32 #foreign physx_sys;

phys_PxGetAggregateType :: (hint: u32) -> s32 #foreign physx_sys;

PxAggregate_release :: (self: *physx_PxAggregate) -> void #foreign physx_sys "PxAggregate_release_mut";

PxAggregate_addActor :: (self: *physx_PxAggregate, actor: *physx_PxActor, bvh: *physx_PxBVH) -> bool #foreign physx_sys "PxAggregate_addActor_mut";

PxAggregate_removeActor :: (self: *physx_PxAggregate, actor: *physx_PxActor) -> bool #foreign physx_sys "PxAggregate_removeActor_mut";

PxAggregate_addArticulation :: (self: *physx_PxAggregate, articulation: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys "PxAggregate_addArticulation_mut";

PxAggregate_removeArticulation :: (self: *physx_PxAggregate, articulation: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys "PxAggregate_removeArticulation_mut";

PxAggregate_getNbActors :: (self: *physx_PxAggregate) -> u32 #foreign physx_sys;

PxAggregate_getMaxNbShapes :: (self: *physx_PxAggregate) -> u32 #foreign physx_sys;

PxAggregate_getActors :: (self: *physx_PxAggregate, userBuffer: **physx_PxActor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxAggregate_getScene :: (self: *physx_PxAggregate) -> *physx_PxScene #foreign physx_sys "PxAggregate_getScene_mut";

PxAggregate_getSelfCollision :: (self: *physx_PxAggregate) -> bool #foreign physx_sys;

PxAggregate_getConcreteTypeName :: (self: *physx_PxAggregate) -> *u8 #foreign physx_sys;

PxConstraintInvMassScale_new :: () -> physx_PxConstraintInvMassScale #foreign physx_sys;

PxConstraintInvMassScale_new_1 :: (lin0: float, ang0: float, lin1: float, ang1: float) -> physx_PxConstraintInvMassScale #foreign physx_sys;

PxConstraintVisualizer_visualizeJointFrames :: (self: *physx_PxConstraintVisualizer, parent: *physx_PxTransform, child: *physx_PxTransform) -> void #foreign physx_sys "PxConstraintVisualizer_visualizeJointFrames_mut";

PxConstraintVisualizer_visualizeLinearLimit :: (self: *physx_PxConstraintVisualizer, t0: *physx_PxTransform, t1: *physx_PxTransform, value: float, active: bool) -> void #foreign physx_sys "PxConstraintVisualizer_visualizeLinearLimit_mut";

PxConstraintVisualizer_visualizeAngularLimit :: (self: *physx_PxConstraintVisualizer, t0: *physx_PxTransform, lower: float, upper: float, active: bool) -> void #foreign physx_sys "PxConstraintVisualizer_visualizeAngularLimit_mut";

PxConstraintVisualizer_visualizeLimitCone :: (self: *physx_PxConstraintVisualizer, t: *physx_PxTransform, tanQSwingY: float, tanQSwingZ: float, active: bool) -> void #foreign physx_sys "PxConstraintVisualizer_visualizeLimitCone_mut";

PxConstraintVisualizer_visualizeDoubleCone :: (self: *physx_PxConstraintVisualizer, t: *physx_PxTransform, angle: float, active: bool) -> void #foreign physx_sys "PxConstraintVisualizer_visualizeDoubleCone_mut";

PxConstraintVisualizer_visualizeLine :: (self: *physx_PxConstraintVisualizer, p0: *Vector3, p1: *Vector3, color: u32) -> void #foreign physx_sys "PxConstraintVisualizer_visualizeLine_mut";

PxConstraintConnector_prepareData :: (self: *physx_PxConstraintConnector) -> *void #foreign physx_sys "PxConstraintConnector_prepareData_mut";

PxConstraintConnector_onConstraintRelease :: (self: *physx_PxConstraintConnector) -> void #foreign physx_sys "PxConstraintConnector_onConstraintRelease_mut";

PxConstraintConnector_onComShift :: (self: *physx_PxConstraintConnector, actor: u32) -> void #foreign physx_sys "PxConstraintConnector_onComShift_mut";

PxConstraintConnector_onOriginShift :: (self: *physx_PxConstraintConnector, shift: *Vector3) -> void #foreign physx_sys "PxConstraintConnector_onOriginShift_mut";

PxConstraintConnector_getSerializable :: (self: *physx_PxConstraintConnector) -> *physx_PxBase #foreign physx_sys "PxConstraintConnector_getSerializable_mut";

PxConstraintConnector_getConstantBlock :: (self: *physx_PxConstraintConnector) -> *void #foreign physx_sys;

PxConstraintConnector_connectToConstraint :: (self: *physx_PxConstraintConnector, anon_param0: *physx_PxConstraint) -> void #foreign physx_sys "PxConstraintConnector_connectToConstraint_mut";

PxConstraintConnector_delete :: (self: *physx_PxConstraintConnector) -> void #foreign physx_sys;

PxSolverBody_new :: () -> physx_PxSolverBody #foreign physx_sys;

PxSolverBodyData_projectVelocity :: (self: *physx_PxSolverBodyData, lin: *Vector3, ang: *Vector3) -> float #foreign physx_sys;

PxSolverConstraintPrepDesc_delete :: (self: *physx_PxSolverConstraintPrepDesc) -> void #foreign physx_sys;

PxConstraintAllocator_reserveConstraintData :: (self: *physx_PxConstraintAllocator, byteSize: u32) -> *u8 #foreign physx_sys "PxConstraintAllocator_reserveConstraintData_mut";

PxConstraintAllocator_reserveFrictionData :: (self: *physx_PxConstraintAllocator, byteSize: u32) -> *u8 #foreign physx_sys "PxConstraintAllocator_reserveFrictionData_mut";

PxConstraintAllocator_delete :: (self: *physx_PxConstraintAllocator) -> void #foreign physx_sys;

PxArticulationLimit_new :: () -> physx_PxArticulationLimit #foreign physx_sys;

PxArticulationLimit_new_1 :: (low_: float, high_: float) -> physx_PxArticulationLimit #foreign physx_sys;

PxArticulationDrive_new :: () -> physx_PxArticulationDrive #foreign physx_sys;

PxArticulationDrive_new_1 :: (stiffness_: float, damping_: float, maxForce_: float, driveType: s32) -> physx_PxArticulationDrive #foreign physx_sys;

PxTGSSolverBodyVel_projectVelocity :: (self: *physx_PxTGSSolverBodyVel, lin: *Vector3, ang: *Vector3) -> float #foreign physx_sys;

PxTGSSolverBodyData_projectVelocity :: (self: *physx_PxTGSSolverBodyData, linear: *Vector3, angular: *Vector3) -> float #foreign physx_sys;

PxTGSSolverConstraintPrepDesc_delete :: (self: *physx_PxTGSSolverConstraintPrepDesc) -> void #foreign physx_sys;

PxArticulationAttachment_setRestLength :: (self: *physx_PxArticulationAttachment, restLength: float) -> void #foreign physx_sys "PxArticulationAttachment_setRestLength_mut";

PxArticulationAttachment_getRestLength :: (self: *physx_PxArticulationAttachment) -> float #foreign physx_sys;

PxArticulationAttachment_setLimitParameters :: (self: *physx_PxArticulationAttachment, parameters: *physx_PxArticulationTendonLimit) -> void #foreign physx_sys "PxArticulationAttachment_setLimitParameters_mut";

PxArticulationAttachment_getLimitParameters :: (self: *physx_PxArticulationAttachment) -> physx_PxArticulationTendonLimit #foreign physx_sys;

PxArticulationAttachment_setRelativeOffset :: (self: *physx_PxArticulationAttachment, offset: *Vector3) -> void #foreign physx_sys "PxArticulationAttachment_setRelativeOffset_mut";

PxArticulationAttachment_getRelativeOffset :: (self: *physx_PxArticulationAttachment) -> Vector3 #foreign physx_sys;

PxArticulationAttachment_setCoefficient :: (self: *physx_PxArticulationAttachment, coefficient: float) -> void #foreign physx_sys "PxArticulationAttachment_setCoefficient_mut";

PxArticulationAttachment_getCoefficient :: (self: *physx_PxArticulationAttachment) -> float #foreign physx_sys;

PxArticulationAttachment_getLink :: (self: *physx_PxArticulationAttachment) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationAttachment_getParent :: (self: *physx_PxArticulationAttachment) -> *physx_PxArticulationAttachment #foreign physx_sys;

PxArticulationAttachment_isLeaf :: (self: *physx_PxArticulationAttachment) -> bool #foreign physx_sys;

PxArticulationAttachment_getTendon :: (self: *physx_PxArticulationAttachment) -> *physx_PxArticulationSpatialTendon #foreign physx_sys;

PxArticulationAttachment_release :: (self: *physx_PxArticulationAttachment) -> void #foreign physx_sys "PxArticulationAttachment_release_mut";

PxArticulationAttachment_getConcreteTypeName :: (self: *physx_PxArticulationAttachment) -> *u8 #foreign physx_sys;

PxArticulationTendonJoint_setCoefficient :: (self: *physx_PxArticulationTendonJoint, axis: s32, coefficient: float, recipCoefficient: float) -> void #foreign physx_sys "PxArticulationTendonJoint_setCoefficient_mut";

PxArticulationTendonJoint_getCoefficient :: (self: *physx_PxArticulationTendonJoint, axis: *s32, coefficient: *float, recipCoefficient: *float) -> void #foreign physx_sys;

PxArticulationTendonJoint_getLink :: (self: *physx_PxArticulationTendonJoint) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationTendonJoint_getParent :: (self: *physx_PxArticulationTendonJoint) -> *physx_PxArticulationTendonJoint #foreign physx_sys;

PxArticulationTendonJoint_getTendon :: (self: *physx_PxArticulationTendonJoint) -> *physx_PxArticulationFixedTendon #foreign physx_sys;

PxArticulationTendonJoint_release :: (self: *physx_PxArticulationTendonJoint) -> void #foreign physx_sys "PxArticulationTendonJoint_release_mut";

PxArticulationTendonJoint_getConcreteTypeName :: (self: *physx_PxArticulationTendonJoint) -> *u8 #foreign physx_sys;

PxArticulationTendon_setStiffness :: (self: *physx_PxArticulationTendon, stiffness: float) -> void #foreign physx_sys "PxArticulationTendon_setStiffness_mut";

PxArticulationTendon_getStiffness :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_setDamping :: (self: *physx_PxArticulationTendon, damping: float) -> void #foreign physx_sys "PxArticulationTendon_setDamping_mut";

PxArticulationTendon_getDamping :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_setLimitStiffness :: (self: *physx_PxArticulationTendon, stiffness: float) -> void #foreign physx_sys "PxArticulationTendon_setLimitStiffness_mut";

PxArticulationTendon_getLimitStiffness :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_setOffset :: (self: *physx_PxArticulationTendon, offset: float, autowake: bool) -> void #foreign physx_sys "PxArticulationTendon_setOffset_mut";

PxArticulationTendon_getOffset :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_getArticulation :: (self: *physx_PxArticulationTendon) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxArticulationTendon_release :: (self: *physx_PxArticulationTendon) -> void #foreign physx_sys "PxArticulationTendon_release_mut";

PxArticulationSpatialTendon_createAttachment :: (self: *physx_PxArticulationSpatialTendon, parent: *physx_PxArticulationAttachment, coefficient: float, relativeOffset: Vector3, link: *physx_PxArticulationLink) -> *physx_PxArticulationAttachment #foreign physx_sys "PxArticulationSpatialTendon_createAttachment_mut";

PxArticulationSpatialTendon_getAttachments :: (self: *physx_PxArticulationSpatialTendon, userBuffer: **physx_PxArticulationAttachment, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationSpatialTendon_getNbAttachments :: (self: *physx_PxArticulationSpatialTendon) -> u32 #foreign physx_sys;

PxArticulationSpatialTendon_getConcreteTypeName :: (self: *physx_PxArticulationSpatialTendon) -> *u8 #foreign physx_sys;

PxArticulationFixedTendon_createTendonJoint :: (self: *physx_PxArticulationFixedTendon, parent: *physx_PxArticulationTendonJoint, axis: s32, coefficient: float, recipCoefficient: float, link: *physx_PxArticulationLink) -> *physx_PxArticulationTendonJoint #foreign physx_sys "PxArticulationFixedTendon_createTendonJoint_mut";

PxArticulationFixedTendon_getTendonJoints :: (self: *physx_PxArticulationFixedTendon, userBuffer: **physx_PxArticulationTendonJoint, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationFixedTendon_getNbTendonJoints :: (self: *physx_PxArticulationFixedTendon) -> u32 #foreign physx_sys;

PxArticulationFixedTendon_setRestLength :: (self: *physx_PxArticulationFixedTendon, restLength: float) -> void #foreign physx_sys "PxArticulationFixedTendon_setRestLength_mut";

PxArticulationFixedTendon_getRestLength :: (self: *physx_PxArticulationFixedTendon) -> float #foreign physx_sys;

PxArticulationFixedTendon_setLimitParameters :: (self: *physx_PxArticulationFixedTendon, parameter: *physx_PxArticulationTendonLimit) -> void #foreign physx_sys "PxArticulationFixedTendon_setLimitParameters_mut";

PxArticulationFixedTendon_getLimitParameters :: (self: *physx_PxArticulationFixedTendon) -> physx_PxArticulationTendonLimit #foreign physx_sys;

PxArticulationFixedTendon_getConcreteTypeName :: (self: *physx_PxArticulationFixedTendon) -> *u8 #foreign physx_sys;

PxArticulationCache_new :: () -> physx_PxArticulationCache #foreign physx_sys;

PxArticulationCache_release :: (self: *physx_PxArticulationCache) -> void #foreign physx_sys "PxArticulationCache_release_mut";

PxArticulationSensor_release :: (self: *physx_PxArticulationSensor) -> void #foreign physx_sys "PxArticulationSensor_release_mut";

PxArticulationSensor_getForces :: (self: *physx_PxArticulationSensor) -> physx_PxSpatialForce #foreign physx_sys;

PxArticulationSensor_getRelativePose :: (self: *physx_PxArticulationSensor) -> physx_PxTransform #foreign physx_sys;

PxArticulationSensor_setRelativePose :: (self: *physx_PxArticulationSensor, pose: *physx_PxTransform) -> void #foreign physx_sys "PxArticulationSensor_setRelativePose_mut";

PxArticulationSensor_getLink :: (self: *physx_PxArticulationSensor) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationSensor_getIndex :: (self: *physx_PxArticulationSensor) -> u32 #foreign physx_sys;

PxArticulationSensor_getArticulation :: (self: *physx_PxArticulationSensor) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxArticulationSensor_getFlags :: (self: *physx_PxArticulationSensor) -> u8 #foreign physx_sys;

PxArticulationSensor_setFlag :: (self: *physx_PxArticulationSensor, flag: s32, enabled: bool) -> void #foreign physx_sys "PxArticulationSensor_setFlag_mut";

PxArticulationSensor_getConcreteTypeName :: (self: *physx_PxArticulationSensor) -> *u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getScene :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxScene #foreign physx_sys;

PxArticulationReducedCoordinate_setSolverIterationCounts :: (self: *physx_PxArticulationReducedCoordinate, minPositionIters: u32, minVelocityIters: u32) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setSolverIterationCounts_mut";

PxArticulationReducedCoordinate_getSolverIterationCounts :: (self: *physx_PxArticulationReducedCoordinate, minPositionIters: *u32, minVelocityIters: *u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_isSleeping :: (self: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys;

PxArticulationReducedCoordinate_setSleepThreshold :: (self: *physx_PxArticulationReducedCoordinate, threshold: float) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setSleepThreshold_mut";

PxArticulationReducedCoordinate_getSleepThreshold :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setStabilizationThreshold :: (self: *physx_PxArticulationReducedCoordinate, threshold: float) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setStabilizationThreshold_mut";

PxArticulationReducedCoordinate_getStabilizationThreshold :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setWakeCounter :: (self: *physx_PxArticulationReducedCoordinate, wakeCounterValue: float) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setWakeCounter_mut";

PxArticulationReducedCoordinate_getWakeCounter :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_wakeUp :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys "PxArticulationReducedCoordinate_wakeUp_mut";

PxArticulationReducedCoordinate_putToSleep :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys "PxArticulationReducedCoordinate_putToSleep_mut";

PxArticulationReducedCoordinate_setMaxCOMLinearVelocity :: (self: *physx_PxArticulationReducedCoordinate, maxLinearVelocity: float) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut";

PxArticulationReducedCoordinate_getMaxCOMLinearVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setMaxCOMAngularVelocity :: (self: *physx_PxArticulationReducedCoordinate, maxAngularVelocity: float) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut";

PxArticulationReducedCoordinate_getMaxCOMAngularVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_createLink :: (self: *physx_PxArticulationReducedCoordinate, parent: *physx_PxArticulationLink, pose: *physx_PxTransform) -> *physx_PxArticulationLink #foreign physx_sys "PxArticulationReducedCoordinate_createLink_mut";

PxArticulationReducedCoordinate_release :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys "PxArticulationReducedCoordinate_release_mut";

PxArticulationReducedCoordinate_getNbLinks :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getLinks :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbShapes :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_setName :: (self: *physx_PxArticulationReducedCoordinate, name: *u8) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setName_mut";

PxArticulationReducedCoordinate_getName :: (self: *physx_PxArticulationReducedCoordinate) -> *u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getWorldBounds :: (self: *physx_PxArticulationReducedCoordinate, inflation: float) -> physx_PxBounds3 #foreign physx_sys;

PxArticulationReducedCoordinate_getAggregate :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxAggregate #foreign physx_sys;

PxArticulationReducedCoordinate_setArticulationFlags :: (self: *physx_PxArticulationReducedCoordinate, flags: u8) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setArticulationFlags_mut";

PxArticulationReducedCoordinate_setArticulationFlag :: (self: *physx_PxArticulationReducedCoordinate, flag: s32, value: bool) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setArticulationFlag_mut";

PxArticulationReducedCoordinate_getArticulationFlags :: (self: *physx_PxArticulationReducedCoordinate) -> u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getDofs :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_createCache :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxArticulationCache #foreign physx_sys;

PxArticulationReducedCoordinate_getCacheDataSize :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_zeroCache :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_applyCache :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, flags: u32, autowake: bool) -> void #foreign physx_sys "PxArticulationReducedCoordinate_applyCache_mut";

PxArticulationReducedCoordinate_copyInternalStateToCache :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, flags: u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_packJointData :: (self: *physx_PxArticulationReducedCoordinate, maximum: *float, reduced: *float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_unpackJointData :: (self: *physx_PxArticulationReducedCoordinate, reduced: *float, maximum: *float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_commonInit :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedGravityForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedExternalForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeJointAcceleration :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeJointForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeDenseJacobian :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, nRows: *u32, nCols: *u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeCoefficientMatrix :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeLambda :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, initialState: *physx_PxArticulationCache, jointTorque: *float, maxIter: u32) -> bool #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedMassMatrix :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_addLoopJoint :: (self: *physx_PxArticulationReducedCoordinate, joint: *physx_PxConstraint) -> void #foreign physx_sys "PxArticulationReducedCoordinate_addLoopJoint_mut";

PxArticulationReducedCoordinate_removeLoopJoint :: (self: *physx_PxArticulationReducedCoordinate, joint: *physx_PxConstraint) -> void #foreign physx_sys "PxArticulationReducedCoordinate_removeLoopJoint_mut";

PxArticulationReducedCoordinate_getNbLoopJoints :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getLoopJoints :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxConstraint, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getCoefficientMatrixSize :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_setRootGlobalPose :: (self: *physx_PxArticulationReducedCoordinate, pose: *physx_PxTransform, autowake: bool) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setRootGlobalPose_mut";

PxArticulationReducedCoordinate_getRootGlobalPose :: (self: *physx_PxArticulationReducedCoordinate) -> physx_PxTransform #foreign physx_sys;

PxArticulationReducedCoordinate_setRootLinearVelocity :: (self: *physx_PxArticulationReducedCoordinate, linearVelocity: *Vector3, autowake: bool) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setRootLinearVelocity_mut";

PxArticulationReducedCoordinate_getRootLinearVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> Vector3 #foreign physx_sys;

PxArticulationReducedCoordinate_setRootAngularVelocity :: (self: *physx_PxArticulationReducedCoordinate, angularVelocity: *Vector3, autowake: bool) -> void #foreign physx_sys "PxArticulationReducedCoordinate_setRootAngularVelocity_mut";

PxArticulationReducedCoordinate_getRootAngularVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> Vector3 #foreign physx_sys;

PxArticulationReducedCoordinate_getLinkAcceleration :: (self: *physx_PxArticulationReducedCoordinate, linkId: u32) -> physx_PxSpatialVelocity #foreign physx_sys "PxArticulationReducedCoordinate_getLinkAcceleration_mut";

PxArticulationReducedCoordinate_getGpuArticulationIndex :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys "PxArticulationReducedCoordinate_getGpuArticulationIndex_mut";

PxArticulationReducedCoordinate_createSpatialTendon :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxArticulationSpatialTendon #foreign physx_sys "PxArticulationReducedCoordinate_createSpatialTendon_mut";

PxArticulationReducedCoordinate_createFixedTendon :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxArticulationFixedTendon #foreign physx_sys "PxArticulationReducedCoordinate_createFixedTendon_mut";

PxArticulationReducedCoordinate_createSensor :: (self: *physx_PxArticulationReducedCoordinate, link: *physx_PxArticulationLink, relativePose: *physx_PxTransform) -> *physx_PxArticulationSensor #foreign physx_sys "PxArticulationReducedCoordinate_createSensor_mut";

PxArticulationReducedCoordinate_getSpatialTendons :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationSpatialTendon, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbSpatialTendons :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys "PxArticulationReducedCoordinate_getNbSpatialTendons_mut";

PxArticulationReducedCoordinate_getFixedTendons :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationFixedTendon, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbFixedTendons :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys "PxArticulationReducedCoordinate_getNbFixedTendons_mut";

PxArticulationReducedCoordinate_getSensors :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationSensor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbSensors :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys "PxArticulationReducedCoordinate_getNbSensors_mut";

PxArticulationReducedCoordinate_updateKinematic :: (self: *physx_PxArticulationReducedCoordinate, flags: u8) -> void #foreign physx_sys "PxArticulationReducedCoordinate_updateKinematic_mut";

PxArticulationJointReducedCoordinate_getParentArticulationLink :: (self: *physx_PxArticulationJointReducedCoordinate) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationJointReducedCoordinate_setParentPose :: (self: *physx_PxArticulationJointReducedCoordinate, pose: *physx_PxTransform) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setParentPose_mut";

PxArticulationJointReducedCoordinate_getParentPose :: (self: *physx_PxArticulationJointReducedCoordinate) -> physx_PxTransform #foreign physx_sys;

PxArticulationJointReducedCoordinate_getChildArticulationLink :: (self: *physx_PxArticulationJointReducedCoordinate) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationJointReducedCoordinate_setChildPose :: (self: *physx_PxArticulationJointReducedCoordinate, pose: *physx_PxTransform) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setChildPose_mut";

PxArticulationJointReducedCoordinate_getChildPose :: (self: *physx_PxArticulationJointReducedCoordinate) -> physx_PxTransform #foreign physx_sys;

PxArticulationJointReducedCoordinate_setJointType :: (self: *physx_PxArticulationJointReducedCoordinate, jointType: s32) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setJointType_mut";

PxArticulationJointReducedCoordinate_getJointType :: (self: *physx_PxArticulationJointReducedCoordinate) -> s32 #foreign physx_sys;

PxArticulationJointReducedCoordinate_setMotion :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, motion: s32) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setMotion_mut";

PxArticulationJointReducedCoordinate_getMotion :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> s32 #foreign physx_sys;

PxArticulationJointReducedCoordinate_setLimitParams :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, limit: *physx_PxArticulationLimit) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setLimitParams_mut";

PxArticulationJointReducedCoordinate_getLimitParams :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> physx_PxArticulationLimit #foreign physx_sys;

PxArticulationJointReducedCoordinate_setDriveParams :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, drive: *physx_PxArticulationDrive) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setDriveParams_mut";

PxArticulationJointReducedCoordinate_getDriveParams :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> physx_PxArticulationDrive #foreign physx_sys;

PxArticulationJointReducedCoordinate_setDriveTarget :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, target: float, autowake: bool) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setDriveTarget_mut";

PxArticulationJointReducedCoordinate_getDriveTarget :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setDriveVelocity :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, targetVel: float, autowake: bool) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setDriveVelocity_mut";

PxArticulationJointReducedCoordinate_getDriveVelocity :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setArmature :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, armature: float) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setArmature_mut";

PxArticulationJointReducedCoordinate_getArmature :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setFrictionCoefficient :: (self: *physx_PxArticulationJointReducedCoordinate, coefficient: float) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut";

PxArticulationJointReducedCoordinate_getFrictionCoefficient :: (self: *physx_PxArticulationJointReducedCoordinate) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setMaxJointVelocity :: (self: *physx_PxArticulationJointReducedCoordinate, maxJointV: float) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut";

PxArticulationJointReducedCoordinate_getMaxJointVelocity :: (self: *physx_PxArticulationJointReducedCoordinate) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setJointPosition :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, jointPos: float) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setJointPosition_mut";

PxArticulationJointReducedCoordinate_getJointPosition :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setJointVelocity :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32, jointVel: float) -> void #foreign physx_sys "PxArticulationJointReducedCoordinate_setJointVelocity_mut";

PxArticulationJointReducedCoordinate_getJointVelocity :: (self: *physx_PxArticulationJointReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_getConcreteTypeName :: (self: *physx_PxArticulationJointReducedCoordinate) -> *u8 #foreign physx_sys;

PxShape_release :: (self: *physx_PxShape) -> void #foreign physx_sys "PxShape_release_mut";

PxShape_setGeometry :: (self: *physx_PxShape, geometry: *physx_PxGeometry) -> void #foreign physx_sys "PxShape_setGeometry_mut";

PxShape_getGeometry :: (self: *physx_PxShape) -> *physx_PxGeometry #foreign physx_sys;

PxShape_getActor :: (self: *physx_PxShape) -> *physx_PxRigidActor #foreign physx_sys;

PxShape_setLocalPose :: (self: *physx_PxShape, pose: *physx_PxTransform) -> void #foreign physx_sys "PxShape_setLocalPose_mut";

PxShape_getLocalPose :: (self: *physx_PxShape) -> physx_PxTransform #foreign physx_sys;

PxShape_setSimulationFilterData :: (self: *physx_PxShape, data: *physx_PxFilterData) -> void #foreign physx_sys "PxShape_setSimulationFilterData_mut";

PxShape_getSimulationFilterData :: (self: *physx_PxShape) -> physx_PxFilterData #foreign physx_sys;

PxShape_setQueryFilterData :: (self: *physx_PxShape, data: *physx_PxFilterData) -> void #foreign physx_sys "PxShape_setQueryFilterData_mut";

PxShape_getQueryFilterData :: (self: *physx_PxShape) -> physx_PxFilterData #foreign physx_sys;

PxShape_setMaterials :: (self: *physx_PxShape, materials: **physx_PxMaterial, materialCount: u16) -> void #foreign physx_sys "PxShape_setMaterials_mut";

PxShape_getNbMaterials :: (self: *physx_PxShape) -> u16 #foreign physx_sys;

PxShape_getMaterials :: (self: *physx_PxShape, userBuffer: **physx_PxMaterial, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxShape_getMaterialFromInternalFaceIndex :: (self: *physx_PxShape, faceIndex: u32) -> *physx_PxBaseMaterial #foreign physx_sys;

PxShape_setContactOffset :: (self: *physx_PxShape, contactOffset: float) -> void #foreign physx_sys "PxShape_setContactOffset_mut";

PxShape_getContactOffset :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setRestOffset :: (self: *physx_PxShape, restOffset: float) -> void #foreign physx_sys "PxShape_setRestOffset_mut";

PxShape_getRestOffset :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setDensityForFluid :: (self: *physx_PxShape, densityForFluid: float) -> void #foreign physx_sys "PxShape_setDensityForFluid_mut";

PxShape_getDensityForFluid :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setTorsionalPatchRadius :: (self: *physx_PxShape, radius: float) -> void #foreign physx_sys "PxShape_setTorsionalPatchRadius_mut";

PxShape_getTorsionalPatchRadius :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setMinTorsionalPatchRadius :: (self: *physx_PxShape, radius: float) -> void #foreign physx_sys "PxShape_setMinTorsionalPatchRadius_mut";

PxShape_getMinTorsionalPatchRadius :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setFlag :: (self: *physx_PxShape, flag: s32, value: bool) -> void #foreign physx_sys "PxShape_setFlag_mut";

PxShape_setFlags :: (self: *physx_PxShape, inFlags: u8) -> void #foreign physx_sys "PxShape_setFlags_mut";

PxShape_getFlags :: (self: *physx_PxShape) -> u8 #foreign physx_sys;

PxShape_isExclusive :: (self: *physx_PxShape) -> bool #foreign physx_sys;

PxShape_setName :: (self: *physx_PxShape, name: *u8) -> void #foreign physx_sys "PxShape_setName_mut";

PxShape_getName :: (self: *physx_PxShape) -> *u8 #foreign physx_sys;

PxShape_getConcreteTypeName :: (self: *physx_PxShape) -> *u8 #foreign physx_sys;

PxRigidActor_release :: (self: *physx_PxRigidActor) -> void #foreign physx_sys "PxRigidActor_release_mut";

PxRigidActor_getInternalActorIndex :: (self: *physx_PxRigidActor) -> u32 #foreign physx_sys;

PxRigidActor_getGlobalPose :: (self: *physx_PxRigidActor) -> physx_PxTransform #foreign physx_sys;

PxRigidActor_setGlobalPose :: (self: *physx_PxRigidActor, pose: *physx_PxTransform, autowake: bool) -> void #foreign physx_sys "PxRigidActor_setGlobalPose_mut";

PxRigidActor_attachShape :: (self: *physx_PxRigidActor, shape: *physx_PxShape) -> bool #foreign physx_sys "PxRigidActor_attachShape_mut";

PxRigidActor_detachShape :: (self: *physx_PxRigidActor, shape: *physx_PxShape, wakeOnLostTouch: bool) -> void #foreign physx_sys "PxRigidActor_detachShape_mut";

PxRigidActor_getNbShapes :: (self: *physx_PxRigidActor) -> u32 #foreign physx_sys;

PxRigidActor_getShapes :: (self: *physx_PxRigidActor, userBuffer: **physx_PxShape, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxRigidActor_getNbConstraints :: (self: *physx_PxRigidActor) -> u32 #foreign physx_sys;

PxRigidActor_getConstraints :: (self: *physx_PxRigidActor, userBuffer: **physx_PxConstraint, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxNodeIndex_new :: (id: u32, articLinkId: u32) -> physx_PxNodeIndex #foreign physx_sys;

PxNodeIndex_new_1 :: (id: u32) -> physx_PxNodeIndex #foreign physx_sys;

PxNodeIndex_index :: (self: *physx_PxNodeIndex) -> u32 #foreign physx_sys;

PxNodeIndex_articulationLinkId :: (self: *physx_PxNodeIndex) -> u32 #foreign physx_sys;

PxNodeIndex_isArticulation :: (self: *physx_PxNodeIndex) -> u32 #foreign physx_sys;

PxNodeIndex_isStaticBody :: (self: *physx_PxNodeIndex) -> bool #foreign physx_sys;

PxNodeIndex_isValid :: (self: *physx_PxNodeIndex) -> bool #foreign physx_sys;

PxNodeIndex_setIndices :: (self: *physx_PxNodeIndex, index: u32, articLinkId: u32) -> void #foreign physx_sys "PxNodeIndex_setIndices_mut";

PxNodeIndex_setIndices :: (self: *physx_PxNodeIndex, index: u32) -> void #foreign physx_sys "PxNodeIndex_setIndices_mut_1";

PxNodeIndex_getInd :: (self: *physx_PxNodeIndex) -> u64 #foreign physx_sys;

PxRigidBody_setCMassLocalPose :: (self: *physx_PxRigidBody, pose: *physx_PxTransform) -> void #foreign physx_sys "PxRigidBody_setCMassLocalPose_mut";

PxRigidBody_getCMassLocalPose :: (self: *physx_PxRigidBody) -> physx_PxTransform #foreign physx_sys;

PxRigidBody_setMass :: (self: *physx_PxRigidBody, mass: float) -> void #foreign physx_sys "PxRigidBody_setMass_mut";

PxRigidBody_getMass :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_getInvMass :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMassSpaceInertiaTensor :: (self: *physx_PxRigidBody, m: *Vector3) -> void #foreign physx_sys "PxRigidBody_setMassSpaceInertiaTensor_mut";

PxRigidBody_getMassSpaceInertiaTensor :: (self: *physx_PxRigidBody) -> Vector3 #foreign physx_sys;

PxRigidBody_getMassSpaceInvInertiaTensor :: (self: *physx_PxRigidBody) -> Vector3 #foreign physx_sys;

PxRigidBody_setLinearDamping :: (self: *physx_PxRigidBody, linDamp: float) -> void #foreign physx_sys "PxRigidBody_setLinearDamping_mut";

PxRigidBody_getLinearDamping :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setAngularDamping :: (self: *physx_PxRigidBody, angDamp: float) -> void #foreign physx_sys "PxRigidBody_setAngularDamping_mut";

PxRigidBody_getAngularDamping :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_getLinearVelocity :: (self: *physx_PxRigidBody) -> Vector3 #foreign physx_sys;

PxRigidBody_getAngularVelocity :: (self: *physx_PxRigidBody) -> Vector3 #foreign physx_sys;

PxRigidBody_setMaxLinearVelocity :: (self: *physx_PxRigidBody, maxLinVel: float) -> void #foreign physx_sys "PxRigidBody_setMaxLinearVelocity_mut";

PxRigidBody_getMaxLinearVelocity :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMaxAngularVelocity :: (self: *physx_PxRigidBody, maxAngVel: float) -> void #foreign physx_sys "PxRigidBody_setMaxAngularVelocity_mut";

PxRigidBody_getMaxAngularVelocity :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_addForce :: (self: *physx_PxRigidBody, force: *Vector3, mode: s32, autowake: bool) -> void #foreign physx_sys "PxRigidBody_addForce_mut";

PxRigidBody_addTorque :: (self: *physx_PxRigidBody, torque: *Vector3, mode: s32, autowake: bool) -> void #foreign physx_sys "PxRigidBody_addTorque_mut";

PxRigidBody_clearForce :: (self: *physx_PxRigidBody, mode: s32) -> void #foreign physx_sys "PxRigidBody_clearForce_mut";

PxRigidBody_clearTorque :: (self: *physx_PxRigidBody, mode: s32) -> void #foreign physx_sys "PxRigidBody_clearTorque_mut";

PxRigidBody_setForceAndTorque :: (self: *physx_PxRigidBody, force: *Vector3, torque: *Vector3, mode: s32) -> void #foreign physx_sys "PxRigidBody_setForceAndTorque_mut";

PxRigidBody_setRigidBodyFlag :: (self: *physx_PxRigidBody, flag: s32, value: bool) -> void #foreign physx_sys "PxRigidBody_setRigidBodyFlag_mut";

PxRigidBody_setRigidBodyFlags :: (self: *physx_PxRigidBody, inFlags: u16) -> void #foreign physx_sys "PxRigidBody_setRigidBodyFlags_mut";

PxRigidBody_getRigidBodyFlags :: (self: *physx_PxRigidBody) -> u16 #foreign physx_sys;

PxRigidBody_setMinCCDAdvanceCoefficient :: (self: *physx_PxRigidBody, advanceCoefficient: float) -> void #foreign physx_sys "PxRigidBody_setMinCCDAdvanceCoefficient_mut";

PxRigidBody_getMinCCDAdvanceCoefficient :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMaxDepenetrationVelocity :: (self: *physx_PxRigidBody, biasClamp: float) -> void #foreign physx_sys "PxRigidBody_setMaxDepenetrationVelocity_mut";

PxRigidBody_getMaxDepenetrationVelocity :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMaxContactImpulse :: (self: *physx_PxRigidBody, maxImpulse: float) -> void #foreign physx_sys "PxRigidBody_setMaxContactImpulse_mut";

PxRigidBody_getMaxContactImpulse :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setContactSlopCoefficient :: (self: *physx_PxRigidBody, slopCoefficient: float) -> void #foreign physx_sys "PxRigidBody_setContactSlopCoefficient_mut";

PxRigidBody_getContactSlopCoefficient :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_getInternalIslandNodeIndex :: (self: *physx_PxRigidBody) -> physx_PxNodeIndex #foreign physx_sys;

PxArticulationLink_release :: (self: *physx_PxArticulationLink) -> void #foreign physx_sys "PxArticulationLink_release_mut";

PxArticulationLink_getArticulation :: (self: *physx_PxArticulationLink) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxArticulationLink_getInboundJoint :: (self: *physx_PxArticulationLink) -> *physx_PxArticulationJointReducedCoordinate #foreign physx_sys;

PxArticulationLink_getInboundJointDof :: (self: *physx_PxArticulationLink) -> u32 #foreign physx_sys;

PxArticulationLink_getNbChildren :: (self: *physx_PxArticulationLink) -> u32 #foreign physx_sys;

PxArticulationLink_getLinkIndex :: (self: *physx_PxArticulationLink) -> u32 #foreign physx_sys;

PxArticulationLink_getChildren :: (self: *physx_PxArticulationLink, userBuffer: **physx_PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationLink_setCfmScale :: (self: *physx_PxArticulationLink, cfm: float) -> void #foreign physx_sys "PxArticulationLink_setCfmScale_mut";

PxArticulationLink_getCfmScale :: (self: *physx_PxArticulationLink) -> float #foreign physx_sys;

PxArticulationLink_getLinearVelocity :: (self: *physx_PxArticulationLink) -> Vector3 #foreign physx_sys;

PxArticulationLink_getAngularVelocity :: (self: *physx_PxArticulationLink) -> Vector3 #foreign physx_sys;

PxArticulationLink_getConcreteTypeName :: (self: *physx_PxArticulationLink) -> *u8 #foreign physx_sys;

PxConeLimitedConstraint_new :: () -> physx_PxConeLimitedConstraint #foreign physx_sys;

PxConstraint_release :: (self: *physx_PxConstraint) -> void #foreign physx_sys "PxConstraint_release_mut";

PxConstraint_getScene :: (self: *physx_PxConstraint) -> *physx_PxScene #foreign physx_sys;

PxConstraint_getActors :: (self: *physx_PxConstraint, actor0: **physx_PxRigidActor, actor1: **physx_PxRigidActor) -> void #foreign physx_sys;

PxConstraint_setActors :: (self: *physx_PxConstraint, actor0: *physx_PxRigidActor, actor1: *physx_PxRigidActor) -> void #foreign physx_sys "PxConstraint_setActors_mut";

PxConstraint_markDirty :: (self: *physx_PxConstraint) -> void #foreign physx_sys "PxConstraint_markDirty_mut";

PxConstraint_getFlags :: (self: *physx_PxConstraint) -> u16 #foreign physx_sys;

PxConstraint_setFlags :: (self: *physx_PxConstraint, flags: u16) -> void #foreign physx_sys "PxConstraint_setFlags_mut";

PxConstraint_setFlag :: (self: *physx_PxConstraint, flag: s32, value: bool) -> void #foreign physx_sys "PxConstraint_setFlag_mut";

PxConstraint_getForce :: (self: *physx_PxConstraint, linear: *Vector3, angular: *Vector3) -> void #foreign physx_sys;

PxConstraint_isValid :: (self: *physx_PxConstraint) -> bool #foreign physx_sys;

PxConstraint_setBreakForce :: (self: *physx_PxConstraint, linear: float, angular: float) -> void #foreign physx_sys "PxConstraint_setBreakForce_mut";

PxConstraint_getBreakForce :: (self: *physx_PxConstraint, linear: *float, angular: *float) -> void #foreign physx_sys;

PxConstraint_setMinResponseThreshold :: (self: *physx_PxConstraint, threshold: float) -> void #foreign physx_sys "PxConstraint_setMinResponseThreshold_mut";

PxConstraint_getMinResponseThreshold :: (self: *physx_PxConstraint) -> float #foreign physx_sys;

PxConstraint_getExternalReference :: (self: *physx_PxConstraint, typeID: *u32) -> *void #foreign physx_sys "PxConstraint_getExternalReference_mut";

PxConstraint_setConstraintFunctions :: (self: *physx_PxConstraint, connector: *physx_PxConstraintConnector, shaders: *physx_PxConstraintShaderTable) -> void #foreign physx_sys "PxConstraint_setConstraintFunctions_mut";

PxConstraint_getConcreteTypeName :: (self: *physx_PxConstraint) -> *u8 #foreign physx_sys;

PxContactStreamIterator_new :: (contactPatches: *u8, contactPoints: *u8, contactFaceIndices: *u32, nbPatches: u32, nbContacts: u32) -> physx_PxContactStreamIterator #foreign physx_sys;

PxContactStreamIterator_hasNextPatch :: (self: *physx_PxContactStreamIterator) -> bool #foreign physx_sys;

PxContactStreamIterator_getTotalContactCount :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_getTotalPatchCount :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_nextPatch :: (self: *physx_PxContactStreamIterator) -> void #foreign physx_sys "PxContactStreamIterator_nextPatch_mut";

PxContactStreamIterator_hasNextContact :: (self: *physx_PxContactStreamIterator) -> bool #foreign physx_sys;

PxContactStreamIterator_nextContact :: (self: *physx_PxContactStreamIterator) -> void #foreign physx_sys "PxContactStreamIterator_nextContact_mut";

PxContactStreamIterator_getContactNormal :: (self: *physx_PxContactStreamIterator) -> *Vector3 #foreign physx_sys;

PxContactStreamIterator_getInvMassScale0 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getInvMassScale1 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getInvInertiaScale0 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getInvInertiaScale1 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getMaxImpulse :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getTargetVel :: (self: *physx_PxContactStreamIterator) -> *Vector3 #foreign physx_sys;

PxContactStreamIterator_getContactPoint :: (self: *physx_PxContactStreamIterator) -> *Vector3 #foreign physx_sys;

PxContactStreamIterator_getSeparation :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getFaceIndex0 :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_getFaceIndex1 :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_getStaticFriction :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getDynamicFriction :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getRestitution :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getDamping :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getMaterialFlags :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_getMaterialIndex0 :: (self: *physx_PxContactStreamIterator) -> u16 #foreign physx_sys;

PxContactStreamIterator_getMaterialIndex1 :: (self: *physx_PxContactStreamIterator) -> u16 #foreign physx_sys;

PxContactStreamIterator_advanceToIndex :: (self: *physx_PxContactStreamIterator, initialIndex: u32) -> bool #foreign physx_sys "PxContactStreamIterator_advanceToIndex_mut";

PxContactSet_getPoint :: (self: *physx_PxContactSet, i: u32) -> *Vector3 #foreign physx_sys;

PxContactSet_setPoint :: (self: *physx_PxContactSet, i: u32, p: *Vector3) -> void #foreign physx_sys "PxContactSet_setPoint_mut";

PxContactSet_getNormal :: (self: *physx_PxContactSet, i: u32) -> *Vector3 #foreign physx_sys;

PxContactSet_setNormal :: (self: *physx_PxContactSet, i: u32, n: *Vector3) -> void #foreign physx_sys "PxContactSet_setNormal_mut";

PxContactSet_getSeparation :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setSeparation :: (self: *physx_PxContactSet, i: u32, s: float) -> void #foreign physx_sys "PxContactSet_setSeparation_mut";

PxContactSet_getTargetVelocity :: (self: *physx_PxContactSet, i: u32) -> *Vector3 #foreign physx_sys;

PxContactSet_setTargetVelocity :: (self: *physx_PxContactSet, i: u32, v: *Vector3) -> void #foreign physx_sys "PxContactSet_setTargetVelocity_mut";

PxContactSet_getInternalFaceIndex0 :: (self: *physx_PxContactSet, i: u32) -> u32 #foreign physx_sys;

PxContactSet_getInternalFaceIndex1 :: (self: *physx_PxContactSet, i: u32) -> u32 #foreign physx_sys;

PxContactSet_getMaxImpulse :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setMaxImpulse :: (self: *physx_PxContactSet, i: u32, s: float) -> void #foreign physx_sys "PxContactSet_setMaxImpulse_mut";

PxContactSet_getRestitution :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setRestitution :: (self: *physx_PxContactSet, i: u32, r: float) -> void #foreign physx_sys "PxContactSet_setRestitution_mut";

PxContactSet_getStaticFriction :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setStaticFriction :: (self: *physx_PxContactSet, i: u32, f: float) -> void #foreign physx_sys "PxContactSet_setStaticFriction_mut";

PxContactSet_getDynamicFriction :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setDynamicFriction :: (self: *physx_PxContactSet, i: u32, f: float) -> void #foreign physx_sys "PxContactSet_setDynamicFriction_mut";

PxContactSet_ignore :: (self: *physx_PxContactSet, i: u32) -> void #foreign physx_sys "PxContactSet_ignore_mut";

PxContactSet_size :: (self: *physx_PxContactSet) -> u32 #foreign physx_sys;

PxContactSet_getInvMassScale0 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_getInvMassScale1 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_getInvInertiaScale0 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_getInvInertiaScale1 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_setInvMassScale0 :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys "PxContactSet_setInvMassScale0_mut";

PxContactSet_setInvMassScale1 :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys "PxContactSet_setInvMassScale1_mut";

PxContactSet_setInvInertiaScale0 :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys "PxContactSet_setInvInertiaScale0_mut";

PxContactSet_setInvInertiaScale1 :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys "PxContactSet_setInvInertiaScale1_mut";

PxContactModifyCallback_onContactModify :: (self: *physx_PxContactModifyCallback, pairs: *physx_PxContactModifyPair, count: u32) -> void #foreign physx_sys "PxContactModifyCallback_onContactModify_mut";

PxCCDContactModifyCallback_onCCDContactModify :: (self: *physx_PxCCDContactModifyCallback, pairs: *physx_PxContactModifyPair, count: u32) -> void #foreign physx_sys "PxCCDContactModifyCallback_onCCDContactModify_mut";

PxDeletionListener_onRelease :: (self: *physx_PxDeletionListener, observed: *physx_PxBase, userData: *void, deletionEvent: s32) -> void #foreign physx_sys "PxDeletionListener_onRelease_mut";

PxBaseMaterial_isKindOf :: (self: *physx_PxBaseMaterial, name: *u8) -> bool #foreign physx_sys;

PxFEMMaterial_setYoungsModulus :: (self: *physx_PxFEMMaterial, young: float) -> void #foreign physx_sys "PxFEMMaterial_setYoungsModulus_mut";

PxFEMMaterial_getYoungsModulus :: (self: *physx_PxFEMMaterial) -> float #foreign physx_sys;

PxFEMMaterial_setPoissons :: (self: *physx_PxFEMMaterial, poisson: float) -> void #foreign physx_sys "PxFEMMaterial_setPoissons_mut";

PxFEMMaterial_getPoissons :: (self: *physx_PxFEMMaterial) -> float #foreign physx_sys;

PxFEMMaterial_setDynamicFriction :: (self: *physx_PxFEMMaterial, dynamicFriction: float) -> void #foreign physx_sys "PxFEMMaterial_setDynamicFriction_mut";

PxFEMMaterial_getDynamicFriction :: (self: *physx_PxFEMMaterial) -> float #foreign physx_sys;

PxFilterData_new :: (anon_param0: s32) -> physx_PxFilterData #foreign physx_sys;

PxFilterData_new_1 :: () -> physx_PxFilterData #foreign physx_sys;

PxFilterData_new_2 :: (w0: u32, w1: u32, w2: u32, w3: u32) -> physx_PxFilterData #foreign physx_sys;

PxFilterData_setToDefault :: (self: *physx_PxFilterData) -> void #foreign physx_sys "PxFilterData_setToDefault_mut";

phys_PxGetFilterObjectType :: (attr: u32) -> s32 #foreign physx_sys;

phys_PxFilterObjectIsKinematic :: (attr: u32) -> bool #foreign physx_sys;

phys_PxFilterObjectIsTrigger :: (attr: u32) -> bool #foreign physx_sys;

PxSimulationFilterCallback_pairFound :: (self: *physx_PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: physx_PxFilterData, a0: *physx_PxActor, s0: *physx_PxShape, attributes1: u32, filterData1: physx_PxFilterData, a1: *physx_PxActor, s1: *physx_PxShape, pairFlags: *u16) -> u16 #foreign physx_sys "PxSimulationFilterCallback_pairFound_mut";

PxSimulationFilterCallback_pairLost :: (self: *physx_PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: physx_PxFilterData, attributes1: u32, filterData1: physx_PxFilterData, objectRemoved: bool) -> void #foreign physx_sys "PxSimulationFilterCallback_pairLost_mut";

PxSimulationFilterCallback_statusChange :: (self: *physx_PxSimulationFilterCallback, pairID: *u32, pairFlags: *u16, filterFlags: *u16) -> bool #foreign physx_sys "PxSimulationFilterCallback_statusChange_mut";

PxLockedData_getDataAccessFlags :: (self: *physx_PxLockedData) -> u8 #foreign physx_sys "PxLockedData_getDataAccessFlags_mut";

PxLockedData_unlock :: (self: *physx_PxLockedData) -> void #foreign physx_sys "PxLockedData_unlock_mut";

PxLockedData_delete :: (self: *physx_PxLockedData) -> void #foreign physx_sys;

PxMaterial_setDynamicFriction :: (self: *physx_PxMaterial, coef: float) -> void #foreign physx_sys "PxMaterial_setDynamicFriction_mut";

PxMaterial_getDynamicFriction :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setStaticFriction :: (self: *physx_PxMaterial, coef: float) -> void #foreign physx_sys "PxMaterial_setStaticFriction_mut";

PxMaterial_getStaticFriction :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setRestitution :: (self: *physx_PxMaterial, rest: float) -> void #foreign physx_sys "PxMaterial_setRestitution_mut";

PxMaterial_getRestitution :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setDamping :: (self: *physx_PxMaterial, damping: float) -> void #foreign physx_sys "PxMaterial_setDamping_mut";

PxMaterial_getDamping :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setFlag :: (self: *physx_PxMaterial, flag: s32, b: bool) -> void #foreign physx_sys "PxMaterial_setFlag_mut";

PxMaterial_setFlags :: (self: *physx_PxMaterial, flags: u16) -> void #foreign physx_sys "PxMaterial_setFlags_mut";

PxMaterial_getFlags :: (self: *physx_PxMaterial) -> u16 #foreign physx_sys;

PxMaterial_setFrictionCombineMode :: (self: *physx_PxMaterial, combMode: s32) -> void #foreign physx_sys "PxMaterial_setFrictionCombineMode_mut";

PxMaterial_getFrictionCombineMode :: (self: *physx_PxMaterial) -> s32 #foreign physx_sys;

PxMaterial_setRestitutionCombineMode :: (self: *physx_PxMaterial, combMode: s32) -> void #foreign physx_sys "PxMaterial_setRestitutionCombineMode_mut";

PxMaterial_getRestitutionCombineMode :: (self: *physx_PxMaterial) -> s32 #foreign physx_sys;

PxMaterial_getConcreteTypeName :: (self: *physx_PxMaterial) -> *u8 #foreign physx_sys;

PxDiffuseParticleParams_new :: () -> physx_PxDiffuseParticleParams #foreign physx_sys;

PxDiffuseParticleParams_setToDefault :: (self: *physx_PxDiffuseParticleParams) -> void #foreign physx_sys "PxDiffuseParticleParams_setToDefault_mut";

PxParticleMaterial_setFriction :: (self: *physx_PxParticleMaterial, friction: float) -> void #foreign physx_sys "PxParticleMaterial_setFriction_mut";

PxParticleMaterial_getFriction :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setDamping :: (self: *physx_PxParticleMaterial, damping: float) -> void #foreign physx_sys "PxParticleMaterial_setDamping_mut";

PxParticleMaterial_getDamping :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setAdhesion :: (self: *physx_PxParticleMaterial, adhesion: float) -> void #foreign physx_sys "PxParticleMaterial_setAdhesion_mut";

PxParticleMaterial_getAdhesion :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setGravityScale :: (self: *physx_PxParticleMaterial, scale: float) -> void #foreign physx_sys "PxParticleMaterial_setGravityScale_mut";

PxParticleMaterial_getGravityScale :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setAdhesionRadiusScale :: (self: *physx_PxParticleMaterial, scale: float) -> void #foreign physx_sys "PxParticleMaterial_setAdhesionRadiusScale_mut";

PxParticleMaterial_getAdhesionRadiusScale :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxPhysics_release :: (self: *physx_PxPhysics) -> void #foreign physx_sys "PxPhysics_release_mut";

PxPhysics_getFoundation :: (self: *physx_PxPhysics) -> *physx_PxFoundation #foreign physx_sys "PxPhysics_getFoundation_mut";

PxPhysics_createAggregate :: (self: *physx_PxPhysics, maxActor: u32, maxShape: u32, filterHint: u32) -> *physx_PxAggregate #foreign physx_sys "PxPhysics_createAggregate_mut";

PxPhysics_getTolerancesScale :: (self: *physx_PxPhysics) -> *physx_PxTolerancesScale #foreign physx_sys;

PxPhysics_createTriangleMesh :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxTriangleMesh #foreign physx_sys "PxPhysics_createTriangleMesh_mut";

PxPhysics_getNbTriangleMeshes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getTriangleMeshes :: (self: *physx_PxPhysics, userBuffer: **physx_PxTriangleMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createTetrahedronMesh :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxTetrahedronMesh #foreign physx_sys "PxPhysics_createTetrahedronMesh_mut";

PxPhysics_createSoftBodyMesh :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxSoftBodyMesh #foreign physx_sys "PxPhysics_createSoftBodyMesh_mut";

PxPhysics_getNbTetrahedronMeshes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getTetrahedronMeshes :: (self: *physx_PxPhysics, userBuffer: **physx_PxTetrahedronMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createHeightField :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxHeightField #foreign physx_sys "PxPhysics_createHeightField_mut";

PxPhysics_getNbHeightFields :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getHeightFields :: (self: *physx_PxPhysics, userBuffer: **physx_PxHeightField, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createConvexMesh :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxConvexMesh #foreign physx_sys "PxPhysics_createConvexMesh_mut";

PxPhysics_getNbConvexMeshes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getConvexMeshes :: (self: *physx_PxPhysics, userBuffer: **physx_PxConvexMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createBVH :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxBVH #foreign physx_sys "PxPhysics_createBVH_mut";

PxPhysics_getNbBVHs :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getBVHs :: (self: *physx_PxPhysics, userBuffer: **physx_PxBVH, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createScene :: (self: *physx_PxPhysics, sceneDesc: *physx_PxSceneDesc) -> *physx_PxScene #foreign physx_sys "PxPhysics_createScene_mut";

PxPhysics_getNbScenes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getScenes :: (self: *physx_PxPhysics, userBuffer: **physx_PxScene, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createRigidStatic :: (self: *physx_PxPhysics, pose: *physx_PxTransform) -> *physx_PxRigidStatic #foreign physx_sys "PxPhysics_createRigidStatic_mut";

PxPhysics_createRigidDynamic :: (self: *physx_PxPhysics, pose: *physx_PxTransform) -> *physx_PxRigidDynamic #foreign physx_sys "PxPhysics_createRigidDynamic_mut";

PxPhysics_createPruningStructure :: (self: *physx_PxPhysics, actors: **physx_PxRigidActor, nbActors: u32) -> *physx_PxPruningStructure #foreign physx_sys "PxPhysics_createPruningStructure_mut";

PxPhysics_createShape :: (self: *physx_PxPhysics, geometry: *physx_PxGeometry, material: *physx_PxMaterial, isExclusive: bool, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys "PxPhysics_createShape_mut";

PxPhysics_createShape :: (self: *physx_PxPhysics, geometry: *physx_PxGeometry, materials: **physx_PxMaterial, materialCount: u16, isExclusive: bool, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys "PxPhysics_createShape_mut_1";

PxPhysics_getNbShapes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getShapes :: (self: *physx_PxPhysics, userBuffer: **physx_PxShape, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createConstraint :: (self: *physx_PxPhysics, actor0: *physx_PxRigidActor, actor1: *physx_PxRigidActor, connector: *physx_PxConstraintConnector, shaders: *physx_PxConstraintShaderTable, dataSize: u32) -> *physx_PxConstraint #foreign physx_sys "PxPhysics_createConstraint_mut";

PxPhysics_createArticulationReducedCoordinate :: (self: *physx_PxPhysics) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys "PxPhysics_createArticulationReducedCoordinate_mut";

PxPhysics_createMaterial :: (self: *physx_PxPhysics, staticFriction: float, dynamicFriction: float, restitution: float) -> *physx_PxMaterial #foreign physx_sys "PxPhysics_createMaterial_mut";

PxPhysics_getNbMaterials :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getMaterials :: (self: *physx_PxPhysics, userBuffer: **physx_PxMaterial, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_registerDeletionListener :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener, deletionEvents: *u8, restrictedObjectSet: bool) -> void #foreign physx_sys "PxPhysics_registerDeletionListener_mut";

PxPhysics_unregisterDeletionListener :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener) -> void #foreign physx_sys "PxPhysics_unregisterDeletionListener_mut";

PxPhysics_registerDeletionListenerObjects :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener, observables: **physx_PxBase, observableCount: u32) -> void #foreign physx_sys "PxPhysics_registerDeletionListenerObjects_mut";

PxPhysics_unregisterDeletionListenerObjects :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener, observables: **physx_PxBase, observableCount: u32) -> void #foreign physx_sys "PxPhysics_unregisterDeletionListenerObjects_mut";

PxPhysics_getPhysicsInsertionCallback :: (self: *physx_PxPhysics) -> *physx_PxInsertionCallback #foreign physx_sys "PxPhysics_getPhysicsInsertionCallback_mut";

phys_PxCreatePhysics :: (version: u32, foundation: *physx_PxFoundation, scale: *physx_PxTolerancesScale, trackOutstandingAllocations: bool, pvd: *physx_PxPvd, omniPvd: *physx_PxOmniPvd) -> *physx_PxPhysics #foreign physx_sys;

phys_PxGetPhysics :: () -> *physx_PxPhysics #foreign physx_sys;

PxActorShape_new :: () -> physx_PxActorShape #foreign physx_sys;

PxActorShape_new_1 :: (a: *physx_PxRigidActor, s: *physx_PxShape) -> physx_PxActorShape #foreign physx_sys;

PxQueryCache_new :: () -> physx_PxQueryCache #foreign physx_sys;

PxQueryCache_new_1 :: (s: *physx_PxShape, findex: u32) -> physx_PxQueryCache #foreign physx_sys;

PxQueryFilterData_new :: () -> physx_PxQueryFilterData #foreign physx_sys;

PxQueryFilterData_new_1 :: (fd: *physx_PxFilterData, f: u16) -> physx_PxQueryFilterData #foreign physx_sys;

PxQueryFilterData_new_2 :: (f: u16) -> physx_PxQueryFilterData #foreign physx_sys;

PxQueryFilterCallback_preFilter :: (self: *physx_PxQueryFilterCallback, filterData: *physx_PxFilterData, shape: *physx_PxShape, actor: *physx_PxRigidActor, queryFlags: *u16) -> s32 #foreign physx_sys "PxQueryFilterCallback_preFilter_mut";

PxQueryFilterCallback_postFilter :: (self: *physx_PxQueryFilterCallback, filterData: *physx_PxFilterData, hit: *physx_PxQueryHit, shape: *physx_PxShape, actor: *physx_PxRigidActor) -> s32 #foreign physx_sys "PxQueryFilterCallback_postFilter_mut";

PxQueryFilterCallback_delete :: (self: *physx_PxQueryFilterCallback) -> void #foreign physx_sys;

PxRigidDynamic_setKinematicTarget :: (self: *physx_PxRigidDynamic, destination: *physx_PxTransform) -> void #foreign physx_sys "PxRigidDynamic_setKinematicTarget_mut";

PxRigidDynamic_getKinematicTarget :: (self: *physx_PxRigidDynamic, target: *physx_PxTransform) -> bool #foreign physx_sys;

PxRigidDynamic_isSleeping :: (self: *physx_PxRigidDynamic) -> bool #foreign physx_sys;

PxRigidDynamic_setSleepThreshold :: (self: *physx_PxRigidDynamic, threshold: float) -> void #foreign physx_sys "PxRigidDynamic_setSleepThreshold_mut";

PxRigidDynamic_getSleepThreshold :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_setStabilizationThreshold :: (self: *physx_PxRigidDynamic, threshold: float) -> void #foreign physx_sys "PxRigidDynamic_setStabilizationThreshold_mut";

PxRigidDynamic_getStabilizationThreshold :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_getRigidDynamicLockFlags :: (self: *physx_PxRigidDynamic) -> u8 #foreign physx_sys;

PxRigidDynamic_setRigidDynamicLockFlag :: (self: *physx_PxRigidDynamic, flag: s32, value: bool) -> void #foreign physx_sys "PxRigidDynamic_setRigidDynamicLockFlag_mut";

PxRigidDynamic_setRigidDynamicLockFlags :: (self: *physx_PxRigidDynamic, flags: u8) -> void #foreign physx_sys "PxRigidDynamic_setRigidDynamicLockFlags_mut";

PxRigidDynamic_getLinearVelocity :: (self: *physx_PxRigidDynamic) -> Vector3 #foreign physx_sys;

PxRigidDynamic_setLinearVelocity :: (self: *physx_PxRigidDynamic, linVel: *Vector3, autowake: bool) -> void #foreign physx_sys "PxRigidDynamic_setLinearVelocity_mut";

PxRigidDynamic_getAngularVelocity :: (self: *physx_PxRigidDynamic) -> Vector3 #foreign physx_sys;

PxRigidDynamic_setAngularVelocity :: (self: *physx_PxRigidDynamic, angVel: *Vector3, autowake: bool) -> void #foreign physx_sys "PxRigidDynamic_setAngularVelocity_mut";

PxRigidDynamic_setWakeCounter :: (self: *physx_PxRigidDynamic, wakeCounterValue: float) -> void #foreign physx_sys "PxRigidDynamic_setWakeCounter_mut";

PxRigidDynamic_getWakeCounter :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_wakeUp :: (self: *physx_PxRigidDynamic) -> void #foreign physx_sys "PxRigidDynamic_wakeUp_mut";

PxRigidDynamic_putToSleep :: (self: *physx_PxRigidDynamic) -> void #foreign physx_sys "PxRigidDynamic_putToSleep_mut";

PxRigidDynamic_setSolverIterationCounts :: (self: *physx_PxRigidDynamic, minPositionIters: u32, minVelocityIters: u32) -> void #foreign physx_sys "PxRigidDynamic_setSolverIterationCounts_mut";

PxRigidDynamic_getSolverIterationCounts :: (self: *physx_PxRigidDynamic, minPositionIters: *u32, minVelocityIters: *u32) -> void #foreign physx_sys;

PxRigidDynamic_getContactReportThreshold :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_setContactReportThreshold :: (self: *physx_PxRigidDynamic, threshold: float) -> void #foreign physx_sys "PxRigidDynamic_setContactReportThreshold_mut";

PxRigidDynamic_getConcreteTypeName :: (self: *physx_PxRigidDynamic) -> *u8 #foreign physx_sys;

PxRigidStatic_getConcreteTypeName :: (self: *physx_PxRigidStatic) -> *u8 #foreign physx_sys;

PxSceneQueryDesc_new :: () -> physx_PxSceneQueryDesc #foreign physx_sys;

PxSceneQueryDesc_setToDefault :: (self: *physx_PxSceneQueryDesc) -> void #foreign physx_sys "PxSceneQueryDesc_setToDefault_mut";

PxSceneQueryDesc_isValid :: (self: *physx_PxSceneQueryDesc) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint :: (self: *physx_PxSceneQuerySystemBase, dynamicTreeRebuildRateHint: u32) -> void #foreign physx_sys "PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut";

PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint :: (self: *physx_PxSceneQuerySystemBase) -> u32 #foreign physx_sys;

PxSceneQuerySystemBase_forceRebuildDynamicTree :: (self: *physx_PxSceneQuerySystemBase, prunerIndex: u32) -> void #foreign physx_sys "PxSceneQuerySystemBase_forceRebuildDynamicTree_mut";

PxSceneQuerySystemBase_setUpdateMode :: (self: *physx_PxSceneQuerySystemBase, updateMode: s32) -> void #foreign physx_sys "PxSceneQuerySystemBase_setUpdateMode_mut";

PxSceneQuerySystemBase_getUpdateMode :: (self: *physx_PxSceneQuerySystemBase) -> s32 #foreign physx_sys;

PxSceneQuerySystemBase_getStaticTimestamp :: (self: *physx_PxSceneQuerySystemBase) -> u32 #foreign physx_sys;

PxSceneQuerySystemBase_flushUpdates :: (self: *physx_PxSceneQuerySystemBase) -> void #foreign physx_sys "PxSceneQuerySystemBase_flushUpdates_mut";

PxSceneQuerySystemBase_raycast :: (self: *physx_PxSceneQuerySystemBase, origin: *Vector3, unitDir: *Vector3, distance: float, hitCall: *physx_PxRaycastCallback, hitFlags: u16, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, queryFlags: u32) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_sweep :: (self: *physx_PxSceneQuerySystemBase, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *Vector3, distance: float, hitCall: *physx_PxSweepCallback, hitFlags: u16, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float, queryFlags: u32) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_overlap :: (self: *physx_PxSceneQuerySystemBase, geometry: *physx_PxGeometry, pose: *physx_PxTransform, hitCall: *physx_PxOverlapCallback, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, queryFlags: u32) -> bool #foreign physx_sys;

PxSceneSQSystem_setSceneQueryUpdateMode :: (self: *physx_PxSceneSQSystem, updateMode: s32) -> void #foreign physx_sys "PxSceneSQSystem_setSceneQueryUpdateMode_mut";

PxSceneSQSystem_getSceneQueryUpdateMode :: (self: *physx_PxSceneSQSystem) -> s32 #foreign physx_sys;

PxSceneSQSystem_getSceneQueryStaticTimestamp :: (self: *physx_PxSceneSQSystem) -> u32 #foreign physx_sys;

PxSceneSQSystem_flushQueryUpdates :: (self: *physx_PxSceneSQSystem) -> void #foreign physx_sys "PxSceneSQSystem_flushQueryUpdates_mut";

PxSceneSQSystem_forceDynamicTreeRebuild :: (self: *physx_PxSceneSQSystem, rebuildStaticStructure: bool, rebuildDynamicStructure: bool) -> void #foreign physx_sys "PxSceneSQSystem_forceDynamicTreeRebuild_mut";

PxSceneSQSystem_getStaticStructure :: (self: *physx_PxSceneSQSystem) -> s32 #foreign physx_sys;

PxSceneSQSystem_getDynamicStructure :: (self: *physx_PxSceneSQSystem) -> s32 #foreign physx_sys;

PxSceneSQSystem_sceneQueriesUpdate :: (self: *physx_PxSceneSQSystem, completionTask: *physx_PxBaseTask, controlSimulation: bool) -> void #foreign physx_sys "PxSceneSQSystem_sceneQueriesUpdate_mut";

PxSceneSQSystem_checkQueries :: (self: *physx_PxSceneSQSystem, block: bool) -> bool #foreign physx_sys "PxSceneSQSystem_checkQueries_mut";

PxSceneSQSystem_fetchQueries :: (self: *physx_PxSceneSQSystem, block: bool) -> bool #foreign physx_sys "PxSceneSQSystem_fetchQueries_mut";

PxSceneQuerySystem_release :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys "PxSceneQuerySystem_release_mut";

PxSceneQuerySystem_acquireReference :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys "PxSceneQuerySystem_acquireReference_mut";

PxSceneQuerySystem_preallocate :: (self: *physx_PxSceneQuerySystem, prunerIndex: u32, nbShapes: u32) -> void #foreign physx_sys "PxSceneQuerySystem_preallocate_mut";

PxSceneQuerySystem_flushMemory :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys "PxSceneQuerySystem_flushMemory_mut";

PxSceneQuerySystem_addSQShape :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape, bounds: *physx_PxBounds3, transform: *physx_PxTransform, compoundHandle: *u32, hasPruningStructure: bool) -> void #foreign physx_sys "PxSceneQuerySystem_addSQShape_mut";

PxSceneQuerySystem_removeSQShape :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape) -> void #foreign physx_sys "PxSceneQuerySystem_removeSQShape_mut";

PxSceneQuerySystem_updateSQShape :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape, transform: *physx_PxTransform) -> void #foreign physx_sys "PxSceneQuerySystem_updateSQShape_mut";

PxSceneQuerySystem_addSQCompound :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shapes: **physx_PxShape, bvh: *physx_PxBVH, transforms: *physx_PxTransform) -> u32 #foreign physx_sys "PxSceneQuerySystem_addSQCompound_mut";

PxSceneQuerySystem_removeSQCompound :: (self: *physx_PxSceneQuerySystem, compoundHandle: u32) -> void #foreign physx_sys "PxSceneQuerySystem_removeSQCompound_mut";

PxSceneQuerySystem_updateSQCompound :: (self: *physx_PxSceneQuerySystem, compoundHandle: u32, compoundTransform: *physx_PxTransform) -> void #foreign physx_sys "PxSceneQuerySystem_updateSQCompound_mut";

PxSceneQuerySystem_shiftOrigin :: (self: *physx_PxSceneQuerySystem, shift: *Vector3) -> void #foreign physx_sys "PxSceneQuerySystem_shiftOrigin_mut";

PxSceneQuerySystem_merge :: (self: *physx_PxSceneQuerySystem, pruningStructure: *physx_PxPruningStructure) -> void #foreign physx_sys "PxSceneQuerySystem_merge_mut";

PxSceneQuerySystem_getHandle :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape, prunerIndex: *u32) -> u32 #foreign physx_sys;

PxSceneQuerySystem_sync :: (self: *physx_PxSceneQuerySystem, prunerIndex: u32, handles: *u32, indices: *u32, bounds: *physx_PxBounds3, transforms: *physx_PxTransformPadded, count: u32, ignoredIndices: *physx_PxBitMap) -> void #foreign physx_sys "PxSceneQuerySystem_sync_mut";

PxSceneQuerySystem_finalizeUpdates :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys "PxSceneQuerySystem_finalizeUpdates_mut";

PxSceneQuerySystem_prepareSceneQueryBuildStep :: (self: *physx_PxSceneQuerySystem, prunerIndex: u32) -> *void #foreign physx_sys "PxSceneQuerySystem_prepareSceneQueryBuildStep_mut";

PxSceneQuerySystem_sceneQueryBuildStep :: (self: *physx_PxSceneQuerySystem, handle: *void) -> void #foreign physx_sys "PxSceneQuerySystem_sceneQueryBuildStep_mut";

PxBroadPhaseDesc_new :: (type: s32) -> physx_PxBroadPhaseDesc #foreign physx_sys;

PxBroadPhaseDesc_isValid :: (self: *physx_PxBroadPhaseDesc) -> bool #foreign physx_sys;

phys_PxGetBroadPhaseStaticFilterGroup :: () -> u32 #foreign physx_sys;

phys_PxGetBroadPhaseDynamicFilterGroup :: (id: u32) -> u32 #foreign physx_sys;

phys_PxGetBroadPhaseKinematicFilterGroup :: (id: u32) -> u32 #foreign physx_sys;

PxBroadPhaseUpdateData_new :: (created: *u32, nbCreated: u32, updated: *u32, nbUpdated: u32, removed: *u32, nbRemoved: u32, bounds: *physx_PxBounds3, groups: *u32, distances: *float, capacity: u32) -> physx_PxBroadPhaseUpdateData #foreign physx_sys;

PxBroadPhaseResults_new :: () -> physx_PxBroadPhaseResults #foreign physx_sys;

PxBroadPhaseRegions_getNbRegions :: (self: *physx_PxBroadPhaseRegions) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_getRegions :: (self: *physx_PxBroadPhaseRegions, userBuffer: *physx_PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_addRegion :: (self: *physx_PxBroadPhaseRegions, region: *physx_PxBroadPhaseRegion, populateRegion: bool, bounds: *physx_PxBounds3, distances: *float) -> u32 #foreign physx_sys "PxBroadPhaseRegions_addRegion_mut";

PxBroadPhaseRegions_removeRegion :: (self: *physx_PxBroadPhaseRegions, handle: u32) -> bool #foreign physx_sys "PxBroadPhaseRegions_removeRegion_mut";

PxBroadPhaseRegions_getNbOutOfBoundsObjects :: (self: *physx_PxBroadPhaseRegions) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_getOutOfBoundsObjects :: (self: *physx_PxBroadPhaseRegions) -> *u32 #foreign physx_sys;

PxBroadPhase_release :: (self: *physx_PxBroadPhase) -> void #foreign physx_sys "PxBroadPhase_release_mut";

PxBroadPhase_getType :: (self: *physx_PxBroadPhase) -> s32 #foreign physx_sys;

PxBroadPhase_getCaps :: (self: *physx_PxBroadPhase, caps: *physx_PxBroadPhaseCaps) -> void #foreign physx_sys;

PxBroadPhase_getRegions :: (self: *physx_PxBroadPhase) -> *physx_PxBroadPhaseRegions #foreign physx_sys "PxBroadPhase_getRegions_mut";

PxBroadPhase_getAllocator :: (self: *physx_PxBroadPhase) -> *physx_PxAllocatorCallback #foreign physx_sys "PxBroadPhase_getAllocator_mut";

PxBroadPhase_getContextID :: (self: *physx_PxBroadPhase) -> u64 #foreign physx_sys;

PxBroadPhase_setScratchBlock :: (self: *physx_PxBroadPhase, scratchBlock: *void, size: u32) -> void #foreign physx_sys "PxBroadPhase_setScratchBlock_mut";

PxBroadPhase_update :: (self: *physx_PxBroadPhase, updateData: *physx_PxBroadPhaseUpdateData, continuation: *physx_PxBaseTask) -> void #foreign physx_sys "PxBroadPhase_update_mut";

PxBroadPhase_fetchResults :: (self: *physx_PxBroadPhase, results: *physx_PxBroadPhaseResults) -> void #foreign physx_sys "PxBroadPhase_fetchResults_mut";

PxBroadPhase_update :: (self: *physx_PxBroadPhase, results: *physx_PxBroadPhaseResults, updateData: *physx_PxBroadPhaseUpdateData) -> void #foreign physx_sys "PxBroadPhase_update_mut_1";

phys_PxCreateBroadPhase :: (desc: *physx_PxBroadPhaseDesc) -> *physx_PxBroadPhase #foreign physx_sys;

PxAABBManager_release :: (self: *physx_PxAABBManager) -> void #foreign physx_sys "PxAABBManager_release_mut";

PxAABBManager_getBroadPhase :: (self: *physx_PxAABBManager) -> *physx_PxBroadPhase #foreign physx_sys "PxAABBManager_getBroadPhase_mut";

PxAABBManager_getBounds :: (self: *physx_PxAABBManager) -> *physx_PxBounds3 #foreign physx_sys;

PxAABBManager_getDistances :: (self: *physx_PxAABBManager) -> *float #foreign physx_sys;

PxAABBManager_getGroups :: (self: *physx_PxAABBManager) -> *u32 #foreign physx_sys;

PxAABBManager_getCapacity :: (self: *physx_PxAABBManager) -> u32 #foreign physx_sys;

PxAABBManager_addObject :: (self: *physx_PxAABBManager, index: u32, bounds: *physx_PxBounds3, group: u32, distance: float) -> void #foreign physx_sys "PxAABBManager_addObject_mut";

PxAABBManager_removeObject :: (self: *physx_PxAABBManager, index: u32) -> void #foreign physx_sys "PxAABBManager_removeObject_mut";

PxAABBManager_updateObject :: (self: *physx_PxAABBManager, index: u32, bounds: *physx_PxBounds3, distance: *float) -> void #foreign physx_sys "PxAABBManager_updateObject_mut";

PxAABBManager_update :: (self: *physx_PxAABBManager, continuation: *physx_PxBaseTask) -> void #foreign physx_sys "PxAABBManager_update_mut";

PxAABBManager_fetchResults :: (self: *physx_PxAABBManager, results: *physx_PxBroadPhaseResults) -> void #foreign physx_sys "PxAABBManager_fetchResults_mut";

PxAABBManager_update :: (self: *physx_PxAABBManager, results: *physx_PxBroadPhaseResults) -> void #foreign physx_sys "PxAABBManager_update_mut_1";

phys_PxCreateAABBManager :: (broadphase: *physx_PxBroadPhase) -> *physx_PxAABBManager #foreign physx_sys;

PxSceneLimits_new :: () -> physx_PxSceneLimits #foreign physx_sys;

PxSceneLimits_setToDefault :: (self: *physx_PxSceneLimits) -> void #foreign physx_sys "PxSceneLimits_setToDefault_mut";

PxSceneLimits_isValid :: (self: *physx_PxSceneLimits) -> bool #foreign physx_sys;

PxgDynamicsMemoryConfig_new :: () -> physx_PxgDynamicsMemoryConfig #foreign physx_sys;

PxgDynamicsMemoryConfig_isValid :: (self: *physx_PxgDynamicsMemoryConfig) -> bool #foreign physx_sys;

PxSceneDesc_new :: (scale: *physx_PxTolerancesScale) -> physx_PxSceneDesc #foreign physx_sys;

PxSceneDesc_setToDefault :: (self: *physx_PxSceneDesc, scale: *physx_PxTolerancesScale) -> void #foreign physx_sys "PxSceneDesc_setToDefault_mut";

PxSceneDesc_isValid :: (self: *physx_PxSceneDesc) -> bool #foreign physx_sys;

PxSceneDesc_getTolerancesScale :: (self: *physx_PxSceneDesc) -> *physx_PxTolerancesScale #foreign physx_sys;

PxSimulationStatistics_getNbBroadPhaseAdds :: (self: *physx_PxSimulationStatistics) -> u32 #foreign physx_sys;

PxSimulationStatistics_getNbBroadPhaseRemoves :: (self: *physx_PxSimulationStatistics) -> u32 #foreign physx_sys;

PxSimulationStatistics_getRbPairStats :: (self: *physx_PxSimulationStatistics, pairType: s32, g0: s32, g1: s32) -> u32 #foreign physx_sys;

PxSimulationStatistics_new :: () -> physx_PxSimulationStatistics #foreign physx_sys;

PxPvdSceneClient_setScenePvdFlag :: (self: *physx_PxPvdSceneClient, flag: s32, value: bool) -> void #foreign physx_sys "PxPvdSceneClient_setScenePvdFlag_mut";

PxPvdSceneClient_setScenePvdFlags :: (self: *physx_PxPvdSceneClient, flags: u8) -> void #foreign physx_sys "PxPvdSceneClient_setScenePvdFlags_mut";

PxPvdSceneClient_getScenePvdFlags :: (self: *physx_PxPvdSceneClient) -> u8 #foreign physx_sys;

PxPvdSceneClient_updateCamera :: (self: *physx_PxPvdSceneClient, name: *u8, origin: *Vector3, up: *Vector3, target: *Vector3) -> void #foreign physx_sys "PxPvdSceneClient_updateCamera_mut";

PxPvdSceneClient_drawPoints :: (self: *physx_PxPvdSceneClient, points: *physx_PxDebugPoint, count: u32) -> void #foreign physx_sys "PxPvdSceneClient_drawPoints_mut";

PxPvdSceneClient_drawLines :: (self: *physx_PxPvdSceneClient, lines: *physx_PxDebugLine, count: u32) -> void #foreign physx_sys "PxPvdSceneClient_drawLines_mut";

PxPvdSceneClient_drawTriangles :: (self: *physx_PxPvdSceneClient, triangles: *physx_PxDebugTriangle, count: u32) -> void #foreign physx_sys "PxPvdSceneClient_drawTriangles_mut";

PxPvdSceneClient_drawText :: (self: *physx_PxPvdSceneClient, text: *physx_PxDebugText) -> void #foreign physx_sys "PxPvdSceneClient_drawText_mut";

PxDominanceGroupPair_new :: (a: u8, b: u8) -> physx_PxDominanceGroupPair #foreign physx_sys;

PxBroadPhaseCallback_delete :: (self: *physx_PxBroadPhaseCallback) -> void #foreign physx_sys;

PxBroadPhaseCallback_onObjectOutOfBounds :: (self: *physx_PxBroadPhaseCallback, shape: *physx_PxShape, actor: *physx_PxActor) -> void #foreign physx_sys "PxBroadPhaseCallback_onObjectOutOfBounds_mut";

PxBroadPhaseCallback_onObjectOutOfBounds :: (self: *physx_PxBroadPhaseCallback, aggregate: *physx_PxAggregate) -> void #foreign physx_sys "PxBroadPhaseCallback_onObjectOutOfBounds_mut_1";

PxScene_release :: (self: *physx_PxScene) -> void #foreign physx_sys "PxScene_release_mut";

PxScene_setFlag :: (self: *physx_PxScene, flag: s32, value: bool) -> void #foreign physx_sys "PxScene_setFlag_mut";

PxScene_getFlags :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setLimits :: (self: *physx_PxScene, limits: *physx_PxSceneLimits) -> void #foreign physx_sys "PxScene_setLimits_mut";

PxScene_getLimits :: (self: *physx_PxScene) -> physx_PxSceneLimits #foreign physx_sys;

PxScene_getPhysics :: (self: *physx_PxScene) -> *physx_PxPhysics #foreign physx_sys "PxScene_getPhysics_mut";

PxScene_getTimestamp :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_addArticulation :: (self: *physx_PxScene, articulation: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys "PxScene_addArticulation_mut";

PxScene_removeArticulation :: (self: *physx_PxScene, articulation: *physx_PxArticulationReducedCoordinate, wakeOnLostTouch: bool) -> void #foreign physx_sys "PxScene_removeArticulation_mut";

PxScene_addActor :: (self: *physx_PxScene, actor: *physx_PxActor, bvh: *physx_PxBVH) -> bool #foreign physx_sys "PxScene_addActor_mut";

PxScene_addActors :: (self: *physx_PxScene, actors: **physx_PxActor, nbActors: u32) -> bool #foreign physx_sys "PxScene_addActors_mut";

PxScene_addActors :: (self: *physx_PxScene, pruningStructure: *physx_PxPruningStructure) -> bool #foreign physx_sys "PxScene_addActors_mut_1";

PxScene_removeActor :: (self: *physx_PxScene, actor: *physx_PxActor, wakeOnLostTouch: bool) -> void #foreign physx_sys "PxScene_removeActor_mut";

PxScene_removeActors :: (self: *physx_PxScene, actors: **physx_PxActor, nbActors: u32, wakeOnLostTouch: bool) -> void #foreign physx_sys "PxScene_removeActors_mut";

PxScene_addAggregate :: (self: *physx_PxScene, aggregate: *physx_PxAggregate) -> bool #foreign physx_sys "PxScene_addAggregate_mut";

PxScene_removeAggregate :: (self: *physx_PxScene, aggregate: *physx_PxAggregate, wakeOnLostTouch: bool) -> void #foreign physx_sys "PxScene_removeAggregate_mut";

PxScene_addCollection :: (self: *physx_PxScene, collection: *physx_PxCollection) -> bool #foreign physx_sys "PxScene_addCollection_mut";

PxScene_getNbActors :: (self: *physx_PxScene, types: u16) -> u32 #foreign physx_sys;

PxScene_getActors :: (self: *physx_PxScene, types: u16, userBuffer: **physx_PxActor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getActiveActors :: (self: *physx_PxScene, nbActorsOut: *u32) -> **physx_PxActor #foreign physx_sys "PxScene_getActiveActors_mut";

PxScene_getNbArticulations :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getArticulations :: (self: *physx_PxScene, userBuffer: **physx_PxArticulationReducedCoordinate, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getNbConstraints :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getConstraints :: (self: *physx_PxScene, userBuffer: **physx_PxConstraint, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getNbAggregates :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getAggregates :: (self: *physx_PxScene, userBuffer: **physx_PxAggregate, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_setDominanceGroupPair :: (self: *physx_PxScene, group1: u8, group2: u8, dominance: *physx_PxDominanceGroupPair) -> void #foreign physx_sys "PxScene_setDominanceGroupPair_mut";

PxScene_getDominanceGroupPair :: (self: *physx_PxScene, group1: u8, group2: u8) -> physx_PxDominanceGroupPair #foreign physx_sys;

PxScene_getCpuDispatcher :: (self: *physx_PxScene) -> *physx_PxCpuDispatcher #foreign physx_sys;

PxScene_createClient :: (self: *physx_PxScene) -> u8 #foreign physx_sys "PxScene_createClient_mut";

PxScene_setSimulationEventCallback :: (self: *physx_PxScene, callback: *physx_PxSimulationEventCallback) -> void #foreign physx_sys "PxScene_setSimulationEventCallback_mut";

PxScene_getSimulationEventCallback :: (self: *physx_PxScene) -> *physx_PxSimulationEventCallback #foreign physx_sys;

PxScene_setContactModifyCallback :: (self: *physx_PxScene, callback: *physx_PxContactModifyCallback) -> void #foreign physx_sys "PxScene_setContactModifyCallback_mut";

PxScene_setCCDContactModifyCallback :: (self: *physx_PxScene, callback: *physx_PxCCDContactModifyCallback) -> void #foreign physx_sys "PxScene_setCCDContactModifyCallback_mut";

PxScene_getContactModifyCallback :: (self: *physx_PxScene) -> *physx_PxContactModifyCallback #foreign physx_sys;

PxScene_getCCDContactModifyCallback :: (self: *physx_PxScene) -> *physx_PxCCDContactModifyCallback #foreign physx_sys;

PxScene_setBroadPhaseCallback :: (self: *physx_PxScene, callback: *physx_PxBroadPhaseCallback) -> void #foreign physx_sys "PxScene_setBroadPhaseCallback_mut";

PxScene_getBroadPhaseCallback :: (self: *physx_PxScene) -> *physx_PxBroadPhaseCallback #foreign physx_sys;

PxScene_setFilterShaderData :: (self: *physx_PxScene, data: *void, dataSize: u32) -> void #foreign physx_sys "PxScene_setFilterShaderData_mut";

PxScene_getFilterShaderData :: (self: *physx_PxScene) -> *void #foreign physx_sys;

PxScene_getFilterShaderDataSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_resetFiltering :: (self: *physx_PxScene, actor: *physx_PxActor) -> bool #foreign physx_sys "PxScene_resetFiltering_mut";

PxScene_resetFiltering :: (self: *physx_PxScene, actor: *physx_PxRigidActor, shapes: **physx_PxShape, shapeCount: u32) -> bool #foreign physx_sys "PxScene_resetFiltering_mut_1";

PxScene_getKinematicKinematicFilteringMode :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_getStaticKinematicFilteringMode :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_simulate :: (self: *physx_PxScene, elapsedTime: float, completionTask: *physx_PxBaseTask, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign physx_sys "PxScene_simulate_mut";

PxScene_advance :: (self: *physx_PxScene, completionTask: *physx_PxBaseTask) -> bool #foreign physx_sys "PxScene_advance_mut";

PxScene_collide :: (self: *physx_PxScene, elapsedTime: float, completionTask: *physx_PxBaseTask, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign physx_sys "PxScene_collide_mut";

PxScene_checkResults :: (self: *physx_PxScene, block: bool) -> bool #foreign physx_sys "PxScene_checkResults_mut";

PxScene_fetchCollision :: (self: *physx_PxScene, block: bool) -> bool #foreign physx_sys "PxScene_fetchCollision_mut";

PxScene_fetchResults :: (self: *physx_PxScene, block: bool, errorState: *u32) -> bool #foreign physx_sys "PxScene_fetchResults_mut";

PxScene_fetchResultsStart :: (self: *physx_PxScene, contactPairs: **physx_PxContactPairHeader, nbContactPairs: *u32, block: bool) -> bool #foreign physx_sys "PxScene_fetchResultsStart_mut";

PxScene_processCallbacks :: (self: *physx_PxScene, continuation: *physx_PxBaseTask) -> void #foreign physx_sys "PxScene_processCallbacks_mut";

PxScene_fetchResultsFinish :: (self: *physx_PxScene, errorState: *u32) -> void #foreign physx_sys "PxScene_fetchResultsFinish_mut";

PxScene_fetchResultsParticleSystem :: (self: *physx_PxScene) -> void #foreign physx_sys "PxScene_fetchResultsParticleSystem_mut";

PxScene_flushSimulation :: (self: *physx_PxScene, sendPendingReports: bool) -> void #foreign physx_sys "PxScene_flushSimulation_mut";

PxScene_setGravity :: (self: *physx_PxScene, vec: *Vector3) -> void #foreign physx_sys "PxScene_setGravity_mut";

PxScene_getGravity :: (self: *physx_PxScene) -> Vector3 #foreign physx_sys;

PxScene_setBounceThresholdVelocity :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys "PxScene_setBounceThresholdVelocity_mut";

PxScene_getBounceThresholdVelocity :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setCCDMaxPasses :: (self: *physx_PxScene, ccdMaxPasses: u32) -> void #foreign physx_sys "PxScene_setCCDMaxPasses_mut";

PxScene_getCCDMaxPasses :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setCCDMaxSeparation :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys "PxScene_setCCDMaxSeparation_mut";

PxScene_getCCDMaxSeparation :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setCCDThreshold :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys "PxScene_setCCDThreshold_mut";

PxScene_getCCDThreshold :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setMaxBiasCoefficient :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys "PxScene_setMaxBiasCoefficient_mut";

PxScene_getMaxBiasCoefficient :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setFrictionOffsetThreshold :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys "PxScene_setFrictionOffsetThreshold_mut";

PxScene_getFrictionOffsetThreshold :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setFrictionCorrelationDistance :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys "PxScene_setFrictionCorrelationDistance_mut";

PxScene_getFrictionCorrelationDistance :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_getFrictionType :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_getSolverType :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_setVisualizationParameter :: (self: *physx_PxScene, param: s32, value: float) -> bool #foreign physx_sys "PxScene_setVisualizationParameter_mut";

PxScene_getVisualizationParameter :: (self: *physx_PxScene, paramEnum: s32) -> float #foreign physx_sys;

PxScene_setVisualizationCullingBox :: (self: *physx_PxScene, box: *physx_PxBounds3) -> void #foreign physx_sys "PxScene_setVisualizationCullingBox_mut";

PxScene_getVisualizationCullingBox :: (self: *physx_PxScene) -> physx_PxBounds3 #foreign physx_sys;

PxScene_getRenderBuffer :: (self: *physx_PxScene) -> *physx_PxRenderBuffer #foreign physx_sys "PxScene_getRenderBuffer_mut";

PxScene_getSimulationStatistics :: (self: *physx_PxScene, stats: *physx_PxSimulationStatistics) -> void #foreign physx_sys;

PxScene_getBroadPhaseType :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_getBroadPhaseCaps :: (self: *physx_PxScene, caps: *physx_PxBroadPhaseCaps) -> bool #foreign physx_sys;

PxScene_getNbBroadPhaseRegions :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getBroadPhaseRegions :: (self: *physx_PxScene, userBuffer: *physx_PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_addBroadPhaseRegion :: (self: *physx_PxScene, region: *physx_PxBroadPhaseRegion, populateRegion: bool) -> u32 #foreign physx_sys "PxScene_addBroadPhaseRegion_mut";

PxScene_removeBroadPhaseRegion :: (self: *physx_PxScene, handle: u32) -> bool #foreign physx_sys "PxScene_removeBroadPhaseRegion_mut";

PxScene_getTaskManager :: (self: *physx_PxScene) -> *physx_PxTaskManager #foreign physx_sys;

PxScene_lockRead :: (self: *physx_PxScene, file: *u8, line: u32) -> void #foreign physx_sys "PxScene_lockRead_mut";

PxScene_unlockRead :: (self: *physx_PxScene) -> void #foreign physx_sys "PxScene_unlockRead_mut";

PxScene_lockWrite :: (self: *physx_PxScene, file: *u8, line: u32) -> void #foreign physx_sys "PxScene_lockWrite_mut";

PxScene_unlockWrite :: (self: *physx_PxScene) -> void #foreign physx_sys "PxScene_unlockWrite_mut";

PxScene_setNbContactDataBlocks :: (self: *physx_PxScene, numBlocks: u32) -> void #foreign physx_sys "PxScene_setNbContactDataBlocks_mut";

PxScene_getNbContactDataBlocksUsed :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getMaxNbContactDataBlocksUsed :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getContactReportStreamBufferSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setSolverBatchSize :: (self: *physx_PxScene, solverBatchSize: u32) -> void #foreign physx_sys "PxScene_setSolverBatchSize_mut";

PxScene_getSolverBatchSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setSolverArticulationBatchSize :: (self: *physx_PxScene, solverBatchSize: u32) -> void #foreign physx_sys "PxScene_setSolverArticulationBatchSize_mut";

PxScene_getSolverArticulationBatchSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getWakeCounterResetValue :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_shiftOrigin :: (self: *physx_PxScene, shift: *Vector3) -> void #foreign physx_sys "PxScene_shiftOrigin_mut";

PxScene_getScenePvdClient :: (self: *physx_PxScene) -> *physx_PxPvdSceneClient #foreign physx_sys "PxScene_getScenePvdClient_mut";

PxScene_copyArticulationData :: (self: *physx_PxScene, data: *void, index: *void, dataType: s32, nbCopyArticulations: u32, copyEvent: *void) -> void #foreign physx_sys "PxScene_copyArticulationData_mut";

PxScene_applyArticulationData :: (self: *physx_PxScene, data: *void, index: *void, dataType: s32, nbUpdatedArticulations: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys "PxScene_applyArticulationData_mut";

PxScene_copySoftBodyData :: (self: *physx_PxScene, data: **void, dataSizes: *void, softBodyIndices: *void, flag: s32, nbCopySoftBodies: u32, maxSize: u32, copyEvent: *void) -> void #foreign physx_sys "PxScene_copySoftBodyData_mut";

PxScene_applySoftBodyData :: (self: *physx_PxScene, data: **void, dataSizes: *void, softBodyIndices: *void, flag: s32, nbUpdatedSoftBodies: u32, maxSize: u32, applyEvent: *void) -> void #foreign physx_sys "PxScene_applySoftBodyData_mut";

PxScene_copyContactData :: (self: *physx_PxScene, data: *void, maxContactPairs: u32, numContactPairs: *void, copyEvent: *void) -> void #foreign physx_sys "PxScene_copyContactData_mut";

PxScene_copyBodyData :: (self: *physx_PxScene, data: *physx_PxGpuBodyData, index: *physx_PxGpuActorPair, nbCopyActors: u32, copyEvent: *void) -> void #foreign physx_sys "PxScene_copyBodyData_mut";

PxScene_applyActorData :: (self: *physx_PxScene, data: *void, index: *physx_PxGpuActorPair, flag: s32, nbUpdatedActors: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys "PxScene_applyActorData_mut";

PxScene_computeDenseJacobians :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys "PxScene_computeDenseJacobians_mut";

PxScene_computeGeneralizedMassMatrices :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys "PxScene_computeGeneralizedMassMatrices_mut";

PxScene_computeGeneralizedGravityForces :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys "PxScene_computeGeneralizedGravityForces_mut";

PxScene_computeCoriolisAndCentrifugalForces :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys "PxScene_computeCoriolisAndCentrifugalForces_mut";

PxScene_getGpuDynamicsConfig :: (self: *physx_PxScene) -> physx_PxgDynamicsMemoryConfig #foreign physx_sys;

PxScene_applyParticleBufferData :: (self: *physx_PxScene, indices: *u32, bufferIndexPair: *physx_PxGpuParticleBufferIndexPair, flags: *u32, nbUpdatedBuffers: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys "PxScene_applyParticleBufferData_mut";

PxSceneReadLock_new_alloc :: (scene: *physx_PxScene, file: *u8, line: u32) -> *physx_PxSceneReadLock #foreign physx_sys;

PxSceneReadLock_delete :: (self: *physx_PxSceneReadLock) -> void #foreign physx_sys;

PxSceneWriteLock_new_alloc :: (scene: *physx_PxScene, file: *u8, line: u32) -> *physx_PxSceneWriteLock #foreign physx_sys;

PxSceneWriteLock_delete :: (self: *physx_PxSceneWriteLock) -> void #foreign physx_sys;

PxContactPairExtraDataItem_new :: () -> physx_PxContactPairExtraDataItem #foreign physx_sys;

PxContactPairVelocity_new :: () -> physx_PxContactPairVelocity #foreign physx_sys;

PxContactPairPose_new :: () -> physx_PxContactPairPose #foreign physx_sys;

PxContactPairIndex_new :: () -> physx_PxContactPairIndex #foreign physx_sys;

PxContactPairExtraDataIterator_new :: (stream: *u8, size: u32) -> physx_PxContactPairExtraDataIterator #foreign physx_sys;

PxContactPairExtraDataIterator_nextItemSet :: (self: *physx_PxContactPairExtraDataIterator) -> bool #foreign physx_sys "PxContactPairExtraDataIterator_nextItemSet_mut";

PxContactPairHeader_new :: () -> physx_PxContactPairHeader #foreign physx_sys;

PxContactPair_new :: () -> physx_PxContactPair #foreign physx_sys;

PxContactPair_extractContacts :: (self: *physx_PxContactPair, userBuffer: *physx_PxContactPairPoint, bufferSize: u32) -> u32 #foreign physx_sys;

PxContactPair_bufferContacts :: (self: *physx_PxContactPair, newPair: *physx_PxContactPair, bufferMemory: *u8) -> void #foreign physx_sys;

PxContactPair_getInternalFaceIndices :: (self: *physx_PxContactPair) -> *u32 #foreign physx_sys;

PxTriggerPair_new :: () -> physx_PxTriggerPair #foreign physx_sys;

PxConstraintInfo_new :: () -> physx_PxConstraintInfo #foreign physx_sys;

PxConstraintInfo_new_1 :: (c: *physx_PxConstraint, extRef: *void, t: u32) -> physx_PxConstraintInfo #foreign physx_sys;

PxSimulationEventCallback_onConstraintBreak :: (self: *physx_PxSimulationEventCallback, constraints: *physx_PxConstraintInfo, count: u32) -> void #foreign physx_sys "PxSimulationEventCallback_onConstraintBreak_mut";

PxSimulationEventCallback_onWake :: (self: *physx_PxSimulationEventCallback, actors: **physx_PxActor, count: u32) -> void #foreign physx_sys "PxSimulationEventCallback_onWake_mut";

PxSimulationEventCallback_onSleep :: (self: *physx_PxSimulationEventCallback, actors: **physx_PxActor, count: u32) -> void #foreign physx_sys "PxSimulationEventCallback_onSleep_mut";

PxSimulationEventCallback_onContact :: (self: *physx_PxSimulationEventCallback, pairHeader: *physx_PxContactPairHeader, pairs: *physx_PxContactPair, nbPairs: u32) -> void #foreign physx_sys "PxSimulationEventCallback_onContact_mut";

PxSimulationEventCallback_onTrigger :: (self: *physx_PxSimulationEventCallback, pairs: *physx_PxTriggerPair, count: u32) -> void #foreign physx_sys "PxSimulationEventCallback_onTrigger_mut";

PxSimulationEventCallback_onAdvance :: (self: *physx_PxSimulationEventCallback, bodyBuffer: **physx_PxRigidBody, poseBuffer: *physx_PxTransform, count: u32) -> void #foreign physx_sys "PxSimulationEventCallback_onAdvance_mut";

PxSimulationEventCallback_delete :: (self: *physx_PxSimulationEventCallback) -> void #foreign physx_sys;

PxFEMParameters_new :: () -> physx_PxFEMParameters #foreign physx_sys;

PxPruningStructure_release :: (self: *physx_PxPruningStructure) -> void #foreign physx_sys "PxPruningStructure_release_mut";

PxPruningStructure_getRigidActors :: (self: *physx_PxPruningStructure, userBuffer: **physx_PxRigidActor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPruningStructure_getNbRigidActors :: (self: *physx_PxPruningStructure) -> u32 #foreign physx_sys;

PxPruningStructure_getStaticMergeData :: (self: *physx_PxPruningStructure) -> *void #foreign physx_sys;

PxPruningStructure_getDynamicMergeData :: (self: *physx_PxPruningStructure) -> *void #foreign physx_sys;

PxPruningStructure_getConcreteTypeName :: (self: *physx_PxPruningStructure) -> *u8 #foreign physx_sys;

PxExtendedVec3_new :: () -> physx_PxExtendedVec3 #foreign physx_sys;

PxExtendedVec3_new_1 :: (_x: float64, _y: float64, _z: float64) -> physx_PxExtendedVec3 #foreign physx_sys;

PxExtendedVec3_isZero :: (self: *physx_PxExtendedVec3) -> bool #foreign physx_sys;

PxExtendedVec3_dot :: (self: *physx_PxExtendedVec3, v: *Vector3) -> float64 #foreign physx_sys;

PxExtendedVec3_distanceSquared :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_magnitudeSquared :: (self: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_magnitude :: (self: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_normalize :: (self: *physx_PxExtendedVec3) -> float64 #foreign physx_sys "PxExtendedVec3_normalize_mut";

PxExtendedVec3_isFinite :: (self: *physx_PxExtendedVec3) -> bool #foreign physx_sys;

PxExtendedVec3_maximum :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> void #foreign physx_sys "PxExtendedVec3_maximum_mut";

PxExtendedVec3_minimum :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> void #foreign physx_sys "PxExtendedVec3_minimum_mut";

PxExtendedVec3_set :: (self: *physx_PxExtendedVec3, x_: float64, y_: float64, z_: float64) -> void #foreign physx_sys "PxExtendedVec3_set_mut";

PxExtendedVec3_setPlusInfinity :: (self: *physx_PxExtendedVec3) -> void #foreign physx_sys "PxExtendedVec3_setPlusInfinity_mut";

PxExtendedVec3_setMinusInfinity :: (self: *physx_PxExtendedVec3) -> void #foreign physx_sys "PxExtendedVec3_setMinusInfinity_mut";

PxExtendedVec3_cross :: (self: *physx_PxExtendedVec3, left: *physx_PxExtendedVec3, right: *Vector3) -> void #foreign physx_sys "PxExtendedVec3_cross_mut";

PxExtendedVec3_cross :: (self: *physx_PxExtendedVec3, left: *physx_PxExtendedVec3, right: *physx_PxExtendedVec3) -> void #foreign physx_sys "PxExtendedVec3_cross_mut_1";

PxExtendedVec3_cross :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> physx_PxExtendedVec3 #foreign physx_sys;

PxExtendedVec3_cross :: (self: *physx_PxExtendedVec3, left: *Vector3, right: *physx_PxExtendedVec3) -> void #foreign physx_sys "PxExtendedVec3_cross_mut_2";

phys_toVec3 :: (v: *physx_PxExtendedVec3) -> Vector3 #foreign physx_sys;

PxObstacle_getType :: (self: *physx_PxObstacle) -> s32 #foreign physx_sys;

PxBoxObstacle_new :: () -> physx_PxBoxObstacle #foreign physx_sys;

PxCapsuleObstacle_new :: () -> physx_PxCapsuleObstacle #foreign physx_sys;

PxObstacleContext_release :: (self: *physx_PxObstacleContext) -> void #foreign physx_sys "PxObstacleContext_release_mut";

PxObstacleContext_getControllerManager :: (self: *physx_PxObstacleContext) -> *physx_PxControllerManager #foreign physx_sys;

PxObstacleContext_addObstacle :: (self: *physx_PxObstacleContext, obstacle: *physx_PxObstacle) -> u32 #foreign physx_sys "PxObstacleContext_addObstacle_mut";

PxObstacleContext_removeObstacle :: (self: *physx_PxObstacleContext, handle: u32) -> bool #foreign physx_sys "PxObstacleContext_removeObstacle_mut";

PxObstacleContext_updateObstacle :: (self: *physx_PxObstacleContext, handle: u32, obstacle: *physx_PxObstacle) -> bool #foreign physx_sys "PxObstacleContext_updateObstacle_mut";

PxObstacleContext_getNbObstacles :: (self: *physx_PxObstacleContext) -> u32 #foreign physx_sys;

PxObstacleContext_getObstacle :: (self: *physx_PxObstacleContext, i: u32) -> *physx_PxObstacle #foreign physx_sys;

PxObstacleContext_getObstacleByHandle :: (self: *physx_PxObstacleContext, handle: u32) -> *physx_PxObstacle #foreign physx_sys;

PxUserControllerHitReport_onShapeHit :: (self: *physx_PxUserControllerHitReport, hit: *physx_PxControllerShapeHit) -> void #foreign physx_sys "PxUserControllerHitReport_onShapeHit_mut";

PxUserControllerHitReport_onControllerHit :: (self: *physx_PxUserControllerHitReport, hit: *physx_PxControllersHit) -> void #foreign physx_sys "PxUserControllerHitReport_onControllerHit_mut";

PxUserControllerHitReport_onObstacleHit :: (self: *physx_PxUserControllerHitReport, hit: *physx_PxControllerObstacleHit) -> void #foreign physx_sys "PxUserControllerHitReport_onObstacleHit_mut";

PxControllerFilterCallback_delete :: (self: *physx_PxControllerFilterCallback) -> void #foreign physx_sys;

PxControllerFilterCallback_filter :: (self: *physx_PxControllerFilterCallback, a: *physx_PxController, b: *physx_PxController) -> bool #foreign physx_sys "PxControllerFilterCallback_filter_mut";

PxControllerFilters_new :: (filterData: *physx_PxFilterData, cb: *physx_PxQueryFilterCallback, cctFilterCb: *physx_PxControllerFilterCallback) -> physx_PxControllerFilters #foreign physx_sys;

PxControllerDesc_isValid :: (self: *physx_PxControllerDesc) -> bool #foreign physx_sys;

PxControllerDesc_getType :: (self: *physx_PxControllerDesc) -> s32 #foreign physx_sys;

PxController_getType :: (self: *physx_PxController) -> s32 #foreign physx_sys;

PxController_release :: (self: *physx_PxController) -> void #foreign physx_sys "PxController_release_mut";

PxController_move :: (self: *physx_PxController, disp: *Vector3, minDist: float, elapsedTime: float, filters: *physx_PxControllerFilters, obstacles: *physx_PxObstacleContext) -> u8 #foreign physx_sys "PxController_move_mut";

PxController_setPosition :: (self: *physx_PxController, position: *physx_PxExtendedVec3) -> bool #foreign physx_sys "PxController_setPosition_mut";

PxController_getPosition :: (self: *physx_PxController) -> *physx_PxExtendedVec3 #foreign physx_sys;

PxController_setFootPosition :: (self: *physx_PxController, position: *physx_PxExtendedVec3) -> bool #foreign physx_sys "PxController_setFootPosition_mut";

PxController_getFootPosition :: (self: *physx_PxController) -> physx_PxExtendedVec3 #foreign physx_sys;

PxController_getActor :: (self: *physx_PxController) -> *physx_PxRigidDynamic #foreign physx_sys;

PxController_setStepOffset :: (self: *physx_PxController, offset: float) -> void #foreign physx_sys "PxController_setStepOffset_mut";

PxController_getStepOffset :: (self: *physx_PxController) -> float #foreign physx_sys;

PxController_setNonWalkableMode :: (self: *physx_PxController, flag: s32) -> void #foreign physx_sys "PxController_setNonWalkableMode_mut";

PxController_getNonWalkableMode :: (self: *physx_PxController) -> s32 #foreign physx_sys;

PxController_getContactOffset :: (self: *physx_PxController) -> float #foreign physx_sys;

PxController_setContactOffset :: (self: *physx_PxController, offset: float) -> void #foreign physx_sys "PxController_setContactOffset_mut";

PxController_getUpDirection :: (self: *physx_PxController) -> Vector3 #foreign physx_sys;

PxController_setUpDirection :: (self: *physx_PxController, up: *Vector3) -> void #foreign physx_sys "PxController_setUpDirection_mut";

PxController_getSlopeLimit :: (self: *physx_PxController) -> float #foreign physx_sys;

PxController_setSlopeLimit :: (self: *physx_PxController, slopeLimit: float) -> void #foreign physx_sys "PxController_setSlopeLimit_mut";

PxController_invalidateCache :: (self: *physx_PxController) -> void #foreign physx_sys "PxController_invalidateCache_mut";

PxController_getScene :: (self: *physx_PxController) -> *physx_PxScene #foreign physx_sys "PxController_getScene_mut";

PxController_getUserData :: (self: *physx_PxController) -> *void #foreign physx_sys;

PxController_setUserData :: (self: *physx_PxController, userData: *void) -> void #foreign physx_sys "PxController_setUserData_mut";

PxController_getState :: (self: *physx_PxController, state: *physx_PxControllerState) -> void #foreign physx_sys;

PxController_getStats :: (self: *physx_PxController, stats: *physx_PxControllerStats) -> void #foreign physx_sys;

PxController_resize :: (self: *physx_PxController, height: float) -> void #foreign physx_sys "PxController_resize_mut";

PxBoxControllerDesc_new_alloc :: () -> *physx_PxBoxControllerDesc #foreign physx_sys;

PxBoxControllerDesc_delete :: (self: *physx_PxBoxControllerDesc) -> void #foreign physx_sys;

PxBoxControllerDesc_setToDefault :: (self: *physx_PxBoxControllerDesc) -> void #foreign physx_sys "PxBoxControllerDesc_setToDefault_mut";

PxBoxControllerDesc_isValid :: (self: *physx_PxBoxControllerDesc) -> bool #foreign physx_sys;

PxBoxController_getHalfHeight :: (self: *physx_PxBoxController) -> float #foreign physx_sys;

PxBoxController_getHalfSideExtent :: (self: *physx_PxBoxController) -> float #foreign physx_sys;

PxBoxController_getHalfForwardExtent :: (self: *physx_PxBoxController) -> float #foreign physx_sys;

PxBoxController_setHalfHeight :: (self: *physx_PxBoxController, halfHeight: float) -> bool #foreign physx_sys "PxBoxController_setHalfHeight_mut";

PxBoxController_setHalfSideExtent :: (self: *physx_PxBoxController, halfSideExtent: float) -> bool #foreign physx_sys "PxBoxController_setHalfSideExtent_mut";

PxBoxController_setHalfForwardExtent :: (self: *physx_PxBoxController, halfForwardExtent: float) -> bool #foreign physx_sys "PxBoxController_setHalfForwardExtent_mut";

PxCapsuleControllerDesc_new_alloc :: () -> *physx_PxCapsuleControllerDesc #foreign physx_sys;

PxCapsuleControllerDesc_delete :: (self: *physx_PxCapsuleControllerDesc) -> void #foreign physx_sys;

PxCapsuleControllerDesc_setToDefault :: (self: *physx_PxCapsuleControllerDesc) -> void #foreign physx_sys "PxCapsuleControllerDesc_setToDefault_mut";

PxCapsuleControllerDesc_isValid :: (self: *physx_PxCapsuleControllerDesc) -> bool #foreign physx_sys;

PxCapsuleController_getRadius :: (self: *physx_PxCapsuleController) -> float #foreign physx_sys;

PxCapsuleController_setRadius :: (self: *physx_PxCapsuleController, radius: float) -> bool #foreign physx_sys "PxCapsuleController_setRadius_mut";

PxCapsuleController_getHeight :: (self: *physx_PxCapsuleController) -> float #foreign physx_sys;

PxCapsuleController_setHeight :: (self: *physx_PxCapsuleController, height: float) -> bool #foreign physx_sys "PxCapsuleController_setHeight_mut";

PxCapsuleController_getClimbingMode :: (self: *physx_PxCapsuleController) -> s32 #foreign physx_sys;

PxCapsuleController_setClimbingMode :: (self: *physx_PxCapsuleController, mode: s32) -> bool #foreign physx_sys "PxCapsuleController_setClimbingMode_mut";

PxControllerBehaviorCallback_getBehaviorFlags :: (self: *physx_PxControllerBehaviorCallback, shape: *physx_PxShape, actor: *physx_PxActor) -> u8 #foreign physx_sys "PxControllerBehaviorCallback_getBehaviorFlags_mut";

PxControllerBehaviorCallback_getBehaviorFlags :: (self: *physx_PxControllerBehaviorCallback, controller: *physx_PxController) -> u8 #foreign physx_sys "PxControllerBehaviorCallback_getBehaviorFlags_mut_1";

PxControllerBehaviorCallback_getBehaviorFlags :: (self: *physx_PxControllerBehaviorCallback, obstacle: *physx_PxObstacle) -> u8 #foreign physx_sys "PxControllerBehaviorCallback_getBehaviorFlags_mut_2";

PxControllerManager_release :: (self: *physx_PxControllerManager) -> void #foreign physx_sys "PxControllerManager_release_mut";

PxControllerManager_getScene :: (self: *physx_PxControllerManager) -> *physx_PxScene #foreign physx_sys;

PxControllerManager_getNbControllers :: (self: *physx_PxControllerManager) -> u32 #foreign physx_sys;

PxControllerManager_getController :: (self: *physx_PxControllerManager, index: u32) -> *physx_PxController #foreign physx_sys "PxControllerManager_getController_mut";

PxControllerManager_createController :: (self: *physx_PxControllerManager, desc: *physx_PxControllerDesc) -> *physx_PxController #foreign physx_sys "PxControllerManager_createController_mut";

PxControllerManager_purgeControllers :: (self: *physx_PxControllerManager) -> void #foreign physx_sys "PxControllerManager_purgeControllers_mut";

PxControllerManager_getRenderBuffer :: (self: *physx_PxControllerManager) -> *physx_PxRenderBuffer #foreign physx_sys "PxControllerManager_getRenderBuffer_mut";

PxControllerManager_setDebugRenderingFlags :: (self: *physx_PxControllerManager, flags: u32) -> void #foreign physx_sys "PxControllerManager_setDebugRenderingFlags_mut";

PxControllerManager_getNbObstacleContexts :: (self: *physx_PxControllerManager) -> u32 #foreign physx_sys;

PxControllerManager_getObstacleContext :: (self: *physx_PxControllerManager, index: u32) -> *physx_PxObstacleContext #foreign physx_sys "PxControllerManager_getObstacleContext_mut";

PxControllerManager_createObstacleContext :: (self: *physx_PxControllerManager) -> *physx_PxObstacleContext #foreign physx_sys "PxControllerManager_createObstacleContext_mut";

PxControllerManager_computeInteractions :: (self: *physx_PxControllerManager, elapsedTime: float, cctFilterCb: *physx_PxControllerFilterCallback) -> void #foreign physx_sys "PxControllerManager_computeInteractions_mut";

PxControllerManager_setTessellation :: (self: *physx_PxControllerManager, flag: bool, maxEdgeLength: float) -> void #foreign physx_sys "PxControllerManager_setTessellation_mut";

PxControllerManager_setOverlapRecoveryModule :: (self: *physx_PxControllerManager, flag: bool) -> void #foreign physx_sys "PxControllerManager_setOverlapRecoveryModule_mut";

PxControllerManager_setPreciseSweeps :: (self: *physx_PxControllerManager, flag: bool) -> void #foreign physx_sys "PxControllerManager_setPreciseSweeps_mut";

PxControllerManager_setPreventVerticalSlidingAgainstCeiling :: (self: *physx_PxControllerManager, flag: bool) -> void #foreign physx_sys "PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut";

PxControllerManager_shiftOrigin :: (self: *physx_PxControllerManager, shift: *Vector3) -> void #foreign physx_sys "PxControllerManager_shiftOrigin_mut";

phys_PxCreateControllerManager :: (scene: *physx_PxScene, lockingEnabled: bool) -> *physx_PxControllerManager #foreign physx_sys;

PxDim3_new :: () -> physx_PxDim3 #foreign physx_sys;

PxSDFDesc_new :: () -> physx_PxSDFDesc #foreign physx_sys;

PxSDFDesc_isValid :: (self: *physx_PxSDFDesc) -> bool #foreign physx_sys;

PxConvexMeshDesc_new :: () -> physx_PxConvexMeshDesc #foreign physx_sys;

PxConvexMeshDesc_setToDefault :: (self: *physx_PxConvexMeshDesc) -> void #foreign physx_sys "PxConvexMeshDesc_setToDefault_mut";

PxConvexMeshDesc_isValid :: (self: *physx_PxConvexMeshDesc) -> bool #foreign physx_sys;

PxTriangleMeshDesc_new :: () -> physx_PxTriangleMeshDesc #foreign physx_sys;

PxTriangleMeshDesc_setToDefault :: (self: *physx_PxTriangleMeshDesc) -> void #foreign physx_sys "PxTriangleMeshDesc_setToDefault_mut";

PxTriangleMeshDesc_isValid :: (self: *physx_PxTriangleMeshDesc) -> bool #foreign physx_sys;

PxTetrahedronMeshDesc_new :: () -> physx_PxTetrahedronMeshDesc #foreign physx_sys;

PxTetrahedronMeshDesc_isValid :: (self: *physx_PxTetrahedronMeshDesc) -> bool #foreign physx_sys;

PxSoftBodySimulationDataDesc_new :: () -> physx_PxSoftBodySimulationDataDesc #foreign physx_sys;

PxSoftBodySimulationDataDesc_isValid :: (self: *physx_PxSoftBodySimulationDataDesc) -> bool #foreign physx_sys;

PxBVH34MidphaseDesc_setToDefault :: (self: *physx_PxBVH34MidphaseDesc) -> void #foreign physx_sys "PxBVH34MidphaseDesc_setToDefault_mut";

PxBVH34MidphaseDesc_isValid :: (self: *physx_PxBVH34MidphaseDesc) -> bool #foreign physx_sys;

PxMidphaseDesc_new :: () -> physx_PxMidphaseDesc #foreign physx_sys;

PxMidphaseDesc_getType :: (self: *physx_PxMidphaseDesc) -> s32 #foreign physx_sys;

PxMidphaseDesc_setToDefault :: (self: *physx_PxMidphaseDesc, type: s32) -> void #foreign physx_sys "PxMidphaseDesc_setToDefault_mut";

PxMidphaseDesc_isValid :: (self: *physx_PxMidphaseDesc) -> bool #foreign physx_sys;

PxBVHDesc_new :: () -> physx_PxBVHDesc #foreign physx_sys;

PxBVHDesc_setToDefault :: (self: *physx_PxBVHDesc) -> void #foreign physx_sys "PxBVHDesc_setToDefault_mut";

PxBVHDesc_isValid :: (self: *physx_PxBVHDesc) -> bool #foreign physx_sys;

PxCookingParams_new :: (sc: *physx_PxTolerancesScale) -> physx_PxCookingParams #foreign physx_sys;

phys_PxGetStandaloneInsertionCallback :: () -> *physx_PxInsertionCallback #foreign physx_sys;

phys_PxCookBVH :: (desc: *physx_PxBVHDesc, stream: *physx_PxOutputStream) -> bool #foreign physx_sys;

phys_PxCreateBVH :: (desc: *physx_PxBVHDesc, insertionCallback: *physx_PxInsertionCallback) -> *physx_PxBVH #foreign physx_sys;

phys_PxCookHeightField :: (desc: *physx_PxHeightFieldDesc, stream: *physx_PxOutputStream) -> bool #foreign physx_sys;

phys_PxCreateHeightField :: (desc: *physx_PxHeightFieldDesc, insertionCallback: *physx_PxInsertionCallback) -> *physx_PxHeightField #foreign physx_sys;

phys_PxCookConvexMesh :: (params: *physx_PxCookingParams, desc: *physx_PxConvexMeshDesc, stream: *physx_PxOutputStream, condition: *s32) -> bool #foreign physx_sys;

phys_PxCreateConvexMesh :: (params: *physx_PxCookingParams, desc: *physx_PxConvexMeshDesc, insertionCallback: *physx_PxInsertionCallback, condition: *s32) -> *physx_PxConvexMesh #foreign physx_sys;

phys_PxValidateConvexMesh :: (params: *physx_PxCookingParams, desc: *physx_PxConvexMeshDesc) -> bool #foreign physx_sys;

phys_PxComputeHullPolygons :: (params: *physx_PxCookingParams, mesh: *physx_PxSimpleTriangleMesh, inCallback: *physx_PxAllocatorCallback, nbVerts: *u32, vertices: **Vector3, nbIndices: *u32, indices: **u32, nbPolygons: *u32, hullPolygons: **physx_PxHullPolygon) -> bool #foreign physx_sys;

phys_PxValidateTriangleMesh :: (params: *physx_PxCookingParams, desc: *physx_PxTriangleMeshDesc) -> bool #foreign physx_sys;

phys_PxCreateTriangleMesh :: (params: *physx_PxCookingParams, desc: *physx_PxTriangleMeshDesc, insertionCallback: *physx_PxInsertionCallback, condition: *s32) -> *physx_PxTriangleMesh #foreign physx_sys;

phys_PxCookTriangleMesh :: (params: *physx_PxCookingParams, desc: *physx_PxTriangleMeshDesc, stream: *physx_PxOutputStream, condition: *s32) -> bool #foreign physx_sys;

PxDefaultMemoryOutputStream_new_alloc :: (allocator: *physx_PxAllocatorCallback) -> *physx_PxDefaultMemoryOutputStream #foreign physx_sys;

PxDefaultMemoryOutputStream_delete :: (self: *physx_PxDefaultMemoryOutputStream) -> void #foreign physx_sys;

PxDefaultMemoryOutputStream_write :: (self: *physx_PxDefaultMemoryOutputStream, src: *void, count: u32) -> u32 #foreign physx_sys "PxDefaultMemoryOutputStream_write_mut";

PxDefaultMemoryOutputStream_getSize :: (self: *physx_PxDefaultMemoryOutputStream) -> u32 #foreign physx_sys;

PxDefaultMemoryOutputStream_getData :: (self: *physx_PxDefaultMemoryOutputStream) -> *u8 #foreign physx_sys;

PxDefaultMemoryInputData_new_alloc :: (data: *u8, length: u32) -> *physx_PxDefaultMemoryInputData #foreign physx_sys;

PxDefaultMemoryInputData_read :: (self: *physx_PxDefaultMemoryInputData, dest: *void, count: u32) -> u32 #foreign physx_sys "PxDefaultMemoryInputData_read_mut";

PxDefaultMemoryInputData_getLength :: (self: *physx_PxDefaultMemoryInputData) -> u32 #foreign physx_sys;

PxDefaultMemoryInputData_seek :: (self: *physx_PxDefaultMemoryInputData, pos: u32) -> void #foreign physx_sys "PxDefaultMemoryInputData_seek_mut";

PxDefaultMemoryInputData_tell :: (self: *physx_PxDefaultMemoryInputData) -> u32 #foreign physx_sys;

PxDefaultFileOutputStream_new_alloc :: (name: *u8) -> *physx_PxDefaultFileOutputStream #foreign physx_sys;

PxDefaultFileOutputStream_delete :: (self: *physx_PxDefaultFileOutputStream) -> void #foreign physx_sys;

PxDefaultFileOutputStream_write :: (self: *physx_PxDefaultFileOutputStream, src: *void, count: u32) -> u32 #foreign physx_sys "PxDefaultFileOutputStream_write_mut";

PxDefaultFileOutputStream_isValid :: (self: *physx_PxDefaultFileOutputStream) -> bool #foreign physx_sys "PxDefaultFileOutputStream_isValid_mut";

PxDefaultFileInputData_new_alloc :: (name: *u8) -> *physx_PxDefaultFileInputData #foreign physx_sys;

PxDefaultFileInputData_delete :: (self: *physx_PxDefaultFileInputData) -> void #foreign physx_sys;

PxDefaultFileInputData_read :: (self: *physx_PxDefaultFileInputData, dest: *void, count: u32) -> u32 #foreign physx_sys "PxDefaultFileInputData_read_mut";

PxDefaultFileInputData_seek :: (self: *physx_PxDefaultFileInputData, pos: u32) -> void #foreign physx_sys "PxDefaultFileInputData_seek_mut";

PxDefaultFileInputData_tell :: (self: *physx_PxDefaultFileInputData) -> u32 #foreign physx_sys;

PxDefaultFileInputData_getLength :: (self: *physx_PxDefaultFileInputData) -> u32 #foreign physx_sys;

PxDefaultFileInputData_isValid :: (self: *physx_PxDefaultFileInputData) -> bool #foreign physx_sys;

phys_platformAlignedAlloc :: (size: u64) -> *void #foreign physx_sys;

phys_platformAlignedFree :: (ptr: *void) -> void #foreign physx_sys;

PxDefaultAllocator_allocate :: (self: *physx_PxDefaultAllocator, size: u64, anon_param1: *u8, anon_param2: *u8, anon_param3: s32) -> *void #foreign physx_sys "PxDefaultAllocator_allocate_mut";

PxDefaultAllocator_deallocate :: (self: *physx_PxDefaultAllocator, ptr: *void) -> void #foreign physx_sys "PxDefaultAllocator_deallocate_mut";

PxDefaultAllocator_delete :: (self: *physx_PxDefaultAllocator) -> void #foreign physx_sys;

PxJoint_setActors :: (self: *physx_PxJoint, actor0: *physx_PxRigidActor, actor1: *physx_PxRigidActor) -> void #foreign physx_sys "PxJoint_setActors_mut";

PxJoint_getActors :: (self: *physx_PxJoint, actor0: **physx_PxRigidActor, actor1: **physx_PxRigidActor) -> void #foreign physx_sys;

PxJoint_setLocalPose :: (self: *physx_PxJoint, actor: s32, localPose: *physx_PxTransform) -> void #foreign physx_sys "PxJoint_setLocalPose_mut";

PxJoint_getLocalPose :: (self: *physx_PxJoint, actor: s32) -> physx_PxTransform #foreign physx_sys;

PxJoint_getRelativeTransform :: (self: *physx_PxJoint) -> physx_PxTransform #foreign physx_sys;

PxJoint_getRelativeLinearVelocity :: (self: *physx_PxJoint) -> Vector3 #foreign physx_sys;

PxJoint_getRelativeAngularVelocity :: (self: *physx_PxJoint) -> Vector3 #foreign physx_sys;

PxJoint_setBreakForce :: (self: *physx_PxJoint, force: float, torque: float) -> void #foreign physx_sys "PxJoint_setBreakForce_mut";

PxJoint_getBreakForce :: (self: *physx_PxJoint, force: *float, torque: *float) -> void #foreign physx_sys;

PxJoint_setConstraintFlags :: (self: *physx_PxJoint, flags: u16) -> void #foreign physx_sys "PxJoint_setConstraintFlags_mut";

PxJoint_setConstraintFlag :: (self: *physx_PxJoint, flag: s32, value: bool) -> void #foreign physx_sys "PxJoint_setConstraintFlag_mut";

PxJoint_getConstraintFlags :: (self: *physx_PxJoint) -> u16 #foreign physx_sys;

PxJoint_setInvMassScale0 :: (self: *physx_PxJoint, invMassScale: float) -> void #foreign physx_sys "PxJoint_setInvMassScale0_mut";

PxJoint_getInvMassScale0 :: (self: *physx_PxJoint) -> float #foreign physx_sys;

PxJoint_setInvInertiaScale0 :: (self: *physx_PxJoint, invInertiaScale: float) -> void #foreign physx_sys "PxJoint_setInvInertiaScale0_mut";

PxJoint_getInvInertiaScale0 :: (self: *physx_PxJoint) -> float #foreign physx_sys;

PxJoint_setInvMassScale1 :: (self: *physx_PxJoint, invMassScale: float) -> void #foreign physx_sys "PxJoint_setInvMassScale1_mut";

PxJoint_getInvMassScale1 :: (self: *physx_PxJoint) -> float #foreign physx_sys;

PxJoint_setInvInertiaScale1 :: (self: *physx_PxJoint, invInertiaScale: float) -> void #foreign physx_sys "PxJoint_setInvInertiaScale1_mut";

PxJoint_getInvInertiaScale1 :: (self: *physx_PxJoint) -> float #foreign physx_sys;

PxJoint_getConstraint :: (self: *physx_PxJoint) -> *physx_PxConstraint #foreign physx_sys;

PxJoint_setName :: (self: *physx_PxJoint, name: *u8) -> void #foreign physx_sys "PxJoint_setName_mut";

PxJoint_getName :: (self: *physx_PxJoint) -> *u8 #foreign physx_sys;

PxJoint_release :: (self: *physx_PxJoint) -> void #foreign physx_sys "PxJoint_release_mut";

PxJoint_getScene :: (self: *physx_PxJoint) -> *physx_PxScene #foreign physx_sys;

PxJoint_getBinaryMetaData :: (stream: *physx_PxOutputStream) -> void #foreign physx_sys;

PxSpring_new :: (stiffness_: float, damping_: float) -> physx_PxSpring #foreign physx_sys;

phys_PxSetJointGlobalFrame :: (joint: *physx_PxJoint, wsAnchor: *Vector3, wsAxis: *Vector3) -> void #foreign physx_sys;

phys_PxDistanceJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxDistanceJoint #foreign physx_sys;

PxDistanceJoint_getDistance :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setMinDistance :: (self: *physx_PxDistanceJoint, distance: float) -> void #foreign physx_sys "PxDistanceJoint_setMinDistance_mut";

PxDistanceJoint_getMinDistance :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setMaxDistance :: (self: *physx_PxDistanceJoint, distance: float) -> void #foreign physx_sys "PxDistanceJoint_setMaxDistance_mut";

PxDistanceJoint_getMaxDistance :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setTolerance :: (self: *physx_PxDistanceJoint, tolerance: float) -> void #foreign physx_sys "PxDistanceJoint_setTolerance_mut";

PxDistanceJoint_getTolerance :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setStiffness :: (self: *physx_PxDistanceJoint, stiffness: float) -> void #foreign physx_sys "PxDistanceJoint_setStiffness_mut";

PxDistanceJoint_getStiffness :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setDamping :: (self: *physx_PxDistanceJoint, damping: float) -> void #foreign physx_sys "PxDistanceJoint_setDamping_mut";

PxDistanceJoint_getDamping :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setContactDistance :: (self: *physx_PxDistanceJoint, contactDistance: float) -> void #foreign physx_sys "PxDistanceJoint_setContactDistance_mut";

PxDistanceJoint_getContactDistance :: (self: *physx_PxDistanceJoint) -> float #foreign physx_sys;

PxDistanceJoint_setDistanceJointFlags :: (self: *physx_PxDistanceJoint, flags: u16) -> void #foreign physx_sys "PxDistanceJoint_setDistanceJointFlags_mut";

PxDistanceJoint_setDistanceJointFlag :: (self: *physx_PxDistanceJoint, flag: s32, value: bool) -> void #foreign physx_sys "PxDistanceJoint_setDistanceJointFlag_mut";

PxDistanceJoint_getDistanceJointFlags :: (self: *physx_PxDistanceJoint) -> u16 #foreign physx_sys;

PxDistanceJoint_getConcreteTypeName :: (self: *physx_PxDistanceJoint) -> *u8 #foreign physx_sys;

phys_PxContactJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxContactJoint #foreign physx_sys;

PxJacobianRow_new :: () -> physx_PxJacobianRow #foreign physx_sys;

PxJacobianRow_new_1 :: (lin0: *Vector3, lin1: *Vector3, ang0: *Vector3, ang1: *Vector3) -> physx_PxJacobianRow #foreign physx_sys;

PxContactJoint_setContact :: (self: *physx_PxContactJoint, contact: *Vector3) -> void #foreign physx_sys "PxContactJoint_setContact_mut";

PxContactJoint_setContactNormal :: (self: *physx_PxContactJoint, contactNormal: *Vector3) -> void #foreign physx_sys "PxContactJoint_setContactNormal_mut";

PxContactJoint_setPenetration :: (self: *physx_PxContactJoint, penetration: float) -> void #foreign physx_sys "PxContactJoint_setPenetration_mut";

PxContactJoint_getContact :: (self: *physx_PxContactJoint) -> Vector3 #foreign physx_sys;

PxContactJoint_getContactNormal :: (self: *physx_PxContactJoint) -> Vector3 #foreign physx_sys;

PxContactJoint_getPenetration :: (self: *physx_PxContactJoint) -> float #foreign physx_sys;

PxContactJoint_getRestitution :: (self: *physx_PxContactJoint) -> float #foreign physx_sys;

PxContactJoint_setRestitution :: (self: *physx_PxContactJoint, restitution: float) -> void #foreign physx_sys "PxContactJoint_setRestitution_mut";

PxContactJoint_getBounceThreshold :: (self: *physx_PxContactJoint) -> float #foreign physx_sys;

PxContactJoint_setBounceThreshold :: (self: *physx_PxContactJoint, bounceThreshold: float) -> void #foreign physx_sys "PxContactJoint_setBounceThreshold_mut";

PxContactJoint_getConcreteTypeName :: (self: *physx_PxContactJoint) -> *u8 #foreign physx_sys;

PxContactJoint_computeJacobians :: (self: *physx_PxContactJoint, jacobian: *physx_PxJacobianRow) -> void #foreign physx_sys;

PxContactJoint_getNbJacobianRows :: (self: *physx_PxContactJoint) -> u32 #foreign physx_sys;

phys_PxFixedJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxFixedJoint #foreign physx_sys;

PxFixedJoint_getConcreteTypeName :: (self: *physx_PxFixedJoint) -> *u8 #foreign physx_sys;

PxJointLimitParameters_new_alloc :: () -> *physx_PxJointLimitParameters #foreign physx_sys;

PxJointLimitParameters_isValid :: (self: *physx_PxJointLimitParameters) -> bool #foreign physx_sys;

PxJointLimitParameters_isSoft :: (self: *physx_PxJointLimitParameters) -> bool #foreign physx_sys;

PxJointLinearLimit_new :: (scale: *physx_PxTolerancesScale, extent: float, contactDist_deprecated: float) -> physx_PxJointLinearLimit #foreign physx_sys;

PxJointLinearLimit_new_1 :: (extent: float, spring: *physx_PxSpring) -> physx_PxJointLinearLimit #foreign physx_sys;

PxJointLinearLimit_isValid :: (self: *physx_PxJointLinearLimit) -> bool #foreign physx_sys;

PxJointLinearLimit_delete :: (self: *physx_PxJointLinearLimit) -> void #foreign physx_sys;

PxJointLinearLimitPair_new :: (scale: *physx_PxTolerancesScale, lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> physx_PxJointLinearLimitPair #foreign physx_sys;

PxJointLinearLimitPair_new_1 :: (lowerLimit: float, upperLimit: float, spring: *physx_PxSpring) -> physx_PxJointLinearLimitPair #foreign physx_sys;

PxJointLinearLimitPair_isValid :: (self: *physx_PxJointLinearLimitPair) -> bool #foreign physx_sys;

PxJointLinearLimitPair_delete :: (self: *physx_PxJointLinearLimitPair) -> void #foreign physx_sys;

PxJointAngularLimitPair_new :: (lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> physx_PxJointAngularLimitPair #foreign physx_sys;

PxJointAngularLimitPair_new_1 :: (lowerLimit: float, upperLimit: float, spring: *physx_PxSpring) -> physx_PxJointAngularLimitPair #foreign physx_sys;

PxJointAngularLimitPair_isValid :: (self: *physx_PxJointAngularLimitPair) -> bool #foreign physx_sys;

PxJointAngularLimitPair_delete :: (self: *physx_PxJointAngularLimitPair) -> void #foreign physx_sys;

PxJointLimitCone_new :: (yLimitAngle: float, zLimitAngle: float, contactDist_deprecated: float) -> physx_PxJointLimitCone #foreign physx_sys;

PxJointLimitCone_new_1 :: (yLimitAngle: float, zLimitAngle: float, spring: *physx_PxSpring) -> physx_PxJointLimitCone #foreign physx_sys;

PxJointLimitCone_isValid :: (self: *physx_PxJointLimitCone) -> bool #foreign physx_sys;

PxJointLimitCone_delete :: (self: *physx_PxJointLimitCone) -> void #foreign physx_sys;

PxJointLimitPyramid_new :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, contactDist_deprecated: float) -> physx_PxJointLimitPyramid #foreign physx_sys;

PxJointLimitPyramid_new_1 :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, spring: *physx_PxSpring) -> physx_PxJointLimitPyramid #foreign physx_sys;

PxJointLimitPyramid_isValid :: (self: *physx_PxJointLimitPyramid) -> bool #foreign physx_sys;

PxJointLimitPyramid_delete :: (self: *physx_PxJointLimitPyramid) -> void #foreign physx_sys;

phys_PxPrismaticJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxPrismaticJoint #foreign physx_sys;

PxPrismaticJoint_getPosition :: (self: *physx_PxPrismaticJoint) -> float #foreign physx_sys;

PxPrismaticJoint_getVelocity :: (self: *physx_PxPrismaticJoint) -> float #foreign physx_sys;

PxPrismaticJoint_setLimit :: (self: *physx_PxPrismaticJoint, anon_param0: *physx_PxJointLinearLimitPair) -> void #foreign physx_sys "PxPrismaticJoint_setLimit_mut";

PxPrismaticJoint_getLimit :: (self: *physx_PxPrismaticJoint) -> physx_PxJointLinearLimitPair #foreign physx_sys;

PxPrismaticJoint_setPrismaticJointFlags :: (self: *physx_PxPrismaticJoint, flags: u16) -> void #foreign physx_sys "PxPrismaticJoint_setPrismaticJointFlags_mut";

PxPrismaticJoint_setPrismaticJointFlag :: (self: *physx_PxPrismaticJoint, flag: s32, value: bool) -> void #foreign physx_sys "PxPrismaticJoint_setPrismaticJointFlag_mut";

PxPrismaticJoint_getPrismaticJointFlags :: (self: *physx_PxPrismaticJoint) -> u16 #foreign physx_sys;

PxPrismaticJoint_getConcreteTypeName :: (self: *physx_PxPrismaticJoint) -> *u8 #foreign physx_sys;

phys_PxRevoluteJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxRevoluteJoint #foreign physx_sys;

PxRevoluteJoint_getAngle :: (self: *physx_PxRevoluteJoint) -> float #foreign physx_sys;

PxRevoluteJoint_getVelocity :: (self: *physx_PxRevoluteJoint) -> float #foreign physx_sys;

PxRevoluteJoint_setLimit :: (self: *physx_PxRevoluteJoint, limits: *physx_PxJointAngularLimitPair) -> void #foreign physx_sys "PxRevoluteJoint_setLimit_mut";

PxRevoluteJoint_getLimit :: (self: *physx_PxRevoluteJoint) -> physx_PxJointAngularLimitPair #foreign physx_sys;

PxRevoluteJoint_setDriveVelocity :: (self: *physx_PxRevoluteJoint, velocity: float, autowake: bool) -> void #foreign physx_sys "PxRevoluteJoint_setDriveVelocity_mut";

PxRevoluteJoint_getDriveVelocity :: (self: *physx_PxRevoluteJoint) -> float #foreign physx_sys;

PxRevoluteJoint_setDriveForceLimit :: (self: *physx_PxRevoluteJoint, limit: float) -> void #foreign physx_sys "PxRevoluteJoint_setDriveForceLimit_mut";

PxRevoluteJoint_getDriveForceLimit :: (self: *physx_PxRevoluteJoint) -> float #foreign physx_sys;

PxRevoluteJoint_setDriveGearRatio :: (self: *physx_PxRevoluteJoint, ratio: float) -> void #foreign physx_sys "PxRevoluteJoint_setDriveGearRatio_mut";

PxRevoluteJoint_getDriveGearRatio :: (self: *physx_PxRevoluteJoint) -> float #foreign physx_sys;

PxRevoluteJoint_setRevoluteJointFlags :: (self: *physx_PxRevoluteJoint, flags: u16) -> void #foreign physx_sys "PxRevoluteJoint_setRevoluteJointFlags_mut";

PxRevoluteJoint_setRevoluteJointFlag :: (self: *physx_PxRevoluteJoint, flag: s32, value: bool) -> void #foreign physx_sys "PxRevoluteJoint_setRevoluteJointFlag_mut";

PxRevoluteJoint_getRevoluteJointFlags :: (self: *physx_PxRevoluteJoint) -> u16 #foreign physx_sys;

PxRevoluteJoint_getConcreteTypeName :: (self: *physx_PxRevoluteJoint) -> *u8 #foreign physx_sys;

phys_PxSphericalJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxSphericalJoint #foreign physx_sys;

PxSphericalJoint_getLimitCone :: (self: *physx_PxSphericalJoint) -> physx_PxJointLimitCone #foreign physx_sys;

PxSphericalJoint_setLimitCone :: (self: *physx_PxSphericalJoint, limit: *physx_PxJointLimitCone) -> void #foreign physx_sys "PxSphericalJoint_setLimitCone_mut";

PxSphericalJoint_getSwingYAngle :: (self: *physx_PxSphericalJoint) -> float #foreign physx_sys;

PxSphericalJoint_getSwingZAngle :: (self: *physx_PxSphericalJoint) -> float #foreign physx_sys;

PxSphericalJoint_setSphericalJointFlags :: (self: *physx_PxSphericalJoint, flags: u16) -> void #foreign physx_sys "PxSphericalJoint_setSphericalJointFlags_mut";

PxSphericalJoint_setSphericalJointFlag :: (self: *physx_PxSphericalJoint, flag: s32, value: bool) -> void #foreign physx_sys "PxSphericalJoint_setSphericalJointFlag_mut";

PxSphericalJoint_getSphericalJointFlags :: (self: *physx_PxSphericalJoint) -> u16 #foreign physx_sys;

PxSphericalJoint_getConcreteTypeName :: (self: *physx_PxSphericalJoint) -> *u8 #foreign physx_sys;

phys_PxD6JointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxD6Joint #foreign physx_sys;

PxD6JointDrive_new :: () -> physx_PxD6JointDrive #foreign physx_sys;

PxD6JointDrive_new_1 :: (driveStiffness: float, driveDamping: float, driveForceLimit: float, isAcceleration: bool) -> physx_PxD6JointDrive #foreign physx_sys;

PxD6JointDrive_isValid :: (self: *physx_PxD6JointDrive) -> bool #foreign physx_sys;

PxD6Joint_setMotion :: (self: *physx_PxD6Joint, axis: s32, type: s32) -> void #foreign physx_sys "PxD6Joint_setMotion_mut";

PxD6Joint_getMotion :: (self: *physx_PxD6Joint, axis: s32) -> s32 #foreign physx_sys;

PxD6Joint_getTwistAngle :: (self: *physx_PxD6Joint) -> float #foreign physx_sys;

PxD6Joint_getSwingYAngle :: (self: *physx_PxD6Joint) -> float #foreign physx_sys;

PxD6Joint_getSwingZAngle :: (self: *physx_PxD6Joint) -> float #foreign physx_sys;

PxD6Joint_setDistanceLimit :: (self: *physx_PxD6Joint, limit: *physx_PxJointLinearLimit) -> void #foreign physx_sys "PxD6Joint_setDistanceLimit_mut";

PxD6Joint_getDistanceLimit :: (self: *physx_PxD6Joint) -> physx_PxJointLinearLimit #foreign physx_sys;

PxD6Joint_setLinearLimit :: (self: *physx_PxD6Joint, axis: s32, limit: *physx_PxJointLinearLimitPair) -> void #foreign physx_sys "PxD6Joint_setLinearLimit_mut";

PxD6Joint_getLinearLimit :: (self: *physx_PxD6Joint, axis: s32) -> physx_PxJointLinearLimitPair #foreign physx_sys;

PxD6Joint_setTwistLimit :: (self: *physx_PxD6Joint, limit: *physx_PxJointAngularLimitPair) -> void #foreign physx_sys "PxD6Joint_setTwistLimit_mut";

PxD6Joint_getTwistLimit :: (self: *physx_PxD6Joint) -> physx_PxJointAngularLimitPair #foreign physx_sys;

PxD6Joint_setSwingLimit :: (self: *physx_PxD6Joint, limit: *physx_PxJointLimitCone) -> void #foreign physx_sys "PxD6Joint_setSwingLimit_mut";

PxD6Joint_getSwingLimit :: (self: *physx_PxD6Joint) -> physx_PxJointLimitCone #foreign physx_sys;

PxD6Joint_setPyramidSwingLimit :: (self: *physx_PxD6Joint, limit: *physx_PxJointLimitPyramid) -> void #foreign physx_sys "PxD6Joint_setPyramidSwingLimit_mut";

PxD6Joint_getPyramidSwingLimit :: (self: *physx_PxD6Joint) -> physx_PxJointLimitPyramid #foreign physx_sys;

PxD6Joint_setDrive :: (self: *physx_PxD6Joint, index: s32, drive: *physx_PxD6JointDrive) -> void #foreign physx_sys "PxD6Joint_setDrive_mut";

PxD6Joint_getDrive :: (self: *physx_PxD6Joint, index: s32) -> physx_PxD6JointDrive #foreign physx_sys;

PxD6Joint_setDrivePosition :: (self: *physx_PxD6Joint, pose: *physx_PxTransform, autowake: bool) -> void #foreign physx_sys "PxD6Joint_setDrivePosition_mut";

PxD6Joint_getDrivePosition :: (self: *physx_PxD6Joint) -> physx_PxTransform #foreign physx_sys;

PxD6Joint_setDriveVelocity :: (self: *physx_PxD6Joint, linear: *Vector3, angular: *Vector3, autowake: bool) -> void #foreign physx_sys "PxD6Joint_setDriveVelocity_mut";

PxD6Joint_getDriveVelocity :: (self: *physx_PxD6Joint, linear: *Vector3, angular: *Vector3) -> void #foreign physx_sys;

PxD6Joint_setProjectionLinearTolerance :: (self: *physx_PxD6Joint, tolerance: float) -> void #foreign physx_sys "PxD6Joint_setProjectionLinearTolerance_mut";

PxD6Joint_getProjectionLinearTolerance :: (self: *physx_PxD6Joint) -> float #foreign physx_sys;

PxD6Joint_setProjectionAngularTolerance :: (self: *physx_PxD6Joint, tolerance: float) -> void #foreign physx_sys "PxD6Joint_setProjectionAngularTolerance_mut";

PxD6Joint_getProjectionAngularTolerance :: (self: *physx_PxD6Joint) -> float #foreign physx_sys;

PxD6Joint_getConcreteTypeName :: (self: *physx_PxD6Joint) -> *u8 #foreign physx_sys;

phys_PxGearJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxGearJoint #foreign physx_sys;

PxGearJoint_setHinges :: (self: *physx_PxGearJoint, hinge0: *physx_PxBase, hinge1: *physx_PxBase) -> bool #foreign physx_sys "PxGearJoint_setHinges_mut";

PxGearJoint_setGearRatio :: (self: *physx_PxGearJoint, ratio: float) -> void #foreign physx_sys "PxGearJoint_setGearRatio_mut";

PxGearJoint_getGearRatio :: (self: *physx_PxGearJoint) -> float #foreign physx_sys;

PxGearJoint_getConcreteTypeName :: (self: *physx_PxGearJoint) -> *u8 #foreign physx_sys;

phys_PxRackAndPinionJointCreate :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxRackAndPinionJoint #foreign physx_sys;

PxRackAndPinionJoint_setJoints :: (self: *physx_PxRackAndPinionJoint, hinge: *physx_PxBase, prismatic: *physx_PxBase) -> bool #foreign physx_sys "PxRackAndPinionJoint_setJoints_mut";

PxRackAndPinionJoint_setRatio :: (self: *physx_PxRackAndPinionJoint, ratio: float) -> void #foreign physx_sys "PxRackAndPinionJoint_setRatio_mut";

PxRackAndPinionJoint_getRatio :: (self: *physx_PxRackAndPinionJoint) -> float #foreign physx_sys;

PxRackAndPinionJoint_setData :: (self: *physx_PxRackAndPinionJoint, nbRackTeeth: u32, nbPinionTeeth: u32, rackLength: float) -> bool #foreign physx_sys "PxRackAndPinionJoint_setData_mut";

PxRackAndPinionJoint_getConcreteTypeName :: (self: *physx_PxRackAndPinionJoint) -> *u8 #foreign physx_sys;

PxGroupsMask_new_alloc :: () -> *physx_PxGroupsMask #foreign physx_sys;

PxGroupsMask_delete :: (self: *physx_PxGroupsMask) -> void #foreign physx_sys;

phys_PxDefaultSimulationFilterShader :: (attributes0: u32, filterData0: physx_PxFilterData, attributes1: u32, filterData1: physx_PxFilterData, pairFlags: *u16, constantBlock: *void, constantBlockSize: u32) -> u16 #foreign physx_sys;

phys_PxGetGroupCollisionFlag :: (group1: u16, group2: u16) -> bool #foreign physx_sys;

phys_PxSetGroupCollisionFlag :: (group1: u16, group2: u16, enable: bool) -> void #foreign physx_sys;

phys_PxGetGroup :: (actor: *physx_PxActor) -> u16 #foreign physx_sys;

phys_PxSetGroup :: (actor: *physx_PxActor, collisionGroup: u16) -> void #foreign physx_sys;

phys_PxGetFilterOps :: (op0: *s32, op1: *s32, op2: *s32) -> void #foreign physx_sys;

phys_PxSetFilterOps :: (op0: *s32, op1: *s32, op2: *s32) -> void #foreign physx_sys;

phys_PxGetFilterBool :: () -> bool #foreign physx_sys;

phys_PxSetFilterBool :: (enable: bool) -> void #foreign physx_sys;

phys_PxGetFilterConstants :: (c0: *physx_PxGroupsMask, c1: *physx_PxGroupsMask) -> void #foreign physx_sys;

phys_PxSetFilterConstants :: (c0: *physx_PxGroupsMask, c1: *physx_PxGroupsMask) -> void #foreign physx_sys;

phys_PxGetGroupsMask :: (actor: *physx_PxActor) -> physx_PxGroupsMask #foreign physx_sys;

phys_PxSetGroupsMask :: (actor: *physx_PxActor, mask: *physx_PxGroupsMask) -> void #foreign physx_sys;

PxDefaultErrorCallback_new_alloc :: () -> *physx_PxDefaultErrorCallback #foreign physx_sys;

PxDefaultErrorCallback_delete :: (self: *physx_PxDefaultErrorCallback) -> void #foreign physx_sys;

PxDefaultErrorCallback_reportError :: (self: *physx_PxDefaultErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys "PxDefaultErrorCallback_reportError_mut";

PxRigidActorExt_createExclusiveShape :: (actor: *physx_PxRigidActor, geometry: *physx_PxGeometry, materials: **physx_PxMaterial, materialCount: u16, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys;

PxRigidActorExt_createExclusiveShape_1 :: (actor: *physx_PxRigidActor, geometry: *physx_PxGeometry, material: *physx_PxMaterial, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys;

PxRigidActorExt_getRigidActorShapeLocalBoundsList :: (actor: *physx_PxRigidActor, numBounds: *u32) -> *physx_PxBounds3 #foreign physx_sys;

PxRigidActorExt_createBVHFromActor :: (physics: *physx_PxPhysics, actor: *physx_PxRigidActor) -> *physx_PxBVH #foreign physx_sys;

PxMassProperties_new :: () -> physx_PxMassProperties #foreign physx_sys;

PxMassProperties_new_1 :: (m: float, inertiaT: *physx_PxMat33, com: *Vector3) -> physx_PxMassProperties #foreign physx_sys;

PxMassProperties_new_2 :: (geometry: *physx_PxGeometry) -> physx_PxMassProperties #foreign physx_sys;

PxMassProperties_translate :: (self: *physx_PxMassProperties, t: *Vector3) -> void #foreign physx_sys "PxMassProperties_translate_mut";

PxMassProperties_getMassSpaceInertia :: (inertia: *physx_PxMat33, massFrame: *Quaternion) -> Vector3 #foreign physx_sys;

PxMassProperties_translateInertia :: (inertia: *physx_PxMat33, mass: float, t: *Vector3) -> physx_PxMat33 #foreign physx_sys;

PxMassProperties_rotateInertia :: (inertia: *physx_PxMat33, q: *Quaternion) -> physx_PxMat33 #foreign physx_sys;

PxMassProperties_scaleInertia :: (inertia: *physx_PxMat33, scaleRotation: *Quaternion, scale: *Vector3) -> physx_PxMat33 #foreign physx_sys;

PxMassProperties_sum :: (props: *physx_PxMassProperties, transforms: *physx_PxTransform, count: u32) -> physx_PxMassProperties #foreign physx_sys;

PxRigidBodyExt_updateMassAndInertia :: (body: *physx_PxRigidBody, shapeDensities: *float, shapeDensityCount: u32, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_updateMassAndInertia_1 :: (body: *physx_PxRigidBody, density: float, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_setMassAndUpdateInertia :: (body: *physx_PxRigidBody, shapeMasses: *float, shapeMassCount: u32, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_setMassAndUpdateInertia_1 :: (body: *physx_PxRigidBody, mass: float, massLocalPose: *Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_computeMassPropertiesFromShapes :: (shapes: **physx_PxShape, shapeCount: u32) -> physx_PxMassProperties #foreign physx_sys;

PxRigidBodyExt_addForceAtPos :: (body: *physx_PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addForceAtLocalPos :: (body: *physx_PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addLocalForceAtPos :: (body: *physx_PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addLocalForceAtLocalPos :: (body: *physx_PxRigidBody, force: *Vector3, pos: *Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_getVelocityAtPos :: (body: *physx_PxRigidBody, pos: *Vector3) -> Vector3 #foreign physx_sys;

PxRigidBodyExt_getLocalVelocityAtLocalPos :: (body: *physx_PxRigidBody, pos: *Vector3) -> Vector3 #foreign physx_sys;

PxRigidBodyExt_getVelocityAtOffset :: (body: *physx_PxRigidBody, pos: *Vector3) -> Vector3 #foreign physx_sys;

PxRigidBodyExt_computeVelocityDeltaFromImpulse :: (body: *physx_PxRigidBody, impulsiveForce: *Vector3, impulsiveTorque: *Vector3, deltaLinearVelocity: *Vector3, deltaAngularVelocity: *Vector3) -> void #foreign physx_sys;

PxRigidBodyExt_computeVelocityDeltaFromImpulse_1 :: (body: *physx_PxRigidBody, globalPose: *physx_PxTransform, point: *Vector3, impulse: *Vector3, invMassScale: float, invInertiaScale: float, deltaLinearVelocity: *Vector3, deltaAngularVelocity: *Vector3) -> void #foreign physx_sys;

PxRigidBodyExt_computeLinearAngularImpulse :: (body: *physx_PxRigidBody, globalPose: *physx_PxTransform, point: *Vector3, impulse: *Vector3, invMassScale: float, invInertiaScale: float, linearImpulse: *Vector3, angularImpulse: *Vector3) -> void #foreign physx_sys;

PxRigidBodyExt_linearSweepSingle :: (body: *physx_PxRigidBody, scene: *physx_PxScene, unitDir: *Vector3, distance: float, outputFlags: u16, closestHit: *physx_PxSweepHit, shapeIndex: *u32, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> bool #foreign physx_sys;

PxRigidBodyExt_linearSweepMultiple :: (body: *physx_PxRigidBody, scene: *physx_PxScene, unitDir: *Vector3, distance: float, outputFlags: u16, touchHitBuffer: *physx_PxSweepHit, touchHitShapeIndices: *u32, touchHitBufferSize: u32, block: *physx_PxSweepHit, blockingShapeIndex: *s32, overflow: *bool, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> u32 #foreign physx_sys;

PxShapeExt_getGlobalPose :: (shape: *physx_PxShape, actor: *physx_PxRigidActor) -> physx_PxTransform #foreign physx_sys;

PxShapeExt_raycast :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, rayOrigin: *Vector3, rayDir: *Vector3, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *physx_PxRaycastHit) -> u32 #foreign physx_sys;

PxShapeExt_overlap :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, otherGeom: *physx_PxGeometry, otherGeomPose: *physx_PxTransform) -> bool #foreign physx_sys;

PxShapeExt_sweep :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, unitDir: *Vector3, distance: float, otherGeom: *physx_PxGeometry, otherGeomPose: *physx_PxTransform, sweepHit: *physx_PxSweepHit, hitFlags: u16) -> bool #foreign physx_sys;

PxShapeExt_getWorldBounds :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, inflation: float) -> physx_PxBounds3 #foreign physx_sys;

PxMeshOverlapUtil_new_alloc :: () -> *physx_PxMeshOverlapUtil #foreign physx_sys;

PxMeshOverlapUtil_delete :: (self: *physx_PxMeshOverlapUtil) -> void #foreign physx_sys;

PxMeshOverlapUtil_findOverlap :: (self: *physx_PxMeshOverlapUtil, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, meshGeom: *physx_PxTriangleMeshGeometry, meshPose: *physx_PxTransform) -> u32 #foreign physx_sys "PxMeshOverlapUtil_findOverlap_mut";

PxMeshOverlapUtil_findOverlap :: (self: *physx_PxMeshOverlapUtil, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, hfGeom: *physx_PxHeightFieldGeometry, hfPose: *physx_PxTransform) -> u32 #foreign physx_sys "PxMeshOverlapUtil_findOverlap_mut_1";

PxMeshOverlapUtil_getResults :: (self: *physx_PxMeshOverlapUtil) -> *u32 #foreign physx_sys;

PxMeshOverlapUtil_getNbResults :: (self: *physx_PxMeshOverlapUtil) -> u32 #foreign physx_sys;

phys_PxComputeTriangleMeshPenetration :: (direction: *Vector3, depth: *float, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, meshGeom: *physx_PxTriangleMeshGeometry, meshPose: *physx_PxTransform, maxIter: u32, usedIter: *u32) -> bool #foreign physx_sys;

phys_PxComputeHeightFieldPenetration :: (direction: *Vector3, depth: *float, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, heightFieldGeom: *physx_PxHeightFieldGeometry, heightFieldPose: *physx_PxTransform, maxIter: u32, usedIter: *u32) -> bool #foreign physx_sys;

PxXmlMiscParameter_new :: () -> physx_PxXmlMiscParameter #foreign physx_sys;

PxXmlMiscParameter_new_1 :: (inUpVector: *Vector3, inScale: physx_PxTolerancesScale) -> physx_PxXmlMiscParameter #foreign physx_sys;

PxSerialization_isSerializable :: (collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, externalReferences: *physx_PxCollection) -> bool #foreign physx_sys;

PxSerialization_complete :: (collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, exceptFor: *physx_PxCollection, followJoints: bool) -> void #foreign physx_sys;

PxSerialization_createSerialObjectIds :: (collection: *physx_PxCollection, base: u64) -> void #foreign physx_sys;

PxSerialization_createCollectionFromXml :: (inputData: *physx_PxInputData, cooking: *physx_PxCooking, sr: *physx_PxSerializationRegistry, externalRefs: *physx_PxCollection, stringTable: *physx_PxStringTable, outArgs: *physx_PxXmlMiscParameter) -> *physx_PxCollection #foreign physx_sys;

PxSerialization_createCollectionFromBinary :: (memBlock: *void, sr: *physx_PxSerializationRegistry, externalRefs: *physx_PxCollection) -> *physx_PxCollection #foreign physx_sys;

PxSerialization_serializeCollectionToXml :: (outputStream: *physx_PxOutputStream, collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, cooking: *physx_PxCooking, externalRefs: *physx_PxCollection, inArgs: *physx_PxXmlMiscParameter) -> bool #foreign physx_sys;

PxSerialization_serializeCollectionToBinary :: (outputStream: *physx_PxOutputStream, collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, externalRefs: *physx_PxCollection, exportNames: bool) -> bool #foreign physx_sys;

PxSerialization_createSerializationRegistry :: (physics: *physx_PxPhysics) -> *physx_PxSerializationRegistry #foreign physx_sys;

PxDefaultCpuDispatcher_release :: (self: *physx_PxDefaultCpuDispatcher) -> void #foreign physx_sys "PxDefaultCpuDispatcher_release_mut";

PxDefaultCpuDispatcher_setRunProfiled :: (self: *physx_PxDefaultCpuDispatcher, runProfiled: bool) -> void #foreign physx_sys "PxDefaultCpuDispatcher_setRunProfiled_mut";

PxDefaultCpuDispatcher_getRunProfiled :: (self: *physx_PxDefaultCpuDispatcher) -> bool #foreign physx_sys;

phys_PxDefaultCpuDispatcherCreate :: (numThreads: u32, affinityMasks: *u32, mode: s32, yieldProcessorCount: u32) -> *physx_PxDefaultCpuDispatcher #foreign physx_sys;

phys_PxBuildSmoothNormals :: (nbTris: u32, nbVerts: u32, verts: *Vector3, dFaces: *u32, wFaces: *u16, normals: *Vector3, flip: bool) -> bool #foreign physx_sys;

phys_PxCreateDynamic :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, geometry: *physx_PxGeometry, material: *physx_PxMaterial, density: float, shapeOffset: *physx_PxTransform) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateDynamic_1 :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, shape: *physx_PxShape, density: float) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateKinematic :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, geometry: *physx_PxGeometry, material: *physx_PxMaterial, density: float, shapeOffset: *physx_PxTransform) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateKinematic_1 :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, shape: *physx_PxShape, density: float) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateStatic :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, geometry: *physx_PxGeometry, material: *physx_PxMaterial, shapeOffset: *physx_PxTransform) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxCreateStatic_1 :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, shape: *physx_PxShape) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxCloneShape :: (physicsSDK: *physx_PxPhysics, shape: *physx_PxShape, isExclusive: bool) -> *physx_PxShape #foreign physx_sys;

phys_PxCloneStatic :: (physicsSDK: *physx_PxPhysics, transform: *physx_PxTransform, actor: *physx_PxRigidActor) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxCloneDynamic :: (physicsSDK: *physx_PxPhysics, transform: *physx_PxTransform, body: *physx_PxRigidDynamic) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreatePlane :: (sdk: *physx_PxPhysics, plane: *Plane3, material: *physx_PxMaterial) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxScaleRigidActor :: (actor: *physx_PxRigidActor, scale: float, scaleMassProps: bool) -> void #foreign physx_sys;

PxStringTableExt_createStringTable :: (inAllocator: *physx_PxAllocatorCallback) -> *physx_PxStringTable #foreign physx_sys;

PxBroadPhaseExt_createRegionsFromWorldBounds :: (regions: *physx_PxBounds3, globalBounds: *physx_PxBounds3, nbSubdiv: u32, upAxis: u32) -> u32 #foreign physx_sys;

PxSceneQueryExt_raycastAny :: (scene: *physx_PxScene, origin: *Vector3, unitDir: *Vector3, distance: float, hit: *physx_PxQueryHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache) -> bool #foreign physx_sys;

PxSceneQueryExt_raycastSingle :: (scene: *physx_PxScene, origin: *Vector3, unitDir: *Vector3, distance: float, outputFlags: u16, hit: *physx_PxRaycastHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache) -> bool #foreign physx_sys;

PxSceneQueryExt_raycastMultiple :: (scene: *physx_PxScene, origin: *Vector3, unitDir: *Vector3, distance: float, outputFlags: u16, hitBuffer: *physx_PxRaycastHit, hitBufferSize: u32, blockingHit: *bool, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache) -> s32 #foreign physx_sys;

PxSceneQueryExt_sweepAny :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *Vector3, distance: float, queryFlags: u16, hit: *physx_PxQueryHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> bool #foreign physx_sys;

PxSceneQueryExt_sweepSingle :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *Vector3, distance: float, outputFlags: u16, hit: *physx_PxSweepHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> bool #foreign physx_sys;

PxSceneQueryExt_sweepMultiple :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *Vector3, distance: float, outputFlags: u16, hitBuffer: *physx_PxSweepHit, hitBufferSize: u32, blockingHit: *bool, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> s32 #foreign physx_sys;

PxSceneQueryExt_overlapMultiple :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, hitBuffer: *physx_PxOverlapHit, hitBufferSize: u32, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback) -> s32 #foreign physx_sys;

PxSceneQueryExt_overlapAny :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, hit: *physx_PxOverlapHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback) -> bool #foreign physx_sys;

PxBatchQueryExt_release :: (self: *physx_PxBatchQueryExt) -> void #foreign physx_sys "PxBatchQueryExt_release_mut";

PxBatchQueryExt_raycast :: (self: *physx_PxBatchQueryExt, origin: *Vector3, unitDir: *Vector3, distance: float, maxNbTouches: u16, hitFlags: u16, filterData: *physx_PxQueryFilterData, cache: *physx_PxQueryCache) -> *physx_PxRaycastBuffer #foreign physx_sys "PxBatchQueryExt_raycast_mut";

PxBatchQueryExt_sweep :: (self: *physx_PxBatchQueryExt, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *Vector3, distance: float, maxNbTouches: u16, hitFlags: u16, filterData: *physx_PxQueryFilterData, cache: *physx_PxQueryCache, inflation: float) -> *physx_PxSweepBuffer #foreign physx_sys "PxBatchQueryExt_sweep_mut";

PxBatchQueryExt_overlap :: (self: *physx_PxBatchQueryExt, geometry: *physx_PxGeometry, pose: *physx_PxTransform, maxNbTouches: u16, filterData: *physx_PxQueryFilterData, cache: *physx_PxQueryCache) -> *physx_PxOverlapBuffer #foreign physx_sys "PxBatchQueryExt_overlap_mut";

PxBatchQueryExt_execute :: (self: *physx_PxBatchQueryExt) -> void #foreign physx_sys "PxBatchQueryExt_execute_mut";

phys_PxCreateBatchQueryExt :: (scene: *physx_PxScene, queryFilterCallback: *physx_PxQueryFilterCallback, maxNbRaycasts: u32, maxNbRaycastTouches: u32, maxNbSweeps: u32, maxNbSweepTouches: u32, maxNbOverlaps: u32, maxNbOverlapTouches: u32) -> *physx_PxBatchQueryExt #foreign physx_sys;

phys_PxCreateBatchQueryExt_1 :: (scene: *physx_PxScene, queryFilterCallback: *physx_PxQueryFilterCallback, raycastBuffers: *physx_PxRaycastBuffer, maxNbRaycasts: u32, raycastTouches: *physx_PxRaycastHit, maxNbRaycastTouches: u32, sweepBuffers: *physx_PxSweepBuffer, maxNbSweeps: u32, sweepTouches: *physx_PxSweepHit, maxNbSweepTouches: u32, overlapBuffers: *physx_PxOverlapBuffer, maxNbOverlaps: u32, overlapTouches: *physx_PxOverlapHit, maxNbOverlapTouches: u32) -> *physx_PxBatchQueryExt #foreign physx_sys;

phys_PxCreateExternalSceneQuerySystem :: (desc: *physx_PxSceneQueryDesc, contextID: u64) -> *physx_PxSceneQuerySystem #foreign physx_sys;

PxCustomSceneQuerySystem_addPruner :: (self: *physx_PxCustomSceneQuerySystem, primaryType: s32, secondaryType: s32, preallocated: u32) -> u32 #foreign physx_sys "PxCustomSceneQuerySystem_addPruner_mut";

PxCustomSceneQuerySystem_startCustomBuildstep :: (self: *physx_PxCustomSceneQuerySystem) -> u32 #foreign physx_sys "PxCustomSceneQuerySystem_startCustomBuildstep_mut";

PxCustomSceneQuerySystem_customBuildstep :: (self: *physx_PxCustomSceneQuerySystem, index: u32) -> void #foreign physx_sys "PxCustomSceneQuerySystem_customBuildstep_mut";

PxCustomSceneQuerySystem_finishCustomBuildstep :: (self: *physx_PxCustomSceneQuerySystem) -> void #foreign physx_sys "PxCustomSceneQuerySystem_finishCustomBuildstep_mut";

PxCustomSceneQuerySystemAdapter_delete :: (self: *physx_PxCustomSceneQuerySystemAdapter) -> void #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_getPrunerIndex :: (self: *physx_PxCustomSceneQuerySystemAdapter, actor: *physx_PxRigidActor, shape: *physx_PxShape) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_processPruner :: (self: *physx_PxCustomSceneQuerySystemAdapter, prunerIndex: u32, context_pod: *physx_PxQueryThreadContext, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback) -> bool #foreign physx_sys;

phys_PxCreateCustomSceneQuerySystem :: (sceneQueryUpdateMode: s32, contextID: u64, adapter: *physx_PxCustomSceneQuerySystemAdapter, usesTreeOfPruners: bool) -> *physx_PxCustomSceneQuerySystem #foreign physx_sys;

phys_PxFindFaceIndex :: (convexGeom: *physx_PxConvexMeshGeometry, geomPose: *physx_PxTransform, impactPos: *Vector3, unitDir: *Vector3) -> u32 #foreign physx_sys;

PxPoissonSampler_setSamplingRadius :: (self: *physx_PxPoissonSampler, samplingRadius: float) -> bool #foreign physx_sys "PxPoissonSampler_setSamplingRadius_mut";

PxPoissonSampler_addSamplesInSphere :: (self: *physx_PxPoissonSampler, sphereCenter: *Vector3, sphereRadius: float, createVolumeSamples: bool) -> void #foreign physx_sys "PxPoissonSampler_addSamplesInSphere_mut";

PxPoissonSampler_addSamplesInBox :: (self: *physx_PxPoissonSampler, axisAlignedBox: *physx_PxBounds3, boxOrientation: *Quaternion, createVolumeSamples: bool) -> void #foreign physx_sys "PxPoissonSampler_addSamplesInBox_mut";

PxPoissonSampler_delete :: (self: *physx_PxPoissonSampler) -> void #foreign physx_sys;

phys_PxCreateShapeSampler :: (geometry: *physx_PxGeometry, transform: *physx_PxTransform, worldBounds: *physx_PxBounds3, initialSamplingRadius: float, numSampleAttemptsAroundPoint: s32) -> *physx_PxPoissonSampler #foreign physx_sys;

PxTriangleMeshPoissonSampler_isPointInTriangleMesh :: (self: *physx_PxTriangleMeshPoissonSampler, p: *Vector3) -> bool #foreign physx_sys "PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut";

PxTriangleMeshPoissonSampler_delete :: (self: *physx_PxTriangleMeshPoissonSampler) -> void #foreign physx_sys;

phys_PxCreateTriangleMeshSampler :: (triangles: *u32, numTriangles: u32, vertices: *Vector3, numVertices: u32, initialSamplingRadius: float, numSampleAttemptsAroundPoint: s32) -> *physx_PxTriangleMeshPoissonSampler #foreign physx_sys;

PxTetrahedronMeshExt_findTetrahedronContainingPoint :: (mesh: *physx_PxTetrahedronMesh, point: *Vector3, bary: *Vector4, tolerance: float) -> s32 #foreign physx_sys;

PxTetrahedronMeshExt_findTetrahedronClosestToPoint :: (mesh: *physx_PxTetrahedronMesh, point: *Vector3, bary: *Vector4) -> s32 #foreign physx_sys;

phys_PxInitExtensions :: (physics: *physx_PxPhysics, pvd: *physx_PxPvd) -> bool #foreign physx_sys;

phys_PxCloseExtensions :: () -> void #foreign physx_sys;

PxRepXObject_new :: (inTypeName: *u8, inSerializable: *void, inId: u64) -> physx_PxRepXObject #foreign physx_sys;

PxRepXObject_isValid :: (self: *physx_PxRepXObject) -> bool #foreign physx_sys;

PxRepXInstantiationArgs_new :: (inPhysics: *physx_PxPhysics, inCooking: *physx_PxCooking, inStringTable: *physx_PxStringTable) -> physx_PxRepXInstantiationArgs #foreign physx_sys;

PxRepXSerializer_getTypeName :: (self: *physx_PxRepXSerializer) -> *u8 #foreign physx_sys "PxRepXSerializer_getTypeName_mut";

PxRepXSerializer_objectToFile :: (self: *physx_PxRepXSerializer, inLiveObject: *physx_PxRepXObject, inCollection: *physx_PxCollection, inWriter: *physx_XmlWriter, inTempBuffer: *physx_MemoryBuffer, inArgs: *physx_PxRepXInstantiationArgs) -> void #foreign physx_sys "PxRepXSerializer_objectToFile_mut";

PxRepXSerializer_fileToObject :: (self: *physx_PxRepXSerializer, inReader: *physx_XmlReader, inAllocator: *physx_XmlMemoryAllocator, inArgs: *physx_PxRepXInstantiationArgs, inCollection: *physx_PxCollection) -> physx_PxRepXObject #foreign physx_sys "PxRepXSerializer_fileToObject_mut";

PxPvd_connect :: (self: *physx_PxPvd, transport: *physx_PxPvdTransport, flags: u8) -> bool #foreign physx_sys "PxPvd_connect_mut";

PxPvd_disconnect :: (self: *physx_PxPvd) -> void #foreign physx_sys "PxPvd_disconnect_mut";

PxPvd_isConnected :: (self: *physx_PxPvd, useCachedStatus: bool) -> bool #foreign physx_sys "PxPvd_isConnected_mut";

PxPvd_getTransport :: (self: *physx_PxPvd) -> *physx_PxPvdTransport #foreign physx_sys "PxPvd_getTransport_mut";

PxPvd_getInstrumentationFlags :: (self: *physx_PxPvd) -> u8 #foreign physx_sys "PxPvd_getInstrumentationFlags_mut";

PxPvd_release :: (self: *physx_PxPvd) -> void #foreign physx_sys "PxPvd_release_mut";

phys_PxCreatePvd :: (foundation: *physx_PxFoundation) -> *physx_PxPvd #foreign physx_sys;

PxPvdTransport_connect :: (self: *physx_PxPvdTransport) -> bool #foreign physx_sys "PxPvdTransport_connect_mut";

PxPvdTransport_disconnect :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys "PxPvdTransport_disconnect_mut";

PxPvdTransport_isConnected :: (self: *physx_PxPvdTransport) -> bool #foreign physx_sys "PxPvdTransport_isConnected_mut";

PxPvdTransport_write :: (self: *physx_PxPvdTransport, inBytes: *u8, inLength: u32) -> bool #foreign physx_sys "PxPvdTransport_write_mut";

PxPvdTransport_lock :: (self: *physx_PxPvdTransport) -> *physx_PxPvdTransport #foreign physx_sys "PxPvdTransport_lock_mut";

PxPvdTransport_unlock :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys "PxPvdTransport_unlock_mut";

PxPvdTransport_flush :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys "PxPvdTransport_flush_mut";

PxPvdTransport_getWrittenDataSize :: (self: *physx_PxPvdTransport) -> u64 #foreign physx_sys "PxPvdTransport_getWrittenDataSize_mut";

PxPvdTransport_release :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys "PxPvdTransport_release_mut";

phys_PxDefaultPvdSocketTransportCreate :: (host: *u8, port: s32, timeoutInMilliseconds: u32) -> *physx_PxPvdTransport #foreign physx_sys;

phys_PxDefaultPvdFileTransportCreate :: (name: *u8) -> *physx_PxPvdTransport #foreign physx_sys;

FilterShaderCallbackInfo :: struct {
    attributes0:       u32;
    attributes1:       u32;
    filterData0:       physx_PxFilterData;
    filterData1:       physx_PxFilterData;
    pairFlags:         *physx_PxPairFlags;
    constantBlock:     *void;
    constantBlockSize: u32;
}

CollisionCallback :: #type (unknown0: *void, unknown1: *physx_PxContactPairHeader, unknown2: *physx_PxContactPair, unknown3: u32) -> void #c_call;
SimulationShaderFilter :: #type (unknown0: *FilterShaderCallbackInfo) -> u16 #c_call;

FilterCallbackData :: struct {
    filter:                           SimulationShaderFilter;
    call_default_filter_shader_first: bool;
}

TriggerCallback :: #type (unknown0: *void, unknown1: *physx_PxTriggerPair, unknown2: u32) -> void #c_call;
ConstraintBreakCallback :: #type (unknown0: *void, unknown1: *physx_PxConstraintInfo, unknown2: u32) -> void #c_call;
WakeSleepCallback :: #type (unknown0: *void, unknown1: **physx_PxActor, unknown2: u32, unknown3: bool) -> void #c_call;
AdvanceCallback :: #type (unknown0: *void, unknown1: **physx_PxRigidBody, unknown2: *physx_PxTransform, unknown3: u32) -> void #c_call;

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

// TODO: Shouldn't we rename this to PreFilterCallback?
RaycastHitCallback :: #type (actor: *physx_PxRigidActor, filterData: *physx_PxFilterData, shape: *physx_PxShape, hitFlags: u32, userData: *void) -> u32 #c_call;
PostFilterCallback :: #type (filterData: *physx_PxFilterData, hit: *physx_PxQueryHit, userData: *void) -> u32 #c_call;

RaycastHitProcessTouchesCallback :: #type (buffer: *physx_PxRaycastHit, nbHits: u32, userdata: *void) -> bool #c_call;
SweepHitProcessTouchesCallback :: #type (buffer: *physx_PxSweepHit, nbHits: u32, userdata: *void) -> bool #c_call;
OverlapHitProcessTouchesCallback :: #type (buffer: *physx_PxOverlapHit, nbHits: u32, userdata: *void) -> bool #c_call;
HitFinalizeQueryCallback :: #type (userdata: *void) -> void #c_call;

AllocCallback :: #type (size: u64, typeName: *u8, filename: *u8, line: s32, userdata: *void) -> *void #c_call;
DeallocCallback :: #type (ptr: *void, userdata: *void) -> void #c_call;

ZoneStartCallback :: #type (typeName: *u8, detached: bool, _context_pod: u64, userdata: *void) -> *void #c_call;
ZoneEndCallback :: #type (profilerData: *void, typeName: *u8, detached: bool, _context_pod: u64, userdata: *void) -> void #c_call;

ErrorCallback :: #type (code: s32, message: *u8, file: *u8, line: s32, userdata: *void) -> void #c_call;

AssertHandler :: #type (expr: *u8, file: *u8, line: s32, should_ignore: *bool, userdata: *void) -> void #c_call;

physx_create_foundation :: () -> *physx_PxFoundation #foreign physx_sys;

physx_create_foundation_with_alloc :: (allocator: *physx_PxAllocatorCallback) -> *physx_PxFoundation #foreign physx_sys;

// fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
// linking against the raw interface
get_default_allocator :: () -> *physx_PxAllocatorCallback #foreign physx_sys;

// fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
// linking against the raw interface
get_default_error_callback :: () -> *physx_PxErrorCallback #foreign physx_sys;

physx_create_physics :: (foundation: *physx_PxFoundation) -> *physx_PxPhysics #foreign physx_sys;

create_raycast_filter_callback :: (actor_to_ignore: *physx_PxRigidActor) -> *physx_PxQueryFilterCallback #foreign physx_sys;

create_raycast_filter_callback_func :: (callback: RaycastHitCallback, userData: *void) -> *physx_PxQueryFilterCallback #foreign physx_sys;

create_pre_and_post_raycast_filter_callback_func :: (preFilter: RaycastHitCallback, postFilter: PostFilterCallback, userData: *void) -> *physx_PxQueryFilterCallback #foreign physx_sys;

create_raycast_buffer :: () -> *physx_PxRaycastCallback #foreign physx_sys;

create_sweep_buffer :: () -> *physx_PxSweepCallback #foreign physx_sys;

create_overlap_buffer :: () -> *physx_PxOverlapCallback #foreign physx_sys;

create_raycast_callback :: (process_touches_callback: RaycastHitProcessTouchesCallback, finalize_query_callback: HitFinalizeQueryCallback, touchesBuffer: *physx_PxRaycastHit, numTouches: u32, userdata: *void) -> *physx_PxRaycastCallback #foreign physx_sys;

delete_raycast_callback :: (callback: *physx_PxRaycastCallback) -> void #foreign physx_sys;

delete_sweep_callback :: (callback: *physx_PxSweepCallback) -> void #foreign physx_sys;

delete_overlap_callback :: (callback: *physx_PxOverlapCallback) -> void #foreign physx_sys;

create_sweep_callback :: (process_touches_callback: SweepHitProcessTouchesCallback, finalize_query_callback: HitFinalizeQueryCallback, touchesBuffer: *physx_PxSweepHit, numTouches: u32, userdata: *void) -> *physx_PxSweepCallback #foreign physx_sys;

create_overlap_callback :: (process_touches_callback: OverlapHitProcessTouchesCallback, finalize_query_callback: HitFinalizeQueryCallback, touchesBuffer: *physx_PxOverlapHit, numTouches: u32, userdata: *void) -> *physx_PxOverlapCallback #foreign physx_sys;

create_alloc_callback :: (alloc_callback: AllocCallback, dealloc_callback: DeallocCallback, userdata: *void) -> *physx_PxAllocatorCallback #foreign physx_sys;

get_alloc_callback_user_data :: (allocator: *physx_PxAllocatorCallback) -> *void #foreign physx_sys;

create_profiler_callback :: (zone_start_callback: ZoneStartCallback, zone_end_callback: ZoneEndCallback, userdata: *void) -> *physx_PxProfilerCallback #foreign physx_sys;

create_error_callback :: (error_callback: ErrorCallback, userdata: *void) -> *physx_PxErrorCallback #foreign physx_sys;

create_assert_handler :: (on_assert: AssertHandler, userdata: *void) -> *physx_PxAssertHandler #foreign physx_sys;

get_default_simulation_filter_shader :: () -> *void #foreign physx_sys;

create_simulation_event_callbacks :: (callbacks: *SimulationEventCallbackInfo) -> *physx_PxSimulationEventCallback #foreign physx_sys;

get_simulation_event_info :: (callback: *physx_PxSimulationEventCallback) -> *SimulationEventCallbackInfo #foreign physx_sys;

destroy_simulation_event_callbacks :: (callback: *physx_PxSimulationEventCallback) -> void #foreign physx_sys;

enable_custom_filter_shader :: (desc: *physx_PxSceneDesc, filter: SimulationShaderFilter, call_default_filter_shader_first: u32) -> void #foreign physx_sys;

// Not generated, used only for testing and examples!
PxAssertHandler_opCall :: (self: *physx_PxErrorCallback, expr: *u8, file: *u8, line: s32, ignore: *bool) -> void #foreign physx_sys "PxAssertHandler_opCall_mut";

#scope_file

physx_sys :: #library,no_dll "windows/physx_sys";
#import "Math";
