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
        NotifyContactPos32s :: 1 << 9;
        DetectDiscreteContact :: 1 << 10;
        DetectCcdContact :: 1 << 11;
        PreSolverVelocity :: 1 << 12;
        PostSolverVelocity :: 1 << 13;
        ContactEventPose :: 1 << 14;
        NextFree :: 1 << 15;
        ContactDefault :: SolveContact | DetectDiscreteContact;
        TriggerDefault :: NotifyTouchFound | NotifyTouchLost | DetectDiscreteContact;
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

physx_Vector3 :: struct {
    x: float;
    y: float;
    z: float;
}

physx_Vector3Padded :: struct {
    x:       float;
    y:       float;
    z:       float;
    padding: u32;
}

physx_PxQuat :: struct {
    x: float;
    y: float;
    z: float;
    w: float;
}

physx_PxTransform :: struct {
    q: physx_PxQuat;
    p: physx_Vector3;
}

physx_PxTransformPadded :: struct {
    transform: physx_PxTransform;
    padding:   u32;
}

physx_Matrix3 :: struct {
    column0: physx_Vector3;
    column1: physx_Vector3;
    column2: physx_Vector3;
}

physx_PxBounds3 :: struct {
    minimum: physx_Vector3;
    maximum: physx_Vector3;
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

physx_Vector4 :: struct {
    x: float;
    y: float;
    z: float;
    w: float;
}

physx_Matrix4 :: struct {
    column0: physx_Vector4;
    column1: physx_Vector4;
    column2: physx_Vector4;
    column3: physx_Vector4;
}

physx_PxPlane :: struct {
    n: physx_Vector3;
    d: float;
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

physx_PxVec2 :: struct {
    x: float;
    y: float;
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

physx_PxDebugPos32 :: struct {
    pos:   physx_Vector3;
    color: u32;
}

physx_PxDebugLine :: struct {
    pos0:   physx_Vector3;
    color0: u32;
    pos1:   physx_Vector3;
    color1: u32;
}

physx_PxDebugTriangle :: struct {
    pos0:   physx_Vector3;
    color0: u32;
    pos1:   physx_Vector3;
    color1: u32;
    pos2:   physx_Vector3;
    color2: u32;
}

physx_PxDebugText :: struct {
    position:       physx_Vector3;
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
    halfExtents:    physx_Vector3;
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
    scale:    physx_Vector3;
    rotation: physx_PxQuat;
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
    position:       physx_Vector3;
    normal:         physx_Vector3;
    distance:       float;
}

physx_PxGeomRaycastHit :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       physx_Vector3;
    normal:         physx_Vector3;
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
    position:       physx_Vector3;
    normal:         physx_Vector3;
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
    pos32s:         physx_PxBoundedData;
    triangles:      physx_PxBoundedData;
    flags:          u16;
    structgen_pad0: [6] u8;
}

physx_PxTriangle :: struct {
    verts: [3] physx_Vector3;
}

physx_PxTrianglePadded :: struct {
    verts:   [3] physx_Vector3;
    padding: u32;
}

physx_PxTriangleMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxBVH34TriangleMesh :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTetrahedron :: struct {
    verts: [4] physx_Vector3;
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

physx_Px1DConstras32Mods :: union {
    spring: physx_PxSpringModifiers;
    bounce: physx_PxRestitutionModifiers;
}

physx_Px1DConstras32 :: struct {
    linear0:        physx_Vector3;
    geometricError: float;
    angular0:       physx_Vector3;
    velocityTarget: float;
    linear1:        physx_Vector3;
    minImpulse:     float;
    angular1:       physx_Vector3;
    maxImpulse:     float;
    mods:           physx_Px1DConstras32Mods;
    forInternalUse: float;
    flags:          u16;
    solveHs32:      u16;
    structgen_pad0: [8] u8;
}

physx_PxConstras32InvMassScale :: struct {
    linear0:  float;
    angular0: float;
    linear1:  float;
    angular1: float;
}

physx_PxConstras32Visualizer :: struct {
    vtable_: *void;
}

physx_PxConstras32Connector :: struct {
    vtable_: *void;
}

physx_PxContactPos32 :: struct {
    normal:             physx_Vector3;
    separation:         float;
    pos32:              physx_Vector3;
    maxImpulse:         float;
    targetVel:          physx_Vector3;
    staticFriction:     float;
    materialFlags:      u8;
    structgen_pad0:     [3] u8;
    s32ernalFaceIndex1: u32;
    dynamicFriction:    float;
    restitution:        float;
    damping:            float;
    structgen_pad1:     [12] u8;
}

physx_PxSolverBody :: struct {
    linearVelocity:            physx_Vector3;
    maxSolverNormalProgress:   u16;
    maxSolverFrictionProgress: u16;
    angularState:              physx_Vector3;
    solverProgress:            u32;
}

physx_PxSolverBodyData :: struct {
    linearVelocity:    physx_Vector3;
    invMass:           float;
    angularVelocity:   physx_Vector3;
    reportThreshold:   float;
    sqrtInvInertia:    physx_Matrix3;
    penBiasClamp:      float;
    nodeIndex:         u32;
    maxContactImpulse: float;
    body2World:        physx_PxTransform;
    pad:               u16;
    structgen_pad0:    [2] u8;
}

physx_PxConstras32BatchHeader :: struct {
    startIndex:     u32;
    stride:         u16;
    constras32Type: u16;
}

physx_PxSolverConstras32Desc :: struct {
    structgen_pad0:         [16] u8;
    bodyADataIndex:         u32;
    bodyBDataIndex:         u32;
    linkIndexA:             u32;
    linkIndexB:             u32;
    constras32:             *u8;
    writeBack:              *void;
    progressA:              u16;
    progressB:              u16;
    constras32LengthOver16: u16;
    padding:                [10] u8;
}

physx_PxSolverConstras32PrepDescBase :: struct {
    invMassScales:  physx_PxConstras32InvMassScale;
    desc:           *physx_PxSolverConstras32Desc;
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

physx_PxSolverConstras32PrepDesc :: struct {
    invMassScales:        physx_PxConstras32InvMassScale;
    desc:                 *physx_PxSolverConstras32Desc;
    body0:                *physx_PxSolverBody;
    body1:                *physx_PxSolverBody;
    data0:                *physx_PxSolverBodyData;
    data1:                *physx_PxSolverBodyData;
    bodyFrame0:           physx_PxTransform;
    bodyFrame1:           physx_PxTransform;
    bodyState0:           s32;
    bodyState1:           s32;
    structgen_pad0:       [8] u8;
    rows:                 *physx_Px1DConstras32;
    numRows:              u32;
    linBreakForce:        float;
    angBreakForce:        float;
    minResponseThreshold: float;
    writeback:            *void;
    disablePreprocessing: bool;
    improvedSlerp:        bool;
    driveLimitsAreForces: bool;
    extendedLimits:       bool;
    disableConstras32:    bool;
    structgen_pad1:       [3] u8;
    body0WorldOffset:     physx_Vector3Padded;
    structgen_pad2:       [8] u8;
}

physx_PxSolverContactDesc :: struct {
    invMassScales:           physx_PxConstras32InvMassScale;
    desc:                    *physx_PxSolverConstras32Desc;
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
    contacts:                *physx_PxContactPos32;
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
    axisConstras32Count:     u16;
    offsetSlop:              float;
    structgen_pad3:          [4] u8;
}

physx_PxConstras32Allocator :: struct {
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
    linearVelocity:       physx_Vector3;
    nbStaticInteractions: u16;
    maxDynamicPartition:  u16;
    angularVelocity:      physx_Vector3;
    partitionMask:        u32;
    deltaAngDt:           physx_Vector3;
    maxAngVel:            float;
    deltaLinDt:           physx_Vector3;
    lockFlags:            u16;
    isKinematic:          bool;
    pad:                  u8;
}

physx_PxTGSSolverBodyTxInertia :: struct {
    deltaBody2World: physx_PxTransform;
    sqrtInvInertia:  physx_Matrix3;
}

physx_PxTGSSolverBodyData :: struct {
    originalLinearVelocity:  physx_Vector3;
    maxContactImpulse:       float;
    originalAngularVelocity: physx_Vector3;
    penBiasClamp:            float;
    invMass:                 float;
    nodeIndex:               u32;
    reportThreshold:         float;
    pad:                     u32;
}

physx_PxTGSSolverConstras32PrepDescBase :: struct {
    invMassScales:  physx_PxConstras32InvMassScale;
    desc:           *physx_PxSolverConstras32Desc;
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

physx_PxTGSSolverConstras32PrepDesc :: struct {
    invMassScales:        physx_PxConstras32InvMassScale;
    desc:                 *physx_PxSolverConstras32Desc;
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
    rows:                 *physx_Px1DConstras32;
    numRows:              u32;
    linBreakForce:        float;
    angBreakForce:        float;
    minResponseThreshold: float;
    writeback:            *void;
    disablePreprocessing: bool;
    improvedSlerp:        bool;
    driveLimitsAreForces: bool;
    extendedLimits:       bool;
    disableConstras32:    bool;
    structgen_pad1:       [3] u8;
    body0WorldOffset:     physx_Vector3Padded;
    cA2w:                 physx_Vector3Padded;
    cB2w:                 physx_Vector3Padded;
    structgen_pad2:       [8] u8;
}

physx_PxTGSSolverContactDesc :: struct {
    invMassScales:           physx_PxConstras32InvMassScale;
    desc:                    *physx_PxSolverConstras32Desc;
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
    contacts:                *physx_PxContactPos32;
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
    axisConstras32Count:     u16;
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

physx_PxArticulationTendonJos32 :: struct {
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
    force:  physx_Vector3;
    pad0:   float;
    torque: physx_Vector3;
    pad1:   float;
}

physx_PxSpatialVelocity :: struct {
    linear:  physx_Vector3;
    pad0:    float;
    angular: physx_Vector3;
    pad1:    float;
}

physx_PxArticulationRootLinkData :: struct {
    transform:     physx_PxTransform;
    worldLinVel:   physx_Vector3;
    worldAngVel:   physx_Vector3;
    worldLinAccel: physx_Vector3;
    worldAngAccel: physx_Vector3;
}

physx_PxArticulationCache :: struct {
    externalForces:    *physx_PxSpatialForce;
    denseJacobian:     *float;
    massMatrix:        *float;
    jos32Velocity:     *float;
    jos32Acceleration: *float;
    jos32Position:     *float;
    jos32Force:        *float;
    jos32SolverForces: *float;
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

physx_PxArticulationJos32ReducedCoordinate :: struct {
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

physx_PxConeLimitedConstras32 :: struct {
    mAxis:      physx_Vector3;
    mAngle:     float;
    mLowLimit:  float;
    mHighLimit: float;
}

physx_PxConeLimitParams :: struct {
    lowHighLimits: physx_Vector4;
    axisAngle:     physx_Vector4;
}

physx_PxConstras32ShaderTable :: struct {
    solverPrep:     *void;
    structgen_pad0: [8] u8;
    visualize:      *void;
    flag:           s32;
    structgen_pad1: [4] u8;
}

physx_PxConstras32 :: struct {
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
    normal:            physx_Vector3;
    restitution:       float;
    dynamicFriction:   float;
    staticFriction:    float;
    damping:           float;
    startContactIndex: u16;
    nbContacts:        u8;
    materialFlags:     u8;
    s32ernalFlags:     u16;
    materialIndex0:    u16;
    materialIndex1:    u16;
    pad:               [5] u16;
}

physx_PxContact :: struct {
    contact:    physx_Vector3;
    separation: float;
}

physx_PxExtendedContact :: struct {
    contact:        physx_Vector3;
    separation:     float;
    targetVelocity: physx_Vector3;
    maxImpulse:     float;
}

physx_PxModifiableContact :: struct {
    contact:         physx_Vector3;
    separation:      float;
    targetVelocity:  physx_Vector3;
    maxImpulse:      float;
    normal:          physx_Vector3;
    restitution:     float;
    materialFlags:   u32;
    materialIndex0:  u16;
    materialIndex1:  u16;
    staticFriction:  float;
    dynamicFriction: float;
}

physx_PxContactStreamIterator :: struct {
    zero:                   physx_Vector3;
    structgen_pad0:         [4] u8;
    patch:                  *physx_PxContactPatch;
    contact:                *physx_PxContact;
    faceIndice:             *u32;
    totalPatches:           u32;
    totalContacts:          u32;
    nextContactIndex:       u32;
    nextPatchIndex:         u32;
    contactPatchHeaderSize: u32;
    contactPos32Size:       u32;
    mStreamFormat:          s32;
    forceNoResponse:        u32;
    pos32Stepped:           bool;
    structgen_pad1:         [3] u8;
    hasFaceIndices:         u32;
}

physx_PxGpuContactPair :: struct {
    contactPatches:     *u8;
    contactPos32s:      *u8;
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
    position:       physx_Vector3;
    normal:         physx_Vector3;
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
    position:       physx_Vector3;
    normal:         physx_Vector3;
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
    dynamicTreeRebuildRateHs32: u32;
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
    maxNbConstras32s:        u32;
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
    dynamicTreeRebuildRateHs32:    u32;
    dynamicTreeSecondaryPruner:    s32;
    staticBVHBuildStrategy:        s32;
    dynamicBVHBuildStrategy:       s32;
    staticNbObjectsPerNode:        u32;
    dynamicNbObjectsPerNode:       u32;
    sceneQueryUpdateMode:          s32;
    gravity:                       physx_Vector3;
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
    nbActiveConstras32s:                 u32;
    nbActiveDynamicBodies:               u32;
    nbActiveKinematicBodies:             u32;
    nbStaticBodies:                      u32;
    nbDynamicBodies:                     u32;
    nbKinematicBodies:                   u32;
    nbShapes:                            [11] u32;
    nbAggregates:                        u32;
    nbArticulations:                     u32;
    nbAxisSolverConstras32s:             u32;
    compressedContactSize:               u32;
    requiredContactConstras32Memory:     u32;
    peakConstras32Memory:                u32;
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
    quat:   physx_PxQuat;
    pos:    physx_Vector4;
    linVel: physx_Vector4;
    angVel: physx_Vector4;
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
    linearVelocity:  [2] physx_Vector3;
    angularVelocity: [2] physx_Vector3;
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

physx_PxContactPairPos32 :: struct {
    position:           physx_Vector3;
    separation:         float;
    normal:             physx_Vector3;
    s32ernalFaceIndex0: u32;
    impulse:            physx_Vector3;
    s32ernalFaceIndex1: u32;
}

physx_PxContactPair :: struct {
    shapes:             [2] *physx_PxShape;
    contactPatches:     *u8;
    contactPos32s:      *u8;
    contactImpulses:    *float;
    requiredBufferSize: u32;
    contactCount:       u8;
    patchCount:         u8;
    contactStreamSize:  u16;
    flags:              u16;
    events:             u16;
    s32ernalData:       [2] u32;
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

physx_PxConstras32Info :: struct {
    constras32:        *physx_PxConstras32;
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
    mRot:           physx_PxQuat;
}

physx_PxBoxObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3;
    mRot:           physx_PxQuat;
    mHalfExtents:   physx_Vector3;
    structgen_pad1: [4] u8;
}

physx_PxCapsuleObstacle :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3;
    mRot:           physx_PxQuat;
    mHalfHeight:    float;
    mRadius:        float;
}

physx_PxObstacleContext :: struct {
    vtable_: *void;
}

physx_PxControllerState :: struct {
    deltaXP:               physx_Vector3;
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
    worldNormal:    physx_Vector3;
    dir:            physx_Vector3;
    length:         float;
    structgen_pad0: [4] u8;
}

physx_PxControllerShapeHit :: struct {
    controller:     *physx_PxController;
    worldPos:       physx_PxExtendedVec3;
    worldNormal:    physx_Vector3;
    dir:            physx_Vector3;
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
    worldNormal:    physx_Vector3;
    dir:            physx_Vector3;
    length:         float;
    structgen_pad0: [4] u8;
    other:          *physx_PxController;
}

physx_PxControllerObstacleHit :: struct {
    controller:     *physx_PxController;
    worldPos:       physx_PxExtendedVec3;
    worldNormal:    physx_Vector3;
    dir:            physx_Vector3;
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
    upDirection:              physx_Vector3;
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
    upDirection:              physx_Vector3;
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
    upDirection:              physx_Vector3;
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
    meshLower:                                      physx_Vector3;
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
    pos32s:         physx_PxBoundedData;
    polygons:       physx_PxBoundedData;
    indices:        physx_PxBoundedData;
    flags:          u16;
    vertexLimit:    u16;
    polygonLimit:   u16;
    quantizedCount: u16;
    sdfDesc:        *physx_PxSDFDesc;
}

physx_PxTriangleMeshDesc :: struct {
    pos32s:         physx_PxBoundedData;
    triangles:      physx_PxBoundedData;
    flags:          u16;
    structgen_pad0: [22] u8;
    sdfDesc:        *physx_PxSDFDesc;
}

physx_PxTetrahedronMeshDesc :: struct {
    structgen_pad0: [16] u8;
    pos32s:         physx_PxBoundedData;
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

physx_PxJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpring :: struct {
    stiffness: float;
    damping:   float;
}

physx_PxDistanceJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxJacobianRow :: struct {
    linear0:  physx_Vector3;
    linear1:  physx_Vector3;
    angular0: physx_Vector3;
    angular1: physx_Vector3;
}

physx_PxContactJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFixedJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxJos32LimitParameters :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
}

physx_PxJos32LinearLimit :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    value:                      float;
}

physx_PxJos32LinearLimitPair :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

physx_PxJos32AngularLimitPair :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

physx_PxJos32LimitCone :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    yAngle:                     float;
    zAngle:                     float;
}

physx_PxJos32LimitPyramid :: struct {
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

physx_PxPrismaticJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRevoluteJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSphericalJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxD6Jos32Drive :: struct {
    stiffness:  float;
    damping:    float;
    forceLimit: float;
    flags:      u32;
}

physx_PxD6Jos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGearJos32 :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRackAndPinionJos32 :: struct {
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
    inertiaTensor: physx_Matrix3;
    centerOfMass:  physx_Vector3;
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
    upVector: physx_Vector3;
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

PxAllocatorCallback_allocate_mut :: (self: *physx_PxAllocatorCallback, size: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign physx_sys;

PxAllocatorCallback_deallocate_mut :: (self: *physx_PxAllocatorCallback, ptr: *void) -> void #foreign physx_sys;

PxAssertHandler_delete :: (self: *physx_PxAssertHandler) -> void #foreign physx_sys;

phys_PxGetAssertHandler :: () -> *physx_PxAssertHandler #foreign physx_sys;

phys_PxSetAssertHandler :: (handler: *physx_PxAssertHandler) -> void #foreign physx_sys;

PxFoundation_release_mut :: (self: *physx_PxFoundation) -> void #foreign physx_sys;

PxFoundation_getErrorCallback_mut :: (self: *physx_PxFoundation) -> *physx_PxErrorCallback #foreign physx_sys;

PxFoundation_setErrorLevel_mut :: (self: *physx_PxFoundation, mask: u32) -> void #foreign physx_sys;

PxFoundation_getErrorLevel :: (self: *physx_PxFoundation) -> u32 #foreign physx_sys;

PxFoundation_getAllocatorCallback_mut :: (self: *physx_PxFoundation) -> *physx_PxAllocatorCallback #foreign physx_sys;

PxFoundation_getReportAllocationNames :: (self: *physx_PxFoundation) -> bool #foreign physx_sys;

PxFoundation_setReportAllocationNames_mut :: (self: *physx_PxFoundation, value: bool) -> void #foreign physx_sys;

PxFoundation_registerAllocationListener_mut :: (self: *physx_PxFoundation, listener: *physx_PxAllocationListener) -> void #foreign physx_sys;

PxFoundation_deregisterAllocationListener_mut :: (self: *physx_PxFoundation, listener: *physx_PxAllocationListener) -> void #foreign physx_sys;

PxFoundation_registerErrorCallback_mut :: (self: *physx_PxFoundation, callback: *physx_PxErrorCallback) -> void #foreign physx_sys;

PxFoundation_deregisterErrorCallback_mut :: (self: *physx_PxFoundation, callback: *physx_PxErrorCallback) -> void #foreign physx_sys;

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

PxAllocator_allocate_mut :: (self: *physx_PxAllocator, size: u64, file: *u8, line: s32) -> *void #foreign physx_sys;

PxAllocator_deallocate_mut :: (self: *physx_PxAllocator, ptr: *void) -> void #foreign physx_sys;

PxRawAllocator_new :: (anon_param0: *u8) -> physx_PxRawAllocator #foreign physx_sys;

PxRawAllocator_allocate_mut :: (self: *physx_PxRawAllocator, size: u64, anon_param1: *u8, anon_param2: s32) -> *void #foreign physx_sys;

PxRawAllocator_deallocate_mut :: (self: *physx_PxRawAllocator, ptr: *void) -> void #foreign physx_sys;

PxVirtualAllocatorCallback_delete :: (self: *physx_PxVirtualAllocatorCallback) -> void #foreign physx_sys;

PxVirtualAllocatorCallback_allocate_mut :: (self: *physx_PxVirtualAllocatorCallback, size: u64, group: s32, file: *u8, line: s32) -> *void #foreign physx_sys;

PxVirtualAllocatorCallback_deallocate_mut :: (self: *physx_PxVirtualAllocatorCallback, ptr: *void) -> void #foreign physx_sys;

PxVirtualAllocator_new :: (callback: *physx_PxVirtualAllocatorCallback, group: s32) -> physx_PxVirtualAllocator #foreign physx_sys;

PxVirtualAllocator_allocate_mut :: (self: *physx_PxVirtualAllocator, size: u64, file: *u8, line: s32) -> *void #foreign physx_sys;

PxVirtualAllocator_deallocate_mut :: (self: *physx_PxVirtualAllocator, ptr: *void) -> void #foreign physx_sys;

PxTempAllocatorChunk_new :: () -> physx_PxTempAllocatorChunk #foreign physx_sys;

PxTempAllocator_new :: (anon_param0: *u8) -> physx_PxTempAllocator #foreign physx_sys;

PxTempAllocator_allocate_mut :: (self: *physx_PxTempAllocator, size: u64, file: *u8, line: s32) -> *void #foreign physx_sys;

PxTempAllocator_deallocate_mut :: (self: *physx_PxTempAllocator, ptr: *void) -> void #foreign physx_sys;

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

Vector3_new :: () -> physx_Vector3 #foreign physx_sys;

Vector3_new_1 :: (anon_param0: s32) -> physx_Vector3 #foreign physx_sys;

Vector3_new_2 :: (a: float) -> physx_Vector3 #foreign physx_sys;

Vector3_new_3 :: (nx: float, ny: float, nz: float) -> physx_Vector3 #foreign physx_sys;

Vector3_isZero :: (self: *physx_Vector3) -> bool #foreign physx_sys;

Vector3_isFinite :: (self: *physx_Vector3) -> bool #foreign physx_sys;

Vector3_isNormalized :: (self: *physx_Vector3) -> bool #foreign physx_sys;

Vector3_magnitudeSquared :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_magnitude :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_dot :: (self: *physx_Vector3, v: *physx_Vector3) -> float #foreign physx_sys;

Vector3_cross :: (self: *physx_Vector3, v: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Vector3_getNormalized :: (self: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Vector3_normalize_mut :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_normalizeSafe_mut :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_normalizeFast_mut :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_multiply :: (self: *physx_Vector3, a: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Vector3_minimum :: (self: *physx_Vector3, v: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Vector3_minElement :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_maximum :: (self: *physx_Vector3, v: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Vector3_maxElement :: (self: *physx_Vector3) -> float #foreign physx_sys;

Vector3_abs :: (self: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Vector3Padded_new_alloc :: () -> *physx_Vector3Padded #foreign physx_sys;

Vector3Padded_delete :: (self: *physx_Vector3Padded) -> void #foreign physx_sys;

Vector3Padded_new_alloc_1 :: (p: *physx_Vector3) -> *physx_Vector3Padded #foreign physx_sys;

Vector3Padded_new_alloc_2 :: (f: float) -> *physx_Vector3Padded #foreign physx_sys;

PxQuat_new :: () -> physx_PxQuat #foreign physx_sys;

PxQuat_new_1 :: (anon_param0: s32) -> physx_PxQuat #foreign physx_sys;

PxQuat_new_2 :: (r: float) -> physx_PxQuat #foreign physx_sys;

PxQuat_new_3 :: (nx: float, ny: float, nz: float, nw: float) -> physx_PxQuat #foreign physx_sys;

PxQuat_new_4 :: (angleRadians: float, unitAxis: *physx_Vector3) -> physx_PxQuat #foreign physx_sys;

PxQuat_new_5 :: (m: *physx_Matrix3) -> physx_PxQuat #foreign physx_sys;

PxQuat_isIdentity :: (self: *physx_PxQuat) -> bool #foreign physx_sys;

PxQuat_isFinite :: (self: *physx_PxQuat) -> bool #foreign physx_sys;

PxQuat_isUnit :: (self: *physx_PxQuat) -> bool #foreign physx_sys;

PxQuat_isSane :: (self: *physx_PxQuat) -> bool #foreign physx_sys;

PxQuat_toRadiansAndUnitAxis :: (self: *physx_PxQuat, angle: *float, axis: *physx_Vector3) -> void #foreign physx_sys;

PxQuat_getAngle :: (self: *physx_PxQuat) -> float #foreign physx_sys;

PxQuat_getAngle_1 :: (self: *physx_PxQuat, q: *physx_PxQuat) -> float #foreign physx_sys;

PxQuat_magnitudeSquared :: (self: *physx_PxQuat) -> float #foreign physx_sys;

PxQuat_dot :: (self: *physx_PxQuat, v: *physx_PxQuat) -> float #foreign physx_sys;

PxQuat_getNormalized :: (self: *physx_PxQuat) -> physx_PxQuat #foreign physx_sys;

PxQuat_magnitude :: (self: *physx_PxQuat) -> float #foreign physx_sys;

PxQuat_normalize_mut :: (self: *physx_PxQuat) -> float #foreign physx_sys;

PxQuat_getConjugate :: (self: *physx_PxQuat) -> physx_PxQuat #foreign physx_sys;

PxQuat_getImaginaryPart :: (self: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

PxQuat_getBasisVector0 :: (self: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

PxQuat_getBasisVector1 :: (self: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

PxQuat_getBasisVector2 :: (self: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

PxQuat_rotate :: (self: *physx_PxQuat, v: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxQuat_rotateInv :: (self: *physx_PxQuat, v: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxTransform_new :: () -> physx_PxTransform #foreign physx_sys;

PxTransform_new_1 :: (position: *physx_Vector3) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_2 :: (anon_param0: s32) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_3 :: (orientation: *physx_PxQuat) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_4 :: (x: float, y: float, z: float, aQ: physx_PxQuat) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_5 :: (p0: *physx_Vector3, q0: *physx_PxQuat) -> physx_PxTransform #foreign physx_sys;

PxTransform_new_6 :: (m: *physx_Matrix4) -> physx_PxTransform #foreign physx_sys;

PxTransform_getInverse :: (self: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxTransform_transform :: (self: *physx_PxTransform, input: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxTransform_transformInv :: (self: *physx_PxTransform, input: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxTransform_rotate :: (self: *physx_PxTransform, input: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxTransform_rotateInv :: (self: *physx_PxTransform, input: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxTransform_transform_1 :: (self: *physx_PxTransform, src: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxTransform_isValid :: (self: *physx_PxTransform) -> bool #foreign physx_sys;

PxTransform_isSane :: (self: *physx_PxTransform) -> bool #foreign physx_sys;

PxTransform_isFinite :: (self: *physx_PxTransform) -> bool #foreign physx_sys;

PxTransform_transformInv_1 :: (self: *physx_PxTransform, src: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

PxTransform_getNormalized :: (self: *physx_PxTransform) -> physx_PxTransform #foreign physx_sys;

Matrix3_new :: () -> physx_Matrix3 #foreign physx_sys;

Matrix3_new_1 :: (anon_param0: s32) -> physx_Matrix3 #foreign physx_sys;

Matrix3_new_2 :: (anon_param0: s32) -> physx_Matrix3 #foreign physx_sys;

Matrix3_new_3 :: (col0: *physx_Vector3, col1: *physx_Vector3, col2: *physx_Vector3) -> physx_Matrix3 #foreign physx_sys;

Matrix3_new_4 :: (r: float) -> physx_Matrix3 #foreign physx_sys;

Matrix3_new_5 :: (values: *float) -> physx_Matrix3 #foreign physx_sys;

Matrix3_new_6 :: (q: *physx_PxQuat) -> physx_Matrix3 #foreign physx_sys;

Matrix3_createDiagonal :: (d: *physx_Vector3) -> physx_Matrix3 #foreign physx_sys;

Matrix3_outer :: (a: *physx_Vector3, b: *physx_Vector3) -> physx_Matrix3 #foreign physx_sys;

Matrix3_getTranspose :: (self: *physx_Matrix3) -> physx_Matrix3 #foreign physx_sys;

Matrix3_getInverse :: (self: *physx_Matrix3) -> physx_Matrix3 #foreign physx_sys;

Matrix3_getDeterminant :: (self: *physx_Matrix3) -> float #foreign physx_sys;

Matrix3_transform :: (self: *physx_Matrix3, other: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Matrix3_transformTranspose :: (self: *physx_Matrix3, other: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Matrix3_front :: (self: *physx_Matrix3) -> *float #foreign physx_sys;

PxBounds3_new :: () -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_new_1 :: (minimum: *physx_Vector3, maximum: *physx_Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_empty :: () -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_boundsOfPos32s :: (v0: *physx_Vector3, v1: *physx_Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_centerExtents :: (center: *physx_Vector3, extent: *physx_Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_basisExtent :: (center: *physx_Vector3, basis: *physx_Matrix3, extent: *physx_Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_poseExtent :: (pose: *physx_PxTransform, extent: *physx_Vector3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformSafe :: (matrix: *physx_Matrix3, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformFast :: (matrix: *physx_Matrix3, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformSafe_1 :: (transform: *physx_PxTransform, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_transformFast_1 :: (transform: *physx_PxTransform, bounds: *physx_PxBounds3) -> physx_PxBounds3 #foreign physx_sys;

PxBounds3_setEmpty_mut :: (self: *physx_PxBounds3) -> void #foreign physx_sys;

PxBounds3_setMaximal_mut :: (self: *physx_PxBounds3) -> void #foreign physx_sys;

PxBounds3_include_mut :: (self: *physx_PxBounds3, v: *physx_Vector3) -> void #foreign physx_sys;

PxBounds3_include_mut_1 :: (self: *physx_PxBounds3, b: *physx_PxBounds3) -> void #foreign physx_sys;

PxBounds3_isEmpty :: (self: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_s32ersects :: (self: *physx_PxBounds3, b: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_s32ersects1D :: (self: *physx_PxBounds3, a: *physx_PxBounds3, axis: u32) -> bool #foreign physx_sys;

PxBounds3_contains :: (self: *physx_PxBounds3, v: *physx_Vector3) -> bool #foreign physx_sys;

PxBounds3_isInside :: (self: *physx_PxBounds3, box: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_getCenter :: (self: *physx_PxBounds3) -> physx_Vector3 #foreign physx_sys;

PxBounds3_getCenter_1 :: (self: *physx_PxBounds3, axis: u32) -> float #foreign physx_sys;

PxBounds3_getExtents :: (self: *physx_PxBounds3, axis: u32) -> float #foreign physx_sys;

PxBounds3_getDimensions :: (self: *physx_PxBounds3) -> physx_Vector3 #foreign physx_sys;

PxBounds3_getExtents_1 :: (self: *physx_PxBounds3) -> physx_Vector3 #foreign physx_sys;

PxBounds3_scaleSafe_mut :: (self: *physx_PxBounds3, scale: float) -> void #foreign physx_sys;

PxBounds3_scaleFast_mut :: (self: *physx_PxBounds3, scale: float) -> void #foreign physx_sys;

PxBounds3_fattenSafe_mut :: (self: *physx_PxBounds3, distance: float) -> void #foreign physx_sys;

PxBounds3_fattenFast_mut :: (self: *physx_PxBounds3, distance: float) -> void #foreign physx_sys;

PxBounds3_isFinite :: (self: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_isValid :: (self: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBounds3_closestPos32 :: (self: *physx_PxBounds3, p: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxErrorCallback_delete :: (self: *physx_PxErrorCallback) -> void #foreign physx_sys;

PxErrorCallback_reportError_mut :: (self: *physx_PxErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys;

PxAllocationListener_onAllocation_mut :: (self: *physx_PxAllocationListener, size: u64, typeName: *u8, filename: *u8, line: s32, allocatedMemory: *void) -> void #foreign physx_sys;

PxAllocationListener_onDeallocation_mut :: (self: *physx_PxAllocationListener, allocatedMemory: *void) -> void #foreign physx_sys;

PxBroadcastingAllocator_new_alloc :: (allocator: *physx_PxAllocatorCallback, error: *physx_PxErrorCallback) -> *physx_PxBroadcastingAllocator #foreign physx_sys;

PxBroadcastingAllocator_delete :: (self: *physx_PxBroadcastingAllocator) -> void #foreign physx_sys;

PxBroadcastingAllocator_allocate_mut :: (self: *physx_PxBroadcastingAllocator, size: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign physx_sys;

PxBroadcastingAllocator_deallocate_mut :: (self: *physx_PxBroadcastingAllocator, ptr: *void) -> void #foreign physx_sys;

PxBroadcastingErrorCallback_new_alloc :: (errorCallback: *physx_PxErrorCallback) -> *physx_PxBroadcastingErrorCallback #foreign physx_sys;

PxBroadcastingErrorCallback_delete :: (self: *physx_PxBroadcastingErrorCallback) -> void #foreign physx_sys;

PxBroadcastingErrorCallback_reportError_mut :: (self: *physx_PxBroadcastingErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys;

phys_PxEnableFPExceptions :: () -> void #foreign physx_sys;

phys_PxDisableFPExceptions :: () -> void #foreign physx_sys;

PxInputStream_read_mut :: (self: *physx_PxInputStream, dest: *void, count: u32) -> u32 #foreign physx_sys;

PxInputStream_delete :: (self: *physx_PxInputStream) -> void #foreign physx_sys;

PxInputData_getLength :: (self: *physx_PxInputData) -> u32 #foreign physx_sys;

PxInputData_seek_mut :: (self: *physx_PxInputData, offset: u32) -> void #foreign physx_sys;

PxInputData_tell :: (self: *physx_PxInputData) -> u32 #foreign physx_sys;

PxInputData_delete :: (self: *physx_PxInputData) -> void #foreign physx_sys;

PxOutputStream_write_mut :: (self: *physx_PxOutputStream, src: *void, count: u32) -> u32 #foreign physx_sys;

PxOutputStream_delete :: (self: *physx_PxOutputStream) -> void #foreign physx_sys;

Vector4_new :: () -> physx_Vector4 #foreign physx_sys;

Vector4_new_1 :: (anon_param0: s32) -> physx_Vector4 #foreign physx_sys;

Vector4_new_2 :: (a: float) -> physx_Vector4 #foreign physx_sys;

Vector4_new_3 :: (nx: float, ny: float, nz: float, nw: float) -> physx_Vector4 #foreign physx_sys;

Vector4_new_4 :: (v: *physx_Vector3, nw: float) -> physx_Vector4 #foreign physx_sys;

Vector4_new_5 :: (v: *float) -> physx_Vector4 #foreign physx_sys;

Vector4_isZero :: (self: *physx_Vector4) -> bool #foreign physx_sys;

Vector4_isFinite :: (self: *physx_Vector4) -> bool #foreign physx_sys;

Vector4_isNormalized :: (self: *physx_Vector4) -> bool #foreign physx_sys;

Vector4_magnitudeSquared :: (self: *physx_Vector4) -> float #foreign physx_sys;

Vector4_magnitude :: (self: *physx_Vector4) -> float #foreign physx_sys;

Vector4_dot :: (self: *physx_Vector4, v: *physx_Vector4) -> float #foreign physx_sys;

Vector4_getNormalized :: (self: *physx_Vector4) -> physx_Vector4 #foreign physx_sys;

Vector4_normalize_mut :: (self: *physx_Vector4) -> float #foreign physx_sys;

Vector4_multiply :: (self: *physx_Vector4, a: *physx_Vector4) -> physx_Vector4 #foreign physx_sys;

Vector4_minimum :: (self: *physx_Vector4, v: *physx_Vector4) -> physx_Vector4 #foreign physx_sys;

Vector4_maximum :: (self: *physx_Vector4, v: *physx_Vector4) -> physx_Vector4 #foreign physx_sys;

Vector4_getXYZ :: (self: *physx_Vector4) -> physx_Vector3 #foreign physx_sys;

Matrix4_new :: () -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_1 :: (anon_param0: s32) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_2 :: (anon_param0: s32) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_3 :: (col0: *physx_Vector4, col1: *physx_Vector4, col2: *physx_Vector4, col3: *physx_Vector4) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_4 :: (r: float) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_5 :: (col0: *physx_Vector3, col1: *physx_Vector3, col2: *physx_Vector3, col3: *physx_Vector3) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_6 :: (values: *float) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_7 :: (q: *physx_PxQuat) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_8 :: (diagonal: *physx_Vector4) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_9 :: (axes: *physx_Matrix3, position: *physx_Vector3) -> physx_Matrix4 #foreign physx_sys;

Matrix4_new_10 :: (t: *physx_PxTransform) -> physx_Matrix4 #foreign physx_sys;

Matrix4_getTranspose :: (self: *physx_Matrix4) -> physx_Matrix4 #foreign physx_sys;

Matrix4_transform :: (self: *physx_Matrix4, other: *physx_Vector4) -> physx_Vector4 #foreign physx_sys;

Matrix4_transform_1 :: (self: *physx_Matrix4, other: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Matrix4_rotate :: (self: *physx_Matrix4, other: *physx_Vector4) -> physx_Vector4 #foreign physx_sys;

Matrix4_rotate_1 :: (self: *physx_Matrix4, other: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

Matrix4_getBasis :: (self: *physx_Matrix4, num: u32) -> physx_Vector3 #foreign physx_sys;

Matrix4_getPosition :: (self: *physx_Matrix4) -> physx_Vector3 #foreign physx_sys;

Matrix4_setPosition_mut :: (self: *physx_Matrix4, position: *physx_Vector3) -> void #foreign physx_sys;

Matrix4_front :: (self: *physx_Matrix4) -> *float #foreign physx_sys;

Matrix4_scale_mut :: (self: *physx_Matrix4, p: *physx_Vector4) -> void #foreign physx_sys;

Matrix4_inverseRT :: (self: *physx_Matrix4) -> physx_Matrix4 #foreign physx_sys;

Matrix4_isFinite :: (self: *physx_Matrix4) -> bool #foreign physx_sys;

PxPlane_new :: () -> physx_PxPlane #foreign physx_sys;

PxPlane_new_1 :: (nx: float, ny: float, nz: float, distance: float) -> physx_PxPlane #foreign physx_sys;

PxPlane_new_2 :: (normal: *physx_Vector3, distance: float) -> physx_PxPlane #foreign physx_sys;

PxPlane_new_3 :: (pos32: *physx_Vector3, normal: *physx_Vector3) -> physx_PxPlane #foreign physx_sys;

PxPlane_new_4 :: (p0: *physx_Vector3, p1: *physx_Vector3, p2: *physx_Vector3) -> physx_PxPlane #foreign physx_sys;

PxPlane_distance :: (self: *physx_PxPlane, p: *physx_Vector3) -> float #foreign physx_sys;

PxPlane_contains :: (self: *physx_PxPlane, p: *physx_Vector3) -> bool #foreign physx_sys;

PxPlane_project :: (self: *physx_PxPlane, p: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxPlane_pos32InPlane :: (self: *physx_PxPlane) -> physx_Vector3 #foreign physx_sys;

PxPlane_normalize_mut :: (self: *physx_PxPlane) -> void #foreign physx_sys;

PxPlane_transform :: (self: *physx_PxPlane, pose: *physx_PxTransform) -> physx_PxPlane #foreign physx_sys;

PxPlane_inverseTransform :: (self: *physx_PxPlane, pose: *physx_PxTransform) -> physx_PxPlane #foreign physx_sys;

phys_PxShortestRotation :: (from: *physx_Vector3, target: *physx_Vector3) -> physx_PxQuat #foreign physx_sys;

phys_PxDiagonalize :: (m: *physx_Matrix3, axes: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

phys_PxTransformFromSegment :: (p0: *physx_Vector3, p1: *physx_Vector3, halfHeight: *float) -> physx_PxTransform #foreign physx_sys;

phys_PxTransformFromPlaneEquation :: (plane: *physx_PxPlane) -> physx_PxTransform #foreign physx_sys;

phys_PxPlaneEquationFromTransform :: (pose: *physx_PxTransform) -> physx_PxPlane #foreign physx_sys;

phys_PxSlerp :: (t: float, left: *physx_PxQuat, right: *physx_PxQuat) -> physx_PxQuat #foreign physx_sys;

phys_PxIntegrateTransform :: (curTrans: *physx_PxTransform, linvel: *physx_Vector3, angvel: *physx_Vector3, timeStep: float, result: *physx_PxTransform) -> void #foreign physx_sys;

phys_PxExp :: (v: *physx_Vector3) -> physx_PxQuat #foreign physx_sys;

phys_PxOptimizeBoundingBox :: (basis: *physx_Matrix3) -> physx_Vector3 #foreign physx_sys;

phys_PxLog :: (q: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

phys_PxLargestAxis :: (v: *physx_Vector3) -> u32 #foreign physx_sys;

phys_PxTanHalf :: (sin: float, cos: float) -> float #foreign physx_sys;

phys_PxEllipseClamp :: (pos32: *physx_Vector3, radii: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

phys_PxSeparateSwingTwist :: (q: *physx_PxQuat, swing: *physx_PxQuat, twist: *physx_PxQuat) -> void #foreign physx_sys;

phys_PxComputeAngle :: (v0: *physx_Vector3, v1: *physx_Vector3) -> float #foreign physx_sys;

phys_PxComputeBasisVectors :: (dir: *physx_Vector3, right: *physx_Vector3, up: *physx_Vector3) -> void #foreign physx_sys;

phys_PxComputeBasisVectors_1 :: (p0: *physx_Vector3, p1: *physx_Vector3, dir: *physx_Vector3, right: *physx_Vector3, up: *physx_Vector3) -> void #foreign physx_sys;

phys_PxGetNextIndex3 :: (i: u32) -> u32 #foreign physx_sys;

phys_computeBarycentric :: (a: *physx_Vector3, b: *physx_Vector3, c: *physx_Vector3, d: *physx_Vector3, p: *physx_Vector3, bary: *physx_Vector4) -> void #foreign physx_sys;

phys_computeBarycentric_1 :: (a: *physx_Vector3, b: *physx_Vector3, c: *physx_Vector3, p: *physx_Vector3, bary: *physx_Vector4) -> void #foreign physx_sys;

Interpolation_PxLerp :: (a: float, b: float, t: float) -> float #foreign physx_sys;

Interpolation_PxBiLerp :: (f00: float, f10: float, f01: float, f11: float, tx: float, ty: float) -> float #foreign physx_sys;

Interpolation_PxTriLerp :: (f000: float, f100: float, f010: float, f110: float, f001: float, f101: float, f011: float, f111: float, tx: float, ty: float, tz: float) -> float #foreign physx_sys;

Interpolation_PxSDFIdx :: (i: u32, j: u32, k: u32, nbX: u32, nbY: u32) -> u32 #foreign physx_sys;

Interpolation_PxSDFSampleImpl :: (sdf: *float, localPos: *physx_Vector3, sdfBoxLower: *physx_Vector3, sdfBoxHigher: *physx_Vector3, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, tolerance: float) -> float #foreign physx_sys;

phys_PxSdfSample :: (sdf: *float, localPos: *physx_Vector3, sdfBoxLower: *physx_Vector3, sdfBoxHigher: *physx_Vector3, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, gradient: *physx_Vector3, tolerance: float) -> float #foreign physx_sys;

PxMutexImpl_new_alloc :: () -> *physx_PxMutexImpl #foreign physx_sys;

PxMutexImpl_delete :: (self: *physx_PxMutexImpl) -> void #foreign physx_sys;

PxMutexImpl_lock_mut :: (self: *physx_PxMutexImpl) -> void #foreign physx_sys;

PxMutexImpl_trylock_mut :: (self: *physx_PxMutexImpl) -> bool #foreign physx_sys;

PxMutexImpl_unlock_mut :: (self: *physx_PxMutexImpl) -> void #foreign physx_sys;

PxMutexImpl_getSize :: () -> u32 #foreign physx_sys;

PxReadWriteLock_new_alloc :: () -> *physx_PxReadWriteLock #foreign physx_sys;

PxReadWriteLock_delete :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys;

PxReadWriteLock_lockReader_mut :: (self: *physx_PxReadWriteLock, takeLock: bool) -> void #foreign physx_sys;

PxReadWriteLock_lockWriter_mut :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys;

PxReadWriteLock_unlockReader_mut :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys;

PxReadWriteLock_unlockWriter_mut :: (self: *physx_PxReadWriteLock) -> void #foreign physx_sys;

PxProfilerCallback_zoneStart_mut :: (self: *physx_PxProfilerCallback, eventName: *u8, detached: bool, contextId: u64) -> *void #foreign physx_sys;

PxProfilerCallback_zoneEnd_mut :: (self: *physx_PxProfilerCallback, profilerData: *void, eventName: *u8, detached: bool, contextId: u64) -> void #foreign physx_sys;

PxProfileScoped_new_alloc :: (callback: *physx_PxProfilerCallback, eventName: *u8, detached: bool, contextId: u64) -> *physx_PxProfileScoped #foreign physx_sys;

PxProfileScoped_delete :: (self: *physx_PxProfileScoped) -> void #foreign physx_sys;

PxSListEntry_new :: () -> physx_PxSListEntry #foreign physx_sys;

PxSListEntry_next_mut :: (self: *physx_PxSListEntry) -> *physx_PxSListEntry #foreign physx_sys;

PxSListImpl_new_alloc :: () -> *physx_PxSListImpl #foreign physx_sys;

PxSListImpl_delete :: (self: *physx_PxSListImpl) -> void #foreign physx_sys;

PxSListImpl_push_mut :: (self: *physx_PxSListImpl, entry: *physx_PxSListEntry) -> void #foreign physx_sys;

PxSListImpl_pop_mut :: (self: *physx_PxSListImpl) -> *physx_PxSListEntry #foreign physx_sys;

PxSListImpl_flush_mut :: (self: *physx_PxSListImpl) -> *physx_PxSListEntry #foreign physx_sys;

PxSListImpl_getSize :: () -> u32 #foreign physx_sys;

PxSyncImpl_new_alloc :: () -> *physx_PxSyncImpl #foreign physx_sys;

PxSyncImpl_delete :: (self: *physx_PxSyncImpl) -> void #foreign physx_sys;

PxSyncImpl_wait_mut :: (self: *physx_PxSyncImpl, milliseconds: u32) -> bool #foreign physx_sys;

PxSyncImpl_set_mut :: (self: *physx_PxSyncImpl) -> void #foreign physx_sys;

PxSyncImpl_reset_mut :: (self: *physx_PxSyncImpl) -> void #foreign physx_sys;

PxSyncImpl_getSize :: () -> u32 #foreign physx_sys;

PxRunnable_new_alloc :: () -> *physx_PxRunnable #foreign physx_sys;

PxRunnable_delete :: (self: *physx_PxRunnable) -> void #foreign physx_sys;

PxRunnable_execute_mut :: (self: *physx_PxRunnable) -> void #foreign physx_sys;

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

PxTime_getElapsedSeconds_mut :: (self: *physx_PxTime) -> float64 #foreign physx_sys;

PxTime_peekElapsedSeconds_mut :: (self: *physx_PxTime) -> float64 #foreign physx_sys;

PxTime_getLastTime :: (self: *physx_PxTime) -> float64 #foreign physx_sys;

PxVec2_new :: () -> physx_PxVec2 #foreign physx_sys;

PxVec2_new_1 :: (anon_param0: s32) -> physx_PxVec2 #foreign physx_sys;

PxVec2_new_2 :: (a: float) -> physx_PxVec2 #foreign physx_sys;

PxVec2_new_3 :: (nx: float, ny: float) -> physx_PxVec2 #foreign physx_sys;

PxVec2_isZero :: (self: *physx_PxVec2) -> bool #foreign physx_sys;

PxVec2_isFinite :: (self: *physx_PxVec2) -> bool #foreign physx_sys;

PxVec2_isNormalized :: (self: *physx_PxVec2) -> bool #foreign physx_sys;

PxVec2_magnitudeSquared :: (self: *physx_PxVec2) -> float #foreign physx_sys;

PxVec2_magnitude :: (self: *physx_PxVec2) -> float #foreign physx_sys;

PxVec2_dot :: (self: *physx_PxVec2, v: *physx_PxVec2) -> float #foreign physx_sys;

PxVec2_getNormalized :: (self: *physx_PxVec2) -> physx_PxVec2 #foreign physx_sys;

PxVec2_normalize_mut :: (self: *physx_PxVec2) -> float #foreign physx_sys;

PxVec2_multiply :: (self: *physx_PxVec2, a: *physx_PxVec2) -> physx_PxVec2 #foreign physx_sys;

PxVec2_minimum :: (self: *physx_PxVec2, v: *physx_PxVec2) -> physx_PxVec2 #foreign physx_sys;

PxVec2_minElement :: (self: *physx_PxVec2) -> float #foreign physx_sys;

PxVec2_maximum :: (self: *physx_PxVec2, v: *physx_PxVec2) -> physx_PxVec2 #foreign physx_sys;

PxVec2_maxElement :: (self: *physx_PxVec2) -> float #foreign physx_sys;

PxStridedData_new :: () -> physx_PxStridedData #foreign physx_sys;

PxBoundedData_new :: () -> physx_PxBoundedData #foreign physx_sys;

PxDebugPos32_new :: (p: *physx_Vector3, c: *u32) -> physx_PxDebugPos32 #foreign physx_sys;

PxDebugLine_new :: (p0: *physx_Vector3, p1: *physx_Vector3, c: *u32) -> physx_PxDebugLine #foreign physx_sys;

PxDebugTriangle_new :: (p0: *physx_Vector3, p1: *physx_Vector3, p2: *physx_Vector3, c: *u32) -> physx_PxDebugTriangle #foreign physx_sys;

PxDebugText_new :: () -> physx_PxDebugText #foreign physx_sys;

PxDebugText_new_1 :: (pos: *physx_Vector3, sz: *float, clr: *u32, str: *u8) -> physx_PxDebugText #foreign physx_sys;

PxRenderBuffer_delete :: (self: *physx_PxRenderBuffer) -> void #foreign physx_sys;

PxRenderBuffer_getNbPos32s :: (self: *physx_PxRenderBuffer) -> u32 #foreign physx_sys;

PxRenderBuffer_getPos32s :: (self: *physx_PxRenderBuffer) -> *physx_PxDebugPos32 #foreign physx_sys;

PxRenderBuffer_addPos32_mut :: (self: *physx_PxRenderBuffer, pos32: *physx_PxDebugPos32) -> void #foreign physx_sys;

PxRenderBuffer_getNbLines :: (self: *physx_PxRenderBuffer) -> u32 #foreign physx_sys;

PxRenderBuffer_getLines :: (self: *physx_PxRenderBuffer) -> *physx_PxDebugLine #foreign physx_sys;

PxRenderBuffer_addLine_mut :: (self: *physx_PxRenderBuffer, line: *physx_PxDebugLine) -> void #foreign physx_sys;

PxRenderBuffer_reserveLines_mut :: (self: *physx_PxRenderBuffer, nbLines: u32) -> *physx_PxDebugLine #foreign physx_sys;

PxRenderBuffer_reservePos32s_mut :: (self: *physx_PxRenderBuffer, nbLines: u32) -> *physx_PxDebugPos32 #foreign physx_sys;

PxRenderBuffer_getNbTriangles :: (self: *physx_PxRenderBuffer) -> u32 #foreign physx_sys;

PxRenderBuffer_getTriangles :: (self: *physx_PxRenderBuffer) -> *physx_PxDebugTriangle #foreign physx_sys;

PxRenderBuffer_addTriangle_mut :: (self: *physx_PxRenderBuffer, triangle: *physx_PxDebugTriangle) -> void #foreign physx_sys;

PxRenderBuffer_append_mut :: (self: *physx_PxRenderBuffer, other: *physx_PxRenderBuffer) -> void #foreign physx_sys;

PxRenderBuffer_clear_mut :: (self: *physx_PxRenderBuffer) -> void #foreign physx_sys;

PxRenderBuffer_shift_mut :: (self: *physx_PxRenderBuffer, delta: *physx_Vector3) -> void #foreign physx_sys;

PxRenderBuffer_empty :: (self: *physx_PxRenderBuffer) -> bool #foreign physx_sys;

PxProcessPxBaseCallback_delete :: (self: *physx_PxProcessPxBaseCallback) -> void #foreign physx_sys;

PxProcessPxBaseCallback_process_mut :: (self: *physx_PxProcessPxBaseCallback, anon_param0: *physx_PxBase) -> void #foreign physx_sys;

PxSerializationContext_registerReference_mut :: (self: *physx_PxSerializationContext, base: *physx_PxBase, kind: u32, reference: u64) -> void #foreign physx_sys;

PxSerializationContext_getCollection :: (self: *physx_PxSerializationContext) -> *physx_PxCollection #foreign physx_sys;

PxSerializationContext_writeData_mut :: (self: *physx_PxSerializationContext, data: *void, size: u32) -> void #foreign physx_sys;

PxSerializationContext_alignData_mut :: (self: *physx_PxSerializationContext, alignment: u32) -> void #foreign physx_sys;

PxSerializationContext_writeName_mut :: (self: *physx_PxSerializationContext, name: *u8) -> void #foreign physx_sys;

PxDeserializationContext_resolveReference :: (self: *physx_PxDeserializationContext, kind: u32, reference: u64) -> *physx_PxBase #foreign physx_sys;

PxDeserializationContext_readName_mut :: (self: *physx_PxDeserializationContext, name: **u8) -> void #foreign physx_sys;

PxDeserializationContext_alignExtraData_mut :: (self: *physx_PxDeserializationContext, alignment: u32) -> void #foreign physx_sys;

PxSerializationRegistry_registerSerializer_mut :: (self: *physx_PxSerializationRegistry, type: u16, serializer: *physx_PxSerializer) -> void #foreign physx_sys;

PxSerializationRegistry_unregisterSerializer_mut :: (self: *physx_PxSerializationRegistry, type: u16) -> *physx_PxSerializer #foreign physx_sys;

PxSerializationRegistry_getSerializer :: (self: *physx_PxSerializationRegistry, type: u16) -> *physx_PxSerializer #foreign physx_sys;

PxSerializationRegistry_registerRepXSerializer_mut :: (self: *physx_PxSerializationRegistry, type: u16, serializer: *physx_PxRepXSerializer) -> void #foreign physx_sys;

PxSerializationRegistry_unregisterRepXSerializer_mut :: (self: *physx_PxSerializationRegistry, type: u16) -> *physx_PxRepXSerializer #foreign physx_sys;

PxSerializationRegistry_getRepXSerializer :: (self: *physx_PxSerializationRegistry, typeName: *u8) -> *physx_PxRepXSerializer #foreign physx_sys;

PxSerializationRegistry_release_mut :: (self: *physx_PxSerializationRegistry) -> void #foreign physx_sys;

PxCollection_add_mut :: (self: *physx_PxCollection, object: *physx_PxBase, id: u64) -> void #foreign physx_sys;

PxCollection_remove_mut :: (self: *physx_PxCollection, object: *physx_PxBase) -> void #foreign physx_sys;

PxCollection_contains :: (self: *physx_PxCollection, object: *physx_PxBase) -> bool #foreign physx_sys;

PxCollection_addId_mut :: (self: *physx_PxCollection, object: *physx_PxBase, id: u64) -> void #foreign physx_sys;

PxCollection_removeId_mut :: (self: *physx_PxCollection, id: u64) -> void #foreign physx_sys;

PxCollection_add_mut_1 :: (self: *physx_PxCollection, collection: *physx_PxCollection) -> void #foreign physx_sys;

PxCollection_remove_mut_1 :: (self: *physx_PxCollection, collection: *physx_PxCollection) -> void #foreign physx_sys;

PxCollection_getNbObjects :: (self: *physx_PxCollection) -> u32 #foreign physx_sys;

PxCollection_getObject :: (self: *physx_PxCollection, index: u32) -> *physx_PxBase #foreign physx_sys;

PxCollection_getObjects :: (self: *physx_PxCollection, userBuffer: **physx_PxBase, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxCollection_find :: (self: *physx_PxCollection, id: u64) -> *physx_PxBase #foreign physx_sys;

PxCollection_getNbIds :: (self: *physx_PxCollection) -> u32 #foreign physx_sys;

PxCollection_getIds :: (self: *physx_PxCollection, userBuffer: *u64, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxCollection_getId :: (self: *physx_PxCollection, object: *physx_PxBase) -> u64 #foreign physx_sys;

PxCollection_release_mut :: (self: *physx_PxCollection) -> void #foreign physx_sys;

phys_PxCreateCollection :: () -> *physx_PxCollection #foreign physx_sys;

PxBase_release_mut :: (self: *physx_PxBase) -> void #foreign physx_sys;

PxBase_getConcreteTypeName :: (self: *physx_PxBase) -> *u8 #foreign physx_sys;

PxBase_getConcreteType :: (self: *physx_PxBase) -> u16 #foreign physx_sys;

PxBase_setBaseFlag_mut :: (self: *physx_PxBase, flag: s32, value: bool) -> void #foreign physx_sys;

PxBase_setBaseFlags_mut :: (self: *physx_PxBase, inFlags: u16) -> void #foreign physx_sys;

PxBase_getBaseFlags :: (self: *physx_PxBase) -> u16 #foreign physx_sys;

PxBase_isReleasable :: (self: *physx_PxBase) -> bool #foreign physx_sys;

PxRefCounted_release_mut :: (self: *physx_PxRefCounted) -> void #foreign physx_sys;

PxRefCounted_getReferenceCount :: (self: *physx_PxRefCounted) -> u32 #foreign physx_sys;

PxRefCounted_acquireReference_mut :: (self: *physx_PxRefCounted) -> void #foreign physx_sys;

PxTolerancesScale_new :: (defaultLength: float, defaultSpeed: float) -> physx_PxTolerancesScale #foreign physx_sys;

PxTolerancesScale_isValid :: (self: *physx_PxTolerancesScale) -> bool #foreign physx_sys;

PxStringTable_allocateStr_mut :: (self: *physx_PxStringTable, inSrc: *u8) -> *u8 #foreign physx_sys;

PxStringTable_release_mut :: (self: *physx_PxStringTable) -> void #foreign physx_sys;

PxSerializer_getConcreteTypeName :: (self: *physx_PxSerializer) -> *u8 #foreign physx_sys;

PxSerializer_requiresObjects :: (self: *physx_PxSerializer, anon_param0: *physx_PxBase, anon_param1: *physx_PxProcessPxBaseCallback) -> void #foreign physx_sys;

PxSerializer_isSubordinate :: (self: *physx_PxSerializer) -> bool #foreign physx_sys;

PxSerializer_exportExtraData :: (self: *physx_PxSerializer, anon_param0: *physx_PxBase, anon_param1: *physx_PxSerializationContext) -> void #foreign physx_sys;

PxSerializer_exportData :: (self: *physx_PxSerializer, anon_param0: *physx_PxBase, anon_param1: *physx_PxSerializationContext) -> void #foreign physx_sys;

PxSerializer_registerReferences :: (self: *physx_PxSerializer, obj: *physx_PxBase, s: *physx_PxSerializationContext) -> void #foreign physx_sys;

PxSerializer_getClassSize :: (self: *physx_PxSerializer) -> u64 #foreign physx_sys;

PxSerializer_createObject :: (self: *physx_PxSerializer, address: **u8, context_pod: *physx_PxDeserializationContext) -> *physx_PxBase #foreign physx_sys;

PxSerializer_delete :: (self: *physx_PxSerializer) -> void #foreign physx_sys;

PxInsertionCallback_buildObjectFromData_mut :: (self: *physx_PxInsertionCallback, type: s32, data: *void) -> *physx_PxBase #foreign physx_sys;

PxTaskManager_setCpuDispatcher_mut :: (self: *physx_PxTaskManager, ref: *physx_PxCpuDispatcher) -> void #foreign physx_sys;

PxTaskManager_getCpuDispatcher :: (self: *physx_PxTaskManager) -> *physx_PxCpuDispatcher #foreign physx_sys;

PxTaskManager_resetDependencies_mut :: (self: *physx_PxTaskManager) -> void #foreign physx_sys;

PxTaskManager_startSimulation_mut :: (self: *physx_PxTaskManager) -> void #foreign physx_sys;

PxTaskManager_stopSimulation_mut :: (self: *physx_PxTaskManager) -> void #foreign physx_sys;

PxTaskManager_taskCompleted_mut :: (self: *physx_PxTaskManager, task: *physx_PxTask) -> void #foreign physx_sys;

PxTaskManager_getNamedTask_mut :: (self: *physx_PxTaskManager, name: *u8) -> u32 #foreign physx_sys;

PxTaskManager_submitNamedTask_mut :: (self: *physx_PxTaskManager, task: *physx_PxTask, name: *u8, type: s32) -> u32 #foreign physx_sys;

PxTaskManager_submitUnnamedTask_mut :: (self: *physx_PxTaskManager, task: *physx_PxTask, type: s32) -> u32 #foreign physx_sys;

PxTaskManager_getTaskFromID_mut :: (self: *physx_PxTaskManager, id: u32) -> *physx_PxTask #foreign physx_sys;

PxTaskManager_release_mut :: (self: *physx_PxTaskManager) -> void #foreign physx_sys;

PxTaskManager_createTaskManager :: (errorCallback: *physx_PxErrorCallback, anon_param1: *physx_PxCpuDispatcher) -> *physx_PxTaskManager #foreign physx_sys;

PxCpuDispatcher_submitTask_mut :: (self: *physx_PxCpuDispatcher, task: *physx_PxBaseTask) -> void #foreign physx_sys;

PxCpuDispatcher_getWorkerCount :: (self: *physx_PxCpuDispatcher) -> u32 #foreign physx_sys;

PxCpuDispatcher_delete :: (self: *physx_PxCpuDispatcher) -> void #foreign physx_sys;

PxBaseTask_run_mut :: (self: *physx_PxBaseTask) -> void #foreign physx_sys;

PxBaseTask_getName :: (self: *physx_PxBaseTask) -> *u8 #foreign physx_sys;

PxBaseTask_addReference_mut :: (self: *physx_PxBaseTask) -> void #foreign physx_sys;

PxBaseTask_removeReference_mut :: (self: *physx_PxBaseTask) -> void #foreign physx_sys;

PxBaseTask_getReference :: (self: *physx_PxBaseTask) -> s32 #foreign physx_sys;

PxBaseTask_release_mut :: (self: *physx_PxBaseTask) -> void #foreign physx_sys;

PxBaseTask_getTaskManager :: (self: *physx_PxBaseTask) -> *physx_PxTaskManager #foreign physx_sys;

PxBaseTask_setContextId_mut :: (self: *physx_PxBaseTask, id: u64) -> void #foreign physx_sys;

PxBaseTask_getContextId :: (self: *physx_PxBaseTask) -> u64 #foreign physx_sys;

PxTask_release_mut :: (self: *physx_PxTask) -> void #foreign physx_sys;

PxTask_finishBefore_mut :: (self: *physx_PxTask, taskID: u32) -> void #foreign physx_sys;

PxTask_startAfter_mut :: (self: *physx_PxTask, taskID: u32) -> void #foreign physx_sys;

PxTask_addReference_mut :: (self: *physx_PxTask) -> void #foreign physx_sys;

PxTask_removeReference_mut :: (self: *physx_PxTask) -> void #foreign physx_sys;

PxTask_getReference :: (self: *physx_PxTask) -> s32 #foreign physx_sys;

PxTask_getTaskID :: (self: *physx_PxTask) -> u32 #foreign physx_sys;

PxTask_submitted_mut :: (self: *physx_PxTask) -> void #foreign physx_sys;

PxLightCpuTask_setContinuation_mut :: (self: *physx_PxLightCpuTask, tm: *physx_PxTaskManager, c: *physx_PxBaseTask) -> void #foreign physx_sys;

PxLightCpuTask_setContinuation_mut_1 :: (self: *physx_PxLightCpuTask, c: *physx_PxBaseTask) -> void #foreign physx_sys;

PxLightCpuTask_getContinuation :: (self: *physx_PxLightCpuTask) -> *physx_PxBaseTask #foreign physx_sys;

PxLightCpuTask_removeReference_mut :: (self: *physx_PxLightCpuTask) -> void #foreign physx_sys;

PxLightCpuTask_getReference :: (self: *physx_PxLightCpuTask) -> s32 #foreign physx_sys;

PxLightCpuTask_addReference_mut :: (self: *physx_PxLightCpuTask) -> void #foreign physx_sys;

PxLightCpuTask_release_mut :: (self: *physx_PxLightCpuTask) -> void #foreign physx_sys;

PxGeometry_getType :: (self: *physx_PxGeometry) -> s32 #foreign physx_sys;

PxBoxGeometry_new :: (hx: float, hy: float, hz: float) -> physx_PxBoxGeometry #foreign physx_sys;

PxBoxGeometry_new_1 :: (halfExtents: physx_Vector3) -> physx_PxBoxGeometry #foreign physx_sys;

PxBoxGeometry_isValid :: (self: *physx_PxBoxGeometry) -> bool #foreign physx_sys;

PxBVHRaycastCallback_delete :: (self: *physx_PxBVHRaycastCallback) -> void #foreign physx_sys;

PxBVHRaycastCallback_reportHit_mut :: (self: *physx_PxBVHRaycastCallback, boundsIndex: u32, distance: *float) -> bool #foreign physx_sys;

PxBVHOverlapCallback_delete :: (self: *physx_PxBVHOverlapCallback) -> void #foreign physx_sys;

PxBVHOverlapCallback_reportHit_mut :: (self: *physx_PxBVHOverlapCallback, boundsIndex: u32) -> bool #foreign physx_sys;

PxBVHTraversalCallback_delete :: (self: *physx_PxBVHTraversalCallback) -> void #foreign physx_sys;

PxBVHTraversalCallback_visitNode_mut :: (self: *physx_PxBVHTraversalCallback, bounds: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBVHTraversalCallback_reportLeaf_mut :: (self: *physx_PxBVHTraversalCallback, nbPrims: u32, prims: *u32) -> bool #foreign physx_sys;

PxBVH_raycast :: (self: *physx_PxBVH, origin: *physx_Vector3, unitDir: *physx_Vector3, maxDist: float, cb: *physx_PxBVHRaycastCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_sweep :: (self: *physx_PxBVH, geom: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *physx_Vector3, maxDist: float, cb: *physx_PxBVHRaycastCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_overlap :: (self: *physx_PxBVH, geom: *physx_PxGeometry, pose: *physx_PxTransform, cb: *physx_PxBVHOverlapCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_cull :: (self: *physx_PxBVH, nbPlanes: u32, planes: *physx_PxPlane, cb: *physx_PxBVHOverlapCallback, queryFlags: u32) -> bool #foreign physx_sys;

PxBVH_getNbBounds :: (self: *physx_PxBVH) -> u32 #foreign physx_sys;

PxBVH_getBounds :: (self: *physx_PxBVH) -> *physx_PxBounds3 #foreign physx_sys;

PxBVH_getBoundsForModification_mut :: (self: *physx_PxBVH) -> *physx_PxBounds3 #foreign physx_sys;

PxBVH_refit_mut :: (self: *physx_PxBVH) -> void #foreign physx_sys;

PxBVH_updateBounds_mut :: (self: *physx_PxBVH, boundsIndex: u32, newBounds: *physx_PxBounds3) -> bool #foreign physx_sys;

PxBVH_partialRefit_mut :: (self: *physx_PxBVH) -> void #foreign physx_sys;

PxBVH_traverse :: (self: *physx_PxBVH, cb: *physx_PxBVHTraversalCallback) -> bool #foreign physx_sys;

PxBVH_getConcreteTypeName :: (self: *physx_PxBVH) -> *u8 #foreign physx_sys;

PxCapsuleGeometry_new :: (radius_: float, halfHeight_: float) -> physx_PxCapsuleGeometry #foreign physx_sys;

PxCapsuleGeometry_isValid :: (self: *physx_PxCapsuleGeometry) -> bool #foreign physx_sys;

PxConvexMesh_getNbVertices :: (self: *physx_PxConvexMesh) -> u32 #foreign physx_sys;

PxConvexMesh_getVertices :: (self: *physx_PxConvexMesh) -> *physx_Vector3 #foreign physx_sys;

PxConvexMesh_getIndexBuffer :: (self: *physx_PxConvexMesh) -> *u8 #foreign physx_sys;

PxConvexMesh_getNbPolygons :: (self: *physx_PxConvexMesh) -> u32 #foreign physx_sys;

PxConvexMesh_getPolygonData :: (self: *physx_PxConvexMesh, index: u32, data: *physx_PxHullPolygon) -> bool #foreign physx_sys;

PxConvexMesh_release_mut :: (self: *physx_PxConvexMesh) -> void #foreign physx_sys;

PxConvexMesh_getMassInformation :: (self: *physx_PxConvexMesh, mass: *float, localInertia: *physx_Matrix3, localCenterOfMass: *physx_Vector3) -> void #foreign physx_sys;

PxConvexMesh_getLocalBounds :: (self: *physx_PxConvexMesh) -> physx_PxBounds3 #foreign physx_sys;

PxConvexMesh_getSDF :: (self: *physx_PxConvexMesh) -> *float #foreign physx_sys;

PxConvexMesh_getConcreteTypeName :: (self: *physx_PxConvexMesh) -> *u8 #foreign physx_sys;

PxConvexMesh_isGpuCompatible :: (self: *physx_PxConvexMesh) -> bool #foreign physx_sys;

PxMeshScale_new :: () -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_new_1 :: (r: float) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_new_2 :: (s: *physx_Vector3) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_new_3 :: (s: *physx_Vector3, r: *physx_PxQuat) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_isIdentity :: (self: *physx_PxMeshScale) -> bool #foreign physx_sys;

PxMeshScale_getInverse :: (self: *physx_PxMeshScale) -> physx_PxMeshScale #foreign physx_sys;

PxMeshScale_toMat33 :: (self: *physx_PxMeshScale) -> physx_Matrix3 #foreign physx_sys;

PxMeshScale_hasNegativeDeterminant :: (self: *physx_PxMeshScale) -> bool #foreign physx_sys;

PxMeshScale_transform :: (self: *physx_PxMeshScale, v: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

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

PxCustomGeometryCallbacks_raycast :: (self: *physx_PxCustomGeometryCallbacks, origin: *physx_Vector3, unitDir: *physx_Vector3, geom: *physx_PxGeometry, pose: *physx_PxTransform, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *physx_PxGeomRaycastHit, stride: u32, threadContext: *physx_PxQueryThreadContext) -> u32 #foreign physx_sys;

PxCustomGeometryCallbacks_overlap :: (self: *physx_PxCustomGeometryCallbacks, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_sweep :: (self: *physx_PxCustomGeometryCallbacks, unitDir: *physx_Vector3, maxDist: float, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, sweepHit: *physx_PxGeomSweepHit, hitFlags: u16, inflation: float, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_computeMassProperties :: (self: *physx_PxCustomGeometryCallbacks, geometry: *physx_PxGeometry, massProperties: *physx_PxMassProperties) -> void #foreign physx_sys;

PxCustomGeometryCallbacks_usePersistentContactManifold :: (self: *physx_PxCustomGeometryCallbacks, geometry: *physx_PxGeometry, breakingThreshold: *float) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_delete :: (self: *physx_PxCustomGeometryCallbacks) -> void #foreign physx_sys;

PxCustomGeometry_new :: () -> physx_PxCustomGeometry #foreign physx_sys;

PxCustomGeometry_new_1 :: (_callbacks: *physx_PxCustomGeometryCallbacks) -> physx_PxCustomGeometry #foreign physx_sys;

PxCustomGeometry_isValid :: (self: *physx_PxCustomGeometry) -> bool #foreign physx_sys;

PxCustomGeometry_getCustomType :: (self: *physx_PxCustomGeometry) -> physx_PxCustomGeometryType #foreign physx_sys;

PxGeometryHolder_getType :: (self: *physx_PxGeometryHolder) -> s32 #foreign physx_sys;

PxGeometryHolder_any_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxGeometry #foreign physx_sys;

PxGeometryHolder_any :: (self: *physx_PxGeometryHolder) -> *physx_PxGeometry #foreign physx_sys;

PxGeometryHolder_sphere_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxSphereGeometry #foreign physx_sys;

PxGeometryHolder_sphere :: (self: *physx_PxGeometryHolder) -> *physx_PxSphereGeometry #foreign physx_sys;

PxGeometryHolder_plane_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxPlaneGeometry #foreign physx_sys;

PxGeometryHolder_plane :: (self: *physx_PxGeometryHolder) -> *physx_PxPlaneGeometry #foreign physx_sys;

PxGeometryHolder_capsule_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxCapsuleGeometry #foreign physx_sys;

PxGeometryHolder_capsule :: (self: *physx_PxGeometryHolder) -> *physx_PxCapsuleGeometry #foreign physx_sys;

PxGeometryHolder_box_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxBoxGeometry #foreign physx_sys;

PxGeometryHolder_box :: (self: *physx_PxGeometryHolder) -> *physx_PxBoxGeometry #foreign physx_sys;

PxGeometryHolder_convexMesh_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxConvexMeshGeometry #foreign physx_sys;

PxGeometryHolder_convexMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxConvexMeshGeometry #foreign physx_sys;

PxGeometryHolder_tetMesh_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxTetrahedronMeshGeometry #foreign physx_sys;

PxGeometryHolder_tetMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxTetrahedronMeshGeometry #foreign physx_sys;

PxGeometryHolder_triangleMesh_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxTriangleMeshGeometry #foreign physx_sys;

PxGeometryHolder_triangleMesh :: (self: *physx_PxGeometryHolder) -> *physx_PxTriangleMeshGeometry #foreign physx_sys;

PxGeometryHolder_heightField_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxHeightFieldGeometry #foreign physx_sys;

PxGeometryHolder_heightField :: (self: *physx_PxGeometryHolder) -> *physx_PxHeightFieldGeometry #foreign physx_sys;

PxGeometryHolder_particleSystem_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxParticleSystemGeometry #foreign physx_sys;

PxGeometryHolder_particleSystem :: (self: *physx_PxGeometryHolder) -> *physx_PxParticleSystemGeometry #foreign physx_sys;

PxGeometryHolder_hairSystem_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxHairSystemGeometry #foreign physx_sys;

PxGeometryHolder_hairSystem :: (self: *physx_PxGeometryHolder) -> *physx_PxHairSystemGeometry #foreign physx_sys;

PxGeometryHolder_custom_mut :: (self: *physx_PxGeometryHolder) -> *physx_PxCustomGeometry #foreign physx_sys;

PxGeometryHolder_custom :: (self: *physx_PxGeometryHolder) -> *physx_PxCustomGeometry #foreign physx_sys;

PxGeometryHolder_storeAny_mut :: (self: *physx_PxGeometryHolder, geometry: *physx_PxGeometry) -> void #foreign physx_sys;

PxGeometryHolder_new :: () -> physx_PxGeometryHolder #foreign physx_sys;

PxGeometryHolder_new_1 :: (geometry: *physx_PxGeometry) -> physx_PxGeometryHolder #foreign physx_sys;

PxGeometryQuery_raycast :: (origin: *physx_Vector3, unitDir: *physx_Vector3, geom: *physx_PxGeometry, pose: *physx_PxTransform, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *physx_PxGeomRaycastHit, stride: u32, queryFlags: u32, threadContext: *physx_PxQueryThreadContext) -> u32 #foreign physx_sys;

PxGeometryQuery_overlap :: (geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, queryFlags: u32, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxGeometryQuery_sweep :: (unitDir: *physx_Vector3, maxDist: float, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, sweepHit: *physx_PxGeomSweepHit, hitFlags: u16, inflation: float, queryFlags: u32, threadContext: *physx_PxQueryThreadContext) -> bool #foreign physx_sys;

PxGeometryQuery_computePenetration :: (direction: *physx_Vector3, depth: *float, geom0: *physx_PxGeometry, pose0: *physx_PxTransform, geom1: *physx_PxGeometry, pose1: *physx_PxTransform, queryFlags: u32) -> bool #foreign physx_sys;

PxGeometryQuery_pos32Distance :: (pos32: *physx_Vector3, geom: *physx_PxGeometry, pose: *physx_PxTransform, closestPos32: *physx_Vector3, closestIndex: *u32, queryFlags: u32) -> float #foreign physx_sys;

PxGeometryQuery_computeGeomBounds :: (bounds: *physx_PxBounds3, geom: *physx_PxGeometry, pose: *physx_PxTransform, offset: float, inflation: float, queryFlags: u32) -> void #foreign physx_sys;

PxGeometryQuery_isValid :: (geom: *physx_PxGeometry) -> bool #foreign physx_sys;

PxHeightFieldSample_tessFlag :: (self: *physx_PxHeightFieldSample) -> u8 #foreign physx_sys;

PxHeightFieldSample_setTessFlag_mut :: (self: *physx_PxHeightFieldSample) -> void #foreign physx_sys;

PxHeightFieldSample_clearTessFlag_mut :: (self: *physx_PxHeightFieldSample) -> void #foreign physx_sys;

PxHeightField_release_mut :: (self: *physx_PxHeightField) -> void #foreign physx_sys;

PxHeightField_saveCells :: (self: *physx_PxHeightField, destBuffer: *void, destBufferSize: u32) -> u32 #foreign physx_sys;

PxHeightField_modifySamples_mut :: (self: *physx_PxHeightField, startCol: s32, startRow: s32, subfieldDesc: *physx_PxHeightFieldDesc, shrinkBounds: bool) -> bool #foreign physx_sys;

PxHeightField_getNbRows :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getNbColumns :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getFormat :: (self: *physx_PxHeightField) -> s32 #foreign physx_sys;

PxHeightField_getSampleStride :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getConvexEdgeThreshold :: (self: *physx_PxHeightField) -> float #foreign physx_sys;

PxHeightField_getFlags :: (self: *physx_PxHeightField) -> u16 #foreign physx_sys;

PxHeightField_getHeight :: (self: *physx_PxHeightField, x: float, z: float) -> float #foreign physx_sys;

PxHeightField_getTriangleMaterialIndex :: (self: *physx_PxHeightField, triangleIndex: u32) -> u16 #foreign physx_sys;

PxHeightField_getTriangleNormal :: (self: *physx_PxHeightField, triangleIndex: u32) -> physx_Vector3 #foreign physx_sys;

PxHeightField_getSample :: (self: *physx_PxHeightField, row: u32, column: u32) -> *physx_PxHeightFieldSample #foreign physx_sys;

PxHeightField_getTimestamp :: (self: *physx_PxHeightField) -> u32 #foreign physx_sys;

PxHeightField_getConcreteTypeName :: (self: *physx_PxHeightField) -> *u8 #foreign physx_sys;

PxHeightFieldDesc_new :: () -> physx_PxHeightFieldDesc #foreign physx_sys;

PxHeightFieldDesc_setToDefault_mut :: (self: *physx_PxHeightFieldDesc) -> void #foreign physx_sys;

PxHeightFieldDesc_isValid :: (self: *physx_PxHeightFieldDesc) -> bool #foreign physx_sys;

PxMeshQuery_getTriangle :: (triGeom: *physx_PxTriangleMeshGeometry, transform: *physx_PxTransform, triangleIndex: u32, triangle: *physx_PxTriangle, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign physx_sys;

PxMeshQuery_getTriangle_1 :: (hfGeom: *physx_PxHeightFieldGeometry, transform: *physx_PxTransform, triangleIndex: u32, triangle: *physx_PxTriangle, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign physx_sys;

PxMeshQuery_findOverlapTriangleMesh :: (geom: *physx_PxGeometry, geomPose: *physx_PxTransform, meshGeom: *physx_PxTriangleMeshGeometry, meshPose: *physx_PxTransform, results: *u32, maxResults: u32, startIndex: u32, overflow: *bool, queryFlags: u32) -> u32 #foreign physx_sys;

PxMeshQuery_findOverlapHeightField :: (geom: *physx_PxGeometry, geomPose: *physx_PxTransform, hfGeom: *physx_PxHeightFieldGeometry, hfPose: *physx_PxTransform, results: *u32, maxResults: u32, startIndex: u32, overflow: *bool, queryFlags: u32) -> u32 #foreign physx_sys;

PxMeshQuery_sweep :: (unitDir: *physx_Vector3, distance: float, geom: *physx_PxGeometry, pose: *physx_PxTransform, triangleCount: u32, triangles: *physx_PxTriangle, sweepHit: *physx_PxGeomSweepHit, hitFlags: u16, cachedIndex: *u32, inflation: float, float64Sided: bool, queryFlags: u32) -> bool #foreign physx_sys;

PxSimpleTriangleMesh_new :: () -> physx_PxSimpleTriangleMesh #foreign physx_sys;

PxSimpleTriangleMesh_setToDefault_mut :: (self: *physx_PxSimpleTriangleMesh) -> void #foreign physx_sys;

PxSimpleTriangleMesh_isValid :: (self: *physx_PxSimpleTriangleMesh) -> bool #foreign physx_sys;

PxTriangle_new_alloc :: () -> *physx_PxTriangle #foreign physx_sys;

PxTriangle_new_alloc_1 :: (p0: *physx_Vector3, p1: *physx_Vector3, p2: *physx_Vector3) -> *physx_PxTriangle #foreign physx_sys;

PxTriangle_delete :: (self: *physx_PxTriangle) -> void #foreign physx_sys;

PxTriangle_normal :: (self: *physx_PxTriangle, _normal: *physx_Vector3) -> void #foreign physx_sys;

PxTriangle_denormalizedNormal :: (self: *physx_PxTriangle, _normal: *physx_Vector3) -> void #foreign physx_sys;

PxTriangle_area :: (self: *physx_PxTriangle) -> float #foreign physx_sys;

PxTriangle_pos32FromUV :: (self: *physx_PxTriangle, u: float, v: float) -> physx_Vector3 #foreign physx_sys;

PxTrianglePadded_new_alloc :: () -> *physx_PxTrianglePadded #foreign physx_sys;

PxTrianglePadded_delete :: (self: *physx_PxTrianglePadded) -> void #foreign physx_sys;

PxTriangleMesh_getNbVertices :: (self: *physx_PxTriangleMesh) -> u32 #foreign physx_sys;

PxTriangleMesh_getVertices :: (self: *physx_PxTriangleMesh) -> *physx_Vector3 #foreign physx_sys;

PxTriangleMesh_getVerticesForModification_mut :: (self: *physx_PxTriangleMesh) -> *physx_Vector3 #foreign physx_sys;

PxTriangleMesh_refitBVH_mut :: (self: *physx_PxTriangleMesh) -> physx_PxBounds3 #foreign physx_sys;

PxTriangleMesh_getNbTriangles :: (self: *physx_PxTriangleMesh) -> u32 #foreign physx_sys;

PxTriangleMesh_getTriangles :: (self: *physx_PxTriangleMesh) -> *void #foreign physx_sys;

PxTriangleMesh_getTriangleMeshFlags :: (self: *physx_PxTriangleMesh) -> u8 #foreign physx_sys;

PxTriangleMesh_getTrianglesRemap :: (self: *physx_PxTriangleMesh) -> *u32 #foreign physx_sys;

PxTriangleMesh_release_mut :: (self: *physx_PxTriangleMesh) -> void #foreign physx_sys;

PxTriangleMesh_getTriangleMaterialIndex :: (self: *physx_PxTriangleMesh, triangleIndex: u32) -> u16 #foreign physx_sys;

PxTriangleMesh_getLocalBounds :: (self: *physx_PxTriangleMesh) -> physx_PxBounds3 #foreign physx_sys;

PxTriangleMesh_getSDF :: (self: *physx_PxTriangleMesh) -> *float #foreign physx_sys;

PxTriangleMesh_getSDFDimensions :: (self: *physx_PxTriangleMesh, numX: *u32, numY: *u32, numZ: *u32) -> void #foreign physx_sys;

PxTriangleMesh_setPreferSDFProjection_mut :: (self: *physx_PxTriangleMesh, preferProjection: bool) -> void #foreign physx_sys;

PxTriangleMesh_getPreferSDFProjection :: (self: *physx_PxTriangleMesh) -> bool #foreign physx_sys;

PxTriangleMesh_getMassInformation :: (self: *physx_PxTriangleMesh, mass: *float, localInertia: *physx_Matrix3, localCenterOfMass: *physx_Vector3) -> void #foreign physx_sys;

PxTetrahedron_new_alloc :: () -> *physx_PxTetrahedron #foreign physx_sys;

PxTetrahedron_new_alloc_1 :: (p0: *physx_Vector3, p1: *physx_Vector3, p2: *physx_Vector3, p3: *physx_Vector3) -> *physx_PxTetrahedron #foreign physx_sys;

PxTetrahedron_delete :: (self: *physx_PxTetrahedron) -> void #foreign physx_sys;

PxSoftBodyAuxData_release_mut :: (self: *physx_PxSoftBodyAuxData) -> void #foreign physx_sys;

PxTetrahedronMesh_getNbVertices :: (self: *physx_PxTetrahedronMesh) -> u32 #foreign physx_sys;

PxTetrahedronMesh_getVertices :: (self: *physx_PxTetrahedronMesh) -> *physx_Vector3 #foreign physx_sys;

PxTetrahedronMesh_getNbTetrahedrons :: (self: *physx_PxTetrahedronMesh) -> u32 #foreign physx_sys;

PxTetrahedronMesh_getTetrahedrons :: (self: *physx_PxTetrahedronMesh) -> *void #foreign physx_sys;

PxTetrahedronMesh_getTetrahedronMeshFlags :: (self: *physx_PxTetrahedronMesh) -> u8 #foreign physx_sys;

PxTetrahedronMesh_getTetrahedraRemap :: (self: *physx_PxTetrahedronMesh) -> *u32 #foreign physx_sys;

PxTetrahedronMesh_getLocalBounds :: (self: *physx_PxTetrahedronMesh) -> physx_PxBounds3 #foreign physx_sys;

PxTetrahedronMesh_release_mut :: (self: *physx_PxTetrahedronMesh) -> void #foreign physx_sys;

PxSoftBodyMesh_getCollisionMesh :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxSoftBodyMesh_getCollisionMesh_mut :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxSoftBodyMesh_getSimulationMesh :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxSoftBodyMesh_getSimulationMesh_mut :: (self: *physx_PxSoftBodyMesh) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxSoftBodyMesh_getSoftBodyAuxData :: (self: *physx_PxSoftBodyMesh) -> *physx_PxSoftBodyAuxData #foreign physx_sys;

PxSoftBodyMesh_getSoftBodyAuxData_mut :: (self: *physx_PxSoftBodyMesh) -> *physx_PxSoftBodyAuxData #foreign physx_sys;

PxSoftBodyMesh_release_mut :: (self: *physx_PxSoftBodyMesh) -> void #foreign physx_sys;

PxCollisionMeshMappingData_release_mut :: (self: *physx_PxCollisionMeshMappingData) -> void #foreign physx_sys;

PxCollisionTetrahedronMeshData_getMesh :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxTetrahedronMeshData #foreign physx_sys;

PxCollisionTetrahedronMeshData_getMesh_mut :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxTetrahedronMeshData #foreign physx_sys;

PxCollisionTetrahedronMeshData_getData :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxSoftBodyCollisionData #foreign physx_sys;

PxCollisionTetrahedronMeshData_getData_mut :: (self: *physx_PxCollisionTetrahedronMeshData) -> *physx_PxSoftBodyCollisionData #foreign physx_sys;

PxCollisionTetrahedronMeshData_release_mut :: (self: *physx_PxCollisionTetrahedronMeshData) -> void #foreign physx_sys;

PxSimulationTetrahedronMeshData_getMesh_mut :: (self: *physx_PxSimulationTetrahedronMeshData) -> *physx_PxTetrahedronMeshData #foreign physx_sys;

PxSimulationTetrahedronMeshData_getData_mut :: (self: *physx_PxSimulationTetrahedronMeshData) -> *physx_PxSoftBodySimulationData #foreign physx_sys;

PxSimulationTetrahedronMeshData_release_mut :: (self: *physx_PxSimulationTetrahedronMeshData) -> void #foreign physx_sys;

PxActor_release_mut :: (self: *physx_PxActor) -> void #foreign physx_sys;

PxActor_getType :: (self: *physx_PxActor) -> s32 #foreign physx_sys;

PxActor_getScene :: (self: *physx_PxActor) -> *physx_PxScene #foreign physx_sys;

PxActor_setName_mut :: (self: *physx_PxActor, name: *u8) -> void #foreign physx_sys;

PxActor_getName :: (self: *physx_PxActor) -> *u8 #foreign physx_sys;

PxActor_getWorldBounds :: (self: *physx_PxActor, inflation: float) -> physx_PxBounds3 #foreign physx_sys;

PxActor_setActorFlag_mut :: (self: *physx_PxActor, flag: s32, value: bool) -> void #foreign physx_sys;

PxActor_setActorFlags_mut :: (self: *physx_PxActor, inFlags: u8) -> void #foreign physx_sys;

PxActor_getActorFlags :: (self: *physx_PxActor) -> u8 #foreign physx_sys;

PxActor_setDominanceGroup_mut :: (self: *physx_PxActor, dominanceGroup: u8) -> void #foreign physx_sys;

PxActor_getDominanceGroup :: (self: *physx_PxActor) -> u8 #foreign physx_sys;

PxActor_setOwnerClient_mut :: (self: *physx_PxActor, inClient: u8) -> void #foreign physx_sys;

PxActor_getOwnerClient :: (self: *physx_PxActor) -> u8 #foreign physx_sys;

PxActor_getAggregate :: (self: *physx_PxActor) -> *physx_PxAggregate #foreign physx_sys;

phys_PxGetAggregateFilterHs32 :: (type: s32, enableSelfCollision: bool) -> u32 #foreign physx_sys;

phys_PxGetAggregateSelfCollisionBit :: (hs32: u32) -> u32 #foreign physx_sys;

phys_PxGetAggregateType :: (hs32: u32) -> s32 #foreign physx_sys;

PxAggregate_release_mut :: (self: *physx_PxAggregate) -> void #foreign physx_sys;

PxAggregate_addActor_mut :: (self: *physx_PxAggregate, actor: *physx_PxActor, bvh: *physx_PxBVH) -> bool #foreign physx_sys;

PxAggregate_removeActor_mut :: (self: *physx_PxAggregate, actor: *physx_PxActor) -> bool #foreign physx_sys;

PxAggregate_addArticulation_mut :: (self: *physx_PxAggregate, articulation: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys;

PxAggregate_removeArticulation_mut :: (self: *physx_PxAggregate, articulation: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys;

PxAggregate_getNbActors :: (self: *physx_PxAggregate) -> u32 #foreign physx_sys;

PxAggregate_getMaxNbShapes :: (self: *physx_PxAggregate) -> u32 #foreign physx_sys;

PxAggregate_getActors :: (self: *physx_PxAggregate, userBuffer: **physx_PxActor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxAggregate_getScene_mut :: (self: *physx_PxAggregate) -> *physx_PxScene #foreign physx_sys;

PxAggregate_getSelfCollision :: (self: *physx_PxAggregate) -> bool #foreign physx_sys;

PxAggregate_getConcreteTypeName :: (self: *physx_PxAggregate) -> *u8 #foreign physx_sys;

PxConstras32InvMassScale_new :: () -> physx_PxConstras32InvMassScale #foreign physx_sys;

PxConstras32InvMassScale_new_1 :: (lin0: float, ang0: float, lin1: float, ang1: float) -> physx_PxConstras32InvMassScale #foreign physx_sys;

PxConstras32Visualizer_visualizeJos32Frames_mut :: (self: *physx_PxConstras32Visualizer, parent: *physx_PxTransform, child: *physx_PxTransform) -> void #foreign physx_sys;

PxConstras32Visualizer_visualizeLinearLimit_mut :: (self: *physx_PxConstras32Visualizer, t0: *physx_PxTransform, t1: *physx_PxTransform, value: float, active: bool) -> void #foreign physx_sys;

PxConstras32Visualizer_visualizeAngularLimit_mut :: (self: *physx_PxConstras32Visualizer, t0: *physx_PxTransform, lower: float, upper: float, active: bool) -> void #foreign physx_sys;

PxConstras32Visualizer_visualizeLimitCone_mut :: (self: *physx_PxConstras32Visualizer, t: *physx_PxTransform, tanQSwingY: float, tanQSwingZ: float, active: bool) -> void #foreign physx_sys;

PxConstras32Visualizer_visualizeDoubleCone_mut :: (self: *physx_PxConstras32Visualizer, t: *physx_PxTransform, angle: float, active: bool) -> void #foreign physx_sys;

PxConstras32Visualizer_visualizeLine_mut :: (self: *physx_PxConstras32Visualizer, p0: *physx_Vector3, p1: *physx_Vector3, color: u32) -> void #foreign physx_sys;

PxConstras32Connector_prepareData_mut :: (self: *physx_PxConstras32Connector) -> *void #foreign physx_sys;

PxConstras32Connector_onConstras32Release_mut :: (self: *physx_PxConstras32Connector) -> void #foreign physx_sys;

PxConstras32Connector_onComShift_mut :: (self: *physx_PxConstras32Connector, actor: u32) -> void #foreign physx_sys;

PxConstras32Connector_onOriginShift_mut :: (self: *physx_PxConstras32Connector, shift: *physx_Vector3) -> void #foreign physx_sys;

PxConstras32Connector_getSerializable_mut :: (self: *physx_PxConstras32Connector) -> *physx_PxBase #foreign physx_sys;

PxConstras32Connector_getConstantBlock :: (self: *physx_PxConstras32Connector) -> *void #foreign physx_sys;

PxConstras32Connector_connectToConstras32_mut :: (self: *physx_PxConstras32Connector, anon_param0: *physx_PxConstras32) -> void #foreign physx_sys;

PxConstras32Connector_delete :: (self: *physx_PxConstras32Connector) -> void #foreign physx_sys;

PxSolverBody_new :: () -> physx_PxSolverBody #foreign physx_sys;

PxSolverBodyData_projectVelocity :: (self: *physx_PxSolverBodyData, lin: *physx_Vector3, ang: *physx_Vector3) -> float #foreign physx_sys;

PxSolverConstras32PrepDesc_delete :: (self: *physx_PxSolverConstras32PrepDesc) -> void #foreign physx_sys;

PxConstras32Allocator_reserveConstras32Data_mut :: (self: *physx_PxConstras32Allocator, byteSize: u32) -> *u8 #foreign physx_sys;

PxConstras32Allocator_reserveFrictionData_mut :: (self: *physx_PxConstras32Allocator, byteSize: u32) -> *u8 #foreign physx_sys;

PxConstras32Allocator_delete :: (self: *physx_PxConstras32Allocator) -> void #foreign physx_sys;

PxArticulationLimit_new :: () -> physx_PxArticulationLimit #foreign physx_sys;

PxArticulationLimit_new_1 :: (low_: float, high_: float) -> physx_PxArticulationLimit #foreign physx_sys;

PxArticulationDrive_new :: () -> physx_PxArticulationDrive #foreign physx_sys;

PxArticulationDrive_new_1 :: (stiffness_: float, damping_: float, maxForce_: float, driveType: s32) -> physx_PxArticulationDrive #foreign physx_sys;

PxTGSSolverBodyVel_projectVelocity :: (self: *physx_PxTGSSolverBodyVel, lin: *physx_Vector3, ang: *physx_Vector3) -> float #foreign physx_sys;

PxTGSSolverBodyData_projectVelocity :: (self: *physx_PxTGSSolverBodyData, linear: *physx_Vector3, angular: *physx_Vector3) -> float #foreign physx_sys;

PxTGSSolverConstras32PrepDesc_delete :: (self: *physx_PxTGSSolverConstras32PrepDesc) -> void #foreign physx_sys;

PxArticulationAttachment_setRestLength_mut :: (self: *physx_PxArticulationAttachment, restLength: float) -> void #foreign physx_sys;

PxArticulationAttachment_getRestLength :: (self: *physx_PxArticulationAttachment) -> float #foreign physx_sys;

PxArticulationAttachment_setLimitParameters_mut :: (self: *physx_PxArticulationAttachment, parameters: *physx_PxArticulationTendonLimit) -> void #foreign physx_sys;

PxArticulationAttachment_getLimitParameters :: (self: *physx_PxArticulationAttachment) -> physx_PxArticulationTendonLimit #foreign physx_sys;

PxArticulationAttachment_setRelativeOffset_mut :: (self: *physx_PxArticulationAttachment, offset: *physx_Vector3) -> void #foreign physx_sys;

PxArticulationAttachment_getRelativeOffset :: (self: *physx_PxArticulationAttachment) -> physx_Vector3 #foreign physx_sys;

PxArticulationAttachment_setCoefficient_mut :: (self: *physx_PxArticulationAttachment, coefficient: float) -> void #foreign physx_sys;

PxArticulationAttachment_getCoefficient :: (self: *physx_PxArticulationAttachment) -> float #foreign physx_sys;

PxArticulationAttachment_getLink :: (self: *physx_PxArticulationAttachment) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationAttachment_getParent :: (self: *physx_PxArticulationAttachment) -> *physx_PxArticulationAttachment #foreign physx_sys;

PxArticulationAttachment_isLeaf :: (self: *physx_PxArticulationAttachment) -> bool #foreign physx_sys;

PxArticulationAttachment_getTendon :: (self: *physx_PxArticulationAttachment) -> *physx_PxArticulationSpatialTendon #foreign physx_sys;

PxArticulationAttachment_release_mut :: (self: *physx_PxArticulationAttachment) -> void #foreign physx_sys;

PxArticulationAttachment_getConcreteTypeName :: (self: *physx_PxArticulationAttachment) -> *u8 #foreign physx_sys;

PxArticulationTendonJos32_setCoefficient_mut :: (self: *physx_PxArticulationTendonJos32, axis: s32, coefficient: float, recipCoefficient: float) -> void #foreign physx_sys;

PxArticulationTendonJos32_getCoefficient :: (self: *physx_PxArticulationTendonJos32, axis: *s32, coefficient: *float, recipCoefficient: *float) -> void #foreign physx_sys;

PxArticulationTendonJos32_getLink :: (self: *physx_PxArticulationTendonJos32) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationTendonJos32_getParent :: (self: *physx_PxArticulationTendonJos32) -> *physx_PxArticulationTendonJos32 #foreign physx_sys;

PxArticulationTendonJos32_getTendon :: (self: *physx_PxArticulationTendonJos32) -> *physx_PxArticulationFixedTendon #foreign physx_sys;

PxArticulationTendonJos32_release_mut :: (self: *physx_PxArticulationTendonJos32) -> void #foreign physx_sys;

PxArticulationTendonJos32_getConcreteTypeName :: (self: *physx_PxArticulationTendonJos32) -> *u8 #foreign physx_sys;

PxArticulationTendon_setStiffness_mut :: (self: *physx_PxArticulationTendon, stiffness: float) -> void #foreign physx_sys;

PxArticulationTendon_getStiffness :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_setDamping_mut :: (self: *physx_PxArticulationTendon, damping: float) -> void #foreign physx_sys;

PxArticulationTendon_getDamping :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_setLimitStiffness_mut :: (self: *physx_PxArticulationTendon, stiffness: float) -> void #foreign physx_sys;

PxArticulationTendon_getLimitStiffness :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_setOffset_mut :: (self: *physx_PxArticulationTendon, offset: float, autowake: bool) -> void #foreign physx_sys;

PxArticulationTendon_getOffset :: (self: *physx_PxArticulationTendon) -> float #foreign physx_sys;

PxArticulationTendon_getArticulation :: (self: *physx_PxArticulationTendon) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxArticulationTendon_release_mut :: (self: *physx_PxArticulationTendon) -> void #foreign physx_sys;

PxArticulationSpatialTendon_createAttachment_mut :: (self: *physx_PxArticulationSpatialTendon, parent: *physx_PxArticulationAttachment, coefficient: float, relativeOffset: physx_Vector3, link: *physx_PxArticulationLink) -> *physx_PxArticulationAttachment #foreign physx_sys;

PxArticulationSpatialTendon_getAttachments :: (self: *physx_PxArticulationSpatialTendon, userBuffer: **physx_PxArticulationAttachment, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationSpatialTendon_getNbAttachments :: (self: *physx_PxArticulationSpatialTendon) -> u32 #foreign physx_sys;

PxArticulationSpatialTendon_getConcreteTypeName :: (self: *physx_PxArticulationSpatialTendon) -> *u8 #foreign physx_sys;

PxArticulationFixedTendon_createTendonJos32_mut :: (self: *physx_PxArticulationFixedTendon, parent: *physx_PxArticulationTendonJos32, axis: s32, coefficient: float, recipCoefficient: float, link: *physx_PxArticulationLink) -> *physx_PxArticulationTendonJos32 #foreign physx_sys;

PxArticulationFixedTendon_getTendonJos32s :: (self: *physx_PxArticulationFixedTendon, userBuffer: **physx_PxArticulationTendonJos32, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationFixedTendon_getNbTendonJos32s :: (self: *physx_PxArticulationFixedTendon) -> u32 #foreign physx_sys;

PxArticulationFixedTendon_setRestLength_mut :: (self: *physx_PxArticulationFixedTendon, restLength: float) -> void #foreign physx_sys;

PxArticulationFixedTendon_getRestLength :: (self: *physx_PxArticulationFixedTendon) -> float #foreign physx_sys;

PxArticulationFixedTendon_setLimitParameters_mut :: (self: *physx_PxArticulationFixedTendon, parameter: *physx_PxArticulationTendonLimit) -> void #foreign physx_sys;

PxArticulationFixedTendon_getLimitParameters :: (self: *physx_PxArticulationFixedTendon) -> physx_PxArticulationTendonLimit #foreign physx_sys;

PxArticulationFixedTendon_getConcreteTypeName :: (self: *physx_PxArticulationFixedTendon) -> *u8 #foreign physx_sys;

PxArticulationCache_new :: () -> physx_PxArticulationCache #foreign physx_sys;

PxArticulationCache_release_mut :: (self: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationSensor_release_mut :: (self: *physx_PxArticulationSensor) -> void #foreign physx_sys;

PxArticulationSensor_getForces :: (self: *physx_PxArticulationSensor) -> physx_PxSpatialForce #foreign physx_sys;

PxArticulationSensor_getRelativePose :: (self: *physx_PxArticulationSensor) -> physx_PxTransform #foreign physx_sys;

PxArticulationSensor_setRelativePose_mut :: (self: *physx_PxArticulationSensor, pose: *physx_PxTransform) -> void #foreign physx_sys;

PxArticulationSensor_getLink :: (self: *physx_PxArticulationSensor) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationSensor_getIndex :: (self: *physx_PxArticulationSensor) -> u32 #foreign physx_sys;

PxArticulationSensor_getArticulation :: (self: *physx_PxArticulationSensor) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxArticulationSensor_getFlags :: (self: *physx_PxArticulationSensor) -> u8 #foreign physx_sys;

PxArticulationSensor_setFlag_mut :: (self: *physx_PxArticulationSensor, flag: s32, enabled: bool) -> void #foreign physx_sys;

PxArticulationSensor_getConcreteTypeName :: (self: *physx_PxArticulationSensor) -> *u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getScene :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxScene #foreign physx_sys;

PxArticulationReducedCoordinate_setSolverIterationCounts_mut :: (self: *physx_PxArticulationReducedCoordinate, minPositionIters: u32, minVelocityIters: u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getSolverIterationCounts :: (self: *physx_PxArticulationReducedCoordinate, minPositionIters: *u32, minVelocityIters: *u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_isSleeping :: (self: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys;

PxArticulationReducedCoordinate_setSleepThreshold_mut :: (self: *physx_PxArticulationReducedCoordinate, threshold: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getSleepThreshold :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setStabilizationThreshold_mut :: (self: *physx_PxArticulationReducedCoordinate, threshold: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getStabilizationThreshold :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setWakeCounter_mut :: (self: *physx_PxArticulationReducedCoordinate, wakeCounterValue: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getWakeCounter :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_wakeUp_mut :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_putToSleep_mut :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut :: (self: *physx_PxArticulationReducedCoordinate, maxLinearVelocity: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getMaxCOMLinearVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut :: (self: *physx_PxArticulationReducedCoordinate, maxAngularVelocity: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getMaxCOMAngularVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_createLink_mut :: (self: *physx_PxArticulationReducedCoordinate, parent: *physx_PxArticulationLink, pose: *physx_PxTransform) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationReducedCoordinate_release_mut :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getNbLinks :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getLinks :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbShapes :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_setName_mut :: (self: *physx_PxArticulationReducedCoordinate, name: *u8) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getName :: (self: *physx_PxArticulationReducedCoordinate) -> *u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getWorldBounds :: (self: *physx_PxArticulationReducedCoordinate, inflation: float) -> physx_PxBounds3 #foreign physx_sys;

PxArticulationReducedCoordinate_getAggregate :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxAggregate #foreign physx_sys;

PxArticulationReducedCoordinate_setArticulationFlags_mut :: (self: *physx_PxArticulationReducedCoordinate, flags: u8) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_setArticulationFlag_mut :: (self: *physx_PxArticulationReducedCoordinate, flag: s32, value: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getArticulationFlags :: (self: *physx_PxArticulationReducedCoordinate) -> u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getDofs :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_createCache :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxArticulationCache #foreign physx_sys;

PxArticulationReducedCoordinate_getCacheDataSize :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_zeroCache :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_applyCache_mut :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, flags: u32, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_copyInternalStateToCache :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, flags: u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_packJos32Data :: (self: *physx_PxArticulationReducedCoordinate, maximum: *float, reduced: *float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_unpackJos32Data :: (self: *physx_PxArticulationReducedCoordinate, reduced: *float, maximum: *float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_commonInit :: (self: *physx_PxArticulationReducedCoordinate) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedGravityForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedExternalForce :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeJos32Acceleration :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeJos32Force :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeDenseJacobian :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, nRows: *u32, nCols: *u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeCoefficientMatrix :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeLambda :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache, initialState: *physx_PxArticulationCache, jos32Torque: *float, maxIter: u32) -> bool #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedMassMatrix :: (self: *physx_PxArticulationReducedCoordinate, cache: *physx_PxArticulationCache) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_addLoopJos32_mut :: (self: *physx_PxArticulationReducedCoordinate, jos32: *physx_PxConstras32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_removeLoopJos32_mut :: (self: *physx_PxArticulationReducedCoordinate, jos32: *physx_PxConstras32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getNbLoopJos32s :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getLoopJos32s :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxConstras32, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getCoefficientMatrixSize :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_setRootGlobalPose_mut :: (self: *physx_PxArticulationReducedCoordinate, pose: *physx_PxTransform, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getRootGlobalPose :: (self: *physx_PxArticulationReducedCoordinate) -> physx_PxTransform #foreign physx_sys;

PxArticulationReducedCoordinate_setRootLinearVelocity_mut :: (self: *physx_PxArticulationReducedCoordinate, linearVelocity: *physx_Vector3, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getRootLinearVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> physx_Vector3 #foreign physx_sys;

PxArticulationReducedCoordinate_setRootAngularVelocity_mut :: (self: *physx_PxArticulationReducedCoordinate, angularVelocity: *physx_Vector3, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getRootAngularVelocity :: (self: *physx_PxArticulationReducedCoordinate) -> physx_Vector3 #foreign physx_sys;

PxArticulationReducedCoordinate_getLinkAcceleration_mut :: (self: *physx_PxArticulationReducedCoordinate, linkId: u32) -> physx_PxSpatialVelocity #foreign physx_sys;

PxArticulationReducedCoordinate_getGpuArticulationIndex_mut :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_createSpatialTendon_mut :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxArticulationSpatialTendon #foreign physx_sys;

PxArticulationReducedCoordinate_createFixedTendon_mut :: (self: *physx_PxArticulationReducedCoordinate) -> *physx_PxArticulationFixedTendon #foreign physx_sys;

PxArticulationReducedCoordinate_createSensor_mut :: (self: *physx_PxArticulationReducedCoordinate, link: *physx_PxArticulationLink, relativePose: *physx_PxTransform) -> *physx_PxArticulationSensor #foreign physx_sys;

PxArticulationReducedCoordinate_getSpatialTendons :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationSpatialTendon, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbSpatialTendons_mut :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getFixedTendons :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationFixedTendon, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbFixedTendons_mut :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getSensors :: (self: *physx_PxArticulationReducedCoordinate, userBuffer: **physx_PxArticulationSensor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbSensors_mut :: (self: *physx_PxArticulationReducedCoordinate) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_updateKinematic_mut :: (self: *physx_PxArticulationReducedCoordinate, flags: u8) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getParentArticulationLink :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setParentPose_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, pose: *physx_PxTransform) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getParentPose :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> physx_PxTransform #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getChildArticulationLink :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> *physx_PxArticulationLink #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setChildPose_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, pose: *physx_PxTransform) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getChildPose :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> physx_PxTransform #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setJos32Type_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, jos32Type: s32) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getJos32Type :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> s32 #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setMotion_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, motion: s32) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getMotion :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> s32 #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setLimitParams_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, limit: *physx_PxArticulationLimit) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getLimitParams :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> physx_PxArticulationLimit #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setDriveParams_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, drive: *physx_PxArticulationDrive) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getDriveParams :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> physx_PxArticulationDrive #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setDriveTarget_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, target: float, autowake: bool) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getDriveTarget :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setDriveVelocity_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, targetVel: float, autowake: bool) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getDriveVelocity :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setArmature_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, armature: float) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getArmature :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setFrictionCoefficient_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, coefficient: float) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getFrictionCoefficient :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setMaxJos32Velocity_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, maxJos32V: float) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getMaxJos32Velocity :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setJos32Position_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, jos32Pos: float) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getJos32Position :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_setJos32Velocity_mut :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32, jos32Vel: float) -> void #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getJos32Velocity :: (self: *physx_PxArticulationJos32ReducedCoordinate, axis: s32) -> float #foreign physx_sys;

PxArticulationJos32ReducedCoordinate_getConcreteTypeName :: (self: *physx_PxArticulationJos32ReducedCoordinate) -> *u8 #foreign physx_sys;

PxShape_release_mut :: (self: *physx_PxShape) -> void #foreign physx_sys;

PxShape_setGeometry_mut :: (self: *physx_PxShape, geometry: *physx_PxGeometry) -> void #foreign physx_sys;

PxShape_getGeometry :: (self: *physx_PxShape) -> *physx_PxGeometry #foreign physx_sys;

PxShape_getActor :: (self: *physx_PxShape) -> *physx_PxRigidActor #foreign physx_sys;

PxShape_setLocalPose_mut :: (self: *physx_PxShape, pose: *physx_PxTransform) -> void #foreign physx_sys;

PxShape_getLocalPose :: (self: *physx_PxShape) -> physx_PxTransform #foreign physx_sys;

PxShape_setSimulationFilterData_mut :: (self: *physx_PxShape, data: *physx_PxFilterData) -> void #foreign physx_sys;

PxShape_getSimulationFilterData :: (self: *physx_PxShape) -> physx_PxFilterData #foreign physx_sys;

PxShape_setQueryFilterData_mut :: (self: *physx_PxShape, data: *physx_PxFilterData) -> void #foreign physx_sys;

PxShape_getQueryFilterData :: (self: *physx_PxShape) -> physx_PxFilterData #foreign physx_sys;

PxShape_setMaterials_mut :: (self: *physx_PxShape, materials: **physx_PxMaterial, materialCount: u16) -> void #foreign physx_sys;

PxShape_getNbMaterials :: (self: *physx_PxShape) -> u16 #foreign physx_sys;

PxShape_getMaterials :: (self: *physx_PxShape, userBuffer: **physx_PxMaterial, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxShape_getMaterialFromInternalFaceIndex :: (self: *physx_PxShape, faceIndex: u32) -> *physx_PxBaseMaterial #foreign physx_sys;

PxShape_setContactOffset_mut :: (self: *physx_PxShape, contactOffset: float) -> void #foreign physx_sys;

PxShape_getContactOffset :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setRestOffset_mut :: (self: *physx_PxShape, restOffset: float) -> void #foreign physx_sys;

PxShape_getRestOffset :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setDensityForFluid_mut :: (self: *physx_PxShape, densityForFluid: float) -> void #foreign physx_sys;

PxShape_getDensityForFluid :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setTorsionalPatchRadius_mut :: (self: *physx_PxShape, radius: float) -> void #foreign physx_sys;

PxShape_getTorsionalPatchRadius :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setMinTorsionalPatchRadius_mut :: (self: *physx_PxShape, radius: float) -> void #foreign physx_sys;

PxShape_getMinTorsionalPatchRadius :: (self: *physx_PxShape) -> float #foreign physx_sys;

PxShape_setFlag_mut :: (self: *physx_PxShape, flag: s32, value: bool) -> void #foreign physx_sys;

PxShape_setFlags_mut :: (self: *physx_PxShape, inFlags: u8) -> void #foreign physx_sys;

PxShape_getFlags :: (self: *physx_PxShape) -> u8 #foreign physx_sys;

PxShape_isExclusive :: (self: *physx_PxShape) -> bool #foreign physx_sys;

PxShape_setName_mut :: (self: *physx_PxShape, name: *u8) -> void #foreign physx_sys;

PxShape_getName :: (self: *physx_PxShape) -> *u8 #foreign physx_sys;

PxShape_getConcreteTypeName :: (self: *physx_PxShape) -> *u8 #foreign physx_sys;

PxRigidActor_release_mut :: (self: *physx_PxRigidActor) -> void #foreign physx_sys;

PxRigidActor_getInternalActorIndex :: (self: *physx_PxRigidActor) -> u32 #foreign physx_sys;

PxRigidActor_getGlobalPose :: (self: *physx_PxRigidActor) -> physx_PxTransform #foreign physx_sys;

PxRigidActor_setGlobalPose_mut :: (self: *physx_PxRigidActor, pose: *physx_PxTransform, autowake: bool) -> void #foreign physx_sys;

PxRigidActor_attachShape_mut :: (self: *physx_PxRigidActor, shape: *physx_PxShape) -> bool #foreign physx_sys;

PxRigidActor_detachShape_mut :: (self: *physx_PxRigidActor, shape: *physx_PxShape, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxRigidActor_getNbShapes :: (self: *physx_PxRigidActor) -> u32 #foreign physx_sys;

PxRigidActor_getShapes :: (self: *physx_PxRigidActor, userBuffer: **physx_PxShape, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxRigidActor_getNbConstras32s :: (self: *physx_PxRigidActor) -> u32 #foreign physx_sys;

PxRigidActor_getConstras32s :: (self: *physx_PxRigidActor, userBuffer: **physx_PxConstras32, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxNodeIndex_new :: (id: u32, articLinkId: u32) -> physx_PxNodeIndex #foreign physx_sys;

PxNodeIndex_new_1 :: (id: u32) -> physx_PxNodeIndex #foreign physx_sys;

PxNodeIndex_index :: (self: *physx_PxNodeIndex) -> u32 #foreign physx_sys;

PxNodeIndex_articulationLinkId :: (self: *physx_PxNodeIndex) -> u32 #foreign physx_sys;

PxNodeIndex_isArticulation :: (self: *physx_PxNodeIndex) -> u32 #foreign physx_sys;

PxNodeIndex_isStaticBody :: (self: *physx_PxNodeIndex) -> bool #foreign physx_sys;

PxNodeIndex_isValid :: (self: *physx_PxNodeIndex) -> bool #foreign physx_sys;

PxNodeIndex_setIndices_mut :: (self: *physx_PxNodeIndex, index: u32, articLinkId: u32) -> void #foreign physx_sys;

PxNodeIndex_setIndices_mut_1 :: (self: *physx_PxNodeIndex, index: u32) -> void #foreign physx_sys;

PxNodeIndex_getInd :: (self: *physx_PxNodeIndex) -> u64 #foreign physx_sys;

PxRigidBody_setCMassLocalPose_mut :: (self: *physx_PxRigidBody, pose: *physx_PxTransform) -> void #foreign physx_sys;

PxRigidBody_getCMassLocalPose :: (self: *physx_PxRigidBody) -> physx_PxTransform #foreign physx_sys;

PxRigidBody_setMass_mut :: (self: *physx_PxRigidBody, mass: float) -> void #foreign physx_sys;

PxRigidBody_getMass :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_getInvMass :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMassSpaceInertiaTensor_mut :: (self: *physx_PxRigidBody, m: *physx_Vector3) -> void #foreign physx_sys;

PxRigidBody_getMassSpaceInertiaTensor :: (self: *physx_PxRigidBody) -> physx_Vector3 #foreign physx_sys;

PxRigidBody_getMassSpaceInvInertiaTensor :: (self: *physx_PxRigidBody) -> physx_Vector3 #foreign physx_sys;

PxRigidBody_setLinearDamping_mut :: (self: *physx_PxRigidBody, linDamp: float) -> void #foreign physx_sys;

PxRigidBody_getLinearDamping :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setAngularDamping_mut :: (self: *physx_PxRigidBody, angDamp: float) -> void #foreign physx_sys;

PxRigidBody_getAngularDamping :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_getLinearVelocity :: (self: *physx_PxRigidBody) -> physx_Vector3 #foreign physx_sys;

PxRigidBody_getAngularVelocity :: (self: *physx_PxRigidBody) -> physx_Vector3 #foreign physx_sys;

PxRigidBody_setMaxLinearVelocity_mut :: (self: *physx_PxRigidBody, maxLinVel: float) -> void #foreign physx_sys;

PxRigidBody_getMaxLinearVelocity :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMaxAngularVelocity_mut :: (self: *physx_PxRigidBody, maxAngVel: float) -> void #foreign physx_sys;

PxRigidBody_getMaxAngularVelocity :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_addForce_mut :: (self: *physx_PxRigidBody, force: *physx_Vector3, mode: s32, autowake: bool) -> void #foreign physx_sys;

PxRigidBody_addTorque_mut :: (self: *physx_PxRigidBody, torque: *physx_Vector3, mode: s32, autowake: bool) -> void #foreign physx_sys;

PxRigidBody_clearForce_mut :: (self: *physx_PxRigidBody, mode: s32) -> void #foreign physx_sys;

PxRigidBody_clearTorque_mut :: (self: *physx_PxRigidBody, mode: s32) -> void #foreign physx_sys;

PxRigidBody_setForceAndTorque_mut :: (self: *physx_PxRigidBody, force: *physx_Vector3, torque: *physx_Vector3, mode: s32) -> void #foreign physx_sys;

PxRigidBody_setRigidBodyFlag_mut :: (self: *physx_PxRigidBody, flag: s32, value: bool) -> void #foreign physx_sys;

PxRigidBody_setRigidBodyFlags_mut :: (self: *physx_PxRigidBody, inFlags: u16) -> void #foreign physx_sys;

PxRigidBody_getRigidBodyFlags :: (self: *physx_PxRigidBody) -> u16 #foreign physx_sys;

PxRigidBody_setMinCCDAdvanceCoefficient_mut :: (self: *physx_PxRigidBody, advanceCoefficient: float) -> void #foreign physx_sys;

PxRigidBody_getMinCCDAdvanceCoefficient :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMaxDepenetrationVelocity_mut :: (self: *physx_PxRigidBody, biasClamp: float) -> void #foreign physx_sys;

PxRigidBody_getMaxDepenetrationVelocity :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setMaxContactImpulse_mut :: (self: *physx_PxRigidBody, maxImpulse: float) -> void #foreign physx_sys;

PxRigidBody_getMaxContactImpulse :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_setContactSlopCoefficient_mut :: (self: *physx_PxRigidBody, slopCoefficient: float) -> void #foreign physx_sys;

PxRigidBody_getContactSlopCoefficient :: (self: *physx_PxRigidBody) -> float #foreign physx_sys;

PxRigidBody_getInternalIslandNodeIndex :: (self: *physx_PxRigidBody) -> physx_PxNodeIndex #foreign physx_sys;

PxArticulationLink_release_mut :: (self: *physx_PxArticulationLink) -> void #foreign physx_sys;

PxArticulationLink_getArticulation :: (self: *physx_PxArticulationLink) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxArticulationLink_getInboundJos32 :: (self: *physx_PxArticulationLink) -> *physx_PxArticulationJos32ReducedCoordinate #foreign physx_sys;

PxArticulationLink_getInboundJos32Dof :: (self: *physx_PxArticulationLink) -> u32 #foreign physx_sys;

PxArticulationLink_getNbChildren :: (self: *physx_PxArticulationLink) -> u32 #foreign physx_sys;

PxArticulationLink_getLinkIndex :: (self: *physx_PxArticulationLink) -> u32 #foreign physx_sys;

PxArticulationLink_getChildren :: (self: *physx_PxArticulationLink, userBuffer: **physx_PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationLink_setCfmScale_mut :: (self: *physx_PxArticulationLink, cfm: float) -> void #foreign physx_sys;

PxArticulationLink_getCfmScale :: (self: *physx_PxArticulationLink) -> float #foreign physx_sys;

PxArticulationLink_getLinearVelocity :: (self: *physx_PxArticulationLink) -> physx_Vector3 #foreign physx_sys;

PxArticulationLink_getAngularVelocity :: (self: *physx_PxArticulationLink) -> physx_Vector3 #foreign physx_sys;

PxArticulationLink_getConcreteTypeName :: (self: *physx_PxArticulationLink) -> *u8 #foreign physx_sys;

PxConeLimitedConstras32_new :: () -> physx_PxConeLimitedConstras32 #foreign physx_sys;

PxConstras32_release_mut :: (self: *physx_PxConstras32) -> void #foreign physx_sys;

PxConstras32_getScene :: (self: *physx_PxConstras32) -> *physx_PxScene #foreign physx_sys;

PxConstras32_getActors :: (self: *physx_PxConstras32, actor0: **physx_PxRigidActor, actor1: **physx_PxRigidActor) -> void #foreign physx_sys;

PxConstras32_setActors_mut :: (self: *physx_PxConstras32, actor0: *physx_PxRigidActor, actor1: *physx_PxRigidActor) -> void #foreign physx_sys;

PxConstras32_markDirty_mut :: (self: *physx_PxConstras32) -> void #foreign physx_sys;

PxConstras32_getFlags :: (self: *physx_PxConstras32) -> u16 #foreign physx_sys;

PxConstras32_setFlags_mut :: (self: *physx_PxConstras32, flags: u16) -> void #foreign physx_sys;

PxConstras32_setFlag_mut :: (self: *physx_PxConstras32, flag: s32, value: bool) -> void #foreign physx_sys;

PxConstras32_getForce :: (self: *physx_PxConstras32, linear: *physx_Vector3, angular: *physx_Vector3) -> void #foreign physx_sys;

PxConstras32_isValid :: (self: *physx_PxConstras32) -> bool #foreign physx_sys;

PxConstras32_setBreakForce_mut :: (self: *physx_PxConstras32, linear: float, angular: float) -> void #foreign physx_sys;

PxConstras32_getBreakForce :: (self: *physx_PxConstras32, linear: *float, angular: *float) -> void #foreign physx_sys;

PxConstras32_setMinResponseThreshold_mut :: (self: *physx_PxConstras32, threshold: float) -> void #foreign physx_sys;

PxConstras32_getMinResponseThreshold :: (self: *physx_PxConstras32) -> float #foreign physx_sys;

PxConstras32_getExternalReference_mut :: (self: *physx_PxConstras32, typeID: *u32) -> *void #foreign physx_sys;

PxConstras32_setConstras32Functions_mut :: (self: *physx_PxConstras32, connector: *physx_PxConstras32Connector, shaders: *physx_PxConstras32ShaderTable) -> void #foreign physx_sys;

PxConstras32_getConcreteTypeName :: (self: *physx_PxConstras32) -> *u8 #foreign physx_sys;

PxContactStreamIterator_new :: (contactPatches: *u8, contactPos32s: *u8, contactFaceIndices: *u32, nbPatches: u32, nbContacts: u32) -> physx_PxContactStreamIterator #foreign physx_sys;

PxContactStreamIterator_hasNextPatch :: (self: *physx_PxContactStreamIterator) -> bool #foreign physx_sys;

PxContactStreamIterator_getTotalContactCount :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_getTotalPatchCount :: (self: *physx_PxContactStreamIterator) -> u32 #foreign physx_sys;

PxContactStreamIterator_nextPatch_mut :: (self: *physx_PxContactStreamIterator) -> void #foreign physx_sys;

PxContactStreamIterator_hasNextContact :: (self: *physx_PxContactStreamIterator) -> bool #foreign physx_sys;

PxContactStreamIterator_nextContact_mut :: (self: *physx_PxContactStreamIterator) -> void #foreign physx_sys;

PxContactStreamIterator_getContactNormal :: (self: *physx_PxContactStreamIterator) -> *physx_Vector3 #foreign physx_sys;

PxContactStreamIterator_getInvMassScale0 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getInvMassScale1 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getInvInertiaScale0 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getInvInertiaScale1 :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getMaxImpulse :: (self: *physx_PxContactStreamIterator) -> float #foreign physx_sys;

PxContactStreamIterator_getTargetVel :: (self: *physx_PxContactStreamIterator) -> *physx_Vector3 #foreign physx_sys;

PxContactStreamIterator_getContactPos32 :: (self: *physx_PxContactStreamIterator) -> *physx_Vector3 #foreign physx_sys;

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

PxContactStreamIterator_advanceToIndex_mut :: (self: *physx_PxContactStreamIterator, initialIndex: u32) -> bool #foreign physx_sys;

PxContactSet_getPos32 :: (self: *physx_PxContactSet, i: u32) -> *physx_Vector3 #foreign physx_sys;

PxContactSet_setPos32_mut :: (self: *physx_PxContactSet, i: u32, p: *physx_Vector3) -> void #foreign physx_sys;

PxContactSet_getNormal :: (self: *physx_PxContactSet, i: u32) -> *physx_Vector3 #foreign physx_sys;

PxContactSet_setNormal_mut :: (self: *physx_PxContactSet, i: u32, n: *physx_Vector3) -> void #foreign physx_sys;

PxContactSet_getSeparation :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setSeparation_mut :: (self: *physx_PxContactSet, i: u32, s: float) -> void #foreign physx_sys;

PxContactSet_getTargetVelocity :: (self: *physx_PxContactSet, i: u32) -> *physx_Vector3 #foreign physx_sys;

PxContactSet_setTargetVelocity_mut :: (self: *physx_PxContactSet, i: u32, v: *physx_Vector3) -> void #foreign physx_sys;

PxContactSet_getInternalFaceIndex0 :: (self: *physx_PxContactSet, i: u32) -> u32 #foreign physx_sys;

PxContactSet_getInternalFaceIndex1 :: (self: *physx_PxContactSet, i: u32) -> u32 #foreign physx_sys;

PxContactSet_getMaxImpulse :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setMaxImpulse_mut :: (self: *physx_PxContactSet, i: u32, s: float) -> void #foreign physx_sys;

PxContactSet_getRestitution :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setRestitution_mut :: (self: *physx_PxContactSet, i: u32, r: float) -> void #foreign physx_sys;

PxContactSet_getStaticFriction :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setStaticFriction_mut :: (self: *physx_PxContactSet, i: u32, f: float) -> void #foreign physx_sys;

PxContactSet_getDynamicFriction :: (self: *physx_PxContactSet, i: u32) -> float #foreign physx_sys;

PxContactSet_setDynamicFriction_mut :: (self: *physx_PxContactSet, i: u32, f: float) -> void #foreign physx_sys;

PxContactSet_ignore_mut :: (self: *physx_PxContactSet, i: u32) -> void #foreign physx_sys;

PxContactSet_size :: (self: *physx_PxContactSet) -> u32 #foreign physx_sys;

PxContactSet_getInvMassScale0 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_getInvMassScale1 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_getInvInertiaScale0 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_getInvInertiaScale1 :: (self: *physx_PxContactSet) -> float #foreign physx_sys;

PxContactSet_setInvMassScale0_mut :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys;

PxContactSet_setInvMassScale1_mut :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys;

PxContactSet_setInvInertiaScale0_mut :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys;

PxContactSet_setInvInertiaScale1_mut :: (self: *physx_PxContactSet, scale: float) -> void #foreign physx_sys;

PxContactModifyCallback_onContactModify_mut :: (self: *physx_PxContactModifyCallback, pairs: *physx_PxContactModifyPair, count: u32) -> void #foreign physx_sys;

PxCCDContactModifyCallback_onCCDContactModify_mut :: (self: *physx_PxCCDContactModifyCallback, pairs: *physx_PxContactModifyPair, count: u32) -> void #foreign physx_sys;

PxDeletionListener_onRelease_mut :: (self: *physx_PxDeletionListener, observed: *physx_PxBase, userData: *void, deletionEvent: s32) -> void #foreign physx_sys;

PxBaseMaterial_isKindOf :: (self: *physx_PxBaseMaterial, name: *u8) -> bool #foreign physx_sys;

PxFEMMaterial_setYoungsModulus_mut :: (self: *physx_PxFEMMaterial, young: float) -> void #foreign physx_sys;

PxFEMMaterial_getYoungsModulus :: (self: *physx_PxFEMMaterial) -> float #foreign physx_sys;

PxFEMMaterial_setPoissons_mut :: (self: *physx_PxFEMMaterial, poisson: float) -> void #foreign physx_sys;

PxFEMMaterial_getPoissons :: (self: *physx_PxFEMMaterial) -> float #foreign physx_sys;

PxFEMMaterial_setDynamicFriction_mut :: (self: *physx_PxFEMMaterial, dynamicFriction: float) -> void #foreign physx_sys;

PxFEMMaterial_getDynamicFriction :: (self: *physx_PxFEMMaterial) -> float #foreign physx_sys;

PxFilterData_new :: (anon_param0: s32) -> physx_PxFilterData #foreign physx_sys;

PxFilterData_new_1 :: () -> physx_PxFilterData #foreign physx_sys;

PxFilterData_new_2 :: (w0: u32, w1: u32, w2: u32, w3: u32) -> physx_PxFilterData #foreign physx_sys;

PxFilterData_setToDefault_mut :: (self: *physx_PxFilterData) -> void #foreign physx_sys;

phys_PxGetFilterObjectType :: (attr: u32) -> s32 #foreign physx_sys;

phys_PxFilterObjectIsKinematic :: (attr: u32) -> bool #foreign physx_sys;

phys_PxFilterObjectIsTrigger :: (attr: u32) -> bool #foreign physx_sys;

PxSimulationFilterCallback_pairFound_mut :: (self: *physx_PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: physx_PxFilterData, a0: *physx_PxActor, s0: *physx_PxShape, attributes1: u32, filterData1: physx_PxFilterData, a1: *physx_PxActor, s1: *physx_PxShape, pairFlags: *u16) -> u16 #foreign physx_sys;

PxSimulationFilterCallback_pairLost_mut :: (self: *physx_PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: physx_PxFilterData, attributes1: u32, filterData1: physx_PxFilterData, objectRemoved: bool) -> void #foreign physx_sys;

PxSimulationFilterCallback_statusChange_mut :: (self: *physx_PxSimulationFilterCallback, pairID: *u32, pairFlags: *u16, filterFlags: *u16) -> bool #foreign physx_sys;

PxLockedData_getDataAccessFlags_mut :: (self: *physx_PxLockedData) -> u8 #foreign physx_sys;

PxLockedData_unlock_mut :: (self: *physx_PxLockedData) -> void #foreign physx_sys;

PxLockedData_delete :: (self: *physx_PxLockedData) -> void #foreign physx_sys;

PxMaterial_setDynamicFriction_mut :: (self: *physx_PxMaterial, coef: float) -> void #foreign physx_sys;

PxMaterial_getDynamicFriction :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setStaticFriction_mut :: (self: *physx_PxMaterial, coef: float) -> void #foreign physx_sys;

PxMaterial_getStaticFriction :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setRestitution_mut :: (self: *physx_PxMaterial, rest: float) -> void #foreign physx_sys;

PxMaterial_getRestitution :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setDamping_mut :: (self: *physx_PxMaterial, damping: float) -> void #foreign physx_sys;

PxMaterial_getDamping :: (self: *physx_PxMaterial) -> float #foreign physx_sys;

PxMaterial_setFlag_mut :: (self: *physx_PxMaterial, flag: s32, b: bool) -> void #foreign physx_sys;

PxMaterial_setFlags_mut :: (self: *physx_PxMaterial, flags: u16) -> void #foreign physx_sys;

PxMaterial_getFlags :: (self: *physx_PxMaterial) -> u16 #foreign physx_sys;

PxMaterial_setFrictionCombineMode_mut :: (self: *physx_PxMaterial, combMode: s32) -> void #foreign physx_sys;

PxMaterial_getFrictionCombineMode :: (self: *physx_PxMaterial) -> s32 #foreign physx_sys;

PxMaterial_setRestitutionCombineMode_mut :: (self: *physx_PxMaterial, combMode: s32) -> void #foreign physx_sys;

PxMaterial_getRestitutionCombineMode :: (self: *physx_PxMaterial) -> s32 #foreign physx_sys;

PxMaterial_getConcreteTypeName :: (self: *physx_PxMaterial) -> *u8 #foreign physx_sys;

PxDiffuseParticleParams_new :: () -> physx_PxDiffuseParticleParams #foreign physx_sys;

PxDiffuseParticleParams_setToDefault_mut :: (self: *physx_PxDiffuseParticleParams) -> void #foreign physx_sys;

PxParticleMaterial_setFriction_mut :: (self: *physx_PxParticleMaterial, friction: float) -> void #foreign physx_sys;

PxParticleMaterial_getFriction :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setDamping_mut :: (self: *physx_PxParticleMaterial, damping: float) -> void #foreign physx_sys;

PxParticleMaterial_getDamping :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setAdhesion_mut :: (self: *physx_PxParticleMaterial, adhesion: float) -> void #foreign physx_sys;

PxParticleMaterial_getAdhesion :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setGravityScale_mut :: (self: *physx_PxParticleMaterial, scale: float) -> void #foreign physx_sys;

PxParticleMaterial_getGravityScale :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxParticleMaterial_setAdhesionRadiusScale_mut :: (self: *physx_PxParticleMaterial, scale: float) -> void #foreign physx_sys;

PxParticleMaterial_getAdhesionRadiusScale :: (self: *physx_PxParticleMaterial) -> float #foreign physx_sys;

PxPhysics_release_mut :: (self: *physx_PxPhysics) -> void #foreign physx_sys;

PxPhysics_getFoundation_mut :: (self: *physx_PxPhysics) -> *physx_PxFoundation #foreign physx_sys;

PxPhysics_createAggregate_mut :: (self: *physx_PxPhysics, maxActor: u32, maxShape: u32, filterHs32: u32) -> *physx_PxAggregate #foreign physx_sys;

PxPhysics_getTolerancesScale :: (self: *physx_PxPhysics) -> *physx_PxTolerancesScale #foreign physx_sys;

PxPhysics_createTriangleMesh_mut :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxTriangleMesh #foreign physx_sys;

PxPhysics_getNbTriangleMeshes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getTriangleMeshes :: (self: *physx_PxPhysics, userBuffer: **physx_PxTriangleMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createTetrahedronMesh_mut :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxTetrahedronMesh #foreign physx_sys;

PxPhysics_createSoftBodyMesh_mut :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxSoftBodyMesh #foreign physx_sys;

PxPhysics_getNbTetrahedronMeshes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getTetrahedronMeshes :: (self: *physx_PxPhysics, userBuffer: **physx_PxTetrahedronMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createHeightField_mut :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxHeightField #foreign physx_sys;

PxPhysics_getNbHeightFields :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getHeightFields :: (self: *physx_PxPhysics, userBuffer: **physx_PxHeightField, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createConvexMesh_mut :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxConvexMesh #foreign physx_sys;

PxPhysics_getNbConvexMeshes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getConvexMeshes :: (self: *physx_PxPhysics, userBuffer: **physx_PxConvexMesh, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createBVH_mut :: (self: *physx_PxPhysics, stream: *physx_PxInputStream) -> *physx_PxBVH #foreign physx_sys;

PxPhysics_getNbBVHs :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getBVHs :: (self: *physx_PxPhysics, userBuffer: **physx_PxBVH, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createScene_mut :: (self: *physx_PxPhysics, sceneDesc: *physx_PxSceneDesc) -> *physx_PxScene #foreign physx_sys;

PxPhysics_getNbScenes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getScenes :: (self: *physx_PxPhysics, userBuffer: **physx_PxScene, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createRigidStatic_mut :: (self: *physx_PxPhysics, pose: *physx_PxTransform) -> *physx_PxRigidStatic #foreign physx_sys;

PxPhysics_createRigidDynamic_mut :: (self: *physx_PxPhysics, pose: *physx_PxTransform) -> *physx_PxRigidDynamic #foreign physx_sys;

PxPhysics_createPruningStructure_mut :: (self: *physx_PxPhysics, actors: **physx_PxRigidActor, nbActors: u32) -> *physx_PxPruningStructure #foreign physx_sys;

PxPhysics_createShape_mut :: (self: *physx_PxPhysics, geometry: *physx_PxGeometry, material: *physx_PxMaterial, isExclusive: bool, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys;

PxPhysics_createShape_mut_1 :: (self: *physx_PxPhysics, geometry: *physx_PxGeometry, materials: **physx_PxMaterial, materialCount: u16, isExclusive: bool, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys;

PxPhysics_getNbShapes :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getShapes :: (self: *physx_PxPhysics, userBuffer: **physx_PxShape, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createConstras32_mut :: (self: *physx_PxPhysics, actor0: *physx_PxRigidActor, actor1: *physx_PxRigidActor, connector: *physx_PxConstras32Connector, shaders: *physx_PxConstras32ShaderTable, dataSize: u32) -> *physx_PxConstras32 #foreign physx_sys;

PxPhysics_createArticulationReducedCoordinate_mut :: (self: *physx_PxPhysics) -> *physx_PxArticulationReducedCoordinate #foreign physx_sys;

PxPhysics_createMaterial_mut :: (self: *physx_PxPhysics, staticFriction: float, dynamicFriction: float, restitution: float) -> *physx_PxMaterial #foreign physx_sys;

PxPhysics_getNbMaterials :: (self: *physx_PxPhysics) -> u32 #foreign physx_sys;

PxPhysics_getMaterials :: (self: *physx_PxPhysics, userBuffer: **physx_PxMaterial, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_registerDeletionListener_mut :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener, deletionEvents: *u8, restrictedObjectSet: bool) -> void #foreign physx_sys;

PxPhysics_unregisterDeletionListener_mut :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener) -> void #foreign physx_sys;

PxPhysics_registerDeletionListenerObjects_mut :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener, observables: **physx_PxBase, observableCount: u32) -> void #foreign physx_sys;

PxPhysics_unregisterDeletionListenerObjects_mut :: (self: *physx_PxPhysics, observer: *physx_PxDeletionListener, observables: **physx_PxBase, observableCount: u32) -> void #foreign physx_sys;

PxPhysics_getPhysicsInsertionCallback_mut :: (self: *physx_PxPhysics) -> *physx_PxInsertionCallback #foreign physx_sys;

phys_PxCreatePhysics :: (version: u32, foundation: *physx_PxFoundation, scale: *physx_PxTolerancesScale, trackOutstandingAllocations: bool, pvd: *physx_PxPvd, omniPvd: *physx_PxOmniPvd) -> *physx_PxPhysics #foreign physx_sys;

phys_PxGetPhysics :: () -> *physx_PxPhysics #foreign physx_sys;

PxActorShape_new :: () -> physx_PxActorShape #foreign physx_sys;

PxActorShape_new_1 :: (a: *physx_PxRigidActor, s: *physx_PxShape) -> physx_PxActorShape #foreign physx_sys;

PxQueryCache_new :: () -> physx_PxQueryCache #foreign physx_sys;

PxQueryCache_new_1 :: (s: *physx_PxShape, findex: u32) -> physx_PxQueryCache #foreign physx_sys;

PxQueryFilterData_new :: () -> physx_PxQueryFilterData #foreign physx_sys;

PxQueryFilterData_new_1 :: (fd: *physx_PxFilterData, f: u16) -> physx_PxQueryFilterData #foreign physx_sys;

PxQueryFilterData_new_2 :: (f: u16) -> physx_PxQueryFilterData #foreign physx_sys;

PxQueryFilterCallback_preFilter_mut :: (self: *physx_PxQueryFilterCallback, filterData: *physx_PxFilterData, shape: *physx_PxShape, actor: *physx_PxRigidActor, queryFlags: *u16) -> s32 #foreign physx_sys;

PxQueryFilterCallback_postFilter_mut :: (self: *physx_PxQueryFilterCallback, filterData: *physx_PxFilterData, hit: *physx_PxQueryHit, shape: *physx_PxShape, actor: *physx_PxRigidActor) -> s32 #foreign physx_sys;

PxQueryFilterCallback_delete :: (self: *physx_PxQueryFilterCallback) -> void #foreign physx_sys;

PxRigidDynamic_setKinematicTarget_mut :: (self: *physx_PxRigidDynamic, destination: *physx_PxTransform) -> void #foreign physx_sys;

PxRigidDynamic_getKinematicTarget :: (self: *physx_PxRigidDynamic, target: *physx_PxTransform) -> bool #foreign physx_sys;

PxRigidDynamic_isSleeping :: (self: *physx_PxRigidDynamic) -> bool #foreign physx_sys;

PxRigidDynamic_setSleepThreshold_mut :: (self: *physx_PxRigidDynamic, threshold: float) -> void #foreign physx_sys;

PxRigidDynamic_getSleepThreshold :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_setStabilizationThreshold_mut :: (self: *physx_PxRigidDynamic, threshold: float) -> void #foreign physx_sys;

PxRigidDynamic_getStabilizationThreshold :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_getRigidDynamicLockFlags :: (self: *physx_PxRigidDynamic) -> u8 #foreign physx_sys;

PxRigidDynamic_setRigidDynamicLockFlag_mut :: (self: *physx_PxRigidDynamic, flag: s32, value: bool) -> void #foreign physx_sys;

PxRigidDynamic_setRigidDynamicLockFlags_mut :: (self: *physx_PxRigidDynamic, flags: u8) -> void #foreign physx_sys;

PxRigidDynamic_getLinearVelocity :: (self: *physx_PxRigidDynamic) -> physx_Vector3 #foreign physx_sys;

PxRigidDynamic_setLinearVelocity_mut :: (self: *physx_PxRigidDynamic, linVel: *physx_Vector3, autowake: bool) -> void #foreign physx_sys;

PxRigidDynamic_getAngularVelocity :: (self: *physx_PxRigidDynamic) -> physx_Vector3 #foreign physx_sys;

PxRigidDynamic_setAngularVelocity_mut :: (self: *physx_PxRigidDynamic, angVel: *physx_Vector3, autowake: bool) -> void #foreign physx_sys;

PxRigidDynamic_setWakeCounter_mut :: (self: *physx_PxRigidDynamic, wakeCounterValue: float) -> void #foreign physx_sys;

PxRigidDynamic_getWakeCounter :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_wakeUp_mut :: (self: *physx_PxRigidDynamic) -> void #foreign physx_sys;

PxRigidDynamic_putToSleep_mut :: (self: *physx_PxRigidDynamic) -> void #foreign physx_sys;

PxRigidDynamic_setSolverIterationCounts_mut :: (self: *physx_PxRigidDynamic, minPositionIters: u32, minVelocityIters: u32) -> void #foreign physx_sys;

PxRigidDynamic_getSolverIterationCounts :: (self: *physx_PxRigidDynamic, minPositionIters: *u32, minVelocityIters: *u32) -> void #foreign physx_sys;

PxRigidDynamic_getContactReportThreshold :: (self: *physx_PxRigidDynamic) -> float #foreign physx_sys;

PxRigidDynamic_setContactReportThreshold_mut :: (self: *physx_PxRigidDynamic, threshold: float) -> void #foreign physx_sys;

PxRigidDynamic_getConcreteTypeName :: (self: *physx_PxRigidDynamic) -> *u8 #foreign physx_sys;

PxRigidStatic_getConcreteTypeName :: (self: *physx_PxRigidStatic) -> *u8 #foreign physx_sys;

PxSceneQueryDesc_new :: () -> physx_PxSceneQueryDesc #foreign physx_sys;

PxSceneQueryDesc_setToDefault_mut :: (self: *physx_PxSceneQueryDesc) -> void #foreign physx_sys;

PxSceneQueryDesc_isValid :: (self: *physx_PxSceneQueryDesc) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_setDynamicTreeRebuildRateHs32_mut :: (self: *physx_PxSceneQuerySystemBase, dynamicTreeRebuildRateHs32: u32) -> void #foreign physx_sys;

PxSceneQuerySystemBase_getDynamicTreeRebuildRateHs32 :: (self: *physx_PxSceneQuerySystemBase) -> u32 #foreign physx_sys;

PxSceneQuerySystemBase_forceRebuildDynamicTree_mut :: (self: *physx_PxSceneQuerySystemBase, prunerIndex: u32) -> void #foreign physx_sys;

PxSceneQuerySystemBase_setUpdateMode_mut :: (self: *physx_PxSceneQuerySystemBase, updateMode: s32) -> void #foreign physx_sys;

PxSceneQuerySystemBase_getUpdateMode :: (self: *physx_PxSceneQuerySystemBase) -> s32 #foreign physx_sys;

PxSceneQuerySystemBase_getStaticTimestamp :: (self: *physx_PxSceneQuerySystemBase) -> u32 #foreign physx_sys;

PxSceneQuerySystemBase_flushUpdates_mut :: (self: *physx_PxSceneQuerySystemBase) -> void #foreign physx_sys;

PxSceneQuerySystemBase_raycast :: (self: *physx_PxSceneQuerySystemBase, origin: *physx_Vector3, unitDir: *physx_Vector3, distance: float, hitCall: *physx_PxRaycastCallback, hitFlags: u16, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, queryFlags: u32) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_sweep :: (self: *physx_PxSceneQuerySystemBase, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *physx_Vector3, distance: float, hitCall: *physx_PxSweepCallback, hitFlags: u16, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float, queryFlags: u32) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_overlap :: (self: *physx_PxSceneQuerySystemBase, geometry: *physx_PxGeometry, pose: *physx_PxTransform, hitCall: *physx_PxOverlapCallback, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, queryFlags: u32) -> bool #foreign physx_sys;

PxSceneSQSystem_setSceneQueryUpdateMode_mut :: (self: *physx_PxSceneSQSystem, updateMode: s32) -> void #foreign physx_sys;

PxSceneSQSystem_getSceneQueryUpdateMode :: (self: *physx_PxSceneSQSystem) -> s32 #foreign physx_sys;

PxSceneSQSystem_getSceneQueryStaticTimestamp :: (self: *physx_PxSceneSQSystem) -> u32 #foreign physx_sys;

PxSceneSQSystem_flushQueryUpdates_mut :: (self: *physx_PxSceneSQSystem) -> void #foreign physx_sys;

PxSceneSQSystem_forceDynamicTreeRebuild_mut :: (self: *physx_PxSceneSQSystem, rebuildStaticStructure: bool, rebuildDynamicStructure: bool) -> void #foreign physx_sys;

PxSceneSQSystem_getStaticStructure :: (self: *physx_PxSceneSQSystem) -> s32 #foreign physx_sys;

PxSceneSQSystem_getDynamicStructure :: (self: *physx_PxSceneSQSystem) -> s32 #foreign physx_sys;

PxSceneSQSystem_sceneQueriesUpdate_mut :: (self: *physx_PxSceneSQSystem, completionTask: *physx_PxBaseTask, controlSimulation: bool) -> void #foreign physx_sys;

PxSceneSQSystem_checkQueries_mut :: (self: *physx_PxSceneSQSystem, block: bool) -> bool #foreign physx_sys;

PxSceneSQSystem_fetchQueries_mut :: (self: *physx_PxSceneSQSystem, block: bool) -> bool #foreign physx_sys;

PxSceneQuerySystem_release_mut :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys;

PxSceneQuerySystem_acquireReference_mut :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys;

PxSceneQuerySystem_preallocate_mut :: (self: *physx_PxSceneQuerySystem, prunerIndex: u32, nbShapes: u32) -> void #foreign physx_sys;

PxSceneQuerySystem_flushMemory_mut :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys;

PxSceneQuerySystem_addSQShape_mut :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape, bounds: *physx_PxBounds3, transform: *physx_PxTransform, compoundHandle: *u32, hasPruningStructure: bool) -> void #foreign physx_sys;

PxSceneQuerySystem_removeSQShape_mut :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape) -> void #foreign physx_sys;

PxSceneQuerySystem_updateSQShape_mut :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape, transform: *physx_PxTransform) -> void #foreign physx_sys;

PxSceneQuerySystem_addSQCompound_mut :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shapes: **physx_PxShape, bvh: *physx_PxBVH, transforms: *physx_PxTransform) -> u32 #foreign physx_sys;

PxSceneQuerySystem_removeSQCompound_mut :: (self: *physx_PxSceneQuerySystem, compoundHandle: u32) -> void #foreign physx_sys;

PxSceneQuerySystem_updateSQCompound_mut :: (self: *physx_PxSceneQuerySystem, compoundHandle: u32, compoundTransform: *physx_PxTransform) -> void #foreign physx_sys;

PxSceneQuerySystem_shiftOrigin_mut :: (self: *physx_PxSceneQuerySystem, shift: *physx_Vector3) -> void #foreign physx_sys;

PxSceneQuerySystem_merge_mut :: (self: *physx_PxSceneQuerySystem, pruningStructure: *physx_PxPruningStructure) -> void #foreign physx_sys;

PxSceneQuerySystem_getHandle :: (self: *physx_PxSceneQuerySystem, actor: *physx_PxRigidActor, shape: *physx_PxShape, prunerIndex: *u32) -> u32 #foreign physx_sys;

PxSceneQuerySystem_sync_mut :: (self: *physx_PxSceneQuerySystem, prunerIndex: u32, handles: *u32, indices: *u32, bounds: *physx_PxBounds3, transforms: *physx_PxTransformPadded, count: u32, ignoredIndices: *physx_PxBitMap) -> void #foreign physx_sys;

PxSceneQuerySystem_finalizeUpdates_mut :: (self: *physx_PxSceneQuerySystem) -> void #foreign physx_sys;

PxSceneQuerySystem_prepareSceneQueryBuildStep_mut :: (self: *physx_PxSceneQuerySystem, prunerIndex: u32) -> *void #foreign physx_sys;

PxSceneQuerySystem_sceneQueryBuildStep_mut :: (self: *physx_PxSceneQuerySystem, handle: *void) -> void #foreign physx_sys;

PxBroadPhaseDesc_new :: (type: s32) -> physx_PxBroadPhaseDesc #foreign physx_sys;

PxBroadPhaseDesc_isValid :: (self: *physx_PxBroadPhaseDesc) -> bool #foreign physx_sys;

phys_PxGetBroadPhaseStaticFilterGroup :: () -> u32 #foreign physx_sys;

phys_PxGetBroadPhaseDynamicFilterGroup :: (id: u32) -> u32 #foreign physx_sys;

phys_PxGetBroadPhaseKinematicFilterGroup :: (id: u32) -> u32 #foreign physx_sys;

PxBroadPhaseUpdateData_new :: (created: *u32, nbCreated: u32, updated: *u32, nbUpdated: u32, removed: *u32, nbRemoved: u32, bounds: *physx_PxBounds3, groups: *u32, distances: *float, capacity: u32) -> physx_PxBroadPhaseUpdateData #foreign physx_sys;

PxBroadPhaseResults_new :: () -> physx_PxBroadPhaseResults #foreign physx_sys;

PxBroadPhaseRegions_getNbRegions :: (self: *physx_PxBroadPhaseRegions) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_getRegions :: (self: *physx_PxBroadPhaseRegions, userBuffer: *physx_PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_addRegion_mut :: (self: *physx_PxBroadPhaseRegions, region: *physx_PxBroadPhaseRegion, populateRegion: bool, bounds: *physx_PxBounds3, distances: *float) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_removeRegion_mut :: (self: *physx_PxBroadPhaseRegions, handle: u32) -> bool #foreign physx_sys;

PxBroadPhaseRegions_getNbOutOfBoundsObjects :: (self: *physx_PxBroadPhaseRegions) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_getOutOfBoundsObjects :: (self: *physx_PxBroadPhaseRegions) -> *u32 #foreign physx_sys;

PxBroadPhase_release_mut :: (self: *physx_PxBroadPhase) -> void #foreign physx_sys;

PxBroadPhase_getType :: (self: *physx_PxBroadPhase) -> s32 #foreign physx_sys;

PxBroadPhase_getCaps :: (self: *physx_PxBroadPhase, caps: *physx_PxBroadPhaseCaps) -> void #foreign physx_sys;

PxBroadPhase_getRegions_mut :: (self: *physx_PxBroadPhase) -> *physx_PxBroadPhaseRegions #foreign physx_sys;

PxBroadPhase_getAllocator_mut :: (self: *physx_PxBroadPhase) -> *physx_PxAllocatorCallback #foreign physx_sys;

PxBroadPhase_getContextID :: (self: *physx_PxBroadPhase) -> u64 #foreign physx_sys;

PxBroadPhase_setScratchBlock_mut :: (self: *physx_PxBroadPhase, scratchBlock: *void, size: u32) -> void #foreign physx_sys;

PxBroadPhase_update_mut :: (self: *physx_PxBroadPhase, updateData: *physx_PxBroadPhaseUpdateData, continuation: *physx_PxBaseTask) -> void #foreign physx_sys;

PxBroadPhase_fetchResults_mut :: (self: *physx_PxBroadPhase, results: *physx_PxBroadPhaseResults) -> void #foreign physx_sys;

PxBroadPhase_update_mut_1 :: (self: *physx_PxBroadPhase, results: *physx_PxBroadPhaseResults, updateData: *physx_PxBroadPhaseUpdateData) -> void #foreign physx_sys;

phys_PxCreateBroadPhase :: (desc: *physx_PxBroadPhaseDesc) -> *physx_PxBroadPhase #foreign physx_sys;

PxAABBManager_release_mut :: (self: *physx_PxAABBManager) -> void #foreign physx_sys;

PxAABBManager_getBroadPhase_mut :: (self: *physx_PxAABBManager) -> *physx_PxBroadPhase #foreign physx_sys;

PxAABBManager_getBounds :: (self: *physx_PxAABBManager) -> *physx_PxBounds3 #foreign physx_sys;

PxAABBManager_getDistances :: (self: *physx_PxAABBManager) -> *float #foreign physx_sys;

PxAABBManager_getGroups :: (self: *physx_PxAABBManager) -> *u32 #foreign physx_sys;

PxAABBManager_getCapacity :: (self: *physx_PxAABBManager) -> u32 #foreign physx_sys;

PxAABBManager_addObject_mut :: (self: *physx_PxAABBManager, index: u32, bounds: *physx_PxBounds3, group: u32, distance: float) -> void #foreign physx_sys;

PxAABBManager_removeObject_mut :: (self: *physx_PxAABBManager, index: u32) -> void #foreign physx_sys;

PxAABBManager_updateObject_mut :: (self: *physx_PxAABBManager, index: u32, bounds: *physx_PxBounds3, distance: *float) -> void #foreign physx_sys;

PxAABBManager_update_mut :: (self: *physx_PxAABBManager, continuation: *physx_PxBaseTask) -> void #foreign physx_sys;

PxAABBManager_fetchResults_mut :: (self: *physx_PxAABBManager, results: *physx_PxBroadPhaseResults) -> void #foreign physx_sys;

PxAABBManager_update_mut_1 :: (self: *physx_PxAABBManager, results: *physx_PxBroadPhaseResults) -> void #foreign physx_sys;

phys_PxCreateAABBManager :: (broadphase: *physx_PxBroadPhase) -> *physx_PxAABBManager #foreign physx_sys;

PxSceneLimits_new :: () -> physx_PxSceneLimits #foreign physx_sys;

PxSceneLimits_setToDefault_mut :: (self: *physx_PxSceneLimits) -> void #foreign physx_sys;

PxSceneLimits_isValid :: (self: *physx_PxSceneLimits) -> bool #foreign physx_sys;

PxgDynamicsMemoryConfig_new :: () -> physx_PxgDynamicsMemoryConfig #foreign physx_sys;

PxgDynamicsMemoryConfig_isValid :: (self: *physx_PxgDynamicsMemoryConfig) -> bool #foreign physx_sys;

PxSceneDesc_new :: (scale: *physx_PxTolerancesScale) -> physx_PxSceneDesc #foreign physx_sys;

PxSceneDesc_setToDefault_mut :: (self: *physx_PxSceneDesc, scale: *physx_PxTolerancesScale) -> void #foreign physx_sys;

PxSceneDesc_isValid :: (self: *physx_PxSceneDesc) -> bool #foreign physx_sys;

PxSceneDesc_getTolerancesScale :: (self: *physx_PxSceneDesc) -> *physx_PxTolerancesScale #foreign physx_sys;

PxSimulationStatistics_getNbBroadPhaseAdds :: (self: *physx_PxSimulationStatistics) -> u32 #foreign physx_sys;

PxSimulationStatistics_getNbBroadPhaseRemoves :: (self: *physx_PxSimulationStatistics) -> u32 #foreign physx_sys;

PxSimulationStatistics_getRbPairStats :: (self: *physx_PxSimulationStatistics, pairType: s32, g0: s32, g1: s32) -> u32 #foreign physx_sys;

PxSimulationStatistics_new :: () -> physx_PxSimulationStatistics #foreign physx_sys;

PxPvdSceneClient_setScenePvdFlag_mut :: (self: *physx_PxPvdSceneClient, flag: s32, value: bool) -> void #foreign physx_sys;

PxPvdSceneClient_setScenePvdFlags_mut :: (self: *physx_PxPvdSceneClient, flags: u8) -> void #foreign physx_sys;

PxPvdSceneClient_getScenePvdFlags :: (self: *physx_PxPvdSceneClient) -> u8 #foreign physx_sys;

PxPvdSceneClient_updateCamera_mut :: (self: *physx_PxPvdSceneClient, name: *u8, origin: *physx_Vector3, up: *physx_Vector3, target: *physx_Vector3) -> void #foreign physx_sys;

PxPvdSceneClient_drawPos32s_mut :: (self: *physx_PxPvdSceneClient, pos32s: *physx_PxDebugPos32, count: u32) -> void #foreign physx_sys;

PxPvdSceneClient_drawLines_mut :: (self: *physx_PxPvdSceneClient, lines: *physx_PxDebugLine, count: u32) -> void #foreign physx_sys;

PxPvdSceneClient_drawTriangles_mut :: (self: *physx_PxPvdSceneClient, triangles: *physx_PxDebugTriangle, count: u32) -> void #foreign physx_sys;

PxPvdSceneClient_drawText_mut :: (self: *physx_PxPvdSceneClient, text: *physx_PxDebugText) -> void #foreign physx_sys;

PxDominanceGroupPair_new :: (a: u8, b: u8) -> physx_PxDominanceGroupPair #foreign physx_sys;

PxBroadPhaseCallback_delete :: (self: *physx_PxBroadPhaseCallback) -> void #foreign physx_sys;

PxBroadPhaseCallback_onObjectOutOfBounds_mut :: (self: *physx_PxBroadPhaseCallback, shape: *physx_PxShape, actor: *physx_PxActor) -> void #foreign physx_sys;

PxBroadPhaseCallback_onObjectOutOfBounds_mut_1 :: (self: *physx_PxBroadPhaseCallback, aggregate: *physx_PxAggregate) -> void #foreign physx_sys;

PxScene_release_mut :: (self: *physx_PxScene) -> void #foreign physx_sys;

PxScene_setFlag_mut :: (self: *physx_PxScene, flag: s32, value: bool) -> void #foreign physx_sys;

PxScene_getFlags :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setLimits_mut :: (self: *physx_PxScene, limits: *physx_PxSceneLimits) -> void #foreign physx_sys;

PxScene_getLimits :: (self: *physx_PxScene) -> physx_PxSceneLimits #foreign physx_sys;

PxScene_getPhysics_mut :: (self: *physx_PxScene) -> *physx_PxPhysics #foreign physx_sys;

PxScene_getTimestamp :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_addArticulation_mut :: (self: *physx_PxScene, articulation: *physx_PxArticulationReducedCoordinate) -> bool #foreign physx_sys;

PxScene_removeArticulation_mut :: (self: *physx_PxScene, articulation: *physx_PxArticulationReducedCoordinate, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_addActor_mut :: (self: *physx_PxScene, actor: *physx_PxActor, bvh: *physx_PxBVH) -> bool #foreign physx_sys;

PxScene_addActors_mut :: (self: *physx_PxScene, actors: **physx_PxActor, nbActors: u32) -> bool #foreign physx_sys;

PxScene_addActors_mut_1 :: (self: *physx_PxScene, pruningStructure: *physx_PxPruningStructure) -> bool #foreign physx_sys;

PxScene_removeActor_mut :: (self: *physx_PxScene, actor: *physx_PxActor, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_removeActors_mut :: (self: *physx_PxScene, actors: **physx_PxActor, nbActors: u32, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_addAggregate_mut :: (self: *physx_PxScene, aggregate: *physx_PxAggregate) -> bool #foreign physx_sys;

PxScene_removeAggregate_mut :: (self: *physx_PxScene, aggregate: *physx_PxAggregate, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_addCollection_mut :: (self: *physx_PxScene, collection: *physx_PxCollection) -> bool #foreign physx_sys;

PxScene_getNbActors :: (self: *physx_PxScene, types: u16) -> u32 #foreign physx_sys;

PxScene_getActors :: (self: *physx_PxScene, types: u16, userBuffer: **physx_PxActor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getActiveActors_mut :: (self: *physx_PxScene, nbActorsOut: *u32) -> **physx_PxActor #foreign physx_sys;

PxScene_getNbArticulations :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getArticulations :: (self: *physx_PxScene, userBuffer: **physx_PxArticulationReducedCoordinate, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getNbConstras32s :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getConstras32s :: (self: *physx_PxScene, userBuffer: **physx_PxConstras32, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getNbAggregates :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getAggregates :: (self: *physx_PxScene, userBuffer: **physx_PxAggregate, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_setDominanceGroupPair_mut :: (self: *physx_PxScene, group1: u8, group2: u8, dominance: *physx_PxDominanceGroupPair) -> void #foreign physx_sys;

PxScene_getDominanceGroupPair :: (self: *physx_PxScene, group1: u8, group2: u8) -> physx_PxDominanceGroupPair #foreign physx_sys;

PxScene_getCpuDispatcher :: (self: *physx_PxScene) -> *physx_PxCpuDispatcher #foreign physx_sys;

PxScene_createClient_mut :: (self: *physx_PxScene) -> u8 #foreign physx_sys;

PxScene_setSimulationEventCallback_mut :: (self: *physx_PxScene, callback: *physx_PxSimulationEventCallback) -> void #foreign physx_sys;

PxScene_getSimulationEventCallback :: (self: *physx_PxScene) -> *physx_PxSimulationEventCallback #foreign physx_sys;

PxScene_setContactModifyCallback_mut :: (self: *physx_PxScene, callback: *physx_PxContactModifyCallback) -> void #foreign physx_sys;

PxScene_setCCDContactModifyCallback_mut :: (self: *physx_PxScene, callback: *physx_PxCCDContactModifyCallback) -> void #foreign physx_sys;

PxScene_getContactModifyCallback :: (self: *physx_PxScene) -> *physx_PxContactModifyCallback #foreign physx_sys;

PxScene_getCCDContactModifyCallback :: (self: *physx_PxScene) -> *physx_PxCCDContactModifyCallback #foreign physx_sys;

PxScene_setBroadPhaseCallback_mut :: (self: *physx_PxScene, callback: *physx_PxBroadPhaseCallback) -> void #foreign physx_sys;

PxScene_getBroadPhaseCallback :: (self: *physx_PxScene) -> *physx_PxBroadPhaseCallback #foreign physx_sys;

PxScene_setFilterShaderData_mut :: (self: *physx_PxScene, data: *void, dataSize: u32) -> void #foreign physx_sys;

PxScene_getFilterShaderData :: (self: *physx_PxScene) -> *void #foreign physx_sys;

PxScene_getFilterShaderDataSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_resetFiltering_mut :: (self: *physx_PxScene, actor: *physx_PxActor) -> bool #foreign physx_sys;

PxScene_resetFiltering_mut_1 :: (self: *physx_PxScene, actor: *physx_PxRigidActor, shapes: **physx_PxShape, shapeCount: u32) -> bool #foreign physx_sys;

PxScene_getKinematicKinematicFilteringMode :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_getStaticKinematicFilteringMode :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_simulate_mut :: (self: *physx_PxScene, elapsedTime: float, completionTask: *physx_PxBaseTask, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign physx_sys;

PxScene_advance_mut :: (self: *physx_PxScene, completionTask: *physx_PxBaseTask) -> bool #foreign physx_sys;

PxScene_collide_mut :: (self: *physx_PxScene, elapsedTime: float, completionTask: *physx_PxBaseTask, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign physx_sys;

PxScene_checkResults_mut :: (self: *physx_PxScene, block: bool) -> bool #foreign physx_sys;

PxScene_fetchCollision_mut :: (self: *physx_PxScene, block: bool) -> bool #foreign physx_sys;

PxScene_fetchResults_mut :: (self: *physx_PxScene, block: bool, errorState: *u32) -> bool #foreign physx_sys;

PxScene_fetchResultsStart_mut :: (self: *physx_PxScene, contactPairs: **physx_PxContactPairHeader, nbContactPairs: *u32, block: bool) -> bool #foreign physx_sys;

PxScene_processCallbacks_mut :: (self: *physx_PxScene, continuation: *physx_PxBaseTask) -> void #foreign physx_sys;

PxScene_fetchResultsFinish_mut :: (self: *physx_PxScene, errorState: *u32) -> void #foreign physx_sys;

PxScene_fetchResultsParticleSystem_mut :: (self: *physx_PxScene) -> void #foreign physx_sys;

PxScene_flushSimulation_mut :: (self: *physx_PxScene, sendPendingReports: bool) -> void #foreign physx_sys;

PxScene_setGravity_mut :: (self: *physx_PxScene, vec: *physx_Vector3) -> void #foreign physx_sys;

PxScene_getGravity :: (self: *physx_PxScene) -> physx_Vector3 #foreign physx_sys;

PxScene_setBounceThresholdVelocity_mut :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys;

PxScene_getBounceThresholdVelocity :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setCCDMaxPasses_mut :: (self: *physx_PxScene, ccdMaxPasses: u32) -> void #foreign physx_sys;

PxScene_getCCDMaxPasses :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setCCDMaxSeparation_mut :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys;

PxScene_getCCDMaxSeparation :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setCCDThreshold_mut :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys;

PxScene_getCCDThreshold :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setMaxBiasCoefficient_mut :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys;

PxScene_getMaxBiasCoefficient :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setFrictionOffsetThreshold_mut :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys;

PxScene_getFrictionOffsetThreshold :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_setFrictionCorrelationDistance_mut :: (self: *physx_PxScene, t: float) -> void #foreign physx_sys;

PxScene_getFrictionCorrelationDistance :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_getFrictionType :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_getSolverType :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_setVisualizationParameter_mut :: (self: *physx_PxScene, param: s32, value: float) -> bool #foreign physx_sys;

PxScene_getVisualizationParameter :: (self: *physx_PxScene, paramEnum: s32) -> float #foreign physx_sys;

PxScene_setVisualizationCullingBox_mut :: (self: *physx_PxScene, box: *physx_PxBounds3) -> void #foreign physx_sys;

PxScene_getVisualizationCullingBox :: (self: *physx_PxScene) -> physx_PxBounds3 #foreign physx_sys;

PxScene_getRenderBuffer_mut :: (self: *physx_PxScene) -> *physx_PxRenderBuffer #foreign physx_sys;

PxScene_getSimulationStatistics :: (self: *physx_PxScene, stats: *physx_PxSimulationStatistics) -> void #foreign physx_sys;

PxScene_getBroadPhaseType :: (self: *physx_PxScene) -> s32 #foreign physx_sys;

PxScene_getBroadPhaseCaps :: (self: *physx_PxScene, caps: *physx_PxBroadPhaseCaps) -> bool #foreign physx_sys;

PxScene_getNbBroadPhaseRegions :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getBroadPhaseRegions :: (self: *physx_PxScene, userBuffer: *physx_PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_addBroadPhaseRegion_mut :: (self: *physx_PxScene, region: *physx_PxBroadPhaseRegion, populateRegion: bool) -> u32 #foreign physx_sys;

PxScene_removeBroadPhaseRegion_mut :: (self: *physx_PxScene, handle: u32) -> bool #foreign physx_sys;

PxScene_getTaskManager :: (self: *physx_PxScene) -> *physx_PxTaskManager #foreign physx_sys;

PxScene_lockRead_mut :: (self: *physx_PxScene, file: *u8, line: u32) -> void #foreign physx_sys;

PxScene_unlockRead_mut :: (self: *physx_PxScene) -> void #foreign physx_sys;

PxScene_lockWrite_mut :: (self: *physx_PxScene, file: *u8, line: u32) -> void #foreign physx_sys;

PxScene_unlockWrite_mut :: (self: *physx_PxScene) -> void #foreign physx_sys;

PxScene_setNbContactDataBlocks_mut :: (self: *physx_PxScene, numBlocks: u32) -> void #foreign physx_sys;

PxScene_getNbContactDataBlocksUsed :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getMaxNbContactDataBlocksUsed :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getContactReportStreamBufferSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setSolverBatchSize_mut :: (self: *physx_PxScene, solverBatchSize: u32) -> void #foreign physx_sys;

PxScene_getSolverBatchSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_setSolverArticulationBatchSize_mut :: (self: *physx_PxScene, solverBatchSize: u32) -> void #foreign physx_sys;

PxScene_getSolverArticulationBatchSize :: (self: *physx_PxScene) -> u32 #foreign physx_sys;

PxScene_getWakeCounterResetValue :: (self: *physx_PxScene) -> float #foreign physx_sys;

PxScene_shiftOrigin_mut :: (self: *physx_PxScene, shift: *physx_Vector3) -> void #foreign physx_sys;

PxScene_getScenePvdClient_mut :: (self: *physx_PxScene) -> *physx_PxPvdSceneClient #foreign physx_sys;

PxScene_copyArticulationData_mut :: (self: *physx_PxScene, data: *void, index: *void, dataType: s32, nbCopyArticulations: u32, copyEvent: *void) -> void #foreign physx_sys;

PxScene_applyArticulationData_mut :: (self: *physx_PxScene, data: *void, index: *void, dataType: s32, nbUpdatedArticulations: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys;

PxScene_copySoftBodyData_mut :: (self: *physx_PxScene, data: **void, dataSizes: *void, softBodyIndices: *void, flag: s32, nbCopySoftBodies: u32, maxSize: u32, copyEvent: *void) -> void #foreign physx_sys;

PxScene_applySoftBodyData_mut :: (self: *physx_PxScene, data: **void, dataSizes: *void, softBodyIndices: *void, flag: s32, nbUpdatedSoftBodies: u32, maxSize: u32, applyEvent: *void) -> void #foreign physx_sys;

PxScene_copyContactData_mut :: (self: *physx_PxScene, data: *void, maxContactPairs: u32, numContactPairs: *void, copyEvent: *void) -> void #foreign physx_sys;

PxScene_copyBodyData_mut :: (self: *physx_PxScene, data: *physx_PxGpuBodyData, index: *physx_PxGpuActorPair, nbCopyActors: u32, copyEvent: *void) -> void #foreign physx_sys;

PxScene_applyActorData_mut :: (self: *physx_PxScene, data: *void, index: *physx_PxGpuActorPair, flag: s32, nbUpdatedActors: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys;

PxScene_computeDenseJacobians_mut :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_computeGeneralizedMassMatrices_mut :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_computeGeneralizedGravityForces_mut :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_computeCoriolisAndCentrifugalForces_mut :: (self: *physx_PxScene, indices: *physx_PxIndexDataPair, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_getGpuDynamicsConfig :: (self: *physx_PxScene) -> physx_PxgDynamicsMemoryConfig #foreign physx_sys;

PxScene_applyParticleBufferData_mut :: (self: *physx_PxScene, indices: *u32, bufferIndexPair: *physx_PxGpuParticleBufferIndexPair, flags: *u32, nbUpdatedBuffers: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys;

PxSceneReadLock_new_alloc :: (scene: *physx_PxScene, file: *u8, line: u32) -> *physx_PxSceneReadLock #foreign physx_sys;

PxSceneReadLock_delete :: (self: *physx_PxSceneReadLock) -> void #foreign physx_sys;

PxSceneWriteLock_new_alloc :: (scene: *physx_PxScene, file: *u8, line: u32) -> *physx_PxSceneWriteLock #foreign physx_sys;

PxSceneWriteLock_delete :: (self: *physx_PxSceneWriteLock) -> void #foreign physx_sys;

PxContactPairExtraDataItem_new :: () -> physx_PxContactPairExtraDataItem #foreign physx_sys;

PxContactPairVelocity_new :: () -> physx_PxContactPairVelocity #foreign physx_sys;

PxContactPairPose_new :: () -> physx_PxContactPairPose #foreign physx_sys;

PxContactPairIndex_new :: () -> physx_PxContactPairIndex #foreign physx_sys;

PxContactPairExtraDataIterator_new :: (stream: *u8, size: u32) -> physx_PxContactPairExtraDataIterator #foreign physx_sys;

PxContactPairExtraDataIterator_nextItemSet_mut :: (self: *physx_PxContactPairExtraDataIterator) -> bool #foreign physx_sys;

PxContactPairHeader_new :: () -> physx_PxContactPairHeader #foreign physx_sys;

PxContactPair_new :: () -> physx_PxContactPair #foreign physx_sys;

PxContactPair_extractContacts :: (self: *physx_PxContactPair, userBuffer: *physx_PxContactPairPos32, bufferSize: u32) -> u32 #foreign physx_sys;

PxContactPair_bufferContacts :: (self: *physx_PxContactPair, newPair: *physx_PxContactPair, bufferMemory: *u8) -> void #foreign physx_sys;

PxContactPair_getInternalFaceIndices :: (self: *physx_PxContactPair) -> *u32 #foreign physx_sys;

PxTriggerPair_new :: () -> physx_PxTriggerPair #foreign physx_sys;

PxConstras32Info_new :: () -> physx_PxConstras32Info #foreign physx_sys;

PxConstras32Info_new_1 :: (c: *physx_PxConstras32, extRef: *void, t: u32) -> physx_PxConstras32Info #foreign physx_sys;

PxSimulationEventCallback_onConstras32Break_mut :: (self: *physx_PxSimulationEventCallback, constras32s: *physx_PxConstras32Info, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onWake_mut :: (self: *physx_PxSimulationEventCallback, actors: **physx_PxActor, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onSleep_mut :: (self: *physx_PxSimulationEventCallback, actors: **physx_PxActor, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onContact_mut :: (self: *physx_PxSimulationEventCallback, pairHeader: *physx_PxContactPairHeader, pairs: *physx_PxContactPair, nbPairs: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onTrigger_mut :: (self: *physx_PxSimulationEventCallback, pairs: *physx_PxTriggerPair, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onAdvance_mut :: (self: *physx_PxSimulationEventCallback, bodyBuffer: **physx_PxRigidBody, poseBuffer: *physx_PxTransform, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_delete :: (self: *physx_PxSimulationEventCallback) -> void #foreign physx_sys;

PxFEMParameters_new :: () -> physx_PxFEMParameters #foreign physx_sys;

PxPruningStructure_release_mut :: (self: *physx_PxPruningStructure) -> void #foreign physx_sys;

PxPruningStructure_getRigidActors :: (self: *physx_PxPruningStructure, userBuffer: **physx_PxRigidActor, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPruningStructure_getNbRigidActors :: (self: *physx_PxPruningStructure) -> u32 #foreign physx_sys;

PxPruningStructure_getStaticMergeData :: (self: *physx_PxPruningStructure) -> *void #foreign physx_sys;

PxPruningStructure_getDynamicMergeData :: (self: *physx_PxPruningStructure) -> *void #foreign physx_sys;

PxPruningStructure_getConcreteTypeName :: (self: *physx_PxPruningStructure) -> *u8 #foreign physx_sys;

PxExtendedVec3_new :: () -> physx_PxExtendedVec3 #foreign physx_sys;

PxExtendedVec3_new_1 :: (_x: float64, _y: float64, _z: float64) -> physx_PxExtendedVec3 #foreign physx_sys;

PxExtendedVec3_isZero :: (self: *physx_PxExtendedVec3) -> bool #foreign physx_sys;

PxExtendedVec3_dot :: (self: *physx_PxExtendedVec3, v: *physx_Vector3) -> float64 #foreign physx_sys;

PxExtendedVec3_distanceSquared :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_magnitudeSquared :: (self: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_magnitude :: (self: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_normalize_mut :: (self: *physx_PxExtendedVec3) -> float64 #foreign physx_sys;

PxExtendedVec3_isFinite :: (self: *physx_PxExtendedVec3) -> bool #foreign physx_sys;

PxExtendedVec3_maximum_mut :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> void #foreign physx_sys;

PxExtendedVec3_minimum_mut :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> void #foreign physx_sys;

PxExtendedVec3_set_mut :: (self: *physx_PxExtendedVec3, x_: float64, y_: float64, z_: float64) -> void #foreign physx_sys;

PxExtendedVec3_setPlusInfinity_mut :: (self: *physx_PxExtendedVec3) -> void #foreign physx_sys;

PxExtendedVec3_setMinusInfinity_mut :: (self: *physx_PxExtendedVec3) -> void #foreign physx_sys;

PxExtendedVec3_cross_mut :: (self: *physx_PxExtendedVec3, left: *physx_PxExtendedVec3, right: *physx_Vector3) -> void #foreign physx_sys;

PxExtendedVec3_cross_mut_1 :: (self: *physx_PxExtendedVec3, left: *physx_PxExtendedVec3, right: *physx_PxExtendedVec3) -> void #foreign physx_sys;

PxExtendedVec3_cross :: (self: *physx_PxExtendedVec3, v: *physx_PxExtendedVec3) -> physx_PxExtendedVec3 #foreign physx_sys;

PxExtendedVec3_cross_mut_2 :: (self: *physx_PxExtendedVec3, left: *physx_Vector3, right: *physx_PxExtendedVec3) -> void #foreign physx_sys;

phys_toVec3 :: (v: *physx_PxExtendedVec3) -> physx_Vector3 #foreign physx_sys;

PxObstacle_getType :: (self: *physx_PxObstacle) -> s32 #foreign physx_sys;

PxBoxObstacle_new :: () -> physx_PxBoxObstacle #foreign physx_sys;

PxCapsuleObstacle_new :: () -> physx_PxCapsuleObstacle #foreign physx_sys;

PxObstacleContext_release_mut :: (self: *physx_PxObstacleContext) -> void #foreign physx_sys;

PxObstacleContext_getControllerManager :: (self: *physx_PxObstacleContext) -> *physx_PxControllerManager #foreign physx_sys;

PxObstacleContext_addObstacle_mut :: (self: *physx_PxObstacleContext, obstacle: *physx_PxObstacle) -> u32 #foreign physx_sys;

PxObstacleContext_removeObstacle_mut :: (self: *physx_PxObstacleContext, handle: u32) -> bool #foreign physx_sys;

PxObstacleContext_updateObstacle_mut :: (self: *physx_PxObstacleContext, handle: u32, obstacle: *physx_PxObstacle) -> bool #foreign physx_sys;

PxObstacleContext_getNbObstacles :: (self: *physx_PxObstacleContext) -> u32 #foreign physx_sys;

PxObstacleContext_getObstacle :: (self: *physx_PxObstacleContext, i: u32) -> *physx_PxObstacle #foreign physx_sys;

PxObstacleContext_getObstacleByHandle :: (self: *physx_PxObstacleContext, handle: u32) -> *physx_PxObstacle #foreign physx_sys;

PxUserControllerHitReport_onShapeHit_mut :: (self: *physx_PxUserControllerHitReport, hit: *physx_PxControllerShapeHit) -> void #foreign physx_sys;

PxUserControllerHitReport_onControllerHit_mut :: (self: *physx_PxUserControllerHitReport, hit: *physx_PxControllersHit) -> void #foreign physx_sys;

PxUserControllerHitReport_onObstacleHit_mut :: (self: *physx_PxUserControllerHitReport, hit: *physx_PxControllerObstacleHit) -> void #foreign physx_sys;

PxControllerFilterCallback_delete :: (self: *physx_PxControllerFilterCallback) -> void #foreign physx_sys;

PxControllerFilterCallback_filter_mut :: (self: *physx_PxControllerFilterCallback, a: *physx_PxController, b: *physx_PxController) -> bool #foreign physx_sys;

PxControllerFilters_new :: (filterData: *physx_PxFilterData, cb: *physx_PxQueryFilterCallback, cctFilterCb: *physx_PxControllerFilterCallback) -> physx_PxControllerFilters #foreign physx_sys;

PxControllerDesc_isValid :: (self: *physx_PxControllerDesc) -> bool #foreign physx_sys;

PxControllerDesc_getType :: (self: *physx_PxControllerDesc) -> s32 #foreign physx_sys;

PxController_getType :: (self: *physx_PxController) -> s32 #foreign physx_sys;

PxController_release_mut :: (self: *physx_PxController) -> void #foreign physx_sys;

PxController_move_mut :: (self: *physx_PxController, disp: *physx_Vector3, minDist: float, elapsedTime: float, filters: *physx_PxControllerFilters, obstacles: *physx_PxObstacleContext) -> u8 #foreign physx_sys;

PxController_setPosition_mut :: (self: *physx_PxController, position: *physx_PxExtendedVec3) -> bool #foreign physx_sys;

PxController_getPosition :: (self: *physx_PxController) -> *physx_PxExtendedVec3 #foreign physx_sys;

PxController_setFootPosition_mut :: (self: *physx_PxController, position: *physx_PxExtendedVec3) -> bool #foreign physx_sys;

PxController_getFootPosition :: (self: *physx_PxController) -> physx_PxExtendedVec3 #foreign physx_sys;

PxController_getActor :: (self: *physx_PxController) -> *physx_PxRigidDynamic #foreign physx_sys;

PxController_setStepOffset_mut :: (self: *physx_PxController, offset: float) -> void #foreign physx_sys;

PxController_getStepOffset :: (self: *physx_PxController) -> float #foreign physx_sys;

PxController_setNonWalkableMode_mut :: (self: *physx_PxController, flag: s32) -> void #foreign physx_sys;

PxController_getNonWalkableMode :: (self: *physx_PxController) -> s32 #foreign physx_sys;

PxController_getContactOffset :: (self: *physx_PxController) -> float #foreign physx_sys;

PxController_setContactOffset_mut :: (self: *physx_PxController, offset: float) -> void #foreign physx_sys;

PxController_getUpDirection :: (self: *physx_PxController) -> physx_Vector3 #foreign physx_sys;

PxController_setUpDirection_mut :: (self: *physx_PxController, up: *physx_Vector3) -> void #foreign physx_sys;

PxController_getSlopeLimit :: (self: *physx_PxController) -> float #foreign physx_sys;

PxController_setSlopeLimit_mut :: (self: *physx_PxController, slopeLimit: float) -> void #foreign physx_sys;

PxController_invalidateCache_mut :: (self: *physx_PxController) -> void #foreign physx_sys;

PxController_getScene_mut :: (self: *physx_PxController) -> *physx_PxScene #foreign physx_sys;

PxController_getUserData :: (self: *physx_PxController) -> *void #foreign physx_sys;

PxController_setUserData_mut :: (self: *physx_PxController, userData: *void) -> void #foreign physx_sys;

PxController_getState :: (self: *physx_PxController, state: *physx_PxControllerState) -> void #foreign physx_sys;

PxController_getStats :: (self: *physx_PxController, stats: *physx_PxControllerStats) -> void #foreign physx_sys;

PxController_resize_mut :: (self: *physx_PxController, height: float) -> void #foreign physx_sys;

PxBoxControllerDesc_new_alloc :: () -> *physx_PxBoxControllerDesc #foreign physx_sys;

PxBoxControllerDesc_delete :: (self: *physx_PxBoxControllerDesc) -> void #foreign physx_sys;

PxBoxControllerDesc_setToDefault_mut :: (self: *physx_PxBoxControllerDesc) -> void #foreign physx_sys;

PxBoxControllerDesc_isValid :: (self: *physx_PxBoxControllerDesc) -> bool #foreign physx_sys;

PxBoxController_getHalfHeight :: (self: *physx_PxBoxController) -> float #foreign physx_sys;

PxBoxController_getHalfSideExtent :: (self: *physx_PxBoxController) -> float #foreign physx_sys;

PxBoxController_getHalfForwardExtent :: (self: *physx_PxBoxController) -> float #foreign physx_sys;

PxBoxController_setHalfHeight_mut :: (self: *physx_PxBoxController, halfHeight: float) -> bool #foreign physx_sys;

PxBoxController_setHalfSideExtent_mut :: (self: *physx_PxBoxController, halfSideExtent: float) -> bool #foreign physx_sys;

PxBoxController_setHalfForwardExtent_mut :: (self: *physx_PxBoxController, halfForwardExtent: float) -> bool #foreign physx_sys;

PxCapsuleControllerDesc_new_alloc :: () -> *physx_PxCapsuleControllerDesc #foreign physx_sys;

PxCapsuleControllerDesc_delete :: (self: *physx_PxCapsuleControllerDesc) -> void #foreign physx_sys;

PxCapsuleControllerDesc_setToDefault_mut :: (self: *physx_PxCapsuleControllerDesc) -> void #foreign physx_sys;

PxCapsuleControllerDesc_isValid :: (self: *physx_PxCapsuleControllerDesc) -> bool #foreign physx_sys;

PxCapsuleController_getRadius :: (self: *physx_PxCapsuleController) -> float #foreign physx_sys;

PxCapsuleController_setRadius_mut :: (self: *physx_PxCapsuleController, radius: float) -> bool #foreign physx_sys;

PxCapsuleController_getHeight :: (self: *physx_PxCapsuleController) -> float #foreign physx_sys;

PxCapsuleController_setHeight_mut :: (self: *physx_PxCapsuleController, height: float) -> bool #foreign physx_sys;

PxCapsuleController_getClimbingMode :: (self: *physx_PxCapsuleController) -> s32 #foreign physx_sys;

PxCapsuleController_setClimbingMode_mut :: (self: *physx_PxCapsuleController, mode: s32) -> bool #foreign physx_sys;

PxControllerBehaviorCallback_getBehaviorFlags_mut :: (self: *physx_PxControllerBehaviorCallback, shape: *physx_PxShape, actor: *physx_PxActor) -> u8 #foreign physx_sys;

PxControllerBehaviorCallback_getBehaviorFlags_mut_1 :: (self: *physx_PxControllerBehaviorCallback, controller: *physx_PxController) -> u8 #foreign physx_sys;

PxControllerBehaviorCallback_getBehaviorFlags_mut_2 :: (self: *physx_PxControllerBehaviorCallback, obstacle: *physx_PxObstacle) -> u8 #foreign physx_sys;

PxControllerManager_release_mut :: (self: *physx_PxControllerManager) -> void #foreign physx_sys;

PxControllerManager_getScene :: (self: *physx_PxControllerManager) -> *physx_PxScene #foreign physx_sys;

PxControllerManager_getNbControllers :: (self: *physx_PxControllerManager) -> u32 #foreign physx_sys;

PxControllerManager_getController_mut :: (self: *physx_PxControllerManager, index: u32) -> *physx_PxController #foreign physx_sys;

PxControllerManager_createController_mut :: (self: *physx_PxControllerManager, desc: *physx_PxControllerDesc) -> *physx_PxController #foreign physx_sys;

PxControllerManager_purgeControllers_mut :: (self: *physx_PxControllerManager) -> void #foreign physx_sys;

PxControllerManager_getRenderBuffer_mut :: (self: *physx_PxControllerManager) -> *physx_PxRenderBuffer #foreign physx_sys;

PxControllerManager_setDebugRenderingFlags_mut :: (self: *physx_PxControllerManager, flags: u32) -> void #foreign physx_sys;

PxControllerManager_getNbObstacleContexts :: (self: *physx_PxControllerManager) -> u32 #foreign physx_sys;

PxControllerManager_getObstacleContext_mut :: (self: *physx_PxControllerManager, index: u32) -> *physx_PxObstacleContext #foreign physx_sys;

PxControllerManager_createObstacleContext_mut :: (self: *physx_PxControllerManager) -> *physx_PxObstacleContext #foreign physx_sys;

PxControllerManager_computeInteractions_mut :: (self: *physx_PxControllerManager, elapsedTime: float, cctFilterCb: *physx_PxControllerFilterCallback) -> void #foreign physx_sys;

PxControllerManager_setTessellation_mut :: (self: *physx_PxControllerManager, flag: bool, maxEdgeLength: float) -> void #foreign physx_sys;

PxControllerManager_setOverlapRecoveryModule_mut :: (self: *physx_PxControllerManager, flag: bool) -> void #foreign physx_sys;

PxControllerManager_setPreciseSweeps_mut :: (self: *physx_PxControllerManager, flag: bool) -> void #foreign physx_sys;

PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut :: (self: *physx_PxControllerManager, flag: bool) -> void #foreign physx_sys;

PxControllerManager_shiftOrigin_mut :: (self: *physx_PxControllerManager, shift: *physx_Vector3) -> void #foreign physx_sys;

phys_PxCreateControllerManager :: (scene: *physx_PxScene, lockingEnabled: bool) -> *physx_PxControllerManager #foreign physx_sys;

PxDim3_new :: () -> physx_PxDim3 #foreign physx_sys;

PxSDFDesc_new :: () -> physx_PxSDFDesc #foreign physx_sys;

PxSDFDesc_isValid :: (self: *physx_PxSDFDesc) -> bool #foreign physx_sys;

PxConvexMeshDesc_new :: () -> physx_PxConvexMeshDesc #foreign physx_sys;

PxConvexMeshDesc_setToDefault_mut :: (self: *physx_PxConvexMeshDesc) -> void #foreign physx_sys;

PxConvexMeshDesc_isValid :: (self: *physx_PxConvexMeshDesc) -> bool #foreign physx_sys;

PxTriangleMeshDesc_new :: () -> physx_PxTriangleMeshDesc #foreign physx_sys;

PxTriangleMeshDesc_setToDefault_mut :: (self: *physx_PxTriangleMeshDesc) -> void #foreign physx_sys;

PxTriangleMeshDesc_isValid :: (self: *physx_PxTriangleMeshDesc) -> bool #foreign physx_sys;

PxTetrahedronMeshDesc_new :: () -> physx_PxTetrahedronMeshDesc #foreign physx_sys;

PxTetrahedronMeshDesc_isValid :: (self: *physx_PxTetrahedronMeshDesc) -> bool #foreign physx_sys;

PxSoftBodySimulationDataDesc_new :: () -> physx_PxSoftBodySimulationDataDesc #foreign physx_sys;

PxSoftBodySimulationDataDesc_isValid :: (self: *physx_PxSoftBodySimulationDataDesc) -> bool #foreign physx_sys;

PxBVH34MidphaseDesc_setToDefault_mut :: (self: *physx_PxBVH34MidphaseDesc) -> void #foreign physx_sys;

PxBVH34MidphaseDesc_isValid :: (self: *physx_PxBVH34MidphaseDesc) -> bool #foreign physx_sys;

PxMidphaseDesc_new :: () -> physx_PxMidphaseDesc #foreign physx_sys;

PxMidphaseDesc_getType :: (self: *physx_PxMidphaseDesc) -> s32 #foreign physx_sys;

PxMidphaseDesc_setToDefault_mut :: (self: *physx_PxMidphaseDesc, type: s32) -> void #foreign physx_sys;

PxMidphaseDesc_isValid :: (self: *physx_PxMidphaseDesc) -> bool #foreign physx_sys;

PxBVHDesc_new :: () -> physx_PxBVHDesc #foreign physx_sys;

PxBVHDesc_setToDefault_mut :: (self: *physx_PxBVHDesc) -> void #foreign physx_sys;

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

phys_PxComputeHullPolygons :: (params: *physx_PxCookingParams, mesh: *physx_PxSimpleTriangleMesh, inCallback: *physx_PxAllocatorCallback, nbVerts: *u32, vertices: **physx_Vector3, nbIndices: *u32, indices: **u32, nbPolygons: *u32, hullPolygons: **physx_PxHullPolygon) -> bool #foreign physx_sys;

phys_PxValidateTriangleMesh :: (params: *physx_PxCookingParams, desc: *physx_PxTriangleMeshDesc) -> bool #foreign physx_sys;

phys_PxCreateTriangleMesh :: (params: *physx_PxCookingParams, desc: *physx_PxTriangleMeshDesc, insertionCallback: *physx_PxInsertionCallback, condition: *s32) -> *physx_PxTriangleMesh #foreign physx_sys;

phys_PxCookTriangleMesh :: (params: *physx_PxCookingParams, desc: *physx_PxTriangleMeshDesc, stream: *physx_PxOutputStream, condition: *s32) -> bool #foreign physx_sys;

PxDefaultMemoryOutputStream_new_alloc :: (allocator: *physx_PxAllocatorCallback) -> *physx_PxDefaultMemoryOutputStream #foreign physx_sys;

PxDefaultMemoryOutputStream_delete :: (self: *physx_PxDefaultMemoryOutputStream) -> void #foreign physx_sys;

PxDefaultMemoryOutputStream_write_mut :: (self: *physx_PxDefaultMemoryOutputStream, src: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultMemoryOutputStream_getSize :: (self: *physx_PxDefaultMemoryOutputStream) -> u32 #foreign physx_sys;

PxDefaultMemoryOutputStream_getData :: (self: *physx_PxDefaultMemoryOutputStream) -> *u8 #foreign physx_sys;

PxDefaultMemoryInputData_new_alloc :: (data: *u8, length: u32) -> *physx_PxDefaultMemoryInputData #foreign physx_sys;

PxDefaultMemoryInputData_read_mut :: (self: *physx_PxDefaultMemoryInputData, dest: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultMemoryInputData_getLength :: (self: *physx_PxDefaultMemoryInputData) -> u32 #foreign physx_sys;

PxDefaultMemoryInputData_seek_mut :: (self: *physx_PxDefaultMemoryInputData, pos: u32) -> void #foreign physx_sys;

PxDefaultMemoryInputData_tell :: (self: *physx_PxDefaultMemoryInputData) -> u32 #foreign physx_sys;

PxDefaultFileOutputStream_new_alloc :: (name: *u8) -> *physx_PxDefaultFileOutputStream #foreign physx_sys;

PxDefaultFileOutputStream_delete :: (self: *physx_PxDefaultFileOutputStream) -> void #foreign physx_sys;

PxDefaultFileOutputStream_write_mut :: (self: *physx_PxDefaultFileOutputStream, src: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultFileOutputStream_isValid_mut :: (self: *physx_PxDefaultFileOutputStream) -> bool #foreign physx_sys;

PxDefaultFileInputData_new_alloc :: (name: *u8) -> *physx_PxDefaultFileInputData #foreign physx_sys;

PxDefaultFileInputData_delete :: (self: *physx_PxDefaultFileInputData) -> void #foreign physx_sys;

PxDefaultFileInputData_read_mut :: (self: *physx_PxDefaultFileInputData, dest: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultFileInputData_seek_mut :: (self: *physx_PxDefaultFileInputData, pos: u32) -> void #foreign physx_sys;

PxDefaultFileInputData_tell :: (self: *physx_PxDefaultFileInputData) -> u32 #foreign physx_sys;

PxDefaultFileInputData_getLength :: (self: *physx_PxDefaultFileInputData) -> u32 #foreign physx_sys;

PxDefaultFileInputData_isValid :: (self: *physx_PxDefaultFileInputData) -> bool #foreign physx_sys;

phys_platformAlignedAlloc :: (size: u64) -> *void #foreign physx_sys;

phys_platformAlignedFree :: (ptr: *void) -> void #foreign physx_sys;

PxDefaultAllocator_allocate_mut :: (self: *physx_PxDefaultAllocator, size: u64, anon_param1: *u8, anon_param2: *u8, anon_param3: s32) -> *void #foreign physx_sys;

PxDefaultAllocator_deallocate_mut :: (self: *physx_PxDefaultAllocator, ptr: *void) -> void #foreign physx_sys;

PxDefaultAllocator_delete :: (self: *physx_PxDefaultAllocator) -> void #foreign physx_sys;

PxJos32_setActors_mut :: (self: *physx_PxJos32, actor0: *physx_PxRigidActor, actor1: *physx_PxRigidActor) -> void #foreign physx_sys;

PxJos32_getActors :: (self: *physx_PxJos32, actor0: **physx_PxRigidActor, actor1: **physx_PxRigidActor) -> void #foreign physx_sys;

PxJos32_setLocalPose_mut :: (self: *physx_PxJos32, actor: s32, localPose: *physx_PxTransform) -> void #foreign physx_sys;

PxJos32_getLocalPose :: (self: *physx_PxJos32, actor: s32) -> physx_PxTransform #foreign physx_sys;

PxJos32_getRelativeTransform :: (self: *physx_PxJos32) -> physx_PxTransform #foreign physx_sys;

PxJos32_getRelativeLinearVelocity :: (self: *physx_PxJos32) -> physx_Vector3 #foreign physx_sys;

PxJos32_getRelativeAngularVelocity :: (self: *physx_PxJos32) -> physx_Vector3 #foreign physx_sys;

PxJos32_setBreakForce_mut :: (self: *physx_PxJos32, force: float, torque: float) -> void #foreign physx_sys;

PxJos32_getBreakForce :: (self: *physx_PxJos32, force: *float, torque: *float) -> void #foreign physx_sys;

PxJos32_setConstras32Flags_mut :: (self: *physx_PxJos32, flags: u16) -> void #foreign physx_sys;

PxJos32_setConstras32Flag_mut :: (self: *physx_PxJos32, flag: s32, value: bool) -> void #foreign physx_sys;

PxJos32_getConstras32Flags :: (self: *physx_PxJos32) -> u16 #foreign physx_sys;

PxJos32_setInvMassScale0_mut :: (self: *physx_PxJos32, invMassScale: float) -> void #foreign physx_sys;

PxJos32_getInvMassScale0 :: (self: *physx_PxJos32) -> float #foreign physx_sys;

PxJos32_setInvInertiaScale0_mut :: (self: *physx_PxJos32, invInertiaScale: float) -> void #foreign physx_sys;

PxJos32_getInvInertiaScale0 :: (self: *physx_PxJos32) -> float #foreign physx_sys;

PxJos32_setInvMassScale1_mut :: (self: *physx_PxJos32, invMassScale: float) -> void #foreign physx_sys;

PxJos32_getInvMassScale1 :: (self: *physx_PxJos32) -> float #foreign physx_sys;

PxJos32_setInvInertiaScale1_mut :: (self: *physx_PxJos32, invInertiaScale: float) -> void #foreign physx_sys;

PxJos32_getInvInertiaScale1 :: (self: *physx_PxJos32) -> float #foreign physx_sys;

PxJos32_getConstras32 :: (self: *physx_PxJos32) -> *physx_PxConstras32 #foreign physx_sys;

PxJos32_setName_mut :: (self: *physx_PxJos32, name: *u8) -> void #foreign physx_sys;

PxJos32_getName :: (self: *physx_PxJos32) -> *u8 #foreign physx_sys;

PxJos32_release_mut :: (self: *physx_PxJos32) -> void #foreign physx_sys;

PxJos32_getScene :: (self: *physx_PxJos32) -> *physx_PxScene #foreign physx_sys;

PxJos32_getBinaryMetaData :: (stream: *physx_PxOutputStream) -> void #foreign physx_sys;

PxSpring_new :: (stiffness_: float, damping_: float) -> physx_PxSpring #foreign physx_sys;

phys_PxSetJos32GlobalFrame :: (jos32: *physx_PxJos32, wsAnchor: *physx_Vector3, wsAxis: *physx_Vector3) -> void #foreign physx_sys;

phys_PxDistanceJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxDistanceJos32 #foreign physx_sys;

PxDistanceJos32_getDistance :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setMinDistance_mut :: (self: *physx_PxDistanceJos32, distance: float) -> void #foreign physx_sys;

PxDistanceJos32_getMinDistance :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setMaxDistance_mut :: (self: *physx_PxDistanceJos32, distance: float) -> void #foreign physx_sys;

PxDistanceJos32_getMaxDistance :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setTolerance_mut :: (self: *physx_PxDistanceJos32, tolerance: float) -> void #foreign physx_sys;

PxDistanceJos32_getTolerance :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setStiffness_mut :: (self: *physx_PxDistanceJos32, stiffness: float) -> void #foreign physx_sys;

PxDistanceJos32_getStiffness :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setDamping_mut :: (self: *physx_PxDistanceJos32, damping: float) -> void #foreign physx_sys;

PxDistanceJos32_getDamping :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setContactDistance_mut :: (self: *physx_PxDistanceJos32, contactDistance: float) -> void #foreign physx_sys;

PxDistanceJos32_getContactDistance :: (self: *physx_PxDistanceJos32) -> float #foreign physx_sys;

PxDistanceJos32_setDistanceJos32Flags_mut :: (self: *physx_PxDistanceJos32, flags: u16) -> void #foreign physx_sys;

PxDistanceJos32_setDistanceJos32Flag_mut :: (self: *physx_PxDistanceJos32, flag: s32, value: bool) -> void #foreign physx_sys;

PxDistanceJos32_getDistanceJos32Flags :: (self: *physx_PxDistanceJos32) -> u16 #foreign physx_sys;

PxDistanceJos32_getConcreteTypeName :: (self: *physx_PxDistanceJos32) -> *u8 #foreign physx_sys;

phys_PxContactJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxContactJos32 #foreign physx_sys;

PxJacobianRow_new :: () -> physx_PxJacobianRow #foreign physx_sys;

PxJacobianRow_new_1 :: (lin0: *physx_Vector3, lin1: *physx_Vector3, ang0: *physx_Vector3, ang1: *physx_Vector3) -> physx_PxJacobianRow #foreign physx_sys;

PxContactJos32_setContact_mut :: (self: *physx_PxContactJos32, contact: *physx_Vector3) -> void #foreign physx_sys;

PxContactJos32_setContactNormal_mut :: (self: *physx_PxContactJos32, contactNormal: *physx_Vector3) -> void #foreign physx_sys;

PxContactJos32_setPenetration_mut :: (self: *physx_PxContactJos32, penetration: float) -> void #foreign physx_sys;

PxContactJos32_getContact :: (self: *physx_PxContactJos32) -> physx_Vector3 #foreign physx_sys;

PxContactJos32_getContactNormal :: (self: *physx_PxContactJos32) -> physx_Vector3 #foreign physx_sys;

PxContactJos32_getPenetration :: (self: *physx_PxContactJos32) -> float #foreign physx_sys;

PxContactJos32_getRestitution :: (self: *physx_PxContactJos32) -> float #foreign physx_sys;

PxContactJos32_setRestitution_mut :: (self: *physx_PxContactJos32, restitution: float) -> void #foreign physx_sys;

PxContactJos32_getBounceThreshold :: (self: *physx_PxContactJos32) -> float #foreign physx_sys;

PxContactJos32_setBounceThreshold_mut :: (self: *physx_PxContactJos32, bounceThreshold: float) -> void #foreign physx_sys;

PxContactJos32_getConcreteTypeName :: (self: *physx_PxContactJos32) -> *u8 #foreign physx_sys;

PxContactJos32_computeJacobians :: (self: *physx_PxContactJos32, jacobian: *physx_PxJacobianRow) -> void #foreign physx_sys;

PxContactJos32_getNbJacobianRows :: (self: *physx_PxContactJos32) -> u32 #foreign physx_sys;

phys_PxFixedJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxFixedJos32 #foreign physx_sys;

PxFixedJos32_getConcreteTypeName :: (self: *physx_PxFixedJos32) -> *u8 #foreign physx_sys;

PxJos32LimitParameters_new_alloc :: () -> *physx_PxJos32LimitParameters #foreign physx_sys;

PxJos32LimitParameters_isValid :: (self: *physx_PxJos32LimitParameters) -> bool #foreign physx_sys;

PxJos32LimitParameters_isSoft :: (self: *physx_PxJos32LimitParameters) -> bool #foreign physx_sys;

PxJos32LinearLimit_new :: (scale: *physx_PxTolerancesScale, extent: float, contactDist_deprecated: float) -> physx_PxJos32LinearLimit #foreign physx_sys;

PxJos32LinearLimit_new_1 :: (extent: float, spring: *physx_PxSpring) -> physx_PxJos32LinearLimit #foreign physx_sys;

PxJos32LinearLimit_isValid :: (self: *physx_PxJos32LinearLimit) -> bool #foreign physx_sys;

PxJos32LinearLimit_delete :: (self: *physx_PxJos32LinearLimit) -> void #foreign physx_sys;

PxJos32LinearLimitPair_new :: (scale: *physx_PxTolerancesScale, lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> physx_PxJos32LinearLimitPair #foreign physx_sys;

PxJos32LinearLimitPair_new_1 :: (lowerLimit: float, upperLimit: float, spring: *physx_PxSpring) -> physx_PxJos32LinearLimitPair #foreign physx_sys;

PxJos32LinearLimitPair_isValid :: (self: *physx_PxJos32LinearLimitPair) -> bool #foreign physx_sys;

PxJos32LinearLimitPair_delete :: (self: *physx_PxJos32LinearLimitPair) -> void #foreign physx_sys;

PxJos32AngularLimitPair_new :: (lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> physx_PxJos32AngularLimitPair #foreign physx_sys;

PxJos32AngularLimitPair_new_1 :: (lowerLimit: float, upperLimit: float, spring: *physx_PxSpring) -> physx_PxJos32AngularLimitPair #foreign physx_sys;

PxJos32AngularLimitPair_isValid :: (self: *physx_PxJos32AngularLimitPair) -> bool #foreign physx_sys;

PxJos32AngularLimitPair_delete :: (self: *physx_PxJos32AngularLimitPair) -> void #foreign physx_sys;

PxJos32LimitCone_new :: (yLimitAngle: float, zLimitAngle: float, contactDist_deprecated: float) -> physx_PxJos32LimitCone #foreign physx_sys;

PxJos32LimitCone_new_1 :: (yLimitAngle: float, zLimitAngle: float, spring: *physx_PxSpring) -> physx_PxJos32LimitCone #foreign physx_sys;

PxJos32LimitCone_isValid :: (self: *physx_PxJos32LimitCone) -> bool #foreign physx_sys;

PxJos32LimitCone_delete :: (self: *physx_PxJos32LimitCone) -> void #foreign physx_sys;

PxJos32LimitPyramid_new :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, contactDist_deprecated: float) -> physx_PxJos32LimitPyramid #foreign physx_sys;

PxJos32LimitPyramid_new_1 :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, spring: *physx_PxSpring) -> physx_PxJos32LimitPyramid #foreign physx_sys;

PxJos32LimitPyramid_isValid :: (self: *physx_PxJos32LimitPyramid) -> bool #foreign physx_sys;

PxJos32LimitPyramid_delete :: (self: *physx_PxJos32LimitPyramid) -> void #foreign physx_sys;

phys_PxPrismaticJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxPrismaticJos32 #foreign physx_sys;

PxPrismaticJos32_getPosition :: (self: *physx_PxPrismaticJos32) -> float #foreign physx_sys;

PxPrismaticJos32_getVelocity :: (self: *physx_PxPrismaticJos32) -> float #foreign physx_sys;

PxPrismaticJos32_setLimit_mut :: (self: *physx_PxPrismaticJos32, anon_param0: *physx_PxJos32LinearLimitPair) -> void #foreign physx_sys;

PxPrismaticJos32_getLimit :: (self: *physx_PxPrismaticJos32) -> physx_PxJos32LinearLimitPair #foreign physx_sys;

PxPrismaticJos32_setPrismaticJos32Flags_mut :: (self: *physx_PxPrismaticJos32, flags: u16) -> void #foreign physx_sys;

PxPrismaticJos32_setPrismaticJos32Flag_mut :: (self: *physx_PxPrismaticJos32, flag: s32, value: bool) -> void #foreign physx_sys;

PxPrismaticJos32_getPrismaticJos32Flags :: (self: *physx_PxPrismaticJos32) -> u16 #foreign physx_sys;

PxPrismaticJos32_getConcreteTypeName :: (self: *physx_PxPrismaticJos32) -> *u8 #foreign physx_sys;

phys_PxRevoluteJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxRevoluteJos32 #foreign physx_sys;

PxRevoluteJos32_getAngle :: (self: *physx_PxRevoluteJos32) -> float #foreign physx_sys;

PxRevoluteJos32_getVelocity :: (self: *physx_PxRevoluteJos32) -> float #foreign physx_sys;

PxRevoluteJos32_setLimit_mut :: (self: *physx_PxRevoluteJos32, limits: *physx_PxJos32AngularLimitPair) -> void #foreign physx_sys;

PxRevoluteJos32_getLimit :: (self: *physx_PxRevoluteJos32) -> physx_PxJos32AngularLimitPair #foreign physx_sys;

PxRevoluteJos32_setDriveVelocity_mut :: (self: *physx_PxRevoluteJos32, velocity: float, autowake: bool) -> void #foreign physx_sys;

PxRevoluteJos32_getDriveVelocity :: (self: *physx_PxRevoluteJos32) -> float #foreign physx_sys;

PxRevoluteJos32_setDriveForceLimit_mut :: (self: *physx_PxRevoluteJos32, limit: float) -> void #foreign physx_sys;

PxRevoluteJos32_getDriveForceLimit :: (self: *physx_PxRevoluteJos32) -> float #foreign physx_sys;

PxRevoluteJos32_setDriveGearRatio_mut :: (self: *physx_PxRevoluteJos32, ratio: float) -> void #foreign physx_sys;

PxRevoluteJos32_getDriveGearRatio :: (self: *physx_PxRevoluteJos32) -> float #foreign physx_sys;

PxRevoluteJos32_setRevoluteJos32Flags_mut :: (self: *physx_PxRevoluteJos32, flags: u16) -> void #foreign physx_sys;

PxRevoluteJos32_setRevoluteJos32Flag_mut :: (self: *physx_PxRevoluteJos32, flag: s32, value: bool) -> void #foreign physx_sys;

PxRevoluteJos32_getRevoluteJos32Flags :: (self: *physx_PxRevoluteJos32) -> u16 #foreign physx_sys;

PxRevoluteJos32_getConcreteTypeName :: (self: *physx_PxRevoluteJos32) -> *u8 #foreign physx_sys;

phys_PxSphericalJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxSphericalJos32 #foreign physx_sys;

PxSphericalJos32_getLimitCone :: (self: *physx_PxSphericalJos32) -> physx_PxJos32LimitCone #foreign physx_sys;

PxSphericalJos32_setLimitCone_mut :: (self: *physx_PxSphericalJos32, limit: *physx_PxJos32LimitCone) -> void #foreign physx_sys;

PxSphericalJos32_getSwingYAngle :: (self: *physx_PxSphericalJos32) -> float #foreign physx_sys;

PxSphericalJos32_getSwingZAngle :: (self: *physx_PxSphericalJos32) -> float #foreign physx_sys;

PxSphericalJos32_setSphericalJos32Flags_mut :: (self: *physx_PxSphericalJos32, flags: u16) -> void #foreign physx_sys;

PxSphericalJos32_setSphericalJos32Flag_mut :: (self: *physx_PxSphericalJos32, flag: s32, value: bool) -> void #foreign physx_sys;

PxSphericalJos32_getSphericalJos32Flags :: (self: *physx_PxSphericalJos32) -> u16 #foreign physx_sys;

PxSphericalJos32_getConcreteTypeName :: (self: *physx_PxSphericalJos32) -> *u8 #foreign physx_sys;

phys_PxD6Jos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxD6Jos32 #foreign physx_sys;

PxD6Jos32Drive_new :: () -> physx_PxD6Jos32Drive #foreign physx_sys;

PxD6Jos32Drive_new_1 :: (driveStiffness: float, driveDamping: float, driveForceLimit: float, isAcceleration: bool) -> physx_PxD6Jos32Drive #foreign physx_sys;

PxD6Jos32Drive_isValid :: (self: *physx_PxD6Jos32Drive) -> bool #foreign physx_sys;

PxD6Jos32_setMotion_mut :: (self: *physx_PxD6Jos32, axis: s32, type: s32) -> void #foreign physx_sys;

PxD6Jos32_getMotion :: (self: *physx_PxD6Jos32, axis: s32) -> s32 #foreign physx_sys;

PxD6Jos32_getTwistAngle :: (self: *physx_PxD6Jos32) -> float #foreign physx_sys;

PxD6Jos32_getSwingYAngle :: (self: *physx_PxD6Jos32) -> float #foreign physx_sys;

PxD6Jos32_getSwingZAngle :: (self: *physx_PxD6Jos32) -> float #foreign physx_sys;

PxD6Jos32_setDistanceLimit_mut :: (self: *physx_PxD6Jos32, limit: *physx_PxJos32LinearLimit) -> void #foreign physx_sys;

PxD6Jos32_getDistanceLimit :: (self: *physx_PxD6Jos32) -> physx_PxJos32LinearLimit #foreign physx_sys;

PxD6Jos32_setLinearLimit_mut :: (self: *physx_PxD6Jos32, axis: s32, limit: *physx_PxJos32LinearLimitPair) -> void #foreign physx_sys;

PxD6Jos32_getLinearLimit :: (self: *physx_PxD6Jos32, axis: s32) -> physx_PxJos32LinearLimitPair #foreign physx_sys;

PxD6Jos32_setTwistLimit_mut :: (self: *physx_PxD6Jos32, limit: *physx_PxJos32AngularLimitPair) -> void #foreign physx_sys;

PxD6Jos32_getTwistLimit :: (self: *physx_PxD6Jos32) -> physx_PxJos32AngularLimitPair #foreign physx_sys;

PxD6Jos32_setSwingLimit_mut :: (self: *physx_PxD6Jos32, limit: *physx_PxJos32LimitCone) -> void #foreign physx_sys;

PxD6Jos32_getSwingLimit :: (self: *physx_PxD6Jos32) -> physx_PxJos32LimitCone #foreign physx_sys;

PxD6Jos32_setPyramidSwingLimit_mut :: (self: *physx_PxD6Jos32, limit: *physx_PxJos32LimitPyramid) -> void #foreign physx_sys;

PxD6Jos32_getPyramidSwingLimit :: (self: *physx_PxD6Jos32) -> physx_PxJos32LimitPyramid #foreign physx_sys;

PxD6Jos32_setDrive_mut :: (self: *physx_PxD6Jos32, index: s32, drive: *physx_PxD6Jos32Drive) -> void #foreign physx_sys;

PxD6Jos32_getDrive :: (self: *physx_PxD6Jos32, index: s32) -> physx_PxD6Jos32Drive #foreign physx_sys;

PxD6Jos32_setDrivePosition_mut :: (self: *physx_PxD6Jos32, pose: *physx_PxTransform, autowake: bool) -> void #foreign physx_sys;

PxD6Jos32_getDrivePosition :: (self: *physx_PxD6Jos32) -> physx_PxTransform #foreign physx_sys;

PxD6Jos32_setDriveVelocity_mut :: (self: *physx_PxD6Jos32, linear: *physx_Vector3, angular: *physx_Vector3, autowake: bool) -> void #foreign physx_sys;

PxD6Jos32_getDriveVelocity :: (self: *physx_PxD6Jos32, linear: *physx_Vector3, angular: *physx_Vector3) -> void #foreign physx_sys;

PxD6Jos32_setProjectionLinearTolerance_mut :: (self: *physx_PxD6Jos32, tolerance: float) -> void #foreign physx_sys;

PxD6Jos32_getProjectionLinearTolerance :: (self: *physx_PxD6Jos32) -> float #foreign physx_sys;

PxD6Jos32_setProjectionAngularTolerance_mut :: (self: *physx_PxD6Jos32, tolerance: float) -> void #foreign physx_sys;

PxD6Jos32_getProjectionAngularTolerance :: (self: *physx_PxD6Jos32) -> float #foreign physx_sys;

PxD6Jos32_getConcreteTypeName :: (self: *physx_PxD6Jos32) -> *u8 #foreign physx_sys;

phys_PxGearJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxGearJos32 #foreign physx_sys;

PxGearJos32_setHinges_mut :: (self: *physx_PxGearJos32, hinge0: *physx_PxBase, hinge1: *physx_PxBase) -> bool #foreign physx_sys;

PxGearJos32_setGearRatio_mut :: (self: *physx_PxGearJos32, ratio: float) -> void #foreign physx_sys;

PxGearJos32_getGearRatio :: (self: *physx_PxGearJos32) -> float #foreign physx_sys;

PxGearJos32_getConcreteTypeName :: (self: *physx_PxGearJos32) -> *u8 #foreign physx_sys;

phys_PxRackAndPinionJos32Create :: (physics: *physx_PxPhysics, actor0: *physx_PxRigidActor, localFrame0: *physx_PxTransform, actor1: *physx_PxRigidActor, localFrame1: *physx_PxTransform) -> *physx_PxRackAndPinionJos32 #foreign physx_sys;

PxRackAndPinionJos32_setJos32s_mut :: (self: *physx_PxRackAndPinionJos32, hinge: *physx_PxBase, prismatic: *physx_PxBase) -> bool #foreign physx_sys;

PxRackAndPinionJos32_setRatio_mut :: (self: *physx_PxRackAndPinionJos32, ratio: float) -> void #foreign physx_sys;

PxRackAndPinionJos32_getRatio :: (self: *physx_PxRackAndPinionJos32) -> float #foreign physx_sys;

PxRackAndPinionJos32_setData_mut :: (self: *physx_PxRackAndPinionJos32, nbRackTeeth: u32, nbPinionTeeth: u32, rackLength: float) -> bool #foreign physx_sys;

PxRackAndPinionJos32_getConcreteTypeName :: (self: *physx_PxRackAndPinionJos32) -> *u8 #foreign physx_sys;

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

PxDefaultErrorCallback_reportError_mut :: (self: *physx_PxDefaultErrorCallback, code: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys;

PxRigidActorExt_createExclusiveShape :: (actor: *physx_PxRigidActor, geometry: *physx_PxGeometry, materials: **physx_PxMaterial, materialCount: u16, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys;

PxRigidActorExt_createExclusiveShape_1 :: (actor: *physx_PxRigidActor, geometry: *physx_PxGeometry, material: *physx_PxMaterial, shapeFlags: u8) -> *physx_PxShape #foreign physx_sys;

PxRigidActorExt_getRigidActorShapeLocalBoundsList :: (actor: *physx_PxRigidActor, numBounds: *u32) -> *physx_PxBounds3 #foreign physx_sys;

PxRigidActorExt_createBVHFromActor :: (physics: *physx_PxPhysics, actor: *physx_PxRigidActor) -> *physx_PxBVH #foreign physx_sys;

PxMassProperties_new :: () -> physx_PxMassProperties #foreign physx_sys;

PxMassProperties_new_1 :: (m: float, inertiaT: *physx_Matrix3, com: *physx_Vector3) -> physx_PxMassProperties #foreign physx_sys;

PxMassProperties_new_2 :: (geometry: *physx_PxGeometry) -> physx_PxMassProperties #foreign physx_sys;

PxMassProperties_translate_mut :: (self: *physx_PxMassProperties, t: *physx_Vector3) -> void #foreign physx_sys;

PxMassProperties_getMassSpaceInertia :: (inertia: *physx_Matrix3, massFrame: *physx_PxQuat) -> physx_Vector3 #foreign physx_sys;

PxMassProperties_translateInertia :: (inertia: *physx_Matrix3, mass: float, t: *physx_Vector3) -> physx_Matrix3 #foreign physx_sys;

PxMassProperties_rotateInertia :: (inertia: *physx_Matrix3, q: *physx_PxQuat) -> physx_Matrix3 #foreign physx_sys;

PxMassProperties_scaleInertia :: (inertia: *physx_Matrix3, scaleRotation: *physx_PxQuat, scale: *physx_Vector3) -> physx_Matrix3 #foreign physx_sys;

PxMassProperties_sum :: (props: *physx_PxMassProperties, transforms: *physx_PxTransform, count: u32) -> physx_PxMassProperties #foreign physx_sys;

PxRigidBodyExt_updateMassAndInertia :: (body: *physx_PxRigidBody, shapeDensities: *float, shapeDensityCount: u32, massLocalPose: *physx_Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_updateMassAndInertia_1 :: (body: *physx_PxRigidBody, density: float, massLocalPose: *physx_Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_setMassAndUpdateInertia :: (body: *physx_PxRigidBody, shapeMasses: *float, shapeMassCount: u32, massLocalPose: *physx_Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_setMassAndUpdateInertia_1 :: (body: *physx_PxRigidBody, mass: float, massLocalPose: *physx_Vector3, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_computeMassPropertiesFromShapes :: (shapes: **physx_PxShape, shapeCount: u32) -> physx_PxMassProperties #foreign physx_sys;

PxRigidBodyExt_addForceAtPos :: (body: *physx_PxRigidBody, force: *physx_Vector3, pos: *physx_Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addForceAtLocalPos :: (body: *physx_PxRigidBody, force: *physx_Vector3, pos: *physx_Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addLocalForceAtPos :: (body: *physx_PxRigidBody, force: *physx_Vector3, pos: *physx_Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addLocalForceAtLocalPos :: (body: *physx_PxRigidBody, force: *physx_Vector3, pos: *physx_Vector3, mode: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_getVelocityAtPos :: (body: *physx_PxRigidBody, pos: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxRigidBodyExt_getLocalVelocityAtLocalPos :: (body: *physx_PxRigidBody, pos: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxRigidBodyExt_getVelocityAtOffset :: (body: *physx_PxRigidBody, pos: *physx_Vector3) -> physx_Vector3 #foreign physx_sys;

PxRigidBodyExt_computeVelocityDeltaFromImpulse :: (body: *physx_PxRigidBody, impulsiveForce: *physx_Vector3, impulsiveTorque: *physx_Vector3, deltaLinearVelocity: *physx_Vector3, deltaAngularVelocity: *physx_Vector3) -> void #foreign physx_sys;

PxRigidBodyExt_computeVelocityDeltaFromImpulse_1 :: (body: *physx_PxRigidBody, globalPose: *physx_PxTransform, pos32: *physx_Vector3, impulse: *physx_Vector3, invMassScale: float, invInertiaScale: float, deltaLinearVelocity: *physx_Vector3, deltaAngularVelocity: *physx_Vector3) -> void #foreign physx_sys;

PxRigidBodyExt_computeLinearAngularImpulse :: (body: *physx_PxRigidBody, globalPose: *physx_PxTransform, pos32: *physx_Vector3, impulse: *physx_Vector3, invMassScale: float, invInertiaScale: float, linearImpulse: *physx_Vector3, angularImpulse: *physx_Vector3) -> void #foreign physx_sys;

PxRigidBodyExt_linearSweepSingle :: (body: *physx_PxRigidBody, scene: *physx_PxScene, unitDir: *physx_Vector3, distance: float, outputFlags: u16, closestHit: *physx_PxSweepHit, shapeIndex: *u32, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> bool #foreign physx_sys;

PxRigidBodyExt_linearSweepMultiple :: (body: *physx_PxRigidBody, scene: *physx_PxScene, unitDir: *physx_Vector3, distance: float, outputFlags: u16, touchHitBuffer: *physx_PxSweepHit, touchHitShapeIndices: *u32, touchHitBufferSize: u32, block: *physx_PxSweepHit, blockingShapeIndex: *s32, overflow: *bool, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> u32 #foreign physx_sys;

PxShapeExt_getGlobalPose :: (shape: *physx_PxShape, actor: *physx_PxRigidActor) -> physx_PxTransform #foreign physx_sys;

PxShapeExt_raycast :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, rayOrigin: *physx_Vector3, rayDir: *physx_Vector3, maxDist: float, hitFlags: u16, maxHits: u32, rayHits: *physx_PxRaycastHit) -> u32 #foreign physx_sys;

PxShapeExt_overlap :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, otherGeom: *physx_PxGeometry, otherGeomPose: *physx_PxTransform) -> bool #foreign physx_sys;

PxShapeExt_sweep :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, unitDir: *physx_Vector3, distance: float, otherGeom: *physx_PxGeometry, otherGeomPose: *physx_PxTransform, sweepHit: *physx_PxSweepHit, hitFlags: u16) -> bool #foreign physx_sys;

PxShapeExt_getWorldBounds :: (shape: *physx_PxShape, actor: *physx_PxRigidActor, inflation: float) -> physx_PxBounds3 #foreign physx_sys;

PxMeshOverlapUtil_new_alloc :: () -> *physx_PxMeshOverlapUtil #foreign physx_sys;

PxMeshOverlapUtil_delete :: (self: *physx_PxMeshOverlapUtil) -> void #foreign physx_sys;

PxMeshOverlapUtil_findOverlap_mut :: (self: *physx_PxMeshOverlapUtil, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, meshGeom: *physx_PxTriangleMeshGeometry, meshPose: *physx_PxTransform) -> u32 #foreign physx_sys;

PxMeshOverlapUtil_findOverlap_mut_1 :: (self: *physx_PxMeshOverlapUtil, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, hfGeom: *physx_PxHeightFieldGeometry, hfPose: *physx_PxTransform) -> u32 #foreign physx_sys;

PxMeshOverlapUtil_getResults :: (self: *physx_PxMeshOverlapUtil) -> *u32 #foreign physx_sys;

PxMeshOverlapUtil_getNbResults :: (self: *physx_PxMeshOverlapUtil) -> u32 #foreign physx_sys;

phys_PxComputeTriangleMeshPenetration :: (direction: *physx_Vector3, depth: *float, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, meshGeom: *physx_PxTriangleMeshGeometry, meshPose: *physx_PxTransform, maxIter: u32, usedIter: *u32) -> bool #foreign physx_sys;

phys_PxComputeHeightFieldPenetration :: (direction: *physx_Vector3, depth: *float, geom: *physx_PxGeometry, geomPose: *physx_PxTransform, heightFieldGeom: *physx_PxHeightFieldGeometry, heightFieldPose: *physx_PxTransform, maxIter: u32, usedIter: *u32) -> bool #foreign physx_sys;

PxXmlMiscParameter_new :: () -> physx_PxXmlMiscParameter #foreign physx_sys;

PxXmlMiscParameter_new_1 :: (inUpVector: *physx_Vector3, inScale: physx_PxTolerancesScale) -> physx_PxXmlMiscParameter #foreign physx_sys;

PxSerialization_isSerializable :: (collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, externalReferences: *physx_PxCollection) -> bool #foreign physx_sys;

PxSerialization_complete :: (collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, exceptFor: *physx_PxCollection, followJos32s: bool) -> void #foreign physx_sys;

PxSerialization_createSerialObjectIds :: (collection: *physx_PxCollection, base: u64) -> void #foreign physx_sys;

PxSerialization_createCollectionFromXml :: (inputData: *physx_PxInputData, cooking: *physx_PxCooking, sr: *physx_PxSerializationRegistry, externalRefs: *physx_PxCollection, stringTable: *physx_PxStringTable, outArgs: *physx_PxXmlMiscParameter) -> *physx_PxCollection #foreign physx_sys;

PxSerialization_createCollectionFromBinary :: (memBlock: *void, sr: *physx_PxSerializationRegistry, externalRefs: *physx_PxCollection) -> *physx_PxCollection #foreign physx_sys;

PxSerialization_serializeCollectionToXml :: (outputStream: *physx_PxOutputStream, collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, cooking: *physx_PxCooking, externalRefs: *physx_PxCollection, inArgs: *physx_PxXmlMiscParameter) -> bool #foreign physx_sys;

PxSerialization_serializeCollectionToBinary :: (outputStream: *physx_PxOutputStream, collection: *physx_PxCollection, sr: *physx_PxSerializationRegistry, externalRefs: *physx_PxCollection, exportNames: bool) -> bool #foreign physx_sys;

PxSerialization_createSerializationRegistry :: (physics: *physx_PxPhysics) -> *physx_PxSerializationRegistry #foreign physx_sys;

PxDefaultCpuDispatcher_release_mut :: (self: *physx_PxDefaultCpuDispatcher) -> void #foreign physx_sys;

PxDefaultCpuDispatcher_setRunProfiled_mut :: (self: *physx_PxDefaultCpuDispatcher, runProfiled: bool) -> void #foreign physx_sys;

PxDefaultCpuDispatcher_getRunProfiled :: (self: *physx_PxDefaultCpuDispatcher) -> bool #foreign physx_sys;

phys_PxDefaultCpuDispatcherCreate :: (numThreads: u32, affinityMasks: *u32, mode: s32, yieldProcessorCount: u32) -> *physx_PxDefaultCpuDispatcher #foreign physx_sys;

phys_PxBuildSmoothNormals :: (nbTris: u32, nbVerts: u32, verts: *physx_Vector3, dFaces: *u32, wFaces: *u16, normals: *physx_Vector3, flip: bool) -> bool #foreign physx_sys;

phys_PxCreateDynamic :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, geometry: *physx_PxGeometry, material: *physx_PxMaterial, density: float, shapeOffset: *physx_PxTransform) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateDynamic_1 :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, shape: *physx_PxShape, density: float) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateKinematic :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, geometry: *physx_PxGeometry, material: *physx_PxMaterial, density: float, shapeOffset: *physx_PxTransform) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateKinematic_1 :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, shape: *physx_PxShape, density: float) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreateStatic :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, geometry: *physx_PxGeometry, material: *physx_PxMaterial, shapeOffset: *physx_PxTransform) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxCreateStatic_1 :: (sdk: *physx_PxPhysics, transform: *physx_PxTransform, shape: *physx_PxShape) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxCloneShape :: (physicsSDK: *physx_PxPhysics, shape: *physx_PxShape, isExclusive: bool) -> *physx_PxShape #foreign physx_sys;

phys_PxCloneStatic :: (physicsSDK: *physx_PxPhysics, transform: *physx_PxTransform, actor: *physx_PxRigidActor) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxCloneDynamic :: (physicsSDK: *physx_PxPhysics, transform: *physx_PxTransform, body: *physx_PxRigidDynamic) -> *physx_PxRigidDynamic #foreign physx_sys;

phys_PxCreatePlane :: (sdk: *physx_PxPhysics, plane: *physx_PxPlane, material: *physx_PxMaterial) -> *physx_PxRigidStatic #foreign physx_sys;

phys_PxScaleRigidActor :: (actor: *physx_PxRigidActor, scale: float, scaleMassProps: bool) -> void #foreign physx_sys;

PxStringTableExt_createStringTable :: (inAllocator: *physx_PxAllocatorCallback) -> *physx_PxStringTable #foreign physx_sys;

PxBroadPhaseExt_createRegionsFromWorldBounds :: (regions: *physx_PxBounds3, globalBounds: *physx_PxBounds3, nbSubdiv: u32, upAxis: u32) -> u32 #foreign physx_sys;

PxSceneQueryExt_raycastAny :: (scene: *physx_PxScene, origin: *physx_Vector3, unitDir: *physx_Vector3, distance: float, hit: *physx_PxQueryHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache) -> bool #foreign physx_sys;

PxSceneQueryExt_raycastSingle :: (scene: *physx_PxScene, origin: *physx_Vector3, unitDir: *physx_Vector3, distance: float, outputFlags: u16, hit: *physx_PxRaycastHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache) -> bool #foreign physx_sys;

PxSceneQueryExt_raycastMultiple :: (scene: *physx_PxScene, origin: *physx_Vector3, unitDir: *physx_Vector3, distance: float, outputFlags: u16, hitBuffer: *physx_PxRaycastHit, hitBufferSize: u32, blockingHit: *bool, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache) -> s32 #foreign physx_sys;

PxSceneQueryExt_sweepAny :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *physx_Vector3, distance: float, queryFlags: u16, hit: *physx_PxQueryHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> bool #foreign physx_sys;

PxSceneQueryExt_sweepSingle :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *physx_Vector3, distance: float, outputFlags: u16, hit: *physx_PxSweepHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> bool #foreign physx_sys;

PxSceneQueryExt_sweepMultiple :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *physx_Vector3, distance: float, outputFlags: u16, hitBuffer: *physx_PxSweepHit, hitBufferSize: u32, blockingHit: *bool, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback, cache: *physx_PxQueryCache, inflation: float) -> s32 #foreign physx_sys;

PxSceneQueryExt_overlapMultiple :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, hitBuffer: *physx_PxOverlapHit, hitBufferSize: u32, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback) -> s32 #foreign physx_sys;

PxSceneQueryExt_overlapAny :: (scene: *physx_PxScene, geometry: *physx_PxGeometry, pose: *physx_PxTransform, hit: *physx_PxOverlapHit, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback) -> bool #foreign physx_sys;

PxBatchQueryExt_release_mut :: (self: *physx_PxBatchQueryExt) -> void #foreign physx_sys;

PxBatchQueryExt_raycast_mut :: (self: *physx_PxBatchQueryExt, origin: *physx_Vector3, unitDir: *physx_Vector3, distance: float, maxNbTouches: u16, hitFlags: u16, filterData: *physx_PxQueryFilterData, cache: *physx_PxQueryCache) -> *physx_PxRaycastBuffer #foreign physx_sys;

PxBatchQueryExt_sweep_mut :: (self: *physx_PxBatchQueryExt, geometry: *physx_PxGeometry, pose: *physx_PxTransform, unitDir: *physx_Vector3, distance: float, maxNbTouches: u16, hitFlags: u16, filterData: *physx_PxQueryFilterData, cache: *physx_PxQueryCache, inflation: float) -> *physx_PxSweepBuffer #foreign physx_sys;

PxBatchQueryExt_overlap_mut :: (self: *physx_PxBatchQueryExt, geometry: *physx_PxGeometry, pose: *physx_PxTransform, maxNbTouches: u16, filterData: *physx_PxQueryFilterData, cache: *physx_PxQueryCache) -> *physx_PxOverlapBuffer #foreign physx_sys;

PxBatchQueryExt_execute_mut :: (self: *physx_PxBatchQueryExt) -> void #foreign physx_sys;

phys_PxCreateBatchQueryExt :: (scene: *physx_PxScene, queryFilterCallback: *physx_PxQueryFilterCallback, maxNbRaycasts: u32, maxNbRaycastTouches: u32, maxNbSweeps: u32, maxNbSweepTouches: u32, maxNbOverlaps: u32, maxNbOverlapTouches: u32) -> *physx_PxBatchQueryExt #foreign physx_sys;

phys_PxCreateBatchQueryExt_1 :: (scene: *physx_PxScene, queryFilterCallback: *physx_PxQueryFilterCallback, raycastBuffers: *physx_PxRaycastBuffer, maxNbRaycasts: u32, raycastTouches: *physx_PxRaycastHit, maxNbRaycastTouches: u32, sweepBuffers: *physx_PxSweepBuffer, maxNbSweeps: u32, sweepTouches: *physx_PxSweepHit, maxNbSweepTouches: u32, overlapBuffers: *physx_PxOverlapBuffer, maxNbOverlaps: u32, overlapTouches: *physx_PxOverlapHit, maxNbOverlapTouches: u32) -> *physx_PxBatchQueryExt #foreign physx_sys;

phys_PxCreateExternalSceneQuerySystem :: (desc: *physx_PxSceneQueryDesc, contextID: u64) -> *physx_PxSceneQuerySystem #foreign physx_sys;

PxCustomSceneQuerySystem_addPruner_mut :: (self: *physx_PxCustomSceneQuerySystem, primaryType: s32, secondaryType: s32, preallocated: u32) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystem_startCustomBuildstep_mut :: (self: *physx_PxCustomSceneQuerySystem) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystem_customBuildstep_mut :: (self: *physx_PxCustomSceneQuerySystem, index: u32) -> void #foreign physx_sys;

PxCustomSceneQuerySystem_finishCustomBuildstep_mut :: (self: *physx_PxCustomSceneQuerySystem) -> void #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_delete :: (self: *physx_PxCustomSceneQuerySystemAdapter) -> void #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_getPrunerIndex :: (self: *physx_PxCustomSceneQuerySystemAdapter, actor: *physx_PxRigidActor, shape: *physx_PxShape) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_processPruner :: (self: *physx_PxCustomSceneQuerySystemAdapter, prunerIndex: u32, context_pod: *physx_PxQueryThreadContext, filterData: *physx_PxQueryFilterData, filterCall: *physx_PxQueryFilterCallback) -> bool #foreign physx_sys;

phys_PxCreateCustomSceneQuerySystem :: (sceneQueryUpdateMode: s32, contextID: u64, adapter: *physx_PxCustomSceneQuerySystemAdapter, usesTreeOfPruners: bool) -> *physx_PxCustomSceneQuerySystem #foreign physx_sys;

phys_PxFindFaceIndex :: (convexGeom: *physx_PxConvexMeshGeometry, geomPose: *physx_PxTransform, impactPos: *physx_Vector3, unitDir: *physx_Vector3) -> u32 #foreign physx_sys;

PxPoissonSampler_setSamplingRadius_mut :: (self: *physx_PxPoissonSampler, samplingRadius: float) -> bool #foreign physx_sys;

PxPoissonSampler_addSamplesInSphere_mut :: (self: *physx_PxPoissonSampler, sphereCenter: *physx_Vector3, sphereRadius: float, createVolumeSamples: bool) -> void #foreign physx_sys;

PxPoissonSampler_addSamplesInBox_mut :: (self: *physx_PxPoissonSampler, axisAlignedBox: *physx_PxBounds3, boxOrientation: *physx_PxQuat, createVolumeSamples: bool) -> void #foreign physx_sys;

PxPoissonSampler_delete :: (self: *physx_PxPoissonSampler) -> void #foreign physx_sys;

phys_PxCreateShapeSampler :: (geometry: *physx_PxGeometry, transform: *physx_PxTransform, worldBounds: *physx_PxBounds3, initialSamplingRadius: float, numSampleAttemptsAroundPos32: s32) -> *physx_PxPoissonSampler #foreign physx_sys;

PxTriangleMeshPoissonSampler_isPos32InTriangleMesh_mut :: (self: *physx_PxTriangleMeshPoissonSampler, p: *physx_Vector3) -> bool #foreign physx_sys;

PxTriangleMeshPoissonSampler_delete :: (self: *physx_PxTriangleMeshPoissonSampler) -> void #foreign physx_sys;

phys_PxCreateTriangleMeshSampler :: (triangles: *u32, numTriangles: u32, vertices: *physx_Vector3, numVertices: u32, initialSamplingRadius: float, numSampleAttemptsAroundPos32: s32) -> *physx_PxTriangleMeshPoissonSampler #foreign physx_sys;

PxTetrahedronMeshExt_findTetrahedronContainingPos32 :: (mesh: *physx_PxTetrahedronMesh, pos32: *physx_Vector3, bary: *physx_Vector4, tolerance: float) -> s32 #foreign physx_sys;

PxTetrahedronMeshExt_findTetrahedronClosestToPos32 :: (mesh: *physx_PxTetrahedronMesh, pos32: *physx_Vector3, bary: *physx_Vector4) -> s32 #foreign physx_sys;

phys_PxInitExtensions :: (physics: *physx_PxPhysics, pvd: *physx_PxPvd) -> bool #foreign physx_sys;

phys_PxCloseExtensions :: () -> void #foreign physx_sys;

PxRepXObject_new :: (inTypeName: *u8, inSerializable: *void, inId: u64) -> physx_PxRepXObject #foreign physx_sys;

PxRepXObject_isValid :: (self: *physx_PxRepXObject) -> bool #foreign physx_sys;

PxRepXInstantiationArgs_new :: (inPhysics: *physx_PxPhysics, inCooking: *physx_PxCooking, inStringTable: *physx_PxStringTable) -> physx_PxRepXInstantiationArgs #foreign physx_sys;

PxRepXSerializer_getTypeName_mut :: (self: *physx_PxRepXSerializer) -> *u8 #foreign physx_sys;

PxRepXSerializer_objectToFile_mut :: (self: *physx_PxRepXSerializer, inLiveObject: *physx_PxRepXObject, inCollection: *physx_PxCollection, inWriter: *physx_XmlWriter, inTempBuffer: *physx_MemoryBuffer, inArgs: *physx_PxRepXInstantiationArgs) -> void #foreign physx_sys;

PxRepXSerializer_fileToObject_mut :: (self: *physx_PxRepXSerializer, inReader: *physx_XmlReader, inAllocator: *physx_XmlMemoryAllocator, inArgs: *physx_PxRepXInstantiationArgs, inCollection: *physx_PxCollection) -> physx_PxRepXObject #foreign physx_sys;

PxPvd_connect_mut :: (self: *physx_PxPvd, transport: *physx_PxPvdTransport, flags: u8) -> bool #foreign physx_sys;

PxPvd_disconnect_mut :: (self: *physx_PxPvd) -> void #foreign physx_sys;

PxPvd_isConnected_mut :: (self: *physx_PxPvd, useCachedStatus: bool) -> bool #foreign physx_sys;

PxPvd_getTransport_mut :: (self: *physx_PxPvd) -> *physx_PxPvdTransport #foreign physx_sys;

PxPvd_getInstrumentationFlags_mut :: (self: *physx_PxPvd) -> u8 #foreign physx_sys;

PxPvd_release_mut :: (self: *physx_PxPvd) -> void #foreign physx_sys;

phys_PxCreatePvd :: (foundation: *physx_PxFoundation) -> *physx_PxPvd #foreign physx_sys;

PxPvdTransport_connect_mut :: (self: *physx_PxPvdTransport) -> bool #foreign physx_sys;

PxPvdTransport_disconnect_mut :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys;

PxPvdTransport_isConnected_mut :: (self: *physx_PxPvdTransport) -> bool #foreign physx_sys;

PxPvdTransport_write_mut :: (self: *physx_PxPvdTransport, inBytes: *u8, inLength: u32) -> bool #foreign physx_sys;

PxPvdTransport_lock_mut :: (self: *physx_PxPvdTransport) -> *physx_PxPvdTransport #foreign physx_sys;

PxPvdTransport_unlock_mut :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys;

PxPvdTransport_flush_mut :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys;

PxPvdTransport_getWrittenDataSize_mut :: (self: *physx_PxPvdTransport) -> u64 #foreign physx_sys;

PxPvdTransport_release_mut :: (self: *physx_PxPvdTransport) -> void #foreign physx_sys;

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
Constras32BreakCallback :: #type (unknown0: *void, unknown1: *physx_PxConstras32Info, unknown2: u32) -> void #c_call;
WakeSleepCallback :: #type (unknown0: *void, unknown1: **physx_PxActor, unknown2: u32, unknown3: bool) -> void #c_call;
AdvanceCallback :: #type (unknown0: *void, unknown1: **physx_PxRigidBody, unknown2: *physx_PxTransform, unknown3: u32) -> void #c_call;

SimulationEventCallbackInfo :: struct {
    // Callback for collision events.
    collisionCallback:       CollisionCallback = null;
    collisionUserData:       *void = null;

    // Callback for trigger shape events (an object entered or left a trigger shape).
    triggerCallback:         TriggerCallback = null;
    triggerUserData:         *void = null;

    // Callback for when a constras32 breaks (such as a jos32 with a force limit)
    constras32BreakCallback: Constras32BreakCallback = null;
    constras32BreakUserData: *void = null;

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
// linking against the raw s32erface
get_default_allocator :: () -> *physx_PxAllocatorCallback #foreign physx_sys;

// fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
// linking against the raw s32erface
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
PxAssertHandler_opCall_mut :: (self: *physx_PxErrorCallback, expr: *u8, file: *u8, line: s32, ignore: *bool) -> void #foreign physx_sys;

#scope_file

physx_sys :: #library,no_dll "windows/physx_sys";
