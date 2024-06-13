//
// This file was auto-generated using the following command:
//
// jai ../ generate.jai
//



NDEBUG :: 1;
_MSC_VER :: 1939;
physx_PxMat34_Pod :: struct {}

physx_PxAllocatorCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxAssertHandler_Pod :: struct {
    vtable_: *void;
}

physx_PxFoundation_Pod :: struct {
    vtable_: *void;
}

physx_PxAllocator_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxRawAllocator_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxVirtualAllocatorCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxVirtualAllocator_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxUserAllocated_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxTempAllocatorChunk_Pod :: union {
    mNext:  *physx_PxTempAllocatorChunk_Pod;
    mIndex: u32;
    mPad:   [16] u8;
}

physx_PxTempAllocator_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxLogTwo_Pod :: struct {}
physx_PxUnConst_Pod :: struct {}
physx_PxBitAndByte_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxBitMap_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxVec3_Pod :: struct {
    x: float;
    y: float;
    z: float;
}

physx_PxVec3Padded_Pod :: struct {
    x:       float;
    y:       float;
    z:       float;
    padding: u32;
}

physx_PxQuat_Pod :: struct {
    x: float;
    y: float;
    z: float;
    w: float;
}

physx_PxTransform_Pod :: struct {
    q: physx_PxQuat_Pod;
    p: physx_PxVec3_Pod;
}

physx_PxTransformPadded_Pod :: struct {
    transform: physx_PxTransform_Pod;
    padding:   u32;
}

physx_PxMat33_Pod :: struct {
    column0: physx_PxVec3_Pod;
    column1: physx_PxVec3_Pod;
    column2: physx_PxVec3_Pod;
}

physx_PxBounds3_Pod :: struct {
    minimum: physx_PxVec3_Pod;
    maximum: physx_PxVec3_Pod;
}

physx_PxErrorCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxAllocationListener_Pod :: struct {
    vtable_: *void;
}

physx_PxBroadcastingAllocator_Pod :: struct {
    structgen_pad0: [176] u8;
}

physx_PxBroadcastingErrorCallback_Pod :: struct {
    structgen_pad0: [160] u8;
}

physx_PxHash_Pod :: struct {}
physx_PxInputStream_Pod :: struct {
    vtable_: *void;
}

physx_PxInputData_Pod :: struct {
    vtable_: *void;
}

physx_PxOutputStream_Pod :: struct {
    vtable_: *void;
}

physx_PxVec4_Pod :: struct {
    x: float;
    y: float;
    z: float;
    w: float;
}

physx_PxMat44_Pod :: struct {
    column0: physx_PxVec4_Pod;
    column1: physx_PxVec4_Pod;
    column2: physx_PxVec4_Pod;
    column3: physx_PxVec4_Pod;
}

physx_PxPlane_Pod :: struct {
    n: physx_PxVec3_Pod;
    d: float;
}

physx_Interpolation_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxMutexImpl_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxReadWriteLock_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxProfilerCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxProfileScoped_Pod :: struct {
    mCallback:      *physx_PxProfilerCallback_Pod;
    mEventName:     *u8;
    mProfilerData:  *void;
    mContextId:     u64;
    mDetached:      bool;
    structgen_pad0: [7] u8;
}

physx_PxSListEntry_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxSListImpl_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSyncImpl_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxRunnable_Pod :: struct {
    vtable_: *void;
}

physx_PxCounterFrequencyToTensOfNanos_Pod :: struct {
    mNumerator:   u64;
    mDenominator: u64;
}

physx_PxTime_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxVec2_Pod :: struct {
    x: float;
    y: float;
}

physx_PxStridedData_Pod :: struct {
    stride:         u32;
    structgen_pad0: [4] u8;
    data:           *void;
}

physx_PxBoundedData_Pod :: struct {
    stride:         u32;
    structgen_pad0: [4] u8;
    data:           *void;
    count:          u32;
    structgen_pad1: [4] u8;
}

physx_PxDebugPoint_Pod :: struct {
    pos:   physx_PxVec3_Pod;
    color: u32;
}

physx_PxDebugLine_Pod :: struct {
    pos0:   physx_PxVec3_Pod;
    color0: u32;
    pos1:   physx_PxVec3_Pod;
    color1: u32;
}

physx_PxDebugTriangle_Pod :: struct {
    pos0:   physx_PxVec3_Pod;
    color0: u32;
    pos1:   physx_PxVec3_Pod;
    color1: u32;
    pos2:   physx_PxVec3_Pod;
    color2: u32;
}

physx_PxDebugText_Pod :: struct {
    position:       physx_PxVec3_Pod;
    size:           float;
    color:          u32;
    structgen_pad0: [4] u8;
    _string:        *u8;
}

physx_PxRenderBuffer_Pod :: struct {
    vtable_: *void;
}

physx_PxProcessPxBaseCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxSerializationContext_Pod :: struct {
    vtable_: *void;
}

physx_PxDeserializationContext_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxSerializationRegistry_Pod :: struct {
    vtable_: *void;
}

physx_PxCollection_Pod :: struct {
    vtable_: *void;
}

physx_PxTypeInfo_Pod :: struct {}

physx_PxFEMSoftBodyMaterial_Pod :: struct {}
physx_PxFEMClothMaterial_Pod :: struct {}
physx_PxPBDMaterial_Pod :: struct {}
physx_PxFLIPMaterial_Pod :: struct {}
physx_PxMPMMaterial_Pod :: struct {}
physx_PxCustomMaterial_Pod :: struct {}

physx_PxBVH33TriangleMesh_Pod :: struct {}

physx_PxParticleSystem_Pod :: struct {}
physx_PxPBDParticleSystem_Pod :: struct {}
physx_PxFLIPParticleSystem_Pod :: struct {}
physx_PxMPMParticleSystem_Pod :: struct {}
physx_PxCustomParticleSystem_Pod :: struct {}
physx_PxSoftBody_Pod :: struct {}
physx_PxFEMCloth_Pod :: struct {}
physx_PxHairSystem_Pod :: struct {}
physx_PxParticleBuffer_Pod :: struct {}
physx_PxParticleAndDiffuseBuffer_Pod :: struct {}
physx_PxParticleClothBuffer_Pod :: struct {}
physx_PxParticleRigidBuffer_Pod :: struct {}
physx_PxBase_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxRefCounted_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTolerancesScale_Pod :: struct {
    length: float;
    speed:  float;
}

physx_PxStringTable_Pod :: struct {
    vtable_: *void;
}

physx_PxSerializer_Pod :: struct {
    vtable_: *void;
}

physx_PxMetaDataEntry_Pod :: struct {
    type:       *u8;
    name:       *u8;
    offset:     u32;
    size:       u32;
    count:      u32;
    offsetSize: u32;
    flags:      u32;
    alignment:  u32;
}

physx_PxInsertionCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxTaskManager_Pod :: struct {
    vtable_: *void;
}

physx_PxCpuDispatcher_Pod :: struct {
    vtable_: *void;
}

physx_PxBaseTask_Pod :: struct {
    structgen_pad0: [24] u8;
}

physx_PxTask_Pod :: struct {
    structgen_pad0: [32] u8;
}

physx_PxLightCpuTask_Pod :: struct {
    structgen_pad0: [40] u8;
}

physx_PxGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

physx_PxBoxGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    halfExtents:    physx_PxVec3_Pod;
}

physx_PxBVHRaycastCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxBVHOverlapCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxBVHTraversalCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxBVH_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxCapsuleGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    radius:         float;
    halfHeight:     float;
}

physx_PxHullPolygon_Pod :: struct {
    mPlane:     [4] float;
    mNbVerts:   u16;
    mIndexBase: u16;
}

physx_PxConvexMesh_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxMeshScale_Pod :: struct {
    scale:    physx_PxVec3_Pod;
    rotation: physx_PxQuat_Pod;
}

physx_PxConvexMeshGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    scale:          physx_PxMeshScale_Pod;
    structgen_pad1: [4] u8;
    convexMesh:     *physx_PxConvexMesh_Pod;
    meshFlags:      u8;
    structgen_pad2: [7] u8;
}

physx_PxSphereGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    radius:         float;
}

physx_PxPlaneGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

physx_PxTriangleMeshGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    scale:          physx_PxMeshScale_Pod;
    meshFlags:      u8;
    structgen_pad1: [3] u8;
    triangleMesh:   *physx_PxTriangleMesh_Pod;
}

physx_PxHeightFieldGeometry_Pod :: struct {
    structgen_pad0:   [4] u8;
    mTypePadding:     float;
    heightField:      *physx_PxHeightField_Pod;
    heightScale:      float;
    rowScale:         float;
    columnScale:      float;
    heightFieldFlags: u8;
    structgen_pad1:   [3] u8;
}

physx_PxParticleSystemGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    mSolverType:    s32;
}

physx_PxHairSystemGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
}

physx_PxTetrahedronMeshGeometry_Pod :: struct {
    structgen_pad0:  [4] u8;
    mTypePadding:    float;
    tetrahedronMesh: *physx_PxTetrahedronMesh_Pod;
}

physx_PxQueryHit_Pod :: struct {
    faceIndex: u32;
}

physx_PxLocationHit_Pod :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       physx_PxVec3_Pod;
    normal:         physx_PxVec3_Pod;
    distance:       float;
}

physx_PxGeomRaycastHit_Pod :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       physx_PxVec3_Pod;
    normal:         physx_PxVec3_Pod;
    distance:       float;
    u:              float;
    v:              float;
}

physx_PxGeomOverlapHit_Pod :: struct {
    faceIndex: u32;
}

physx_PxGeomSweepHit_Pod :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       physx_PxVec3_Pod;
    normal:         physx_PxVec3_Pod;
    distance:       float;
}

physx_PxGeomIndexPair_Pod :: struct {
    id0: u32;
    id1: u32;
}

physx_PxQueryThreadContext_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxContactBuffer_Pod :: struct {}
physx_PxRenderOutput_Pod :: struct {}

physx_PxCustomGeometryType_Pod :: struct {
    structgen_pad0: [4] u8;
}

physx_PxCustomGeometryCallbacks_Pod :: struct {
    vtable_: *void;
}

physx_PxCustomGeometry_Pod :: struct {
    structgen_pad0: [4] u8;
    mTypePadding:   float;
    callbacks:      *physx_PxCustomGeometryCallbacks_Pod;
}

physx_PxGeometryHolder_Pod :: struct {
    structgen_pad0: [56] u8;
}

physx_PxGeometryQuery_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxHeightFieldSample_Pod :: struct {
    height:         s16;
    materialIndex0: physx_PxBitAndByte_Pod;
    materialIndex1: physx_PxBitAndByte_Pod;
}

physx_PxHeightField_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxHeightFieldDesc_Pod :: struct {
    nbRows:              u32;
    nbColumns:           u32;
    format:              s32;
    structgen_pad0:      [4] u8;
    samples:             physx_PxStridedData_Pod;
    convexEdgeThreshold: float;
    flags:               u16;
    structgen_pad1:      [2] u8;
}

physx_PxMeshQuery_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSimpleTriangleMesh_Pod :: struct {
    points:         physx_PxBoundedData_Pod;
    triangles:      physx_PxBoundedData_Pod;
    flags:          u16;
    structgen_pad0: [6] u8;
}

physx_PxTriangle_Pod :: struct {
    verts: [3] physx_PxVec3_Pod;
}

physx_PxTrianglePadded_Pod :: struct {
    verts:   [3] physx_PxVec3_Pod;
    padding: u32;
}

physx_PxTriangleMesh_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxBVH34TriangleMesh_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTetrahedron_Pod :: struct {
    verts: [4] physx_PxVec3_Pod;
}

physx_PxSoftBodyAuxData_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxTetrahedronMesh_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxSoftBodyMesh_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxCollisionMeshMappingData_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxSoftBodyCollisionData_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxTetrahedronMeshData_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSoftBodySimulationData_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxCollisionTetrahedronMeshData_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxSimulationTetrahedronMeshData_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxActor_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxAggregate_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpringModifiers_Pod :: struct {
    stiffness:      float;
    damping:        float;
    structgen_pad0: [8] u8;
}

physx_PxRestitutionModifiers_Pod :: struct {
    restitution:       float;
    velocityThreshold: float;
    structgen_pad0:    [8] u8;
}

physx_Px1DConstraintMods_Pod :: union {
    spring: physx_PxSpringModifiers_Pod;
    bounce: physx_PxRestitutionModifiers_Pod;
}

physx_Px1DConstraint_Pod :: struct {
    linear0:        physx_PxVec3_Pod;
    geometricError: float;
    angular0:       physx_PxVec3_Pod;
    velocityTarget: float;
    linear1:        physx_PxVec3_Pod;
    minImpulse:     float;
    angular1:       physx_PxVec3_Pod;
    maxImpulse:     float;
    mods:           physx_Px1DConstraintMods_Pod;
    forInternalUse: float;
    flags:          u16;
    solveHint:      u16;
    structgen_pad0: [8] u8;
}

physx_PxConstraintInvMassScale_Pod :: struct {
    linear0:  float;
    angular0: float;
    linear1:  float;
    angular1: float;
}

physx_PxConstraintVisualizer_Pod :: struct {
    vtable_: *void;
}

physx_PxConstraintConnector_Pod :: struct {
    vtable_: *void;
}

physx_PxContactPoint_Pod :: struct {
    normal:             physx_PxVec3_Pod;
    separation:         float;
    point:              physx_PxVec3_Pod;
    maxImpulse:         float;
    targetVel:          physx_PxVec3_Pod;
    staticFriction:     float;
    materialFlags:      u8;
    structgen_pad0:     [3] u8;
    internalFaceIndex1: u32;
    dynamicFriction:    float;
    restitution:        float;
    damping:            float;
    structgen_pad1:     [12] u8;
}

physx_PxSolverBody_Pod :: struct {
    linearVelocity:            physx_PxVec3_Pod;
    maxSolverNormalProgress:   u16;
    maxSolverFrictionProgress: u16;
    angularState:              physx_PxVec3_Pod;
    solverProgress:            u32;
}

physx_PxSolverBodyData_Pod :: struct {
    linearVelocity:    physx_PxVec3_Pod;
    invMass:           float;
    angularVelocity:   physx_PxVec3_Pod;
    reportThreshold:   float;
    sqrtInvInertia:    physx_PxMat33_Pod;
    penBiasClamp:      float;
    nodeIndex:         u32;
    maxContactImpulse: float;
    body2World:        physx_PxTransform_Pod;
    pad:               u16;
    structgen_pad0:    [2] u8;
}

physx_PxConstraintBatchHeader_Pod :: struct {
    startIndex:     u32;
    stride:         u16;
    constraintType: u16;
}

physx_PxSolverConstraintDesc_Pod :: struct {
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

physx_PxSolverConstraintPrepDescBase_Pod :: struct {
    invMassScales:  physx_PxConstraintInvMassScale_Pod;
    desc:           *physx_PxSolverConstraintDesc_Pod;
    body0:          *physx_PxSolverBody_Pod;
    body1:          *physx_PxSolverBody_Pod;
    data0:          *physx_PxSolverBodyData_Pod;
    data1:          *physx_PxSolverBodyData_Pod;
    bodyFrame0:     physx_PxTransform_Pod;
    bodyFrame1:     physx_PxTransform_Pod;
    bodyState0:     s32;
    bodyState1:     s32;
    structgen_pad0: [8] u8;
}

physx_PxSolverConstraintPrepDesc_Pod :: struct {
    invMassScales:        physx_PxConstraintInvMassScale_Pod;
    desc:                 *physx_PxSolverConstraintDesc_Pod;
    body0:                *physx_PxSolverBody_Pod;
    body1:                *physx_PxSolverBody_Pod;
    data0:                *physx_PxSolverBodyData_Pod;
    data1:                *physx_PxSolverBodyData_Pod;
    bodyFrame0:           physx_PxTransform_Pod;
    bodyFrame1:           physx_PxTransform_Pod;
    bodyState0:           s32;
    bodyState1:           s32;
    structgen_pad0:       [8] u8;
    rows:                 *physx_Px1DConstraint_Pod;
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
    body0WorldOffset:     physx_PxVec3Padded_Pod;
    structgen_pad2:       [8] u8;
}

physx_PxSolverContactDesc_Pod :: struct {
    invMassScales:           physx_PxConstraintInvMassScale_Pod;
    desc:                    *physx_PxSolverConstraintDesc_Pod;
    body0:                   *physx_PxSolverBody_Pod;
    body1:                   *physx_PxSolverBody_Pod;
    data0:                   *physx_PxSolverBodyData_Pod;
    data1:                   *physx_PxSolverBodyData_Pod;
    bodyFrame0:              physx_PxTransform_Pod;
    bodyFrame1:              physx_PxTransform_Pod;
    bodyState0:              s32;
    bodyState1:              s32;
    structgen_pad0:          [8] u8;
    shapeInteraction:        *void;
    contacts:                *physx_PxContactPoint_Pod;
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

physx_PxConstraintAllocator_Pod :: struct {
    vtable_: *void;
}

physx_PxArticulationLimit_Pod :: struct {
    low:  float;
    high: float;
}

physx_PxArticulationDrive_Pod :: struct {
    stiffness: float;
    damping:   float;
    maxForce:  float;
    driveType: s32;
}

physx_PxTGSSolverBodyVel_Pod :: struct {
    linearVelocity:       physx_PxVec3_Pod;
    nbStaticInteractions: u16;
    maxDynamicPartition:  u16;
    angularVelocity:      physx_PxVec3_Pod;
    partitionMask:        u32;
    deltaAngDt:           physx_PxVec3_Pod;
    maxAngVel:            float;
    deltaLinDt:           physx_PxVec3_Pod;
    lockFlags:            u16;
    isKinematic:          bool;
    pad:                  u8;
}

physx_PxTGSSolverBodyTxInertia_Pod :: struct {
    deltaBody2World: physx_PxTransform_Pod;
    sqrtInvInertia:  physx_PxMat33_Pod;
}

physx_PxTGSSolverBodyData_Pod :: struct {
    originalLinearVelocity:  physx_PxVec3_Pod;
    maxContactImpulse:       float;
    originalAngularVelocity: physx_PxVec3_Pod;
    penBiasClamp:            float;
    invMass:                 float;
    nodeIndex:               u32;
    reportThreshold:         float;
    pad:                     u32;
}

physx_PxTGSSolverConstraintPrepDescBase_Pod :: struct {
    invMassScales:  physx_PxConstraintInvMassScale_Pod;
    desc:           *physx_PxSolverConstraintDesc_Pod;
    body0:          *physx_PxTGSSolverBodyVel_Pod;
    body1:          *physx_PxTGSSolverBodyVel_Pod;
    body0TxI:       *physx_PxTGSSolverBodyTxInertia_Pod;
    body1TxI:       *physx_PxTGSSolverBodyTxInertia_Pod;
    bodyData0:      *physx_PxTGSSolverBodyData_Pod;
    bodyData1:      *physx_PxTGSSolverBodyData_Pod;
    bodyFrame0:     physx_PxTransform_Pod;
    bodyFrame1:     physx_PxTransform_Pod;
    bodyState0:     s32;
    bodyState1:     s32;
    structgen_pad0: [8] u8;
}

physx_PxTGSSolverConstraintPrepDesc_Pod :: struct {
    invMassScales:        physx_PxConstraintInvMassScale_Pod;
    desc:                 *physx_PxSolverConstraintDesc_Pod;
    body0:                *physx_PxTGSSolverBodyVel_Pod;
    body1:                *physx_PxTGSSolverBodyVel_Pod;
    body0TxI:             *physx_PxTGSSolverBodyTxInertia_Pod;
    body1TxI:             *physx_PxTGSSolverBodyTxInertia_Pod;
    bodyData0:            *physx_PxTGSSolverBodyData_Pod;
    bodyData1:            *physx_PxTGSSolverBodyData_Pod;
    bodyFrame0:           physx_PxTransform_Pod;
    bodyFrame1:           physx_PxTransform_Pod;
    bodyState0:           s32;
    bodyState1:           s32;
    structgen_pad0:       [8] u8;
    rows:                 *physx_Px1DConstraint_Pod;
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
    body0WorldOffset:     physx_PxVec3Padded_Pod;
    cA2w:                 physx_PxVec3Padded_Pod;
    cB2w:                 physx_PxVec3Padded_Pod;
    structgen_pad2:       [8] u8;
}

physx_PxTGSSolverContactDesc_Pod :: struct {
    invMassScales:           physx_PxConstraintInvMassScale_Pod;
    desc:                    *physx_PxSolverConstraintDesc_Pod;
    body0:                   *physx_PxTGSSolverBodyVel_Pod;
    body1:                   *physx_PxTGSSolverBodyVel_Pod;
    body0TxI:                *physx_PxTGSSolverBodyTxInertia_Pod;
    body1TxI:                *physx_PxTGSSolverBodyTxInertia_Pod;
    bodyData0:               *physx_PxTGSSolverBodyData_Pod;
    bodyData1:               *physx_PxTGSSolverBodyData_Pod;
    bodyFrame0:              physx_PxTransform_Pod;
    bodyFrame1:              physx_PxTransform_Pod;
    bodyState0:              s32;
    bodyState1:              s32;
    structgen_pad0:          [8] u8;
    shapeInteraction:        *void;
    contacts:                *physx_PxContactPoint_Pod;
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

physx_PxArticulationTendonLimit_Pod :: struct {
    lowLimit:  float;
    highLimit: float;
}

physx_PxArticulationAttachment_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationTendonJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationTendon_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationSpatialTendon_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationFixedTendon_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpatialForce_Pod :: struct {
    force:  physx_PxVec3_Pod;
    pad0:   float;
    torque: physx_PxVec3_Pod;
    pad1:   float;
}

physx_PxSpatialVelocity_Pod :: struct {
    linear:  physx_PxVec3_Pod;
    pad0:    float;
    angular: physx_PxVec3_Pod;
    pad1:    float;
}

physx_PxArticulationRootLinkData_Pod :: struct {
    transform:     physx_PxTransform_Pod;
    worldLinVel:   physx_PxVec3_Pod;
    worldAngVel:   physx_PxVec3_Pod;
    worldLinAccel: physx_PxVec3_Pod;
    worldAngAccel: physx_PxVec3_Pod;
}

physx_PxArticulationCache_Pod :: struct {
    externalForces:    *physx_PxSpatialForce_Pod;
    denseJacobian:     *float;
    massMatrix:        *float;
    jointVelocity:     *float;
    jointAcceleration: *float;
    jointPosition:     *float;
    jointForce:        *float;
    jointSolverForces: *float;
    linkVelocity:      *physx_PxSpatialVelocity_Pod;
    linkAcceleration:  *physx_PxSpatialVelocity_Pod;
    rootLinkData:      *physx_PxArticulationRootLinkData_Pod;
    sensorForces:      *physx_PxSpatialForce_Pod;
    coefficientMatrix: *float;
    lambda:            *float;
    scratchMemory:     *void;
    scratchAllocator:  *void;
    version:           u32;
    structgen_pad0:    [4] u8;
}

physx_PxArticulationSensor_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationReducedCoordinate_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationJointReducedCoordinate_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxShape_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRigidActor_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxNodeIndex_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxRigidBody_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxArticulationLink_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxConeLimitedConstraint_Pod :: struct {
    mAxis:      physx_PxVec3_Pod;
    mAngle:     float;
    mLowLimit:  float;
    mHighLimit: float;
}

physx_PxConeLimitParams_Pod :: struct {
    lowHighLimits: physx_PxVec4_Pod;
    axisAngle:     physx_PxVec4_Pod;
}

physx_PxConstraintShaderTable_Pod :: struct {
    solverPrep:     *void;
    structgen_pad0: [8] u8;
    visualize:      *void;
    flag:           s32;
    structgen_pad1: [4] u8;
}

physx_PxConstraint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxMassModificationProps_Pod :: struct {
    mInvMassScale0:    float;
    mInvInertiaScale0: float;
    mInvMassScale1:    float;
    mInvInertiaScale1: float;
}

physx_PxContactPatch_Pod :: struct {
    mMassModification: physx_PxMassModificationProps_Pod;
    normal:            physx_PxVec3_Pod;
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

physx_PxContact_Pod :: struct {
    contact:    physx_PxVec3_Pod;
    separation: float;
}

physx_PxExtendedContact_Pod :: struct {
    contact:        physx_PxVec3_Pod;
    separation:     float;
    targetVelocity: physx_PxVec3_Pod;
    maxImpulse:     float;
}

physx_PxModifiableContact_Pod :: struct {
    contact:         physx_PxVec3_Pod;
    separation:      float;
    targetVelocity:  physx_PxVec3_Pod;
    maxImpulse:      float;
    normal:          physx_PxVec3_Pod;
    restitution:     float;
    materialFlags:   u32;
    materialIndex0:  u16;
    materialIndex1:  u16;
    staticFriction:  float;
    dynamicFriction: float;
}

physx_PxContactStreamIterator_Pod :: struct {
    zero:                   physx_PxVec3_Pod;
    structgen_pad0:         [4] u8;
    patch:                  *physx_PxContactPatch_Pod;
    contact:                *physx_PxContact_Pod;
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

physx_PxGpuContactPair_Pod :: struct {
    contactPatches:     *u8;
    contactPoints:      *u8;
    contactForces:      *float;
    transformCacheRef0: u32;
    transformCacheRef1: u32;
    nodeIndex0:         physx_PxNodeIndex_Pod;
    nodeIndex1:         physx_PxNodeIndex_Pod;
    actor0:             *physx_PxActor_Pod;
    actor1:             *physx_PxActor_Pod;
    nbContacts:         u16;
    nbPatches:          u16;
    structgen_pad0:     [4] u8;
}

physx_PxContactSet_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxContactModifyPair_Pod :: struct {
    actor:     [2] *physx_PxRigidActor_Pod;
    shape:     [2] *physx_PxShape_Pod;
    transform: [2] physx_PxTransform_Pod;
    contacts:  physx_PxContactSet_Pod;
}

physx_PxContactModifyCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxCCDContactModifyCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxDeletionListener_Pod :: struct {
    vtable_: *void;
}

physx_PxBaseMaterial_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFEMMaterial_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFilterData_Pod :: struct {
    word0: u32;
    word1: u32;
    word2: u32;
    word3: u32;
}

physx_PxSimulationFilterCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxParticleRigidFilterPair_Pod :: struct {
    mID0: u64;
    mID1: u64;
}

physx_PxLockedData_Pod :: struct {
    vtable_: *void;
}

physx_PxMaterial_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGpuParticleBufferIndexPair_Pod :: struct {
    systemIndex: u32;
    bufferIndex: u32;
}

physx_PxCudaContextManager_Pod :: struct {}
physx_PxParticleRigidAttachment_Pod :: struct {}
physx_PxParticleVolume_Pod :: struct {
    bound:                 physx_PxBounds3_Pod;
    particleIndicesOffset: u32;
    numParticles:          u32;
}

physx_PxDiffuseParticleParams_Pod :: struct {
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

physx_PxParticleSpring_Pod :: struct {
    ind0:      u32;
    ind1:      u32;
    length:    float;
    stiffness: float;
    damping:   float;
    pad:       float;
}

physx_PxParticleMaterial_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxOmniPvd_Pod :: struct {}
physx_PxPhysics_Pod :: struct {
    vtable_: *void;
}

physx_PxActorShape_Pod :: struct {
    actor: *physx_PxRigidActor_Pod;
    shape: *physx_PxShape_Pod;
}

physx_PxRaycastHit_Pod :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       physx_PxVec3_Pod;
    normal:         physx_PxVec3_Pod;
    distance:       float;
    u:              float;
    v:              float;
    structgen_pad1: [4] u8;
    actor:          *physx_PxRigidActor_Pod;
    shape:          *physx_PxShape_Pod;
}

physx_PxOverlapHit_Pod :: struct {
    faceIndex:      u32;
    structgen_pad0: [4] u8;
    actor:          *physx_PxRigidActor_Pod;
    shape:          *physx_PxShape_Pod;
}

physx_PxSweepHit_Pod :: struct {
    faceIndex:      u32;
    flags:          u16;
    structgen_pad0: [2] u8;
    position:       physx_PxVec3_Pod;
    normal:         physx_PxVec3_Pod;
    distance:       float;
    structgen_pad1: [4] u8;
    actor:          *physx_PxRigidActor_Pod;
    shape:          *physx_PxShape_Pod;
}

physx_PxRaycastCallback_Pod :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxRaycastHit_Pod;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxRaycastHit_Pod;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxOverlapCallback_Pod :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxOverlapHit_Pod;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxOverlapHit_Pod;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxSweepCallback_Pod :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxSweepHit_Pod;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxSweepHit_Pod;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxRaycastBuffer_Pod :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxRaycastHit_Pod;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxRaycastHit_Pod;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxOverlapBuffer_Pod :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxOverlapHit_Pod;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxOverlapHit_Pod;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxSweepBuffer_Pod :: struct {
    structgen_pad0: [8] u8;
    block:          physx_PxSweepHit_Pod;
    hasBlock:       bool;
    structgen_pad1: [7] u8;
    touches:        *physx_PxSweepHit_Pod;
    maxNbTouches:   u32;
    nbTouches:      u32;
}

physx_PxQueryCache_Pod :: struct {
    shape:          *physx_PxShape_Pod;
    actor:          *physx_PxRigidActor_Pod;
    faceIndex:      u32;
    structgen_pad0: [4] u8;
}

physx_PxQueryFilterData_Pod :: struct {
    data:           physx_PxFilterData_Pod;
    flags:          u16;
    structgen_pad0: [2] u8;
}

physx_PxQueryFilterCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxRigidDynamic_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRigidStatic_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSceneQueryDesc_Pod :: struct {
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

physx_PxSceneQuerySystemBase_Pod :: struct {
    vtable_: *void;
}

physx_PxSceneSQSystem_Pod :: struct {
    vtable_: *void;
}

physx_PxSceneQuerySystem_Pod :: struct {
    vtable_: *void;
}

physx_PxBroadPhaseRegion_Pod :: struct {
    mBounds:   physx_PxBounds3_Pod;
    mUserData: *void;
}

physx_PxBroadPhaseRegionInfo_Pod :: struct {
    mRegion:           physx_PxBroadPhaseRegion_Pod;
    mNbStaticObjects:  u32;
    mNbDynamicObjects: u32;
    mActive:           bool;
    mOverlap:          bool;
    structgen_pad0:    [6] u8;
}

physx_PxBroadPhaseCaps_Pod :: struct {
    mMaxNbRegions: u32;
}

physx_PxBroadPhaseDesc_Pod :: struct {
    mType:                        s32;
    structgen_pad0:               [4] u8;
    mContextID:                   u64;
    structgen_pad1:               [8] u8;
    mFoundLostPairsCapacity:      u32;
    mDiscardStaticVsKinematic:    bool;
    mDiscardKinematicVsKinematic: bool;
    structgen_pad2:               [2] u8;
}

physx_PxBroadPhaseUpdateData_Pod :: struct {
    mCreated:       *u32;
    mNbCreated:     u32;
    structgen_pad0: [4] u8;
    mUpdated:       *u32;
    mNbUpdated:     u32;
    structgen_pad1: [4] u8;
    mRemoved:       *u32;
    mNbRemoved:     u32;
    structgen_pad2: [4] u8;
    mBounds:        *physx_PxBounds3_Pod;
    mGroups:        *u32;
    mDistances:     *float;
    mCapacity:      u32;
    structgen_pad3: [4] u8;
}

physx_PxBroadPhasePair_Pod :: struct {
    mID0: u32;
    mID1: u32;
}

physx_PxBroadPhaseResults_Pod :: struct {
    mNbCreatedPairs: u32;
    structgen_pad0:  [4] u8;
    mCreatedPairs:   *physx_PxBroadPhasePair_Pod;
    mNbDeletedPairs: u32;
    structgen_pad1:  [4] u8;
    mDeletedPairs:   *physx_PxBroadPhasePair_Pod;
}

physx_PxBroadPhaseRegions_Pod :: struct {
    vtable_: *void;
}

physx_PxBroadPhase_Pod :: struct {
    vtable_: *void;
}

physx_PxAABBManager_Pod :: struct {
    vtable_: *void;
}

physx_PxSceneLimits_Pod :: struct {
    maxNbActors:             u32;
    maxNbBodies:             u32;
    maxNbStaticShapes:       u32;
    maxNbDynamicShapes:      u32;
    maxNbAggregates:         u32;
    maxNbConstraints:        u32;
    maxNbRegions:            u32;
    maxNbBroadPhaseOverlaps: u32;
}

physx_PxgDynamicsMemoryConfig_Pod :: struct {
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

physx_PxSceneDesc_Pod :: struct {
    staticStructure:               s32;
    dynamicStructure:              s32;
    dynamicTreeRebuildRateHint:    u32;
    dynamicTreeSecondaryPruner:    s32;
    staticBVHBuildStrategy:        s32;
    dynamicBVHBuildStrategy:       s32;
    staticNbObjectsPerNode:        u32;
    dynamicNbObjectsPerNode:       u32;
    sceneQueryUpdateMode:          s32;
    gravity:                       physx_PxVec3_Pod;
    simulationEventCallback:       *physx_PxSimulationEventCallback_Pod;
    contactModifyCallback:         *physx_PxContactModifyCallback_Pod;
    ccdContactModifyCallback:      *physx_PxCCDContactModifyCallback_Pod;
    filterShaderData:              *void;
    filterShaderDataSize:          u32;
    structgen_pad0:                [4] u8;
    filterShader:                  *void;
    filterCallback:                *physx_PxSimulationFilterCallback_Pod;
    kineKineFilteringMode:         s32;
    staticKineFilteringMode:       s32;
    broadPhaseType:                s32;
    structgen_pad1:                [4] u8;
    broadPhaseCallback:            *physx_PxBroadPhaseCallback_Pod;
    limits:                        physx_PxSceneLimits_Pod;
    frictionType:                  s32;
    solverType:                    s32;
    bounceThresholdVelocity:       float;
    frictionOffsetThreshold:       float;
    frictionCorrelationDistance:   float;
    flags:                         u32;
    cpuDispatcher:                 *physx_PxCpuDispatcher_Pod;
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
    sanityBounds:                  physx_PxBounds3_Pod;
    gpuDynamicsConfig:             physx_PxgDynamicsMemoryConfig_Pod;
    gpuMaxNumPartitions:           u32;
    gpuMaxNumStaticPartitions:     u32;
    gpuComputeVersion:             u32;
    contactPairSlabSize:           u32;
    sceneQuerySystem:              *physx_PxSceneQuerySystem_Pod;
    structgen_pad3:                [8] u8;
}

physx_PxSimulationStatistics_Pod :: struct {
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

physx_PxGpuBodyData_Pod :: struct {
    quat:   physx_PxQuat_Pod;
    pos:    physx_PxVec4_Pod;
    linVel: physx_PxVec4_Pod;
    angVel: physx_PxVec4_Pod;
}

physx_PxGpuActorPair_Pod :: struct {
    srcIndex:       u32;
    structgen_pad0: [4] u8;
    nodeIndex:      physx_PxNodeIndex_Pod;
}

physx_PxIndexDataPair_Pod :: struct {
    index:          u32;
    structgen_pad0: [4] u8;
    data:           *void;
}

physx_PxPvdSceneClient_Pod :: struct {
    vtable_: *void;
}

physx_PxDominanceGroupPair_Pod :: struct {
    dominance0: u8;
    dominance1: u8;
}

physx_PxBroadPhaseCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxScene_Pod :: struct {
    structgen_pad0: [8] u8;
    userData:       *void;
}

physx_PxSceneReadLock_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxSceneWriteLock_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxContactPairExtraDataItem_Pod :: struct {
    type: u8;
}

physx_PxContactPairVelocity_Pod :: struct {
    type:            u8;
    structgen_pad0:  [3] u8;
    linearVelocity:  [2] physx_PxVec3_Pod;
    angularVelocity: [2] physx_PxVec3_Pod;
}

physx_PxContactPairPose_Pod :: struct {
    type:           u8;
    structgen_pad0: [3] u8;
    globalPose:     [2] physx_PxTransform_Pod;
}

physx_PxContactPairIndex_Pod :: struct {
    type:           u8;
    structgen_pad0: [1] u8;
    index:          u16;
}

physx_PxContactPairExtraDataIterator_Pod :: struct {
    currPtr:            *u8;
    endPtr:             *u8;
    preSolverVelocity:  *physx_PxContactPairVelocity_Pod;
    postSolverVelocity: *physx_PxContactPairVelocity_Pod;
    eventPose:          *physx_PxContactPairPose_Pod;
    contactPairIndex:   u32;
    structgen_pad0:     [4] u8;
}

physx_PxContactPairHeader_Pod :: struct {
    actors:              [2] *physx_PxActor_Pod;
    extraDataStream:     *u8;
    extraDataStreamSize: u16;
    flags:               u16;
    structgen_pad0:      [4] u8;
    pairs:               *physx_PxContactPair_Pod;
    nbPairs:             u32;
    structgen_pad1:      [4] u8;
}

physx_PxContactPairPoint_Pod :: struct {
    position:           physx_PxVec3_Pod;
    separation:         float;
    normal:             physx_PxVec3_Pod;
    internalFaceIndex0: u32;
    impulse:            physx_PxVec3_Pod;
    internalFaceIndex1: u32;
}

physx_PxContactPair_Pod :: struct {
    shapes:             [2] *physx_PxShape_Pod;
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

physx_PxTriggerPair_Pod :: struct {
    triggerShape:   *physx_PxShape_Pod;
    triggerActor:   *physx_PxActor_Pod;
    otherShape:     *physx_PxShape_Pod;
    otherActor:     *physx_PxActor_Pod;
    status:         s32;
    flags:          u8;
    structgen_pad0: [3] u8;
}

physx_PxConstraintInfo_Pod :: struct {
    constraint:        *physx_PxConstraint_Pod;
    externalReference: *void;
    type:              u32;
    structgen_pad0:    [4] u8;
}

physx_PxSimulationEventCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxFEMParameters_Pod :: struct {
    velocityDamping:              float;
    settlingThreshold:            float;
    sleepThreshold:               float;
    sleepDamping:                 float;
    selfCollisionFilterDistance:  float;
    selfCollisionStressTolerance: float;
}

physx_PxPruningStructure_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxExtendedVec3_Pod :: struct {
    x: float64;
    y: float64;
    z: float64;
}

physx_PxObstacle_Pod :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3_Pod;
    mRot:           physx_PxQuat_Pod;
}

physx_PxBoxObstacle_Pod :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3_Pod;
    mRot:           physx_PxQuat_Pod;
    mHalfExtents:   physx_PxVec3_Pod;
    structgen_pad1: [4] u8;
}

physx_PxCapsuleObstacle_Pod :: struct {
    structgen_pad0: [8] u8;
    mUserData:      *void;
    mPos:           physx_PxExtendedVec3_Pod;
    mRot:           physx_PxQuat_Pod;
    mHalfHeight:    float;
    mRadius:        float;
}

physx_PxObstacleContext_Pod :: struct {
    vtable_: *void;
}

physx_PxControllerState_Pod :: struct {
    deltaXP:               physx_PxVec3_Pod;
    structgen_pad0:        [4] u8;
    touchedShape:          *physx_PxShape_Pod;
    touchedActor:          *physx_PxRigidActor_Pod;
    touchedObstacleHandle: u32;
    collisionFlags:        u32;
    standOnAnotherCCT:     bool;
    standOnObstacle:       bool;
    isMovingUp:            bool;
    structgen_pad1:        [5] u8;
}

physx_PxControllerStats_Pod :: struct {
    nbIterations:     u16;
    nbFullUpdates:    u16;
    nbPartialUpdates: u16;
    nbTessellation:   u16;
}

physx_PxControllerHit_Pod :: struct {
    controller:     *physx_PxController_Pod;
    worldPos:       physx_PxExtendedVec3_Pod;
    worldNormal:    physx_PxVec3_Pod;
    dir:            physx_PxVec3_Pod;
    length:         float;
    structgen_pad0: [4] u8;
}

physx_PxControllerShapeHit_Pod :: struct {
    controller:     *physx_PxController_Pod;
    worldPos:       physx_PxExtendedVec3_Pod;
    worldNormal:    physx_PxVec3_Pod;
    dir:            physx_PxVec3_Pod;
    length:         float;
    structgen_pad0: [4] u8;
    shape:          *physx_PxShape_Pod;
    actor:          *physx_PxRigidActor_Pod;
    triangleIndex:  u32;
    structgen_pad1: [4] u8;
}

physx_PxControllersHit_Pod :: struct {
    controller:     *physx_PxController_Pod;
    worldPos:       physx_PxExtendedVec3_Pod;
    worldNormal:    physx_PxVec3_Pod;
    dir:            physx_PxVec3_Pod;
    length:         float;
    structgen_pad0: [4] u8;
    other:          *physx_PxController_Pod;
}

physx_PxControllerObstacleHit_Pod :: struct {
    controller:     *physx_PxController_Pod;
    worldPos:       physx_PxExtendedVec3_Pod;
    worldNormal:    physx_PxVec3_Pod;
    dir:            physx_PxVec3_Pod;
    length:         float;
    structgen_pad0: [4] u8;
    userData:       *void;
}

physx_PxUserControllerHitReport_Pod :: struct {
    vtable_: *void;
}

physx_PxControllerFilterCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxControllerFilters_Pod :: struct {
    mFilterData:        *physx_PxFilterData_Pod;
    mFilterCallback:    *physx_PxQueryFilterCallback_Pod;
    mFilterFlags:       u16;
    structgen_pad0:     [6] u8;
    mCCTFilterCallback: *physx_PxControllerFilterCallback_Pod;
}

physx_PxControllerDesc_Pod :: struct {
    structgen_pad0:           [8] u8;
    position:                 physx_PxExtendedVec3_Pod;
    upDirection:              physx_PxVec3_Pod;
    slopeLimit:               float;
    invisibleWallHeight:      float;
    maxJumpHeight:            float;
    contactOffset:            float;
    stepOffset:               float;
    density:                  float;
    scaleCoeff:               float;
    volumeGrowth:             float;
    structgen_pad1:           [4] u8;
    reportCallback:           *physx_PxUserControllerHitReport_Pod;
    behaviorCallback:         *physx_PxControllerBehaviorCallback_Pod;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *physx_PxMaterial_Pod;
    registerDeletionListener: bool;
    clientID:                 u8;
    structgen_pad3:           [6] u8;
    userData:                 *void;
    structgen_pad4:           [8] u8;
}

physx_PxController_Pod :: struct {
    vtable_: *void;
}

physx_PxBoxControllerDesc_Pod :: struct {
    structgen_pad0:           [8] u8;
    position:                 physx_PxExtendedVec3_Pod;
    upDirection:              physx_PxVec3_Pod;
    slopeLimit:               float;
    invisibleWallHeight:      float;
    maxJumpHeight:            float;
    contactOffset:            float;
    stepOffset:               float;
    density:                  float;
    scaleCoeff:               float;
    volumeGrowth:             float;
    structgen_pad1:           [4] u8;
    reportCallback:           *physx_PxUserControllerHitReport_Pod;
    behaviorCallback:         *physx_PxControllerBehaviorCallback_Pod;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *physx_PxMaterial_Pod;
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

physx_PxBoxController_Pod :: struct {
    vtable_: *void;
}

physx_PxCapsuleControllerDesc_Pod :: struct {
    structgen_pad0:           [8] u8;
    position:                 physx_PxExtendedVec3_Pod;
    upDirection:              physx_PxVec3_Pod;
    slopeLimit:               float;
    invisibleWallHeight:      float;
    maxJumpHeight:            float;
    contactOffset:            float;
    stepOffset:               float;
    density:                  float;
    scaleCoeff:               float;
    volumeGrowth:             float;
    structgen_pad1:           [4] u8;
    reportCallback:           *physx_PxUserControllerHitReport_Pod;
    behaviorCallback:         *physx_PxControllerBehaviorCallback_Pod;
    nonWalkableMode:          s32;
    structgen_pad2:           [4] u8;
    material:                 *physx_PxMaterial_Pod;
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

physx_PxCapsuleController_Pod :: struct {
    vtable_: *void;
}

physx_PxControllerBehaviorCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxControllerManager_Pod :: struct {
    vtable_: *void;
}

physx_PxDim3_Pod :: struct {
    x: u32;
    y: u32;
    z: u32;
}

physx_PxSDFDesc_Pod :: struct {
    sdf:                                            physx_PxBoundedData_Pod;
    dims:                                           physx_PxDim3_Pod;
    meshLower:                                      physx_PxVec3_Pod;
    spacing:                                        float;
    subgridSize:                                    u32;
    bitsPerSubgridPixel:                            s32;
    sdfSubgrids3DTexBlockDim:                       physx_PxDim3_Pod;
    sdfSubgrids:                                    physx_PxBoundedData_Pod;
    sdfStartSlots:                                  physx_PxBoundedData_Pod;
    subgridsMinSdfValue:                            float;
    subgridsMaxSdfValue:                            float;
    sdfBounds:                                      physx_PxBounds3_Pod;
    narrowBandThicknessRelativeToSdfBoundsDiagonal: float;
    numThreadsForSdfConstruction:                   u32;
}

physx_PxConvexMeshDesc_Pod :: struct {
    points:         physx_PxBoundedData_Pod;
    polygons:       physx_PxBoundedData_Pod;
    indices:        physx_PxBoundedData_Pod;
    flags:          u16;
    vertexLimit:    u16;
    polygonLimit:   u16;
    quantizedCount: u16;
    sdfDesc:        *physx_PxSDFDesc_Pod;
}

physx_PxTriangleMeshDesc_Pod :: struct {
    points:         physx_PxBoundedData_Pod;
    triangles:      physx_PxBoundedData_Pod;
    flags:          u16;
    structgen_pad0: [22] u8;
    sdfDesc:        *physx_PxSDFDesc_Pod;
}

physx_PxTetrahedronMeshDesc_Pod :: struct {
    structgen_pad0: [16] u8;
    points:         physx_PxBoundedData_Pod;
    tetrahedrons:   physx_PxBoundedData_Pod;
    flags:          u16;
    tetsPerElement: u16;
    structgen_pad1: [4] u8;
}

physx_PxSoftBodySimulationDataDesc_Pod :: struct {
    vertexToTet: physx_PxBoundedData_Pod;
}

physx_PxBVH34MidphaseDesc_Pod :: struct {
    numPrimsPerLeaf: u32;
    buildStrategy:   s32;
    quantized:       bool;
    structgen_pad0:  [3] u8;
}

physx_PxMidphaseDesc_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxBVHDesc_Pod :: struct {
    bounds:          physx_PxBoundedData_Pod;
    enlargement:     float;
    numPrimsPerLeaf: u32;
    buildStrategy:   s32;
    structgen_pad0:  [4] u8;
}

physx_PxCookingParams_Pod :: struct {
    areaTestEpsilon:                float;
    planeTolerance:                 float;
    convexMeshCookingType:          s32;
    suppressTriangleMeshRemapTable: bool;
    buildTriangleAdjacencies:       bool;
    buildGPUData:                   bool;
    structgen_pad0:                 [1] u8;
    scale:                          physx_PxTolerancesScale_Pod;
    meshPreprocessParams:           u32;
    meshWeldTolerance:              float;
    midphaseDesc:                   physx_PxMidphaseDesc_Pod;
    gaussMapLimit:                  u32;
    maxWeightRatioInTet:            float;
}

physx_PxDefaultMemoryOutputStream_Pod :: struct {
    structgen_pad0: [32] u8;
}

physx_PxDefaultMemoryInputData_Pod :: struct {
    structgen_pad0: [32] u8;
}

physx_PxDefaultFileOutputStream_Pod :: struct {
    structgen_pad0: [16] u8;
}

physx_PxDefaultFileInputData_Pod :: struct {
    structgen_pad0: [24] u8;
}

physx_PxDefaultAllocator_Pod :: struct {
    vtable_: *void;
}

physx_PxJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSpring_Pod :: struct {
    stiffness: float;
    damping:   float;
}

physx_PxDistanceJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxJacobianRow_Pod :: struct {
    linear0:  physx_PxVec3_Pod;
    linear1:  physx_PxVec3_Pod;
    angular0: physx_PxVec3_Pod;
    angular1: physx_PxVec3_Pod;
}

physx_PxContactJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxFixedJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxJointLimitParameters_Pod :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
}

physx_PxJointLinearLimit_Pod :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    value:                      float;
}

physx_PxJointLinearLimitPair_Pod :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

physx_PxJointAngularLimitPair_Pod :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    upper:                      float;
    lower:                      float;
}

physx_PxJointLimitCone_Pod :: struct {
    restitution:                float;
    bounceThreshold:            float;
    stiffness:                  float;
    damping:                    float;
    contactDistance_deprecated: float;
    yAngle:                     float;
    zAngle:                     float;
}

physx_PxJointLimitPyramid_Pod :: struct {
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

physx_PxPrismaticJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRevoluteJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxSphericalJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxD6JointDrive_Pod :: struct {
    stiffness:  float;
    damping:    float;
    forceLimit: float;
    flags:      u32;
}

physx_PxD6Joint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGearJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxRackAndPinionJoint_Pod :: struct {
    structgen_pad0: [16] u8;
    userData:       *void;
}

physx_PxGroupsMask_Pod :: struct {
    bits0: u16;
    bits1: u16;
    bits2: u16;
    bits3: u16;
}

physx_PxDefaultErrorCallback_Pod :: struct {
    vtable_: *void;
}

physx_PxRigidActorExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxMassProperties_Pod :: struct {
    inertiaTensor: physx_PxMat33_Pod;
    centerOfMass:  physx_PxVec3_Pod;
    mass:          float;
}

physx_PxRigidBodyExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxShapeExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxMeshOverlapUtil_Pod :: struct {
    structgen_pad0: [1040] u8;
}

physx_PxBinaryConverter_Pod :: struct {}
physx_PxXmlMiscParameter_Pod :: struct {
    upVector: physx_PxVec3_Pod;
    scale:    physx_PxTolerancesScale_Pod;
}

physx_PxSerialization_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxDefaultCpuDispatcher_Pod :: struct {
    vtable_: *void;
}

physx_PxStringTableExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxBroadPhaseExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxSceneQueryExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxBatchQueryExt_Pod :: struct {
    vtable_: *void;
}

physx_PxCustomSceneQuerySystem_Pod :: struct {
    vtable_: *void;
}

physx_PxCustomSceneQuerySystemAdapter_Pod :: struct {
    vtable_: *void;
}

physx_PxSamplingExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxPoissonSampler_Pod :: struct {
    structgen_pad0: [8] u8;
}

physx_PxTriangleMeshPoissonSampler_Pod :: struct {
    structgen_pad0: [24] u8;
}

physx_PxTetrahedronMeshExt_Pod :: struct {
    structgen_pad0: [1] u8;
}

physx_PxRepXObject_Pod :: struct {
    typeName:     *u8;
    serializable: *void;
    id:           u64;
}

physx_PxCooking_Pod :: struct {}
physx_PxRepXInstantiationArgs_Pod :: struct {
    structgen_pad0: [8] u8;
    cooker:         *physx_PxCooking_Pod;
    stringTable:    *physx_PxStringTable_Pod;
}

physx_XmlMemoryAllocator_Pod :: struct {}
physx_XmlWriter_Pod :: struct {}
physx_XmlReader_Pod :: struct {}
physx_MemoryBuffer_Pod :: struct {}
physx_PxRepXSerializer_Pod :: struct {
    vtable_: *void;
}

physx_PxVehicleWheels4SimData_Pod :: struct {}
physx_PxVehicleWheels4DynData_Pod :: struct {}
physx_PxVehicleTireForceCalculator_Pod :: struct {}
physx_PxVehicleDrivableSurfaceToTireFrictionPairs_Pod :: struct {}
physx_PxVehicleTelemetryData_Pod :: struct {}

physx_PxPvd_Pod :: struct {
    vtable_: *void;
}

physx_PxPvdTransport_Pod :: struct {
    vtable_: *void;
}

PxAllocatorCallback_delete :: (self__pod: *physx_PxAllocatorCallback_Pod) -> void #foreign physx_sys;

PxAllocatorCallback_allocate_mut :: (self__pod: *physx_PxAllocatorCallback_Pod, size_pod: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign physx_sys;

PxAllocatorCallback_deallocate_mut :: (self__pod: *physx_PxAllocatorCallback_Pod, ptr: *void) -> void #foreign physx_sys;

PxAssertHandler_delete :: (self__pod: *physx_PxAssertHandler_Pod) -> void #foreign physx_sys;

phys_PxGetAssertHandler :: () -> *physx_PxAssertHandler_Pod #foreign physx_sys;

phys_PxSetAssertHandler :: (handler_pod: *physx_PxAssertHandler_Pod) -> void #foreign physx_sys;

PxFoundation_release_mut :: (self__pod: *physx_PxFoundation_Pod) -> void #foreign physx_sys;

PxFoundation_getErrorCallback_mut :: (self__pod: *physx_PxFoundation_Pod) -> *physx_PxErrorCallback_Pod #foreign physx_sys;

PxFoundation_setErrorLevel_mut :: (self__pod: *physx_PxFoundation_Pod, mask: u32) -> void #foreign physx_sys;

PxFoundation_getErrorLevel :: (self__pod: *physx_PxFoundation_Pod) -> u32 #foreign physx_sys;

PxFoundation_getAllocatorCallback_mut :: (self__pod: *physx_PxFoundation_Pod) -> *physx_PxAllocatorCallback_Pod #foreign physx_sys;

PxFoundation_getReportAllocationNames :: (self__pod: *physx_PxFoundation_Pod) -> bool #foreign physx_sys;

PxFoundation_setReportAllocationNames_mut :: (self__pod: *physx_PxFoundation_Pod, value: bool) -> void #foreign physx_sys;

PxFoundation_registerAllocationListener_mut :: (self__pod: *physx_PxFoundation_Pod, listener_pod: *physx_PxAllocationListener_Pod) -> void #foreign physx_sys;

PxFoundation_deregisterAllocationListener_mut :: (self__pod: *physx_PxFoundation_Pod, listener_pod: *physx_PxAllocationListener_Pod) -> void #foreign physx_sys;

PxFoundation_registerErrorCallback_mut :: (self__pod: *physx_PxFoundation_Pod, callback_pod: *physx_PxErrorCallback_Pod) -> void #foreign physx_sys;

PxFoundation_deregisterErrorCallback_mut :: (self__pod: *physx_PxFoundation_Pod, callback_pod: *physx_PxErrorCallback_Pod) -> void #foreign physx_sys;

phys_PxCreateFoundation :: (version: u32, allocator_pod: *physx_PxAllocatorCallback_Pod, errorCallback_pod: *physx_PxErrorCallback_Pod) -> *physx_PxFoundation_Pod #foreign physx_sys;

phys_PxSetFoundationInstance :: (foundation_pod: *physx_PxFoundation_Pod) -> void #foreign physx_sys;

phys_PxGetFoundation :: () -> *physx_PxFoundation_Pod #foreign physx_sys;

phys_PxGetProfilerCallback :: () -> *physx_PxProfilerCallback_Pod #foreign physx_sys;

phys_PxSetProfilerCallback :: (profiler_pod: *physx_PxProfilerCallback_Pod) -> void #foreign physx_sys;

phys_PxGetAllocatorCallback :: () -> *physx_PxAllocatorCallback_Pod #foreign physx_sys;

phys_PxGetBroadcastAllocator :: () -> *physx_PxAllocatorCallback_Pod #foreign physx_sys;

phys_PxGetErrorCallback :: () -> *physx_PxErrorCallback_Pod #foreign physx_sys;

phys_PxGetBroadcastError :: () -> *physx_PxErrorCallback_Pod #foreign physx_sys;

phys_PxGetWarnOnceTimeStamp :: () -> u32 #foreign physx_sys;

phys_PxDecFoundationRefCount :: () -> void #foreign physx_sys;

phys_PxIncFoundationRefCount :: () -> void #foreign physx_sys;

PxAllocator_new :: (anon_param0: *u8) -> physx_PxAllocator_Pod #foreign physx_sys;

PxAllocator_allocate_mut :: (self__pod: *physx_PxAllocator_Pod, size_pod: u64, file: *u8, line: s32) -> *void #foreign physx_sys;

PxAllocator_deallocate_mut :: (self__pod: *physx_PxAllocator_Pod, ptr: *void) -> void #foreign physx_sys;

PxRawAllocator_new :: (anon_param0: *u8) -> physx_PxRawAllocator_Pod #foreign physx_sys;

PxRawAllocator_allocate_mut :: (self__pod: *physx_PxRawAllocator_Pod, size_pod: u64, anon_param1: *u8, anon_param2: s32) -> *void #foreign physx_sys;

PxRawAllocator_deallocate_mut :: (self__pod: *physx_PxRawAllocator_Pod, ptr: *void) -> void #foreign physx_sys;

PxVirtualAllocatorCallback_delete :: (self__pod: *physx_PxVirtualAllocatorCallback_Pod) -> void #foreign physx_sys;

PxVirtualAllocatorCallback_allocate_mut :: (self__pod: *physx_PxVirtualAllocatorCallback_Pod, size_pod: u64, group: s32, file: *u8, line: s32) -> *void #foreign physx_sys;

PxVirtualAllocatorCallback_deallocate_mut :: (self__pod: *physx_PxVirtualAllocatorCallback_Pod, ptr: *void) -> void #foreign physx_sys;

PxVirtualAllocator_new :: (callback_pod: *physx_PxVirtualAllocatorCallback_Pod, group: s32) -> physx_PxVirtualAllocator_Pod #foreign physx_sys;

PxVirtualAllocator_allocate_mut :: (self__pod: *physx_PxVirtualAllocator_Pod, size_pod: u64, file: *u8, line: s32) -> *void #foreign physx_sys;

PxVirtualAllocator_deallocate_mut :: (self__pod: *physx_PxVirtualAllocator_Pod, ptr: *void) -> void #foreign physx_sys;

PxTempAllocatorChunk_new :: () -> physx_PxTempAllocatorChunk_Pod #foreign physx_sys;

PxTempAllocator_new :: (anon_param0: *u8) -> physx_PxTempAllocator_Pod #foreign physx_sys;

PxTempAllocator_allocate_mut :: (self__pod: *physx_PxTempAllocator_Pod, size_pod: u64, file: *u8, line: s32) -> *void #foreign physx_sys;

PxTempAllocator_deallocate_mut :: (self__pod: *physx_PxTempAllocator_Pod, ptr: *void) -> void #foreign physx_sys;

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

PxVec3_new :: () -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_new_1 :: (anon_param0_pod: s32) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_new_2 :: (a: float) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_new_3 :: (nx: float, ny: float, nz: float) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_isZero :: (self__pod: *physx_PxVec3_Pod) -> bool #foreign physx_sys;

PxVec3_isFinite :: (self__pod: *physx_PxVec3_Pod) -> bool #foreign physx_sys;

PxVec3_isNormalized :: (self__pod: *physx_PxVec3_Pod) -> bool #foreign physx_sys;

PxVec3_magnitudeSquared :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_magnitude :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_dot :: (self__pod: *physx_PxVec3_Pod, v_pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_cross :: (self__pod: *physx_PxVec3_Pod, v_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_getNormalized :: (self__pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_normalize_mut :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_normalizeSafe_mut :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_normalizeFast_mut :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_multiply :: (self__pod: *physx_PxVec3_Pod, a_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_minimum :: (self__pod: *physx_PxVec3_Pod, v_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_minElement :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_maximum :: (self__pod: *physx_PxVec3_Pod, v_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3_maxElement :: (self__pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxVec3_abs :: (self__pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxVec3Padded_new_alloc :: () -> *physx_PxVec3Padded_Pod #foreign physx_sys;

PxVec3Padded_delete :: (self__pod: *physx_PxVec3Padded_Pod) -> void #foreign physx_sys;

PxVec3Padded_new_alloc_1 :: (p_pod: *physx_PxVec3_Pod) -> *physx_PxVec3Padded_Pod #foreign physx_sys;

PxVec3Padded_new_alloc_2 :: (f: float) -> *physx_PxVec3Padded_Pod #foreign physx_sys;

PxQuat_new :: () -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_new_1 :: (anon_param0_pod: s32) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_new_2 :: (r: float) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_new_3 :: (nx: float, ny: float, nz: float, nw: float) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_new_4 :: (angleRadians: float, unitAxis_pod: *physx_PxVec3_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_new_5 :: (m_pod: *physx_PxMat33_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_isIdentity :: (self__pod: *physx_PxQuat_Pod) -> bool #foreign physx_sys;

PxQuat_isFinite :: (self__pod: *physx_PxQuat_Pod) -> bool #foreign physx_sys;

PxQuat_isUnit :: (self__pod: *physx_PxQuat_Pod) -> bool #foreign physx_sys;

PxQuat_isSane :: (self__pod: *physx_PxQuat_Pod) -> bool #foreign physx_sys;

PxQuat_toRadiansAndUnitAxis :: (self__pod: *physx_PxQuat_Pod, angle_pod: *float, axis_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxQuat_getAngle :: (self__pod: *physx_PxQuat_Pod) -> float #foreign physx_sys;

PxQuat_getAngle_1 :: (self__pod: *physx_PxQuat_Pod, q_pod: *physx_PxQuat_Pod) -> float #foreign physx_sys;

PxQuat_magnitudeSquared :: (self__pod: *physx_PxQuat_Pod) -> float #foreign physx_sys;

PxQuat_dot :: (self__pod: *physx_PxQuat_Pod, v_pod: *physx_PxQuat_Pod) -> float #foreign physx_sys;

PxQuat_getNormalized :: (self__pod: *physx_PxQuat_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_magnitude :: (self__pod: *physx_PxQuat_Pod) -> float #foreign physx_sys;

PxQuat_normalize_mut :: (self__pod: *physx_PxQuat_Pod) -> float #foreign physx_sys;

PxQuat_getConjugate :: (self__pod: *physx_PxQuat_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

PxQuat_getImaginaryPart :: (self__pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxQuat_getBasisVector0 :: (self__pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxQuat_getBasisVector1 :: (self__pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxQuat_getBasisVector2 :: (self__pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxQuat_rotate :: (self__pod: *physx_PxQuat_Pod, v_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxQuat_rotateInv :: (self__pod: *physx_PxQuat_Pod, v_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxTransform_new :: () -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_new_1 :: (position_pod: *physx_PxVec3_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_new_2 :: (anon_param0_pod: s32) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_new_3 :: (orientation_pod: *physx_PxQuat_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_new_4 :: (x: float, y: float, z: float, aQ_pod: physx_PxQuat_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_new_5 :: (p0_pod: *physx_PxVec3_Pod, q0_pod: *physx_PxQuat_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_new_6 :: (m_pod: *physx_PxMat44_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_getInverse :: (self__pod: *physx_PxTransform_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_transform :: (self__pod: *physx_PxTransform_Pod, input_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxTransform_transformInv :: (self__pod: *physx_PxTransform_Pod, input_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxTransform_rotate :: (self__pod: *physx_PxTransform_Pod, input_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxTransform_rotateInv :: (self__pod: *physx_PxTransform_Pod, input_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxTransform_transform_1 :: (self__pod: *physx_PxTransform_Pod, src_pod: *physx_PxTransform_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_isValid :: (self__pod: *physx_PxTransform_Pod) -> bool #foreign physx_sys;

PxTransform_isSane :: (self__pod: *physx_PxTransform_Pod) -> bool #foreign physx_sys;

PxTransform_isFinite :: (self__pod: *physx_PxTransform_Pod) -> bool #foreign physx_sys;

PxTransform_transformInv_1 :: (self__pod: *physx_PxTransform_Pod, src_pod: *physx_PxTransform_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxTransform_getNormalized :: (self__pod: *physx_PxTransform_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxMat33_new :: () -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_new_1 :: (anon_param0_pod: s32) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_new_2 :: (anon_param0_pod: s32) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_new_3 :: (col0_pod: *physx_PxVec3_Pod, col1_pod: *physx_PxVec3_Pod, col2_pod: *physx_PxVec3_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_new_4 :: (r: float) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_new_5 :: (values: *float) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_new_6 :: (q_pod: *physx_PxQuat_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_createDiagonal :: (d_pod: *physx_PxVec3_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_outer :: (a_pod: *physx_PxVec3_Pod, b_pod: *physx_PxVec3_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_getTranspose :: (self__pod: *physx_PxMat33_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_getInverse :: (self__pod: *physx_PxMat33_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMat33_getDeterminant :: (self__pod: *physx_PxMat33_Pod) -> float #foreign physx_sys;

PxMat33_transform :: (self__pod: *physx_PxMat33_Pod, other_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat33_transformTranspose :: (self__pod: *physx_PxMat33_Pod, other_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat33_front :: (self__pod: *physx_PxMat33_Pod) -> *float #foreign physx_sys;

PxBounds3_new :: () -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_new_1 :: (minimum_pod: *physx_PxVec3_Pod, maximum_pod: *physx_PxVec3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_empty :: () -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_boundsOfPoints :: (v0_pod: *physx_PxVec3_Pod, v1_pod: *physx_PxVec3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_centerExtents :: (center_pod: *physx_PxVec3_Pod, extent_pod: *physx_PxVec3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_basisExtent :: (center_pod: *physx_PxVec3_Pod, basis_pod: *physx_PxMat33_Pod, extent_pod: *physx_PxVec3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_poseExtent :: (pose_pod: *physx_PxTransform_Pod, extent_pod: *physx_PxVec3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_transformSafe :: (matrix_pod: *physx_PxMat33_Pod, bounds_pod: *physx_PxBounds3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_transformFast :: (matrix_pod: *physx_PxMat33_Pod, bounds_pod: *physx_PxBounds3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_transformSafe_1 :: (transform_pod: *physx_PxTransform_Pod, bounds_pod: *physx_PxBounds3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_transformFast_1 :: (transform_pod: *physx_PxTransform_Pod, bounds_pod: *physx_PxBounds3_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxBounds3_setEmpty_mut :: (self__pod: *physx_PxBounds3_Pod) -> void #foreign physx_sys;

PxBounds3_setMaximal_mut :: (self__pod: *physx_PxBounds3_Pod) -> void #foreign physx_sys;

PxBounds3_include_mut :: (self__pod: *physx_PxBounds3_Pod, v_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxBounds3_include_mut_1 :: (self__pod: *physx_PxBounds3_Pod, b_pod: *physx_PxBounds3_Pod) -> void #foreign physx_sys;

PxBounds3_isEmpty :: (self__pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBounds3_intersects :: (self__pod: *physx_PxBounds3_Pod, b_pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBounds3_intersects1D :: (self__pod: *physx_PxBounds3_Pod, a_pod: *physx_PxBounds3_Pod, axis: u32) -> bool #foreign physx_sys;

PxBounds3_contains :: (self__pod: *physx_PxBounds3_Pod, v_pod: *physx_PxVec3_Pod) -> bool #foreign physx_sys;

PxBounds3_isInside :: (self__pod: *physx_PxBounds3_Pod, box_pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBounds3_getCenter :: (self__pod: *physx_PxBounds3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxBounds3_getCenter_1 :: (self__pod: *physx_PxBounds3_Pod, axis: u32) -> float #foreign physx_sys;

PxBounds3_getExtents :: (self__pod: *physx_PxBounds3_Pod, axis: u32) -> float #foreign physx_sys;

PxBounds3_getDimensions :: (self__pod: *physx_PxBounds3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxBounds3_getExtents_1 :: (self__pod: *physx_PxBounds3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxBounds3_scaleSafe_mut :: (self__pod: *physx_PxBounds3_Pod, scale: float) -> void #foreign physx_sys;

PxBounds3_scaleFast_mut :: (self__pod: *physx_PxBounds3_Pod, scale: float) -> void #foreign physx_sys;

PxBounds3_fattenSafe_mut :: (self__pod: *physx_PxBounds3_Pod, distance: float) -> void #foreign physx_sys;

PxBounds3_fattenFast_mut :: (self__pod: *physx_PxBounds3_Pod, distance: float) -> void #foreign physx_sys;

PxBounds3_isFinite :: (self__pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBounds3_isValid :: (self__pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBounds3_closestPoint :: (self__pod: *physx_PxBounds3_Pod, p_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxErrorCallback_delete :: (self__pod: *physx_PxErrorCallback_Pod) -> void #foreign physx_sys;

PxErrorCallback_reportError_mut :: (self__pod: *physx_PxErrorCallback_Pod, code_pod: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys;

PxAllocationListener_onAllocation_mut :: (self__pod: *physx_PxAllocationListener_Pod, size_pod: u64, typeName: *u8, filename: *u8, line: s32, allocatedMemory: *void) -> void #foreign physx_sys;

PxAllocationListener_onDeallocation_mut :: (self__pod: *physx_PxAllocationListener_Pod, allocatedMemory: *void) -> void #foreign physx_sys;

PxBroadcastingAllocator_new_alloc :: (allocator_pod: *physx_PxAllocatorCallback_Pod, error_pod: *physx_PxErrorCallback_Pod) -> *physx_PxBroadcastingAllocator_Pod #foreign physx_sys;

PxBroadcastingAllocator_delete :: (self__pod: *physx_PxBroadcastingAllocator_Pod) -> void #foreign physx_sys;

PxBroadcastingAllocator_allocate_mut :: (self__pod: *physx_PxBroadcastingAllocator_Pod, size_pod: u64, typeName: *u8, filename: *u8, line: s32) -> *void #foreign physx_sys;

PxBroadcastingAllocator_deallocate_mut :: (self__pod: *physx_PxBroadcastingAllocator_Pod, ptr: *void) -> void #foreign physx_sys;

PxBroadcastingErrorCallback_new_alloc :: (errorCallback_pod: *physx_PxErrorCallback_Pod) -> *physx_PxBroadcastingErrorCallback_Pod #foreign physx_sys;

PxBroadcastingErrorCallback_delete :: (self__pod: *physx_PxBroadcastingErrorCallback_Pod) -> void #foreign physx_sys;

PxBroadcastingErrorCallback_reportError_mut :: (self__pod: *physx_PxBroadcastingErrorCallback_Pod, code_pod: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys;

phys_PxEnableFPExceptions :: () -> void #foreign physx_sys;

phys_PxDisableFPExceptions :: () -> void #foreign physx_sys;

PxInputStream_read_mut :: (self__pod: *physx_PxInputStream_Pod, dest: *void, count: u32) -> u32 #foreign physx_sys;

PxInputStream_delete :: (self__pod: *physx_PxInputStream_Pod) -> void #foreign physx_sys;

PxInputData_getLength :: (self__pod: *physx_PxInputData_Pod) -> u32 #foreign physx_sys;

PxInputData_seek_mut :: (self__pod: *physx_PxInputData_Pod, offset: u32) -> void #foreign physx_sys;

PxInputData_tell :: (self__pod: *physx_PxInputData_Pod) -> u32 #foreign physx_sys;

PxInputData_delete :: (self__pod: *physx_PxInputData_Pod) -> void #foreign physx_sys;

PxOutputStream_write_mut :: (self__pod: *physx_PxOutputStream_Pod, src: *void, count: u32) -> u32 #foreign physx_sys;

PxOutputStream_delete :: (self__pod: *physx_PxOutputStream_Pod) -> void #foreign physx_sys;

PxVec4_new :: () -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_new_1 :: (anon_param0_pod: s32) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_new_2 :: (a: float) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_new_3 :: (nx: float, ny: float, nz: float, nw: float) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_new_4 :: (v_pod: *physx_PxVec3_Pod, nw: float) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_new_5 :: (v: *float) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_isZero :: (self__pod: *physx_PxVec4_Pod) -> bool #foreign physx_sys;

PxVec4_isFinite :: (self__pod: *physx_PxVec4_Pod) -> bool #foreign physx_sys;

PxVec4_isNormalized :: (self__pod: *physx_PxVec4_Pod) -> bool #foreign physx_sys;

PxVec4_magnitudeSquared :: (self__pod: *physx_PxVec4_Pod) -> float #foreign physx_sys;

PxVec4_magnitude :: (self__pod: *physx_PxVec4_Pod) -> float #foreign physx_sys;

PxVec4_dot :: (self__pod: *physx_PxVec4_Pod, v_pod: *physx_PxVec4_Pod) -> float #foreign physx_sys;

PxVec4_getNormalized :: (self__pod: *physx_PxVec4_Pod) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_normalize_mut :: (self__pod: *physx_PxVec4_Pod) -> float #foreign physx_sys;

PxVec4_multiply :: (self__pod: *physx_PxVec4_Pod, a_pod: *physx_PxVec4_Pod) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_minimum :: (self__pod: *physx_PxVec4_Pod, v_pod: *physx_PxVec4_Pod) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_maximum :: (self__pod: *physx_PxVec4_Pod, v_pod: *physx_PxVec4_Pod) -> physx_PxVec4_Pod #foreign physx_sys;

PxVec4_getXYZ :: (self__pod: *physx_PxVec4_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat44_new :: () -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_1 :: (anon_param0_pod: s32) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_2 :: (anon_param0_pod: s32) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_3 :: (col0_pod: *physx_PxVec4_Pod, col1_pod: *physx_PxVec4_Pod, col2_pod: *physx_PxVec4_Pod, col3_pod: *physx_PxVec4_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_4 :: (r: float) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_5 :: (col0_pod: *physx_PxVec3_Pod, col1_pod: *physx_PxVec3_Pod, col2_pod: *physx_PxVec3_Pod, col3_pod: *physx_PxVec3_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_6 :: (values: *float) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_7 :: (q_pod: *physx_PxQuat_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_8 :: (diagonal_pod: *physx_PxVec4_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_9 :: (axes_pod: *physx_PxMat33_Pod, position_pod: *physx_PxVec3_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_new_10 :: (t_pod: *physx_PxTransform_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_getTranspose :: (self__pod: *physx_PxMat44_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_transform :: (self__pod: *physx_PxMat44_Pod, other_pod: *physx_PxVec4_Pod) -> physx_PxVec4_Pod #foreign physx_sys;

PxMat44_transform_1 :: (self__pod: *physx_PxMat44_Pod, other_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat44_rotate :: (self__pod: *physx_PxMat44_Pod, other_pod: *physx_PxVec4_Pod) -> physx_PxVec4_Pod #foreign physx_sys;

PxMat44_rotate_1 :: (self__pod: *physx_PxMat44_Pod, other_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat44_getBasis :: (self__pod: *physx_PxMat44_Pod, num: u32) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat44_getPosition :: (self__pod: *physx_PxMat44_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMat44_setPosition_mut :: (self__pod: *physx_PxMat44_Pod, position_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxMat44_front :: (self__pod: *physx_PxMat44_Pod) -> *float #foreign physx_sys;

PxMat44_scale_mut :: (self__pod: *physx_PxMat44_Pod, p_pod: *physx_PxVec4_Pod) -> void #foreign physx_sys;

PxMat44_inverseRT :: (self__pod: *physx_PxMat44_Pod) -> physx_PxMat44_Pod #foreign physx_sys;

PxMat44_isFinite :: (self__pod: *physx_PxMat44_Pod) -> bool #foreign physx_sys;

PxPlane_new :: () -> physx_PxPlane_Pod #foreign physx_sys;

PxPlane_new_1 :: (nx: float, ny: float, nz: float, distance: float) -> physx_PxPlane_Pod #foreign physx_sys;

PxPlane_new_2 :: (normal_pod: *physx_PxVec3_Pod, distance: float) -> physx_PxPlane_Pod #foreign physx_sys;

PxPlane_new_3 :: (point_pod: *physx_PxVec3_Pod, normal_pod: *physx_PxVec3_Pod) -> physx_PxPlane_Pod #foreign physx_sys;

PxPlane_new_4 :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, p2_pod: *physx_PxVec3_Pod) -> physx_PxPlane_Pod #foreign physx_sys;

PxPlane_distance :: (self__pod: *physx_PxPlane_Pod, p_pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxPlane_contains :: (self__pod: *physx_PxPlane_Pod, p_pod: *physx_PxVec3_Pod) -> bool #foreign physx_sys;

PxPlane_project :: (self__pod: *physx_PxPlane_Pod, p_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxPlane_pointInPlane :: (self__pod: *physx_PxPlane_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxPlane_normalize_mut :: (self__pod: *physx_PxPlane_Pod) -> void #foreign physx_sys;

PxPlane_transform :: (self__pod: *physx_PxPlane_Pod, pose_pod: *physx_PxTransform_Pod) -> physx_PxPlane_Pod #foreign physx_sys;

PxPlane_inverseTransform :: (self__pod: *physx_PxPlane_Pod, pose_pod: *physx_PxTransform_Pod) -> physx_PxPlane_Pod #foreign physx_sys;

phys_PxShortestRotation :: (from_pod: *physx_PxVec3_Pod, target_pod: *physx_PxVec3_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

phys_PxDiagonalize :: (m_pod: *physx_PxMat33_Pod, axes_pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

phys_PxTransformFromSegment :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, halfHeight: *float) -> physx_PxTransform_Pod #foreign physx_sys;

phys_PxTransformFromPlaneEquation :: (plane_pod: *physx_PxPlane_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

phys_PxPlaneEquationFromTransform :: (pose_pod: *physx_PxTransform_Pod) -> physx_PxPlane_Pod #foreign physx_sys;

phys_PxSlerp :: (t: float, left_pod: *physx_PxQuat_Pod, right_pod: *physx_PxQuat_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

phys_PxIntegrateTransform :: (curTrans_pod: *physx_PxTransform_Pod, linvel_pod: *physx_PxVec3_Pod, angvel_pod: *physx_PxVec3_Pod, timeStep: float, result_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

phys_PxExp :: (v_pod: *physx_PxVec3_Pod) -> physx_PxQuat_Pod #foreign physx_sys;

phys_PxOptimizeBoundingBox :: (basis_pod: *physx_PxMat33_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

phys_PxLog :: (q_pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

phys_PxLargestAxis :: (v_pod: *physx_PxVec3_Pod) -> u32 #foreign physx_sys;

phys_PxTanHalf :: (sin: float, cos: float) -> float #foreign physx_sys;

phys_PxEllipseClamp :: (point_pod: *physx_PxVec3_Pod, radii_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

phys_PxSeparateSwingTwist :: (q_pod: *physx_PxQuat_Pod, swing_pod: *physx_PxQuat_Pod, twist_pod: *physx_PxQuat_Pod) -> void #foreign physx_sys;

phys_PxComputeAngle :: (v0_pod: *physx_PxVec3_Pod, v1_pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

phys_PxComputeBasisVectors :: (dir_pod: *physx_PxVec3_Pod, right_pod: *physx_PxVec3_Pod, up_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

phys_PxComputeBasisVectors_1 :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, dir_pod: *physx_PxVec3_Pod, right_pod: *physx_PxVec3_Pod, up_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

phys_PxGetNextIndex3 :: (i: u32) -> u32 #foreign physx_sys;

phys_computeBarycentric :: (a_pod: *physx_PxVec3_Pod, b_pod: *physx_PxVec3_Pod, c_pod: *physx_PxVec3_Pod, d_pod: *physx_PxVec3_Pod, p_pod: *physx_PxVec3_Pod, bary_pod: *physx_PxVec4_Pod) -> void #foreign physx_sys;

phys_computeBarycentric_1 :: (a_pod: *physx_PxVec3_Pod, b_pod: *physx_PxVec3_Pod, c_pod: *physx_PxVec3_Pod, p_pod: *physx_PxVec3_Pod, bary_pod: *physx_PxVec4_Pod) -> void #foreign physx_sys;

Interpolation_PxLerp :: (a: float, b: float, t: float) -> float #foreign physx_sys;

Interpolation_PxBiLerp :: (f00: float, f10: float, f01: float, f11: float, tx: float, ty: float) -> float #foreign physx_sys;

Interpolation_PxTriLerp :: (f000: float, f100: float, f010: float, f110: float, f001: float, f101: float, f011: float, f111: float, tx: float, ty: float, tz: float) -> float #foreign physx_sys;

Interpolation_PxSDFIdx :: (i: u32, j: u32, k: u32, nbX: u32, nbY: u32) -> u32 #foreign physx_sys;

Interpolation_PxSDFSampleImpl :: (sdf: *float, localPos_pod: *physx_PxVec3_Pod, sdfBoxLower_pod: *physx_PxVec3_Pod, sdfBoxHigher_pod: *physx_PxVec3_Pod, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, tolerance: float) -> float #foreign physx_sys;

phys_PxSdfSample :: (sdf: *float, localPos_pod: *physx_PxVec3_Pod, sdfBoxLower_pod: *physx_PxVec3_Pod, sdfBoxHigher_pod: *physx_PxVec3_Pod, sdfDx: float, invSdfDx: float, dimX: u32, dimY: u32, dimZ: u32, gradient_pod: *physx_PxVec3_Pod, tolerance: float) -> float #foreign physx_sys;

PxMutexImpl_new_alloc :: () -> *physx_PxMutexImpl_Pod #foreign physx_sys;

PxMutexImpl_delete :: (self__pod: *physx_PxMutexImpl_Pod) -> void #foreign physx_sys;

PxMutexImpl_lock_mut :: (self__pod: *physx_PxMutexImpl_Pod) -> void #foreign physx_sys;

PxMutexImpl_trylock_mut :: (self__pod: *physx_PxMutexImpl_Pod) -> bool #foreign physx_sys;

PxMutexImpl_unlock_mut :: (self__pod: *physx_PxMutexImpl_Pod) -> void #foreign physx_sys;

PxMutexImpl_getSize :: () -> u32 #foreign physx_sys;

PxReadWriteLock_new_alloc :: () -> *physx_PxReadWriteLock_Pod #foreign physx_sys;

PxReadWriteLock_delete :: (self__pod: *physx_PxReadWriteLock_Pod) -> void #foreign physx_sys;

PxReadWriteLock_lockReader_mut :: (self__pod: *physx_PxReadWriteLock_Pod, takeLock: bool) -> void #foreign physx_sys;

PxReadWriteLock_lockWriter_mut :: (self__pod: *physx_PxReadWriteLock_Pod) -> void #foreign physx_sys;

PxReadWriteLock_unlockReader_mut :: (self__pod: *physx_PxReadWriteLock_Pod) -> void #foreign physx_sys;

PxReadWriteLock_unlockWriter_mut :: (self__pod: *physx_PxReadWriteLock_Pod) -> void #foreign physx_sys;

PxProfilerCallback_zoneStart_mut :: (self__pod: *physx_PxProfilerCallback_Pod, eventName: *u8, detached: bool, contextId: u64) -> *void #foreign physx_sys;

PxProfilerCallback_zoneEnd_mut :: (self__pod: *physx_PxProfilerCallback_Pod, profilerData: *void, eventName: *u8, detached: bool, contextId: u64) -> void #foreign physx_sys;

PxProfileScoped_new_alloc :: (callback_pod: *physx_PxProfilerCallback_Pod, eventName: *u8, detached: bool, contextId: u64) -> *physx_PxProfileScoped_Pod #foreign physx_sys;

PxProfileScoped_delete :: (self__pod: *physx_PxProfileScoped_Pod) -> void #foreign physx_sys;

PxSListEntry_new :: () -> physx_PxSListEntry_Pod #foreign physx_sys;

PxSListEntry_next_mut :: (self__pod: *physx_PxSListEntry_Pod) -> *physx_PxSListEntry_Pod #foreign physx_sys;

PxSListImpl_new_alloc :: () -> *physx_PxSListImpl_Pod #foreign physx_sys;

PxSListImpl_delete :: (self__pod: *physx_PxSListImpl_Pod) -> void #foreign physx_sys;

PxSListImpl_push_mut :: (self__pod: *physx_PxSListImpl_Pod, entry_pod: *physx_PxSListEntry_Pod) -> void #foreign physx_sys;

PxSListImpl_pop_mut :: (self__pod: *physx_PxSListImpl_Pod) -> *physx_PxSListEntry_Pod #foreign physx_sys;

PxSListImpl_flush_mut :: (self__pod: *physx_PxSListImpl_Pod) -> *physx_PxSListEntry_Pod #foreign physx_sys;

PxSListImpl_getSize :: () -> u32 #foreign physx_sys;

PxSyncImpl_new_alloc :: () -> *physx_PxSyncImpl_Pod #foreign physx_sys;

PxSyncImpl_delete :: (self__pod: *physx_PxSyncImpl_Pod) -> void #foreign physx_sys;

PxSyncImpl_wait_mut :: (self__pod: *physx_PxSyncImpl_Pod, milliseconds: u32) -> bool #foreign physx_sys;

PxSyncImpl_set_mut :: (self__pod: *physx_PxSyncImpl_Pod) -> void #foreign physx_sys;

PxSyncImpl_reset_mut :: (self__pod: *physx_PxSyncImpl_Pod) -> void #foreign physx_sys;

PxSyncImpl_getSize :: () -> u32 #foreign physx_sys;

PxRunnable_new_alloc :: () -> *physx_PxRunnable_Pod #foreign physx_sys;

PxRunnable_delete :: (self__pod: *physx_PxRunnable_Pod) -> void #foreign physx_sys;

PxRunnable_execute_mut :: (self__pod: *physx_PxRunnable_Pod) -> void #foreign physx_sys;

phys_PxTlsAlloc :: () -> u32 #foreign physx_sys;

phys_PxTlsFree :: (index: u32) -> void #foreign physx_sys;

phys_PxTlsGet :: (index: u32) -> *void #foreign physx_sys;

phys_PxTlsGetValue :: (index: u32) -> u64 #foreign physx_sys;

phys_PxTlsSet :: (index: u32, value: *void) -> u32 #foreign physx_sys;

phys_PxTlsSetValue :: (index: u32, value_pod: u64) -> u32 #foreign physx_sys;

PxCounterFrequencyToTensOfNanos_new :: (inNum: u64, inDenom: u64) -> physx_PxCounterFrequencyToTensOfNanos_Pod #foreign physx_sys;

PxCounterFrequencyToTensOfNanos_toTensOfNanos :: (self__pod: *physx_PxCounterFrequencyToTensOfNanos_Pod, inCounter: u64) -> u64 #foreign physx_sys;

PxTime_getBootCounterFrequency :: () -> *physx_PxCounterFrequencyToTensOfNanos_Pod #foreign physx_sys;

PxTime_getCounterFrequency :: () -> physx_PxCounterFrequencyToTensOfNanos_Pod #foreign physx_sys;

PxTime_getCurrentCounterValue :: () -> u64 #foreign physx_sys;

PxTime_getCurrentTimeInTensOfNanoSeconds :: () -> u64 #foreign physx_sys;

PxTime_new :: () -> physx_PxTime_Pod #foreign physx_sys;

PxTime_getElapsedSeconds_mut :: (self__pod: *physx_PxTime_Pod) -> float64 #foreign physx_sys;

PxTime_peekElapsedSeconds_mut :: (self__pod: *physx_PxTime_Pod) -> float64 #foreign physx_sys;

PxTime_getLastTime :: (self__pod: *physx_PxTime_Pod) -> float64 #foreign physx_sys;

PxVec2_new :: () -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_new_1 :: (anon_param0_pod: s32) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_new_2 :: (a: float) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_new_3 :: (nx: float, ny: float) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_isZero :: (self__pod: *physx_PxVec2_Pod) -> bool #foreign physx_sys;

PxVec2_isFinite :: (self__pod: *physx_PxVec2_Pod) -> bool #foreign physx_sys;

PxVec2_isNormalized :: (self__pod: *physx_PxVec2_Pod) -> bool #foreign physx_sys;

PxVec2_magnitudeSquared :: (self__pod: *physx_PxVec2_Pod) -> float #foreign physx_sys;

PxVec2_magnitude :: (self__pod: *physx_PxVec2_Pod) -> float #foreign physx_sys;

PxVec2_dot :: (self__pod: *physx_PxVec2_Pod, v_pod: *physx_PxVec2_Pod) -> float #foreign physx_sys;

PxVec2_getNormalized :: (self__pod: *physx_PxVec2_Pod) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_normalize_mut :: (self__pod: *physx_PxVec2_Pod) -> float #foreign physx_sys;

PxVec2_multiply :: (self__pod: *physx_PxVec2_Pod, a_pod: *physx_PxVec2_Pod) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_minimum :: (self__pod: *physx_PxVec2_Pod, v_pod: *physx_PxVec2_Pod) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_minElement :: (self__pod: *physx_PxVec2_Pod) -> float #foreign physx_sys;

PxVec2_maximum :: (self__pod: *physx_PxVec2_Pod, v_pod: *physx_PxVec2_Pod) -> physx_PxVec2_Pod #foreign physx_sys;

PxVec2_maxElement :: (self__pod: *physx_PxVec2_Pod) -> float #foreign physx_sys;

PxStridedData_new :: () -> physx_PxStridedData_Pod #foreign physx_sys;

PxBoundedData_new :: () -> physx_PxBoundedData_Pod #foreign physx_sys;

PxDebugPoint_new :: (p_pod: *physx_PxVec3_Pod, c_pod: *u32) -> physx_PxDebugPoint_Pod #foreign physx_sys;

PxDebugLine_new :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, c_pod: *u32) -> physx_PxDebugLine_Pod #foreign physx_sys;

PxDebugTriangle_new :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, p2_pod: *physx_PxVec3_Pod, c_pod: *u32) -> physx_PxDebugTriangle_Pod #foreign physx_sys;

PxDebugText_new :: () -> physx_PxDebugText_Pod #foreign physx_sys;

PxDebugText_new_1 :: (pos_pod: *physx_PxVec3_Pod, sz_pod: *float, clr_pod: *u32, str: *u8) -> physx_PxDebugText_Pod #foreign physx_sys;

PxRenderBuffer_delete :: (self__pod: *physx_PxRenderBuffer_Pod) -> void #foreign physx_sys;

PxRenderBuffer_getNbPoints :: (self__pod: *physx_PxRenderBuffer_Pod) -> u32 #foreign physx_sys;

PxRenderBuffer_getPoints :: (self__pod: *physx_PxRenderBuffer_Pod) -> *physx_PxDebugPoint_Pod #foreign physx_sys;

PxRenderBuffer_addPoint_mut :: (self__pod: *physx_PxRenderBuffer_Pod, point_pod: *physx_PxDebugPoint_Pod) -> void #foreign physx_sys;

PxRenderBuffer_getNbLines :: (self__pod: *physx_PxRenderBuffer_Pod) -> u32 #foreign physx_sys;

PxRenderBuffer_getLines :: (self__pod: *physx_PxRenderBuffer_Pod) -> *physx_PxDebugLine_Pod #foreign physx_sys;

PxRenderBuffer_addLine_mut :: (self__pod: *physx_PxRenderBuffer_Pod, line_pod: *physx_PxDebugLine_Pod) -> void #foreign physx_sys;

PxRenderBuffer_reserveLines_mut :: (self__pod: *physx_PxRenderBuffer_Pod, nbLines: u32) -> *physx_PxDebugLine_Pod #foreign physx_sys;

PxRenderBuffer_reservePoints_mut :: (self__pod: *physx_PxRenderBuffer_Pod, nbLines: u32) -> *physx_PxDebugPoint_Pod #foreign physx_sys;

PxRenderBuffer_getNbTriangles :: (self__pod: *physx_PxRenderBuffer_Pod) -> u32 #foreign physx_sys;

PxRenderBuffer_getTriangles :: (self__pod: *physx_PxRenderBuffer_Pod) -> *physx_PxDebugTriangle_Pod #foreign physx_sys;

PxRenderBuffer_addTriangle_mut :: (self__pod: *physx_PxRenderBuffer_Pod, triangle_pod: *physx_PxDebugTriangle_Pod) -> void #foreign physx_sys;

PxRenderBuffer_append_mut :: (self__pod: *physx_PxRenderBuffer_Pod, other_pod: *physx_PxRenderBuffer_Pod) -> void #foreign physx_sys;

PxRenderBuffer_clear_mut :: (self__pod: *physx_PxRenderBuffer_Pod) -> void #foreign physx_sys;

PxRenderBuffer_shift_mut :: (self__pod: *physx_PxRenderBuffer_Pod, delta_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxRenderBuffer_empty :: (self__pod: *physx_PxRenderBuffer_Pod) -> bool #foreign physx_sys;

PxProcessPxBaseCallback_delete :: (self__pod: *physx_PxProcessPxBaseCallback_Pod) -> void #foreign physx_sys;

PxProcessPxBaseCallback_process_mut :: (self__pod: *physx_PxProcessPxBaseCallback_Pod, anon_param0_pod: *physx_PxBase_Pod) -> void #foreign physx_sys;

PxSerializationContext_registerReference_mut :: (self__pod: *physx_PxSerializationContext_Pod, base_pod: *physx_PxBase_Pod, kind: u32, reference_pod: u64) -> void #foreign physx_sys;

PxSerializationContext_getCollection :: (self__pod: *physx_PxSerializationContext_Pod) -> *physx_PxCollection_Pod #foreign physx_sys;

PxSerializationContext_writeData_mut :: (self__pod: *physx_PxSerializationContext_Pod, data: *void, size: u32) -> void #foreign physx_sys;

PxSerializationContext_alignData_mut :: (self__pod: *physx_PxSerializationContext_Pod, alignment: u32) -> void #foreign physx_sys;

PxSerializationContext_writeName_mut :: (self__pod: *physx_PxSerializationContext_Pod, name: *u8) -> void #foreign physx_sys;

PxDeserializationContext_resolveReference :: (self__pod: *physx_PxDeserializationContext_Pod, kind: u32, reference_pod: u64) -> *physx_PxBase_Pod #foreign physx_sys;

PxDeserializationContext_readName_mut :: (self__pod: *physx_PxDeserializationContext_Pod, name_pod: **u8) -> void #foreign physx_sys;

PxDeserializationContext_alignExtraData_mut :: (self__pod: *physx_PxDeserializationContext_Pod, alignment: u32) -> void #foreign physx_sys;

PxSerializationRegistry_registerSerializer_mut :: (self__pod: *physx_PxSerializationRegistry_Pod, type: u16, serializer_pod: *physx_PxSerializer_Pod) -> void #foreign physx_sys;

PxSerializationRegistry_unregisterSerializer_mut :: (self__pod: *physx_PxSerializationRegistry_Pod, type: u16) -> *physx_PxSerializer_Pod #foreign physx_sys;

PxSerializationRegistry_getSerializer :: (self__pod: *physx_PxSerializationRegistry_Pod, type: u16) -> *physx_PxSerializer_Pod #foreign physx_sys;

PxSerializationRegistry_registerRepXSerializer_mut :: (self__pod: *physx_PxSerializationRegistry_Pod, type: u16, serializer_pod: *physx_PxRepXSerializer_Pod) -> void #foreign physx_sys;

PxSerializationRegistry_unregisterRepXSerializer_mut :: (self__pod: *physx_PxSerializationRegistry_Pod, type: u16) -> *physx_PxRepXSerializer_Pod #foreign physx_sys;

PxSerializationRegistry_getRepXSerializer :: (self__pod: *physx_PxSerializationRegistry_Pod, typeName: *u8) -> *physx_PxRepXSerializer_Pod #foreign physx_sys;

PxSerializationRegistry_release_mut :: (self__pod: *physx_PxSerializationRegistry_Pod) -> void #foreign physx_sys;

PxCollection_add_mut :: (self__pod: *physx_PxCollection_Pod, object_pod: *physx_PxBase_Pod, id: u64) -> void #foreign physx_sys;

PxCollection_remove_mut :: (self__pod: *physx_PxCollection_Pod, object_pod: *physx_PxBase_Pod) -> void #foreign physx_sys;

PxCollection_contains :: (self__pod: *physx_PxCollection_Pod, object_pod: *physx_PxBase_Pod) -> bool #foreign physx_sys;

PxCollection_addId_mut :: (self__pod: *physx_PxCollection_Pod, object_pod: *physx_PxBase_Pod, id: u64) -> void #foreign physx_sys;

PxCollection_removeId_mut :: (self__pod: *physx_PxCollection_Pod, id: u64) -> void #foreign physx_sys;

PxCollection_add_mut_1 :: (self__pod: *physx_PxCollection_Pod, collection_pod: *physx_PxCollection_Pod) -> void #foreign physx_sys;

PxCollection_remove_mut_1 :: (self__pod: *physx_PxCollection_Pod, collection_pod: *physx_PxCollection_Pod) -> void #foreign physx_sys;

PxCollection_getNbObjects :: (self__pod: *physx_PxCollection_Pod) -> u32 #foreign physx_sys;

PxCollection_getObject :: (self__pod: *physx_PxCollection_Pod, index: u32) -> *physx_PxBase_Pod #foreign physx_sys;

PxCollection_getObjects :: (self__pod: *physx_PxCollection_Pod, userBuffer_pod: **physx_PxBase_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxCollection_find :: (self__pod: *physx_PxCollection_Pod, id: u64) -> *physx_PxBase_Pod #foreign physx_sys;

PxCollection_getNbIds :: (self__pod: *physx_PxCollection_Pod) -> u32 #foreign physx_sys;

PxCollection_getIds :: (self__pod: *physx_PxCollection_Pod, userBuffer: *u64, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxCollection_getId :: (self__pod: *physx_PxCollection_Pod, object_pod: *physx_PxBase_Pod) -> u64 #foreign physx_sys;

PxCollection_release_mut :: (self__pod: *physx_PxCollection_Pod) -> void #foreign physx_sys;

phys_PxCreateCollection :: () -> *physx_PxCollection_Pod #foreign physx_sys;

PxBase_release_mut :: (self__pod: *physx_PxBase_Pod) -> void #foreign physx_sys;

PxBase_getConcreteTypeName :: (self__pod: *physx_PxBase_Pod) -> *u8 #foreign physx_sys;

PxBase_getConcreteType :: (self__pod: *physx_PxBase_Pod) -> u16 #foreign physx_sys;

PxBase_setBaseFlag_mut :: (self__pod: *physx_PxBase_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxBase_setBaseFlags_mut :: (self__pod: *physx_PxBase_Pod, inFlags_pod: u16) -> void #foreign physx_sys;

PxBase_getBaseFlags :: (self__pod: *physx_PxBase_Pod) -> u16 #foreign physx_sys;

PxBase_isReleasable :: (self__pod: *physx_PxBase_Pod) -> bool #foreign physx_sys;

PxRefCounted_release_mut :: (self__pod: *physx_PxRefCounted_Pod) -> void #foreign physx_sys;

PxRefCounted_getReferenceCount :: (self__pod: *physx_PxRefCounted_Pod) -> u32 #foreign physx_sys;

PxRefCounted_acquireReference_mut :: (self__pod: *physx_PxRefCounted_Pod) -> void #foreign physx_sys;

PxTolerancesScale_new :: (defaultLength: float, defaultSpeed: float) -> physx_PxTolerancesScale_Pod #foreign physx_sys;

PxTolerancesScale_isValid :: (self__pod: *physx_PxTolerancesScale_Pod) -> bool #foreign physx_sys;

PxStringTable_allocateStr_mut :: (self__pod: *physx_PxStringTable_Pod, inSrc: *u8) -> *u8 #foreign physx_sys;

PxStringTable_release_mut :: (self__pod: *physx_PxStringTable_Pod) -> void #foreign physx_sys;

PxSerializer_getConcreteTypeName :: (self__pod: *physx_PxSerializer_Pod) -> *u8 #foreign physx_sys;

PxSerializer_requiresObjects :: (self__pod: *physx_PxSerializer_Pod, anon_param0_pod: *physx_PxBase_Pod, anon_param1_pod: *physx_PxProcessPxBaseCallback_Pod) -> void #foreign physx_sys;

PxSerializer_isSubordinate :: (self__pod: *physx_PxSerializer_Pod) -> bool #foreign physx_sys;

PxSerializer_exportExtraData :: (self__pod: *physx_PxSerializer_Pod, anon_param0_pod: *physx_PxBase_Pod, anon_param1_pod: *physx_PxSerializationContext_Pod) -> void #foreign physx_sys;

PxSerializer_exportData :: (self__pod: *physx_PxSerializer_Pod, anon_param0_pod: *physx_PxBase_Pod, anon_param1_pod: *physx_PxSerializationContext_Pod) -> void #foreign physx_sys;

PxSerializer_registerReferences :: (self__pod: *physx_PxSerializer_Pod, obj_pod: *physx_PxBase_Pod, s_pod: *physx_PxSerializationContext_Pod) -> void #foreign physx_sys;

PxSerializer_getClassSize :: (self__pod: *physx_PxSerializer_Pod) -> u64 #foreign physx_sys;

PxSerializer_createObject :: (self__pod: *physx_PxSerializer_Pod, address_pod: **u8, context_pod: *physx_PxDeserializationContext_Pod) -> *physx_PxBase_Pod #foreign physx_sys;

PxSerializer_delete :: (self__pod: *physx_PxSerializer_Pod) -> void #foreign physx_sys;

PxInsertionCallback_buildObjectFromData_mut :: (self__pod: *physx_PxInsertionCallback_Pod, type_pod: s32, data: *void) -> *physx_PxBase_Pod #foreign physx_sys;

PxTaskManager_setCpuDispatcher_mut :: (self__pod: *physx_PxTaskManager_Pod, ref_pod: *physx_PxCpuDispatcher_Pod) -> void #foreign physx_sys;

PxTaskManager_getCpuDispatcher :: (self__pod: *physx_PxTaskManager_Pod) -> *physx_PxCpuDispatcher_Pod #foreign physx_sys;

PxTaskManager_resetDependencies_mut :: (self__pod: *physx_PxTaskManager_Pod) -> void #foreign physx_sys;

PxTaskManager_startSimulation_mut :: (self__pod: *physx_PxTaskManager_Pod) -> void #foreign physx_sys;

PxTaskManager_stopSimulation_mut :: (self__pod: *physx_PxTaskManager_Pod) -> void #foreign physx_sys;

PxTaskManager_taskCompleted_mut :: (self__pod: *physx_PxTaskManager_Pod, task_pod: *physx_PxTask_Pod) -> void #foreign physx_sys;

PxTaskManager_getNamedTask_mut :: (self__pod: *physx_PxTaskManager_Pod, name: *u8) -> u32 #foreign physx_sys;

PxTaskManager_submitNamedTask_mut :: (self__pod: *physx_PxTaskManager_Pod, task_pod: *physx_PxTask_Pod, name: *u8, type_pod: s32) -> u32 #foreign physx_sys;

PxTaskManager_submitUnnamedTask_mut :: (self__pod: *physx_PxTaskManager_Pod, task_pod: *physx_PxTask_Pod, type_pod: s32) -> u32 #foreign physx_sys;

PxTaskManager_getTaskFromID_mut :: (self__pod: *physx_PxTaskManager_Pod, id: u32) -> *physx_PxTask_Pod #foreign physx_sys;

PxTaskManager_release_mut :: (self__pod: *physx_PxTaskManager_Pod) -> void #foreign physx_sys;

PxTaskManager_createTaskManager :: (errorCallback_pod: *physx_PxErrorCallback_Pod, anon_param1_pod: *physx_PxCpuDispatcher_Pod) -> *physx_PxTaskManager_Pod #foreign physx_sys;

PxCpuDispatcher_submitTask_mut :: (self__pod: *physx_PxCpuDispatcher_Pod, task_pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxCpuDispatcher_getWorkerCount :: (self__pod: *physx_PxCpuDispatcher_Pod) -> u32 #foreign physx_sys;

PxCpuDispatcher_delete :: (self__pod: *physx_PxCpuDispatcher_Pod) -> void #foreign physx_sys;

PxBaseTask_run_mut :: (self__pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxBaseTask_getName :: (self__pod: *physx_PxBaseTask_Pod) -> *u8 #foreign physx_sys;

PxBaseTask_addReference_mut :: (self__pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxBaseTask_removeReference_mut :: (self__pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxBaseTask_getReference :: (self__pod: *physx_PxBaseTask_Pod) -> s32 #foreign physx_sys;

PxBaseTask_release_mut :: (self__pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxBaseTask_getTaskManager :: (self__pod: *physx_PxBaseTask_Pod) -> *physx_PxTaskManager_Pod #foreign physx_sys;

PxBaseTask_setContextId_mut :: (self__pod: *physx_PxBaseTask_Pod, id: u64) -> void #foreign physx_sys;

PxBaseTask_getContextId :: (self__pod: *physx_PxBaseTask_Pod) -> u64 #foreign physx_sys;

PxTask_release_mut :: (self__pod: *physx_PxTask_Pod) -> void #foreign physx_sys;

PxTask_finishBefore_mut :: (self__pod: *physx_PxTask_Pod, taskID: u32) -> void #foreign physx_sys;

PxTask_startAfter_mut :: (self__pod: *physx_PxTask_Pod, taskID: u32) -> void #foreign physx_sys;

PxTask_addReference_mut :: (self__pod: *physx_PxTask_Pod) -> void #foreign physx_sys;

PxTask_removeReference_mut :: (self__pod: *physx_PxTask_Pod) -> void #foreign physx_sys;

PxTask_getReference :: (self__pod: *physx_PxTask_Pod) -> s32 #foreign physx_sys;

PxTask_getTaskID :: (self__pod: *physx_PxTask_Pod) -> u32 #foreign physx_sys;

PxTask_submitted_mut :: (self__pod: *physx_PxTask_Pod) -> void #foreign physx_sys;

PxLightCpuTask_setContinuation_mut :: (self__pod: *physx_PxLightCpuTask_Pod, tm_pod: *physx_PxTaskManager_Pod, c_pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxLightCpuTask_setContinuation_mut_1 :: (self__pod: *physx_PxLightCpuTask_Pod, c_pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxLightCpuTask_getContinuation :: (self__pod: *physx_PxLightCpuTask_Pod) -> *physx_PxBaseTask_Pod #foreign physx_sys;

PxLightCpuTask_removeReference_mut :: (self__pod: *physx_PxLightCpuTask_Pod) -> void #foreign physx_sys;

PxLightCpuTask_getReference :: (self__pod: *physx_PxLightCpuTask_Pod) -> s32 #foreign physx_sys;

PxLightCpuTask_addReference_mut :: (self__pod: *physx_PxLightCpuTask_Pod) -> void #foreign physx_sys;

PxLightCpuTask_release_mut :: (self__pod: *physx_PxLightCpuTask_Pod) -> void #foreign physx_sys;

PxGeometry_getType :: (self__pod: *physx_PxGeometry_Pod) -> s32 #foreign physx_sys;

PxBoxGeometry_new :: (hx: float, hy: float, hz: float) -> physx_PxBoxGeometry_Pod #foreign physx_sys;

PxBoxGeometry_new_1 :: (halfExtents__pod: physx_PxVec3_Pod) -> physx_PxBoxGeometry_Pod #foreign physx_sys;

PxBoxGeometry_isValid :: (self__pod: *physx_PxBoxGeometry_Pod) -> bool #foreign physx_sys;

PxBVHRaycastCallback_delete :: (self__pod: *physx_PxBVHRaycastCallback_Pod) -> void #foreign physx_sys;

PxBVHRaycastCallback_reportHit_mut :: (self__pod: *physx_PxBVHRaycastCallback_Pod, boundsIndex: u32, distance_pod: *float) -> bool #foreign physx_sys;

PxBVHOverlapCallback_delete :: (self__pod: *physx_PxBVHOverlapCallback_Pod) -> void #foreign physx_sys;

PxBVHOverlapCallback_reportHit_mut :: (self__pod: *physx_PxBVHOverlapCallback_Pod, boundsIndex: u32) -> bool #foreign physx_sys;

PxBVHTraversalCallback_delete :: (self__pod: *physx_PxBVHTraversalCallback_Pod) -> void #foreign physx_sys;

PxBVHTraversalCallback_visitNode_mut :: (self__pod: *physx_PxBVHTraversalCallback_Pod, bounds_pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBVHTraversalCallback_reportLeaf_mut :: (self__pod: *physx_PxBVHTraversalCallback_Pod, nbPrims: u32, prims: *u32) -> bool #foreign physx_sys;

PxBVH_raycast :: (self__pod: *physx_PxBVH_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, maxDist: float, cb_pod: *physx_PxBVHRaycastCallback_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxBVH_sweep :: (self__pod: *physx_PxBVH_Pod, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, unitDir_pod: *physx_PxVec3_Pod, maxDist: float, cb_pod: *physx_PxBVHRaycastCallback_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxBVH_overlap :: (self__pod: *physx_PxBVH_Pod, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, cb_pod: *physx_PxBVHOverlapCallback_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxBVH_cull :: (self__pod: *physx_PxBVH_Pod, nbPlanes: u32, planes_pod: *physx_PxPlane_Pod, cb_pod: *physx_PxBVHOverlapCallback_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxBVH_getNbBounds :: (self__pod: *physx_PxBVH_Pod) -> u32 #foreign physx_sys;

PxBVH_getBounds :: (self__pod: *physx_PxBVH_Pod) -> *physx_PxBounds3_Pod #foreign physx_sys;

PxBVH_getBoundsForModification_mut :: (self__pod: *physx_PxBVH_Pod) -> *physx_PxBounds3_Pod #foreign physx_sys;

PxBVH_refit_mut :: (self__pod: *physx_PxBVH_Pod) -> void #foreign physx_sys;

PxBVH_updateBounds_mut :: (self__pod: *physx_PxBVH_Pod, boundsIndex: u32, newBounds_pod: *physx_PxBounds3_Pod) -> bool #foreign physx_sys;

PxBVH_partialRefit_mut :: (self__pod: *physx_PxBVH_Pod) -> void #foreign physx_sys;

PxBVH_traverse :: (self__pod: *physx_PxBVH_Pod, cb_pod: *physx_PxBVHTraversalCallback_Pod) -> bool #foreign physx_sys;

PxBVH_getConcreteTypeName :: (self__pod: *physx_PxBVH_Pod) -> *u8 #foreign physx_sys;

PxCapsuleGeometry_new :: (radius_: float, halfHeight_: float) -> physx_PxCapsuleGeometry_Pod #foreign physx_sys;

PxCapsuleGeometry_isValid :: (self__pod: *physx_PxCapsuleGeometry_Pod) -> bool #foreign physx_sys;

PxConvexMesh_getNbVertices :: (self__pod: *physx_PxConvexMesh_Pod) -> u32 #foreign physx_sys;

PxConvexMesh_getVertices :: (self__pod: *physx_PxConvexMesh_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxConvexMesh_getIndexBuffer :: (self__pod: *physx_PxConvexMesh_Pod) -> *u8 #foreign physx_sys;

PxConvexMesh_getNbPolygons :: (self__pod: *physx_PxConvexMesh_Pod) -> u32 #foreign physx_sys;

PxConvexMesh_getPolygonData :: (self__pod: *physx_PxConvexMesh_Pod, index: u32, data_pod: *physx_PxHullPolygon_Pod) -> bool #foreign physx_sys;

PxConvexMesh_release_mut :: (self__pod: *physx_PxConvexMesh_Pod) -> void #foreign physx_sys;

PxConvexMesh_getMassInformation :: (self__pod: *physx_PxConvexMesh_Pod, mass_pod: *float, localInertia_pod: *physx_PxMat33_Pod, localCenterOfMass_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxConvexMesh_getLocalBounds :: (self__pod: *physx_PxConvexMesh_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxConvexMesh_getSDF :: (self__pod: *physx_PxConvexMesh_Pod) -> *float #foreign physx_sys;

PxConvexMesh_getConcreteTypeName :: (self__pod: *physx_PxConvexMesh_Pod) -> *u8 #foreign physx_sys;

PxConvexMesh_isGpuCompatible :: (self__pod: *physx_PxConvexMesh_Pod) -> bool #foreign physx_sys;

PxMeshScale_new :: () -> physx_PxMeshScale_Pod #foreign physx_sys;

PxMeshScale_new_1 :: (r: float) -> physx_PxMeshScale_Pod #foreign physx_sys;

PxMeshScale_new_2 :: (s_pod: *physx_PxVec3_Pod) -> physx_PxMeshScale_Pod #foreign physx_sys;

PxMeshScale_new_3 :: (s_pod: *physx_PxVec3_Pod, r_pod: *physx_PxQuat_Pod) -> physx_PxMeshScale_Pod #foreign physx_sys;

PxMeshScale_isIdentity :: (self__pod: *physx_PxMeshScale_Pod) -> bool #foreign physx_sys;

PxMeshScale_getInverse :: (self__pod: *physx_PxMeshScale_Pod) -> physx_PxMeshScale_Pod #foreign physx_sys;

PxMeshScale_toMat33 :: (self__pod: *physx_PxMeshScale_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMeshScale_hasNegativeDeterminant :: (self__pod: *physx_PxMeshScale_Pod) -> bool #foreign physx_sys;

PxMeshScale_transform :: (self__pod: *physx_PxMeshScale_Pod, v_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMeshScale_isValidForTriangleMesh :: (self__pod: *physx_PxMeshScale_Pod) -> bool #foreign physx_sys;

PxMeshScale_isValidForConvexMesh :: (self__pod: *physx_PxMeshScale_Pod) -> bool #foreign physx_sys;

PxConvexMeshGeometry_new :: (mesh_pod: *physx_PxConvexMesh_Pod, scaling_pod: *physx_PxMeshScale_Pod, flags_pod: u8) -> physx_PxConvexMeshGeometry_Pod #foreign physx_sys;

PxConvexMeshGeometry_isValid :: (self__pod: *physx_PxConvexMeshGeometry_Pod) -> bool #foreign physx_sys;

PxSphereGeometry_new :: (ir: float) -> physx_PxSphereGeometry_Pod #foreign physx_sys;

PxSphereGeometry_isValid :: (self__pod: *physx_PxSphereGeometry_Pod) -> bool #foreign physx_sys;

PxPlaneGeometry_new :: () -> physx_PxPlaneGeometry_Pod #foreign physx_sys;

PxPlaneGeometry_isValid :: (self__pod: *physx_PxPlaneGeometry_Pod) -> bool #foreign physx_sys;

PxTriangleMeshGeometry_new :: (mesh_pod: *physx_PxTriangleMesh_Pod, scaling_pod: *physx_PxMeshScale_Pod, flags_pod: u8) -> physx_PxTriangleMeshGeometry_Pod #foreign physx_sys;

PxTriangleMeshGeometry_isValid :: (self__pod: *physx_PxTriangleMeshGeometry_Pod) -> bool #foreign physx_sys;

PxHeightFieldGeometry_new :: (hf_pod: *physx_PxHeightField_Pod, flags_pod: u8, heightScale_: float, rowScale_: float, columnScale_: float) -> physx_PxHeightFieldGeometry_Pod #foreign physx_sys;

PxHeightFieldGeometry_isValid :: (self__pod: *physx_PxHeightFieldGeometry_Pod) -> bool #foreign physx_sys;

PxParticleSystemGeometry_new :: () -> physx_PxParticleSystemGeometry_Pod #foreign physx_sys;

PxParticleSystemGeometry_isValid :: (self__pod: *physx_PxParticleSystemGeometry_Pod) -> bool #foreign physx_sys;

PxHairSystemGeometry_new :: () -> physx_PxHairSystemGeometry_Pod #foreign physx_sys;

PxHairSystemGeometry_isValid :: (self__pod: *physx_PxHairSystemGeometry_Pod) -> bool #foreign physx_sys;

PxTetrahedronMeshGeometry_new :: (mesh_pod: *physx_PxTetrahedronMesh_Pod) -> physx_PxTetrahedronMeshGeometry_Pod #foreign physx_sys;

PxTetrahedronMeshGeometry_isValid :: (self__pod: *physx_PxTetrahedronMeshGeometry_Pod) -> bool #foreign physx_sys;

PxQueryHit_new :: () -> physx_PxQueryHit_Pod #foreign physx_sys;

PxLocationHit_new :: () -> physx_PxLocationHit_Pod #foreign physx_sys;

PxLocationHit_hadInitialOverlap :: (self__pod: *physx_PxLocationHit_Pod) -> bool #foreign physx_sys;

PxGeomRaycastHit_new :: () -> physx_PxGeomRaycastHit_Pod #foreign physx_sys;

PxGeomOverlapHit_new :: () -> physx_PxGeomOverlapHit_Pod #foreign physx_sys;

PxGeomSweepHit_new :: () -> physx_PxGeomSweepHit_Pod #foreign physx_sys;

PxGeomIndexPair_new :: () -> physx_PxGeomIndexPair_Pod #foreign physx_sys;

PxGeomIndexPair_new_1 :: (_id0: u32, _id1: u32) -> physx_PxGeomIndexPair_Pod #foreign physx_sys;

phys_PxCustomGeometry_getUniqueID :: () -> u32 #foreign physx_sys;

PxCustomGeometryType_new :: () -> physx_PxCustomGeometryType_Pod #foreign physx_sys;

PxCustomGeometryType_INVALID :: () -> physx_PxCustomGeometryType_Pod #foreign physx_sys;

PxCustomGeometryCallbacks_getCustomType :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod) -> physx_PxCustomGeometryType_Pod #foreign physx_sys;

PxCustomGeometryCallbacks_getLocalBounds :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod, geometry_pod: *physx_PxGeometry_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxCustomGeometryCallbacks_raycast :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, maxDist: float, hitFlags_pod: u16, maxHits: u32, rayHits_pod: *physx_PxGeomRaycastHit_Pod, stride: u32, threadContext_pod: *physx_PxQueryThreadContext_Pod) -> u32 #foreign physx_sys;

PxCustomGeometryCallbacks_overlap :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod, geom0_pod: *physx_PxGeometry_Pod, pose0_pod: *physx_PxTransform_Pod, geom1_pod: *physx_PxGeometry_Pod, pose1_pod: *physx_PxTransform_Pod, threadContext_pod: *physx_PxQueryThreadContext_Pod) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_sweep :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod, unitDir_pod: *physx_PxVec3_Pod, maxDist: float, geom0_pod: *physx_PxGeometry_Pod, pose0_pod: *physx_PxTransform_Pod, geom1_pod: *physx_PxGeometry_Pod, pose1_pod: *physx_PxTransform_Pod, sweepHit_pod: *physx_PxGeomSweepHit_Pod, hitFlags_pod: u16, inflation: float, threadContext_pod: *physx_PxQueryThreadContext_Pod) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_computeMassProperties :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod, geometry_pod: *physx_PxGeometry_Pod, massProperties_pod: *physx_PxMassProperties_Pod) -> void #foreign physx_sys;

PxCustomGeometryCallbacks_usePersistentContactManifold :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod, geometry_pod: *physx_PxGeometry_Pod, breakingThreshold_pod: *float) -> bool #foreign physx_sys;

PxCustomGeometryCallbacks_delete :: (self__pod: *physx_PxCustomGeometryCallbacks_Pod) -> void #foreign physx_sys;

PxCustomGeometry_new :: () -> physx_PxCustomGeometry_Pod #foreign physx_sys;

PxCustomGeometry_new_1 :: (_callbacks_pod: *physx_PxCustomGeometryCallbacks_Pod) -> physx_PxCustomGeometry_Pod #foreign physx_sys;

PxCustomGeometry_isValid :: (self__pod: *physx_PxCustomGeometry_Pod) -> bool #foreign physx_sys;

PxCustomGeometry_getCustomType :: (self__pod: *physx_PxCustomGeometry_Pod) -> physx_PxCustomGeometryType_Pod #foreign physx_sys;

PxGeometryHolder_getType :: (self__pod: *physx_PxGeometryHolder_Pod) -> s32 #foreign physx_sys;

PxGeometryHolder_any_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxGeometry_Pod #foreign physx_sys;

PxGeometryHolder_any :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxGeometry_Pod #foreign physx_sys;

PxGeometryHolder_sphere_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxSphereGeometry_Pod #foreign physx_sys;

PxGeometryHolder_sphere :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxSphereGeometry_Pod #foreign physx_sys;

PxGeometryHolder_plane_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxPlaneGeometry_Pod #foreign physx_sys;

PxGeometryHolder_plane :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxPlaneGeometry_Pod #foreign physx_sys;

PxGeometryHolder_capsule_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxCapsuleGeometry_Pod #foreign physx_sys;

PxGeometryHolder_capsule :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxCapsuleGeometry_Pod #foreign physx_sys;

PxGeometryHolder_box_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxBoxGeometry_Pod #foreign physx_sys;

PxGeometryHolder_box :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxBoxGeometry_Pod #foreign physx_sys;

PxGeometryHolder_convexMesh_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxConvexMeshGeometry_Pod #foreign physx_sys;

PxGeometryHolder_convexMesh :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxConvexMeshGeometry_Pod #foreign physx_sys;

PxGeometryHolder_tetMesh_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxTetrahedronMeshGeometry_Pod #foreign physx_sys;

PxGeometryHolder_tetMesh :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxTetrahedronMeshGeometry_Pod #foreign physx_sys;

PxGeometryHolder_triangleMesh_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxTriangleMeshGeometry_Pod #foreign physx_sys;

PxGeometryHolder_triangleMesh :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxTriangleMeshGeometry_Pod #foreign physx_sys;

PxGeometryHolder_heightField_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxHeightFieldGeometry_Pod #foreign physx_sys;

PxGeometryHolder_heightField :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxHeightFieldGeometry_Pod #foreign physx_sys;

PxGeometryHolder_particleSystem_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxParticleSystemGeometry_Pod #foreign physx_sys;

PxGeometryHolder_particleSystem :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxParticleSystemGeometry_Pod #foreign physx_sys;

PxGeometryHolder_hairSystem_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxHairSystemGeometry_Pod #foreign physx_sys;

PxGeometryHolder_hairSystem :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxHairSystemGeometry_Pod #foreign physx_sys;

PxGeometryHolder_custom_mut :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxCustomGeometry_Pod #foreign physx_sys;

PxGeometryHolder_custom :: (self__pod: *physx_PxGeometryHolder_Pod) -> *physx_PxCustomGeometry_Pod #foreign physx_sys;

PxGeometryHolder_storeAny_mut :: (self__pod: *physx_PxGeometryHolder_Pod, geometry_pod: *physx_PxGeometry_Pod) -> void #foreign physx_sys;

PxGeometryHolder_new :: () -> physx_PxGeometryHolder_Pod #foreign physx_sys;

PxGeometryHolder_new_1 :: (geometry_pod: *physx_PxGeometry_Pod) -> physx_PxGeometryHolder_Pod #foreign physx_sys;

PxGeometryQuery_raycast :: (origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, maxDist: float, hitFlags_pod: u16, maxHits: u32, rayHits_pod: *physx_PxGeomRaycastHit_Pod, stride: u32, queryFlags_pod: u32, threadContext_pod: *physx_PxQueryThreadContext_Pod) -> u32 #foreign physx_sys;

PxGeometryQuery_overlap :: (geom0_pod: *physx_PxGeometry_Pod, pose0_pod: *physx_PxTransform_Pod, geom1_pod: *physx_PxGeometry_Pod, pose1_pod: *physx_PxTransform_Pod, queryFlags_pod: u32, threadContext_pod: *physx_PxQueryThreadContext_Pod) -> bool #foreign physx_sys;

PxGeometryQuery_sweep :: (unitDir_pod: *physx_PxVec3_Pod, maxDist: float, geom0_pod: *physx_PxGeometry_Pod, pose0_pod: *physx_PxTransform_Pod, geom1_pod: *physx_PxGeometry_Pod, pose1_pod: *physx_PxTransform_Pod, sweepHit_pod: *physx_PxGeomSweepHit_Pod, hitFlags_pod: u16, inflation: float, queryFlags_pod: u32, threadContext_pod: *physx_PxQueryThreadContext_Pod) -> bool #foreign physx_sys;

PxGeometryQuery_computePenetration :: (direction_pod: *physx_PxVec3_Pod, depth_pod: *float, geom0_pod: *physx_PxGeometry_Pod, pose0_pod: *physx_PxTransform_Pod, geom1_pod: *physx_PxGeometry_Pod, pose1_pod: *physx_PxTransform_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxGeometryQuery_pointDistance :: (point_pod: *physx_PxVec3_Pod, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, closestPoint_pod: *physx_PxVec3_Pod, closestIndex: *u32, queryFlags_pod: u32) -> float #foreign physx_sys;

PxGeometryQuery_computeGeomBounds :: (bounds_pod: *physx_PxBounds3_Pod, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, offset: float, inflation: float, queryFlags_pod: u32) -> void #foreign physx_sys;

PxGeometryQuery_isValid :: (geom_pod: *physx_PxGeometry_Pod) -> bool #foreign physx_sys;

PxHeightFieldSample_tessFlag :: (self__pod: *physx_PxHeightFieldSample_Pod) -> u8 #foreign physx_sys;

PxHeightFieldSample_setTessFlag_mut :: (self__pod: *physx_PxHeightFieldSample_Pod) -> void #foreign physx_sys;

PxHeightFieldSample_clearTessFlag_mut :: (self__pod: *physx_PxHeightFieldSample_Pod) -> void #foreign physx_sys;

PxHeightField_release_mut :: (self__pod: *physx_PxHeightField_Pod) -> void #foreign physx_sys;

PxHeightField_saveCells :: (self__pod: *physx_PxHeightField_Pod, destBuffer: *void, destBufferSize: u32) -> u32 #foreign physx_sys;

PxHeightField_modifySamples_mut :: (self__pod: *physx_PxHeightField_Pod, startCol: s32, startRow: s32, subfieldDesc_pod: *physx_PxHeightFieldDesc_Pod, shrinkBounds: bool) -> bool #foreign physx_sys;

PxHeightField_getNbRows :: (self__pod: *physx_PxHeightField_Pod) -> u32 #foreign physx_sys;

PxHeightField_getNbColumns :: (self__pod: *physx_PxHeightField_Pod) -> u32 #foreign physx_sys;

PxHeightField_getFormat :: (self__pod: *physx_PxHeightField_Pod) -> s32 #foreign physx_sys;

PxHeightField_getSampleStride :: (self__pod: *physx_PxHeightField_Pod) -> u32 #foreign physx_sys;

PxHeightField_getConvexEdgeThreshold :: (self__pod: *physx_PxHeightField_Pod) -> float #foreign physx_sys;

PxHeightField_getFlags :: (self__pod: *physx_PxHeightField_Pod) -> u16 #foreign physx_sys;

PxHeightField_getHeight :: (self__pod: *physx_PxHeightField_Pod, x: float, z: float) -> float #foreign physx_sys;

PxHeightField_getTriangleMaterialIndex :: (self__pod: *physx_PxHeightField_Pod, triangleIndex: u32) -> u16 #foreign physx_sys;

PxHeightField_getTriangleNormal :: (self__pod: *physx_PxHeightField_Pod, triangleIndex: u32) -> physx_PxVec3_Pod #foreign physx_sys;

PxHeightField_getSample :: (self__pod: *physx_PxHeightField_Pod, row: u32, column: u32) -> *physx_PxHeightFieldSample_Pod #foreign physx_sys;

PxHeightField_getTimestamp :: (self__pod: *physx_PxHeightField_Pod) -> u32 #foreign physx_sys;

PxHeightField_getConcreteTypeName :: (self__pod: *physx_PxHeightField_Pod) -> *u8 #foreign physx_sys;

PxHeightFieldDesc_new :: () -> physx_PxHeightFieldDesc_Pod #foreign physx_sys;

PxHeightFieldDesc_setToDefault_mut :: (self__pod: *physx_PxHeightFieldDesc_Pod) -> void #foreign physx_sys;

PxHeightFieldDesc_isValid :: (self__pod: *physx_PxHeightFieldDesc_Pod) -> bool #foreign physx_sys;

PxMeshQuery_getTriangle :: (triGeom_pod: *physx_PxTriangleMeshGeometry_Pod, transform_pod: *physx_PxTransform_Pod, triangleIndex: u32, triangle_pod: *physx_PxTriangle_Pod, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign physx_sys;

PxMeshQuery_getTriangle_1 :: (hfGeom_pod: *physx_PxHeightFieldGeometry_Pod, transform_pod: *physx_PxTransform_Pod, triangleIndex: u32, triangle_pod: *physx_PxTriangle_Pod, vertexIndices: *u32, adjacencyIndices: *u32) -> void #foreign physx_sys;

PxMeshQuery_findOverlapTriangleMesh :: (geom_pod: *physx_PxGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, meshGeom_pod: *physx_PxTriangleMeshGeometry_Pod, meshPose_pod: *physx_PxTransform_Pod, results: *u32, maxResults: u32, startIndex: u32, overflow_pod: *bool, queryFlags_pod: u32) -> u32 #foreign physx_sys;

PxMeshQuery_findOverlapHeightField :: (geom_pod: *physx_PxGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, hfGeom_pod: *physx_PxHeightFieldGeometry_Pod, hfPose_pod: *physx_PxTransform_Pod, results: *u32, maxResults: u32, startIndex: u32, overflow_pod: *bool, queryFlags_pod: u32) -> u32 #foreign physx_sys;

PxMeshQuery_sweep :: (unitDir_pod: *physx_PxVec3_Pod, distance: float, geom_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, triangleCount: u32, triangles_pod: *physx_PxTriangle_Pod, sweepHit_pod: *physx_PxGeomSweepHit_Pod, hitFlags_pod: u16, cachedIndex: *u32, inflation: float, doubleSided: bool, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxSimpleTriangleMesh_new :: () -> physx_PxSimpleTriangleMesh_Pod #foreign physx_sys;

PxSimpleTriangleMesh_setToDefault_mut :: (self__pod: *physx_PxSimpleTriangleMesh_Pod) -> void #foreign physx_sys;

PxSimpleTriangleMesh_isValid :: (self__pod: *physx_PxSimpleTriangleMesh_Pod) -> bool #foreign physx_sys;

PxTriangle_new_alloc :: () -> *physx_PxTriangle_Pod #foreign physx_sys;

PxTriangle_new_alloc_1 :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, p2_pod: *physx_PxVec3_Pod) -> *physx_PxTriangle_Pod #foreign physx_sys;

PxTriangle_delete :: (self__pod: *physx_PxTriangle_Pod) -> void #foreign physx_sys;

PxTriangle_normal :: (self__pod: *physx_PxTriangle_Pod, _normal_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxTriangle_denormalizedNormal :: (self__pod: *physx_PxTriangle_Pod, _normal_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxTriangle_area :: (self__pod: *physx_PxTriangle_Pod) -> float #foreign physx_sys;

PxTriangle_pointFromUV :: (self__pod: *physx_PxTriangle_Pod, u: float, v: float) -> physx_PxVec3_Pod #foreign physx_sys;

PxTrianglePadded_new_alloc :: () -> *physx_PxTrianglePadded_Pod #foreign physx_sys;

PxTrianglePadded_delete :: (self__pod: *physx_PxTrianglePadded_Pod) -> void #foreign physx_sys;

PxTriangleMesh_getNbVertices :: (self__pod: *physx_PxTriangleMesh_Pod) -> u32 #foreign physx_sys;

PxTriangleMesh_getVertices :: (self__pod: *physx_PxTriangleMesh_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxTriangleMesh_getVerticesForModification_mut :: (self__pod: *physx_PxTriangleMesh_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxTriangleMesh_refitBVH_mut :: (self__pod: *physx_PxTriangleMesh_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxTriangleMesh_getNbTriangles :: (self__pod: *physx_PxTriangleMesh_Pod) -> u32 #foreign physx_sys;

PxTriangleMesh_getTriangles :: (self__pod: *physx_PxTriangleMesh_Pod) -> *void #foreign physx_sys;

PxTriangleMesh_getTriangleMeshFlags :: (self__pod: *physx_PxTriangleMesh_Pod) -> u8 #foreign physx_sys;

PxTriangleMesh_getTrianglesRemap :: (self__pod: *physx_PxTriangleMesh_Pod) -> *u32 #foreign physx_sys;

PxTriangleMesh_release_mut :: (self__pod: *physx_PxTriangleMesh_Pod) -> void #foreign physx_sys;

PxTriangleMesh_getTriangleMaterialIndex :: (self__pod: *physx_PxTriangleMesh_Pod, triangleIndex: u32) -> u16 #foreign physx_sys;

PxTriangleMesh_getLocalBounds :: (self__pod: *physx_PxTriangleMesh_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxTriangleMesh_getSDF :: (self__pod: *physx_PxTriangleMesh_Pod) -> *float #foreign physx_sys;

PxTriangleMesh_getSDFDimensions :: (self__pod: *physx_PxTriangleMesh_Pod, numX_pod: *u32, numY_pod: *u32, numZ_pod: *u32) -> void #foreign physx_sys;

PxTriangleMesh_setPreferSDFProjection_mut :: (self__pod: *physx_PxTriangleMesh_Pod, preferProjection: bool) -> void #foreign physx_sys;

PxTriangleMesh_getPreferSDFProjection :: (self__pod: *physx_PxTriangleMesh_Pod) -> bool #foreign physx_sys;

PxTriangleMesh_getMassInformation :: (self__pod: *physx_PxTriangleMesh_Pod, mass_pod: *float, localInertia_pod: *physx_PxMat33_Pod, localCenterOfMass_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxTetrahedron_new_alloc :: () -> *physx_PxTetrahedron_Pod #foreign physx_sys;

PxTetrahedron_new_alloc_1 :: (p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, p2_pod: *physx_PxVec3_Pod, p3_pod: *physx_PxVec3_Pod) -> *physx_PxTetrahedron_Pod #foreign physx_sys;

PxTetrahedron_delete :: (self__pod: *physx_PxTetrahedron_Pod) -> void #foreign physx_sys;

PxSoftBodyAuxData_release_mut :: (self__pod: *physx_PxSoftBodyAuxData_Pod) -> void #foreign physx_sys;

PxTetrahedronMesh_getNbVertices :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> u32 #foreign physx_sys;

PxTetrahedronMesh_getVertices :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxTetrahedronMesh_getNbTetrahedrons :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> u32 #foreign physx_sys;

PxTetrahedronMesh_getTetrahedrons :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> *void #foreign physx_sys;

PxTetrahedronMesh_getTetrahedronMeshFlags :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> u8 #foreign physx_sys;

PxTetrahedronMesh_getTetrahedraRemap :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> *u32 #foreign physx_sys;

PxTetrahedronMesh_getLocalBounds :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxTetrahedronMesh_release_mut :: (self__pod: *physx_PxTetrahedronMesh_Pod) -> void #foreign physx_sys;

PxSoftBodyMesh_getCollisionMesh :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> *physx_PxTetrahedronMesh_Pod #foreign physx_sys;

PxSoftBodyMesh_getCollisionMesh_mut :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> *physx_PxTetrahedronMesh_Pod #foreign physx_sys;

PxSoftBodyMesh_getSimulationMesh :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> *physx_PxTetrahedronMesh_Pod #foreign physx_sys;

PxSoftBodyMesh_getSimulationMesh_mut :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> *physx_PxTetrahedronMesh_Pod #foreign physx_sys;

PxSoftBodyMesh_getSoftBodyAuxData :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> *physx_PxSoftBodyAuxData_Pod #foreign physx_sys;

PxSoftBodyMesh_getSoftBodyAuxData_mut :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> *physx_PxSoftBodyAuxData_Pod #foreign physx_sys;

PxSoftBodyMesh_release_mut :: (self__pod: *physx_PxSoftBodyMesh_Pod) -> void #foreign physx_sys;

PxCollisionMeshMappingData_release_mut :: (self__pod: *physx_PxCollisionMeshMappingData_Pod) -> void #foreign physx_sys;

PxCollisionTetrahedronMeshData_getMesh :: (self__pod: *physx_PxCollisionTetrahedronMeshData_Pod) -> *physx_PxTetrahedronMeshData_Pod #foreign physx_sys;

PxCollisionTetrahedronMeshData_getMesh_mut :: (self__pod: *physx_PxCollisionTetrahedronMeshData_Pod) -> *physx_PxTetrahedronMeshData_Pod #foreign physx_sys;

PxCollisionTetrahedronMeshData_getData :: (self__pod: *physx_PxCollisionTetrahedronMeshData_Pod) -> *physx_PxSoftBodyCollisionData_Pod #foreign physx_sys;

PxCollisionTetrahedronMeshData_getData_mut :: (self__pod: *physx_PxCollisionTetrahedronMeshData_Pod) -> *physx_PxSoftBodyCollisionData_Pod #foreign physx_sys;

PxCollisionTetrahedronMeshData_release_mut :: (self__pod: *physx_PxCollisionTetrahedronMeshData_Pod) -> void #foreign physx_sys;

PxSimulationTetrahedronMeshData_getMesh_mut :: (self__pod: *physx_PxSimulationTetrahedronMeshData_Pod) -> *physx_PxTetrahedronMeshData_Pod #foreign physx_sys;

PxSimulationTetrahedronMeshData_getData_mut :: (self__pod: *physx_PxSimulationTetrahedronMeshData_Pod) -> *physx_PxSoftBodySimulationData_Pod #foreign physx_sys;

PxSimulationTetrahedronMeshData_release_mut :: (self__pod: *physx_PxSimulationTetrahedronMeshData_Pod) -> void #foreign physx_sys;

PxActor_release_mut :: (self__pod: *physx_PxActor_Pod) -> void #foreign physx_sys;

PxActor_getType :: (self__pod: *physx_PxActor_Pod) -> s32 #foreign physx_sys;

PxActor_getScene :: (self__pod: *physx_PxActor_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxActor_setName_mut :: (self__pod: *physx_PxActor_Pod, name: *u8) -> void #foreign physx_sys;

PxActor_getName :: (self__pod: *physx_PxActor_Pod) -> *u8 #foreign physx_sys;

PxActor_getWorldBounds :: (self__pod: *physx_PxActor_Pod, inflation: float) -> physx_PxBounds3_Pod #foreign physx_sys;

PxActor_setActorFlag_mut :: (self__pod: *physx_PxActor_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxActor_setActorFlags_mut :: (self__pod: *physx_PxActor_Pod, inFlags_pod: u8) -> void #foreign physx_sys;

PxActor_getActorFlags :: (self__pod: *physx_PxActor_Pod) -> u8 #foreign physx_sys;

PxActor_setDominanceGroup_mut :: (self__pod: *physx_PxActor_Pod, dominanceGroup: u8) -> void #foreign physx_sys;

PxActor_getDominanceGroup :: (self__pod: *physx_PxActor_Pod) -> u8 #foreign physx_sys;

PxActor_setOwnerClient_mut :: (self__pod: *physx_PxActor_Pod, inClient: u8) -> void #foreign physx_sys;

PxActor_getOwnerClient :: (self__pod: *physx_PxActor_Pod) -> u8 #foreign physx_sys;

PxActor_getAggregate :: (self__pod: *physx_PxActor_Pod) -> *physx_PxAggregate_Pod #foreign physx_sys;

phys_PxGetAggregateFilterHint :: (type_pod: s32, enableSelfCollision: bool) -> u32 #foreign physx_sys;

phys_PxGetAggregateSelfCollisionBit :: (hint: u32) -> u32 #foreign physx_sys;

phys_PxGetAggregateType :: (hint: u32) -> s32 #foreign physx_sys;

PxAggregate_release_mut :: (self__pod: *physx_PxAggregate_Pod) -> void #foreign physx_sys;

PxAggregate_addActor_mut :: (self__pod: *physx_PxAggregate_Pod, actor_pod: *physx_PxActor_Pod, bvh_pod: *physx_PxBVH_Pod) -> bool #foreign physx_sys;

PxAggregate_removeActor_mut :: (self__pod: *physx_PxAggregate_Pod, actor_pod: *physx_PxActor_Pod) -> bool #foreign physx_sys;

PxAggregate_addArticulation_mut :: (self__pod: *physx_PxAggregate_Pod, articulation_pod: *physx_PxArticulationReducedCoordinate_Pod) -> bool #foreign physx_sys;

PxAggregate_removeArticulation_mut :: (self__pod: *physx_PxAggregate_Pod, articulation_pod: *physx_PxArticulationReducedCoordinate_Pod) -> bool #foreign physx_sys;

PxAggregate_getNbActors :: (self__pod: *physx_PxAggregate_Pod) -> u32 #foreign physx_sys;

PxAggregate_getMaxNbShapes :: (self__pod: *physx_PxAggregate_Pod) -> u32 #foreign physx_sys;

PxAggregate_getActors :: (self__pod: *physx_PxAggregate_Pod, userBuffer_pod: **physx_PxActor_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxAggregate_getScene_mut :: (self__pod: *physx_PxAggregate_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxAggregate_getSelfCollision :: (self__pod: *physx_PxAggregate_Pod) -> bool #foreign physx_sys;

PxAggregate_getConcreteTypeName :: (self__pod: *physx_PxAggregate_Pod) -> *u8 #foreign physx_sys;

PxConstraintInvMassScale_new :: () -> physx_PxConstraintInvMassScale_Pod #foreign physx_sys;

PxConstraintInvMassScale_new_1 :: (lin0: float, ang0: float, lin1: float, ang1: float) -> physx_PxConstraintInvMassScale_Pod #foreign physx_sys;

PxConstraintVisualizer_visualizeJointFrames_mut :: (self__pod: *physx_PxConstraintVisualizer_Pod, parent_pod: *physx_PxTransform_Pod, child_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxConstraintVisualizer_visualizeLinearLimit_mut :: (self__pod: *physx_PxConstraintVisualizer_Pod, t0_pod: *physx_PxTransform_Pod, t1_pod: *physx_PxTransform_Pod, value: float, active: bool) -> void #foreign physx_sys;

PxConstraintVisualizer_visualizeAngularLimit_mut :: (self__pod: *physx_PxConstraintVisualizer_Pod, t0_pod: *physx_PxTransform_Pod, lower: float, upper: float, active: bool) -> void #foreign physx_sys;

PxConstraintVisualizer_visualizeLimitCone_mut :: (self__pod: *physx_PxConstraintVisualizer_Pod, t_pod: *physx_PxTransform_Pod, tanQSwingY: float, tanQSwingZ: float, active: bool) -> void #foreign physx_sys;

PxConstraintVisualizer_visualizeDoubleCone_mut :: (self__pod: *physx_PxConstraintVisualizer_Pod, t_pod: *physx_PxTransform_Pod, angle: float, active: bool) -> void #foreign physx_sys;

PxConstraintVisualizer_visualizeLine_mut :: (self__pod: *physx_PxConstraintVisualizer_Pod, p0_pod: *physx_PxVec3_Pod, p1_pod: *physx_PxVec3_Pod, color: u32) -> void #foreign physx_sys;

PxConstraintConnector_prepareData_mut :: (self__pod: *physx_PxConstraintConnector_Pod) -> *void #foreign physx_sys;

PxConstraintConnector_onConstraintRelease_mut :: (self__pod: *physx_PxConstraintConnector_Pod) -> void #foreign physx_sys;

PxConstraintConnector_onComShift_mut :: (self__pod: *physx_PxConstraintConnector_Pod, actor: u32) -> void #foreign physx_sys;

PxConstraintConnector_onOriginShift_mut :: (self__pod: *physx_PxConstraintConnector_Pod, shift_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxConstraintConnector_getSerializable_mut :: (self__pod: *physx_PxConstraintConnector_Pod) -> *physx_PxBase_Pod #foreign physx_sys;

PxConstraintConnector_getConstantBlock :: (self__pod: *physx_PxConstraintConnector_Pod) -> *void #foreign physx_sys;

PxConstraintConnector_connectToConstraint_mut :: (self__pod: *physx_PxConstraintConnector_Pod, anon_param0_pod: *physx_PxConstraint_Pod) -> void #foreign physx_sys;

PxConstraintConnector_delete :: (self__pod: *physx_PxConstraintConnector_Pod) -> void #foreign physx_sys;

PxSolverBody_new :: () -> physx_PxSolverBody_Pod #foreign physx_sys;

PxSolverBodyData_projectVelocity :: (self__pod: *physx_PxSolverBodyData_Pod, lin_pod: *physx_PxVec3_Pod, ang_pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxSolverConstraintPrepDesc_delete :: (self__pod: *physx_PxSolverConstraintPrepDesc_Pod) -> void #foreign physx_sys;

PxConstraintAllocator_reserveConstraintData_mut :: (self__pod: *physx_PxConstraintAllocator_Pod, byteSize: u32) -> *u8 #foreign physx_sys;

PxConstraintAllocator_reserveFrictionData_mut :: (self__pod: *physx_PxConstraintAllocator_Pod, byteSize: u32) -> *u8 #foreign physx_sys;

PxConstraintAllocator_delete :: (self__pod: *physx_PxConstraintAllocator_Pod) -> void #foreign physx_sys;

PxArticulationLimit_new :: () -> physx_PxArticulationLimit_Pod #foreign physx_sys;

PxArticulationLimit_new_1 :: (low_: float, high_: float) -> physx_PxArticulationLimit_Pod #foreign physx_sys;

PxArticulationDrive_new :: () -> physx_PxArticulationDrive_Pod #foreign physx_sys;

PxArticulationDrive_new_1 :: (stiffness_: float, damping_: float, maxForce_: float, driveType__pod: s32) -> physx_PxArticulationDrive_Pod #foreign physx_sys;

PxTGSSolverBodyVel_projectVelocity :: (self__pod: *physx_PxTGSSolverBodyVel_Pod, lin_pod: *physx_PxVec3_Pod, ang_pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxTGSSolverBodyData_projectVelocity :: (self__pod: *physx_PxTGSSolverBodyData_Pod, linear_pod: *physx_PxVec3_Pod, angular_pod: *physx_PxVec3_Pod) -> float #foreign physx_sys;

PxTGSSolverConstraintPrepDesc_delete :: (self__pod: *physx_PxTGSSolverConstraintPrepDesc_Pod) -> void #foreign physx_sys;

PxArticulationAttachment_setRestLength_mut :: (self__pod: *physx_PxArticulationAttachment_Pod, restLength: float) -> void #foreign physx_sys;

PxArticulationAttachment_getRestLength :: (self__pod: *physx_PxArticulationAttachment_Pod) -> float #foreign physx_sys;

PxArticulationAttachment_setLimitParameters_mut :: (self__pod: *physx_PxArticulationAttachment_Pod, parameters_pod: *physx_PxArticulationTendonLimit_Pod) -> void #foreign physx_sys;

PxArticulationAttachment_getLimitParameters :: (self__pod: *physx_PxArticulationAttachment_Pod) -> physx_PxArticulationTendonLimit_Pod #foreign physx_sys;

PxArticulationAttachment_setRelativeOffset_mut :: (self__pod: *physx_PxArticulationAttachment_Pod, offset_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxArticulationAttachment_getRelativeOffset :: (self__pod: *physx_PxArticulationAttachment_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxArticulationAttachment_setCoefficient_mut :: (self__pod: *physx_PxArticulationAttachment_Pod, coefficient: float) -> void #foreign physx_sys;

PxArticulationAttachment_getCoefficient :: (self__pod: *physx_PxArticulationAttachment_Pod) -> float #foreign physx_sys;

PxArticulationAttachment_getLink :: (self__pod: *physx_PxArticulationAttachment_Pod) -> *physx_PxArticulationLink_Pod #foreign physx_sys;

PxArticulationAttachment_getParent :: (self__pod: *physx_PxArticulationAttachment_Pod) -> *physx_PxArticulationAttachment_Pod #foreign physx_sys;

PxArticulationAttachment_isLeaf :: (self__pod: *physx_PxArticulationAttachment_Pod) -> bool #foreign physx_sys;

PxArticulationAttachment_getTendon :: (self__pod: *physx_PxArticulationAttachment_Pod) -> *physx_PxArticulationSpatialTendon_Pod #foreign physx_sys;

PxArticulationAttachment_release_mut :: (self__pod: *physx_PxArticulationAttachment_Pod) -> void #foreign physx_sys;

PxArticulationAttachment_getConcreteTypeName :: (self__pod: *physx_PxArticulationAttachment_Pod) -> *u8 #foreign physx_sys;

PxArticulationTendonJoint_setCoefficient_mut :: (self__pod: *physx_PxArticulationTendonJoint_Pod, axis_pod: s32, coefficient: float, recipCoefficient: float) -> void #foreign physx_sys;

PxArticulationTendonJoint_getCoefficient :: (self__pod: *physx_PxArticulationTendonJoint_Pod, axis_pod: *s32, coefficient_pod: *float, recipCoefficient_pod: *float) -> void #foreign physx_sys;

PxArticulationTendonJoint_getLink :: (self__pod: *physx_PxArticulationTendonJoint_Pod) -> *physx_PxArticulationLink_Pod #foreign physx_sys;

PxArticulationTendonJoint_getParent :: (self__pod: *physx_PxArticulationTendonJoint_Pod) -> *physx_PxArticulationTendonJoint_Pod #foreign physx_sys;

PxArticulationTendonJoint_getTendon :: (self__pod: *physx_PxArticulationTendonJoint_Pod) -> *physx_PxArticulationFixedTendon_Pod #foreign physx_sys;

PxArticulationTendonJoint_release_mut :: (self__pod: *physx_PxArticulationTendonJoint_Pod) -> void #foreign physx_sys;

PxArticulationTendonJoint_getConcreteTypeName :: (self__pod: *physx_PxArticulationTendonJoint_Pod) -> *u8 #foreign physx_sys;

PxArticulationTendon_setStiffness_mut :: (self__pod: *physx_PxArticulationTendon_Pod, stiffness: float) -> void #foreign physx_sys;

PxArticulationTendon_getStiffness :: (self__pod: *physx_PxArticulationTendon_Pod) -> float #foreign physx_sys;

PxArticulationTendon_setDamping_mut :: (self__pod: *physx_PxArticulationTendon_Pod, damping: float) -> void #foreign physx_sys;

PxArticulationTendon_getDamping :: (self__pod: *physx_PxArticulationTendon_Pod) -> float #foreign physx_sys;

PxArticulationTendon_setLimitStiffness_mut :: (self__pod: *physx_PxArticulationTendon_Pod, stiffness: float) -> void #foreign physx_sys;

PxArticulationTendon_getLimitStiffness :: (self__pod: *physx_PxArticulationTendon_Pod) -> float #foreign physx_sys;

PxArticulationTendon_setOffset_mut :: (self__pod: *physx_PxArticulationTendon_Pod, offset: float, autowake: bool) -> void #foreign physx_sys;

PxArticulationTendon_getOffset :: (self__pod: *physx_PxArticulationTendon_Pod) -> float #foreign physx_sys;

PxArticulationTendon_getArticulation :: (self__pod: *physx_PxArticulationTendon_Pod) -> *physx_PxArticulationReducedCoordinate_Pod #foreign physx_sys;

PxArticulationTendon_release_mut :: (self__pod: *physx_PxArticulationTendon_Pod) -> void #foreign physx_sys;

PxArticulationSpatialTendon_createAttachment_mut :: (self__pod: *physx_PxArticulationSpatialTendon_Pod, parent_pod: *physx_PxArticulationAttachment_Pod, coefficient: float, relativeOffset_pod: physx_PxVec3_Pod, link_pod: *physx_PxArticulationLink_Pod) -> *physx_PxArticulationAttachment_Pod #foreign physx_sys;

PxArticulationSpatialTendon_getAttachments :: (self__pod: *physx_PxArticulationSpatialTendon_Pod, userBuffer_pod: **physx_PxArticulationAttachment_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationSpatialTendon_getNbAttachments :: (self__pod: *physx_PxArticulationSpatialTendon_Pod) -> u32 #foreign physx_sys;

PxArticulationSpatialTendon_getConcreteTypeName :: (self__pod: *physx_PxArticulationSpatialTendon_Pod) -> *u8 #foreign physx_sys;

PxArticulationFixedTendon_createTendonJoint_mut :: (self__pod: *physx_PxArticulationFixedTendon_Pod, parent_pod: *physx_PxArticulationTendonJoint_Pod, axis_pod: s32, coefficient: float, recipCoefficient: float, link_pod: *physx_PxArticulationLink_Pod) -> *physx_PxArticulationTendonJoint_Pod #foreign physx_sys;

PxArticulationFixedTendon_getTendonJoints :: (self__pod: *physx_PxArticulationFixedTendon_Pod, userBuffer_pod: **physx_PxArticulationTendonJoint_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationFixedTendon_getNbTendonJoints :: (self__pod: *physx_PxArticulationFixedTendon_Pod) -> u32 #foreign physx_sys;

PxArticulationFixedTendon_setRestLength_mut :: (self__pod: *physx_PxArticulationFixedTendon_Pod, restLength: float) -> void #foreign physx_sys;

PxArticulationFixedTendon_getRestLength :: (self__pod: *physx_PxArticulationFixedTendon_Pod) -> float #foreign physx_sys;

PxArticulationFixedTendon_setLimitParameters_mut :: (self__pod: *physx_PxArticulationFixedTendon_Pod, parameter_pod: *physx_PxArticulationTendonLimit_Pod) -> void #foreign physx_sys;

PxArticulationFixedTendon_getLimitParameters :: (self__pod: *physx_PxArticulationFixedTendon_Pod) -> physx_PxArticulationTendonLimit_Pod #foreign physx_sys;

PxArticulationFixedTendon_getConcreteTypeName :: (self__pod: *physx_PxArticulationFixedTendon_Pod) -> *u8 #foreign physx_sys;

PxArticulationCache_new :: () -> physx_PxArticulationCache_Pod #foreign physx_sys;

PxArticulationCache_release_mut :: (self__pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationSensor_release_mut :: (self__pod: *physx_PxArticulationSensor_Pod) -> void #foreign physx_sys;

PxArticulationSensor_getForces :: (self__pod: *physx_PxArticulationSensor_Pod) -> physx_PxSpatialForce_Pod #foreign physx_sys;

PxArticulationSensor_getRelativePose :: (self__pod: *physx_PxArticulationSensor_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxArticulationSensor_setRelativePose_mut :: (self__pod: *physx_PxArticulationSensor_Pod, pose_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxArticulationSensor_getLink :: (self__pod: *physx_PxArticulationSensor_Pod) -> *physx_PxArticulationLink_Pod #foreign physx_sys;

PxArticulationSensor_getIndex :: (self__pod: *physx_PxArticulationSensor_Pod) -> u32 #foreign physx_sys;

PxArticulationSensor_getArticulation :: (self__pod: *physx_PxArticulationSensor_Pod) -> *physx_PxArticulationReducedCoordinate_Pod #foreign physx_sys;

PxArticulationSensor_getFlags :: (self__pod: *physx_PxArticulationSensor_Pod) -> u8 #foreign physx_sys;

PxArticulationSensor_setFlag_mut :: (self__pod: *physx_PxArticulationSensor_Pod, flag_pod: s32, enabled: bool) -> void #foreign physx_sys;

PxArticulationSensor_getConcreteTypeName :: (self__pod: *physx_PxArticulationSensor_Pod) -> *u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getScene :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_setSolverIterationCounts_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, minPositionIters: u32, minVelocityIters: u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getSolverIterationCounts :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, minPositionIters_pod: *u32, minVelocityIters_pod: *u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_isSleeping :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> bool #foreign physx_sys;

PxArticulationReducedCoordinate_setSleepThreshold_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, threshold: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getSleepThreshold :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setStabilizationThreshold_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, threshold: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getStabilizationThreshold :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setWakeCounter_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, wakeCounterValue: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getWakeCounter :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_wakeUp_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_putToSleep_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, maxLinearVelocity: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getMaxCOMLinearVelocity :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, maxAngularVelocity: float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getMaxCOMAngularVelocity :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationReducedCoordinate_createLink_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, parent_pod: *physx_PxArticulationLink_Pod, pose_pod: *physx_PxTransform_Pod) -> *physx_PxArticulationLink_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_release_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getNbLinks :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getLinks :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, userBuffer_pod: **physx_PxArticulationLink_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbShapes :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_setName_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, name: *u8) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getName :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> *u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getWorldBounds :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, inflation: float) -> physx_PxBounds3_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_getAggregate :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> *physx_PxAggregate_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_setArticulationFlags_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, flags_pod: u8) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_setArticulationFlag_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getArticulationFlags :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u8 #foreign physx_sys;

PxArticulationReducedCoordinate_getDofs :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_createCache :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> *physx_PxArticulationCache_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_getCacheDataSize :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_zeroCache :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_applyCache_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod, flags_pod: u32, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_copyInternalStateToCache :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod, flags_pod: u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_packJointData :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, maximum: *float, reduced: *float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_unpackJointData :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, reduced: *float, maximum: *float) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_commonInit :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedGravityForce :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedExternalForce :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeJointAcceleration :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeJointForce :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeDenseJacobian :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod, nRows_pod: *u32, nCols_pod: *u32) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeCoefficientMatrix :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_computeLambda :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod, initialState_pod: *physx_PxArticulationCache_Pod, jointTorque: *float, maxIter: u32) -> bool #foreign physx_sys;

PxArticulationReducedCoordinate_computeGeneralizedMassMatrix :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, cache_pod: *physx_PxArticulationCache_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_addLoopJoint_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, joint_pod: *physx_PxConstraint_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_removeLoopJoint_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, joint_pod: *physx_PxConstraint_Pod) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getNbLoopJoints :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getLoopJoints :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, userBuffer_pod: **physx_PxConstraint_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getCoefficientMatrixSize :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_setRootGlobalPose_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, pose_pod: *physx_PxTransform_Pod, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getRootGlobalPose :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_setRootLinearVelocity_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, linearVelocity_pod: *physx_PxVec3_Pod, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getRootLinearVelocity :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_setRootAngularVelocity_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, angularVelocity_pod: *physx_PxVec3_Pod, autowake: bool) -> void #foreign physx_sys;

PxArticulationReducedCoordinate_getRootAngularVelocity :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_getLinkAcceleration_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, linkId: u32) -> physx_PxSpatialVelocity_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_getGpuArticulationIndex_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_createSpatialTendon_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> *physx_PxArticulationSpatialTendon_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_createFixedTendon_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> *physx_PxArticulationFixedTendon_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_createSensor_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, link_pod: *physx_PxArticulationLink_Pod, relativePose_pod: *physx_PxTransform_Pod) -> *physx_PxArticulationSensor_Pod #foreign physx_sys;

PxArticulationReducedCoordinate_getSpatialTendons :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, userBuffer_pod: **physx_PxArticulationSpatialTendon_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbSpatialTendons_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getFixedTendons :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, userBuffer_pod: **physx_PxArticulationFixedTendon_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbFixedTendons_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getSensors :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, userBuffer_pod: **physx_PxArticulationSensor_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_getNbSensors_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod) -> u32 #foreign physx_sys;

PxArticulationReducedCoordinate_updateKinematic_mut :: (self__pod: *physx_PxArticulationReducedCoordinate_Pod, flags_pod: u8) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getParentArticulationLink :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> *physx_PxArticulationLink_Pod #foreign physx_sys;

PxArticulationJointReducedCoordinate_setParentPose_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, pose_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getParentPose :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxArticulationJointReducedCoordinate_getChildArticulationLink :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> *physx_PxArticulationLink_Pod #foreign physx_sys;

PxArticulationJointReducedCoordinate_setChildPose_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, pose_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getChildPose :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxArticulationJointReducedCoordinate_setJointType_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, jointType_pod: s32) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getJointType :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> s32 #foreign physx_sys;

PxArticulationJointReducedCoordinate_setMotion_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, motion_pod: s32) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getMotion :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> s32 #foreign physx_sys;

PxArticulationJointReducedCoordinate_setLimitParams_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, limit_pod: *physx_PxArticulationLimit_Pod) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getLimitParams :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> physx_PxArticulationLimit_Pod #foreign physx_sys;

PxArticulationJointReducedCoordinate_setDriveParams_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, drive_pod: *physx_PxArticulationDrive_Pod) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getDriveParams :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> physx_PxArticulationDrive_Pod #foreign physx_sys;

PxArticulationJointReducedCoordinate_setDriveTarget_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, target: float, autowake: bool) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getDriveTarget :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setDriveVelocity_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, targetVel: float, autowake: bool) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getDriveVelocity :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setArmature_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, armature: float) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getArmature :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, coefficient: float) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getFrictionCoefficient :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, maxJointV: float) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getMaxJointVelocity :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setJointPosition_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, jointPos: float) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getJointPosition :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_setJointVelocity_mut :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32, jointVel: float) -> void #foreign physx_sys;

PxArticulationJointReducedCoordinate_getJointVelocity :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod, axis_pod: s32) -> float #foreign physx_sys;

PxArticulationJointReducedCoordinate_getConcreteTypeName :: (self__pod: *physx_PxArticulationJointReducedCoordinate_Pod) -> *u8 #foreign physx_sys;

PxShape_release_mut :: (self__pod: *physx_PxShape_Pod) -> void #foreign physx_sys;

PxShape_setGeometry_mut :: (self__pod: *physx_PxShape_Pod, geometry_pod: *physx_PxGeometry_Pod) -> void #foreign physx_sys;

PxShape_getGeometry :: (self__pod: *physx_PxShape_Pod) -> *physx_PxGeometry_Pod #foreign physx_sys;

PxShape_getActor :: (self__pod: *physx_PxShape_Pod) -> *physx_PxRigidActor_Pod #foreign physx_sys;

PxShape_setLocalPose_mut :: (self__pod: *physx_PxShape_Pod, pose_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxShape_getLocalPose :: (self__pod: *physx_PxShape_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxShape_setSimulationFilterData_mut :: (self__pod: *physx_PxShape_Pod, data_pod: *physx_PxFilterData_Pod) -> void #foreign physx_sys;

PxShape_getSimulationFilterData :: (self__pod: *physx_PxShape_Pod) -> physx_PxFilterData_Pod #foreign physx_sys;

PxShape_setQueryFilterData_mut :: (self__pod: *physx_PxShape_Pod, data_pod: *physx_PxFilterData_Pod) -> void #foreign physx_sys;

PxShape_getQueryFilterData :: (self__pod: *physx_PxShape_Pod) -> physx_PxFilterData_Pod #foreign physx_sys;

PxShape_setMaterials_mut :: (self__pod: *physx_PxShape_Pod, materials_pod: **physx_PxMaterial_Pod, materialCount: u16) -> void #foreign physx_sys;

PxShape_getNbMaterials :: (self__pod: *physx_PxShape_Pod) -> u16 #foreign physx_sys;

PxShape_getMaterials :: (self__pod: *physx_PxShape_Pod, userBuffer_pod: **physx_PxMaterial_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxShape_getMaterialFromInternalFaceIndex :: (self__pod: *physx_PxShape_Pod, faceIndex: u32) -> *physx_PxBaseMaterial_Pod #foreign physx_sys;

PxShape_setContactOffset_mut :: (self__pod: *physx_PxShape_Pod, contactOffset: float) -> void #foreign physx_sys;

PxShape_getContactOffset :: (self__pod: *physx_PxShape_Pod) -> float #foreign physx_sys;

PxShape_setRestOffset_mut :: (self__pod: *physx_PxShape_Pod, restOffset: float) -> void #foreign physx_sys;

PxShape_getRestOffset :: (self__pod: *physx_PxShape_Pod) -> float #foreign physx_sys;

PxShape_setDensityForFluid_mut :: (self__pod: *physx_PxShape_Pod, densityForFluid: float) -> void #foreign physx_sys;

PxShape_getDensityForFluid :: (self__pod: *physx_PxShape_Pod) -> float #foreign physx_sys;

PxShape_setTorsionalPatchRadius_mut :: (self__pod: *physx_PxShape_Pod, radius: float) -> void #foreign physx_sys;

PxShape_getTorsionalPatchRadius :: (self__pod: *physx_PxShape_Pod) -> float #foreign physx_sys;

PxShape_setMinTorsionalPatchRadius_mut :: (self__pod: *physx_PxShape_Pod, radius: float) -> void #foreign physx_sys;

PxShape_getMinTorsionalPatchRadius :: (self__pod: *physx_PxShape_Pod) -> float #foreign physx_sys;

PxShape_setFlag_mut :: (self__pod: *physx_PxShape_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxShape_setFlags_mut :: (self__pod: *physx_PxShape_Pod, inFlags_pod: u8) -> void #foreign physx_sys;

PxShape_getFlags :: (self__pod: *physx_PxShape_Pod) -> u8 #foreign physx_sys;

PxShape_isExclusive :: (self__pod: *physx_PxShape_Pod) -> bool #foreign physx_sys;

PxShape_setName_mut :: (self__pod: *physx_PxShape_Pod, name: *u8) -> void #foreign physx_sys;

PxShape_getName :: (self__pod: *physx_PxShape_Pod) -> *u8 #foreign physx_sys;

PxShape_getConcreteTypeName :: (self__pod: *physx_PxShape_Pod) -> *u8 #foreign physx_sys;

PxRigidActor_release_mut :: (self__pod: *physx_PxRigidActor_Pod) -> void #foreign physx_sys;

PxRigidActor_getInternalActorIndex :: (self__pod: *physx_PxRigidActor_Pod) -> u32 #foreign physx_sys;

PxRigidActor_getGlobalPose :: (self__pod: *physx_PxRigidActor_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxRigidActor_setGlobalPose_mut :: (self__pod: *physx_PxRigidActor_Pod, pose_pod: *physx_PxTransform_Pod, autowake: bool) -> void #foreign physx_sys;

PxRigidActor_attachShape_mut :: (self__pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod) -> bool #foreign physx_sys;

PxRigidActor_detachShape_mut :: (self__pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxRigidActor_getNbShapes :: (self__pod: *physx_PxRigidActor_Pod) -> u32 #foreign physx_sys;

PxRigidActor_getShapes :: (self__pod: *physx_PxRigidActor_Pod, userBuffer_pod: **physx_PxShape_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxRigidActor_getNbConstraints :: (self__pod: *physx_PxRigidActor_Pod) -> u32 #foreign physx_sys;

PxRigidActor_getConstraints :: (self__pod: *physx_PxRigidActor_Pod, userBuffer_pod: **physx_PxConstraint_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxNodeIndex_new :: (id: u32, articLinkId: u32) -> physx_PxNodeIndex_Pod #foreign physx_sys;

PxNodeIndex_new_1 :: (id: u32) -> physx_PxNodeIndex_Pod #foreign physx_sys;

PxNodeIndex_index :: (self__pod: *physx_PxNodeIndex_Pod) -> u32 #foreign physx_sys;

PxNodeIndex_articulationLinkId :: (self__pod: *physx_PxNodeIndex_Pod) -> u32 #foreign physx_sys;

PxNodeIndex_isArticulation :: (self__pod: *physx_PxNodeIndex_Pod) -> u32 #foreign physx_sys;

PxNodeIndex_isStaticBody :: (self__pod: *physx_PxNodeIndex_Pod) -> bool #foreign physx_sys;

PxNodeIndex_isValid :: (self__pod: *physx_PxNodeIndex_Pod) -> bool #foreign physx_sys;

PxNodeIndex_setIndices_mut :: (self__pod: *physx_PxNodeIndex_Pod, index: u32, articLinkId: u32) -> void #foreign physx_sys;

PxNodeIndex_setIndices_mut_1 :: (self__pod: *physx_PxNodeIndex_Pod, index: u32) -> void #foreign physx_sys;

PxNodeIndex_getInd :: (self__pod: *physx_PxNodeIndex_Pod) -> u64 #foreign physx_sys;

PxRigidBody_setCMassLocalPose_mut :: (self__pod: *physx_PxRigidBody_Pod, pose_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxRigidBody_getCMassLocalPose :: (self__pod: *physx_PxRigidBody_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxRigidBody_setMass_mut :: (self__pod: *physx_PxRigidBody_Pod, mass: float) -> void #foreign physx_sys;

PxRigidBody_getMass :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_getInvMass :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_setMassSpaceInertiaTensor_mut :: (self__pod: *physx_PxRigidBody_Pod, m_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxRigidBody_getMassSpaceInertiaTensor :: (self__pod: *physx_PxRigidBody_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBody_getMassSpaceInvInertiaTensor :: (self__pod: *physx_PxRigidBody_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBody_setLinearDamping_mut :: (self__pod: *physx_PxRigidBody_Pod, linDamp: float) -> void #foreign physx_sys;

PxRigidBody_getLinearDamping :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_setAngularDamping_mut :: (self__pod: *physx_PxRigidBody_Pod, angDamp: float) -> void #foreign physx_sys;

PxRigidBody_getAngularDamping :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_getLinearVelocity :: (self__pod: *physx_PxRigidBody_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBody_getAngularVelocity :: (self__pod: *physx_PxRigidBody_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBody_setMaxLinearVelocity_mut :: (self__pod: *physx_PxRigidBody_Pod, maxLinVel: float) -> void #foreign physx_sys;

PxRigidBody_getMaxLinearVelocity :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_setMaxAngularVelocity_mut :: (self__pod: *physx_PxRigidBody_Pod, maxAngVel: float) -> void #foreign physx_sys;

PxRigidBody_getMaxAngularVelocity :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_addForce_mut :: (self__pod: *physx_PxRigidBody_Pod, force_pod: *physx_PxVec3_Pod, mode_pod: s32, autowake: bool) -> void #foreign physx_sys;

PxRigidBody_addTorque_mut :: (self__pod: *physx_PxRigidBody_Pod, torque_pod: *physx_PxVec3_Pod, mode_pod: s32, autowake: bool) -> void #foreign physx_sys;

PxRigidBody_clearForce_mut :: (self__pod: *physx_PxRigidBody_Pod, mode_pod: s32) -> void #foreign physx_sys;

PxRigidBody_clearTorque_mut :: (self__pod: *physx_PxRigidBody_Pod, mode_pod: s32) -> void #foreign physx_sys;

PxRigidBody_setForceAndTorque_mut :: (self__pod: *physx_PxRigidBody_Pod, force_pod: *physx_PxVec3_Pod, torque_pod: *physx_PxVec3_Pod, mode_pod: s32) -> void #foreign physx_sys;

PxRigidBody_setRigidBodyFlag_mut :: (self__pod: *physx_PxRigidBody_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxRigidBody_setRigidBodyFlags_mut :: (self__pod: *physx_PxRigidBody_Pod, inFlags_pod: u16) -> void #foreign physx_sys;

PxRigidBody_getRigidBodyFlags :: (self__pod: *physx_PxRigidBody_Pod) -> u16 #foreign physx_sys;

PxRigidBody_setMinCCDAdvanceCoefficient_mut :: (self__pod: *physx_PxRigidBody_Pod, advanceCoefficient: float) -> void #foreign physx_sys;

PxRigidBody_getMinCCDAdvanceCoefficient :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_setMaxDepenetrationVelocity_mut :: (self__pod: *physx_PxRigidBody_Pod, biasClamp: float) -> void #foreign physx_sys;

PxRigidBody_getMaxDepenetrationVelocity :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_setMaxContactImpulse_mut :: (self__pod: *physx_PxRigidBody_Pod, maxImpulse: float) -> void #foreign physx_sys;

PxRigidBody_getMaxContactImpulse :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_setContactSlopCoefficient_mut :: (self__pod: *physx_PxRigidBody_Pod, slopCoefficient: float) -> void #foreign physx_sys;

PxRigidBody_getContactSlopCoefficient :: (self__pod: *physx_PxRigidBody_Pod) -> float #foreign physx_sys;

PxRigidBody_getInternalIslandNodeIndex :: (self__pod: *physx_PxRigidBody_Pod) -> physx_PxNodeIndex_Pod #foreign physx_sys;

PxArticulationLink_release_mut :: (self__pod: *physx_PxArticulationLink_Pod) -> void #foreign physx_sys;

PxArticulationLink_getArticulation :: (self__pod: *physx_PxArticulationLink_Pod) -> *physx_PxArticulationReducedCoordinate_Pod #foreign physx_sys;

PxArticulationLink_getInboundJoint :: (self__pod: *physx_PxArticulationLink_Pod) -> *physx_PxArticulationJointReducedCoordinate_Pod #foreign physx_sys;

PxArticulationLink_getInboundJointDof :: (self__pod: *physx_PxArticulationLink_Pod) -> u32 #foreign physx_sys;

PxArticulationLink_getNbChildren :: (self__pod: *physx_PxArticulationLink_Pod) -> u32 #foreign physx_sys;

PxArticulationLink_getLinkIndex :: (self__pod: *physx_PxArticulationLink_Pod) -> u32 #foreign physx_sys;

PxArticulationLink_getChildren :: (self__pod: *physx_PxArticulationLink_Pod, userBuffer_pod: **physx_PxArticulationLink_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxArticulationLink_setCfmScale_mut :: (self__pod: *physx_PxArticulationLink_Pod, cfm: float) -> void #foreign physx_sys;

PxArticulationLink_getCfmScale :: (self__pod: *physx_PxArticulationLink_Pod) -> float #foreign physx_sys;

PxArticulationLink_getLinearVelocity :: (self__pod: *physx_PxArticulationLink_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxArticulationLink_getAngularVelocity :: (self__pod: *physx_PxArticulationLink_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxArticulationLink_getConcreteTypeName :: (self__pod: *physx_PxArticulationLink_Pod) -> *u8 #foreign physx_sys;

PxConeLimitedConstraint_new :: () -> physx_PxConeLimitedConstraint_Pod #foreign physx_sys;

PxConstraint_release_mut :: (self__pod: *physx_PxConstraint_Pod) -> void #foreign physx_sys;

PxConstraint_getScene :: (self__pod: *physx_PxConstraint_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxConstraint_getActors :: (self__pod: *physx_PxConstraint_Pod, actor0_pod: **physx_PxRigidActor_Pod, actor1_pod: **physx_PxRigidActor_Pod) -> void #foreign physx_sys;

PxConstraint_setActors_mut :: (self__pod: *physx_PxConstraint_Pod, actor0_pod: *physx_PxRigidActor_Pod, actor1_pod: *physx_PxRigidActor_Pod) -> void #foreign physx_sys;

PxConstraint_markDirty_mut :: (self__pod: *physx_PxConstraint_Pod) -> void #foreign physx_sys;

PxConstraint_getFlags :: (self__pod: *physx_PxConstraint_Pod) -> u16 #foreign physx_sys;

PxConstraint_setFlags_mut :: (self__pod: *physx_PxConstraint_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxConstraint_setFlag_mut :: (self__pod: *physx_PxConstraint_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxConstraint_getForce :: (self__pod: *physx_PxConstraint_Pod, linear_pod: *physx_PxVec3_Pod, angular_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxConstraint_isValid :: (self__pod: *physx_PxConstraint_Pod) -> bool #foreign physx_sys;

PxConstraint_setBreakForce_mut :: (self__pod: *physx_PxConstraint_Pod, linear: float, angular: float) -> void #foreign physx_sys;

PxConstraint_getBreakForce :: (self__pod: *physx_PxConstraint_Pod, linear_pod: *float, angular_pod: *float) -> void #foreign physx_sys;

PxConstraint_setMinResponseThreshold_mut :: (self__pod: *physx_PxConstraint_Pod, threshold: float) -> void #foreign physx_sys;

PxConstraint_getMinResponseThreshold :: (self__pod: *physx_PxConstraint_Pod) -> float #foreign physx_sys;

PxConstraint_getExternalReference_mut :: (self__pod: *physx_PxConstraint_Pod, typeID_pod: *u32) -> *void #foreign physx_sys;

PxConstraint_setConstraintFunctions_mut :: (self__pod: *physx_PxConstraint_Pod, connector_pod: *physx_PxConstraintConnector_Pod, shaders_pod: *physx_PxConstraintShaderTable_Pod) -> void #foreign physx_sys;

PxConstraint_getConcreteTypeName :: (self__pod: *physx_PxConstraint_Pod) -> *u8 #foreign physx_sys;

PxContactStreamIterator_new :: (contactPatches: *u8, contactPoints: *u8, contactFaceIndices: *u32, nbPatches: u32, nbContacts: u32) -> physx_PxContactStreamIterator_Pod #foreign physx_sys;

PxContactStreamIterator_hasNextPatch :: (self__pod: *physx_PxContactStreamIterator_Pod) -> bool #foreign physx_sys;

PxContactStreamIterator_getTotalContactCount :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u32 #foreign physx_sys;

PxContactStreamIterator_getTotalPatchCount :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u32 #foreign physx_sys;

PxContactStreamIterator_nextPatch_mut :: (self__pod: *physx_PxContactStreamIterator_Pod) -> void #foreign physx_sys;

PxContactStreamIterator_hasNextContact :: (self__pod: *physx_PxContactStreamIterator_Pod) -> bool #foreign physx_sys;

PxContactStreamIterator_nextContact_mut :: (self__pod: *physx_PxContactStreamIterator_Pod) -> void #foreign physx_sys;

PxContactStreamIterator_getContactNormal :: (self__pod: *physx_PxContactStreamIterator_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxContactStreamIterator_getInvMassScale0 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getInvMassScale1 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getInvInertiaScale0 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getInvInertiaScale1 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getMaxImpulse :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getTargetVel :: (self__pod: *physx_PxContactStreamIterator_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxContactStreamIterator_getContactPoint :: (self__pod: *physx_PxContactStreamIterator_Pod) -> *physx_PxVec3_Pod #foreign physx_sys;

PxContactStreamIterator_getSeparation :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getFaceIndex0 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u32 #foreign physx_sys;

PxContactStreamIterator_getFaceIndex1 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u32 #foreign physx_sys;

PxContactStreamIterator_getStaticFriction :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getDynamicFriction :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getRestitution :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getDamping :: (self__pod: *physx_PxContactStreamIterator_Pod) -> float #foreign physx_sys;

PxContactStreamIterator_getMaterialFlags :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u32 #foreign physx_sys;

PxContactStreamIterator_getMaterialIndex0 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u16 #foreign physx_sys;

PxContactStreamIterator_getMaterialIndex1 :: (self__pod: *physx_PxContactStreamIterator_Pod) -> u16 #foreign physx_sys;

PxContactStreamIterator_advanceToIndex_mut :: (self__pod: *physx_PxContactStreamIterator_Pod, initialIndex: u32) -> bool #foreign physx_sys;

PxContactSet_getPoint :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> *physx_PxVec3_Pod #foreign physx_sys;

PxContactSet_setPoint_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, p_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxContactSet_getNormal :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> *physx_PxVec3_Pod #foreign physx_sys;

PxContactSet_setNormal_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, n_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxContactSet_getSeparation :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> float #foreign physx_sys;

PxContactSet_setSeparation_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, s: float) -> void #foreign physx_sys;

PxContactSet_getTargetVelocity :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> *physx_PxVec3_Pod #foreign physx_sys;

PxContactSet_setTargetVelocity_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, v_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxContactSet_getInternalFaceIndex0 :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> u32 #foreign physx_sys;

PxContactSet_getInternalFaceIndex1 :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> u32 #foreign physx_sys;

PxContactSet_getMaxImpulse :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> float #foreign physx_sys;

PxContactSet_setMaxImpulse_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, s: float) -> void #foreign physx_sys;

PxContactSet_getRestitution :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> float #foreign physx_sys;

PxContactSet_setRestitution_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, r: float) -> void #foreign physx_sys;

PxContactSet_getStaticFriction :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> float #foreign physx_sys;

PxContactSet_setStaticFriction_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, f: float) -> void #foreign physx_sys;

PxContactSet_getDynamicFriction :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> float #foreign physx_sys;

PxContactSet_setDynamicFriction_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32, f: float) -> void #foreign physx_sys;

PxContactSet_ignore_mut :: (self__pod: *physx_PxContactSet_Pod, i: u32) -> void #foreign physx_sys;

PxContactSet_size :: (self__pod: *physx_PxContactSet_Pod) -> u32 #foreign physx_sys;

PxContactSet_getInvMassScale0 :: (self__pod: *physx_PxContactSet_Pod) -> float #foreign physx_sys;

PxContactSet_getInvMassScale1 :: (self__pod: *physx_PxContactSet_Pod) -> float #foreign physx_sys;

PxContactSet_getInvInertiaScale0 :: (self__pod: *physx_PxContactSet_Pod) -> float #foreign physx_sys;

PxContactSet_getInvInertiaScale1 :: (self__pod: *physx_PxContactSet_Pod) -> float #foreign physx_sys;

PxContactSet_setInvMassScale0_mut :: (self__pod: *physx_PxContactSet_Pod, scale: float) -> void #foreign physx_sys;

PxContactSet_setInvMassScale1_mut :: (self__pod: *physx_PxContactSet_Pod, scale: float) -> void #foreign physx_sys;

PxContactSet_setInvInertiaScale0_mut :: (self__pod: *physx_PxContactSet_Pod, scale: float) -> void #foreign physx_sys;

PxContactSet_setInvInertiaScale1_mut :: (self__pod: *physx_PxContactSet_Pod, scale: float) -> void #foreign physx_sys;

PxContactModifyCallback_onContactModify_mut :: (self__pod: *physx_PxContactModifyCallback_Pod, pairs_pod: *physx_PxContactModifyPair_Pod, count: u32) -> void #foreign physx_sys;

PxCCDContactModifyCallback_onCCDContactModify_mut :: (self__pod: *physx_PxCCDContactModifyCallback_Pod, pairs_pod: *physx_PxContactModifyPair_Pod, count: u32) -> void #foreign physx_sys;

PxDeletionListener_onRelease_mut :: (self__pod: *physx_PxDeletionListener_Pod, observed_pod: *physx_PxBase_Pod, userData: *void, deletionEvent_pod: s32) -> void #foreign physx_sys;

PxBaseMaterial_isKindOf :: (self__pod: *physx_PxBaseMaterial_Pod, name: *u8) -> bool #foreign physx_sys;

PxFEMMaterial_setYoungsModulus_mut :: (self__pod: *physx_PxFEMMaterial_Pod, young: float) -> void #foreign physx_sys;

PxFEMMaterial_getYoungsModulus :: (self__pod: *physx_PxFEMMaterial_Pod) -> float #foreign physx_sys;

PxFEMMaterial_setPoissons_mut :: (self__pod: *physx_PxFEMMaterial_Pod, poisson: float) -> void #foreign physx_sys;

PxFEMMaterial_getPoissons :: (self__pod: *physx_PxFEMMaterial_Pod) -> float #foreign physx_sys;

PxFEMMaterial_setDynamicFriction_mut :: (self__pod: *physx_PxFEMMaterial_Pod, dynamicFriction: float) -> void #foreign physx_sys;

PxFEMMaterial_getDynamicFriction :: (self__pod: *physx_PxFEMMaterial_Pod) -> float #foreign physx_sys;

PxFilterData_new :: (anon_param0_pod: s32) -> physx_PxFilterData_Pod #foreign physx_sys;

PxFilterData_new_1 :: () -> physx_PxFilterData_Pod #foreign physx_sys;

PxFilterData_new_2 :: (w0: u32, w1: u32, w2: u32, w3: u32) -> physx_PxFilterData_Pod #foreign physx_sys;

PxFilterData_setToDefault_mut :: (self__pod: *physx_PxFilterData_Pod) -> void #foreign physx_sys;

phys_PxGetFilterObjectType :: (attr: u32) -> s32 #foreign physx_sys;

phys_PxFilterObjectIsKinematic :: (attr: u32) -> bool #foreign physx_sys;

phys_PxFilterObjectIsTrigger :: (attr: u32) -> bool #foreign physx_sys;

PxSimulationFilterCallback_pairFound_mut :: (self__pod: *physx_PxSimulationFilterCallback_Pod, pairID: u32, attributes0: u32, filterData0_pod: physx_PxFilterData_Pod, a0_pod: *physx_PxActor_Pod, s0_pod: *physx_PxShape_Pod, attributes1: u32, filterData1_pod: physx_PxFilterData_Pod, a1_pod: *physx_PxActor_Pod, s1_pod: *physx_PxShape_Pod, pairFlags_pod: *u16) -> u16 #foreign physx_sys;

PxSimulationFilterCallback_pairLost_mut :: (self__pod: *physx_PxSimulationFilterCallback_Pod, pairID: u32, attributes0: u32, filterData0_pod: physx_PxFilterData_Pod, attributes1: u32, filterData1_pod: physx_PxFilterData_Pod, objectRemoved: bool) -> void #foreign physx_sys;

PxSimulationFilterCallback_statusChange_mut :: (self__pod: *physx_PxSimulationFilterCallback_Pod, pairID_pod: *u32, pairFlags_pod: *u16, filterFlags_pod: *u16) -> bool #foreign physx_sys;

PxLockedData_getDataAccessFlags_mut :: (self__pod: *physx_PxLockedData_Pod) -> u8 #foreign physx_sys;

PxLockedData_unlock_mut :: (self__pod: *physx_PxLockedData_Pod) -> void #foreign physx_sys;

PxLockedData_delete :: (self__pod: *physx_PxLockedData_Pod) -> void #foreign physx_sys;

PxMaterial_setDynamicFriction_mut :: (self__pod: *physx_PxMaterial_Pod, coef: float) -> void #foreign physx_sys;

PxMaterial_getDynamicFriction :: (self__pod: *physx_PxMaterial_Pod) -> float #foreign physx_sys;

PxMaterial_setStaticFriction_mut :: (self__pod: *physx_PxMaterial_Pod, coef: float) -> void #foreign physx_sys;

PxMaterial_getStaticFriction :: (self__pod: *physx_PxMaterial_Pod) -> float #foreign physx_sys;

PxMaterial_setRestitution_mut :: (self__pod: *physx_PxMaterial_Pod, rest: float) -> void #foreign physx_sys;

PxMaterial_getRestitution :: (self__pod: *physx_PxMaterial_Pod) -> float #foreign physx_sys;

PxMaterial_setDamping_mut :: (self__pod: *physx_PxMaterial_Pod, damping: float) -> void #foreign physx_sys;

PxMaterial_getDamping :: (self__pod: *physx_PxMaterial_Pod) -> float #foreign physx_sys;

PxMaterial_setFlag_mut :: (self__pod: *physx_PxMaterial_Pod, flag_pod: s32, b: bool) -> void #foreign physx_sys;

PxMaterial_setFlags_mut :: (self__pod: *physx_PxMaterial_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxMaterial_getFlags :: (self__pod: *physx_PxMaterial_Pod) -> u16 #foreign physx_sys;

PxMaterial_setFrictionCombineMode_mut :: (self__pod: *physx_PxMaterial_Pod, combMode_pod: s32) -> void #foreign physx_sys;

PxMaterial_getFrictionCombineMode :: (self__pod: *physx_PxMaterial_Pod) -> s32 #foreign physx_sys;

PxMaterial_setRestitutionCombineMode_mut :: (self__pod: *physx_PxMaterial_Pod, combMode_pod: s32) -> void #foreign physx_sys;

PxMaterial_getRestitutionCombineMode :: (self__pod: *physx_PxMaterial_Pod) -> s32 #foreign physx_sys;

PxMaterial_getConcreteTypeName :: (self__pod: *physx_PxMaterial_Pod) -> *u8 #foreign physx_sys;

PxDiffuseParticleParams_new :: () -> physx_PxDiffuseParticleParams_Pod #foreign physx_sys;

PxDiffuseParticleParams_setToDefault_mut :: (self__pod: *physx_PxDiffuseParticleParams_Pod) -> void #foreign physx_sys;

PxParticleMaterial_setFriction_mut :: (self__pod: *physx_PxParticleMaterial_Pod, friction: float) -> void #foreign physx_sys;

PxParticleMaterial_getFriction :: (self__pod: *physx_PxParticleMaterial_Pod) -> float #foreign physx_sys;

PxParticleMaterial_setDamping_mut :: (self__pod: *physx_PxParticleMaterial_Pod, damping: float) -> void #foreign physx_sys;

PxParticleMaterial_getDamping :: (self__pod: *physx_PxParticleMaterial_Pod) -> float #foreign physx_sys;

PxParticleMaterial_setAdhesion_mut :: (self__pod: *physx_PxParticleMaterial_Pod, adhesion: float) -> void #foreign physx_sys;

PxParticleMaterial_getAdhesion :: (self__pod: *physx_PxParticleMaterial_Pod) -> float #foreign physx_sys;

PxParticleMaterial_setGravityScale_mut :: (self__pod: *physx_PxParticleMaterial_Pod, scale: float) -> void #foreign physx_sys;

PxParticleMaterial_getGravityScale :: (self__pod: *physx_PxParticleMaterial_Pod) -> float #foreign physx_sys;

PxParticleMaterial_setAdhesionRadiusScale_mut :: (self__pod: *physx_PxParticleMaterial_Pod, scale: float) -> void #foreign physx_sys;

PxParticleMaterial_getAdhesionRadiusScale :: (self__pod: *physx_PxParticleMaterial_Pod) -> float #foreign physx_sys;

PxPhysics_release_mut :: (self__pod: *physx_PxPhysics_Pod) -> void #foreign physx_sys;

PxPhysics_getFoundation_mut :: (self__pod: *physx_PxPhysics_Pod) -> *physx_PxFoundation_Pod #foreign physx_sys;

PxPhysics_createAggregate_mut :: (self__pod: *physx_PxPhysics_Pod, maxActor: u32, maxShape: u32, filterHint: u32) -> *physx_PxAggregate_Pod #foreign physx_sys;

PxPhysics_getTolerancesScale :: (self__pod: *physx_PxPhysics_Pod) -> *physx_PxTolerancesScale_Pod #foreign physx_sys;

PxPhysics_createTriangleMesh_mut :: (self__pod: *physx_PxPhysics_Pod, stream_pod: *physx_PxInputStream_Pod) -> *physx_PxTriangleMesh_Pod #foreign physx_sys;

PxPhysics_getNbTriangleMeshes :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getTriangleMeshes :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxTriangleMesh_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createTetrahedronMesh_mut :: (self__pod: *physx_PxPhysics_Pod, stream_pod: *physx_PxInputStream_Pod) -> *physx_PxTetrahedronMesh_Pod #foreign physx_sys;

PxPhysics_createSoftBodyMesh_mut :: (self__pod: *physx_PxPhysics_Pod, stream_pod: *physx_PxInputStream_Pod) -> *physx_PxSoftBodyMesh_Pod #foreign physx_sys;

PxPhysics_getNbTetrahedronMeshes :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getTetrahedronMeshes :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxTetrahedronMesh_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createHeightField_mut :: (self__pod: *physx_PxPhysics_Pod, stream_pod: *physx_PxInputStream_Pod) -> *physx_PxHeightField_Pod #foreign physx_sys;

PxPhysics_getNbHeightFields :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getHeightFields :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxHeightField_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createConvexMesh_mut :: (self__pod: *physx_PxPhysics_Pod, stream_pod: *physx_PxInputStream_Pod) -> *physx_PxConvexMesh_Pod #foreign physx_sys;

PxPhysics_getNbConvexMeshes :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getConvexMeshes :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxConvexMesh_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createBVH_mut :: (self__pod: *physx_PxPhysics_Pod, stream_pod: *physx_PxInputStream_Pod) -> *physx_PxBVH_Pod #foreign physx_sys;

PxPhysics_getNbBVHs :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getBVHs :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxBVH_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createScene_mut :: (self__pod: *physx_PxPhysics_Pod, sceneDesc_pod: *physx_PxSceneDesc_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxPhysics_getNbScenes :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getScenes :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxScene_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createRigidStatic_mut :: (self__pod: *physx_PxPhysics_Pod, pose_pod: *physx_PxTransform_Pod) -> *physx_PxRigidStatic_Pod #foreign physx_sys;

PxPhysics_createRigidDynamic_mut :: (self__pod: *physx_PxPhysics_Pod, pose_pod: *physx_PxTransform_Pod) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

PxPhysics_createPruningStructure_mut :: (self__pod: *physx_PxPhysics_Pod, actors_pod: **physx_PxRigidActor_Pod, nbActors: u32) -> *physx_PxPruningStructure_Pod #foreign physx_sys;

PxPhysics_createShape_mut :: (self__pod: *physx_PxPhysics_Pod, geometry_pod: *physx_PxGeometry_Pod, material_pod: *physx_PxMaterial_Pod, isExclusive: bool, shapeFlags_pod: u8) -> *physx_PxShape_Pod #foreign physx_sys;

PxPhysics_createShape_mut_1 :: (self__pod: *physx_PxPhysics_Pod, geometry_pod: *physx_PxGeometry_Pod, materials_pod: **physx_PxMaterial_Pod, materialCount: u16, isExclusive: bool, shapeFlags_pod: u8) -> *physx_PxShape_Pod #foreign physx_sys;

PxPhysics_getNbShapes :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getShapes :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxShape_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_createConstraint_mut :: (self__pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, actor1_pod: *physx_PxRigidActor_Pod, connector_pod: *physx_PxConstraintConnector_Pod, shaders_pod: *physx_PxConstraintShaderTable_Pod, dataSize: u32) -> *physx_PxConstraint_Pod #foreign physx_sys;

PxPhysics_createArticulationReducedCoordinate_mut :: (self__pod: *physx_PxPhysics_Pod) -> *physx_PxArticulationReducedCoordinate_Pod #foreign physx_sys;

PxPhysics_createMaterial_mut :: (self__pod: *physx_PxPhysics_Pod, staticFriction: float, dynamicFriction: float, restitution: float) -> *physx_PxMaterial_Pod #foreign physx_sys;

PxPhysics_getNbMaterials :: (self__pod: *physx_PxPhysics_Pod) -> u32 #foreign physx_sys;

PxPhysics_getMaterials :: (self__pod: *physx_PxPhysics_Pod, userBuffer_pod: **physx_PxMaterial_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPhysics_registerDeletionListener_mut :: (self__pod: *physx_PxPhysics_Pod, observer_pod: *physx_PxDeletionListener_Pod, deletionEvents_pod: *u8, restrictedObjectSet: bool) -> void #foreign physx_sys;

PxPhysics_unregisterDeletionListener_mut :: (self__pod: *physx_PxPhysics_Pod, observer_pod: *physx_PxDeletionListener_Pod) -> void #foreign physx_sys;

PxPhysics_registerDeletionListenerObjects_mut :: (self__pod: *physx_PxPhysics_Pod, observer_pod: *physx_PxDeletionListener_Pod, observables_pod: **physx_PxBase_Pod, observableCount: u32) -> void #foreign physx_sys;

PxPhysics_unregisterDeletionListenerObjects_mut :: (self__pod: *physx_PxPhysics_Pod, observer_pod: *physx_PxDeletionListener_Pod, observables_pod: **physx_PxBase_Pod, observableCount: u32) -> void #foreign physx_sys;

PxPhysics_getPhysicsInsertionCallback_mut :: (self__pod: *physx_PxPhysics_Pod) -> *physx_PxInsertionCallback_Pod #foreign physx_sys;

phys_PxCreatePhysics :: (version: u32, foundation_pod: *physx_PxFoundation_Pod, scale_pod: *physx_PxTolerancesScale_Pod, trackOutstandingAllocations: bool, pvd_pod: *physx_PxPvd_Pod, omniPvd_pod: *physx_PxOmniPvd_Pod) -> *physx_PxPhysics_Pod #foreign physx_sys;

phys_PxGetPhysics :: () -> *physx_PxPhysics_Pod #foreign physx_sys;

PxActorShape_new :: () -> physx_PxActorShape_Pod #foreign physx_sys;

PxActorShape_new_1 :: (a_pod: *physx_PxRigidActor_Pod, s_pod: *physx_PxShape_Pod) -> physx_PxActorShape_Pod #foreign physx_sys;

PxQueryCache_new :: () -> physx_PxQueryCache_Pod #foreign physx_sys;

PxQueryCache_new_1 :: (s_pod: *physx_PxShape_Pod, findex: u32) -> physx_PxQueryCache_Pod #foreign physx_sys;

PxQueryFilterData_new :: () -> physx_PxQueryFilterData_Pod #foreign physx_sys;

PxQueryFilterData_new_1 :: (fd_pod: *physx_PxFilterData_Pod, f_pod: u16) -> physx_PxQueryFilterData_Pod #foreign physx_sys;

PxQueryFilterData_new_2 :: (f_pod: u16) -> physx_PxQueryFilterData_Pod #foreign physx_sys;

PxQueryFilterCallback_preFilter_mut :: (self__pod: *physx_PxQueryFilterCallback_Pod, filterData_pod: *physx_PxFilterData_Pod, shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod, queryFlags_pod: *u16) -> s32 #foreign physx_sys;

PxQueryFilterCallback_postFilter_mut :: (self__pod: *physx_PxQueryFilterCallback_Pod, filterData_pod: *physx_PxFilterData_Pod, hit_pod: *physx_PxQueryHit_Pod, shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod) -> s32 #foreign physx_sys;

PxQueryFilterCallback_delete :: (self__pod: *physx_PxQueryFilterCallback_Pod) -> void #foreign physx_sys;

PxRigidDynamic_setKinematicTarget_mut :: (self__pod: *physx_PxRigidDynamic_Pod, destination_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxRigidDynamic_getKinematicTarget :: (self__pod: *physx_PxRigidDynamic_Pod, target_pod: *physx_PxTransform_Pod) -> bool #foreign physx_sys;

PxRigidDynamic_isSleeping :: (self__pod: *physx_PxRigidDynamic_Pod) -> bool #foreign physx_sys;

PxRigidDynamic_setSleepThreshold_mut :: (self__pod: *physx_PxRigidDynamic_Pod, threshold: float) -> void #foreign physx_sys;

PxRigidDynamic_getSleepThreshold :: (self__pod: *physx_PxRigidDynamic_Pod) -> float #foreign physx_sys;

PxRigidDynamic_setStabilizationThreshold_mut :: (self__pod: *physx_PxRigidDynamic_Pod, threshold: float) -> void #foreign physx_sys;

PxRigidDynamic_getStabilizationThreshold :: (self__pod: *physx_PxRigidDynamic_Pod) -> float #foreign physx_sys;

PxRigidDynamic_getRigidDynamicLockFlags :: (self__pod: *physx_PxRigidDynamic_Pod) -> u8 #foreign physx_sys;

PxRigidDynamic_setRigidDynamicLockFlag_mut :: (self__pod: *physx_PxRigidDynamic_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxRigidDynamic_setRigidDynamicLockFlags_mut :: (self__pod: *physx_PxRigidDynamic_Pod, flags_pod: u8) -> void #foreign physx_sys;

PxRigidDynamic_getLinearVelocity :: (self__pod: *physx_PxRigidDynamic_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidDynamic_setLinearVelocity_mut :: (self__pod: *physx_PxRigidDynamic_Pod, linVel_pod: *physx_PxVec3_Pod, autowake: bool) -> void #foreign physx_sys;

PxRigidDynamic_getAngularVelocity :: (self__pod: *physx_PxRigidDynamic_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidDynamic_setAngularVelocity_mut :: (self__pod: *physx_PxRigidDynamic_Pod, angVel_pod: *physx_PxVec3_Pod, autowake: bool) -> void #foreign physx_sys;

PxRigidDynamic_setWakeCounter_mut :: (self__pod: *physx_PxRigidDynamic_Pod, wakeCounterValue: float) -> void #foreign physx_sys;

PxRigidDynamic_getWakeCounter :: (self__pod: *physx_PxRigidDynamic_Pod) -> float #foreign physx_sys;

PxRigidDynamic_wakeUp_mut :: (self__pod: *physx_PxRigidDynamic_Pod) -> void #foreign physx_sys;

PxRigidDynamic_putToSleep_mut :: (self__pod: *physx_PxRigidDynamic_Pod) -> void #foreign physx_sys;

PxRigidDynamic_setSolverIterationCounts_mut :: (self__pod: *physx_PxRigidDynamic_Pod, minPositionIters: u32, minVelocityIters: u32) -> void #foreign physx_sys;

PxRigidDynamic_getSolverIterationCounts :: (self__pod: *physx_PxRigidDynamic_Pod, minPositionIters_pod: *u32, minVelocityIters_pod: *u32) -> void #foreign physx_sys;

PxRigidDynamic_getContactReportThreshold :: (self__pod: *physx_PxRigidDynamic_Pod) -> float #foreign physx_sys;

PxRigidDynamic_setContactReportThreshold_mut :: (self__pod: *physx_PxRigidDynamic_Pod, threshold: float) -> void #foreign physx_sys;

PxRigidDynamic_getConcreteTypeName :: (self__pod: *physx_PxRigidDynamic_Pod) -> *u8 #foreign physx_sys;

PxRigidStatic_getConcreteTypeName :: (self__pod: *physx_PxRigidStatic_Pod) -> *u8 #foreign physx_sys;

PxSceneQueryDesc_new :: () -> physx_PxSceneQueryDesc_Pod #foreign physx_sys;

PxSceneQueryDesc_setToDefault_mut :: (self__pod: *physx_PxSceneQueryDesc_Pod) -> void #foreign physx_sys;

PxSceneQueryDesc_isValid :: (self__pod: *physx_PxSceneQueryDesc_Pod) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut :: (self__pod: *physx_PxSceneQuerySystemBase_Pod, dynamicTreeRebuildRateHint: u32) -> void #foreign physx_sys;

PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint :: (self__pod: *physx_PxSceneQuerySystemBase_Pod) -> u32 #foreign physx_sys;

PxSceneQuerySystemBase_forceRebuildDynamicTree_mut :: (self__pod: *physx_PxSceneQuerySystemBase_Pod, prunerIndex: u32) -> void #foreign physx_sys;

PxSceneQuerySystemBase_setUpdateMode_mut :: (self__pod: *physx_PxSceneQuerySystemBase_Pod, updateMode_pod: s32) -> void #foreign physx_sys;

PxSceneQuerySystemBase_getUpdateMode :: (self__pod: *physx_PxSceneQuerySystemBase_Pod) -> s32 #foreign physx_sys;

PxSceneQuerySystemBase_getStaticTimestamp :: (self__pod: *physx_PxSceneQuerySystemBase_Pod) -> u32 #foreign physx_sys;

PxSceneQuerySystemBase_flushUpdates_mut :: (self__pod: *physx_PxSceneQuerySystemBase_Pod) -> void #foreign physx_sys;

PxSceneQuerySystemBase_raycast :: (self__pod: *physx_PxSceneQuerySystemBase_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, hitCall_pod: *physx_PxRaycastCallback_Pod, hitFlags_pod: u16, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_sweep :: (self__pod: *physx_PxSceneQuerySystemBase_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, hitCall_pod: *physx_PxSweepCallback_Pod, hitFlags_pod: u16, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxSceneQuerySystemBase_overlap :: (self__pod: *physx_PxSceneQuerySystemBase_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, hitCall_pod: *physx_PxOverlapCallback_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, queryFlags_pod: u32) -> bool #foreign physx_sys;

PxSceneSQSystem_setSceneQueryUpdateMode_mut :: (self__pod: *physx_PxSceneSQSystem_Pod, updateMode_pod: s32) -> void #foreign physx_sys;

PxSceneSQSystem_getSceneQueryUpdateMode :: (self__pod: *physx_PxSceneSQSystem_Pod) -> s32 #foreign physx_sys;

PxSceneSQSystem_getSceneQueryStaticTimestamp :: (self__pod: *physx_PxSceneSQSystem_Pod) -> u32 #foreign physx_sys;

PxSceneSQSystem_flushQueryUpdates_mut :: (self__pod: *physx_PxSceneSQSystem_Pod) -> void #foreign physx_sys;

PxSceneSQSystem_forceDynamicTreeRebuild_mut :: (self__pod: *physx_PxSceneSQSystem_Pod, rebuildStaticStructure: bool, rebuildDynamicStructure: bool) -> void #foreign physx_sys;

PxSceneSQSystem_getStaticStructure :: (self__pod: *physx_PxSceneSQSystem_Pod) -> s32 #foreign physx_sys;

PxSceneSQSystem_getDynamicStructure :: (self__pod: *physx_PxSceneSQSystem_Pod) -> s32 #foreign physx_sys;

PxSceneSQSystem_sceneQueriesUpdate_mut :: (self__pod: *physx_PxSceneSQSystem_Pod, completionTask_pod: *physx_PxBaseTask_Pod, controlSimulation: bool) -> void #foreign physx_sys;

PxSceneSQSystem_checkQueries_mut :: (self__pod: *physx_PxSceneSQSystem_Pod, block: bool) -> bool #foreign physx_sys;

PxSceneSQSystem_fetchQueries_mut :: (self__pod: *physx_PxSceneSQSystem_Pod, block: bool) -> bool #foreign physx_sys;

PxSceneQuerySystem_release_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_acquireReference_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_preallocate_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, prunerIndex: u32, nbShapes: u32) -> void #foreign physx_sys;

PxSceneQuerySystem_flushMemory_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_addSQShape_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, actor_pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod, bounds_pod: *physx_PxBounds3_Pod, transform_pod: *physx_PxTransform_Pod, compoundHandle: *u32, hasPruningStructure: bool) -> void #foreign physx_sys;

PxSceneQuerySystem_removeSQShape_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, actor_pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_updateSQShape_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, actor_pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod, transform_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_addSQCompound_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, actor_pod: *physx_PxRigidActor_Pod, shapes_pod: **physx_PxShape_Pod, bvh_pod: *physx_PxBVH_Pod, transforms_pod: *physx_PxTransform_Pod) -> u32 #foreign physx_sys;

PxSceneQuerySystem_removeSQCompound_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, compoundHandle: u32) -> void #foreign physx_sys;

PxSceneQuerySystem_updateSQCompound_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, compoundHandle: u32, compoundTransform_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_shiftOrigin_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, shift_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_merge_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, pruningStructure_pod: *physx_PxPruningStructure_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_getHandle :: (self__pod: *physx_PxSceneQuerySystem_Pod, actor_pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod, prunerIndex_pod: *u32) -> u32 #foreign physx_sys;

PxSceneQuerySystem_sync_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, prunerIndex: u32, handles: *u32, indices: *u32, bounds_pod: *physx_PxBounds3_Pod, transforms_pod: *physx_PxTransformPadded_Pod, count: u32, ignoredIndices_pod: *physx_PxBitMap_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_finalizeUpdates_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod) -> void #foreign physx_sys;

PxSceneQuerySystem_prepareSceneQueryBuildStep_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, prunerIndex: u32) -> *void #foreign physx_sys;

PxSceneQuerySystem_sceneQueryBuildStep_mut :: (self__pod: *physx_PxSceneQuerySystem_Pod, handle: *void) -> void #foreign physx_sys;

PxBroadPhaseDesc_new :: (type_pod: s32) -> physx_PxBroadPhaseDesc_Pod #foreign physx_sys;

PxBroadPhaseDesc_isValid :: (self__pod: *physx_PxBroadPhaseDesc_Pod) -> bool #foreign physx_sys;

phys_PxGetBroadPhaseStaticFilterGroup :: () -> u32 #foreign physx_sys;

phys_PxGetBroadPhaseDynamicFilterGroup :: (id: u32) -> u32 #foreign physx_sys;

phys_PxGetBroadPhaseKinematicFilterGroup :: (id: u32) -> u32 #foreign physx_sys;

PxBroadPhaseUpdateData_new :: (created: *u32, nbCreated: u32, updated: *u32, nbUpdated: u32, removed: *u32, nbRemoved: u32, bounds_pod: *physx_PxBounds3_Pod, groups: *u32, distances: *float, capacity: u32) -> physx_PxBroadPhaseUpdateData_Pod #foreign physx_sys;

PxBroadPhaseResults_new :: () -> physx_PxBroadPhaseResults_Pod #foreign physx_sys;

PxBroadPhaseRegions_getNbRegions :: (self__pod: *physx_PxBroadPhaseRegions_Pod) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_getRegions :: (self__pod: *physx_PxBroadPhaseRegions_Pod, userBuffer_pod: *physx_PxBroadPhaseRegionInfo_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_addRegion_mut :: (self__pod: *physx_PxBroadPhaseRegions_Pod, region_pod: *physx_PxBroadPhaseRegion_Pod, populateRegion: bool, bounds_pod: *physx_PxBounds3_Pod, distances: *float) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_removeRegion_mut :: (self__pod: *physx_PxBroadPhaseRegions_Pod, handle: u32) -> bool #foreign physx_sys;

PxBroadPhaseRegions_getNbOutOfBoundsObjects :: (self__pod: *physx_PxBroadPhaseRegions_Pod) -> u32 #foreign physx_sys;

PxBroadPhaseRegions_getOutOfBoundsObjects :: (self__pod: *physx_PxBroadPhaseRegions_Pod) -> *u32 #foreign physx_sys;

PxBroadPhase_release_mut :: (self__pod: *physx_PxBroadPhase_Pod) -> void #foreign physx_sys;

PxBroadPhase_getType :: (self__pod: *physx_PxBroadPhase_Pod) -> s32 #foreign physx_sys;

PxBroadPhase_getCaps :: (self__pod: *physx_PxBroadPhase_Pod, caps_pod: *physx_PxBroadPhaseCaps_Pod) -> void #foreign physx_sys;

PxBroadPhase_getRegions_mut :: (self__pod: *physx_PxBroadPhase_Pod) -> *physx_PxBroadPhaseRegions_Pod #foreign physx_sys;

PxBroadPhase_getAllocator_mut :: (self__pod: *physx_PxBroadPhase_Pod) -> *physx_PxAllocatorCallback_Pod #foreign physx_sys;

PxBroadPhase_getContextID :: (self__pod: *physx_PxBroadPhase_Pod) -> u64 #foreign physx_sys;

PxBroadPhase_setScratchBlock_mut :: (self__pod: *physx_PxBroadPhase_Pod, scratchBlock: *void, size: u32) -> void #foreign physx_sys;

PxBroadPhase_update_mut :: (self__pod: *physx_PxBroadPhase_Pod, updateData_pod: *physx_PxBroadPhaseUpdateData_Pod, continuation_pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxBroadPhase_fetchResults_mut :: (self__pod: *physx_PxBroadPhase_Pod, results_pod: *physx_PxBroadPhaseResults_Pod) -> void #foreign physx_sys;

PxBroadPhase_update_mut_1 :: (self__pod: *physx_PxBroadPhase_Pod, results_pod: *physx_PxBroadPhaseResults_Pod, updateData_pod: *physx_PxBroadPhaseUpdateData_Pod) -> void #foreign physx_sys;

phys_PxCreateBroadPhase :: (desc_pod: *physx_PxBroadPhaseDesc_Pod) -> *physx_PxBroadPhase_Pod #foreign physx_sys;

PxAABBManager_release_mut :: (self__pod: *physx_PxAABBManager_Pod) -> void #foreign physx_sys;

PxAABBManager_getBroadPhase_mut :: (self__pod: *physx_PxAABBManager_Pod) -> *physx_PxBroadPhase_Pod #foreign physx_sys;

PxAABBManager_getBounds :: (self__pod: *physx_PxAABBManager_Pod) -> *physx_PxBounds3_Pod #foreign physx_sys;

PxAABBManager_getDistances :: (self__pod: *physx_PxAABBManager_Pod) -> *float #foreign physx_sys;

PxAABBManager_getGroups :: (self__pod: *physx_PxAABBManager_Pod) -> *u32 #foreign physx_sys;

PxAABBManager_getCapacity :: (self__pod: *physx_PxAABBManager_Pod) -> u32 #foreign physx_sys;

PxAABBManager_addObject_mut :: (self__pod: *physx_PxAABBManager_Pod, index: u32, bounds_pod: *physx_PxBounds3_Pod, group: u32, distance: float) -> void #foreign physx_sys;

PxAABBManager_removeObject_mut :: (self__pod: *physx_PxAABBManager_Pod, index: u32) -> void #foreign physx_sys;

PxAABBManager_updateObject_mut :: (self__pod: *physx_PxAABBManager_Pod, index: u32, bounds_pod: *physx_PxBounds3_Pod, distance: *float) -> void #foreign physx_sys;

PxAABBManager_update_mut :: (self__pod: *physx_PxAABBManager_Pod, continuation_pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxAABBManager_fetchResults_mut :: (self__pod: *physx_PxAABBManager_Pod, results_pod: *physx_PxBroadPhaseResults_Pod) -> void #foreign physx_sys;

PxAABBManager_update_mut_1 :: (self__pod: *physx_PxAABBManager_Pod, results_pod: *physx_PxBroadPhaseResults_Pod) -> void #foreign physx_sys;

phys_PxCreateAABBManager :: (broadphase_pod: *physx_PxBroadPhase_Pod) -> *physx_PxAABBManager_Pod #foreign physx_sys;

PxSceneLimits_new :: () -> physx_PxSceneLimits_Pod #foreign physx_sys;

PxSceneLimits_setToDefault_mut :: (self__pod: *physx_PxSceneLimits_Pod) -> void #foreign physx_sys;

PxSceneLimits_isValid :: (self__pod: *physx_PxSceneLimits_Pod) -> bool #foreign physx_sys;

PxgDynamicsMemoryConfig_new :: () -> physx_PxgDynamicsMemoryConfig_Pod #foreign physx_sys;

PxgDynamicsMemoryConfig_isValid :: (self__pod: *physx_PxgDynamicsMemoryConfig_Pod) -> bool #foreign physx_sys;

PxSceneDesc_new :: (scale_pod: *physx_PxTolerancesScale_Pod) -> physx_PxSceneDesc_Pod #foreign physx_sys;

PxSceneDesc_setToDefault_mut :: (self__pod: *physx_PxSceneDesc_Pod, scale_pod: *physx_PxTolerancesScale_Pod) -> void #foreign physx_sys;

PxSceneDesc_isValid :: (self__pod: *physx_PxSceneDesc_Pod) -> bool #foreign physx_sys;

PxSceneDesc_getTolerancesScale :: (self__pod: *physx_PxSceneDesc_Pod) -> *physx_PxTolerancesScale_Pod #foreign physx_sys;

PxSimulationStatistics_getNbBroadPhaseAdds :: (self__pod: *physx_PxSimulationStatistics_Pod) -> u32 #foreign physx_sys;

PxSimulationStatistics_getNbBroadPhaseRemoves :: (self__pod: *physx_PxSimulationStatistics_Pod) -> u32 #foreign physx_sys;

PxSimulationStatistics_getRbPairStats :: (self__pod: *physx_PxSimulationStatistics_Pod, pairType_pod: s32, g0_pod: s32, g1_pod: s32) -> u32 #foreign physx_sys;

PxSimulationStatistics_new :: () -> physx_PxSimulationStatistics_Pod #foreign physx_sys;

PxPvdSceneClient_setScenePvdFlag_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxPvdSceneClient_setScenePvdFlags_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, flags_pod: u8) -> void #foreign physx_sys;

PxPvdSceneClient_getScenePvdFlags :: (self__pod: *physx_PxPvdSceneClient_Pod) -> u8 #foreign physx_sys;

PxPvdSceneClient_updateCamera_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, name: *u8, origin_pod: *physx_PxVec3_Pod, up_pod: *physx_PxVec3_Pod, target_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxPvdSceneClient_drawPoints_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, points_pod: *physx_PxDebugPoint_Pod, count: u32) -> void #foreign physx_sys;

PxPvdSceneClient_drawLines_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, lines_pod: *physx_PxDebugLine_Pod, count: u32) -> void #foreign physx_sys;

PxPvdSceneClient_drawTriangles_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, triangles_pod: *physx_PxDebugTriangle_Pod, count: u32) -> void #foreign physx_sys;

PxPvdSceneClient_drawText_mut :: (self__pod: *physx_PxPvdSceneClient_Pod, text_pod: *physx_PxDebugText_Pod) -> void #foreign physx_sys;

PxDominanceGroupPair_new :: (a: u8, b: u8) -> physx_PxDominanceGroupPair_Pod #foreign physx_sys;

PxBroadPhaseCallback_delete :: (self__pod: *physx_PxBroadPhaseCallback_Pod) -> void #foreign physx_sys;

PxBroadPhaseCallback_onObjectOutOfBounds_mut :: (self__pod: *physx_PxBroadPhaseCallback_Pod, shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxActor_Pod) -> void #foreign physx_sys;

PxBroadPhaseCallback_onObjectOutOfBounds_mut_1 :: (self__pod: *physx_PxBroadPhaseCallback_Pod, aggregate_pod: *physx_PxAggregate_Pod) -> void #foreign physx_sys;

PxScene_release_mut :: (self__pod: *physx_PxScene_Pod) -> void #foreign physx_sys;

PxScene_setFlag_mut :: (self__pod: *physx_PxScene_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxScene_getFlags :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_setLimits_mut :: (self__pod: *physx_PxScene_Pod, limits_pod: *physx_PxSceneLimits_Pod) -> void #foreign physx_sys;

PxScene_getLimits :: (self__pod: *physx_PxScene_Pod) -> physx_PxSceneLimits_Pod #foreign physx_sys;

PxScene_getPhysics_mut :: (self__pod: *physx_PxScene_Pod) -> *physx_PxPhysics_Pod #foreign physx_sys;

PxScene_getTimestamp :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_addArticulation_mut :: (self__pod: *physx_PxScene_Pod, articulation_pod: *physx_PxArticulationReducedCoordinate_Pod) -> bool #foreign physx_sys;

PxScene_removeArticulation_mut :: (self__pod: *physx_PxScene_Pod, articulation_pod: *physx_PxArticulationReducedCoordinate_Pod, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_addActor_mut :: (self__pod: *physx_PxScene_Pod, actor_pod: *physx_PxActor_Pod, bvh_pod: *physx_PxBVH_Pod) -> bool #foreign physx_sys;

PxScene_addActors_mut :: (self__pod: *physx_PxScene_Pod, actors_pod: **physx_PxActor_Pod, nbActors: u32) -> bool #foreign physx_sys;

PxScene_addActors_mut_1 :: (self__pod: *physx_PxScene_Pod, pruningStructure_pod: *physx_PxPruningStructure_Pod) -> bool #foreign physx_sys;

PxScene_removeActor_mut :: (self__pod: *physx_PxScene_Pod, actor_pod: *physx_PxActor_Pod, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_removeActors_mut :: (self__pod: *physx_PxScene_Pod, actors_pod: **physx_PxActor_Pod, nbActors: u32, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_addAggregate_mut :: (self__pod: *physx_PxScene_Pod, aggregate_pod: *physx_PxAggregate_Pod) -> bool #foreign physx_sys;

PxScene_removeAggregate_mut :: (self__pod: *physx_PxScene_Pod, aggregate_pod: *physx_PxAggregate_Pod, wakeOnLostTouch: bool) -> void #foreign physx_sys;

PxScene_addCollection_mut :: (self__pod: *physx_PxScene_Pod, collection_pod: *physx_PxCollection_Pod) -> bool #foreign physx_sys;

PxScene_getNbActors :: (self__pod: *physx_PxScene_Pod, types_pod: u16) -> u32 #foreign physx_sys;

PxScene_getActors :: (self__pod: *physx_PxScene_Pod, types_pod: u16, userBuffer_pod: **physx_PxActor_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getActiveActors_mut :: (self__pod: *physx_PxScene_Pod, nbActorsOut_pod: *u32) -> **physx_PxActor_Pod #foreign physx_sys;

PxScene_getNbArticulations :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getArticulations :: (self__pod: *physx_PxScene_Pod, userBuffer_pod: **physx_PxArticulationReducedCoordinate_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getNbConstraints :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getConstraints :: (self__pod: *physx_PxScene_Pod, userBuffer_pod: **physx_PxConstraint_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_getNbAggregates :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getAggregates :: (self__pod: *physx_PxScene_Pod, userBuffer_pod: **physx_PxAggregate_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_setDominanceGroupPair_mut :: (self__pod: *physx_PxScene_Pod, group1: u8, group2: u8, dominance_pod: *physx_PxDominanceGroupPair_Pod) -> void #foreign physx_sys;

PxScene_getDominanceGroupPair :: (self__pod: *physx_PxScene_Pod, group1: u8, group2: u8) -> physx_PxDominanceGroupPair_Pod #foreign physx_sys;

PxScene_getCpuDispatcher :: (self__pod: *physx_PxScene_Pod) -> *physx_PxCpuDispatcher_Pod #foreign physx_sys;

PxScene_createClient_mut :: (self__pod: *physx_PxScene_Pod) -> u8 #foreign physx_sys;

PxScene_setSimulationEventCallback_mut :: (self__pod: *physx_PxScene_Pod, callback_pod: *physx_PxSimulationEventCallback_Pod) -> void #foreign physx_sys;

PxScene_getSimulationEventCallback :: (self__pod: *physx_PxScene_Pod) -> *physx_PxSimulationEventCallback_Pod #foreign physx_sys;

PxScene_setContactModifyCallback_mut :: (self__pod: *physx_PxScene_Pod, callback_pod: *physx_PxContactModifyCallback_Pod) -> void #foreign physx_sys;

PxScene_setCCDContactModifyCallback_mut :: (self__pod: *physx_PxScene_Pod, callback_pod: *physx_PxCCDContactModifyCallback_Pod) -> void #foreign physx_sys;

PxScene_getContactModifyCallback :: (self__pod: *physx_PxScene_Pod) -> *physx_PxContactModifyCallback_Pod #foreign physx_sys;

PxScene_getCCDContactModifyCallback :: (self__pod: *physx_PxScene_Pod) -> *physx_PxCCDContactModifyCallback_Pod #foreign physx_sys;

PxScene_setBroadPhaseCallback_mut :: (self__pod: *physx_PxScene_Pod, callback_pod: *physx_PxBroadPhaseCallback_Pod) -> void #foreign physx_sys;

PxScene_getBroadPhaseCallback :: (self__pod: *physx_PxScene_Pod) -> *physx_PxBroadPhaseCallback_Pod #foreign physx_sys;

PxScene_setFilterShaderData_mut :: (self__pod: *physx_PxScene_Pod, data: *void, dataSize: u32) -> void #foreign physx_sys;

PxScene_getFilterShaderData :: (self__pod: *physx_PxScene_Pod) -> *void #foreign physx_sys;

PxScene_getFilterShaderDataSize :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_resetFiltering_mut :: (self__pod: *physx_PxScene_Pod, actor_pod: *physx_PxActor_Pod) -> bool #foreign physx_sys;

PxScene_resetFiltering_mut_1 :: (self__pod: *physx_PxScene_Pod, actor_pod: *physx_PxRigidActor_Pod, shapes_pod: **physx_PxShape_Pod, shapeCount: u32) -> bool #foreign physx_sys;

PxScene_getKinematicKinematicFilteringMode :: (self__pod: *physx_PxScene_Pod) -> s32 #foreign physx_sys;

PxScene_getStaticKinematicFilteringMode :: (self__pod: *physx_PxScene_Pod) -> s32 #foreign physx_sys;

PxScene_simulate_mut :: (self__pod: *physx_PxScene_Pod, elapsedTime: float, completionTask_pod: *physx_PxBaseTask_Pod, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign physx_sys;

PxScene_advance_mut :: (self__pod: *physx_PxScene_Pod, completionTask_pod: *physx_PxBaseTask_Pod) -> bool #foreign physx_sys;

PxScene_collide_mut :: (self__pod: *physx_PxScene_Pod, elapsedTime: float, completionTask_pod: *physx_PxBaseTask_Pod, scratchMemBlock: *void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool #foreign physx_sys;

PxScene_checkResults_mut :: (self__pod: *physx_PxScene_Pod, block: bool) -> bool #foreign physx_sys;

PxScene_fetchCollision_mut :: (self__pod: *physx_PxScene_Pod, block: bool) -> bool #foreign physx_sys;

PxScene_fetchResults_mut :: (self__pod: *physx_PxScene_Pod, block: bool, errorState: *u32) -> bool #foreign physx_sys;

PxScene_fetchResultsStart_mut :: (self__pod: *physx_PxScene_Pod, contactPairs_pod: **physx_PxContactPairHeader_Pod, nbContactPairs_pod: *u32, block: bool) -> bool #foreign physx_sys;

PxScene_processCallbacks_mut :: (self__pod: *physx_PxScene_Pod, continuation_pod: *physx_PxBaseTask_Pod) -> void #foreign physx_sys;

PxScene_fetchResultsFinish_mut :: (self__pod: *physx_PxScene_Pod, errorState: *u32) -> void #foreign physx_sys;

PxScene_fetchResultsParticleSystem_mut :: (self__pod: *physx_PxScene_Pod) -> void #foreign physx_sys;

PxScene_flushSimulation_mut :: (self__pod: *physx_PxScene_Pod, sendPendingReports: bool) -> void #foreign physx_sys;

PxScene_setGravity_mut :: (self__pod: *physx_PxScene_Pod, vec_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxScene_getGravity :: (self__pod: *physx_PxScene_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxScene_setBounceThresholdVelocity_mut :: (self__pod: *physx_PxScene_Pod, t: float) -> void #foreign physx_sys;

PxScene_getBounceThresholdVelocity :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_setCCDMaxPasses_mut :: (self__pod: *physx_PxScene_Pod, ccdMaxPasses: u32) -> void #foreign physx_sys;

PxScene_getCCDMaxPasses :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_setCCDMaxSeparation_mut :: (self__pod: *physx_PxScene_Pod, t: float) -> void #foreign physx_sys;

PxScene_getCCDMaxSeparation :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_setCCDThreshold_mut :: (self__pod: *physx_PxScene_Pod, t: float) -> void #foreign physx_sys;

PxScene_getCCDThreshold :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_setMaxBiasCoefficient_mut :: (self__pod: *physx_PxScene_Pod, t: float) -> void #foreign physx_sys;

PxScene_getMaxBiasCoefficient :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_setFrictionOffsetThreshold_mut :: (self__pod: *physx_PxScene_Pod, t: float) -> void #foreign physx_sys;

PxScene_getFrictionOffsetThreshold :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_setFrictionCorrelationDistance_mut :: (self__pod: *physx_PxScene_Pod, t: float) -> void #foreign physx_sys;

PxScene_getFrictionCorrelationDistance :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_getFrictionType :: (self__pod: *physx_PxScene_Pod) -> s32 #foreign physx_sys;

PxScene_getSolverType :: (self__pod: *physx_PxScene_Pod) -> s32 #foreign physx_sys;

PxScene_setVisualizationParameter_mut :: (self__pod: *physx_PxScene_Pod, param_pod: s32, value: float) -> bool #foreign physx_sys;

PxScene_getVisualizationParameter :: (self__pod: *physx_PxScene_Pod, paramEnum_pod: s32) -> float #foreign physx_sys;

PxScene_setVisualizationCullingBox_mut :: (self__pod: *physx_PxScene_Pod, box_pod: *physx_PxBounds3_Pod) -> void #foreign physx_sys;

PxScene_getVisualizationCullingBox :: (self__pod: *physx_PxScene_Pod) -> physx_PxBounds3_Pod #foreign physx_sys;

PxScene_getRenderBuffer_mut :: (self__pod: *physx_PxScene_Pod) -> *physx_PxRenderBuffer_Pod #foreign physx_sys;

PxScene_getSimulationStatistics :: (self__pod: *physx_PxScene_Pod, stats_pod: *physx_PxSimulationStatistics_Pod) -> void #foreign physx_sys;

PxScene_getBroadPhaseType :: (self__pod: *physx_PxScene_Pod) -> s32 #foreign physx_sys;

PxScene_getBroadPhaseCaps :: (self__pod: *physx_PxScene_Pod, caps_pod: *physx_PxBroadPhaseCaps_Pod) -> bool #foreign physx_sys;

PxScene_getNbBroadPhaseRegions :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getBroadPhaseRegions :: (self__pod: *physx_PxScene_Pod, userBuffer_pod: *physx_PxBroadPhaseRegionInfo_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxScene_addBroadPhaseRegion_mut :: (self__pod: *physx_PxScene_Pod, region_pod: *physx_PxBroadPhaseRegion_Pod, populateRegion: bool) -> u32 #foreign physx_sys;

PxScene_removeBroadPhaseRegion_mut :: (self__pod: *physx_PxScene_Pod, handle: u32) -> bool #foreign physx_sys;

PxScene_getTaskManager :: (self__pod: *physx_PxScene_Pod) -> *physx_PxTaskManager_Pod #foreign physx_sys;

PxScene_lockRead_mut :: (self__pod: *physx_PxScene_Pod, file: *u8, line: u32) -> void #foreign physx_sys;

PxScene_unlockRead_mut :: (self__pod: *physx_PxScene_Pod) -> void #foreign physx_sys;

PxScene_lockWrite_mut :: (self__pod: *physx_PxScene_Pod, file: *u8, line: u32) -> void #foreign physx_sys;

PxScene_unlockWrite_mut :: (self__pod: *physx_PxScene_Pod) -> void #foreign physx_sys;

PxScene_setNbContactDataBlocks_mut :: (self__pod: *physx_PxScene_Pod, numBlocks: u32) -> void #foreign physx_sys;

PxScene_getNbContactDataBlocksUsed :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getMaxNbContactDataBlocksUsed :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getContactReportStreamBufferSize :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_setSolverBatchSize_mut :: (self__pod: *physx_PxScene_Pod, solverBatchSize: u32) -> void #foreign physx_sys;

PxScene_getSolverBatchSize :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_setSolverArticulationBatchSize_mut :: (self__pod: *physx_PxScene_Pod, solverBatchSize: u32) -> void #foreign physx_sys;

PxScene_getSolverArticulationBatchSize :: (self__pod: *physx_PxScene_Pod) -> u32 #foreign physx_sys;

PxScene_getWakeCounterResetValue :: (self__pod: *physx_PxScene_Pod) -> float #foreign physx_sys;

PxScene_shiftOrigin_mut :: (self__pod: *physx_PxScene_Pod, shift_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxScene_getScenePvdClient_mut :: (self__pod: *physx_PxScene_Pod) -> *physx_PxPvdSceneClient_Pod #foreign physx_sys;

PxScene_copyArticulationData_mut :: (self__pod: *physx_PxScene_Pod, data: *void, index: *void, dataType_pod: s32, nbCopyArticulations: u32, copyEvent: *void) -> void #foreign physx_sys;

PxScene_applyArticulationData_mut :: (self__pod: *physx_PxScene_Pod, data: *void, index: *void, dataType_pod: s32, nbUpdatedArticulations: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys;

PxScene_copySoftBodyData_mut :: (self__pod: *physx_PxScene_Pod, data: **void, dataSizes: *void, softBodyIndices: *void, flag_pod: s32, nbCopySoftBodies: u32, maxSize: u32, copyEvent: *void) -> void #foreign physx_sys;

PxScene_applySoftBodyData_mut :: (self__pod: *physx_PxScene_Pod, data: **void, dataSizes: *void, softBodyIndices: *void, flag_pod: s32, nbUpdatedSoftBodies: u32, maxSize: u32, applyEvent: *void) -> void #foreign physx_sys;

PxScene_copyContactData_mut :: (self__pod: *physx_PxScene_Pod, data: *void, maxContactPairs: u32, numContactPairs: *void, copyEvent: *void) -> void #foreign physx_sys;

PxScene_copyBodyData_mut :: (self__pod: *physx_PxScene_Pod, data_pod: *physx_PxGpuBodyData_Pod, index_pod: *physx_PxGpuActorPair_Pod, nbCopyActors: u32, copyEvent: *void) -> void #foreign physx_sys;

PxScene_applyActorData_mut :: (self__pod: *physx_PxScene_Pod, data: *void, index_pod: *physx_PxGpuActorPair_Pod, flag_pod: s32, nbUpdatedActors: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys;

PxScene_computeDenseJacobians_mut :: (self__pod: *physx_PxScene_Pod, indices_pod: *physx_PxIndexDataPair_Pod, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_computeGeneralizedMassMatrices_mut :: (self__pod: *physx_PxScene_Pod, indices_pod: *physx_PxIndexDataPair_Pod, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_computeGeneralizedGravityForces_mut :: (self__pod: *physx_PxScene_Pod, indices_pod: *physx_PxIndexDataPair_Pod, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_computeCoriolisAndCentrifugalForces_mut :: (self__pod: *physx_PxScene_Pod, indices_pod: *physx_PxIndexDataPair_Pod, nbIndices: u32, computeEvent: *void) -> void #foreign physx_sys;

PxScene_getGpuDynamicsConfig :: (self__pod: *physx_PxScene_Pod) -> physx_PxgDynamicsMemoryConfig_Pod #foreign physx_sys;

PxScene_applyParticleBufferData_mut :: (self__pod: *physx_PxScene_Pod, indices: *u32, bufferIndexPair_pod: *physx_PxGpuParticleBufferIndexPair_Pod, flags_pod: *u32, nbUpdatedBuffers: u32, waitEvent: *void, signalEvent: *void) -> void #foreign physx_sys;

PxSceneReadLock_new_alloc :: (scene_pod: *physx_PxScene_Pod, file: *u8, line: u32) -> *physx_PxSceneReadLock_Pod #foreign physx_sys;

PxSceneReadLock_delete :: (self__pod: *physx_PxSceneReadLock_Pod) -> void #foreign physx_sys;

PxSceneWriteLock_new_alloc :: (scene_pod: *physx_PxScene_Pod, file: *u8, line: u32) -> *physx_PxSceneWriteLock_Pod #foreign physx_sys;

PxSceneWriteLock_delete :: (self__pod: *physx_PxSceneWriteLock_Pod) -> void #foreign physx_sys;

PxContactPairExtraDataItem_new :: () -> physx_PxContactPairExtraDataItem_Pod #foreign physx_sys;

PxContactPairVelocity_new :: () -> physx_PxContactPairVelocity_Pod #foreign physx_sys;

PxContactPairPose_new :: () -> physx_PxContactPairPose_Pod #foreign physx_sys;

PxContactPairIndex_new :: () -> physx_PxContactPairIndex_Pod #foreign physx_sys;

PxContactPairExtraDataIterator_new :: (stream: *u8, size: u32) -> physx_PxContactPairExtraDataIterator_Pod #foreign physx_sys;

PxContactPairExtraDataIterator_nextItemSet_mut :: (self__pod: *physx_PxContactPairExtraDataIterator_Pod) -> bool #foreign physx_sys;

PxContactPairHeader_new :: () -> physx_PxContactPairHeader_Pod #foreign physx_sys;

PxContactPair_new :: () -> physx_PxContactPair_Pod #foreign physx_sys;

PxContactPair_extractContacts :: (self__pod: *physx_PxContactPair_Pod, userBuffer_pod: *physx_PxContactPairPoint_Pod, bufferSize: u32) -> u32 #foreign physx_sys;

PxContactPair_bufferContacts :: (self__pod: *physx_PxContactPair_Pod, newPair_pod: *physx_PxContactPair_Pod, bufferMemory: *u8) -> void #foreign physx_sys;

PxContactPair_getInternalFaceIndices :: (self__pod: *physx_PxContactPair_Pod) -> *u32 #foreign physx_sys;

PxTriggerPair_new :: () -> physx_PxTriggerPair_Pod #foreign physx_sys;

PxConstraintInfo_new :: () -> physx_PxConstraintInfo_Pod #foreign physx_sys;

PxConstraintInfo_new_1 :: (c_pod: *physx_PxConstraint_Pod, extRef: *void, t: u32) -> physx_PxConstraintInfo_Pod #foreign physx_sys;

PxSimulationEventCallback_onConstraintBreak_mut :: (self__pod: *physx_PxSimulationEventCallback_Pod, constraints_pod: *physx_PxConstraintInfo_Pod, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onWake_mut :: (self__pod: *physx_PxSimulationEventCallback_Pod, actors_pod: **physx_PxActor_Pod, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onSleep_mut :: (self__pod: *physx_PxSimulationEventCallback_Pod, actors_pod: **physx_PxActor_Pod, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onContact_mut :: (self__pod: *physx_PxSimulationEventCallback_Pod, pairHeader_pod: *physx_PxContactPairHeader_Pod, pairs_pod: *physx_PxContactPair_Pod, nbPairs: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onTrigger_mut :: (self__pod: *physx_PxSimulationEventCallback_Pod, pairs_pod: *physx_PxTriggerPair_Pod, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_onAdvance_mut :: (self__pod: *physx_PxSimulationEventCallback_Pod, bodyBuffer_pod: **physx_PxRigidBody_Pod, poseBuffer_pod: *physx_PxTransform_Pod, count: u32) -> void #foreign physx_sys;

PxSimulationEventCallback_delete :: (self__pod: *physx_PxSimulationEventCallback_Pod) -> void #foreign physx_sys;

PxFEMParameters_new :: () -> physx_PxFEMParameters_Pod #foreign physx_sys;

PxPruningStructure_release_mut :: (self__pod: *physx_PxPruningStructure_Pod) -> void #foreign physx_sys;

PxPruningStructure_getRigidActors :: (self__pod: *physx_PxPruningStructure_Pod, userBuffer_pod: **physx_PxRigidActor_Pod, bufferSize: u32, startIndex: u32) -> u32 #foreign physx_sys;

PxPruningStructure_getNbRigidActors :: (self__pod: *physx_PxPruningStructure_Pod) -> u32 #foreign physx_sys;

PxPruningStructure_getStaticMergeData :: (self__pod: *physx_PxPruningStructure_Pod) -> *void #foreign physx_sys;

PxPruningStructure_getDynamicMergeData :: (self__pod: *physx_PxPruningStructure_Pod) -> *void #foreign physx_sys;

PxPruningStructure_getConcreteTypeName :: (self__pod: *physx_PxPruningStructure_Pod) -> *u8 #foreign physx_sys;

PxExtendedVec3_new :: () -> physx_PxExtendedVec3_Pod #foreign physx_sys;

PxExtendedVec3_new_1 :: (_x: float64, _y: float64, _z: float64) -> physx_PxExtendedVec3_Pod #foreign physx_sys;

PxExtendedVec3_isZero :: (self__pod: *physx_PxExtendedVec3_Pod) -> bool #foreign physx_sys;

PxExtendedVec3_dot :: (self__pod: *physx_PxExtendedVec3_Pod, v_pod: *physx_PxVec3_Pod) -> float64 #foreign physx_sys;

PxExtendedVec3_distanceSquared :: (self__pod: *physx_PxExtendedVec3_Pod, v_pod: *physx_PxExtendedVec3_Pod) -> float64 #foreign physx_sys;

PxExtendedVec3_magnitudeSquared :: (self__pod: *physx_PxExtendedVec3_Pod) -> float64 #foreign physx_sys;

PxExtendedVec3_magnitude :: (self__pod: *physx_PxExtendedVec3_Pod) -> float64 #foreign physx_sys;

PxExtendedVec3_normalize_mut :: (self__pod: *physx_PxExtendedVec3_Pod) -> float64 #foreign physx_sys;

PxExtendedVec3_isFinite :: (self__pod: *physx_PxExtendedVec3_Pod) -> bool #foreign physx_sys;

PxExtendedVec3_maximum_mut :: (self__pod: *physx_PxExtendedVec3_Pod, v_pod: *physx_PxExtendedVec3_Pod) -> void #foreign physx_sys;

PxExtendedVec3_minimum_mut :: (self__pod: *physx_PxExtendedVec3_Pod, v_pod: *physx_PxExtendedVec3_Pod) -> void #foreign physx_sys;

PxExtendedVec3_set_mut :: (self__pod: *physx_PxExtendedVec3_Pod, x_: float64, y_: float64, z_: float64) -> void #foreign physx_sys;

PxExtendedVec3_setPlusInfinity_mut :: (self__pod: *physx_PxExtendedVec3_Pod) -> void #foreign physx_sys;

PxExtendedVec3_setMinusInfinity_mut :: (self__pod: *physx_PxExtendedVec3_Pod) -> void #foreign physx_sys;

PxExtendedVec3_cross_mut :: (self__pod: *physx_PxExtendedVec3_Pod, left_pod: *physx_PxExtendedVec3_Pod, right_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxExtendedVec3_cross_mut_1 :: (self__pod: *physx_PxExtendedVec3_Pod, left_pod: *physx_PxExtendedVec3_Pod, right_pod: *physx_PxExtendedVec3_Pod) -> void #foreign physx_sys;

PxExtendedVec3_cross :: (self__pod: *physx_PxExtendedVec3_Pod, v_pod: *physx_PxExtendedVec3_Pod) -> physx_PxExtendedVec3_Pod #foreign physx_sys;

PxExtendedVec3_cross_mut_2 :: (self__pod: *physx_PxExtendedVec3_Pod, left_pod: *physx_PxVec3_Pod, right_pod: *physx_PxExtendedVec3_Pod) -> void #foreign physx_sys;

phys_toVec3 :: (v_pod: *physx_PxExtendedVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxObstacle_getType :: (self__pod: *physx_PxObstacle_Pod) -> s32 #foreign physx_sys;

PxBoxObstacle_new :: () -> physx_PxBoxObstacle_Pod #foreign physx_sys;

PxCapsuleObstacle_new :: () -> physx_PxCapsuleObstacle_Pod #foreign physx_sys;

PxObstacleContext_release_mut :: (self__pod: *physx_PxObstacleContext_Pod) -> void #foreign physx_sys;

PxObstacleContext_getControllerManager :: (self__pod: *physx_PxObstacleContext_Pod) -> *physx_PxControllerManager_Pod #foreign physx_sys;

PxObstacleContext_addObstacle_mut :: (self__pod: *physx_PxObstacleContext_Pod, obstacle_pod: *physx_PxObstacle_Pod) -> u32 #foreign physx_sys;

PxObstacleContext_removeObstacle_mut :: (self__pod: *physx_PxObstacleContext_Pod, handle: u32) -> bool #foreign physx_sys;

PxObstacleContext_updateObstacle_mut :: (self__pod: *physx_PxObstacleContext_Pod, handle: u32, obstacle_pod: *physx_PxObstacle_Pod) -> bool #foreign physx_sys;

PxObstacleContext_getNbObstacles :: (self__pod: *physx_PxObstacleContext_Pod) -> u32 #foreign physx_sys;

PxObstacleContext_getObstacle :: (self__pod: *physx_PxObstacleContext_Pod, i: u32) -> *physx_PxObstacle_Pod #foreign physx_sys;

PxObstacleContext_getObstacleByHandle :: (self__pod: *physx_PxObstacleContext_Pod, handle: u32) -> *physx_PxObstacle_Pod #foreign physx_sys;

PxUserControllerHitReport_onShapeHit_mut :: (self__pod: *physx_PxUserControllerHitReport_Pod, hit_pod: *physx_PxControllerShapeHit_Pod) -> void #foreign physx_sys;

PxUserControllerHitReport_onControllerHit_mut :: (self__pod: *physx_PxUserControllerHitReport_Pod, hit_pod: *physx_PxControllersHit_Pod) -> void #foreign physx_sys;

PxUserControllerHitReport_onObstacleHit_mut :: (self__pod: *physx_PxUserControllerHitReport_Pod, hit_pod: *physx_PxControllerObstacleHit_Pod) -> void #foreign physx_sys;

PxControllerFilterCallback_delete :: (self__pod: *physx_PxControllerFilterCallback_Pod) -> void #foreign physx_sys;

PxControllerFilterCallback_filter_mut :: (self__pod: *physx_PxControllerFilterCallback_Pod, a_pod: *physx_PxController_Pod, b_pod: *physx_PxController_Pod) -> bool #foreign physx_sys;

PxControllerFilters_new :: (filterData_pod: *physx_PxFilterData_Pod, cb_pod: *physx_PxQueryFilterCallback_Pod, cctFilterCb_pod: *physx_PxControllerFilterCallback_Pod) -> physx_PxControllerFilters_Pod #foreign physx_sys;

PxControllerDesc_isValid :: (self__pod: *physx_PxControllerDesc_Pod) -> bool #foreign physx_sys;

PxControllerDesc_getType :: (self__pod: *physx_PxControllerDesc_Pod) -> s32 #foreign physx_sys;

PxController_getType :: (self__pod: *physx_PxController_Pod) -> s32 #foreign physx_sys;

PxController_release_mut :: (self__pod: *physx_PxController_Pod) -> void #foreign physx_sys;

PxController_move_mut :: (self__pod: *physx_PxController_Pod, disp_pod: *physx_PxVec3_Pod, minDist: float, elapsedTime: float, filters_pod: *physx_PxControllerFilters_Pod, obstacles_pod: *physx_PxObstacleContext_Pod) -> u8 #foreign physx_sys;

PxController_setPosition_mut :: (self__pod: *physx_PxController_Pod, position_pod: *physx_PxExtendedVec3_Pod) -> bool #foreign physx_sys;

PxController_getPosition :: (self__pod: *physx_PxController_Pod) -> *physx_PxExtendedVec3_Pod #foreign physx_sys;

PxController_setFootPosition_mut :: (self__pod: *physx_PxController_Pod, position_pod: *physx_PxExtendedVec3_Pod) -> bool #foreign physx_sys;

PxController_getFootPosition :: (self__pod: *physx_PxController_Pod) -> physx_PxExtendedVec3_Pod #foreign physx_sys;

PxController_getActor :: (self__pod: *physx_PxController_Pod) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

PxController_setStepOffset_mut :: (self__pod: *physx_PxController_Pod, offset: float) -> void #foreign physx_sys;

PxController_getStepOffset :: (self__pod: *physx_PxController_Pod) -> float #foreign physx_sys;

PxController_setNonWalkableMode_mut :: (self__pod: *physx_PxController_Pod, flag_pod: s32) -> void #foreign physx_sys;

PxController_getNonWalkableMode :: (self__pod: *physx_PxController_Pod) -> s32 #foreign physx_sys;

PxController_getContactOffset :: (self__pod: *physx_PxController_Pod) -> float #foreign physx_sys;

PxController_setContactOffset_mut :: (self__pod: *physx_PxController_Pod, offset: float) -> void #foreign physx_sys;

PxController_getUpDirection :: (self__pod: *physx_PxController_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxController_setUpDirection_mut :: (self__pod: *physx_PxController_Pod, up_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxController_getSlopeLimit :: (self__pod: *physx_PxController_Pod) -> float #foreign physx_sys;

PxController_setSlopeLimit_mut :: (self__pod: *physx_PxController_Pod, slopeLimit: float) -> void #foreign physx_sys;

PxController_invalidateCache_mut :: (self__pod: *physx_PxController_Pod) -> void #foreign physx_sys;

PxController_getScene_mut :: (self__pod: *physx_PxController_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxController_getUserData :: (self__pod: *physx_PxController_Pod) -> *void #foreign physx_sys;

PxController_setUserData_mut :: (self__pod: *physx_PxController_Pod, userData: *void) -> void #foreign physx_sys;

PxController_getState :: (self__pod: *physx_PxController_Pod, state_pod: *physx_PxControllerState_Pod) -> void #foreign physx_sys;

PxController_getStats :: (self__pod: *physx_PxController_Pod, stats_pod: *physx_PxControllerStats_Pod) -> void #foreign physx_sys;

PxController_resize_mut :: (self__pod: *physx_PxController_Pod, height: float) -> void #foreign physx_sys;

PxBoxControllerDesc_new_alloc :: () -> *physx_PxBoxControllerDesc_Pod #foreign physx_sys;

PxBoxControllerDesc_delete :: (self__pod: *physx_PxBoxControllerDesc_Pod) -> void #foreign physx_sys;

PxBoxControllerDesc_setToDefault_mut :: (self__pod: *physx_PxBoxControllerDesc_Pod) -> void #foreign physx_sys;

PxBoxControllerDesc_isValid :: (self__pod: *physx_PxBoxControllerDesc_Pod) -> bool #foreign physx_sys;

PxBoxController_getHalfHeight :: (self__pod: *physx_PxBoxController_Pod) -> float #foreign physx_sys;

PxBoxController_getHalfSideExtent :: (self__pod: *physx_PxBoxController_Pod) -> float #foreign physx_sys;

PxBoxController_getHalfForwardExtent :: (self__pod: *physx_PxBoxController_Pod) -> float #foreign physx_sys;

PxBoxController_setHalfHeight_mut :: (self__pod: *physx_PxBoxController_Pod, halfHeight: float) -> bool #foreign physx_sys;

PxBoxController_setHalfSideExtent_mut :: (self__pod: *physx_PxBoxController_Pod, halfSideExtent: float) -> bool #foreign physx_sys;

PxBoxController_setHalfForwardExtent_mut :: (self__pod: *physx_PxBoxController_Pod, halfForwardExtent: float) -> bool #foreign physx_sys;

PxCapsuleControllerDesc_new_alloc :: () -> *physx_PxCapsuleControllerDesc_Pod #foreign physx_sys;

PxCapsuleControllerDesc_delete :: (self__pod: *physx_PxCapsuleControllerDesc_Pod) -> void #foreign physx_sys;

PxCapsuleControllerDesc_setToDefault_mut :: (self__pod: *physx_PxCapsuleControllerDesc_Pod) -> void #foreign physx_sys;

PxCapsuleControllerDesc_isValid :: (self__pod: *physx_PxCapsuleControllerDesc_Pod) -> bool #foreign physx_sys;

PxCapsuleController_getRadius :: (self__pod: *physx_PxCapsuleController_Pod) -> float #foreign physx_sys;

PxCapsuleController_setRadius_mut :: (self__pod: *physx_PxCapsuleController_Pod, radius: float) -> bool #foreign physx_sys;

PxCapsuleController_getHeight :: (self__pod: *physx_PxCapsuleController_Pod) -> float #foreign physx_sys;

PxCapsuleController_setHeight_mut :: (self__pod: *physx_PxCapsuleController_Pod, height: float) -> bool #foreign physx_sys;

PxCapsuleController_getClimbingMode :: (self__pod: *physx_PxCapsuleController_Pod) -> s32 #foreign physx_sys;

PxCapsuleController_setClimbingMode_mut :: (self__pod: *physx_PxCapsuleController_Pod, mode_pod: s32) -> bool #foreign physx_sys;

PxControllerBehaviorCallback_getBehaviorFlags_mut :: (self__pod: *physx_PxControllerBehaviorCallback_Pod, shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxActor_Pod) -> u8 #foreign physx_sys;

PxControllerBehaviorCallback_getBehaviorFlags_mut_1 :: (self__pod: *physx_PxControllerBehaviorCallback_Pod, controller_pod: *physx_PxController_Pod) -> u8 #foreign physx_sys;

PxControllerBehaviorCallback_getBehaviorFlags_mut_2 :: (self__pod: *physx_PxControllerBehaviorCallback_Pod, obstacle_pod: *physx_PxObstacle_Pod) -> u8 #foreign physx_sys;

PxControllerManager_release_mut :: (self__pod: *physx_PxControllerManager_Pod) -> void #foreign physx_sys;

PxControllerManager_getScene :: (self__pod: *physx_PxControllerManager_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxControllerManager_getNbControllers :: (self__pod: *physx_PxControllerManager_Pod) -> u32 #foreign physx_sys;

PxControllerManager_getController_mut :: (self__pod: *physx_PxControllerManager_Pod, index: u32) -> *physx_PxController_Pod #foreign physx_sys;

PxControllerManager_createController_mut :: (self__pod: *physx_PxControllerManager_Pod, desc_pod: *physx_PxControllerDesc_Pod) -> *physx_PxController_Pod #foreign physx_sys;

PxControllerManager_purgeControllers_mut :: (self__pod: *physx_PxControllerManager_Pod) -> void #foreign physx_sys;

PxControllerManager_getRenderBuffer_mut :: (self__pod: *physx_PxControllerManager_Pod) -> *physx_PxRenderBuffer_Pod #foreign physx_sys;

PxControllerManager_setDebugRenderingFlags_mut :: (self__pod: *physx_PxControllerManager_Pod, flags_pod: u32) -> void #foreign physx_sys;

PxControllerManager_getNbObstacleContexts :: (self__pod: *physx_PxControllerManager_Pod) -> u32 #foreign physx_sys;

PxControllerManager_getObstacleContext_mut :: (self__pod: *physx_PxControllerManager_Pod, index: u32) -> *physx_PxObstacleContext_Pod #foreign physx_sys;

PxControllerManager_createObstacleContext_mut :: (self__pod: *physx_PxControllerManager_Pod) -> *physx_PxObstacleContext_Pod #foreign physx_sys;

PxControllerManager_computeInteractions_mut :: (self__pod: *physx_PxControllerManager_Pod, elapsedTime: float, cctFilterCb_pod: *physx_PxControllerFilterCallback_Pod) -> void #foreign physx_sys;

PxControllerManager_setTessellation_mut :: (self__pod: *physx_PxControllerManager_Pod, flag: bool, maxEdgeLength: float) -> void #foreign physx_sys;

PxControllerManager_setOverlapRecoveryModule_mut :: (self__pod: *physx_PxControllerManager_Pod, flag: bool) -> void #foreign physx_sys;

PxControllerManager_setPreciseSweeps_mut :: (self__pod: *physx_PxControllerManager_Pod, flag: bool) -> void #foreign physx_sys;

PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut :: (self__pod: *physx_PxControllerManager_Pod, flag: bool) -> void #foreign physx_sys;

PxControllerManager_shiftOrigin_mut :: (self__pod: *physx_PxControllerManager_Pod, shift_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

phys_PxCreateControllerManager :: (scene_pod: *physx_PxScene_Pod, lockingEnabled: bool) -> *physx_PxControllerManager_Pod #foreign physx_sys;

PxDim3_new :: () -> physx_PxDim3_Pod #foreign physx_sys;

PxSDFDesc_new :: () -> physx_PxSDFDesc_Pod #foreign physx_sys;

PxSDFDesc_isValid :: (self__pod: *physx_PxSDFDesc_Pod) -> bool #foreign physx_sys;

PxConvexMeshDesc_new :: () -> physx_PxConvexMeshDesc_Pod #foreign physx_sys;

PxConvexMeshDesc_setToDefault_mut :: (self__pod: *physx_PxConvexMeshDesc_Pod) -> void #foreign physx_sys;

PxConvexMeshDesc_isValid :: (self__pod: *physx_PxConvexMeshDesc_Pod) -> bool #foreign physx_sys;

PxTriangleMeshDesc_new :: () -> physx_PxTriangleMeshDesc_Pod #foreign physx_sys;

PxTriangleMeshDesc_setToDefault_mut :: (self__pod: *physx_PxTriangleMeshDesc_Pod) -> void #foreign physx_sys;

PxTriangleMeshDesc_isValid :: (self__pod: *physx_PxTriangleMeshDesc_Pod) -> bool #foreign physx_sys;

PxTetrahedronMeshDesc_new :: () -> physx_PxTetrahedronMeshDesc_Pod #foreign physx_sys;

PxTetrahedronMeshDesc_isValid :: (self__pod: *physx_PxTetrahedronMeshDesc_Pod) -> bool #foreign physx_sys;

PxSoftBodySimulationDataDesc_new :: () -> physx_PxSoftBodySimulationDataDesc_Pod #foreign physx_sys;

PxSoftBodySimulationDataDesc_isValid :: (self__pod: *physx_PxSoftBodySimulationDataDesc_Pod) -> bool #foreign physx_sys;

PxBVH34MidphaseDesc_setToDefault_mut :: (self__pod: *physx_PxBVH34MidphaseDesc_Pod) -> void #foreign physx_sys;

PxBVH34MidphaseDesc_isValid :: (self__pod: *physx_PxBVH34MidphaseDesc_Pod) -> bool #foreign physx_sys;

PxMidphaseDesc_new :: () -> physx_PxMidphaseDesc_Pod #foreign physx_sys;

PxMidphaseDesc_getType :: (self__pod: *physx_PxMidphaseDesc_Pod) -> s32 #foreign physx_sys;

PxMidphaseDesc_setToDefault_mut :: (self__pod: *physx_PxMidphaseDesc_Pod, type_pod: s32) -> void #foreign physx_sys;

PxMidphaseDesc_isValid :: (self__pod: *physx_PxMidphaseDesc_Pod) -> bool #foreign physx_sys;

PxBVHDesc_new :: () -> physx_PxBVHDesc_Pod #foreign physx_sys;

PxBVHDesc_setToDefault_mut :: (self__pod: *physx_PxBVHDesc_Pod) -> void #foreign physx_sys;

PxBVHDesc_isValid :: (self__pod: *physx_PxBVHDesc_Pod) -> bool #foreign physx_sys;

PxCookingParams_new :: (sc_pod: *physx_PxTolerancesScale_Pod) -> physx_PxCookingParams_Pod #foreign physx_sys;

phys_PxGetStandaloneInsertionCallback :: () -> *physx_PxInsertionCallback_Pod #foreign physx_sys;

phys_PxCookBVH :: (desc_pod: *physx_PxBVHDesc_Pod, stream_pod: *physx_PxOutputStream_Pod) -> bool #foreign physx_sys;

phys_PxCreateBVH :: (desc_pod: *physx_PxBVHDesc_Pod, insertionCallback_pod: *physx_PxInsertionCallback_Pod) -> *physx_PxBVH_Pod #foreign physx_sys;

phys_PxCookHeightField :: (desc_pod: *physx_PxHeightFieldDesc_Pod, stream_pod: *physx_PxOutputStream_Pod) -> bool #foreign physx_sys;

phys_PxCreateHeightField :: (desc_pod: *physx_PxHeightFieldDesc_Pod, insertionCallback_pod: *physx_PxInsertionCallback_Pod) -> *physx_PxHeightField_Pod #foreign physx_sys;

phys_PxCookConvexMesh :: (params_pod: *physx_PxCookingParams_Pod, desc_pod: *physx_PxConvexMeshDesc_Pod, stream_pod: *physx_PxOutputStream_Pod, condition_pod: *s32) -> bool #foreign physx_sys;

phys_PxCreateConvexMesh :: (params_pod: *physx_PxCookingParams_Pod, desc_pod: *physx_PxConvexMeshDesc_Pod, insertionCallback_pod: *physx_PxInsertionCallback_Pod, condition_pod: *s32) -> *physx_PxConvexMesh_Pod #foreign physx_sys;

phys_PxValidateConvexMesh :: (params_pod: *physx_PxCookingParams_Pod, desc_pod: *physx_PxConvexMeshDesc_Pod) -> bool #foreign physx_sys;

phys_PxComputeHullPolygons :: (params_pod: *physx_PxCookingParams_Pod, mesh_pod: *physx_PxSimpleTriangleMesh_Pod, inCallback_pod: *physx_PxAllocatorCallback_Pod, nbVerts_pod: *u32, vertices_pod: **physx_PxVec3_Pod, nbIndices_pod: *u32, indices_pod: **u32, nbPolygons_pod: *u32, hullPolygons_pod: **physx_PxHullPolygon_Pod) -> bool #foreign physx_sys;

phys_PxValidateTriangleMesh :: (params_pod: *physx_PxCookingParams_Pod, desc_pod: *physx_PxTriangleMeshDesc_Pod) -> bool #foreign physx_sys;

phys_PxCreateTriangleMesh :: (params_pod: *physx_PxCookingParams_Pod, desc_pod: *physx_PxTriangleMeshDesc_Pod, insertionCallback_pod: *physx_PxInsertionCallback_Pod, condition_pod: *s32) -> *physx_PxTriangleMesh_Pod #foreign physx_sys;

phys_PxCookTriangleMesh :: (params_pod: *physx_PxCookingParams_Pod, desc_pod: *physx_PxTriangleMeshDesc_Pod, stream_pod: *physx_PxOutputStream_Pod, condition_pod: *s32) -> bool #foreign physx_sys;

PxDefaultMemoryOutputStream_new_alloc :: (allocator_pod: *physx_PxAllocatorCallback_Pod) -> *physx_PxDefaultMemoryOutputStream_Pod #foreign physx_sys;

PxDefaultMemoryOutputStream_delete :: (self__pod: *physx_PxDefaultMemoryOutputStream_Pod) -> void #foreign physx_sys;

PxDefaultMemoryOutputStream_write_mut :: (self__pod: *physx_PxDefaultMemoryOutputStream_Pod, src: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultMemoryOutputStream_getSize :: (self__pod: *physx_PxDefaultMemoryOutputStream_Pod) -> u32 #foreign physx_sys;

PxDefaultMemoryOutputStream_getData :: (self__pod: *physx_PxDefaultMemoryOutputStream_Pod) -> *u8 #foreign physx_sys;

PxDefaultMemoryInputData_new_alloc :: (data: *u8, length: u32) -> *physx_PxDefaultMemoryInputData_Pod #foreign physx_sys;

PxDefaultMemoryInputData_read_mut :: (self__pod: *physx_PxDefaultMemoryInputData_Pod, dest: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultMemoryInputData_getLength :: (self__pod: *physx_PxDefaultMemoryInputData_Pod) -> u32 #foreign physx_sys;

PxDefaultMemoryInputData_seek_mut :: (self__pod: *physx_PxDefaultMemoryInputData_Pod, pos: u32) -> void #foreign physx_sys;

PxDefaultMemoryInputData_tell :: (self__pod: *physx_PxDefaultMemoryInputData_Pod) -> u32 #foreign physx_sys;

PxDefaultFileOutputStream_new_alloc :: (name: *u8) -> *physx_PxDefaultFileOutputStream_Pod #foreign physx_sys;

PxDefaultFileOutputStream_delete :: (self__pod: *physx_PxDefaultFileOutputStream_Pod) -> void #foreign physx_sys;

PxDefaultFileOutputStream_write_mut :: (self__pod: *physx_PxDefaultFileOutputStream_Pod, src: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultFileOutputStream_isValid_mut :: (self__pod: *physx_PxDefaultFileOutputStream_Pod) -> bool #foreign physx_sys;

PxDefaultFileInputData_new_alloc :: (name: *u8) -> *physx_PxDefaultFileInputData_Pod #foreign physx_sys;

PxDefaultFileInputData_delete :: (self__pod: *physx_PxDefaultFileInputData_Pod) -> void #foreign physx_sys;

PxDefaultFileInputData_read_mut :: (self__pod: *physx_PxDefaultFileInputData_Pod, dest: *void, count: u32) -> u32 #foreign physx_sys;

PxDefaultFileInputData_seek_mut :: (self__pod: *physx_PxDefaultFileInputData_Pod, pos: u32) -> void #foreign physx_sys;

PxDefaultFileInputData_tell :: (self__pod: *physx_PxDefaultFileInputData_Pod) -> u32 #foreign physx_sys;

PxDefaultFileInputData_getLength :: (self__pod: *physx_PxDefaultFileInputData_Pod) -> u32 #foreign physx_sys;

PxDefaultFileInputData_isValid :: (self__pod: *physx_PxDefaultFileInputData_Pod) -> bool #foreign physx_sys;

phys_platformAlignedAlloc :: (size_pod: u64) -> *void #foreign physx_sys;

phys_platformAlignedFree :: (ptr: *void) -> void #foreign physx_sys;

PxDefaultAllocator_allocate_mut :: (self__pod: *physx_PxDefaultAllocator_Pod, size_pod: u64, anon_param1: *u8, anon_param2: *u8, anon_param3: s32) -> *void #foreign physx_sys;

PxDefaultAllocator_deallocate_mut :: (self__pod: *physx_PxDefaultAllocator_Pod, ptr: *void) -> void #foreign physx_sys;

PxDefaultAllocator_delete :: (self__pod: *physx_PxDefaultAllocator_Pod) -> void #foreign physx_sys;

PxJoint_setActors_mut :: (self__pod: *physx_PxJoint_Pod, actor0_pod: *physx_PxRigidActor_Pod, actor1_pod: *physx_PxRigidActor_Pod) -> void #foreign physx_sys;

PxJoint_getActors :: (self__pod: *physx_PxJoint_Pod, actor0_pod: **physx_PxRigidActor_Pod, actor1_pod: **physx_PxRigidActor_Pod) -> void #foreign physx_sys;

PxJoint_setLocalPose_mut :: (self__pod: *physx_PxJoint_Pod, actor_pod: s32, localPose_pod: *physx_PxTransform_Pod) -> void #foreign physx_sys;

PxJoint_getLocalPose :: (self__pod: *physx_PxJoint_Pod, actor_pod: s32) -> physx_PxTransform_Pod #foreign physx_sys;

PxJoint_getRelativeTransform :: (self__pod: *physx_PxJoint_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxJoint_getRelativeLinearVelocity :: (self__pod: *physx_PxJoint_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxJoint_getRelativeAngularVelocity :: (self__pod: *physx_PxJoint_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxJoint_setBreakForce_mut :: (self__pod: *physx_PxJoint_Pod, force: float, torque: float) -> void #foreign physx_sys;

PxJoint_getBreakForce :: (self__pod: *physx_PxJoint_Pod, force_pod: *float, torque_pod: *float) -> void #foreign physx_sys;

PxJoint_setConstraintFlags_mut :: (self__pod: *physx_PxJoint_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxJoint_setConstraintFlag_mut :: (self__pod: *physx_PxJoint_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxJoint_getConstraintFlags :: (self__pod: *physx_PxJoint_Pod) -> u16 #foreign physx_sys;

PxJoint_setInvMassScale0_mut :: (self__pod: *physx_PxJoint_Pod, invMassScale: float) -> void #foreign physx_sys;

PxJoint_getInvMassScale0 :: (self__pod: *physx_PxJoint_Pod) -> float #foreign physx_sys;

PxJoint_setInvInertiaScale0_mut :: (self__pod: *physx_PxJoint_Pod, invInertiaScale: float) -> void #foreign physx_sys;

PxJoint_getInvInertiaScale0 :: (self__pod: *physx_PxJoint_Pod) -> float #foreign physx_sys;

PxJoint_setInvMassScale1_mut :: (self__pod: *physx_PxJoint_Pod, invMassScale: float) -> void #foreign physx_sys;

PxJoint_getInvMassScale1 :: (self__pod: *physx_PxJoint_Pod) -> float #foreign physx_sys;

PxJoint_setInvInertiaScale1_mut :: (self__pod: *physx_PxJoint_Pod, invInertiaScale: float) -> void #foreign physx_sys;

PxJoint_getInvInertiaScale1 :: (self__pod: *physx_PxJoint_Pod) -> float #foreign physx_sys;

PxJoint_getConstraint :: (self__pod: *physx_PxJoint_Pod) -> *physx_PxConstraint_Pod #foreign physx_sys;

PxJoint_setName_mut :: (self__pod: *physx_PxJoint_Pod, name: *u8) -> void #foreign physx_sys;

PxJoint_getName :: (self__pod: *physx_PxJoint_Pod) -> *u8 #foreign physx_sys;

PxJoint_release_mut :: (self__pod: *physx_PxJoint_Pod) -> void #foreign physx_sys;

PxJoint_getScene :: (self__pod: *physx_PxJoint_Pod) -> *physx_PxScene_Pod #foreign physx_sys;

PxJoint_getBinaryMetaData :: (stream_pod: *physx_PxOutputStream_Pod) -> void #foreign physx_sys;

PxSpring_new :: (stiffness_: float, damping_: float) -> physx_PxSpring_Pod #foreign physx_sys;

phys_PxSetJointGlobalFrame :: (joint_pod: *physx_PxJoint_Pod, wsAnchor_pod: *physx_PxVec3_Pod, wsAxis_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

phys_PxDistanceJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxDistanceJoint_Pod #foreign physx_sys;

PxDistanceJoint_getDistance :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setMinDistance_mut :: (self__pod: *physx_PxDistanceJoint_Pod, distance: float) -> void #foreign physx_sys;

PxDistanceJoint_getMinDistance :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setMaxDistance_mut :: (self__pod: *physx_PxDistanceJoint_Pod, distance: float) -> void #foreign physx_sys;

PxDistanceJoint_getMaxDistance :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setTolerance_mut :: (self__pod: *physx_PxDistanceJoint_Pod, tolerance: float) -> void #foreign physx_sys;

PxDistanceJoint_getTolerance :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setStiffness_mut :: (self__pod: *physx_PxDistanceJoint_Pod, stiffness: float) -> void #foreign physx_sys;

PxDistanceJoint_getStiffness :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setDamping_mut :: (self__pod: *physx_PxDistanceJoint_Pod, damping: float) -> void #foreign physx_sys;

PxDistanceJoint_getDamping :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setContactDistance_mut :: (self__pod: *physx_PxDistanceJoint_Pod, contactDistance: float) -> void #foreign physx_sys;

PxDistanceJoint_getContactDistance :: (self__pod: *physx_PxDistanceJoint_Pod) -> float #foreign physx_sys;

PxDistanceJoint_setDistanceJointFlags_mut :: (self__pod: *physx_PxDistanceJoint_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxDistanceJoint_setDistanceJointFlag_mut :: (self__pod: *physx_PxDistanceJoint_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxDistanceJoint_getDistanceJointFlags :: (self__pod: *physx_PxDistanceJoint_Pod) -> u16 #foreign physx_sys;

PxDistanceJoint_getConcreteTypeName :: (self__pod: *physx_PxDistanceJoint_Pod) -> *u8 #foreign physx_sys;

phys_PxContactJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxContactJoint_Pod #foreign physx_sys;

PxJacobianRow_new :: () -> physx_PxJacobianRow_Pod #foreign physx_sys;

PxJacobianRow_new_1 :: (lin0_pod: *physx_PxVec3_Pod, lin1_pod: *physx_PxVec3_Pod, ang0_pod: *physx_PxVec3_Pod, ang1_pod: *physx_PxVec3_Pod) -> physx_PxJacobianRow_Pod #foreign physx_sys;

PxContactJoint_setContact_mut :: (self__pod: *physx_PxContactJoint_Pod, contact_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxContactJoint_setContactNormal_mut :: (self__pod: *physx_PxContactJoint_Pod, contactNormal_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxContactJoint_setPenetration_mut :: (self__pod: *physx_PxContactJoint_Pod, penetration: float) -> void #foreign physx_sys;

PxContactJoint_getContact :: (self__pod: *physx_PxContactJoint_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxContactJoint_getContactNormal :: (self__pod: *physx_PxContactJoint_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxContactJoint_getPenetration :: (self__pod: *physx_PxContactJoint_Pod) -> float #foreign physx_sys;

PxContactJoint_getRestitution :: (self__pod: *physx_PxContactJoint_Pod) -> float #foreign physx_sys;

PxContactJoint_setRestitution_mut :: (self__pod: *physx_PxContactJoint_Pod, restitution: float) -> void #foreign physx_sys;

PxContactJoint_getBounceThreshold :: (self__pod: *physx_PxContactJoint_Pod) -> float #foreign physx_sys;

PxContactJoint_setBounceThreshold_mut :: (self__pod: *physx_PxContactJoint_Pod, bounceThreshold: float) -> void #foreign physx_sys;

PxContactJoint_getConcreteTypeName :: (self__pod: *physx_PxContactJoint_Pod) -> *u8 #foreign physx_sys;

PxContactJoint_computeJacobians :: (self__pod: *physx_PxContactJoint_Pod, jacobian_pod: *physx_PxJacobianRow_Pod) -> void #foreign physx_sys;

PxContactJoint_getNbJacobianRows :: (self__pod: *physx_PxContactJoint_Pod) -> u32 #foreign physx_sys;

phys_PxFixedJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxFixedJoint_Pod #foreign physx_sys;

PxFixedJoint_getConcreteTypeName :: (self__pod: *physx_PxFixedJoint_Pod) -> *u8 #foreign physx_sys;

PxJointLimitParameters_new_alloc :: () -> *physx_PxJointLimitParameters_Pod #foreign physx_sys;

PxJointLimitParameters_isValid :: (self__pod: *physx_PxJointLimitParameters_Pod) -> bool #foreign physx_sys;

PxJointLimitParameters_isSoft :: (self__pod: *physx_PxJointLimitParameters_Pod) -> bool #foreign physx_sys;

PxJointLinearLimit_new :: (scale_pod: *physx_PxTolerancesScale_Pod, extent: float, contactDist_deprecated: float) -> physx_PxJointLinearLimit_Pod #foreign physx_sys;

PxJointLinearLimit_new_1 :: (extent: float, spring_pod: *physx_PxSpring_Pod) -> physx_PxJointLinearLimit_Pod #foreign physx_sys;

PxJointLinearLimit_isValid :: (self__pod: *physx_PxJointLinearLimit_Pod) -> bool #foreign physx_sys;

PxJointLinearLimit_delete :: (self__pod: *physx_PxJointLinearLimit_Pod) -> void #foreign physx_sys;

PxJointLinearLimitPair_new :: (scale_pod: *physx_PxTolerancesScale_Pod, lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> physx_PxJointLinearLimitPair_Pod #foreign physx_sys;

PxJointLinearLimitPair_new_1 :: (lowerLimit: float, upperLimit: float, spring_pod: *physx_PxSpring_Pod) -> physx_PxJointLinearLimitPair_Pod #foreign physx_sys;

PxJointLinearLimitPair_isValid :: (self__pod: *physx_PxJointLinearLimitPair_Pod) -> bool #foreign physx_sys;

PxJointLinearLimitPair_delete :: (self__pod: *physx_PxJointLinearLimitPair_Pod) -> void #foreign physx_sys;

PxJointAngularLimitPair_new :: (lowerLimit: float, upperLimit: float, contactDist_deprecated: float) -> physx_PxJointAngularLimitPair_Pod #foreign physx_sys;

PxJointAngularLimitPair_new_1 :: (lowerLimit: float, upperLimit: float, spring_pod: *physx_PxSpring_Pod) -> physx_PxJointAngularLimitPair_Pod #foreign physx_sys;

PxJointAngularLimitPair_isValid :: (self__pod: *physx_PxJointAngularLimitPair_Pod) -> bool #foreign physx_sys;

PxJointAngularLimitPair_delete :: (self__pod: *physx_PxJointAngularLimitPair_Pod) -> void #foreign physx_sys;

PxJointLimitCone_new :: (yLimitAngle: float, zLimitAngle: float, contactDist_deprecated: float) -> physx_PxJointLimitCone_Pod #foreign physx_sys;

PxJointLimitCone_new_1 :: (yLimitAngle: float, zLimitAngle: float, spring_pod: *physx_PxSpring_Pod) -> physx_PxJointLimitCone_Pod #foreign physx_sys;

PxJointLimitCone_isValid :: (self__pod: *physx_PxJointLimitCone_Pod) -> bool #foreign physx_sys;

PxJointLimitCone_delete :: (self__pod: *physx_PxJointLimitCone_Pod) -> void #foreign physx_sys;

PxJointLimitPyramid_new :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, contactDist_deprecated: float) -> physx_PxJointLimitPyramid_Pod #foreign physx_sys;

PxJointLimitPyramid_new_1 :: (yLimitAngleMin: float, yLimitAngleMax: float, zLimitAngleMin: float, zLimitAngleMax: float, spring_pod: *physx_PxSpring_Pod) -> physx_PxJointLimitPyramid_Pod #foreign physx_sys;

PxJointLimitPyramid_isValid :: (self__pod: *physx_PxJointLimitPyramid_Pod) -> bool #foreign physx_sys;

PxJointLimitPyramid_delete :: (self__pod: *physx_PxJointLimitPyramid_Pod) -> void #foreign physx_sys;

phys_PxPrismaticJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxPrismaticJoint_Pod #foreign physx_sys;

PxPrismaticJoint_getPosition :: (self__pod: *physx_PxPrismaticJoint_Pod) -> float #foreign physx_sys;

PxPrismaticJoint_getVelocity :: (self__pod: *physx_PxPrismaticJoint_Pod) -> float #foreign physx_sys;

PxPrismaticJoint_setLimit_mut :: (self__pod: *physx_PxPrismaticJoint_Pod, anon_param0_pod: *physx_PxJointLinearLimitPair_Pod) -> void #foreign physx_sys;

PxPrismaticJoint_getLimit :: (self__pod: *physx_PxPrismaticJoint_Pod) -> physx_PxJointLinearLimitPair_Pod #foreign physx_sys;

PxPrismaticJoint_setPrismaticJointFlags_mut :: (self__pod: *physx_PxPrismaticJoint_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxPrismaticJoint_setPrismaticJointFlag_mut :: (self__pod: *physx_PxPrismaticJoint_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxPrismaticJoint_getPrismaticJointFlags :: (self__pod: *physx_PxPrismaticJoint_Pod) -> u16 #foreign physx_sys;

PxPrismaticJoint_getConcreteTypeName :: (self__pod: *physx_PxPrismaticJoint_Pod) -> *u8 #foreign physx_sys;

phys_PxRevoluteJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxRevoluteJoint_Pod #foreign physx_sys;

PxRevoluteJoint_getAngle :: (self__pod: *physx_PxRevoluteJoint_Pod) -> float #foreign physx_sys;

PxRevoluteJoint_getVelocity :: (self__pod: *physx_PxRevoluteJoint_Pod) -> float #foreign physx_sys;

PxRevoluteJoint_setLimit_mut :: (self__pod: *physx_PxRevoluteJoint_Pod, limits_pod: *physx_PxJointAngularLimitPair_Pod) -> void #foreign physx_sys;

PxRevoluteJoint_getLimit :: (self__pod: *physx_PxRevoluteJoint_Pod) -> physx_PxJointAngularLimitPair_Pod #foreign physx_sys;

PxRevoluteJoint_setDriveVelocity_mut :: (self__pod: *physx_PxRevoluteJoint_Pod, velocity: float, autowake: bool) -> void #foreign physx_sys;

PxRevoluteJoint_getDriveVelocity :: (self__pod: *physx_PxRevoluteJoint_Pod) -> float #foreign physx_sys;

PxRevoluteJoint_setDriveForceLimit_mut :: (self__pod: *physx_PxRevoluteJoint_Pod, limit: float) -> void #foreign physx_sys;

PxRevoluteJoint_getDriveForceLimit :: (self__pod: *physx_PxRevoluteJoint_Pod) -> float #foreign physx_sys;

PxRevoluteJoint_setDriveGearRatio_mut :: (self__pod: *physx_PxRevoluteJoint_Pod, ratio: float) -> void #foreign physx_sys;

PxRevoluteJoint_getDriveGearRatio :: (self__pod: *physx_PxRevoluteJoint_Pod) -> float #foreign physx_sys;

PxRevoluteJoint_setRevoluteJointFlags_mut :: (self__pod: *physx_PxRevoluteJoint_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxRevoluteJoint_setRevoluteJointFlag_mut :: (self__pod: *physx_PxRevoluteJoint_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxRevoluteJoint_getRevoluteJointFlags :: (self__pod: *physx_PxRevoluteJoint_Pod) -> u16 #foreign physx_sys;

PxRevoluteJoint_getConcreteTypeName :: (self__pod: *physx_PxRevoluteJoint_Pod) -> *u8 #foreign physx_sys;

phys_PxSphericalJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxSphericalJoint_Pod #foreign physx_sys;

PxSphericalJoint_getLimitCone :: (self__pod: *physx_PxSphericalJoint_Pod) -> physx_PxJointLimitCone_Pod #foreign physx_sys;

PxSphericalJoint_setLimitCone_mut :: (self__pod: *physx_PxSphericalJoint_Pod, limit_pod: *physx_PxJointLimitCone_Pod) -> void #foreign physx_sys;

PxSphericalJoint_getSwingYAngle :: (self__pod: *physx_PxSphericalJoint_Pod) -> float #foreign physx_sys;

PxSphericalJoint_getSwingZAngle :: (self__pod: *physx_PxSphericalJoint_Pod) -> float #foreign physx_sys;

PxSphericalJoint_setSphericalJointFlags_mut :: (self__pod: *physx_PxSphericalJoint_Pod, flags_pod: u16) -> void #foreign physx_sys;

PxSphericalJoint_setSphericalJointFlag_mut :: (self__pod: *physx_PxSphericalJoint_Pod, flag_pod: s32, value: bool) -> void #foreign physx_sys;

PxSphericalJoint_getSphericalJointFlags :: (self__pod: *physx_PxSphericalJoint_Pod) -> u16 #foreign physx_sys;

PxSphericalJoint_getConcreteTypeName :: (self__pod: *physx_PxSphericalJoint_Pod) -> *u8 #foreign physx_sys;

phys_PxD6JointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxD6Joint_Pod #foreign physx_sys;

PxD6JointDrive_new :: () -> physx_PxD6JointDrive_Pod #foreign physx_sys;

PxD6JointDrive_new_1 :: (driveStiffness: float, driveDamping: float, driveForceLimit: float, isAcceleration: bool) -> physx_PxD6JointDrive_Pod #foreign physx_sys;

PxD6JointDrive_isValid :: (self__pod: *physx_PxD6JointDrive_Pod) -> bool #foreign physx_sys;

PxD6Joint_setMotion_mut :: (self__pod: *physx_PxD6Joint_Pod, axis_pod: s32, type_pod: s32) -> void #foreign physx_sys;

PxD6Joint_getMotion :: (self__pod: *physx_PxD6Joint_Pod, axis_pod: s32) -> s32 #foreign physx_sys;

PxD6Joint_getTwistAngle :: (self__pod: *physx_PxD6Joint_Pod) -> float #foreign physx_sys;

PxD6Joint_getSwingYAngle :: (self__pod: *physx_PxD6Joint_Pod) -> float #foreign physx_sys;

PxD6Joint_getSwingZAngle :: (self__pod: *physx_PxD6Joint_Pod) -> float #foreign physx_sys;

PxD6Joint_setDistanceLimit_mut :: (self__pod: *physx_PxD6Joint_Pod, limit_pod: *physx_PxJointLinearLimit_Pod) -> void #foreign physx_sys;

PxD6Joint_getDistanceLimit :: (self__pod: *physx_PxD6Joint_Pod) -> physx_PxJointLinearLimit_Pod #foreign physx_sys;

PxD6Joint_setLinearLimit_mut :: (self__pod: *physx_PxD6Joint_Pod, axis_pod: s32, limit_pod: *physx_PxJointLinearLimitPair_Pod) -> void #foreign physx_sys;

PxD6Joint_getLinearLimit :: (self__pod: *physx_PxD6Joint_Pod, axis_pod: s32) -> physx_PxJointLinearLimitPair_Pod #foreign physx_sys;

PxD6Joint_setTwistLimit_mut :: (self__pod: *physx_PxD6Joint_Pod, limit_pod: *physx_PxJointAngularLimitPair_Pod) -> void #foreign physx_sys;

PxD6Joint_getTwistLimit :: (self__pod: *physx_PxD6Joint_Pod) -> physx_PxJointAngularLimitPair_Pod #foreign physx_sys;

PxD6Joint_setSwingLimit_mut :: (self__pod: *physx_PxD6Joint_Pod, limit_pod: *physx_PxJointLimitCone_Pod) -> void #foreign physx_sys;

PxD6Joint_getSwingLimit :: (self__pod: *physx_PxD6Joint_Pod) -> physx_PxJointLimitCone_Pod #foreign physx_sys;

PxD6Joint_setPyramidSwingLimit_mut :: (self__pod: *physx_PxD6Joint_Pod, limit_pod: *physx_PxJointLimitPyramid_Pod) -> void #foreign physx_sys;

PxD6Joint_getPyramidSwingLimit :: (self__pod: *physx_PxD6Joint_Pod) -> physx_PxJointLimitPyramid_Pod #foreign physx_sys;

PxD6Joint_setDrive_mut :: (self__pod: *physx_PxD6Joint_Pod, index_pod: s32, drive_pod: *physx_PxD6JointDrive_Pod) -> void #foreign physx_sys;

PxD6Joint_getDrive :: (self__pod: *physx_PxD6Joint_Pod, index_pod: s32) -> physx_PxD6JointDrive_Pod #foreign physx_sys;

PxD6Joint_setDrivePosition_mut :: (self__pod: *physx_PxD6Joint_Pod, pose_pod: *physx_PxTransform_Pod, autowake: bool) -> void #foreign physx_sys;

PxD6Joint_getDrivePosition :: (self__pod: *physx_PxD6Joint_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxD6Joint_setDriveVelocity_mut :: (self__pod: *physx_PxD6Joint_Pod, linear_pod: *physx_PxVec3_Pod, angular_pod: *physx_PxVec3_Pod, autowake: bool) -> void #foreign physx_sys;

PxD6Joint_getDriveVelocity :: (self__pod: *physx_PxD6Joint_Pod, linear_pod: *physx_PxVec3_Pod, angular_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxD6Joint_setProjectionLinearTolerance_mut :: (self__pod: *physx_PxD6Joint_Pod, tolerance: float) -> void #foreign physx_sys;

PxD6Joint_getProjectionLinearTolerance :: (self__pod: *physx_PxD6Joint_Pod) -> float #foreign physx_sys;

PxD6Joint_setProjectionAngularTolerance_mut :: (self__pod: *physx_PxD6Joint_Pod, tolerance: float) -> void #foreign physx_sys;

PxD6Joint_getProjectionAngularTolerance :: (self__pod: *physx_PxD6Joint_Pod) -> float #foreign physx_sys;

PxD6Joint_getConcreteTypeName :: (self__pod: *physx_PxD6Joint_Pod) -> *u8 #foreign physx_sys;

phys_PxGearJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxGearJoint_Pod #foreign physx_sys;

PxGearJoint_setHinges_mut :: (self__pod: *physx_PxGearJoint_Pod, hinge0_pod: *physx_PxBase_Pod, hinge1_pod: *physx_PxBase_Pod) -> bool #foreign physx_sys;

PxGearJoint_setGearRatio_mut :: (self__pod: *physx_PxGearJoint_Pod, ratio: float) -> void #foreign physx_sys;

PxGearJoint_getGearRatio :: (self__pod: *physx_PxGearJoint_Pod) -> float #foreign physx_sys;

PxGearJoint_getConcreteTypeName :: (self__pod: *physx_PxGearJoint_Pod) -> *u8 #foreign physx_sys;

phys_PxRackAndPinionJointCreate :: (physics_pod: *physx_PxPhysics_Pod, actor0_pod: *physx_PxRigidActor_Pod, localFrame0_pod: *physx_PxTransform_Pod, actor1_pod: *physx_PxRigidActor_Pod, localFrame1_pod: *physx_PxTransform_Pod) -> *physx_PxRackAndPinionJoint_Pod #foreign physx_sys;

PxRackAndPinionJoint_setJoints_mut :: (self__pod: *physx_PxRackAndPinionJoint_Pod, hinge_pod: *physx_PxBase_Pod, prismatic_pod: *physx_PxBase_Pod) -> bool #foreign physx_sys;

PxRackAndPinionJoint_setRatio_mut :: (self__pod: *physx_PxRackAndPinionJoint_Pod, ratio: float) -> void #foreign physx_sys;

PxRackAndPinionJoint_getRatio :: (self__pod: *physx_PxRackAndPinionJoint_Pod) -> float #foreign physx_sys;

PxRackAndPinionJoint_setData_mut :: (self__pod: *physx_PxRackAndPinionJoint_Pod, nbRackTeeth: u32, nbPinionTeeth: u32, rackLength: float) -> bool #foreign physx_sys;

PxRackAndPinionJoint_getConcreteTypeName :: (self__pod: *physx_PxRackAndPinionJoint_Pod) -> *u8 #foreign physx_sys;

PxGroupsMask_new_alloc :: () -> *physx_PxGroupsMask_Pod #foreign physx_sys;

PxGroupsMask_delete :: (self__pod: *physx_PxGroupsMask_Pod) -> void #foreign physx_sys;

phys_PxDefaultSimulationFilterShader :: (attributes0: u32, filterData0_pod: physx_PxFilterData_Pod, attributes1: u32, filterData1_pod: physx_PxFilterData_Pod, pairFlags_pod: *u16, constantBlock: *void, constantBlockSize: u32) -> u16 #foreign physx_sys;

phys_PxGetGroupCollisionFlag :: (group1: u16, group2: u16) -> bool #foreign physx_sys;

phys_PxSetGroupCollisionFlag :: (group1: u16, group2: u16, enable: bool) -> void #foreign physx_sys;

phys_PxGetGroup :: (actor_pod: *physx_PxActor_Pod) -> u16 #foreign physx_sys;

phys_PxSetGroup :: (actor_pod: *physx_PxActor_Pod, collisionGroup: u16) -> void #foreign physx_sys;

phys_PxGetFilterOps :: (op0_pod: *s32, op1_pod: *s32, op2_pod: *s32) -> void #foreign physx_sys;

phys_PxSetFilterOps :: (op0_pod: *s32, op1_pod: *s32, op2_pod: *s32) -> void #foreign physx_sys;

phys_PxGetFilterBool :: () -> bool #foreign physx_sys;

phys_PxSetFilterBool :: (enable: bool) -> void #foreign physx_sys;

phys_PxGetFilterConstants :: (c0_pod: *physx_PxGroupsMask_Pod, c1_pod: *physx_PxGroupsMask_Pod) -> void #foreign physx_sys;

phys_PxSetFilterConstants :: (c0_pod: *physx_PxGroupsMask_Pod, c1_pod: *physx_PxGroupsMask_Pod) -> void #foreign physx_sys;

phys_PxGetGroupsMask :: (actor_pod: *physx_PxActor_Pod) -> physx_PxGroupsMask_Pod #foreign physx_sys;

phys_PxSetGroupsMask :: (actor_pod: *physx_PxActor_Pod, mask_pod: *physx_PxGroupsMask_Pod) -> void #foreign physx_sys;

PxDefaultErrorCallback_new_alloc :: () -> *physx_PxDefaultErrorCallback_Pod #foreign physx_sys;

PxDefaultErrorCallback_delete :: (self__pod: *physx_PxDefaultErrorCallback_Pod) -> void #foreign physx_sys;

PxDefaultErrorCallback_reportError_mut :: (self__pod: *physx_PxDefaultErrorCallback_Pod, code_pod: s32, message: *u8, file: *u8, line: s32) -> void #foreign physx_sys;

PxRigidActorExt_createExclusiveShape :: (actor_pod: *physx_PxRigidActor_Pod, geometry_pod: *physx_PxGeometry_Pod, materials_pod: **physx_PxMaterial_Pod, materialCount: u16, shapeFlags_pod: u8) -> *physx_PxShape_Pod #foreign physx_sys;

PxRigidActorExt_createExclusiveShape_1 :: (actor_pod: *physx_PxRigidActor_Pod, geometry_pod: *physx_PxGeometry_Pod, material_pod: *physx_PxMaterial_Pod, shapeFlags_pod: u8) -> *physx_PxShape_Pod #foreign physx_sys;

PxRigidActorExt_getRigidActorShapeLocalBoundsList :: (actor_pod: *physx_PxRigidActor_Pod, numBounds_pod: *u32) -> *physx_PxBounds3_Pod #foreign physx_sys;

PxRigidActorExt_createBVHFromActor :: (physics_pod: *physx_PxPhysics_Pod, actor_pod: *physx_PxRigidActor_Pod) -> *physx_PxBVH_Pod #foreign physx_sys;

PxMassProperties_new :: () -> physx_PxMassProperties_Pod #foreign physx_sys;

PxMassProperties_new_1 :: (m: float, inertiaT_pod: *physx_PxMat33_Pod, com_pod: *physx_PxVec3_Pod) -> physx_PxMassProperties_Pod #foreign physx_sys;

PxMassProperties_new_2 :: (geometry_pod: *physx_PxGeometry_Pod) -> physx_PxMassProperties_Pod #foreign physx_sys;

PxMassProperties_translate_mut :: (self__pod: *physx_PxMassProperties_Pod, t_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxMassProperties_getMassSpaceInertia :: (inertia_pod: *physx_PxMat33_Pod, massFrame_pod: *physx_PxQuat_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxMassProperties_translateInertia :: (inertia_pod: *physx_PxMat33_Pod, mass: float, t_pod: *physx_PxVec3_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMassProperties_rotateInertia :: (inertia_pod: *physx_PxMat33_Pod, q_pod: *physx_PxQuat_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMassProperties_scaleInertia :: (inertia_pod: *physx_PxMat33_Pod, scaleRotation_pod: *physx_PxQuat_Pod, scale_pod: *physx_PxVec3_Pod) -> physx_PxMat33_Pod #foreign physx_sys;

PxMassProperties_sum :: (props_pod: *physx_PxMassProperties_Pod, transforms_pod: *physx_PxTransform_Pod, count: u32) -> physx_PxMassProperties_Pod #foreign physx_sys;

PxRigidBodyExt_updateMassAndInertia :: (body_pod: *physx_PxRigidBody_Pod, shapeDensities: *float, shapeDensityCount: u32, massLocalPose_pod: *physx_PxVec3_Pod, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_updateMassAndInertia_1 :: (body_pod: *physx_PxRigidBody_Pod, density: float, massLocalPose_pod: *physx_PxVec3_Pod, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_setMassAndUpdateInertia :: (body_pod: *physx_PxRigidBody_Pod, shapeMasses: *float, shapeMassCount: u32, massLocalPose_pod: *physx_PxVec3_Pod, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_setMassAndUpdateInertia_1 :: (body_pod: *physx_PxRigidBody_Pod, mass: float, massLocalPose_pod: *physx_PxVec3_Pod, includeNonSimShapes: bool) -> bool #foreign physx_sys;

PxRigidBodyExt_computeMassPropertiesFromShapes :: (shapes_pod: **physx_PxShape_Pod, shapeCount: u32) -> physx_PxMassProperties_Pod #foreign physx_sys;

PxRigidBodyExt_addForceAtPos :: (body_pod: *physx_PxRigidBody_Pod, force_pod: *physx_PxVec3_Pod, pos_pod: *physx_PxVec3_Pod, mode_pod: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addForceAtLocalPos :: (body_pod: *physx_PxRigidBody_Pod, force_pod: *physx_PxVec3_Pod, pos_pod: *physx_PxVec3_Pod, mode_pod: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addLocalForceAtPos :: (body_pod: *physx_PxRigidBody_Pod, force_pod: *physx_PxVec3_Pod, pos_pod: *physx_PxVec3_Pod, mode_pod: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_addLocalForceAtLocalPos :: (body_pod: *physx_PxRigidBody_Pod, force_pod: *physx_PxVec3_Pod, pos_pod: *physx_PxVec3_Pod, mode_pod: s32, wakeup: bool) -> void #foreign physx_sys;

PxRigidBodyExt_getVelocityAtPos :: (body_pod: *physx_PxRigidBody_Pod, pos_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBodyExt_getLocalVelocityAtLocalPos :: (body_pod: *physx_PxRigidBody_Pod, pos_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBodyExt_getVelocityAtOffset :: (body_pod: *physx_PxRigidBody_Pod, pos_pod: *physx_PxVec3_Pod) -> physx_PxVec3_Pod #foreign physx_sys;

PxRigidBodyExt_computeVelocityDeltaFromImpulse :: (body_pod: *physx_PxRigidBody_Pod, impulsiveForce_pod: *physx_PxVec3_Pod, impulsiveTorque_pod: *physx_PxVec3_Pod, deltaLinearVelocity_pod: *physx_PxVec3_Pod, deltaAngularVelocity_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxRigidBodyExt_computeVelocityDeltaFromImpulse_1 :: (body_pod: *physx_PxRigidBody_Pod, globalPose_pod: *physx_PxTransform_Pod, point_pod: *physx_PxVec3_Pod, impulse_pod: *physx_PxVec3_Pod, invMassScale: float, invInertiaScale: float, deltaLinearVelocity_pod: *physx_PxVec3_Pod, deltaAngularVelocity_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxRigidBodyExt_computeLinearAngularImpulse :: (body_pod: *physx_PxRigidBody_Pod, globalPose_pod: *physx_PxTransform_Pod, point_pod: *physx_PxVec3_Pod, impulse_pod: *physx_PxVec3_Pod, invMassScale: float, invInertiaScale: float, linearImpulse_pod: *physx_PxVec3_Pod, angularImpulse_pod: *physx_PxVec3_Pod) -> void #foreign physx_sys;

PxRigidBodyExt_linearSweepSingle :: (body_pod: *physx_PxRigidBody_Pod, scene_pod: *physx_PxScene_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, outputFlags_pod: u16, closestHit_pod: *physx_PxSweepHit_Pod, shapeIndex_pod: *u32, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float) -> bool #foreign physx_sys;

PxRigidBodyExt_linearSweepMultiple :: (body_pod: *physx_PxRigidBody_Pod, scene_pod: *physx_PxScene_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, outputFlags_pod: u16, touchHitBuffer_pod: *physx_PxSweepHit_Pod, touchHitShapeIndices: *u32, touchHitBufferSize: u32, block_pod: *physx_PxSweepHit_Pod, blockingShapeIndex_pod: *s32, overflow_pod: *bool, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float) -> u32 #foreign physx_sys;

PxShapeExt_getGlobalPose :: (shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod) -> physx_PxTransform_Pod #foreign physx_sys;

PxShapeExt_raycast :: (shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod, rayOrigin_pod: *physx_PxVec3_Pod, rayDir_pod: *physx_PxVec3_Pod, maxDist: float, hitFlags_pod: u16, maxHits: u32, rayHits_pod: *physx_PxRaycastHit_Pod) -> u32 #foreign physx_sys;

PxShapeExt_overlap :: (shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod, otherGeom_pod: *physx_PxGeometry_Pod, otherGeomPose_pod: *physx_PxTransform_Pod) -> bool #foreign physx_sys;

PxShapeExt_sweep :: (shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, otherGeom_pod: *physx_PxGeometry_Pod, otherGeomPose_pod: *physx_PxTransform_Pod, sweepHit_pod: *physx_PxSweepHit_Pod, hitFlags_pod: u16) -> bool #foreign physx_sys;

PxShapeExt_getWorldBounds :: (shape_pod: *physx_PxShape_Pod, actor_pod: *physx_PxRigidActor_Pod, inflation: float) -> physx_PxBounds3_Pod #foreign physx_sys;

PxMeshOverlapUtil_new_alloc :: () -> *physx_PxMeshOverlapUtil_Pod #foreign physx_sys;

PxMeshOverlapUtil_delete :: (self__pod: *physx_PxMeshOverlapUtil_Pod) -> void #foreign physx_sys;

PxMeshOverlapUtil_findOverlap_mut :: (self__pod: *physx_PxMeshOverlapUtil_Pod, geom_pod: *physx_PxGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, meshGeom_pod: *physx_PxTriangleMeshGeometry_Pod, meshPose_pod: *physx_PxTransform_Pod) -> u32 #foreign physx_sys;

PxMeshOverlapUtil_findOverlap_mut_1 :: (self__pod: *physx_PxMeshOverlapUtil_Pod, geom_pod: *physx_PxGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, hfGeom_pod: *physx_PxHeightFieldGeometry_Pod, hfPose_pod: *physx_PxTransform_Pod) -> u32 #foreign physx_sys;

PxMeshOverlapUtil_getResults :: (self__pod: *physx_PxMeshOverlapUtil_Pod) -> *u32 #foreign physx_sys;

PxMeshOverlapUtil_getNbResults :: (self__pod: *physx_PxMeshOverlapUtil_Pod) -> u32 #foreign physx_sys;

phys_PxComputeTriangleMeshPenetration :: (direction_pod: *physx_PxVec3_Pod, depth_pod: *float, geom_pod: *physx_PxGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, meshGeom_pod: *physx_PxTriangleMeshGeometry_Pod, meshPose_pod: *physx_PxTransform_Pod, maxIter: u32, usedIter: *u32) -> bool #foreign physx_sys;

phys_PxComputeHeightFieldPenetration :: (direction_pod: *physx_PxVec3_Pod, depth_pod: *float, geom_pod: *physx_PxGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, heightFieldGeom_pod: *physx_PxHeightFieldGeometry_Pod, heightFieldPose_pod: *physx_PxTransform_Pod, maxIter: u32, usedIter: *u32) -> bool #foreign physx_sys;

PxXmlMiscParameter_new :: () -> physx_PxXmlMiscParameter_Pod #foreign physx_sys;

PxXmlMiscParameter_new_1 :: (inUpVector_pod: *physx_PxVec3_Pod, inScale_pod: physx_PxTolerancesScale_Pod) -> physx_PxXmlMiscParameter_Pod #foreign physx_sys;

PxSerialization_isSerializable :: (collection_pod: *physx_PxCollection_Pod, sr_pod: *physx_PxSerializationRegistry_Pod, externalReferences_pod: *physx_PxCollection_Pod) -> bool #foreign physx_sys;

PxSerialization_complete :: (collection_pod: *physx_PxCollection_Pod, sr_pod: *physx_PxSerializationRegistry_Pod, exceptFor_pod: *physx_PxCollection_Pod, followJoints: bool) -> void #foreign physx_sys;

PxSerialization_createSerialObjectIds :: (collection_pod: *physx_PxCollection_Pod, base: u64) -> void #foreign physx_sys;

PxSerialization_createCollectionFromXml :: (inputData_pod: *physx_PxInputData_Pod, cooking_pod: *physx_PxCooking_Pod, sr_pod: *physx_PxSerializationRegistry_Pod, externalRefs_pod: *physx_PxCollection_Pod, stringTable_pod: *physx_PxStringTable_Pod, outArgs_pod: *physx_PxXmlMiscParameter_Pod) -> *physx_PxCollection_Pod #foreign physx_sys;

PxSerialization_createCollectionFromBinary :: (memBlock: *void, sr_pod: *physx_PxSerializationRegistry_Pod, externalRefs_pod: *physx_PxCollection_Pod) -> *physx_PxCollection_Pod #foreign physx_sys;

PxSerialization_serializeCollectionToXml :: (outputStream_pod: *physx_PxOutputStream_Pod, collection_pod: *physx_PxCollection_Pod, sr_pod: *physx_PxSerializationRegistry_Pod, cooking_pod: *physx_PxCooking_Pod, externalRefs_pod: *physx_PxCollection_Pod, inArgs_pod: *physx_PxXmlMiscParameter_Pod) -> bool #foreign physx_sys;

PxSerialization_serializeCollectionToBinary :: (outputStream_pod: *physx_PxOutputStream_Pod, collection_pod: *physx_PxCollection_Pod, sr_pod: *physx_PxSerializationRegistry_Pod, externalRefs_pod: *physx_PxCollection_Pod, exportNames: bool) -> bool #foreign physx_sys;

PxSerialization_createSerializationRegistry :: (physics_pod: *physx_PxPhysics_Pod) -> *physx_PxSerializationRegistry_Pod #foreign physx_sys;

PxDefaultCpuDispatcher_release_mut :: (self__pod: *physx_PxDefaultCpuDispatcher_Pod) -> void #foreign physx_sys;

PxDefaultCpuDispatcher_setRunProfiled_mut :: (self__pod: *physx_PxDefaultCpuDispatcher_Pod, runProfiled: bool) -> void #foreign physx_sys;

PxDefaultCpuDispatcher_getRunProfiled :: (self__pod: *physx_PxDefaultCpuDispatcher_Pod) -> bool #foreign physx_sys;

phys_PxDefaultCpuDispatcherCreate :: (numThreads: u32, affinityMasks: *u32, mode_pod: s32, yieldProcessorCount: u32) -> *physx_PxDefaultCpuDispatcher_Pod #foreign physx_sys;

phys_PxBuildSmoothNormals :: (nbTris: u32, nbVerts: u32, verts_pod: *physx_PxVec3_Pod, dFaces: *u32, wFaces: *u16, normals_pod: *physx_PxVec3_Pod, flip: bool) -> bool #foreign physx_sys;

phys_PxCreateDynamic :: (sdk_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, geometry_pod: *physx_PxGeometry_Pod, material_pod: *physx_PxMaterial_Pod, density: float, shapeOffset_pod: *physx_PxTransform_Pod) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

phys_PxCreateDynamic_1 :: (sdk_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, shape_pod: *physx_PxShape_Pod, density: float) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

phys_PxCreateKinematic :: (sdk_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, geometry_pod: *physx_PxGeometry_Pod, material_pod: *physx_PxMaterial_Pod, density: float, shapeOffset_pod: *physx_PxTransform_Pod) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

phys_PxCreateKinematic_1 :: (sdk_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, shape_pod: *physx_PxShape_Pod, density: float) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

phys_PxCreateStatic :: (sdk_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, geometry_pod: *physx_PxGeometry_Pod, material_pod: *physx_PxMaterial_Pod, shapeOffset_pod: *physx_PxTransform_Pod) -> *physx_PxRigidStatic_Pod #foreign physx_sys;

phys_PxCreateStatic_1 :: (sdk_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, shape_pod: *physx_PxShape_Pod) -> *physx_PxRigidStatic_Pod #foreign physx_sys;

phys_PxCloneShape :: (physicsSDK_pod: *physx_PxPhysics_Pod, shape_pod: *physx_PxShape_Pod, isExclusive: bool) -> *physx_PxShape_Pod #foreign physx_sys;

phys_PxCloneStatic :: (physicsSDK_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, actor_pod: *physx_PxRigidActor_Pod) -> *physx_PxRigidStatic_Pod #foreign physx_sys;

phys_PxCloneDynamic :: (physicsSDK_pod: *physx_PxPhysics_Pod, transform_pod: *physx_PxTransform_Pod, body_pod: *physx_PxRigidDynamic_Pod) -> *physx_PxRigidDynamic_Pod #foreign physx_sys;

phys_PxCreatePlane :: (sdk_pod: *physx_PxPhysics_Pod, plane_pod: *physx_PxPlane_Pod, material_pod: *physx_PxMaterial_Pod) -> *physx_PxRigidStatic_Pod #foreign physx_sys;

phys_PxScaleRigidActor :: (actor_pod: *physx_PxRigidActor_Pod, scale: float, scaleMassProps: bool) -> void #foreign physx_sys;

PxStringTableExt_createStringTable :: (inAllocator_pod: *physx_PxAllocatorCallback_Pod) -> *physx_PxStringTable_Pod #foreign physx_sys;

PxBroadPhaseExt_createRegionsFromWorldBounds :: (regions_pod: *physx_PxBounds3_Pod, globalBounds_pod: *physx_PxBounds3_Pod, nbSubdiv: u32, upAxis: u32) -> u32 #foreign physx_sys;

PxSceneQueryExt_raycastAny :: (scene_pod: *physx_PxScene_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, hit_pod: *physx_PxQueryHit_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod) -> bool #foreign physx_sys;

PxSceneQueryExt_raycastSingle :: (scene_pod: *physx_PxScene_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, outputFlags_pod: u16, hit_pod: *physx_PxRaycastHit_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod) -> bool #foreign physx_sys;

PxSceneQueryExt_raycastMultiple :: (scene_pod: *physx_PxScene_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, outputFlags_pod: u16, hitBuffer_pod: *physx_PxRaycastHit_Pod, hitBufferSize: u32, blockingHit_pod: *bool, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod) -> s32 #foreign physx_sys;

PxSceneQueryExt_sweepAny :: (scene_pod: *physx_PxScene_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, queryFlags_pod: u16, hit_pod: *physx_PxQueryHit_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float) -> bool #foreign physx_sys;

PxSceneQueryExt_sweepSingle :: (scene_pod: *physx_PxScene_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, outputFlags_pod: u16, hit_pod: *physx_PxSweepHit_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float) -> bool #foreign physx_sys;

PxSceneQueryExt_sweepMultiple :: (scene_pod: *physx_PxScene_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, outputFlags_pod: u16, hitBuffer_pod: *physx_PxSweepHit_Pod, hitBufferSize: u32, blockingHit_pod: *bool, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float) -> s32 #foreign physx_sys;

PxSceneQueryExt_overlapMultiple :: (scene_pod: *physx_PxScene_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, hitBuffer_pod: *physx_PxOverlapHit_Pod, hitBufferSize: u32, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod) -> s32 #foreign physx_sys;

PxSceneQueryExt_overlapAny :: (scene_pod: *physx_PxScene_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, hit_pod: *physx_PxOverlapHit_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod) -> bool #foreign physx_sys;

PxBatchQueryExt_release_mut :: (self__pod: *physx_PxBatchQueryExt_Pod) -> void #foreign physx_sys;

PxBatchQueryExt_raycast_mut :: (self__pod: *physx_PxBatchQueryExt_Pod, origin_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, maxNbTouches: u16, hitFlags_pod: u16, filterData_pod: *physx_PxQueryFilterData_Pod, cache_pod: *physx_PxQueryCache_Pod) -> *physx_PxRaycastBuffer_Pod #foreign physx_sys;

PxBatchQueryExt_sweep_mut :: (self__pod: *physx_PxBatchQueryExt_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, unitDir_pod: *physx_PxVec3_Pod, distance: float, maxNbTouches: u16, hitFlags_pod: u16, filterData_pod: *physx_PxQueryFilterData_Pod, cache_pod: *physx_PxQueryCache_Pod, inflation: float) -> *physx_PxSweepBuffer_Pod #foreign physx_sys;

PxBatchQueryExt_overlap_mut :: (self__pod: *physx_PxBatchQueryExt_Pod, geometry_pod: *physx_PxGeometry_Pod, pose_pod: *physx_PxTransform_Pod, maxNbTouches: u16, filterData_pod: *physx_PxQueryFilterData_Pod, cache_pod: *physx_PxQueryCache_Pod) -> *physx_PxOverlapBuffer_Pod #foreign physx_sys;

PxBatchQueryExt_execute_mut :: (self__pod: *physx_PxBatchQueryExt_Pod) -> void #foreign physx_sys;

phys_PxCreateBatchQueryExt :: (scene_pod: *physx_PxScene_Pod, queryFilterCallback_pod: *physx_PxQueryFilterCallback_Pod, maxNbRaycasts: u32, maxNbRaycastTouches: u32, maxNbSweeps: u32, maxNbSweepTouches: u32, maxNbOverlaps: u32, maxNbOverlapTouches: u32) -> *physx_PxBatchQueryExt_Pod #foreign physx_sys;

phys_PxCreateBatchQueryExt_1 :: (scene_pod: *physx_PxScene_Pod, queryFilterCallback_pod: *physx_PxQueryFilterCallback_Pod, raycastBuffers_pod: *physx_PxRaycastBuffer_Pod, maxNbRaycasts: u32, raycastTouches_pod: *physx_PxRaycastHit_Pod, maxNbRaycastTouches: u32, sweepBuffers_pod: *physx_PxSweepBuffer_Pod, maxNbSweeps: u32, sweepTouches_pod: *physx_PxSweepHit_Pod, maxNbSweepTouches: u32, overlapBuffers_pod: *physx_PxOverlapBuffer_Pod, maxNbOverlaps: u32, overlapTouches_pod: *physx_PxOverlapHit_Pod, maxNbOverlapTouches: u32) -> *physx_PxBatchQueryExt_Pod #foreign physx_sys;

phys_PxCreateExternalSceneQuerySystem :: (desc_pod: *physx_PxSceneQueryDesc_Pod, contextID: u64) -> *physx_PxSceneQuerySystem_Pod #foreign physx_sys;

PxCustomSceneQuerySystem_addPruner_mut :: (self__pod: *physx_PxCustomSceneQuerySystem_Pod, primaryType_pod: s32, secondaryType_pod: s32, preallocated: u32) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystem_startCustomBuildstep_mut :: (self__pod: *physx_PxCustomSceneQuerySystem_Pod) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystem_customBuildstep_mut :: (self__pod: *physx_PxCustomSceneQuerySystem_Pod, index: u32) -> void #foreign physx_sys;

PxCustomSceneQuerySystem_finishCustomBuildstep_mut :: (self__pod: *physx_PxCustomSceneQuerySystem_Pod) -> void #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_delete :: (self__pod: *physx_PxCustomSceneQuerySystemAdapter_Pod) -> void #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_getPrunerIndex :: (self__pod: *physx_PxCustomSceneQuerySystemAdapter_Pod, actor_pod: *physx_PxRigidActor_Pod, shape_pod: *physx_PxShape_Pod) -> u32 #foreign physx_sys;

PxCustomSceneQuerySystemAdapter_processPruner :: (self__pod: *physx_PxCustomSceneQuerySystemAdapter_Pod, prunerIndex: u32, context_pod: *physx_PxQueryThreadContext_Pod, filterData_pod: *physx_PxQueryFilterData_Pod, filterCall_pod: *physx_PxQueryFilterCallback_Pod) -> bool #foreign physx_sys;

phys_PxCreateCustomSceneQuerySystem :: (sceneQueryUpdateMode_pod: s32, contextID: u64, adapter_pod: *physx_PxCustomSceneQuerySystemAdapter_Pod, usesTreeOfPruners: bool) -> *physx_PxCustomSceneQuerySystem_Pod #foreign physx_sys;

phys_PxFindFaceIndex :: (convexGeom_pod: *physx_PxConvexMeshGeometry_Pod, geomPose_pod: *physx_PxTransform_Pod, impactPos_pod: *physx_PxVec3_Pod, unitDir_pod: *physx_PxVec3_Pod) -> u32 #foreign physx_sys;

PxPoissonSampler_setSamplingRadius_mut :: (self__pod: *physx_PxPoissonSampler_Pod, samplingRadius: float) -> bool #foreign physx_sys;

PxPoissonSampler_addSamplesInSphere_mut :: (self__pod: *physx_PxPoissonSampler_Pod, sphereCenter_pod: *physx_PxVec3_Pod, sphereRadius: float, createVolumeSamples: bool) -> void #foreign physx_sys;

PxPoissonSampler_addSamplesInBox_mut :: (self__pod: *physx_PxPoissonSampler_Pod, axisAlignedBox_pod: *physx_PxBounds3_Pod, boxOrientation_pod: *physx_PxQuat_Pod, createVolumeSamples: bool) -> void #foreign physx_sys;

PxPoissonSampler_delete :: (self__pod: *physx_PxPoissonSampler_Pod) -> void #foreign physx_sys;

phys_PxCreateShapeSampler :: (geometry_pod: *physx_PxGeometry_Pod, transform_pod: *physx_PxTransform_Pod, worldBounds_pod: *physx_PxBounds3_Pod, initialSamplingRadius: float, numSampleAttemptsAroundPoint: s32) -> *physx_PxPoissonSampler_Pod #foreign physx_sys;

PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut :: (self__pod: *physx_PxTriangleMeshPoissonSampler_Pod, p_pod: *physx_PxVec3_Pod) -> bool #foreign physx_sys;

PxTriangleMeshPoissonSampler_delete :: (self__pod: *physx_PxTriangleMeshPoissonSampler_Pod) -> void #foreign physx_sys;

phys_PxCreateTriangleMeshSampler :: (triangles: *u32, numTriangles: u32, vertices_pod: *physx_PxVec3_Pod, numVertices: u32, initialSamplingRadius: float, numSampleAttemptsAroundPoint: s32) -> *physx_PxTriangleMeshPoissonSampler_Pod #foreign physx_sys;

PxTetrahedronMeshExt_findTetrahedronContainingPoint :: (mesh_pod: *physx_PxTetrahedronMesh_Pod, point_pod: *physx_PxVec3_Pod, bary_pod: *physx_PxVec4_Pod, tolerance: float) -> s32 #foreign physx_sys;

PxTetrahedronMeshExt_findTetrahedronClosestToPoint :: (mesh_pod: *physx_PxTetrahedronMesh_Pod, point_pod: *physx_PxVec3_Pod, bary_pod: *physx_PxVec4_Pod) -> s32 #foreign physx_sys;

phys_PxInitExtensions :: (physics_pod: *physx_PxPhysics_Pod, pvd_pod: *physx_PxPvd_Pod) -> bool #foreign physx_sys;

phys_PxCloseExtensions :: () -> void #foreign physx_sys;

PxRepXObject_new :: (inTypeName: *u8, inSerializable: *void, inId: u64) -> physx_PxRepXObject_Pod #foreign physx_sys;

PxRepXObject_isValid :: (self__pod: *physx_PxRepXObject_Pod) -> bool #foreign physx_sys;

PxRepXInstantiationArgs_new :: (inPhysics_pod: *physx_PxPhysics_Pod, inCooking_pod: *physx_PxCooking_Pod, inStringTable_pod: *physx_PxStringTable_Pod) -> physx_PxRepXInstantiationArgs_Pod #foreign physx_sys;

PxRepXSerializer_getTypeName_mut :: (self__pod: *physx_PxRepXSerializer_Pod) -> *u8 #foreign physx_sys;

PxRepXSerializer_objectToFile_mut :: (self__pod: *physx_PxRepXSerializer_Pod, inLiveObject_pod: *physx_PxRepXObject_Pod, inCollection_pod: *physx_PxCollection_Pod, inWriter_pod: *physx_XmlWriter_Pod, inTempBuffer_pod: *physx_MemoryBuffer_Pod, inArgs_pod: *physx_PxRepXInstantiationArgs_Pod) -> void #foreign physx_sys;

PxRepXSerializer_fileToObject_mut :: (self__pod: *physx_PxRepXSerializer_Pod, inReader_pod: *physx_XmlReader_Pod, inAllocator_pod: *physx_XmlMemoryAllocator_Pod, inArgs_pod: *physx_PxRepXInstantiationArgs_Pod, inCollection_pod: *physx_PxCollection_Pod) -> physx_PxRepXObject_Pod #foreign physx_sys;

PxPvd_connect_mut :: (self__pod: *physx_PxPvd_Pod, transport_pod: *physx_PxPvdTransport_Pod, flags_pod: u8) -> bool #foreign physx_sys;

PxPvd_disconnect_mut :: (self__pod: *physx_PxPvd_Pod) -> void #foreign physx_sys;

PxPvd_isConnected_mut :: (self__pod: *physx_PxPvd_Pod, useCachedStatus: bool) -> bool #foreign physx_sys;

PxPvd_getTransport_mut :: (self__pod: *physx_PxPvd_Pod) -> *physx_PxPvdTransport_Pod #foreign physx_sys;

PxPvd_getInstrumentationFlags_mut :: (self__pod: *physx_PxPvd_Pod) -> u8 #foreign physx_sys;

PxPvd_release_mut :: (self__pod: *physx_PxPvd_Pod) -> void #foreign physx_sys;

phys_PxCreatePvd :: (foundation_pod: *physx_PxFoundation_Pod) -> *physx_PxPvd_Pod #foreign physx_sys;

PxPvdTransport_connect_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> bool #foreign physx_sys;

PxPvdTransport_disconnect_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> void #foreign physx_sys;

PxPvdTransport_isConnected_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> bool #foreign physx_sys;

PxPvdTransport_write_mut :: (self__pod: *physx_PxPvdTransport_Pod, inBytes: *u8, inLength: u32) -> bool #foreign physx_sys;

PxPvdTransport_lock_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> *physx_PxPvdTransport_Pod #foreign physx_sys;

PxPvdTransport_unlock_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> void #foreign physx_sys;

PxPvdTransport_flush_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> void #foreign physx_sys;

PxPvdTransport_getWrittenDataSize_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> u64 #foreign physx_sys;

PxPvdTransport_release_mut :: (self__pod: *physx_PxPvdTransport_Pod) -> void #foreign physx_sys;

phys_PxDefaultPvdSocketTransportCreate :: (host: *u8, port: s32, timeoutInMilliseconds: u32) -> *physx_PxPvdTransport_Pod #foreign physx_sys;

phys_PxDefaultPvdFileTransportCreate :: (name: *u8) -> *physx_PxPvdTransport_Pod #foreign physx_sys;

#scope_file

physx_sys :: #library,no_dll "windows/physx_sys";
